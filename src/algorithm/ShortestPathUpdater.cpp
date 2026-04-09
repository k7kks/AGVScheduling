#include "algorithm/ShortestPathUpdater.h"
#include <thread>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>  // 新增：用于std::min和std::max
#include <unordered_set>

// 构造函数实现 - 修复std::clamp错误
ShortestPathUpdater::ShortestPathUpdater(const MapInfo& mapInfo, double directionPenalty, int maxWorkerNum)
    : mapInfo(mapInfo),
      globalMaxSpeed(mapInfo.getGlobalMaxSpeed()),
      directionPenalty(directionPenalty),
      matrixInitialized(false),
      maxWorkerNum(std::max(1, std::min(maxWorkerNum, 16))),
      activeWorkers(0),
      shouldExit(false) {
    // 启动固定大小的工作线程池（常驻）
    for (int i = 0; i < this->maxWorkerNum; ++i) {
        workers.emplace_back([this]() { this->workerFunction(); });
        ++activeWorkers;
    }
}

ShortestPathUpdater::~ShortestPathUpdater() {
    // 请求退出并唤醒所有工作线程
    shouldExit = true;
    queueCv.notify_all();
    for (auto &th : workers) {
        if (th.joinable()) th.join();
    }
}
// 对外接口实现
void ShortestPathUpdater::initializeMatrix(int amrTypeNum) {
    if (amrTypeNum <= 0) {
        throw std::invalid_argument("AMR类型数必须大于0");
    }
    int nodeNum = mapInfo.getNodes().size();
    // 稀疏式初始化：仅初始化第一维，第二维按需在 query 时分配
    {
        std::lock_guard<std::mutex> lock(matrixMutex);
        distanceMatrix.clear();
        distanceMatrix.resize(nodeNum);
        amrTypeNum_ = amrTypeNum;
        matrixInitialized = true;
    }
}

// 按需为某个源节点分配一行矩阵
static inline void ensureRowAllocated(std::vector<std::vector<std::vector<double>>>& m,
                                      std::mutex& mu,
                                      int rowIdx,
                                      int nodeNum,
                                      int amrTypeNum) {
    std::lock_guard<std::mutex> lk(mu);
    if (rowIdx >= 0 && rowIdx < (int)m.size()) {
        if (m[rowIdx].empty()) {
            m[rowIdx].assign(nodeNum, std::vector<double>(amrTypeNum, (double)DistanceState::UNCALCULATED));
        }
    }
}

double ShortestPathUpdater::query(int startIdx, int endIdx, int amrType) const{
    if (!matrixInitialized) {
        throw std::runtime_error("请先调用initializeMatrix初始化距离矩阵");
    }
    int nodeNum = mapInfo.getNodes().size();
    int amrTypeNum = amrTypeNum_;
    if (startIdx < 0 || startIdx >= nodeNum || endIdx < 0 || endIdx >= nodeNum) {
        throw std::invalid_argument("起点/终点索引无效");
    }
    if (amrType < 0 || amrType >= amrTypeNum) {
        throw std::invalid_argument("AMR类型无效");
    }

    // 确保本行存在
    ensureRowAllocated(const_cast<std::vector<std::vector<std::vector<double>>>&>(distanceMatrix),
                      const_cast<std::mutex&>(matrixMutex), startIdx, nodeNum, amrTypeNum);

    // 1. 优先读取缓存
    double cachedDist;
    {  // 使用代码块限制lock_guard的作用域，自动解锁
        std::lock_guard<std::mutex> matrixLock(matrixMutex);
        cachedDist = distanceMatrix[startIdx][endIdx][amrType];
    }  // 此处自动解锁，替代手动调用unlock()

    if (cachedDist == static_cast<double>(DistanceState::UNREACHABLE)) {
        return std::numeric_limits<double>::infinity();
    }
    if (cachedDist > 0) {
        return cachedDist;
    }

    // 2. 检查路径有效性（可通过环境变量关闭预过滤）
    bool prefilter = true;
    if (const char* ev = std::getenv("SPU_DISABLE_PREFILTER")) {
        std::string v(ev);
        if (!v.empty() && v != "0" && v != "false" && v != "False") prefilter = false;
    }
    if (prefilter) {
        if (!isPathValid(startIdx, endIdx)) {
            std::lock_guard<std::mutex> matrixLock2(matrixMutex);
            distanceMatrix[startIdx][endIdx][amrType] = static_cast<double>(DistanceState::UNREACHABLE);
            return std::numeric_limits<double>::infinity();
        }
    }

    // 3. 加入队列并启动线程
    double heuristicDist = minkowskiHeuristic(startIdx, endIdx);
    {
        std::lock_guard<std::mutex> queueLock(queueMutex);
        auto it = std::find_if(queryQueue.begin(), queryQueue.end(),
            [&](const std::tuple<int, int, int>& task) {
                return std::get<0>(task) == startIdx && 
                       std::get<1>(task) == endIdx && 
                       std::get<2>(task) == amrType;
            });
        if (it == queryQueue.end()) {
            queryQueue.emplace_back(startIdx, endIdx, amrType);
        }
    }

    // 通知线程池有新任务
    queueCv.notify_one();

    return heuristicDist;
}

double ShortestPathUpdater::queryByNodeId(int startNodeId, int endNodeId, int amrType) const {
    // 映射 nodeId -> index（内部使用 MapInfo 提供的映射）
    const auto& id2idx = mapInfo.getId2Index();
    auto itS = id2idx.find(startNodeId);
    auto itE = id2idx.find(endNodeId);
    if (itS == id2idx.end() || itE == id2idx.end()) {
        return static_cast<double>(DistanceState::UNREACHABLE);
    }
    int sIdx = itS->second;
    int eIdx = itE->second;
    // 调用索引版query以填充/触发计算
    (void)query(sIdx, eIdx, amrType);
    // 读取当前矩阵状态值返回（可能为 UNCALCULATED/UNREACHABLE/有效时间）
    double v;
    {
        std::lock_guard<std::mutex> lock(matrixMutex);
        v = distanceMatrix[sIdx][eIdx][amrType];
    }
    return v;
}

void ShortestPathUpdater::batchQueryByNodeId(int startNodeId, const std::vector<int>& targetNodeIds, int amrType) const {
    if (!matrixInitialized) throw std::runtime_error("请先调用initializeMatrix初始化距离矩阵");
    const auto& id2idx = mapInfo.getId2Index();
    auto itS = id2idx.find(startNodeId);
    if (itS == id2idx.end()) return;
    int sIdx = itS->second;
    // 映射并去重目标索引
    std::vector<int> targets;
    targets.reserve(targetNodeIds.size());
    for (int nid : targetNodeIds) {
        auto it = id2idx.find(nid);
        if (it != id2idx.end()) targets.push_back(it->second);
    }
    if (targets.empty()) return;
    // 若该行未分配，先分配
    ensureRowAllocated(const_cast<std::vector<std::vector<std::vector<double>>>&>(distanceMatrix),
                      const_cast<std::mutex&>(matrixMutex), sIdx, (int)mapInfo.getNodes().size(), amrTypeNum_);
    // 多目标 Dijkstra：直到所有目标 settled
    const double INF = std::numeric_limits<double>::infinity();
    int nodeNum = mapInfo.getNodes().size();
    std::vector<double> dist(nodeNum, INF);
    std::vector<int> dir(nodeNum, 0);
    double fallbackSpeed = mapInfo.getGlobalMaxSpeed();
    if (!(fallbackSpeed > 0.0)) fallbackSpeed = 1000.0;
    auto resolveNodeSpeed = [&](int nodeIdx)->double{
        const Node& node = mapInfo.getNode(nodeIdx);
        double speed = node.maxSpeed;
        if (!(speed > 0.0)) speed = fallbackSpeed;
        return speed;
    };
    auto typeKey = std::to_string(amrType);
    auto passAllowed = [&](const Node& n)->bool{
        if (n.allowPass == 0) return false;
        if (!n.allowPassAmrTypeList.empty()) {
            return std::find(n.allowPassAmrTypeList.begin(), n.allowPassAmrTypeList.end(), typeKey) != n.allowPassAmrTypeList.end();
        }
        return true;
    };
    auto rotAllowed = [&](const Node& n)->bool{
        if (n.allowRot == 0) return false;
        if (!n.allowRotAmrTypeList.empty()) {
            return std::find(n.allowRotAmrTypeList.begin(), n.allowRotAmrTypeList.end(), typeKey) != n.allowRotAmrTypeList.end();
        }
        return true;
    };
    // 过滤非法起点
    const Node& sn = mapInfo.getNode(sIdx);
    if (!passAllowed(sn)) {
        std::lock_guard<std::mutex> lock(matrixMutex);
        for (int tIdx : targets) distanceMatrix[sIdx][tIdx][amrType] = (double)DistanceState::UNREACHABLE;
        return;
    }
    std::unordered_set<int> targetSet(targets.begin(), targets.end());
    int remain = (int)targetSet.size();
    using P = std::pair<double,int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    dist[sIdx] = 0.0;
    pq.emplace(0.0, sIdx);
    while (!pq.empty() && remain > 0) {
        auto [d,u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        const Node& currNode = mapInfo.getNode(u);
        if (targetSet.count(u)) {
            // 写入矩阵
            std::lock_guard<std::mutex> lock(matrixMutex);
            distanceMatrix[sIdx][u][amrType] = d;
            targetSet.erase(u);
            --remain;
            continue;
        }
        const auto& aft = mapInfo.getAftNode();
        auto it = aft.find(u);
        if (it == aft.end()) continue;
        int curDir = dir[u];
        for (int v : it->second) {
            const Edge* e = mapInfo.getEdge(u, v);
            if (!e) continue;
            const Node& nextNode = mapInfo.getNode(v);
            if (!passAllowed(nextNode)) continue;
            double speedFrom = resolveNodeSpeed(u);
            double travelTime = e->weight / speedFrom;
            int eDir = e->direction;
            double pen = 0.0;
            if (curDir != 0 && curDir != eDir) {
                if (!rotAllowed(currNode)) continue;
                pen = directionPenalty / speedFrom;
            }
            double nd = d + travelTime + pen;
            if (nd < dist[v]) { dist[v] = nd; dir[v] = eDir; pq.emplace(nd, v); }
        }
    }
    // 未抵达的目标标为 UNREACHABLE
    if (!targetSet.empty()) {
        std::lock_guard<std::mutex> lock(matrixMutex);
        for (int t : targetSet) distanceMatrix[sIdx][t][amrType] = (double)DistanceState::UNREACHABLE;
    }
}

void ShortestPathUpdater::setDirectionPenalty(double penalty) {
    if (penalty < 0) {
        throw std::invalid_argument("转向惩罚不能为负数");
    }
    directionPenalty = penalty;
}

const std::vector<std::vector<std::vector<double>>>& ShortestPathUpdater::getDistanceMatrix() const {
    return distanceMatrix;
}

// 辅助方法实现
double ShortestPathUpdater::minkowskiHeuristic(int startIdx, int endIdx, double p) const {
    const Node& s = mapInfo.getNode(startIdx);
    const Node& e = mapInfo.getNode(endIdx);
    double dx = std::abs(s.x - e.x);
    double dy = std::abs(s.y - e.y);
    double dist = 0.0;
    if (p <= 1.0) {
        dist = dx + dy;
    } else {
        dist = std::pow(std::pow(dx, p) + std::pow(dy, p), 1.0 / p);
    }
    double speed = mapInfo.getGlobalMaxSpeed();
    if (!(speed > 0.0)) speed = 1000.0;
    return dist / speed;
}

bool ShortestPathUpdater::isPathValid(int startIdx, int endIdx) const {
    if (!mapInfo.getReachAnalyzer()->isConnected(startIdx, endIdx)) {
        return false;
    }
    const auto& aftNode = mapInfo.getAftNode();
    const auto& preNode = mapInfo.getPreNode();
    return aftNode.count(startIdx) && preNode.count(endIdx);
}

void ShortestPathUpdater::shortestPathFinder(int startIdx, int endIdx, int amrType) {
    // 实现保持不变...
    int nodeNum = mapInfo.getNodes().size();
    const double INF = std::numeric_limits<double>::infinity();

    std::unordered_map<int, bool> openSet;
    std::unordered_map<int, bool> closedSet;
    std::unordered_map<int, double> gScore;
    std::unordered_map<int, double> fScore;
    std::unordered_map<int, int> cameFrom;
    std::unordered_map<int, int> nodeDir;

    for (int i = 0; i < nodeNum; ++i) {
        gScore[i] = INF;
        fScore[i] = INF;
        openSet[i] = false;
        closedSet[i] = false;
    }

    gScore[startIdx] = 0.0;
    fScore[startIdx] = minkowskiHeuristic(startIdx, endIdx);
    openSet[startIdx] = true;
    nodeDir[startIdx] = 0;

    const Node& startNode = mapInfo.getNode(startIdx);
    auto resolveNodeSpeed = [&](int nodeIdx)->double {
        const Node& node = mapInfo.getNode(nodeIdx);
        double speed = node.maxSpeed;
        if (!(speed > 0.0)) speed = mapInfo.getGlobalMaxSpeed();
        if (!(speed > 0.0)) speed = 1000.0;
        return speed;
    };
    auto typeKey = std::to_string(amrType);
    auto passAllowed = [&](const Node& n)->bool{
        if (n.allowPass == 0) return false;
        if (!n.allowPassAmrTypeList.empty()) {
            return std::find(n.allowPassAmrTypeList.begin(), n.allowPassAmrTypeList.end(), typeKey) != n.allowPassAmrTypeList.end();
        }
        return true;
    };
    auto rotAllowed = [&](const Node& n)->bool{
        if (n.allowRot == 0) return false;
        if (!n.allowRotAmrTypeList.empty()) {
            return std::find(n.allowRotAmrTypeList.begin(), n.allowRotAmrTypeList.end(), typeKey) != n.allowRotAmrTypeList.end();
        }
        return true;
    };
    if (!passAllowed(startNode)) {
        std::lock_guard<std::mutex> matrixLock(matrixMutex);
        distanceMatrix[startIdx][endIdx][amrType] = static_cast<double>(DistanceState::UNREACHABLE);
        return;
    }

    std::priority_queue<QueueElement, std::vector<QueueElement>, QueueComparator> heap;
    heap.push({fScore[startIdx], amrType, startIdx});

    int currentNode = -1;
    while (!heap.empty()) {
        QueueElement elem = heap.top();
        heap.pop();
        currentNode = elem.nodeIndex;
        int validAmr = elem.amrType;

        if (currentNode == endIdx) {
            break;
        }

        if (closedSet[currentNode]) {
            continue;
        }
        openSet[currentNode] = false;
        closedSet[currentNode] = true;
        int currentDir = nodeDir[currentNode];

        const auto& aftNode = mapInfo.getAftNode();
        auto it = aftNode.find(currentNode);
        if (it == aftNode.end()) {
            continue;
        }
        const std::vector<int>& neighbors = it->second;
        const Node& currentNodeInfo = mapInfo.getNode(currentNode);

        for (int neighbor : neighbors) {
            if (closedSet[neighbor]) {
                continue;
            }

            const Edge* edge = mapInfo.getEdge(currentNode, neighbor);
            if (!edge) {
                continue;
            }
            double edgeDistance = edge->weight;
            int edgeDir = edge->direction;

            double speedFrom = resolveNodeSpeed(currentNode);
            double travelTime = edgeDistance / speedFrom;

            double dirPenalty = 0.0;
            if (currentDir != 0 && currentDir != edgeDir) {
                if (!rotAllowed(currentNodeInfo)) {
                    continue;
                }
                dirPenalty = directionPenalty / speedFrom;
            }

            double tentativeG = gScore[currentNode] + travelTime + dirPenalty;
            if (tentativeG < gScore[neighbor]) {
                cameFrom[neighbor] = currentNode;
                nodeDir[neighbor] = edgeDir;
                gScore[neighbor] = tentativeG;
                fScore[neighbor] = tentativeG + minkowskiHeuristic(neighbor, endIdx);

                const Node& neighborNode = mapInfo.getNode(neighbor);
                if (!passAllowed(neighborNode)) {
                    continue;
                }

                if (!openSet[neighbor]) {
                    openSet[neighbor] = true;
                    heap.push({fScore[neighbor], validAmr, neighbor});
                }
            }
        }
    }

    std::lock_guard<std::mutex> matrixLock(matrixMutex);
    if (currentNode == endIdx && gScore[endIdx] < INF) {
        distanceMatrix[startIdx][endIdx][amrType] = gScore[endIdx];
    } else {
        distanceMatrix[startIdx][endIdx][amrType] = static_cast<double>(DistanceState::UNREACHABLE);
    }
}

// ShortestPathUpdater.cpp workerFunction 修复
void ShortestPathUpdater::workerFunction() {
    while (true) {
        std::tuple<int,int,int> task;
        {
            std::unique_lock<std::mutex> lk(queueMutex);
            queueCv.wait(lk, [this](){ return shouldExit || !queryQueue.empty(); });
            if (shouldExit && queryQueue.empty()) break;
            task = queryQueue.front();
            queryQueue.erase(queryQueue.begin());
        }
        int start = std::get<0>(task);
        int end = std::get<1>(task);
        int amrType = std::get<2>(task);
        try {
            shortestPathFinder(start, end, amrType);
        } catch (...) {
            std::lock_guard<std::mutex> matrixLock(matrixMutex);
            distanceMatrix[start][end][amrType] = static_cast<double>(DistanceState::UNREACHABLE);
        }
    }
}

    
