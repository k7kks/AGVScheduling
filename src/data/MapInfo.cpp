#include "data/MapInfo.h"
#include <stdexcept>
#include <iostream>  // 引入cout、endl所需的头文件
#include <algorithm>
#include <functional>  // 用于std::hash

// -------------------------- UnionFind类实现 --------------------------
UnionFind::UnionFind(int n) {
    parent.resize(n); // 节点索引从1开始，预留0号位置
    rank.resize(n, 0);
    for (int i = 0; i < n; ++i) {
        parent[i] = i; // 初始时每个节点的父节点是自身
    }
}

int UnionFind::find(int x) {
    if (x < 0 || x >= static_cast<int>(parent.size())) return -1;
    int root = x;
    int steps = 0;
    while (parent[root] != root) {
        int p = parent[root];
        if (p < 0 || p >= static_cast<int>(parent.size())) return -1;
        root = p;
        if (++steps > static_cast<int>(parent.size())) return -1;
    }
    // 路径压缩：将路径上的节点直接指向根节点（仅构建/单线程阶段使用）
    while (parent[x] != x) {
        int p = parent[x];
        if (p < 0 || p >= static_cast<int>(parent.size())) break;
        parent[x] = root;
        x = p;
    }
    return root;
}

int UnionFind::find(int x) const {
    // 只读 find：不做路径压缩，避免并发 isConnected 查询时写 parent 触发数据竞争
    if (x < 0 || x >= static_cast<int>(parent.size())) return -1;
    int root = x;
    int steps = 0;
    while (parent[root] != root) {
        int p = parent[root];
        if (p < 0 || p >= static_cast<int>(parent.size())) return -1;
        root = p;
        if (++steps > static_cast<int>(parent.size())) return -1;
    }
    return root;
}

void UnionFind::unite(int x, int y) {
    int rootX = find(x);
    int rootY = find(y);
    if (rootX < 0 || rootY < 0) return;
    if (rootX == rootY) return; // 已在同一集合，无需合并

    // 按秩合并：将秩小的集合合并到秩大的集合
    if (rank[rootX] < rank[rootY]) {
        parent[rootX] = rootY;
    } else {
        parent[rootY] = rootX;
        if (rank[rootX] == rank[rootY]) {
            rank[rootX]++;
        }
    }
}

bool UnionFind::isConnected(int x, int y) const{
    int rx = find(x);
    if (rx < 0) return false;
    int ry = find(y);
    if (ry < 0) return false;
    return rx == ry;
}

// -------------------------- MapInfo类实现 --------------------------
MapInfo::MapInfo(int amrTypeNum) : reachAnalyzer(nullptr), globalMaxSpeed(0.0), amrTypeNum(amrTypeNum) {}

MapInfo::~MapInfo() {
    if (reachAnalyzer != nullptr) {
        delete reachAnalyzer; // 释放并查集内存
    }
}

// 辅助函数：计算向量方向（1：右，2：上，3：左，4：下，对应MATLAB的vector_direction）
int MapInfo::calculateDirection(double dx, double dy) {
    const double eps = 1e-6; // 浮点误差容忍值
    if (fabs(dx) > fabs(dy)) {
        return (dx > eps) ? 1 : 3; // 水平方向：右（dx正）或左（dx负）
    } else {
        return (dy > eps) ? 2 : 4; // 垂直方向：上（dy正）或下（dy负）
    }
}

// 加载节点数据（对应MATLAB的read_node_info）
void MapInfo::loadNodes(const std::vector<Node>& rawNodes, const std::vector<Area>& mapAreas, double globalMaxSpeed) {
    this->globalMaxSpeed = globalMaxSpeed;
    this->areas = mapAreas;
    this->nodes.clear();
    this->id2index.clear();

    int nodeCount = rawNodes.size();
    if (nodeCount == 0) {
        throw std::runtime_error("原始节点列表为空，无法加载节点数据");
    }

    // 1. 初始化id2index和节点基础信息
    int validIndex = 0; // 节点索引从0开始
    std::vector<Node> tempNodes;
    for (const auto& rawNode : rawNodes) {
        Node newNode = rawNode;
        // 过去跳过type=-1节点，现在保留以供SPU/路径规划使用
        // 记录ID到索引的映射
        id2index[newNode.id] = validIndex;
        newNode.index = validIndex;
        // allowPass/allowRot 已改为布尔 + 类型字符串列表，取消位掩码逻辑

        // 3. 处理区域限速（取全局最大速度与区域速度的最小值，对应MATLAB的area循环）
        newNode.maxSpeed = globalMaxSpeed;
        for (const auto& area : areas) {
            if (newNode.x >= area.minX && newNode.x <= area.maxX &&
                newNode.y >= area.minY && newNode.y <= area.maxY) {
                newNode.maxSpeed = std::min(newNode.maxSpeed, area.maxSpeed);
            }
        }

        tempNodes.push_back(newNode);
        validIndex++;
    }

    this->nodes = tempNodes;
    // 初始化可达性分析器（并查集），大小为有效节点数
    if (reachAnalyzer != nullptr) {
        delete reachAnalyzer;
    }
    reachAnalyzer = new UnionFind(validIndex); // validIndex-1为有效节点总数
}

// 加载边数据,重写loadEdges方法：在加载边时同步构建哈希表
void MapInfo::loadEdges(const std::vector<Edge>& rawEdges) {
    this->edges.clear();
    this->edgeIndex.clear();  // 清空旧索引
    this->preNode.clear();
    this->aftNode.clear();

    if (nodes.empty()) {
        throw std::runtime_error("请先加载节点数据（调用loadNodes），再加载边数据");
    }

    for (const auto& rawEdge : rawEdges) {
        if (rawEdge.mode == 0) continue;

        auto startIt = id2index.find(rawEdge.startNodeId);
        auto endIt = id2index.find(rawEdge.endNodeId);
        if (startIt == id2index.end() || endIt == id2index.end()) continue;

        int startIndex = startIt->second;
        int endIndex = endIt->second;
        const Node& startNode = nodes[startIndex];
        const Node& endNode = nodes[endIndex];

        // type=-1 表示不可达节点，虽然保留在节点表中供其他模块使用，
        // 但在最短路径图里需要直接跳过相关边
        if (startNode.type == -1 || endNode.type == -1) {
            continue;
        }

        // 计算方向（已有逻辑）
        double dx = endNode.x - startNode.x;
        double dy = endNode.y - startNode.y;
        int dir = calculateDirection(dx, dy);

        // 构造当前边并添加到列表
        Edge currentEdge = rawEdge;
        // 边权保持为原始距离（mm）；时间换算由 ShortestPathUpdater 完成
        currentEdge.direction = dir;
        edges.push_back(currentEdge);  // 边对象存入vector
        const Edge* currentEdgePtr = &edges.back();  // 获取刚插入的边的指针

        // 生成键并添加到哈希表
        size_t key = getEdgeKey(startIndex, endIndex);
        edgeIndex[key] = edges.size() - 1;

        // 更新前驱/后继表
        aftNode[startIndex].push_back(endIndex);
        preNode[endIndex].push_back(startIndex);

        // 处理双向边（mode=2）
        if (rawEdge.mode == 2) {
            int reverseDir;
            switch (dir) {
                case 1: reverseDir = 3; break;
                case 2: reverseDir = 4; break;
                case 3: reverseDir = 1; break;
                case 4: reverseDir = 2; break;
                default: reverseDir = 0;
            }

            // 构造反向边并添加到列表
            Edge reverseEdge = rawEdge;
            std::swap(reverseEdge.startNodeId, reverseEdge.endNodeId);
            reverseEdge.direction = reverseDir;
            edges.push_back(reverseEdge);
            const Edge* reverseEdgePtr = &edges.back();

            // 反向边的键添加到哈希表
            size_t reverseKey = getEdgeKey(endIndex, startIndex);
            edgeIndex[reverseKey] = edges.size() - 1;
            // 更新反向边的前驱/后继表
            aftNode[endIndex].push_back(startIndex);
            preNode[startIndex].push_back(endIndex);
        }

        reachAnalyzer->unite(startIndex, endIndex);
    }
}


// -------------------------- Getter方法实现 --------------------------
const std::vector<Node>& MapInfo::getNodes() const { return nodes; }
const std::unordered_map<int, int>& MapInfo::getId2Index() const { return id2index; }
const std::unordered_map<int, std::vector<int>>& MapInfo::getPreNode() const { return preNode; }
const std::unordered_map<int, std::vector<int>>& MapInfo::getAftNode() const { return aftNode; }
const UnionFind* MapInfo::getReachAnalyzer() const { return reachAnalyzer; }
double MapInfo::getGlobalMaxSpeed() const { return globalMaxSpeed; }
int MapInfo::getAmrTypeNum() const { return amrTypeNum; }

bool MapInfo::isConnectedById(int startNodeId, int endNodeId) const {
    auto itS = id2index.find(startNodeId);
    auto itE = id2index.find(endNodeId);
    if (itS == id2index.end() || itE == id2index.end()) return false;
    if (!reachAnalyzer) return false;
    return reachAnalyzer->isConnected(itS->second, itE->second);
}

// 获取充电站列表（type=7或9，对应MATLAB的charge_station_list）
std::vector<int> MapInfo::getChargeStationList() const {
    std::vector<int> chargeStations;
    for (const auto& node : nodes) {
        if (node.type == 7 || node.type == 9) {
            chargeStations.push_back(node.index);
        }
    }
    return chargeStations;
}

// 获取有效任务节点列表（type=1或2，且与keyNode连通，对应MATLAB的valid_point_id）
std::vector<int> MapInfo::getValidTaskNodeList(int keyNodeIndex) const {
    std::vector<int> validTaskNodes;
    if (reachAnalyzer == nullptr) {
        throw std::runtime_error("可达性分析器未初始化，无法筛选有效任务节点");
    }

    for (const auto& node : nodes) {
        // 条件1：节点类型为1或2；条件2：与关键节点（keyNode）连通
        if ((node.type == 1 || node.type == 2) && 
            reachAnalyzer->isConnected(node.index, keyNodeIndex)) {
            validTaskNodes.push_back(node.index);
        }
    }
    return validTaskNodes;
}

// 方法1：获取所有有效边列表
const std::vector<Edge>& MapInfo::getEdges() const {
    return edges;  // 返回私有成员edges的常量引用
}

// 方法2：根据索引获取节点（索引从0开始）
const Node& MapInfo::getNode(int nodeIndex) const {
    // 检查索引有效性（节点索引从0开始，且小于等于节点总数）
    if (nodeIndex < 0 || nodeIndex >= static_cast<int>(nodes.size())) {
        throw std::out_of_range("节点索引无效：" + std::to_string(nodeIndex) + 
                              "（有效范围：0~" + std::to_string(nodes.size() - 1) + "）");
    }
    // nodes是0索引vector，nodeIndex-1对应实际位置
    return nodes[nodeIndex];
}

// 方法2b：根据节点ID获取节点（外部统一以ID查询，内部隐式使用映射）
const Node& MapInfo::getNodeById(int nodeId) const {
    auto it = id2index.find(nodeId);
    if (it == id2index.end()) {
        throw std::invalid_argument("节点ID无效：" + std::to_string(nodeId));
    }
    return getNode(it->second);
}


// 辅助函数：生成边的唯一键（将起点和终点索引组合为一个size_t）
// 逻辑：高32位存储起点索引，低32位存储终点索引（支持索引范围0~2^32-1）
size_t MapInfo::getEdgeKey(int startIndex, int endIndex) const {
    return (static_cast<size_t>(startIndex) << 32) | (static_cast<size_t>(endIndex) & 0xFFFFFFFF);
}


// 高效查询边（O(1)时间复杂度）
// 查询边时通过索引访问（修正后）
const Edge* MapInfo::getEdge(int startIndex, int endIndex) const {
    size_t key = getEdgeKey(startIndex, endIndex);
    auto it = edgeIndex.find(key);
    if (it != edgeIndex.end()) {
        size_t edgeIdx = it->second;
        if (edgeIdx < edges.size()) {  // 索引有效性检查
            return &edges[edgeIdx];
        }
    }
    return nullptr;
}

double MapInfo::getEdgeMaxSpeed(int startNodeId, int endNodeId) const {
    double fallback = globalMaxSpeed > 0.0 ? globalMaxSpeed : 1000.0;
    auto itS = id2index.find(startNodeId);
    auto itE = id2index.find(endNodeId);
    if (itS == id2index.end() || itE == id2index.end()) {
        return fallback;
    }
    const Edge* edge = getEdge(itS->second, itE->second);
    if (edge && edge->maxSpeed > 0.0) {
        return edge->maxSpeed;
    }
    const Node& startNode = nodes[itS->second];
    const Node& endNode = nodes[itE->second];
    double speed = fallback;
    if (startNode.maxSpeed > 0.0) {
        speed = std::min(speed, startNode.maxSpeed);
    }
    if (endNode.maxSpeed > 0.0) {
        speed = std::min(speed, endNode.maxSpeed);
    }
    return speed;
}


// 在MapInfo类的public方法中添加以下实现

// 根据坐标(x, y)查找最近的节点ID（从所有节点中搜索）
int MapInfo::findNearestNodeId(double x, double y) const {
    if (nodes.empty()) {
        throw std::runtime_error("节点列表为空，无法查找最近节点");
    }

    int nearestNodeId = -1;
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& node : nodes) {
        // 计算欧氏距离的平方（避免开方运算，提高效率）
        double dx = node.x - x;
        double dy = node.y - y;
        double distanceSq = dx * dx + dy * dy;

        if (distanceSq < minDistance) {
            minDistance = distanceSq;
            nearestNodeId = node.id;
        }
    }

    return nearestNodeId;
}

// 根据坐标(x, y)和参考节点ID，从参考节点的前驱、后继及自身中查找最近节点ID
int MapInfo::findNearestNodeIdFromNeighbors(double x, double y, int refNodeId) const {
    // 查找参考节点的索引
    auto it = id2index.find(refNodeId);
    if (it == id2index.end()) {
        throw std::invalid_argument("参考节点ID无效：" + std::to_string(refNodeId));
    }
    int refIndex = it->second;

    // 收集候选节点：参考节点自身 + 前驱节点 + 后继节点
    std::vector<int> candidateIndices;
    candidateIndices.push_back(refIndex);  // 自身

    // 添加前驱节点
    auto preIt = preNode.find(refIndex);
    if (preIt != preNode.end()) {
        candidateIndices.insert(candidateIndices.end(), preIt->second.begin(), preIt->second.end());
    }

    // 添加后继节点
    auto aftIt = aftNode.find(refIndex);
    if (aftIt != aftNode.end()) {
        candidateIndices.insert(candidateIndices.end(), aftIt->second.begin(), aftIt->second.end());
    }

    // 从候选节点中查找最近节点
    int nearestNodeId = -1;
    double minDistance = std::numeric_limits<double>::max();
    for (int idx : candidateIndices) {
        const Node& node = getNode(idx);  // 利用已有的getNode方法，自带索引检查
        double dx = node.x - x;
        double dy = node.y - y;
        double distanceSq = dx * dx + dy * dy;

        if (distanceSq < minDistance) {
            minDistance = distanceSq;
            nearestNodeId = node.id;
        }
    }

    return nearestNodeId;
}
