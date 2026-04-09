#include "StaticPathTable.h"
#include "common/PathPlanningConstants.h"
#include "data/MapInfo.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <queue>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ctime>
#include <iomanip>
#include <filesystem>

// ==================== 构造函数实现 ====================

StaticPathTable::PathCandidate::PathCandidate()
    : baseDistance(0.0), baseCost(0.0), estimatedTime(0.0),
      turnCount(0), segmentCount(0), avgSegmentLength(0.0),
      quality(OPTIMAL), hasNarrowSection(false), crossMainTraffic(false),
      maxConcurrentAgvs(1) {}

StaticPathTable::PathEntry::PathEntry()
    : startNode(-1), endNode(-1), reachable(false),
      generateTime(0), queryCount(0) {}

StaticPathTable::DynamicContext::DynamicContext()
    : preferLessTurns(false), avoidCongestion(true),
      preferShortest(false), turnPenaltyWeight(1.0) {}

double StaticPathTable::DynamicContext::getNodeCongestion(int nodeId) const {
    auto it = nodeCongestion.find(nodeId);
    return it != nodeCongestion.end() ? it->second : 0.0;
}

StaticPathTable::GenerateConfig::GenerateConfig()
    : topK(3),
      turnPenaltyMm(PathPlanningConstants::resolveTurnPenaltyMm()),
      defaultSpeed(PathPlanningConstants::resolvePlannerSpeedMmPerSec(0.0)),
      numThreads(8), enableCompression(true), verboseOutput(true) {}

StaticPathTable::StaticPathTable() {
    stats_.totalPairs = 0;
    stats_.reachablePairs = 0;
    stats_.unreachablePairs = 0;
    stats_.reachabilityRate = 0.0;
    stats_.totalCandidates = 0;
    stats_.avgCandidatesPerPair = 0.0;
    stats_.fileSizeBytes = 0;
}

// ==================== 节点提取 ====================

std::vector<int> StaticPathTable::extractTaskNodes(const MapInfo& mapInfo) {
    std::vector<int> taskNodes;
    const auto& nodes = mapInfo.getNodes();
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        const Node& node = nodes[i];
        if (node.type == 1 || node.type == 2) {
            taskNodes.push_back(node.id); // 存储节点ID，不是索引！
        }
    }
    
    return taskNodes;
}

std::vector<int> StaticPathTable::extractChargeNodes(const MapInfo& mapInfo) {
    std::vector<int> chargeNodes;
    const auto& nodes = mapInfo.getNodes();
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        const Node& node = nodes[i];
        if (node.type == 7 || node.type == 9) {
            chargeNodes.push_back(node.id); // 存储节点ID，不是索引！
        }
    }
    
    return chargeNodes;
}

// ==================== 路径分析工具 ====================

int StaticPathTable::countTurns(const std::vector<int>& path, const MapInfo& mapInfo) {
    return PathPlanningConstants::countTurnsByIndices(path, mapInfo);
}

std::vector<int> StaticPathTable::extractKeyNodes(
    const std::vector<int>& path,
    const MapInfo& mapInfo
) {
    (void)mapInfo;
    return path;
}

std::vector<int> StaticPathTable::findBottlenecks(
    const std::vector<int>& path,
    const MapInfo& mapInfo
) {
    std::vector<int> bottlenecks;
    const auto& aftNode = mapInfo.getAftNode();
    
    for (int nodeIdx : path) {
        // 瓶颈判断：出度 <= 2（窄通道或交叉口）
        auto it = aftNode.find(nodeIdx);
        if (it != aftNode.end() && it->second.size() <= 2) {
            bottlenecks.push_back(nodeIdx);
        }
    }
    
    return bottlenecks;
}

double StaticPathTable::calculatePathDistance(
    const std::vector<int>& path,
    const MapInfo& mapInfo
) {
    if (path.size() < 2) return 0.0;
    
    double totalDistance = 0.0;
    const auto& nodes = mapInfo.getNodes();
    
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Node& curr = nodes[path[i]];
        const Node& next = nodes[path[i+1]];
        
        double dx = next.x - curr.x;
        double dy = next.y - curr.y;
        totalDistance += std::sqrt(dx*dx + dy*dy);
    }
    
    return totalDistance;
}

double StaticPathTable::calculatePathCost(
    const std::vector<int>& path,
    const MapInfo& mapInfo,
    double turnPenalty
) {
    double distance = calculatePathDistance(path, mapInfo);
    return distance + PathPlanningConstants::calculatePathTurnPenaltyByIndices(path, mapInfo, turnPenalty);
}

void StaticPathTable::analyzePathFeatures(
    PathCandidate& candidate,
    const MapInfo& mapInfo
) {
    // 转弯次数
    candidate.turnCount = countTurns(candidate.fullPath, mapInfo);
    
    // 关键节点
    candidate.keyNodes = extractKeyNodes(candidate.fullPath, mapInfo);
    
    // 瓶颈节点
    candidate.bottleneckNodes = findBottlenecks(candidate.fullPath, mapInfo);
    
    // 路段统计
    candidate.segmentCount = candidate.fullPath.size() - 1;
    if (candidate.segmentCount > 0) {
        candidate.avgSegmentLength = candidate.baseDistance / candidate.segmentCount;
    }
    
    // 窄通道检测
    candidate.hasNarrowSection = !candidate.bottleneckNodes.empty();
    
    // 主干道检测（简化：瓶颈节点多表示经过主干道）
    candidate.crossMainTraffic = candidate.bottleneckNodes.size() > 3;
    
    // 最大并发AGV数（简化估算）
    candidate.maxConcurrentAgvs = std::max(1, 5 - (int)candidate.bottleneckNodes.size());
}

// ==================== A*路径重构 ====================

struct AStarState {
    int prev;
    int curr;

    bool operator==(const AStarState& other) const {
        return prev == other.prev && curr == other.curr;
    }
};

struct StateHasher {
    size_t operator()(const AStarState& s) const {
        size_t h1 = std::hash<int>{}(s.prev);
        size_t h2 = std::hash<int>{}(s.curr);
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

struct AStarNode {
    int prev;
    int curr;
    double gScore;
    double fScore;
};

struct CompareNode {
    bool operator()(const AStarNode& a, const AStarNode& b) const {
        return a.fScore > b.fScore;
    }
};

static std::vector<int> reconstructPath(
    const std::unordered_map<AStarState, AStarState, StateHasher>& cameFrom,
    const AStarState& endState
) {
    std::vector<int> path;
    AStarState current = endState;
    path.push_back(current.curr);

    while (true) {
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) {
            break;
        }
        current = it->second;
        path.push_back(current.curr);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

// 简化的A*实现（返回完整路径）
std::vector<int> runAStarWithPath(
    int startIdx,
    int endIdx,
    const MapInfo& mapInfo,
    double turnPenalty,
    const std::set<std::pair<int,int>>& penalizedEdges = {}
) {
    const int nodeNum = mapInfo.getNodes().size();
    const double INF = std::numeric_limits<double>::infinity();

    auto turnPenaltyCost = [&](int prevIdx, int currIdx, int nextIdx) -> double {
        if (prevIdx < 0) return 0.0;
        const auto& nodes = mapInfo.getNodes();
        if (prevIdx >= nodeNum || currIdx >= nodeNum || nextIdx >= nodeNum) {
            return 0.0;
        }
        return PathPlanningConstants::calculateTurnPenaltyMm(
            nodes[prevIdx],
            nodes[currIdx],
            nodes[nextIdx],
            turnPenalty);
    };

    auto heuristic = [&](int fromIdx, int toIdx) -> double {
        const Node& from = mapInfo.getNodes()[fromIdx];
        const Node& to = mapInfo.getNodes()[toIdx];
        return std::abs(to.x - from.x) + std::abs(to.y - from.y);
    };

    using State = AStarState;
    State startState{-1, startIdx};

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNode> openSet;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, double, StateHasher> gScores;
    std::unordered_map<State, State, StateHasher> cameFrom;

    gScores[startState] = 0.0;
    openSet.push({-1, startIdx, 0.0, heuristic(startIdx, endIdx)});

    const auto& nodes = mapInfo.getNodes();
    const auto& aftNode = mapInfo.getAftNode();

    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();

        State currentState{current.prev, current.curr};
        auto gItCurrent = gScores.find(currentState);
        if (gItCurrent != gScores.end() && current.gScore > gItCurrent->second) {
            continue;
        }

        if (current.curr == endIdx) {
            return reconstructPath(cameFrom, currentState);
        }

        if (closedSet.count(currentState)) {
            continue;
        }
        closedSet.insert(currentState);

        auto it = aftNode.find(current.curr);
        if (it == aftNode.end()) {
            continue;
        }

        for (int neighbor : it->second) {
            if (neighbor < 0 || neighbor >= nodeNum) continue;

            // 检查边是否被惩罚
            double edgePenalty = 0.0;
            if (penalizedEdges.count({current.curr, neighbor})) {
                edgePenalty = 50000.0;
            }

            const Node& currNode = nodes[current.curr];
            const Node& neighNode = nodes[neighbor];

            double dx = neighNode.x - currNode.x;
            double dy = neighNode.y - currNode.y;
            double edgeCost = std::sqrt(dx * dx + dy * dy) + edgePenalty;

            double turnCost = turnPenaltyCost(current.prev, current.curr, neighbor);
            double tentativeG = current.gScore + edgeCost + turnCost;

            State nextState{current.curr, neighbor};
            if (closedSet.count(nextState)) {
                continue;
            }

            auto gIt = gScores.find(nextState);
            if (gIt == gScores.end() || tentativeG < gIt->second) {
                cameFrom[nextState] = currentState;
                gScores[nextState] = tentativeG;
                double fScore = tentativeG + heuristic(neighbor, endIdx);
                openSet.push({current.curr, neighbor, tentativeG, fScore});
            }
        }
    }

    return {};
}

// ==================== Top-K路径计算 ====================

std::vector<StaticPathTable::PathCandidate> StaticPathTable::computeTopKPaths(
    int startIdx,
    int endIdx,
    const MapInfo& mapInfo,
    const GenerateConfig& config
) {
    std::vector<PathCandidate> candidates;
    std::set<std::vector<int>> foundPaths;
    std::set<std::pair<int,int>> penalizedEdges;
    const auto& nodes = mapInfo.getNodes();
    auto toNodeIds = [&](const std::vector<int>& indices) {
        std::vector<int> ids;
        ids.reserve(indices.size());
        for (int idx : indices) {
            if (idx >= 0 && idx < static_cast<int>(nodes.size())) {
                ids.push_back(nodes[idx].id);
            } else {
                ids.push_back(-1);
            }
        }
        return ids;
    };
    
    for (int k = 0; k < config.topK; ++k) {
        // 运行A*
        auto path = runAStarWithPath(startIdx, endIdx, mapInfo, config.turnPenaltyMm, penalizedEdges);
        
        if (path.empty()) break;
        if (foundPaths.count(path)) break;
        
        foundPaths.insert(path);
        
        // 创建候选路径（内部先用索引计算，再转为节点ID）
        PathCandidate candidate;
        candidate.fullPath = path;
        candidate.baseDistance = calculatePathDistance(path, mapInfo);
        candidate.baseCost = calculatePathCost(path, mapInfo, config.turnPenaltyMm);
        candidate.estimatedTime = candidate.baseCost / config.defaultSpeed;
        
        // 分析路径特征
        analyzePathFeatures(candidate, mapInfo);

        // 转换为节点ID，避免后续节点ID/索引混用
        candidate.fullPath = toNodeIds(path);
        candidate.keyNodes = toNodeIds(candidate.keyNodes);
        candidate.bottleneckNodes = toNodeIds(candidate.bottleneckNodes);
        candidate.criticalTurns = toNodeIds(candidate.criticalTurns);
        
        // 分配质量标签
        if (k == 0) {
            candidate.quality = PathCandidate::OPTIMAL;
        } else if (candidate.turnCount < candidates[0].turnCount) {
            candidate.quality = PathCandidate::FAST;
        } else {
            candidate.quality = PathCandidate::SAFE;
        }
        
        candidates.push_back(candidate);
        
        // 惩罚当前路径的边
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            penalizedEdges.insert({path[i], path[i+1]});
        }
    }
    
    return candidates;
}

// ==================== 主生成方法 ====================

bool StaticPathTable::generate(const MapInfo& mapInfo, const GenerateConfig& config) {
    auto startTime = std::chrono::steady_clock::now();
    
    if (config.verboseOutput) {
        std::cout << "========================================" << std::endl;
        std::cout << "  静态路径表生成工具 v1.0" << std::endl;
        std::cout << "  对标: 海康RCS-2000" << std::endl;
        std::cout << "========================================" << std::endl;
    }
    
    // 1. 提取关键节点（现在返回的是节点ID）
    auto taskNodeIds = extractTaskNodes(mapInfo);
    auto chargeNodeIds = extractChargeNodes(mapInfo);
    
    // 使用MapInfo的ID到索引映射（重要：MapInfo会跳过type=-1的节点）
    const auto& nodeIdToIndex = mapInfo.getId2Index();
    const auto& nodes = mapInfo.getNodes();
    
    // 转换为索引（用于内部计算）
    std::vector<int> taskNodeIndices;
    std::vector<int> chargeNodeIndices;
    for (int id : taskNodeIds) {
        auto it = nodeIdToIndex.find(id);
        if (it != nodeIdToIndex.end()) {
            taskNodeIndices.push_back(it->second);
        }
    }
    for (int id : chargeNodeIds) {
        auto it = nodeIdToIndex.find(id);
        if (it != nodeIdToIndex.end()) {
            chargeNodeIndices.push_back(it->second);
        }
    }
    
    if (config.verboseOutput) {
        std::cout << "\n[配置信息]" << std::endl;
        std::cout << "  任务点数量: " << taskNodeIds.size() << std::endl;
        std::cout << "  充电站数量: " << chargeNodeIds.size() << std::endl;
        std::cout << "  Top-K路径: " << config.topK << std::endl;
        std::cout << "  转向惩罚: " << config.turnPenaltyMm << " mm" << std::endl;
        std::cout << "  默认速度: " << config.defaultSpeed << " mm/s" << std::endl;
        std::cout << "  线程数: " << config.numThreads << std::endl;
    }
    
    // 2. 计算路径对数量
    size_t taskToTask = taskNodeIds.size() * (taskNodeIds.size() - 1);
    size_t taskToCharge = taskNodeIds.size() * chargeNodeIds.size();
    size_t chargeToTask = chargeNodeIds.size() * taskNodeIds.size();
    size_t totalPairs = taskToTask + taskToCharge + chargeToTask;
    
    if (config.verboseOutput) {
        std::cout << "\n[路径对统计]" << std::endl;
        std::cout << "  任务点→任务点: " << taskToTask << std::endl;
        std::cout << "  任务点→充电站: " << taskToCharge << std::endl;
        std::cout << "  充电站→任务点: " << chargeToTask << std::endl;
        std::cout << "  总计: " << totalPairs << std::endl;
    }
    
    // 3. 预检测连通性
    if (config.verboseOutput) {
        std::cout << "\n[连通性分析]" << std::endl;
    }
    
    const UnionFind* reachAnalyzer = mapInfo.getReachAnalyzer();
    size_t reachableCount = 0;
    size_t unreachableCount = 0;
    
    for (size_t i = 0; i < taskNodeIndices.size(); ++i) {
        for (size_t j = 0; j < taskNodeIndices.size(); ++j) {
            if (i == j) continue;
            if (reachAnalyzer->isConnected(taskNodeIndices[i], taskNodeIndices[j])) {
                reachableCount++;
            } else {
                unreachableCount++;
            }
        }
    }
    
    if (config.verboseOutput) {
        std::cout << "  可达路径对: " << reachableCount << std::endl;
        std::cout << "  不可达路径对: " << unreachableCount << std::endl;
        std::cout << "  不可达率: " << (100.0 * unreachableCount / (reachableCount + unreachableCount)) << "%" << std::endl;
    }
    
    // 4. 多线程生成
    if (config.verboseOutput) {
        std::cout << "\n[开始生成路径表]" << std::endl;
    }
    
    std::atomic<size_t> completed(0);
    std::atomic<size_t> reachableGenerated(0);
    std::atomic<size_t> unreachableSkipped(0);
    std::vector<std::thread> threads;
    std::mutex tableMutex;
    
    auto processRange = [&](
        const std::vector<int>& startNodes,
        const std::vector<int>& endNodes,
        int threadId,
        int totalThreads
    ) {
        for (size_t i = threadId; i < startNodes.size(); i += totalThreads) {
            for (size_t j = 0; j < endNodes.size(); ++j) {
                int startIdx = startNodes[i];
                int endIdx = endNodes[j];
                
                if (startIdx == endIdx) continue;
                
                PathEntry entry;
                // 存储节点ID，不是索引！
                entry.startNode = nodes[startIdx].id;
                entry.endNode = nodes[endIdx].id;
                entry.generateTime = std::time(nullptr);
                
                // 检查连通性
                if (!reachAnalyzer->isConnected(startIdx, endIdx)) {
                    entry.reachable = false;
                    unreachableSkipped++;
                } else {
                    // 计算Top-K路径
                    entry.candidates = computeTopKPaths(
                        startIdx, endIdx, mapInfo, config
                    );
                    entry.reachable = !entry.candidates.empty();
                    
                    if (entry.reachable) {
                        reachableGenerated++;
                    } else {
                        unreachableSkipped++;
                    }
                }
                
                // 保存（使用节点ID作为key，不是索引！）
                {
                    std::lock_guard<std::mutex> lock(tableMutex);
                    pathTable_[{entry.startNode, entry.endNode}] = entry;
                }
                
                completed++;
                
                if (config.verboseOutput && completed % 1000 == 0) {
                    double progress = 100.0 * completed / totalPairs;
                    std::cout << "  进度: " << completed << "/" << totalPairs 
                             << " (" << std::fixed << std::setprecision(1) << progress << "%)"
                             << " | 可达:" << reachableGenerated 
                             << " 不可达:" << unreachableSkipped
                             << std::endl;
                }
            }
        }
    };
    
    // 5.1 任务点→任务点
    if (config.verboseOutput) {
        std::cout << "\n[1/3] 任务点→任务点" << std::endl;
    }
    
    for (int t = 0; t < config.numThreads; ++t) {
        threads.emplace_back(processRange, 
            std::ref(taskNodeIndices), std::ref(taskNodeIndices), t, config.numThreads);
    }
    for (auto& th : threads) th.join();
    threads.clear();
    
    // 5.2 任务点→充电站
    if (config.verboseOutput) {
        std::cout << "\n[2/3] 任务点→充电站" << std::endl;
    }
    
    for (int t = 0; t < config.numThreads; ++t) {
        threads.emplace_back(processRange,
            std::ref(taskNodeIndices), std::ref(chargeNodeIndices), t, config.numThreads);
    }
    for (auto& th : threads) th.join();
    threads.clear();
    
    // 5.3 充电站→任务点
    if (config.verboseOutput) {
        std::cout << "\n[3/3] 充电站→任务点" << std::endl;
    }
    
    for (int t = 0; t < config.numThreads; ++t) {
        threads.emplace_back(processRange,
            std::ref(chargeNodeIndices), std::ref(taskNodeIndices), t, config.numThreads);
    }
    for (auto& th : threads) th.join();
    
    // 6. 统计信息
    stats_.totalPairs = pathTable_.size();
    stats_.reachablePairs = reachableGenerated;
    stats_.unreachablePairs = unreachableSkipped;
    stats_.reachabilityRate = 100.0 * reachableGenerated / stats_.totalPairs;
    
    size_t totalCandidates = 0;
    for (const auto& [key, entry] : pathTable_) {
        totalCandidates += entry.candidates.size();
    }
    stats_.totalCandidates = totalCandidates;
    stats_.avgCandidatesPerPair = (double)totalCandidates / stats_.totalPairs;
    
    auto endTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
    
    if (config.verboseOutput) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "  生成完成!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "总路径对数: " << stats_.totalPairs << std::endl;
        std::cout << "  可达: " << stats_.reachablePairs 
                 << " (" << stats_.reachabilityRate << "%)" << std::endl;
        std::cout << "  不可达: " << stats_.unreachablePairs << std::endl;
        std::cout << "总候选路径: " << stats_.totalCandidates << std::endl;
        std::cout << "平均候选数: " << std::fixed << std::setprecision(2) 
                 << stats_.avgCandidatesPerPair << std::endl;
        std::cout << "总耗时: " << duration.count() << " 秒 (" 
                 << (duration.count() / 60.0) << " 分钟)" << std::endl;
        std::cout << "平均速度: " << (stats_.totalPairs / std::max(1L, duration.count())) 
                 << " 路径/秒" << std::endl;
    }
    
    return true;
}

// ==================== 查询方法 ====================

std::optional<StaticPathTable::PathCandidate> StaticPathTable::query(
    int startNode,
    int endNode,
    const DynamicContext& context
) const {
    PathKey key{startNode, endNode};
    
    auto it = pathTable_.find(key);
    if (it == pathTable_.end() || !it->second.reachable) {
        return std::nullopt;
    }
    
    std::cout << "[StaticPathTable] hit: " << startNode << " -> " << endNode
              << " (candidates=" << it->second.candidates.size() << ")" << std::endl;
    return selectBestCandidate(it->second.candidates, context);
}

std::optional<StaticPathTable::PathCandidate> StaticPathTable::selectBestCandidate(
    const std::vector<PathCandidate>& candidates,
    const DynamicContext& context
) const {
    if (candidates.empty()) return std::nullopt;

    auto violates_hard_constraints = [&](const PathCandidate& candidate) -> bool {
        if (!context.blockedNodes.empty()) {
            for (int nodeId : candidate.fullPath) {
                if (context.blockedNodes.count(nodeId)) {
                    return true;
                }
            }
        }
        if (!context.blockedEdges.empty() && candidate.fullPath.size() >= 2) {
            for (size_t i = 0; i + 1 < candidate.fullPath.size(); ++i) {
                const int from = candidate.fullPath[i];
                const int to = candidate.fullPath[i + 1];
                if (context.blockedEdges.count({from, to})) {
                    return true;
                }
            }
        }
        return false;
    };

    if (candidates.size() == 1) {
        if (violates_hard_constraints(candidates[0])) return std::nullopt;
        return candidates[0];
    }
    
    double bestScore = -1e9;
    std::optional<PathCandidate> bestCandidate;
    
    for (const auto& candidate : candidates) {
        if (violates_hard_constraints(candidate)) {
            continue;
        }
        double score = scorePath(candidate, context);
        if (score > bestScore) {
            bestScore = score;
            bestCandidate = candidate;
        }
    }
    
    return bestCandidate;
}

double StaticPathTable::scorePath(
    const PathCandidate& candidate,
    const DynamicContext& context
) const {
    double score = 0.0;
    
    // 基础成本（权重50%）
    score -= 0.5 * candidate.baseCost;
    
    // 拥堵惩罚（权重30%）
    if (context.avoidCongestion) {
        double congestion = 0.0;
        for (int nodeId : candidate.bottleneckNodes) {
            congestion += context.getNodeCongestion(nodeId);
        }
        score -= 0.3 * congestion * 10000.0;
    }
    
    // 转弯惩罚（权重20%）
    if (context.preferLessTurns) {
        score -= 0.2 * candidate.turnCount * context.turnPenaltyWeight * 1000.0;
    }
    
    return score;
}

// ==================== 文件保存/加载 ====================

bool StaticPathTable::saveToFile(const std::string& filePath) const {
    std::ofstream ofs(filePath, std::ios::binary);
    if (!ofs) {
        std::cerr << "无法创建文件: " << filePath << std::endl;
        return false;
    }
    
    // 文件头
    uint32_t magic = 0x48495041; // "HIPA" (HIkvision PAth)
    uint32_t version = 2;
    uint32_t entryCount = pathTable_.size();
    uint64_t timestamp = std::time(nullptr);
    
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&entryCount), sizeof(entryCount));
    ofs.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp));
    
    // 写入统计信息
    ofs.write(reinterpret_cast<const char*>(&stats_.totalPairs), sizeof(stats_.totalPairs));
    ofs.write(reinterpret_cast<const char*>(&stats_.reachablePairs), sizeof(stats_.reachablePairs));
    ofs.write(reinterpret_cast<const char*>(&stats_.unreachablePairs), sizeof(stats_.unreachablePairs));
    
    // 写入每个条目
    for (const auto& [key, entry] : pathTable_) {
        ofs.write(reinterpret_cast<const char*>(&entry.startNode), sizeof(entry.startNode));
        ofs.write(reinterpret_cast<const char*>(&entry.endNode), sizeof(entry.endNode));
        ofs.write(reinterpret_cast<const char*>(&entry.reachable), sizeof(entry.reachable));
        
        uint8_t candidateCount = entry.candidates.size();
        ofs.write(reinterpret_cast<const char*>(&candidateCount), sizeof(candidateCount));
        
        for (const auto& candidate : entry.candidates) {
            // 完整路径
            uint16_t pathSize = candidate.fullPath.size();
            ofs.write(reinterpret_cast<const char*>(&pathSize), sizeof(pathSize));
            if (pathSize > 0) {
                ofs.write(reinterpret_cast<const char*>(candidate.fullPath.data()), 
                         pathSize * sizeof(int));
            }
            
            // 关键节点
            uint16_t keySize = candidate.keyNodes.size();
            ofs.write(reinterpret_cast<const char*>(&keySize), sizeof(keySize));
            if (keySize > 0) {
                ofs.write(reinterpret_cast<const char*>(candidate.keyNodes.data()),
                         keySize * sizeof(int));
            }
            
            // 成本信息
            ofs.write(reinterpret_cast<const char*>(&candidate.baseDistance), sizeof(candidate.baseDistance));
            ofs.write(reinterpret_cast<const char*>(&candidate.baseCost), sizeof(candidate.baseCost));
            ofs.write(reinterpret_cast<const char*>(&candidate.estimatedTime), sizeof(candidate.estimatedTime));
            
            // 特征
            ofs.write(reinterpret_cast<const char*>(&candidate.turnCount), sizeof(candidate.turnCount));
            ofs.write(reinterpret_cast<const char*>(&candidate.quality), sizeof(candidate.quality));
        }
    }
    
    ofs.close();
    
    // 获取文件大小
    std::ifstream ifs(filePath, std::ios::binary | std::ios::ate);
    const_cast<Statistics&>(stats_).fileSizeBytes = ifs.tellg();
    
    std::cout << "\n路径表已保存: " << filePath << std::endl;
    std::cout << "文件大小: " << (stats_.fileSizeBytes / 1024.0 / 1024.0) << " MB" << std::endl;
    
    return true;
}

bool StaticPathTable::loadFromFile(const std::string& filePath) {
    namespace fs = std::filesystem;
    std::string resolvedPath = filePath;
    std::error_code ec;
    auto canonicalPath = fs::canonical(fs::path(filePath), ec);
    if (!ec) {
        resolvedPath = canonicalPath.string();
    } else {
        ec.clear();
        auto absPath = fs::absolute(fs::path(filePath), ec);
        if (!ec) {
            resolvedPath = absPath.lexically_normal().string();
        }
    }
    std::cout << "加载静态路径表文件: " << resolvedPath;
    if (resolvedPath != filePath) {
        std::cout << " (请求: " << filePath << ")";
    }
    std::cout << std::endl;

    std::ifstream ifs(filePath, std::ios::binary);
    if (!ifs) {
        std::cerr << "无法打开文件: " << filePath << std::endl;
        return false;
    }
    
    // 读取文件头
    uint32_t magic, version, entryCount;
    uint64_t timestamp;
    
    ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    ifs.read(reinterpret_cast<char*>(&entryCount), sizeof(entryCount));
    ifs.read(reinterpret_cast<char*>(&timestamp), sizeof(timestamp));
    
    if (magic != 0x48495041) {
        std::cerr << "无效的文件格式" << std::endl;
        return false;
    }
    
    if (version != 2) {
        std::cerr << "不支持的路径表版本: " << version
                  << " (需要重新生成静态表)" << std::endl;
        return false;
    }

    std::cout << "加载路径表: 版本=" << version << ", 条目数=" << entryCount << std::endl;
    
    // 读取统计信息
    ifs.read(reinterpret_cast<char*>(&stats_.totalPairs), sizeof(stats_.totalPairs));
    ifs.read(reinterpret_cast<char*>(&stats_.reachablePairs), sizeof(stats_.reachablePairs));
    ifs.read(reinterpret_cast<char*>(&stats_.unreachablePairs), sizeof(stats_.unreachablePairs));
    
    // 读取条目
    for (uint32_t i = 0; i < entryCount; ++i) {
        PathEntry entry;
        
        ifs.read(reinterpret_cast<char*>(&entry.startNode), sizeof(entry.startNode));
        ifs.read(reinterpret_cast<char*>(&entry.endNode), sizeof(entry.endNode));
        ifs.read(reinterpret_cast<char*>(&entry.reachable), sizeof(entry.reachable));
        
        uint8_t candidateCount;
        ifs.read(reinterpret_cast<char*>(&candidateCount), sizeof(candidateCount));
        
        for (uint8_t j = 0; j < candidateCount; ++j) {
            PathCandidate candidate;
            
            // 完整路径
            uint16_t pathSize;
            ifs.read(reinterpret_cast<char*>(&pathSize), sizeof(pathSize));
            if (pathSize > 0) {
                candidate.fullPath.resize(pathSize);
                ifs.read(reinterpret_cast<char*>(candidate.fullPath.data()),
                        pathSize * sizeof(int));
            }
            
            // 关键节点
            uint16_t keySize;
            ifs.read(reinterpret_cast<char*>(&keySize), sizeof(keySize));
            if (keySize > 0) {
                candidate.keyNodes.resize(keySize);
                ifs.read(reinterpret_cast<char*>(candidate.keyNodes.data()),
                        keySize * sizeof(int));
            }
            
            // 成本信息
            ifs.read(reinterpret_cast<char*>(&candidate.baseDistance), sizeof(candidate.baseDistance));
            ifs.read(reinterpret_cast<char*>(&candidate.baseCost), sizeof(candidate.baseCost));
            ifs.read(reinterpret_cast<char*>(&candidate.estimatedTime), sizeof(candidate.estimatedTime));
            
            // 特征
            ifs.read(reinterpret_cast<char*>(&candidate.turnCount), sizeof(candidate.turnCount));
            ifs.read(reinterpret_cast<char*>(&candidate.quality), sizeof(candidate.quality));
            
            entry.candidates.push_back(candidate);
        }
        
        pathTable_[{entry.startNode, entry.endNode}] = entry;
        
        if ((i + 1) % 10000 == 0) {
            std::cout << "  已加载: " << (i + 1) << "/" << entryCount << std::endl;
        }
    }
    
    ifs.close();
    std::cout << "✅ 加载完成!" << std::endl;
    
    return true;
}

// ==================== 统计与验证 ====================

StaticPathTable::Statistics StaticPathTable::getStatistics() const {
    return stats_;
}

bool StaticPathTable::validate() const {
    if (pathTable_.empty()) {
        std::cerr << "路径表为空" << std::endl;
        return false;
    }
    
    size_t invalidCount = 0;
    for (const auto& [key, entry] : pathTable_) {
        if (entry.reachable && entry.candidates.empty()) {
            invalidCount++;
        }
    }
    
    if (invalidCount > 0) {
        std::cerr << "发现 " << invalidCount << " 个无效条目" << std::endl;
        return false;
    }
    
    return true;
}
