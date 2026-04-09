#include "GlobalPathPlanner.h"
#include "common/PathPlanningConstants.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <unordered_set>

static bool astar_success_logging_enabled() {
    const char* v = std::getenv("ASTAR_LOG_SUCCESS");
    if (!v) return false;
    std::string s(v);
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return (s == "1" || s == "true" || s == "yes" || s == "on");
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 构造与初始化
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

GlobalPathPlanner::GlobalPathPlanner(const MapInfo& mapInfo, const Config& config)
    : mapInfo_(mapInfo)
    , config_(config)
    , initialized_(false)
{
    // 创建A*规划器
    aStarPlanner_ = std::make_unique<AStarPathFinder>(mapInfo_);
}

bool GlobalPathPlanner::initialize() {
    logInfo("初始化全局路径规划器...");

    // 1. 加载静态路径表（纯 A* 或未提供路径表时跳过）
    if (config_.forcePureAStar || (config_.staticTablePath.empty() && externalStaticTable_ == nullptr)) {
        logWarn("纯 A* 模式，跳过静态路径表加载。");
    } else {
        if (externalStaticTable_) {
            logInfo("使用预加载静态路径表，跳过文件加载。");
        } else {
            if (!staticTable_.loadFromFile(config_.staticTablePath)) {
                logError("静态路径表加载失败: " + config_.staticTablePath);
                return false;
            }
            logInfo("静态路径表加载成功");
        }
    }
    
    // 2. 提取所有关键节点（与StaticPathTable保持一致）
    allKeyNodes_.clear();
    const auto& nodes = mapInfo_.getNodes();
    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& node = nodes[i];
        // 任务点(type=1或2) 和 充电站(type=7或9) 是关键节点
        if (node.type == 1 || node.type == 2 || node.type == 7 || node.type == 9) {
            allKeyNodes_.push_back(node.id);
        }
    }
    
    logInfo("关键节点数量: " + std::to_string(allKeyNodes_.size()));
    
    // 3. 预计算最近关键节点缓存（可选）
    if (config_.enableNearestNodeCache) {
        logInfo("预计算最近关键节点缓存...");
        // 为所有普通节点预计算最近的关键节点
        for (size_t i = 0; i < nodes.size(); ++i) {
            const auto& node = nodes[i];
            if (node.type == 0) {  // 普通节点
                findNearestKeyNode(node.id);  // 会自动缓存
            }
        }
        logInfo("缓存构建完成，缓存大小: " + std::to_string(nearestKeyNodeCache_.size()));
    }
    
    initialized_ = true;
    logInfo("全局路径规划器初始化完成");
    
    return true;
}

void GlobalPathPlanner::setStaticPathTable(const StaticPathTable* table) {
    externalStaticTable_ = table;
}

const StaticPathTable* GlobalPathPlanner::staticTablePtr() const {
    return externalStaticTable_ ? externalStaticTable_ : &staticTable_;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 核心接口实现
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

GlobalPathPlanner::PathResult GlobalPathPlanner::planGlobalPath(
    int startNode, 
    int endNode,
    const StaticPathTable::DynamicContext& context
) {
    if (!initialized_) {
        logError("规划器未初始化！");
        return PathResult{};
    }
    
    auto startTime = std::chrono::high_resolution_clock::now();
    stats_.totalQueries++;

    // 纯 A* 模式（未提供静态表路径且无外部静态表）：直接用 A*
    if (config_.forcePureAStar ||
        (config_.staticTablePath.empty() && externalStaticTable_ == nullptr)) {
        auto path = aStarSearch(startNode, endNode, context);
        PathResult result;
        result.path = path;
        result.fullPath = path;
        if (!path.empty()) {
            result.reachable = true;
            result.source = PathSource::PURE_ASTAR;
            double dist = 0.0;
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                dist += euclideanDistance(path[i], path[i + 1]);
            }
            result.distance = dist;
            double speed = mapInfo_.getGlobalMaxSpeed();
            if (!(speed > 0.0)) speed = 1000.0;
            result.estimatedTime = dist / speed;
        } else {
            result.reachable = false;
            result.source = PathSource::UNREACHABLE;
        }
        auto endTime = std::chrono::high_resolution_clock::now();
        result.planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        stats_.pureAStarCount++;
        stats_.totalPlanningTime += result.planningTime;
        if (!result.reachable) stats_.unreachableCount++;
        stats_.update();
        return result;
    }
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 策略1：优先查询静态表（数据驱动）
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 调试输出
    if (config_.verboseLogging) {
        std::cout << "[DEBUG] 查询静态表: " << startNode << " -> " << endNode << std::endl;
    }
    
    auto staticResult = staticTablePtr()->query(startNode, endNode, context);
    
    if (staticResult.has_value()) {
        // 命中静态表 → 直接返回
        stats_.staticTableHits++;
        
        auto endTime = std::chrono::high_resolution_clock::now();
        double planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        stats_.totalPlanningTime += planningTime;
        stats_.update();
        
        logInfo("路径来源: 静态表查询 (耗时: " + std::to_string(planningTime) + " ms)");
        
        PathResult result;
        result.path = staticResult->fullPath;
        result.fullPath = staticResult->fullPath;
        result.distance = staticResult->baseDistance;
        result.estimatedTime = staticResult->estimatedTime;
        result.turnCount = staticResult->turnCount;
        result.reachable = true;
        result.source = PathSource::STATIC_TABLE;
        result.planningTime = planningTime;
        
        return result;
    }
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 策略2：静态表未命中 → 混合规划
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    if (!config_.enableMixedPlanning) {
        logWarn("混合规划已禁用，无法规划路径");
        stats_.unreachableCount++;
        stats_.update();
        
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    logInfo("路径来源: 混合规划（静态表+动态A*）");
    
    PathResult result = mixedPlanning(startNode, endNode, context);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    double planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    result.planningTime = planningTime;
    stats_.totalPlanningTime += planningTime;
    stats_.update();
    
    return result;
}

std::vector<GlobalPathPlanner::PathResult> GlobalPathPlanner::planBatchPaths(
    const std::vector<std::pair<int, int>>& requests,
    const StaticPathTable::DynamicContext& context
) {
    std::vector<PathResult> results;
    results.reserve(requests.size());
    
    for (const auto& [start, end] : requests) {
        results.push_back(planGlobalPath(start, end, context));
    }
    
    return results;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 混合规划实现
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

GlobalPathPlanner::PathResult GlobalPathPlanner::mixedPlanning(
    int startNode, 
    int endNode,
    const StaticPathTable::DynamicContext& context
) {
    // 检查起点和终点是否在静态表中
    bool startInTable = isNodeInStaticTable(startNode);
    bool endInTable = isNodeInStaticTable(endNode);
    
    if (!startInTable && !endInTable) {
        // 情况1: 起点和终点都不在表中
        return case1_BothNotInTable(startNode, endNode, context);
    } 
    else if (!startInTable && endInTable) {
        // 情况2: 只有起点不在表中（最常见：AGV当前位置→任务点）
        return case2_StartNotInTable(startNode, endNode, context);
    } 
    else if (startInTable && !endInTable) {
        // 情况3: 只有终点不在表中
        return case3_EndNotInTable(startNode, endNode, context);
    } 
    else {
        // 情况4: 都在表中但查询失败（不可达或表损坏）
        return case4_BothInTableButFailed(startNode, endNode, context);
    }
}

GlobalPathPlanner::PathResult GlobalPathPlanner::case1_BothNotInTable(
    int start, int end, 
    const StaticPathTable::DynamicContext& context
) {
    logInfo("情况1: 起点和终点都不在静态表");
    
    // 检查是否是短距离，直接用A*
    double directDist = euclideanDistance(start, end);
    if (directDist < config_.shortDistanceThreshold) {
        logInfo("短距离路径，直接使用A*");
        auto path = aStarSearch(start, end, context);
        
        if (path.empty()) {
            stats_.unreachableCount++;
            PathResult result;
            result.reachable = false;
            result.source = PathSource::UNREACHABLE;
            return result;
        }
        
        stats_.pureAStarCount++;
        
        PathResult result;
        result.path = path;
        result.fullPath = path;
        result.distance = calculatePathDistance(path);
        result.estimatedTime = calculateEstimatedTime(path, result.distance);
        result.turnCount = countTurns(path);
        result.reachable = true;
        result.source = PathSource::PURE_ASTAR;
        
        return result;
    }
    
    // 找两个最近的关键节点，通过它们中转
    int nearestToStart = findNearestKeyNode(start);
    int nearestToEnd = findNearestKeyNode(end);
    
    if (nearestToStart == -1 || nearestToEnd == -1) {
        logWarn("无法找到最近的关键节点，降级到纯A*");
        // Fallback: 直接尝试纯A*
        auto path = aStarSearch(start, end, context);
        if (!path.empty()) {
            stats_.pureAStarCount++;
            PathResult result;
            result.path = path;
            result.fullPath = path;
            result.distance = calculatePathDistance(path);
            result.estimatedTime = calculateEstimatedTime(path, result.distance);
            result.turnCount = countTurns(path);
            result.reachable = true;
            result.source = PathSource::PURE_ASTAR;
            return result;
        }
        
        // 真的不可达
        logError("纯A*也失败，路径确实不可达");
        stats_.unreachableCount++;
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    // 计算三段路径
    auto segment1 = aStarSearch(start, nearestToStart, context);           // A*
    auto staticResult = staticTablePtr()->query(nearestToStart, nearestToEnd, context); // 静态表
    auto segment3 = aStarSearch(nearestToEnd, end, context);               // A*
    
    if (segment1.empty() || !staticResult.has_value() || segment3.empty()) {
        logWarn("路径段计算失败，尝试纯A*");
        auto path = aStarSearch(start, end, context);
        if (path.empty()) {
            stats_.unreachableCount++;
            PathResult result;
            result.reachable = false;
            result.source = PathSource::UNREACHABLE;
            return result;
        }
        
        stats_.pureAStarCount++;
        
        PathResult result;
        result.path = path;
        result.fullPath = path;
        result.distance = calculatePathDistance(path);
        result.estimatedTime = calculateEstimatedTime(path, result.distance);
        result.turnCount = countTurns(path);
        result.reachable = true;
        result.source = PathSource::PURE_ASTAR;
        
        return result;
    }
    
    // 拼接路径
    std::vector<std::vector<int>> segments = {segment1, staticResult->fullPath, segment3};
    stats_.mixedPlanningCount++;
    
    return concatenatePaths(segments, PathSource::MIXED_PLANNING);
}

GlobalPathPlanner::PathResult GlobalPathPlanner::case2_StartNotInTable(
    int start, int end,
    const StaticPathTable::DynamicContext& context
) {
    logInfo("情况2: 起点不在静态表（AGV当前位置→任务点）");
    
    // 找离起点最近的关键节点
    int nearestKeyNode = findNearestKeyNode(start);
    
    if (nearestKeyNode == -1) {
        logWarn("无法找到最近的关键节点，降级到纯A*");
        // Fallback 1: 直接尝试纯A*
        auto path = aStarSearch(start, end, context);
        if (!path.empty()) {
            stats_.pureAStarCount++;
            PathResult result;
            result.path = path;
            result.fullPath = path;
            result.distance = calculatePathDistance(path);
            result.estimatedTime = calculateEstimatedTime(path, result.distance);
            result.turnCount = countTurns(path);
            result.reachable = true;
            result.source = PathSource::PURE_ASTAR;
            return result;
        }
        
        // 真的不可达
        logError("纯A*也失败，路径确实不可达");
        stats_.unreachableCount++;
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    // 两段路径
    auto segment1 = aStarSearch(start, nearestKeyNode, context);        // A*
    auto staticResult = staticTablePtr()->query(nearestKeyNode, end, context);  // 静态表
    
    if (segment1.empty() || !staticResult.has_value()) {
        logWarn("路径段计算失败，降级到纯A*");
        // Fallback 2: 混合规划失败，尝试纯A*
        auto path = aStarSearch(start, end, context);
        if (!path.empty()) {
            stats_.pureAStarCount++;
            PathResult result;
            result.path = path;
            result.fullPath = path;
            result.distance = calculatePathDistance(path);
            result.estimatedTime = calculateEstimatedTime(path, result.distance);
            result.turnCount = countTurns(path);
            result.reachable = true;
            result.source = PathSource::PURE_ASTAR;
            return result;
        }
        
        // 真的不可达
        logError("纯A*也失败，路径确实不可达");
        stats_.unreachableCount++;
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    std::vector<std::vector<int>> segments = {segment1, staticResult->fullPath};
    stats_.mixedPlanningCount++;
    
    return concatenatePaths(segments, PathSource::MIXED_PLANNING);
}

GlobalPathPlanner::PathResult GlobalPathPlanner::case3_EndNotInTable(
    int start, int end,
    const StaticPathTable::DynamicContext& context
) {
    logInfo("情况3: 终点不在静态表");
    
    int nearestKeyNode = findNearestKeyNode(end);
    
    if (nearestKeyNode == -1) {
        logWarn("无法找到最近的关键节点，降级到纯A*");
        // Fallback 1: 直接尝试纯A*
        auto path = aStarSearch(start, end, context);
        if (!path.empty()) {
            stats_.pureAStarCount++;
            PathResult result;
            result.path = path;
            result.fullPath = path;
            result.distance = calculatePathDistance(path);
            result.estimatedTime = calculateEstimatedTime(path, result.distance);
            result.turnCount = countTurns(path);
            result.reachable = true;
            result.source = PathSource::PURE_ASTAR;
            return result;
        }
        
        // 真的不可达
        logError("纯A*也失败，路径确实不可达");
        stats_.unreachableCount++;
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    auto staticResult = staticTablePtr()->query(start, nearestKeyNode, context);  // 静态表
    auto segment2 = aStarSearch(nearestKeyNode, end, context);          // A*
    
    if (!staticResult.has_value() || segment2.empty()) {
        logWarn("路径段计算失败，降级到纯A*");
        // Fallback 2: 混合规划失败，尝试纯A*
        auto path = aStarSearch(start, end, context);
        if (!path.empty()) {
            stats_.pureAStarCount++;
            PathResult result;
            result.path = path;
            result.fullPath = path;
            result.distance = calculatePathDistance(path);
            result.estimatedTime = calculateEstimatedTime(path, result.distance);
            result.turnCount = countTurns(path);
            result.reachable = true;
            result.source = PathSource::PURE_ASTAR;
            return result;
        }
        
        // 真的不可达
        logError("纯A*也失败，路径确实不可达");
        stats_.unreachableCount++;
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    std::vector<std::vector<int>> segments = {staticResult->fullPath, segment2};
    stats_.mixedPlanningCount++;
    
    return concatenatePaths(segments, PathSource::MIXED_PLANNING);
}

GlobalPathPlanner::PathResult GlobalPathPlanner::case4_BothInTableButFailed(
    int start, int end,
    const StaticPathTable::DynamicContext& context
) {
    logWarn("情况4: 静态表查询失败，降级到纯A*");
    
    auto path = aStarSearch(start, end, context);
    
    if (path.empty()) {
        stats_.unreachableCount++;
        PathResult result;
        result.reachable = false;
        result.source = PathSource::UNREACHABLE;
        return result;
    }
    
    stats_.pureAStarCount++;
    
    PathResult result;
    result.path = path;
    result.fullPath = path;
    result.distance = calculatePathDistance(path);
    result.estimatedTime = calculateEstimatedTime(path, result.distance);
    result.turnCount = countTurns(path);
    result.reachable = true;
    result.source = PathSource::PURE_ASTAR;
    
    return result;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 辅助函数实现
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

int GlobalPathPlanner::findNearestKeyNode(int nodeId) {
    // 如果已经是关键节点，直接返回
    if (isNodeInStaticTable(nodeId)) {
        return nodeId;
    }
    
    // 检查缓存
    if (config_.enableNearestNodeCache) {
        auto it = nearestKeyNodeCache_.find(nodeId);
        if (it != nearestKeyNodeCache_.end()) {
            return it->second;
        }
    }
    
    // 调试：检查关键节点列表
    if (allKeyNodes_.empty()) {
        logError("关键节点列表为空！");
        return -1;
    }
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 优化策略：增强型最近节点搜索（对标海康RCS-2000）
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 策略1: 欧几里得距离快速筛选（增加候选数到50）
    auto candidates = findNearestKKeyNodesByEuclidean(nodeId, config_.nearestKeyCandidates);
    
    if (candidates.empty()) {
        logError("无法找到候选关键节点（欧几里得距离筛选失败）");
        logError("节点ID: " + std::to_string(nodeId) + ", 关键节点总数: " + std::to_string(allKeyNodes_.size()));
        return -1;
    }
    
    // 策略2: 实际路径距离精确选择（并行验证多个候选）
    int bestNode = -1;
    double minDist = std::numeric_limits<double>::infinity();
    int successCount = 0;
    
    for (int candidate : candidates) {
        auto path = aStarSearch(nodeId, candidate, StaticPathTable::DynamicContext());
        if (!path.empty()) {
            double dist = calculatePathDistance(path);
            successCount++;
            
            if (dist < minDist) {
                minDist = dist;
                bestNode = candidate;
            }
            
            // 早停优化：如果找到非常近的节点（<5m），直接返回
            if (dist < 5000.0) {
                break;
            }
        }
    }
    
    // 策略3: 如果A*成功率低，降级到欧几里得距离
    if (successCount == 0 && !candidates.empty()) {
        logWarn("所有候选节点A*搜索失败（" + std::to_string(candidates.size()) + "个），降级使用欧几里得距离");
        bestNode = candidates[0];
    }
    
    // 策略4: 如果成功率很低（<10%），记录警告
    if (successCount > 0 && successCount < candidates.size() / 10) {
        logWarn("A*成功率低: " + std::to_string(successCount) + "/" + std::to_string(candidates.size()) + 
                " (节点ID: " + std::to_string(nodeId) + ")");
    }
    
    // 缓存结果
    if (config_.enableNearestNodeCache && bestNode != -1) {
        nearestKeyNodeCache_[nodeId] = bestNode;
    }
    
    if (bestNode == -1) {
        logError("findNearestKeyNode完全失败，节点ID: " + std::to_string(nodeId));
    }
    
    return bestNode;
}

std::vector<int> GlobalPathPlanner::findNearestKKeyNodesByEuclidean(int nodeId, int k) {
    const auto& nodes = mapInfo_.getNodes();
    
    // 找到目标节点
    const Node* targetNode = nullptr;
    for (const auto& node : nodes) {
        if (node.id == nodeId) {
            targetNode = &node;
            break;
        }
    }
    
    if (!targetNode) {
        return {};
    }
    
    // 计算所有关键节点的欧几里得距离
    using DistNodePair = std::pair<double, int>;
    std::priority_queue<DistNodePair, std::vector<DistNodePair>, std::greater<DistNodePair>> pq;
    
    for (int keyNodeId : allKeyNodes_) {
        if (keyNodeId == nodeId) continue;
        
        double dist = euclideanDistance(nodeId, keyNodeId);
        pq.push({dist, keyNodeId});
    }
    
    // 取前K个
    std::vector<int> result;
    for (int i = 0; i < k && !pq.empty(); ++i) {
        result.push_back(pq.top().second);
        pq.pop();
    }
    
    return result;
}

std::vector<int> GlobalPathPlanner::aStarSearch(int start, int end, const StaticPathTable::DynamicContext& context) {
    const bool cacheable = context.blockedNodes.empty() && context.blockedEdges.empty();
    // 检查短路径缓存（仅对无动态阻塞的请求启用）
    if (cacheable && config_.enableShortPathCache) {
        PathCacheKey key{start, end};
        auto it = shortPathCache_.find(key);
        if (it != shortPathCache_.end()) {
            return it->second;
        }
    }
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 增强型A*搜索：多策略重试机制（提升鲁棒性）
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    std::vector<int> path;

    std::unordered_set<int> banned;
    const std::unordered_set<int>* bannedPtr = nullptr;
    if (!context.blockedNodes.empty()) {
        banned.insert(context.blockedNodes.begin(), context.blockedNodes.end());
        banned.erase(start);
        banned.erase(end);
        if (!banned.empty()) bannedPtr = &banned;
    }
    const std::set<std::pair<int,int>>* bannedEdgesPtr = nullptr;
    if (!context.blockedEdges.empty()) {
        bannedEdgesPtr = &context.blockedEdges;
    }
    
    // 策略1: 标准A*搜索（转弯惩罚3000mm）
    const double defaultTurnPenaltyMm = PathPlanningConstants::resolveTurnPenaltyMm();
    auto result = aStarPlanner_->findPath(start, end, defaultTurnPenaltyMm, bannedPtr, bannedEdgesPtr);
    if (result.found) {
        path = result.path;
        if (config_.verboseLogging && astar_success_logging_enabled()) {
            logInfo("A*策略1成功：标准搜索");
        }
    }
    
    // 策略2: 如果失败，减小转弯惩罚重试（允许更多转弯）
    if (path.empty()) {
        result = aStarPlanner_->findPath(start, end, defaultTurnPenaltyMm / 3.0, bannedPtr, bannedEdgesPtr);
        if (result.found) {
            path = result.path;
            if (config_.verboseLogging && astar_success_logging_enabled()) {
                logInfo("A*策略2成功：减小转弯惩罚");
            }
        }
    }
    
    // 策略3: 如果还失败，完全取消转弯惩罚
    if (path.empty()) {
        result = aStarPlanner_->findPath(start, end, 0.0, bannedPtr, bannedEdgesPtr);
        if (result.found) {
            path = result.path;
            if (config_.verboseLogging && astar_success_logging_enabled()) {
                logInfo("A*策略3成功：取消转弯惩罚");
            }
        }
    }
    
    // 策略4: 尝试通过中转点（找一个中间的关键节点）
    if (path.empty()) {
        auto nearestToStart = findNearestKKeyNodesByEuclidean(start, 3);
        auto nearestToEnd = findNearestKKeyNodesByEuclidean(end, 3);
        
        for (int midStart : nearestToStart) {
            if (midStart == start || midStart == end) continue;
            if (bannedPtr && bannedPtr->count(midStart)) continue;
            
            for (int midEnd : nearestToEnd) {
                if (midEnd == start || midEnd == end || midEnd == midStart) continue;
                if (bannedPtr && bannedPtr->count(midEnd)) continue;
                
                auto seg1 = aStarPlanner_->findPath(start, midStart, 0.0, bannedPtr, bannedEdgesPtr);
                auto seg2 = aStarPlanner_->findPath(midStart, midEnd, 0.0, bannedPtr, bannedEdgesPtr);
                auto seg3 = aStarPlanner_->findPath(midEnd, end, 0.0, bannedPtr, bannedEdgesPtr);
                
                if (seg1.found && seg2.found && seg3.found) {
                    // 拼接路径
                    path = seg1.path;
                    for (size_t i = 1; i < seg2.path.size(); ++i) {
                        path.push_back(seg2.path[i]);
                    }
                    for (size_t i = 1; i < seg3.path.size(); ++i) {
                        path.push_back(seg3.path[i]);
                    }
                    
                    if (config_.verboseLogging && astar_success_logging_enabled()) {
                        logInfo("A*策略4成功：中转点路径 (" + 
                                std::to_string(midStart) + " -> " + 
                                std::to_string(midEnd) + ")");
                    }
                    break;
                }
            }
            if (!path.empty()) break;
        }
    }
    
    // 策略5: 尝试直接连接到任意可达的关键节点
    if (path.empty()) {
        auto candidates = findNearestKKeyNodesByEuclidean(end, 10);
        for (int candidate : candidates) {
            if (candidate == start || candidate == end) continue;
            if (bannedPtr && bannedPtr->count(candidate)) continue;
            
            auto seg1 = aStarPlanner_->findPath(start, candidate, 0.0, bannedPtr, bannedEdgesPtr);
            auto seg2 = aStarPlanner_->findPath(candidate, end, 0.0, bannedPtr, bannedEdgesPtr);
            
            if (seg1.found && seg2.found) {
                path = seg1.path;
                for (size_t i = 1; i < seg2.path.size(); ++i) {
                    path.push_back(seg2.path[i]);
                }
                
                if (config_.verboseLogging && astar_success_logging_enabled()) {
                    logInfo("A*策略5成功：单中转点 (" + std::to_string(candidate) + ")");
                }
                break;
            }
        }
    }
    
    // 如果所有策略都失败
    if (path.empty() && config_.verboseLogging) {
        logWarn("A*所有5种策略均失败: " + std::to_string(start) + " -> " + std::to_string(end));
    }
    
    // 缓存结果（LRU策略：简单实现，超过大小就清空）
    if (cacheable && config_.enableShortPathCache && !path.empty()) {
        if (shortPathCache_.size() >= static_cast<size_t>(config_.shortPathCacheSize)) {
            shortPathCache_.clear();  // 简单清空策略
        }
        PathCacheKey key{start, end};
        shortPathCache_[key] = path;
    }
    
    return path;
}

GlobalPathPlanner::PathResult GlobalPathPlanner::concatenatePaths(
    const std::vector<std::vector<int>>& segments,
    PathSource source
) {
    PathResult result;
    result.source = source;
    result.reachable = true;
    
    // 拼接路径，去除重复节点
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        if (segment.empty()) continue;
        
        if (result.path.empty()) {
            result.path = segment;
        } else {
            // 跳过重复的起点
            for (size_t j = 1; j < segment.size(); ++j) {
                result.path.push_back(segment[j]);
            }
        }
    }
    
    // 计算路径属性
    result.distance = calculatePathDistance(result.path);
    result.estimatedTime = calculateEstimatedTime(result.path, result.distance);
    result.turnCount = countTurns(result.path);
    result.fullPath = result.path;
    
    return result;
}

double GlobalPathPlanner::calculatePathDistance(const std::vector<int>& path) {
    if (path.size() < 2) {
        return 0.0;
    }
    
    const auto& nodes = mapInfo_.getNodes();
    double totalDist = 0.0;
    
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Node* node1 = nullptr;
        const Node* node2 = nullptr;
        
        for (const auto& node : nodes) {
            if (node.id == path[i]) node1 = &node;
            if (node.id == path[i+1]) node2 = &node;
        }
        
        if (node1 && node2) {
            double dx = node2->x - node1->x;
            double dy = node2->y - node1->y;
            totalDist += std::sqrt(dx*dx + dy*dy);
        }
    }
    
    return totalDist;
}

double GlobalPathPlanner::calculateEstimatedTime(const std::vector<int>& path, double distance) {
    if (distance < 1e-6) {
        return 0.0;
    }

    const double speedMmPerSec =
        PathPlanningConstants::resolvePlannerSpeedMmPerSec(mapInfo_.getGlobalMaxSpeed());
    const double turnPenaltyMm = PathPlanningConstants::calculatePathTurnPenaltyByNodeIds(
        path,
        mapInfo_,
        PathPlanningConstants::resolveTurnPenaltyMm());
    return (distance + turnPenaltyMm) / speedMmPerSec;
}

int GlobalPathPlanner::countTurns(const std::vector<int>& path) {
    return PathPlanningConstants::countTurnsByNodeIds(path, mapInfo_);
}

double GlobalPathPlanner::euclideanDistance(int node1, int node2) {
    const auto& nodes = mapInfo_.getNodes();
    
    const Node* n1 = nullptr;
    const Node* n2 = nullptr;
    
    for (const auto& node : nodes) {
        if (node.id == node1) n1 = &node;
        if (node.id == node2) n2 = &node;
    }
    
    if (!n1 || !n2) {
        return std::numeric_limits<double>::infinity();
    }
    
    double dx = n2->x - n1->x;
    double dy = n2->y - n1->y;
    
    return std::sqrt(dx*dx + dy*dy);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 查询与统计
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void GlobalPathPlanner::resetStatistics() {
    stats_ = Statistics{};
}

bool GlobalPathPlanner::isNodeInStaticTable(int nodeId) const {
    // 检查节点是否是关键节点（任务点或充电站）
    return std::find(allKeyNodes_.begin(), allKeyNodes_.end(), nodeId) != allKeyNodes_.end();
}

std::vector<int> GlobalPathPlanner::getAllKeyNodes() const {
    return allKeyNodes_;
}

void GlobalPathPlanner::clearCache() {
    nearestKeyNodeCache_.clear();
    shortPathCache_.clear();
    logInfo("缓存已清空");
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 日志函数
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void GlobalPathPlanner::logInfo(const std::string& msg) {
    if (config_.verboseLogging) {
        std::cout << "[INFO] " << msg << std::endl;
    }
}

void GlobalPathPlanner::logWarn(const std::string& msg) {
    if (config_.verboseLogging) {
        std::cout << "[WARN] " << msg << std::endl;
    }
}

void GlobalPathPlanner::logError(const std::string& msg) {
    std::cerr << "[ERROR] " << msg << std::endl;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// ✅ 新增：带优先级信息的路径规划
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

GlobalPathPlanner::PathResult GlobalPathPlanner::planGlobalPathWithPriority(
    int startNode, 
    int endNode,
    int agvId,
    int taskId,
    int taskPriority,
    int batteryLevel,
    int agvType,
    double waitingTime,
    double startTime,
    const StaticPathTable::DynamicContext& context
) {
    // 1. 调用原有路径规划
    PathResult result = planGlobalPath(startNode, endNode, context);
    
    if (!result.reachable) {
        return result;
    }
    
    // 2. 填充AGV和任务信息
    result.agvId = agvId;
    result.taskId = taskId;
    result.startTime = startTime;
    
    // 3. 填充优先级因子
    result.priorityFactors.taskUrgency = static_cast<double>(taskPriority);  // 直接使用任务优先级
    result.priorityFactors.waitingTime = waitingTime;
    result.priorityFactors.batteryLevel = batteryLevel / 100.0;  // 转换为0-1
    result.priorityFactors.agvType = agvType;
    result.priorityFactors.pathLength = result.distance / 1000.0;  // mm转m
    
    // 4. 计算综合优先级
    result.priority = calculatePriority(result.priorityFactors);
    
    return result;
}

// ✅ 优先级计算算法（参考海康RCS-2000）
double GlobalPathPlanner::calculatePriority(const PathResult::PriorityFactors& factors) {
    double priority = 0.0;
    
    // 因素1: 任务紧急度/优先级 (40%)
    // 假设任务优先级范围 0-10，归一化到 0-40分
    double urgencyScore = std::min(factors.taskUrgency / 10.0, 1.0) * 40.0;
    priority += urgencyScore;
    
    // 因素2: 路径长度 (20%)
    // 短路径优先，假设最长路径1000米
    double pathScore = 1.0 - std::min(factors.pathLength / 1000.0, 1.0);
    priority += pathScore * 20.0;
    
    // 因素3: 等待时间 (20%)
    // 等待久的优先，假设最长等待300秒（5分钟）
    double waitScore = std::min(factors.waitingTime / 300.0, 1.0);
    priority += waitScore * 20.0;
    
    // 因素4: 电量 (10%)
    // 低电量优先（去充电），电量越低分数越高
    double batteryScore = 1.0 - factors.batteryLevel;
    priority += batteryScore * 10.0;
    
    // 因素5: AGV类型 (10%)
    // 优先级AGV得满分，普通AGV得0分
    double typeScore = (factors.agvType > 0) ? 1.0 : 0.0;
    priority += typeScore * 10.0;
    
    return priority;  // 范围: 0-100
}
