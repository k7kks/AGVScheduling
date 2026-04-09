/**
 * @file RobustPathPlanner.cpp
 * @brief 鲁棒路径规划器实现
 */

#include "RobustPathPlanner.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <chrono>
#include <queue>

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 构造与初始化
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

RobustPathPlanner::RobustPathPlanner(const MapInfo& mapInfo, const Config& config)
    : mapInfo_(mapInfo)
    , config_(config)
    , initialized_(false)
{
    // 预创建增强规划器（避免重复加载静态表）
    GlobalPathPlanner::Config enhancedConfig;
    enhancedConfig.staticTablePath = config_.staticTablePath;
    enhancedConfig.verboseLogging = false;  // 关闭详细日志
    enhancedConfig.nearestKeyCandidates = 100;
    
    enhancedPlanner_ = std::make_unique<GlobalPathPlanner>(mapInfo_, enhancedConfig);
}

bool RobustPathPlanner::initialize() {
    logInfo("初始化鲁棒路径规划器...");
    
    // 创建标准规划器（candidates=50）
    GlobalPathPlanner::Config plannerConfig;
    plannerConfig.staticTablePath = config_.staticTablePath;
    plannerConfig.verboseLogging = false;  // 关闭详细日志
    plannerConfig.nearestKeyCandidates = 50;
    
    planner_ = std::make_unique<GlobalPathPlanner>(mapInfo_, plannerConfig);
    
    if (!planner_->initialize()) {
        logError("标准规划器初始化失败");
        return false;
    }
    
    logInfo("标准规划器初始化完成");
    
    // 初始化增强规划器（candidates=100）
    if (!enhancedPlanner_->initialize()) {
        logError("增强规划器初始化失败");
        return false;
    }
    
    logInfo("增强规划器初始化完成");
    
    // 提取所有关键节点
    allKeyNodes_ = planner_->getAllKeyNodes();
    logInfo("关键节点数量: " + std::to_string(allKeyNodes_.size()));
    
    initialized_ = true;
    logInfo("鲁棒路径规划器初始化完成");
    
    return true;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 核心接口实现
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

RobustPathPlanner::PathResult RobustPathPlanner::planRobustPath(
    int startNode, 
    int endNode
) {
    if (!initialized_) {
        logError("规划器未初始化！");
        return PathResult{};
    }
    
    auto startTime = std::chrono::high_resolution_clock::now();
    stats_.totalQueries++;
    
    logInfo("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    logInfo("鲁棒路径规划: " + std::to_string(startNode) + " -> " + std::to_string(endNode));
    logInfo("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 五级降级策略
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 策略1: 标准规划
    logInfo("尝试策略1: 标准规划（candidates=50）");
    auto result1 = strategy1_StandardPlanning(startNode, endNode);
    if (result1.has_value()) {
        stats_.strategy1Success++;
        auto endTime = std::chrono::high_resolution_clock::now();
        result1->planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        logInfo("✓ 策略1成功！耗时: " + std::to_string(result1->planningTime) + " ms");
        return *result1;
    }
    logWarn("✗ 策略1失败");
    
    // 策略2: 增强搜索
    logInfo("尝试策略2: 增强搜索（candidates=100）");
    auto result2 = strategy2_EnhancedSearch(startNode, endNode);
    if (result2.has_value()) {
        stats_.strategy2Success++;
        auto endTime = std::chrono::high_resolution_clock::now();
        result2->planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        logInfo("✓ 策略2成功！耗时: " + std::to_string(result2->planningTime) + " ms");
        return *result2;
    }
    logWarn("✗ 策略2失败");
    
    // 策略3: 单中转点
    logInfo("尝试策略3: 单中转点规划");
    auto result3 = strategy3_SingleIntermediate(startNode, endNode);
    if (result3.has_value()) {
        stats_.strategy3Success++;
        auto endTime = std::chrono::high_resolution_clock::now();
        result3->planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        logInfo("✓ 策略3成功！耗时: " + std::to_string(result3->planningTime) + " ms");
        return *result3;
    }
    logWarn("✗ 策略3失败");
    
    // 策略4: 多中转点
    logInfo("尝试策略4: 多中转点规划");
    auto result4 = strategy4_MultipleIntermediate(startNode, endNode);
    if (result4.has_value()) {
        stats_.strategy4Success++;
        auto endTime = std::chrono::high_resolution_clock::now();
        result4->planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        logInfo("✓ 策略4成功！耗时: " + std::to_string(result4->planningTime) + " ms");
        return *result4;
    }
    logWarn("✗ 策略4失败");
    
    // 策略5: 暴力搜索
    logInfo("尝试策略5: 暴力搜索");
    auto result5 = strategy5_BruteForce(startNode, endNode);
    if (result5.has_value()) {
        stats_.strategy5Success++;
        auto endTime = std::chrono::high_resolution_clock::now();
        result5->planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        logInfo("✓ 策略5成功！耗时: " + std::to_string(result5->planningTime) + " ms");
        return *result5;
    }
    
    // 所有策略都失败
    logError("❌ 所有策略都失败，路径物理不可达！");
    stats_.totalFailures++;
    
    PathResult failResult;
    failResult.reachable = false;
    failResult.strategyLevel = 0;
    failResult.strategyName = "完全失败";
    
    auto endTime = std::chrono::high_resolution_clock::now();
    failResult.planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    
    return failResult;
}

std::vector<RobustPathPlanner::PathResult> RobustPathPlanner::planBatchPaths(
    const std::vector<std::pair<int, int>>& requests
) {
    std::vector<PathResult> results;
    results.reserve(requests.size());
    
    for (const auto& [start, end] : requests) {
        results.push_back(planRobustPath(start, end));
    }
    
    return results;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 五级降级策略实现
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

std::optional<RobustPathPlanner::PathResult> 
RobustPathPlanner::strategy1_StandardPlanning(int start, int end) {
    // 使用默认配置（candidates=50）
    auto result = planner_->planGlobalPath(start, end);
    
    if (result.reachable) {
        PathResult robustResult;
        robustResult.path = result.path;
        robustResult.distance = result.distance;
        robustResult.estimatedTime = result.estimatedTime;
        robustResult.turnCount = result.turnCount;
        robustResult.reachable = true;
        robustResult.strategyLevel = 1;
        robustResult.strategyName = "标准规划";
        robustResult.detourRatio = 1.0;
        
        return robustResult;
    }
    
    return std::nullopt;
}

std::optional<RobustPathPlanner::PathResult> 
RobustPathPlanner::strategy2_EnhancedSearch(int start, int end) {
    // 策略2: 使用预初始化的增强规划器（避免重复加载静态表）
    auto result = enhancedPlanner_->planGlobalPath(start, end);
    
    if (result.reachable) {
        PathResult robustResult;
        robustResult.path = result.path;
        robustResult.distance = result.distance;
        robustResult.estimatedTime = result.estimatedTime;
        robustResult.turnCount = result.turnCount;
        robustResult.reachable = true;
        robustResult.strategyLevel = 2;
        robustResult.strategyName = "增强搜索";
        robustResult.detourRatio = 1.0;
        
        return robustResult;
    }
    
    return std::nullopt;
}

std::optional<RobustPathPlanner::PathResult> 
RobustPathPlanner::strategy3_SingleIntermediate(int start, int end) {
    // 找一个最佳中转节点
    auto intermediates = findBestIntermediateNodes(start, end, 1);
    
    if (intermediates.empty()) {
        return std::nullopt;
    }
    
    int intermediate = intermediates[0];
    
    // 分两段规划
    auto seg1 = planner_->planGlobalPath(start, intermediate);
    auto seg2 = planner_->planGlobalPath(intermediate, end);
    
    if (seg1.reachable && seg2.reachable) {
        PathResult result = concatenateSegments({seg1, seg2}, 3, "单中转点");
        result.intermediateNodes = {intermediate};
        
        // 计算绕行比例
        double directDist = euclideanDistance(start, end);
        result.detourRatio = result.distance / directDist;
        
        if (result.detourRatio <= config_.maxDetourRatio) {
            return result;
        }
    }
    
    return std::nullopt;
}

std::optional<RobustPathPlanner::PathResult> 
RobustPathPlanner::strategy4_MultipleIntermediate(int start, int end) {
    // 找多个中转节点
    auto intermediates = findBestIntermediateNodes(start, end, config_.maxIntermediateNodes);
    
    if (intermediates.empty()) {
        return std::nullopt;
    }
    
    // 尝试不同数量的中转节点
    for (size_t count = 2; count <= intermediates.size(); ++count) {
        std::vector<int> nodes = {start};
        for (size_t i = 0; i < count; ++i) {
            nodes.push_back(intermediates[i]);
        }
        nodes.push_back(end);
        
        // 分段规划
        std::vector<GlobalPathPlanner::PathResult> segments;
        bool allReachable = true;
        
        for (size_t i = 0; i + 1 < nodes.size(); ++i) {
            auto seg = planner_->planGlobalPath(nodes[i], nodes[i+1]);
            if (!seg.reachable) {
                allReachable = false;
                break;
            }
            segments.push_back(seg);
        }
        
        if (allReachable) {
            PathResult result = concatenateSegments(segments, 4, "多中转点");
            result.intermediateNodes = std::vector<int>(intermediates.begin(), intermediates.begin() + count);
            
            // 计算绕行比例
            double directDist = euclideanDistance(start, end);
            result.detourRatio = result.distance / directDist;
            
            if (result.detourRatio <= config_.maxDetourRatio) {
                return result;
            }
        }
    }
    
    return std::nullopt;
}

std::optional<RobustPathPlanner::PathResult> 
RobustPathPlanner::strategy5_BruteForce(int start, int end) {
    logInfo("暴力搜索：遍历所有关键节点作为中转点");
    
    // 限制搜索范围，避免超时
    int maxAttempts = std::min(50, static_cast<int>(allKeyNodes_.size()));
    
    // 按欧几里得距离排序关键节点
    std::vector<std::pair<double, int>> sortedNodes;
    for (int node : allKeyNodes_) {
        if (node == start || node == end) continue;
        
        double dist1 = euclideanDistance(start, node);
        double dist2 = euclideanDistance(node, end);
        double totalDist = dist1 + dist2;
        
        sortedNodes.push_back({totalDist, node});
    }
    
    std::sort(sortedNodes.begin(), sortedNodes.end());
    
    // 尝试前maxAttempts个节点
    for (int i = 0; i < maxAttempts && i < static_cast<int>(sortedNodes.size()); ++i) {
        int intermediate = sortedNodes[i].second;
        
        auto seg1 = planner_->planGlobalPath(start, intermediate);
        auto seg2 = planner_->planGlobalPath(intermediate, end);
        
        if (seg1.reachable && seg2.reachable) {
            PathResult result = concatenateSegments({seg1, seg2}, 5, "暴力搜索");
            result.intermediateNodes = {intermediate};
            
            double directDist = euclideanDistance(start, end);
            result.detourRatio = result.distance / directDist;
            
            // 暴力搜索允许更大的绕行比例
            if (result.detourRatio <= config_.maxDetourRatio * 2.0) {
                logInfo("找到可行路径，中转点: " + std::to_string(intermediate));
                return result;
            }
        }
    }
    
    return std::nullopt;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 辅助函数实现
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

std::vector<int> RobustPathPlanner::findBestIntermediateNodes(
    int start, int end, int maxCount
) {
    // 评估所有关键节点作为中转点的质量
    std::vector<std::pair<double, int>> scores;
    
    for (int node : allKeyNodes_) {
        if (node == start || node == end) continue;
        
        double score = evaluateIntermediateNode(start, node, end);
        scores.push_back({score, node});
    }
    
    // 按分数排序（分数越小越好）
    std::sort(scores.begin(), scores.end());
    
    // 返回前maxCount个
    std::vector<int> result;
    for (int i = 0; i < maxCount && i < static_cast<int>(scores.size()); ++i) {
        result.push_back(scores[i].second);
    }
    
    return result;
}

double RobustPathPlanner::evaluateIntermediateNode(
    int start, int intermediate, int end
) {
    // 评估标准：
    // 1. 总距离（欧几里得）
    // 2. 绕行比例
    // 3. 角度偏差
    
    double dist1 = euclideanDistance(start, intermediate);
    double dist2 = euclideanDistance(intermediate, end);
    double directDist = euclideanDistance(start, end);
    
    double totalDist = dist1 + dist2;
    double detourRatio = totalDist / directDist;
    
    // 分数 = 绕行比例（越小越好）
    return detourRatio;
}

RobustPathPlanner::PathResult RobustPathPlanner::concatenateSegments(
    const std::vector<GlobalPathPlanner::PathResult>& segments,
    int strategyLevel,
    const std::string& strategyName
) {
    PathResult result;
    result.strategyLevel = strategyLevel;
    result.strategyName = strategyName;
    result.reachable = true;
    
    // 拼接路径
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        
        if (result.path.empty()) {
            result.path = seg.path;
        } else {
            // 跳过重复的起点
            for (size_t j = 1; j < seg.path.size(); ++j) {
                result.path.push_back(seg.path[j]);
            }
        }
        
        result.distance += seg.distance;
        result.estimatedTime += seg.estimatedTime;
        result.turnCount += seg.turnCount;
    }
    
    return result;
}

double RobustPathPlanner::euclideanDistance(int node1, int node2) {
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
// 统计与日志
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void RobustPathPlanner::Statistics::update() {
    if (totalQueries > 0) {
        avgPlanningTime = avgPlanningTime / totalQueries;
    }
}

void RobustPathPlanner::Statistics::print() const {
    std::cout << "\n【鲁棒路径规划统计】\n";
    std::cout << "  总查询次数: " << totalQueries << "\n";
    std::cout << "  策略1成功: " << strategy1Success << " (" 
              << (100.0 * strategy1Success / totalQueries) << "%)\n";
    std::cout << "  策略2成功: " << strategy2Success << " (" 
              << (100.0 * strategy2Success / totalQueries) << "%)\n";
    std::cout << "  策略3成功: " << strategy3Success << " (" 
              << (100.0 * strategy3Success / totalQueries) << "%)\n";
    std::cout << "  策略4成功: " << strategy4Success << " (" 
              << (100.0 * strategy4Success / totalQueries) << "%)\n";
    std::cout << "  策略5成功: " << strategy5Success << " (" 
              << (100.0 * strategy5Success / totalQueries) << "%)\n";
    std::cout << "  完全失败: " << totalFailures << " (" 
              << (100.0 * totalFailures / totalQueries) << "%)\n";
    std::cout << "  总成功率: " 
              << (100.0 * (totalQueries - totalFailures) / totalQueries) << "%\n";
}

void RobustPathPlanner::resetStatistics() {
    stats_ = Statistics{};
}

void RobustPathPlanner::logInfo(const std::string& msg) {
    if (config_.verboseLogging) {
        std::cout << "[INFO] " << msg << std::endl;
    }
}

void RobustPathPlanner::logWarn(const std::string& msg) {
    if (config_.verboseLogging) {
        std::cout << "[WARN] " << msg << std::endl;
    }
}

void RobustPathPlanner::logError(const std::string& msg) {
    std::cerr << "[ERROR] " << msg << std::endl;
}

