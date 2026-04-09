/**
 * @file RobustPathPlanner.h
 * @brief 鲁棒路径规划器（工业级容错）
 * 
 * 对标海康RCS-2000的容错机制：
 * 1. 多策略路径搜索
 * 2. 自动降级机制
 * 3. 备选路径生成
 * 4. 智能路径修复
 * 5. 100%成功保证
 */

#ifndef ROBUST_PATH_PLANNER_H
#define ROBUST_PATH_PLANNER_H

#include <vector>
#include <string>
#include <optional>
#include <memory>
#include "GlobalPathPlanner.h"
#include "data/MapInfo.h"

/**
 * @brief 鲁棒路径规划器
 * 
 * 核心特性：
 * - 多级降级策略
 * - 备选路径生成
 * - 智能中转点选择
 * - 路径修复机制
 * - 100%成功保证（除非物理不可达）
 */
class RobustPathPlanner {
public:
    // 配置参数
    struct Config {
        std::string staticTablePath;
        int maxRetryAttempts = 5;           // 最大重试次数
        int maxIntermediateNodes = 3;       // 最大中转节点数
        double maxDetourRatio = 3.0;        // 最大绕行比例
        bool enableMultiPath = true;        // 启用多路径搜索
        bool enablePathRepair = true;       // 启用路径修复
        bool verboseLogging = false;
    };
    
    // 路径结果（增强版）
    struct PathResult {
        std::vector<int> path;              // 路径节点序列
        double distance;                    // 路径距离（mm）
        double estimatedTime;               // 预计时间（秒）
        int turnCount;                      // 转弯次数
        bool reachable;                     // 是否可达
        
        // 增强信息
        int strategyLevel;                  // 使用的策略级别（1-5）
        std::string strategyName;           // 策略名称
        std::vector<int> intermediateNodes; // 中转节点
        double detourRatio;                 // 绕行比例
        double planningTime;                // 规划耗时（ms）
        
        PathResult() 
            : distance(0), estimatedTime(0), turnCount(0), 
              reachable(false), strategyLevel(0), detourRatio(1.0), 
              planningTime(0) {}
    };
    
    RobustPathPlanner(const MapInfo& mapInfo, const Config& config);
    ~RobustPathPlanner() = default;
    
    // 初始化
    bool initialize();
    
    /**
     * @brief 鲁棒路径规划（主接口）
     * 
     * 保证：
     * - 如果物理可达，必定返回路径
     * - 如果物理不可达，返回最佳近似路径
     * 
     * @param startNode 起点节点ID
     * @param endNode 终点节点ID
     * @return 路径规划结果（reachable=true表示成功）
     */
    PathResult planRobustPath(int startNode, int endNode);
    
    /**
     * @brief 批量规划（带容错）
     */
    std::vector<PathResult> planBatchPaths(
        const std::vector<std::pair<int, int>>& requests
    );
    
    // 统计信息
    struct Statistics {
        int totalQueries = 0;
        int strategy1Success = 0;  // 直接规划
        int strategy2Success = 0;  // 增加候选数
        int strategy3Success = 0;  // 单中转点
        int strategy4Success = 0;  // 多中转点
        int strategy5Success = 0;  // 暴力搜索
        int totalFailures = 0;     // 完全失败
        double avgPlanningTime = 0.0;
        
        void update();
        void print() const;
    };
    
    const Statistics& getStatistics() const { return stats_; }
    void resetStatistics();

private:
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 五级降级策略
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * 策略1: 标准规划（nearestKeyCandidates=50）
     */
    std::optional<PathResult> strategy1_StandardPlanning(int start, int end);
    
    /**
     * 策略2: 增强搜索（nearestKeyCandidates=100）
     */
    std::optional<PathResult> strategy2_EnhancedSearch(int start, int end);
    
    /**
     * 策略3: 单中转点规划
     * 找一个中间关键节点，分两段规划
     */
    std::optional<PathResult> strategy3_SingleIntermediate(int start, int end);
    
    /**
     * 策略4: 多中转点规划
     * 找多个中间节点，分多段规划
     */
    std::optional<PathResult> strategy4_MultipleIntermediate(int start, int end);
    
    /**
     * 策略5: 暴力搜索
     * 遍历所有可能的中转点组合
     */
    std::optional<PathResult> strategy5_BruteForce(int start, int end);
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 辅助函数
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 找最佳中转节点
    std::vector<int> findBestIntermediateNodes(
        int start, int end, int maxCount
    );
    
    // 评估中转节点质量
    double evaluateIntermediateNode(int start, int intermediate, int end);
    
    // 拼接多段路径
    PathResult concatenateSegments(
        const std::vector<GlobalPathPlanner::PathResult>& segments,
        int strategyLevel,
        const std::string& strategyName
    );
    
    // 欧几里得距离
    double euclideanDistance(int node1, int node2);
    
    // 日志
    void logInfo(const std::string& msg);
    void logWarn(const std::string& msg);
    void logError(const std::string& msg);
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 成员变量
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    const MapInfo& mapInfo_;
    Config config_;
    std::unique_ptr<GlobalPathPlanner> planner_;          // 标准规划器（candidates=50）
    std::unique_ptr<GlobalPathPlanner> enhancedPlanner_;  // 增强规划器（candidates=100）
    std::vector<int> allKeyNodes_;
    mutable Statistics stats_;
    bool initialized_;
};

#endif // ROBUST_PATH_PLANNER_H

