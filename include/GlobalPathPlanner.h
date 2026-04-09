#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <unordered_map>
#include <chrono>
#include "StaticPathTable.h"
#include "data/MapInfo.h"
#include "AStarPathFinder.h"

/**
 * @brief 全局路径规划器
 * 
 * 对标海康RCS-2000的工业级实现
 * 核心策略：静态表优先 + 动态A*补全
 * 
 * 功能特性：
 * - 静态路径表查询（98%场景，<1ms）
 * - 混合路径规划（2%场景，5-20ms）
 * - 最近关键节点缓存
 * - 短路径缓存（LRU）
 * - 路径拼接
 * - 性能统计
 */
class GlobalPathPlanner {
public:
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 数据结构
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 路径来源
    enum class PathSource {
        STATIC_TABLE,      // 静态表查询
        MIXED_PLANNING,    // 混合规划（静态+动态）
        PURE_ASTAR,        // 纯A*规划
        UNREACHABLE        // 不可达
    };
    
    // 路径结果
    struct PathResult {
        std::vector<int> path;           // 路径节点序列（关键节点）
        std::vector<int> fullPath;       // 完整路径（所有节点，可选）
        double distance;                 // 路径距离（mm）
        double estimatedTime;            // 预计时间（秒）
        int turnCount;                   // 转弯次数
        bool reachable;                  // 是否可达
        PathSource source;               // 路径来源
        double planningTime;             // 规划耗时（ms）
        
        // ✅ 新增：优先级相关字段（用于动态路径规划）
        int agvId = -1;                  // AGV ID
        int taskId = -1;                 // 任务ID（或任务序号）
        double priority = 0.0;           // 综合优先级（0-100）
        double startTime = 0.0;          // 计划开始时间（秒）
        
        // ✅ 优先级计算因子（用于动态调度/扩展策略）
        struct PriorityFactors {
            double taskUrgency = 0.0;     // 任务紧急度/优先级（0-10）
            double waitingTime = 0.0;     // 等待时间（秒）
            double batteryLevel = 1.0;    // 电量（0-1）
            int agvType = 0;              // AGV类型（0=普通，1=优先）
            double pathLength = 0.0;      // 路径长度（米）
        } priorityFactors;
        
        PathResult() 
            : distance(0), estimatedTime(0), turnCount(0), 
              reachable(true), source(PathSource::STATIC_TABLE), 
              planningTime(0) {}
    };
    
    // 配置参数
    struct Config {
        std::string staticTablePath;           // 静态表文件路径
        bool enableMixedPlanning = true;       // 启用混合规划
        bool enableNearestNodeCache = true;    // 启用最近节点缓存
        bool enableShortPathCache = true;      // 启用短路径缓存
        int shortPathCacheSize = 1000;         // 短路径缓存大小
        double shortDistanceThreshold = 10000.0; // 短距离阈值（mm）
        int nearestKeyCandidates = 50;         // 最近关键节点候选数（增加到50）
        bool verboseLogging = false;           // 详细日志
        bool forcePureAStar = false;           // 强制纯A*模式（跳过静态表）
    };
    
    // 性能统计
    struct Statistics {
        int totalQueries = 0;                  // 总查询次数
        int staticTableHits = 0;               // 静态表命中次数
        int mixedPlanningCount = 0;            // 混合规划次数
        int pureAStarCount = 0;                // 纯A*次数
        int unreachableCount = 0;              // 不可达次数
        double totalPlanningTime = 0.0;        // 总规划时间（ms）
        double avgPlanningTime = 0.0;          // 平均规划时间（ms）
        double staticTableHitRate = 0.0;       // 静态表命中率（%）
        
        void update() {
            if (totalQueries > 0) {
                avgPlanningTime = totalPlanningTime / totalQueries;
                staticTableHitRate = 100.0 * staticTableHits / totalQueries;
            }
        }
    };
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 构造与初始化
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    GlobalPathPlanner(const MapInfo& mapInfo, const Config& config);
    ~GlobalPathPlanner() = default;
    
    // 初始化（加载静态表）
    bool initialize();
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 核心接口
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 规划全局路径（主接口）
     * @param startNode 起点节点ID
     * @param endNode 终点节点ID
     * @param context 动态上下文（拥堵信息等）
     * @return 路径规划结果
     */
    PathResult planGlobalPath(
        int startNode, 
        int endNode,
        const StaticPathTable::DynamicContext& context = StaticPathTable::DynamicContext()
    );
    
    /**
     * @brief ✅ 规划全局路径（带优先级信息，用于动态路径规划）
     * @param startNode 起点节点ID
     * @param endNode 终点节点ID
     * @param agvId AGV ID
     * @param taskId 任务ID
     * @param taskPriority 任务优先级（来自Task对象）
     * @param batteryLevel AGV电量（0-100）
     * @param agvType AGV类型（0=普通，1=优先，来自Amr对象）
     * @param waitingTime 等待时间（秒）
     * @param startTime 计划开始时间（秒）
     * @param context 动态上下文
     * @return 带优先级信息的路径规划结果
     */
    PathResult planGlobalPathWithPriority(
        int startNode, 
        int endNode,
        int agvId,
        int taskId,
        int taskPriority,                 // 任务优先级（来自Task::getPriority()）
        int batteryLevel = 100,           // 电量（来自Amr::getBatteryLevel()）
        int agvType = 0,                  // AGV类型（来自Amr::getDeviceType()）
        double waitingTime = 0.0,         // 等待时间（秒）
        double startTime = 0.0,           // 计划开始时间（秒）
        const StaticPathTable::DynamicContext& context = StaticPathTable::DynamicContext()
    );
    
    /**
     * @brief 批量规划路径（为多个AGV规划）
     * @param requests 路径请求列表 [(startNode, endNode), ...]
     * @return 路径结果列表
     */
    std::vector<PathResult> planBatchPaths(
        const std::vector<std::pair<int, int>>& requests,
        const StaticPathTable::DynamicContext& context = StaticPathTable::DynamicContext()
    );

    /**
     * @brief 规划全局路径（仅使用A*，用于调试）
     * @param startNode 起点节点ID
     * @param endNode 终点节点ID
     * @return 路径规划结果
     */
    PathResult planGlobalPathPureAStar(int startNode, int endNode);
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 查询与统计
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 获取性能统计
    const Statistics& getStatistics() const { return stats_; }
    
    // 重置统计
    void resetStatistics();

    // 使用外部预加载静态表（避免重复加载）；传入 nullptr 恢复内部表
    void setStaticPathTable(const StaticPathTable* table);
    
    // 检查节点是否在静态表中
    bool isNodeInStaticTable(int nodeId) const;
    
    // 获取所有关键节点
    std::vector<int> getAllKeyNodes() const;
    
    // 清空缓存
    void clearCache();

    // 暴露地图引用，方便统一解析 AMR 起点等操作
    const MapInfo& getMapInfo() const { return mapInfo_; }

private:
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 内部实现
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 混合规划（静态表未命中时）
    PathResult mixedPlanning(
        int startNode, 
        int endNode,
        const StaticPathTable::DynamicContext& context
    );
    
    // 情况1: 起点和终点都不在表中
    PathResult case1_BothNotInTable(
        int start, int end, 
        const StaticPathTable::DynamicContext& context
    );
    
    // 情况2: 只有起点不在表中（最常见）
    PathResult case2_StartNotInTable(
        int start, int end,
        const StaticPathTable::DynamicContext& context
    );
    
    // 情况3: 只有终点不在表中
    PathResult case3_EndNotInTable(
        int start, int end,
        const StaticPathTable::DynamicContext& context
    );
    
    // 情况4: 都在表中但查询失败（不可达）
    PathResult case4_BothInTableButFailed(
        int start, int end,
        const StaticPathTable::DynamicContext& context
    );
    
    // 找最近的关键节点
    int findNearestKeyNode(int nodeId);
    
    // 找最近的K个关键节点（欧几里得距离）
    std::vector<int> findNearestKKeyNodesByEuclidean(int nodeId, int k);
    
    // A*搜索（短路径）
    std::vector<int> aStarSearch(int start, int end, const StaticPathTable::DynamicContext& context);
    
    // 路径拼接
    PathResult concatenatePaths(
        const std::vector<std::vector<int>>& segments,
        PathSource source
    );
    
    // 计算路径距离
    double calculatePathDistance(const std::vector<int>& path);
    
    // 计算预计时间
    double calculateEstimatedTime(const std::vector<int>& path, double distance);
    
    // 计算转弯次数
    int countTurns(const std::vector<int>& path);
    
    // 欧几里得距离
    double euclideanDistance(int node1, int node2);
    
    // 日志
    void logInfo(const std::string& msg);
    void logWarn(const std::string& msg);
    void logError(const std::string& msg);

    // 当前可用的静态表指针
    const StaticPathTable* staticTablePtr() const;
    
    // ✅ 优先级计算（参考海康RCS-2000算法）
    double calculatePriority(const PathResult::PriorityFactors& factors);
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 成员变量
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    const MapInfo& mapInfo_;                    // 地图信息
    Config config_;                             // 配置参数
    StaticPathTable staticTable_;               // 静态路径表（内部）
    const StaticPathTable* externalStaticTable_ {nullptr}; // 可选：外部预加载静态表
    std::unique_ptr<AStarPathFinder> aStarPlanner_; // A*规划器
    
    // 缓存
    std::unordered_map<int, int> nearestKeyNodeCache_;  // 最近关键节点缓存
    std::vector<int> allKeyNodes_;                      // 所有关键节点列表
    
    // 短路径缓存（LRU）
    struct PathCacheKey {
        int start;
        int end;
        bool operator==(const PathCacheKey& other) const {
            return start == other.start && end == other.end;
        }
    };
    struct PathCacheKeyHash {
        std::size_t operator()(const PathCacheKey& k) const {
            return std::hash<int>()(k.start) ^ (std::hash<int>()(k.end) << 1);
        }
    };
    std::unordered_map<PathCacheKey, std::vector<int>, PathCacheKeyHash> shortPathCache_;
    
    // 统计
    mutable Statistics stats_;
    
    // 初始化标志
    bool initialized_;
};

#endif // GLOBAL_PATH_PLANNER_H
