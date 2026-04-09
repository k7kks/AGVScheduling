#ifndef PRIORITY_PATH_PLANNER_H
#define PRIORITY_PATH_PLANNER_H

#include <vector>
#include <string>
#include <memory>
#include "GlobalPathPlanner.h"
#include "data/MapInfo.h"
#include "data/Task.h"
#include "data/Amr.h"

/**
 * @brief 优先级路径规划器
 * 
 * 功能：将静态路径规划结果转化为带优先级属性的路径
 * 考虑因素：
 *   1. 任务本身优先级 (Task.priority)
 *   2. AGV状态优先级 (电量、类型)
 *   3. 分阶段优先级 (空载/负载)
 *   4. 执行状态优先级 (未开始/执行中/快完成)
 * 
 * 参考：海康RCS-2000、极智嘉调度系统
 */
class PriorityPathPlanner {
public:
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 数据结构
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 执行状态枚举
     */
    enum class ExecutionState {
        NOT_STARTED,           // 未开始
        IN_PROGRESS_EMPTY,     // 执行中-空载
        IN_PROGRESS_LOADED,    // 执行中-负载
        NEAR_COMPLETION,       // 快完成(>90%)
        COMPLETED              // 已完成
    };
    
    /**
     * @brief 路径段类型
     */
    enum class SegmentType {
        TO_PICKUP,      // 去取货（空载）
        TO_DELIVERY,    // 去送货（负载）
        TO_CHARGING,    // 去充电（空载/负载）
        TO_PARKING      // 去停车（空载）
    };
    
    /**
     * @brief 优先级路径结果（扩展自GlobalPathPlanner::PathResult）
     */
    struct PriorityPathResult {
        // ========== 基础路径信息 ==========
        std::vector<int> path;           // 路径节点序列
        std::vector<int> fullPath;       // 完整路径
        double distance;                 // 路径距离（mm）
        double estimatedTime;            // 预计时间（秒）
        int turnCount;                   // 转弯次数
        bool reachable;                  // 是否可达
        GlobalPathPlanner::PathSource source;  // 路径来源
        double planningTime;             // 规划耗时（ms）
        
        // ========== 任务和AGV信息 ==========
        int agvId;                       // AGV ID
        int taskId;                      // 任务ID
        std::string agvDeviceId;         // AGV设备编号
        std::string taskMessageId;       // 任务消息ID
        
        // ========== 优先级信息 ==========
        double basePriority;             // 基础优先级（固定，0-100）
        double effectivePriority;        // 有效优先级（动态，0-200+）
        
        // ========== 状态信息 ==========
        SegmentType segmentType;         // 路径段类型
        bool isLoaded;                   // 是否负载
        ExecutionState executionState;   // 执行状态
        double completionRatio;          // 完成度（0-1）
        double startTime;                // 计划开始时间（秒）
        double endTime;                  // 计划结束时间（秒）
        
        // ========== 优先级计算因子（详细） ==========
        struct DetailedPriorityFactors {
            // 任务因素
            int taskPriority;            // 任务优先级（0-10）
            double taskUrgency;          // 任务紧急度（0-10）
            bool isNearTimeout;          // 是否即将超时
            double timeoutMargin;        // 距离超时的余量（秒）
            
            // AGV因素
            int batteryLevel;            // 电量（0-100%）
            int agvType;                 // AGV类型（0=普通，1=优先）
            bool isLowBattery;           // 是否低电量（<20%）
            
            // 路径因素
            double pathLength;           // 路径长度（米）
            double waitingTime;          // 等待时间（秒）
            
            // 执行因素
            bool isLoaded;               // 是否负载
            double completionRatio;      // 完成度（0-1）
            ExecutionState execState;    // 执行状态
            
            // 加成分数（用于调试）
            double taskPriorityScore;    // 任务优先级得分（0-40）
            double pathLengthScore;      // 路径长度得分（0-20）
            double waitingTimeScore;     // 等待时间得分（0-20）
            double batteryScore;         // 电量得分（0-10）
            double agvTypeScore;         // AGV类型得分（0-10）
            double loadBonus;            // 负载加成（0-30）
            double executionBonus;       // 执行状态加成（0-100）
            double completionBonus;      // 完成度加成（0-30）
            double timeoutBonus;         // 超时加成（0-100）
        } detailedFactors;
        
        // 构造函数
        PriorityPathResult() 
            : distance(0), estimatedTime(0), turnCount(0), 
              reachable(true), planningTime(0),
              agvId(-1), taskId(-1),
              basePriority(0), effectivePriority(0),
              segmentType(SegmentType::TO_PICKUP),
              isLoaded(false), 
              executionState(ExecutionState::NOT_STARTED),
              completionRatio(0), startTime(0), endTime(0) {}
    };
    
    /**
     * @brief 多段路径任务（一个完整任务可能包含多段路径）
     */
    struct MultiSegmentTask {
        int taskId;                              // 任务ID
        int agvId;                               // AGV ID
        std::vector<PriorityPathResult> segments; // 路径段列表
        double totalDistance;                    // 总距离（米）
        double totalTime;                        // 总时间（秒）
        double startTime;                        // 开始时间（秒）
        double endTime;                          // 结束时间（秒）
        
        MultiSegmentTask() 
            : taskId(-1), agvId(-1), 
              totalDistance(0), totalTime(0),
              startTime(0), endTime(0) {}
    };
    
    /**
     * @brief 配置参数
     */
    struct Config {
        // 优先级权重配置（总和100%）
        double taskPriorityWeight = 0.40;        // 任务优先级权重（40%）
        double pathLengthWeight = 0.20;          // 路径长度权重（20%）
        double waitingTimeWeight = 0.20;         // 等待时间权重（20%）
        double batteryWeight = 0.10;             // 电量权重（10%）
        double agvTypeWeight = 0.10;             // AGV类型权重（10%）
        
        // 优先级加成配置
        double loadPriorityBonus = 30.0;         // 负载加成（+30分）
        double executionEmptyBonus = 30.0;       // 执行中-空载加成（+30分）
        double executionLoadedBonus = 60.0;      // 执行中-负载加成（+60分）
        double nearCompletionBonus = 30.0;       // 快完成加成（+30分）
        double timeoutBonus = 100.0;             // 即将超时加成（+100分）
        
        // 阈值配置
        double lowBatteryThreshold = 20.0;       // 低电量阈值（20%）
        double nearCompletionThreshold = 0.90;   // 快完成阈值（90%）
        double nearTimeoutThreshold = 60.0;      // 即将超时阈值（60秒）
        
        // 归一化参数
        double maxPathLength = 1000.0;           // 最长路径（米）
        double maxWaitingTime = 300.0;           // 最长等待时间（秒）
        double maxTaskPriority = 10.0;           // 最高任务优先级
        
        // 启用选项
        bool enableLoadBonus = true;             // 启用负载加成
        bool enableExecutionBonus = true;        // 启用执行状态加成
        bool enableCompletionBonus = true;       // 启用完成度加成
        bool enableTimeoutBonus = true;          // 启用超时加成
        bool enableFallbackStrategies = true;    // 启用备选策略
        bool verboseLogging = false;             // 详细日志
    };
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 构造与初始化
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 构造函数
     * @param globalPlanner 全局路径规划器
     * @param config 配置参数
     */
    PriorityPathPlanner(
        std::shared_ptr<GlobalPathPlanner> globalPlanner
    );
    
    PriorityPathPlanner(
        std::shared_ptr<GlobalPathPlanner> globalPlanner,
        const Config& config
    );
    
    ~PriorityPathPlanner() = default;
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 核心接口：单段路径规划
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 规划单段优先级路径
     * @param startNode 起点节点ID
     * @param endNode 终点节点ID
     * @param task 任务对象
     * @param amr AMR对象
     * @param segmentType 路径段类型
     * @param waitingTime 等待时间（秒）
     * @param startTime 计划开始时间（秒）
     * @return 优先级路径结果
     */
    PriorityPathResult planPriorityPath(
        int startNode,
        int endNode,
        const Task& task,
        const Amr& amr,
        SegmentType segmentType,
        double waitingTime = 0.0,
        double startTime = 0.0
    );
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 核心接口：多段路径规划
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 规划完整任务的多段路径（取货-送货）
     * @param task 任务对象（包含取货点、送货点）
     * @param amr AMR对象
     * @param startTime 计划开始时间（秒）
     * @return 多段路径任务
     */
    MultiSegmentTask planMultiSegmentTask(
        const Task& task,
        const Amr& amr,
        double startTime = 0.0
    );
    
    /**
     * @brief 批量规划多个任务的优先级路径
     * @param tasks 任务列表
     * @param amrs AMR列表
     * @param taskAssignments 任务分配结果 [(taskId, amrId), ...]
     * @param startTime 计划开始时间（秒）
     * @return 所有任务的路径段列表
     */
    std::vector<PriorityPathResult> planBatchTasks(
        const std::vector<Task>& tasks,
        const std::vector<Amr>& amrs,
        const std::vector<std::pair<int, int>>& taskAssignments,
        double startTime = 0.0
    );
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 优先级计算与更新
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 计算基础优先级（任务、AGV、路径因素）
     * @param factors 优先级因子
     * @return 基础优先级（0-100）
     */
    double calculateBasePriority(
        const PriorityPathResult::DetailedPriorityFactors& factors
    );
    
    /**
     * @brief 计算有效优先级（基础优先级 + 状态加成）
     * @param pathResult 路径结果
     * @return 有效优先级（0-200+）
     */
    double calculateEffectivePriority(const PriorityPathResult& pathResult);
    
    /**
     * @brief 更新路径的执行状态和优先级
     * @param pathResult 路径结果（会被修改）
     * @param currentNodeId 当前节点ID
     * @param currentTime 当前时间（秒）
     */
    void updateExecutionState(
        PriorityPathResult& pathResult,
        int currentNodeId,
        double currentTime
    );
    
    /**
     * @brief 批量更新所有路径的优先级
     * @param paths 路径列表（会被修改）
     * @param amrs AMR列表（当前状态）
     * @param currentTime 当前时间（秒）
     */
    void updateAllPriorities(
        std::vector<PriorityPathResult>& paths,
        const std::vector<Amr>& amrs,
        double currentTime
    );
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 查询与工具
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    /**
     * @brief 按优先级排序路径（降序）
     * @param paths 路径列表
     */
    void sortByPriority(std::vector<PriorityPathResult>& paths);
    
    /**
     * @brief 打印路径优先级详情
     * @param pathResult 路径结果
     */
    void printPriorityDetails(const PriorityPathResult& pathResult);
    
    /**
     * @brief 获取配置
     */
    const Config& getConfig() const { return config_; }
    
    /**
     * @brief 设置配置
     */
    void setConfig(const Config& config) { config_ = config; }
    
private:
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 内部辅助方法
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 填充优先级因子
    void fillPriorityFactors(
        PriorityPathResult::DetailedPriorityFactors& factors,
        const Task& task,
        const Amr& amr,
        double pathLength,
        double waitingTime,
        bool isLoaded
    );
    
    // 计算各因素得分
    void calculateFactorScores(
        PriorityPathResult::DetailedPriorityFactors& factors
    );
    
    // 计算执行状态加成
    double getExecutionStateBonus(ExecutionState state);
    
    // 计算完成度
    double calculateCompletionRatio(
        const PriorityPathResult& pathResult,
        int currentNodeId
    );
    
    // 日志
    void logInfo(const std::string& msg);
    void logWarn(const std::string& msg);
    void logError(const std::string& msg);
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 成员变量
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    std::shared_ptr<GlobalPathPlanner> globalPlanner_;  // 全局路径规划器
    Config config_;                                       // 配置参数
};

#endif // PRIORITY_PATH_PLANNER_H

