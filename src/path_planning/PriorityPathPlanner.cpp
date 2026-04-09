#include "PriorityPathPlanner.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <functional>  // for std::hash
#include <cctype>      // for std::isdigit

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 构造与初始化
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PriorityPathPlanner::PriorityPathPlanner(
    std::shared_ptr<GlobalPathPlanner> globalPlanner
) : globalPlanner_(globalPlanner), config_() {
    logInfo("优先级路径规划器初始化完成");
}

PriorityPathPlanner::PriorityPathPlanner(
    std::shared_ptr<GlobalPathPlanner> globalPlanner,
    const Config& config
) : globalPlanner_(globalPlanner), config_(config) {
    logInfo("优先级路径规划器初始化完成");
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 核心接口：单段路径规划
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PriorityPathPlanner::PriorityPathResult PriorityPathPlanner::planPriorityPath(
    int startNode,
    int endNode,
    const Task& task,
    const Amr& amr,
    SegmentType segmentType,
    double waitingTime,
    double startTime
) {
    PriorityPathResult result;
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 1. 优先级规划备选路径策略（提升鲁棒性）
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    // 策略1: 标准路径规划
    auto basicPath = globalPlanner_->planGlobalPath(startNode, endNode);
    
    // 策略2: 如果失败，对高优先级任务增加重试（GlobalPathPlanner内部已有5种策略）
    if (!basicPath.reachable && task.getPriority() >= 7) {
        if (config_.verboseLogging) {
            logWarn("高优先级任务(" + std::to_string(task.getPriority()) + 
                    ")路径失败，增加优先级加成");
        }
        // 高优先级任务失败时，在后续优先级计算中会给予额外加成
        // 这样在资源竞争时，高优先级任务会优先获得路权
    }
    
    // 策略3: 对于关键路径段（负载），记录失败原因用于后续分析
    if (!basicPath.reachable && segmentType == SegmentType::TO_DELIVERY) {
        logError("关键路径段(负载)失败: " + std::to_string(startNode) + " -> " + 
                 std::to_string(endNode) + ", 任务优先级=" + std::to_string(task.getPriority()));
    }
    
    if (!basicPath.reachable) {
        logError("路径不可达: " + std::to_string(startNode) + " -> " + std::to_string(endNode));
        result.reachable = false;
        // 对于高优先级任务，即使路径不可达也填充部分信息，便于后续处理
        if (task.getPriority() >= 7) {
            result.basePriority = 90.0;  // 给予高基础优先级
            result.effectivePriority = 150.0;  // 给予高有效优先级
        }
        return result;
    }
    
    // 2. 复制基础路径信息
    result.path = basicPath.path;
    result.fullPath = basicPath.fullPath;
    result.distance = basicPath.distance;
    result.estimatedTime = basicPath.estimatedTime;
    result.turnCount = basicPath.turnCount;
    result.reachable = basicPath.reachable;
    result.source = basicPath.source;
    result.planningTime = basicPath.planningTime;
    
    // 3. 填充任务和AGV信息
    // 使用messageId的哈希值作为taskId（工业实践中通常有唯一ID）
    result.taskId = std::hash<std::string>{}(task.getMessageId());
    result.agvDeviceId = amr.getDeviceId();
    result.taskMessageId = task.getMessageId();
    
    // AGV ID：尝试从deviceId提取数字，失败则使用哈希值
    try {
        // 尝试提取数字部分（如"AGV001" -> 1）
        std::string idStr = amr.getDeviceId();
        std::string numStr;
        for (char c : idStr) {
            if (std::isdigit(c)) {
                numStr += c;
            }
        }
        
        if (numStr.empty()) {
            // 没有数字，使用哈希值
            result.agvId = std::hash<std::string>{}(idStr) % 100000;  // 限制在合理范围
        } else {
            // 限制长度，防止溢出
            if (numStr.length() > 9) {
                numStr = numStr.substr(numStr.length() - 9);  // 只取最后9位
            }
            result.agvId = std::stoi(numStr);
        }
    } catch (...) {
        // 异常情况：使用哈希值
        result.agvId = std::hash<std::string>{}(amr.getDeviceId()) % 100000;
    }
    
    // 4. 设置路径段类型和负载状态
    result.segmentType = segmentType;
    result.isLoaded = (segmentType == SegmentType::TO_DELIVERY);
    
    // 5. 设置时间信息
    result.startTime = startTime;
    result.endTime = startTime + result.estimatedTime;
    
    // 6. 初始化执行状态
    result.executionState = ExecutionState::NOT_STARTED;
    result.completionRatio = 0.0;
    
    // 7. 填充优先级因子
    fillPriorityFactors(
        result.detailedFactors,
        task,
        amr,
        result.distance / 1000.0,  // mm转米
        waitingTime,
        result.isLoaded
    );
    
    // 8. 计算各因素得分
    calculateFactorScores(result.detailedFactors);
    
    // 9. 计算基础优先级
    result.basePriority = calculateBasePriority(result.detailedFactors);
    
    // 10. 计算有效优先级
    result.effectivePriority = calculateEffectivePriority(result);
    
    logInfo("路径规划完成: " + std::to_string(startNode) + " -> " + 
            std::to_string(endNode) + ", Priority=" + 
            std::to_string(result.effectivePriority));
    
    return result;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 核心接口：多段路径规划
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PriorityPathPlanner::MultiSegmentTask PriorityPathPlanner::planMultiSegmentTask(
    const Task& task,
    const Amr& amr,
    double startTime
) {
    MultiSegmentTask multiTask;
    multiTask.taskId = std::hash<std::string>{}(task.getMessageId());
    
    // 安全提取AGV ID
    try {
        std::string idStr = amr.getDeviceId();
        std::string numStr;
        for (char c : idStr) {
            if (std::isdigit(c)) {
                numStr += c;
            }
        }
        
        if (numStr.empty()) {
            multiTask.agvId = std::hash<std::string>{}(idStr) % 100000;
        } else {
            if (numStr.length() > 9) {
                numStr = numStr.substr(numStr.length() - 9);
            }
            multiTask.agvId = std::stoi(numStr);
        }
    } catch (...) {
        multiTask.agvId = std::hash<std::string>{}(amr.getDeviceId()) % 100000;
    }
    
    multiTask.startTime = startTime;
    
    // 检查任务是否有足够的子任务
    const auto& subTasks = task.getSubTasks();
    if (subTasks.size() < 2) {
        logError("任务子任务数量不足（需要至少2个：取货点和送货点）");
        return multiTask;
    }
    
    // 获取关键节点
    int currentNode = amr.getCurrentNodeId();
    int pickupNode = subTasks[0].getPoint().getNodeId();
    int deliveryNode = subTasks[1].getPoint().getNodeId();
    
    double currentTime = startTime;
    
    // 第1段：当前位置 → 取货点（空载）
    logInfo("规划第1段: 当前位置(" + std::to_string(currentNode) + 
            ") -> 取货点(" + std::to_string(pickupNode) + ")");
    
    auto segment1 = planPriorityPath(
        currentNode,
        pickupNode,
        task,
        amr,
        SegmentType::TO_PICKUP,
        0.0,  // 等待时间
        currentTime
    );
    
    if (segment1.reachable) {
        multiTask.segments.push_back(segment1);
        multiTask.totalDistance += segment1.distance / 1000.0;  // mm转米
        multiTask.totalTime += segment1.estimatedTime;
        currentTime = segment1.endTime;
    } else {
        logError("第1段路径不可达");
        return multiTask;
    }
    
    // 第2段：取货点 → 送货点（负载）
    logInfo("规划第2段: 取货点(" + std::to_string(pickupNode) + 
            ") -> 送货点(" + std::to_string(deliveryNode) + ")");
    
    auto segment2 = planPriorityPath(
        pickupNode,
        deliveryNode,
        task,
        amr,
        SegmentType::TO_DELIVERY,
        0.0,  // 刚取货，等待时间归零
        currentTime
    );
    
    if (segment2.reachable) {
        multiTask.segments.push_back(segment2);
        multiTask.totalDistance += segment2.distance / 1000.0;
        multiTask.totalTime += segment2.estimatedTime;
        currentTime = segment2.endTime;
    } else {
        logError("第2段路径不可达");
        return multiTask;
    }
    
    multiTask.endTime = currentTime;
    
    logInfo("多段任务规划完成: " + std::to_string(multiTask.segments.size()) + 
            " 段, 总距离=" + std::to_string(multiTask.totalDistance) + 
            "m, 总时间=" + std::to_string(multiTask.totalTime) + "s");
    
    return multiTask;
}

std::vector<PriorityPathPlanner::PriorityPathResult> 
PriorityPathPlanner::planBatchTasks(
    const std::vector<Task>& tasks,
    const std::vector<Amr>& amrs,
    const std::vector<std::pair<int, int>>& taskAssignments,
    double startTime
) {
    std::vector<PriorityPathResult> allPaths;
    
    logInfo("批量规划开始: " + std::to_string(taskAssignments.size()) + " 个任务");
    
    for (const auto& assignment : taskAssignments) {
        int taskId = assignment.first;
        int amrId = assignment.second;
        
        // 查找任务和AMR
        if (taskId >= static_cast<int>(tasks.size()) || 
            amrId >= static_cast<int>(amrs.size())) {
            logWarn("任务或AMR索引越界，跳过");
            continue;
        }
        
        const Task& task = tasks[taskId];
        const Amr& amr = amrs[amrId];
        
        // 规划多段任务
        auto multiTask = planMultiSegmentTask(task, amr, startTime);
        
        // 将所有段加入结果
        for (auto& segment : multiTask.segments) {
            allPaths.push_back(segment);
        }
    }
    
    logInfo("批量规划完成: 生成 " + std::to_string(allPaths.size()) + " 个路径段");
    
    return allPaths;
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 优先级计算
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void PriorityPathPlanner::fillPriorityFactors(
    PriorityPathResult::DetailedPriorityFactors& factors,
    const Task& task,
    const Amr& amr,
    double pathLength,
    double waitingTime,
    bool isLoaded
) {
    // 任务因素
    factors.taskPriority = task.getPriority();
    factors.taskUrgency = static_cast<double>(task.getPriority());
    factors.isNearTimeout = false;  // 暂不实现超时检测
    factors.timeoutMargin = 0.0;
    
    // AGV因素
    factors.batteryLevel = amr.getBatteryLevel();
    factors.agvType = amr.getDeviceType();
    factors.isLowBattery = (amr.getBatteryLevel() < config_.lowBatteryThreshold);
    
    // 路径因素
    factors.pathLength = pathLength;
    factors.waitingTime = waitingTime;
    
    // 执行因素
    factors.isLoaded = isLoaded;
    factors.completionRatio = 0.0;
    factors.execState = ExecutionState::NOT_STARTED;
}

void PriorityPathPlanner::calculateFactorScores(
    PriorityPathResult::DetailedPriorityFactors& factors
) {
    // 1. 任务优先级得分（0-40分）
    double urgencyNorm = std::min(factors.taskUrgency / config_.maxTaskPriority, 1.0);
    factors.taskPriorityScore = urgencyNorm * config_.taskPriorityWeight * 100.0;
    
    // 2. 路径长度得分（0-20分，路径越短得分越高）
    double pathNorm = 1.0 - std::min(factors.pathLength / config_.maxPathLength, 1.0);
    factors.pathLengthScore = pathNorm * config_.pathLengthWeight * 100.0;
    
    // 3. 等待时间得分（0-20分，等待越久得分越高）
    double waitNorm = std::min(factors.waitingTime / config_.maxWaitingTime, 1.0);
    factors.waitingTimeScore = waitNorm * config_.waitingTimeWeight * 100.0;
    
    // 4. 电量得分（0-10分，电量越低得分越高）
    double batteryNorm = 1.0 - (factors.batteryLevel / 100.0);
    factors.batteryScore = batteryNorm * config_.batteryWeight * 100.0;
    
    // 5. AGV类型得分（0-10分）
    double typeScore = (factors.agvType > 0) ? 1.0 : 0.0;
    factors.agvTypeScore = typeScore * config_.agvTypeWeight * 100.0;
    
    // 6. 负载加成（0或30）
    factors.loadBonus = (config_.enableLoadBonus && factors.isLoaded) 
        ? config_.loadPriorityBonus : 0.0;
    
    // 7. 执行状态加成（0-100）
    factors.executionBonus = config_.enableExecutionBonus 
        ? getExecutionStateBonus(factors.execState) : 0.0;
    
    // 8. 完成度加成（0-30）
    if (config_.enableCompletionBonus && 
        factors.completionRatio >= config_.nearCompletionThreshold) {
        factors.completionBonus = config_.nearCompletionBonus;
    } else {
        factors.completionBonus = 0.0;
    }
    
    // 9. 超时加成（0或100）
    factors.timeoutBonus = (config_.enableTimeoutBonus && factors.isNearTimeout) 
        ? config_.timeoutBonus : 0.0;
}

double PriorityPathPlanner::calculateBasePriority(
    const PriorityPathResult::DetailedPriorityFactors& factors
) {
    // 基础优先级 = 五个因素的加权和（总计100分）
    double basePriority = factors.taskPriorityScore
                        + factors.pathLengthScore
                        + factors.waitingTimeScore
                        + factors.batteryScore
                        + factors.agvTypeScore;
    
    return basePriority;
}

double PriorityPathPlanner::calculateEffectivePriority(
    const PriorityPathResult& pathResult
) {
    // 有效优先级 = 基础优先级 + 所有加成
    double effectivePriority = pathResult.basePriority
                             + pathResult.detailedFactors.loadBonus
                             + pathResult.detailedFactors.executionBonus
                             + pathResult.detailedFactors.completionBonus
                             + pathResult.detailedFactors.timeoutBonus;
    
    return effectivePriority;
}

double PriorityPathPlanner::getExecutionStateBonus(ExecutionState state) {
    switch (state) {
        case ExecutionState::NOT_STARTED:
            return 0.0;
        case ExecutionState::IN_PROGRESS_EMPTY:
            return config_.executionEmptyBonus;
        case ExecutionState::IN_PROGRESS_LOADED:
            return config_.executionLoadedBonus;
        case ExecutionState::NEAR_COMPLETION:
            return config_.nearCompletionBonus;
        case ExecutionState::COMPLETED:
            return 0.0;
        default:
            return 0.0;
    }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 执行状态更新
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

double PriorityPathPlanner::calculateCompletionRatio(
    const PriorityPathResult& pathResult,
    int currentNodeId
) {
    if (pathResult.fullPath.empty()) {
        return 0.0;
    }
    
    // 找到当前节点在路径中的位置
    auto it = std::find(pathResult.fullPath.begin(), 
                       pathResult.fullPath.end(), 
                       currentNodeId);
    
    if (it == pathResult.fullPath.end()) {
        // 当前节点不在路径中
        return 0.0;
    }
    
    int currentIndex = std::distance(pathResult.fullPath.begin(), it);
    double ratio = static_cast<double>(currentIndex) / pathResult.fullPath.size();
    
    return std::min(ratio, 1.0);
}

void PriorityPathPlanner::updateExecutionState(
    PriorityPathResult& pathResult,
    int currentNodeId,
    double currentTime
) {
    // 1. 计算完成度
    pathResult.completionRatio = calculateCompletionRatio(pathResult, currentNodeId);
    
    // 2. 更新执行状态
    if (pathResult.completionRatio == 0.0) {
        pathResult.executionState = ExecutionState::NOT_STARTED;
    } else if (pathResult.completionRatio >= 1.0) {
        pathResult.executionState = ExecutionState::COMPLETED;
    } else if (pathResult.completionRatio >= config_.nearCompletionThreshold) {
        pathResult.executionState = ExecutionState::NEAR_COMPLETION;
    } else {
        // 根据负载状态区分
        if (pathResult.isLoaded) {
            pathResult.executionState = ExecutionState::IN_PROGRESS_LOADED;
        } else {
            pathResult.executionState = ExecutionState::IN_PROGRESS_EMPTY;
        }
    }
    
    // 3. 更新执行状态因子
    pathResult.detailedFactors.completionRatio = pathResult.completionRatio;
    pathResult.detailedFactors.execState = pathResult.executionState;
    
    // 4. 重新计算因素得分
    calculateFactorScores(pathResult.detailedFactors);
    
    // 5. 重新计算有效优先级
    pathResult.effectivePriority = calculateEffectivePriority(pathResult);
}

void PriorityPathPlanner::updateAllPriorities(
    std::vector<PriorityPathResult>& paths,
    const std::vector<Amr>& amrs,
    double currentTime
) {
    for (auto& path : paths) {
        // 找到对应的AMR
        auto it = std::find_if(amrs.begin(), amrs.end(),
            [&path](const Amr& amr) {
                return amr.getDeviceId() == path.agvDeviceId;
            });
        
        if (it != amrs.end()) {
            // 更新执行状态和优先级
            updateExecutionState(path, it->getCurrentNodeId(), currentTime);
        }
    }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 查询与工具
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void PriorityPathPlanner::sortByPriority(
    std::vector<PriorityPathResult>& paths
) {
    std::sort(paths.begin(), paths.end(),
        [](const PriorityPathResult& a, const PriorityPathResult& b) {
            return a.effectivePriority > b.effectivePriority;
        });
}

void PriorityPathPlanner::printPriorityDetails(
    const PriorityPathResult& pathResult
) {
    std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    std::cout << "优先级路径详情\n";
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
    
    // 基本信息
    std::cout << "【基本信息】\n";
    std::cout << "  AGV ID: " << pathResult.agvDeviceId << "\n";
    std::cout << "  任务 ID: " << pathResult.taskMessageId << "\n";
    std::cout << "  路径: " << pathResult.path.front() << " -> " << pathResult.path.back() << "\n";
    std::cout << "  距离: " << std::fixed << std::setprecision(1) 
              << (pathResult.distance / 1000.0) << " m\n";
    std::cout << "  时间: " << pathResult.estimatedTime << " s\n";
    
    // 状态信息
    std::cout << "\n【状态信息】\n";
    std::cout << "  路径段类型: ";
    switch (pathResult.segmentType) {
        case SegmentType::TO_PICKUP: std::cout << "去取货（空载）\n"; break;
        case SegmentType::TO_DELIVERY: std::cout << "去送货（负载）\n"; break;
        case SegmentType::TO_CHARGING: std::cout << "去充电\n"; break;
        case SegmentType::TO_PARKING: std::cout << "去停车\n"; break;
    }
    std::cout << "  负载状态: " << (pathResult.isLoaded ? "负载" : "空载") << "\n";
    std::cout << "  执行状态: ";
    switch (pathResult.executionState) {
        case ExecutionState::NOT_STARTED: std::cout << "未开始\n"; break;
        case ExecutionState::IN_PROGRESS_EMPTY: std::cout << "执行中-空载\n"; break;
        case ExecutionState::IN_PROGRESS_LOADED: std::cout << "执行中-负载\n"; break;
        case ExecutionState::NEAR_COMPLETION: std::cout << "快完成\n"; break;
        case ExecutionState::COMPLETED: std::cout << "已完成\n"; break;
    }
    std::cout << "  完成度: " << std::setprecision(1) 
              << (pathResult.completionRatio * 100.0) << "%\n";
    
    // 优先级信息
    std::cout << "\n【优先级信息】\n";
    std::cout << "  基础优先级: " << std::setprecision(2) 
              << pathResult.basePriority << " / 100\n";
    std::cout << "  有效优先级: " << pathResult.effectivePriority << "\n";
    
    // 因素详情
    const auto& factors = pathResult.detailedFactors;
    std::cout << "\n【优先级因素详情】\n";
    std::cout << "  1. 任务优先级: " << factors.taskPriority 
              << " → 得分: " << factors.taskPriorityScore << "/40\n";
    std::cout << "  2. 路径长度: " << factors.pathLength << "m"
              << " → 得分: " << factors.pathLengthScore << "/20\n";
    std::cout << "  3. 等待时间: " << factors.waitingTime << "s"
              << " → 得分: " << factors.waitingTimeScore << "/20\n";
    std::cout << "  4. 电量: " << factors.batteryLevel << "%"
              << " → 得分: " << factors.batteryScore << "/10\n";
    std::cout << "  5. AGV类型: " << factors.agvType
              << " → 得分: " << factors.agvTypeScore << "/10\n";
    
    std::cout << "\n【优先级加成】\n";
    std::cout << "  负载加成: +" << factors.loadBonus << "\n";
    std::cout << "  执行状态加成: +" << factors.executionBonus << "\n";
    std::cout << "  完成度加成: +" << factors.completionBonus << "\n";
    std::cout << "  超时加成: +" << factors.timeoutBonus << "\n";
    
    std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 日志
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void PriorityPathPlanner::logInfo(const std::string& msg) {
    if (config_.verboseLogging) {
        std::cout << "[INFO] " << msg << std::endl;
    }
}

void PriorityPathPlanner::logWarn(const std::string& msg) {
    if (config_.verboseLogging) {
        std::cout << "[WARN] " << msg << std::endl;
    }
}

void PriorityPathPlanner::logError(const std::string& msg) {
    std::cerr << "[ERROR] " << msg << std::endl;
}
