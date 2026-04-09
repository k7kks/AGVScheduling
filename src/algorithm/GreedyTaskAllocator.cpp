#include "algorithm/GreedyTaskAllocator.h"
#include <limits>
#include <iostream>
#include <cstdlib>
#include "data/Amr.h"
#include "data/Task.h"

// 贪心分配实现（修改版本：直接使用AMR和Task列表，移除类型匹配矩阵）
AllocationResult GreedyTaskAllocator::allocate(
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList
) const {
    AllocationResult result;
    int taskNum = taskList.size();                    // 从任务列表获取任务数量
    int amrNum = amrList.size();                      // 从AMR列表获取AMR数量
    const double INF = std::numeric_limits<double>::max();

    // 初始化：每个AMR的任务列表、任务分配状态
    std::vector<std::vector<int>> amrTasks(amrNum);   // 存储任务索引（对应taskList）
    std::vector<bool> isAllocated(taskNum, false);
    int unallocatedNum = taskNum;
    int stage = 0;  // 0：正常分配（可根据实际需求增加自定义过滤逻辑），1：强制分配
    // 续航秒（与其它模块一致：e<=0 视为无限）
    std::vector<double> enduranceSec(amrNum, 1e18);
    for (int i = 0; i < amrNum; ++i) {
        double e = amrList[i].getEndurance();
        enduranceSec[i] = (e > 0 ? e * 3600.0 * 1000.0 : 1e18);
    }
    // 累计已分配时间（毫秒）
    std::vector<double> amrAccum(amrNum, 0.0);
    // 任务到达时间（优先取 createTimestampISO，其次 expectedStartTimeISO）
    std::vector<double> taskArriveSec(taskNum, -1.0);
    for (int i = 0; i < taskNum; ++i) {
        const auto& t = taskList[i];
        const std::string& ts = !t.getCreateTimestampISO().empty()
            ? t.getCreateTimestampISO()
            : t.getExpectedStartTimeISO();
        if (!ts.empty()) {
            double sec = 0.0;
            if (TaskAllocationUtils::parseIso8601ToUnixSeconds(ts, sec)) {
                taskArriveSec[i] = sec;
            }
        }
    }
    const double baseNowSec = TaskAllocationUtils::nowUnixSeconds();
    double waitPenaltyCoeff = 0.0;
    if (const char* ev = std::getenv("ALLOC_WAIT_PENALTY_COEFF")) {
        try { waitPenaltyCoeff = std::stod(ev); } catch(...) {}
    }

    while (unallocatedNum > 0) {
        int prevUnallocated = unallocatedNum;
        std::vector<int> tempAllocate(taskNum, -1);  // 任务→AMR映射（-1未分配）
        std::vector<double> tempCost(taskNum, INF);   // 任务分配成本

        // 遍历每个AMR，选择最优任务
        for (int amrId = 0; amrId < amrNum; ++amrId) {
            int bestTask = -1;
            double bestCost = INF;

            for (int taskId = 0; taskId < taskNum; ++taskId) {
                if (isAllocated[taskId]) continue;

                // （可选）阶段0：可添加自定义过滤逻辑（如AMR负载、任务优先级等）
                if (stage == 0) {
                    // 示例：可添加基于AMR和任务属性的过滤
                    // 例如：if (amrList[amrId].getBatteryLevel() < 20 && taskList[taskId].getPriority() > 3) continue;
                }

                // 计算基础成本（秒）：首个任务用初始成本，后续用转移成本
                double cost;
                if (amrTasks[amrId].empty()) {
                    cost = TaskAllocationUtils::getCost(taskNum, taskId, amrId);
                } else {
                    int lastTask = amrTasks[amrId].back();
                    cost = TaskAllocationUtils::getCost(lastTask, taskId, amrId);
                }

                // 跳过不可达/无效段（确保Greedy不产生包含INF的解）
                if (!std::isfinite(cost)) continue;

                // 等待时间惩罚（与 POSTA 一致的到达时间定义）
                double effectiveCost = cost;
                if (waitPenaltyCoeff > 0.0 && baseNowSec > 0.0 &&
                    taskId < (int)taskArriveSec.size() && taskArriveSec[taskId] > 0.0) {
                    double startSec = baseNowSec + (amrAccum[amrId] / 1000.0);
                    double waitSec = startSec - taskArriveSec[taskId];
                    if (waitSec > 0.0) {
                        effectiveCost += waitPenaltyCoeff * waitSec * 1000.0;
                    }
                }

                // // 续航约束（不含充电任务，约定taskType==3为充电）
                // if (taskList[taskId].getTaskType() != 3) {
                //     if (amrAccum[amrId] + cost > enduranceSec[amrId]) continue;
                // }

                // 更新最优任务
                if (effectiveCost < bestCost ||
                    (effectiveCost == bestCost && tempAllocate[taskId] == -1)) {
                    bestCost = effectiveCost;
                    bestTask = taskId;
                }
            }

            // 更新临时分配表
            if (bestTask != -1 && bestCost < tempCost[bestTask]) {
                tempAllocate[bestTask] = amrId;
                tempCost[bestTask] = bestCost;
            }
        }

        // 执行实际分配
        for (int taskId = 0; taskId < taskNum; ++taskId) {
            if (tempAllocate[taskId] != -1) {
                int amrId = tempAllocate[taskId];
                amrTasks[amrId].push_back(taskId);
                isAllocated[taskId] = true;
                unallocatedNum--;
                // 更新该AMR累计时间
                double inc = 0.0;
                if (amrTasks[amrId].size() == 1) inc = TaskAllocationUtils::getCost(taskNum, taskId, amrId);
                else {
                    int lastTask = amrTasks[amrId][amrTasks[amrId].size()-2];
                    inc = TaskAllocationUtils::getCost(lastTask, taskId, amrId);
                }
                if (taskList[taskId].getTaskType() != 3) amrAccum[amrId] += inc;
            }
        }

        // 若未分配任务数未减少，则无法继续推进，退出循环（剩余任务交由修复阶段处理）
        if (unallocatedNum == prevUnallocated) {
            break;
        }
    }

    // 生成编码并修复无效任务（Greedy 不做 detour 截断）
    result.amrTasks = amrTasks;
    result.code = TaskAllocationUtils::deParseSolution(amrTasks);
    
    // 通过将 detour 阈值置零，禁用修复阶段的 detour 截断，保留其它修复逻辑
    std::vector<Amr> amrListNoDetour = amrList;
    for (auto& a : amrListNoDetour) { a.setMaxTimeAheadMs(0); a.setMaxDetourDistanceMm(0); }
    result = repairInvalidDuties(result, amrListNoDetour, taskList);
    return result;
}
