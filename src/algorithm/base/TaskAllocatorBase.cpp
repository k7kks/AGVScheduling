#include "algorithm/base/TaskAllocatorBase.h"
#include "data/Amr.h"
#include "data/Task.h"
#include <sstream>
#include <limits>
#include <algorithm>
#include <cmath>
// 新增：添加 <tuple> 头文件，用于识别 std::tuple 和 std::get
#include <tuple>


// 修复无效任务（默认实现，完全对应MATLAB）
AllocationResult TaskAllocatorBase::repairInvalidDuties(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList
) const {
    AllocationResult repairedResult = result;  // 初始化修复结果为当前结果
    std::stringstream repairLogStream;         // 修复日志流
    int amrNum = amrList.size();              // AMR数量（对应MATLAB的amr_count）
    int taskNum = taskList.size();            // 任务数量（对应MATLAB的num_tasks）
    const double INF = std::numeric_limits<double>::infinity();

    // -------------------------- 1. 解析编码向量（对应MATLAB解析amr_tasks） --------------------------
    // 调用通用工具类解析编码：code → amrTasks（0开始任务ID）
    std::vector<std::vector<int>> amrTasks = TaskAllocationUtils::parseSolution(result.code);
    // 确保amrTasks数量与AMR数量一致（补全空列表）
    if (amrTasks.size() < amrNum) {
        amrTasks.resize(amrNum);
    }
    int tot_task_num = taskList.size();
    int tot_amr_num = amrList.size();

    // 后处理工具：为空闲AMR分配一条最近的未分配任务（前提 agent_cost 非INF）
    auto fillIdleAmrs = [&](std::vector<std::vector<int>>& tasksByAmr,
                            std::vector<int>& unalloc)->bool {
        if ((int)tasksByAmr.size() < amrNum) tasksByAmr.resize(amrNum);
        std::vector<char> assigned(taskNum, 0);
        for (int i=0;i<amrNum;++i) for (int t : tasksByAmr[i]) if (t>=0 && t<taskNum) assigned[t]=1;
        std::vector<char> unflag(taskNum, 0);
        for (int t : unalloc) if (t>=0 && t<taskNum && !assigned[t]) unflag[t]=1;
        bool changed=false;
        for (int i=0;i<amrNum;++i) {
            if (!tasksByAmr[i].empty()) continue; // 仅对空闲AMR做兜底分配
            double best = std::numeric_limits<double>::infinity(); int bestT=-1;
            for (int t=0;t<taskNum;++t) {
                if (!unflag[t]) continue;
                double c = TaskAllocationUtils::getCost(taskNum, t, i);
                if (c < best) { best = c; bestT = t; }
            }
            if (bestT != -1 && best < std::numeric_limits<double>::infinity()) {
                tasksByAmr[i].push_back(bestT);
                unflag[bestT]=0;
                changed = true;
            }
        }
        if (changed) {
            unalloc.clear();
            for (int t=0;t<taskNum;++t) if (unflag[t]) unalloc.push_back(t);
        }
        return changed;
    };
    // -------------------------- 2. 识别无效任务（类型不匹配） --------------------------
    // 存储无效任务：tuple<taskIdx0(0开始), originalAmrIdx(0开始), originalPos(0开始)>
    std::vector<std::tuple<int, int, int>> invalidTasks;

    for (int amrIdx = 0; amrIdx < amrNum; ++amrIdx) {
        const auto& tasks = amrTasks[amrIdx];
        const auto& amr = amrList[amrIdx];
        // 使用类型索引语义，不再计算位掩码
        if (tasks.empty()) continue;

        std::vector<int> validTasks;  // 存储当前AMR的有效任务
        for (size_t pos = 0; pos < tasks.size(); ++pos) {
            int taskIdx0 = tasks[pos];  // 任务索引（0开始）
            if (taskIdx0 < 0 || taskIdx0 >= taskNum) {
                // 越界，无效
                invalidTasks.emplace_back(taskIdx0, amrIdx, static_cast<int>(pos));
                continue;
            }
            // 检查可执行性：AMR→任务的初始成本非INF即视为可执行
            if (TaskAllocationUtils::getCost(taskNum, taskIdx0, amrIdx) < std::numeric_limits<double>::infinity()) {
                validTasks.push_back(taskIdx0);  // 有效任务保留
            } else {
                // 无效任务，记录（任务索引0开始，AMR索引0开始，位置0开始）
                invalidTasks.emplace_back(taskIdx0, amrIdx, static_cast<int>(pos));
            }
        }

        // 更新当前AMR的任务列表（移除无效任务）
        amrTasks[amrIdx] = validTasks;
    }

    // （移除重复的续航检查块，保留下一块包含充电任务排除的实现）

    // 续航约束：若某AMR累计非充电任务时间超过续航，将溢出任务标记为无效，进入后续修复
    {
    std::vector<double> enduranceSec(tot_amr_num, 1e18);
    for (int i=0;i<tot_amr_num;++i) {
        double e = amrList[i].getEndurance();
        enduranceSec[i] = (e > 0 ? e * 3600.0 * 1000.0 : 1e18);
        }
        for (int amrIdx=0; amrIdx<tot_amr_num; ++amrIdx) {
            double acc = 0.0;
            const auto& seq = amrTasks[amrIdx];
            for (size_t pos=0; pos<seq.size(); ++pos) {
                int taskIdx0 = seq[pos];
                double inc = 0.0;
                if (pos==0) inc = TaskAllocationUtils::getCost(taskNum, taskIdx0, amrIdx);
                else { int preIdx0 = seq[pos-1]; inc = TaskAllocationUtils::getCost(preIdx0, taskIdx0, amrIdx); }
                // 充电任务（taskType==3）不计入续航
                if (taskList[taskIdx0].getTaskType() != 3) acc += inc;
                if (acc > enduranceSec[amrIdx]) {
                    invalidTasks.emplace_back(taskIdx0, amrIdx, (int)pos);
                }
            }
        }
    }

    // 注：不再在此处提前返回。即便没有无效任务，也会继续执行 detour 截断与后处理，
    // 以确保“后处理在截断之后”这一流程顺序。

    // -------------------------- 4. 处理每个无效任务 --------------------------
    // 预计算各AMR当前的非充电累计时长
    std::vector<double> amrNonCharge(tot_amr_num, 0.0);
    for (int amrIdx=0; amrIdx<tot_amr_num; ++amrIdx) {
        const auto& seq = amrTasks[amrIdx];
        for (size_t pos=0; pos<seq.size(); ++pos) {
            int taskIdx0 = seq[pos];
            double inc = (pos==0)? TaskAllocationUtils::getCost(taskNum, taskIdx0, amrIdx)
                                 : TaskAllocationUtils::getCost(seq[pos-1], taskIdx0, amrIdx);
            if (taskList[taskIdx0].getTaskType() != 3) amrNonCharge[amrIdx] += inc;
        }
    }
    std::vector<double> enduranceSec2(tot_amr_num, 1e18);
    for (int i=0;i<tot_amr_num;++i) {
        double e = amrList[i].getEndurance();
        enduranceSec2[i] = (e > 0 ? e * 3600.0 * 1000.0 : 1e18);
    }

    for (const auto& invalid : invalidTasks) {
        int taskIdx0 = std::get<0>(invalid);    // 无效任务索引（0开始）
        int originalAmrIdx = std::get<1>(invalid);  // 原AMR索引（0开始）
        int originalPos = std::get<2>(invalid);     // 原任务位置（0开始）

        // -------------------------- 4.1 寻找可执行该任务的AMR（类型匹配） --------------------------
        std::vector<int> validAmrIdxs;  // 可执行该任务的AMR索引（0开始）
        for (int amrIdx = 0; amrIdx < amrNum; ++amrIdx) {
            if (TaskAllocationUtils::getCost(taskNum, taskIdx0, amrIdx) < std::numeric_limits<double>::infinity()) {
                validAmrIdxs.push_back(amrIdx);
            }
        }

        // 无可用AMR，记录日志
        if (validAmrIdxs.empty()) {
            repairLogStream << "任务" << taskIdx0 << "没有可执行的AMR；";
            repairedResult.unallocatedTaskIds.push_back(taskIdx0);
            continue;
        }

        // -------------------------- 4.2 计算最佳插入位置（最小成本增量） --------------------------
        double bestCostDelta = INF;    // 最佳成本增量
        int bestAmrIdx = -1;           // 最佳目标AMR索引
        int bestPos = -1;              // 最佳插入位置（0开始）

        for (int targetAmrIdx : validAmrIdxs) {
            const auto& targetTasks = amrTasks[targetAmrIdx];  // 目标AMR的当前任务列表
            int posNum = targetTasks.size() + 1;               // 可插入位置数（n+1个）

            for (int pos = 0; pos < posNum; ++pos) {
                double costDelta = 0.0;  // 成本增量（对应MATLAB的cost_delta）

                // -------------------------- 计算成本增量（完全对应MATLAB逻辑） --------------------------
                if (targetTasks.empty()) {
                    // 目标AMR无任务：添加“AMR→任务”的成本（costMatrix最后一行）
                    costDelta += TaskAllocationUtils::getCost(taskNum, taskIdx0, targetAmrIdx);
                } else {
                    if (pos > 0 && pos < posNum - 1) {
                        // 插入中间：移除“前任务→后任务”成本，添加“前→当前”和“当前→后”成本
                        int preTaskIdx0 = targetTasks[pos - 1];       // 前任务索引（0开始）
                        int nextTaskIdx0 = targetTasks[pos];          // 后任务索引（0开始）

                        costDelta -= TaskAllocationUtils::getCost(preTaskIdx0, nextTaskIdx0, targetAmrIdx);
                        costDelta += TaskAllocationUtils::getCost(preTaskIdx0, taskIdx0, targetAmrIdx);
                        costDelta += TaskAllocationUtils::getCost(taskIdx0, nextTaskIdx0, targetAmrIdx);
                    } else if (pos == 0) {
                        // 插入开头：移除“AMR→首任务”成本，添加“AMR→当前”和“当前→首任务”成本
                        int nextTaskIdx0 = targetTasks[pos];     // 首任务索引（0开始）

                        costDelta -= TaskAllocationUtils::getCost(taskNum, nextTaskIdx0, targetAmrIdx);
                        costDelta += TaskAllocationUtils::getCost(taskNum, taskIdx0, targetAmrIdx);
                        costDelta += TaskAllocationUtils::getCost(taskIdx0, nextTaskIdx0, targetAmrIdx);
                    } else {
                        // 插入末尾：添加“尾任务→当前任务”成本
                        int preTaskIdx0 = targetTasks[pos - 1];  // 尾任务索引（0开始）

                        costDelta += TaskAllocationUtils::getCost(preTaskIdx0, taskIdx0, targetAmrIdx);
                    }
                }

                // 续航检查：若是非充电任务，需要保证新增后不超过续航
                bool ok = true;
                if (taskList[taskIdx0].getTaskType() != 3) {
                    if (amrNonCharge[targetAmrIdx] + costDelta > enduranceSec2[targetAmrIdx]) ok = false;
                }
                // 更新最佳位置（成本增量最小）
                if (ok && costDelta < bestCostDelta) {
                    bestCostDelta = costDelta;
                    bestAmrIdx = targetAmrIdx;
                    bestPos = pos;
                }
            }
        }

        // -------------------------- 4.3 执行最佳插入 --------------------------
        if (bestAmrIdx != -1) {
            auto& targetTasks = amrTasks[bestAmrIdx];
            // 插入任务到最佳位置（0开始）
            if (bestPos == static_cast<int>(targetTasks.size())) {
                targetTasks.push_back(taskIdx0);  // 插入末尾
            } else {
                targetTasks.insert(targetTasks.begin() + bestPos, taskIdx0);  // 插入中间
            }
            // 更新续航累计
            if (taskList[taskIdx0].getTaskType() != 3) {
                amrNonCharge[bestAmrIdx] += bestCostDelta;
            }

            // 从原AMR中移除该任务（避免重复）
            if (bestAmrIdx != originalAmrIdx) {
                auto& originalTasks = amrTasks[originalAmrIdx];
                // 找到任务并删除（erase-remove惯用法）
                originalTasks.erase(
                    std::remove(originalTasks.begin(), originalTasks.end(), taskIdx0),
                    originalTasks.end()
                );
            }

            // 记录修复日志（AMR编号+1，与MATLAB显示一致）
            repairLogStream << "任务" << taskIdx0 << "从AMR" << (originalAmrIdx + 1)
                          << "移动到AMR" << (bestAmrIdx + 1) << "的位置" << (bestPos + 1) << "；";
        } else {
            repairLogStream << "任务" << taskIdx0 << "没有找到合适的插入位置（续航/类型约束）；";
            repairedResult.unallocatedTaskIds.push_back(taskIdx0);
        }
    }

    // -------------------------- 5. 清理不可达转移（移除造成 INF 的任务） --------------------------
    {
        for (int amrIdx = 0; amrIdx < tot_amr_num; ++amrIdx) {
            auto& seq = amrTasks[amrIdx];
            if (seq.empty()) continue;
            std::vector<int> cleaned;
            cleaned.reserve(seq.size());
            for (size_t pos = 0; pos < seq.size(); ++pos) {
                int cur = seq[pos];
                bool ok = true;
                if (cleaned.empty()) {
                    double c0 = TaskAllocationUtils::getCost(taskNum, cur, amrIdx);
                    if (!(c0 < std::numeric_limits<double>::infinity())) ok = false;
                } else {
                    int prev = cleaned.back();
                    double c = TaskAllocationUtils::getCost(prev, cur, amrIdx);
                    if (!(c < std::numeric_limits<double>::infinity())) ok = false;
                }
                if (ok) cleaned.push_back(cur);
                else {
                    repairedResult.unallocatedTaskIds.push_back(cur);
                    repairLogStream << "任务" << cur << " 因不可达转移被移除；";
                }
            }
            seq.swap(cleaned);
        }
    }

    // -------------------------- 6. detour筛选（仅在最后进行） --------------------------
    for (int amrIdx=0; amrIdx<tot_amr_num; ++amrIdx) {
        auto& seq = amrTasks[amrIdx];
        if (seq.size() <= 1) continue; // 单任务不筛选
        int maxAheadMs = amrList[amrIdx].getMaxTimeAheadMs();
        int maxDetourMm = amrList[amrIdx].getMaxDetourDistanceMm();
        if (maxAheadMs <= 0 && maxDetourMm <= 0) continue;
        double t = 0.0;
        double cumDetourMm = 0.0; // 累计绕路距离（任务间距离）
        const double speedMmPerSec = TaskAllocationUtils::getGlobalSpeedMmPerSec();
        // 计算每个任务的开始时间（毫秒）并判断，同时按需累计任务间距离
        int cutPosTime = -1;
        int cutPosDist = -1;
        int prevIdx0_for_dist = -1;
        for (size_t pos=0; pos<seq.size(); ++pos) {
            int taskIdx0 = seq[pos];
            // 先进行距离阈值累计与判断（从第二个任务开始拥有前驱）
            if (prevIdx0_for_dist != -1 && maxDetourMm > 0) {
                double geomMm = TaskAllocationUtils::minkowskiDistanceBetweenTasks(taskList[prevIdx0_for_dist], taskList[taskIdx0]);
                if (geomMm > 0) cumDetourMm += geomMm;
                if (cumDetourMm >= static_cast<double>(maxDetourMm)) { cutPosDist = static_cast<int>(pos); break; }
            }
            prevIdx0_for_dist = taskIdx0;

            // 时间阈值：任务开始时间
            if (maxAheadMs > 0) {
                double startTime = 0.0;
                if (pos==0) {
                    startTime = TaskAllocationUtils::getCost(taskNum, taskIdx0, amrIdx);
                    t = startTime;
                } else {
                    startTime = t; // 上一循环已累计
                }
                if (startTime >= static_cast<double>(maxAheadMs)) { cutPosTime = static_cast<int>(pos); break; }
                // 累加到下一任务的到达时间
                if (pos < seq.size()-1) {
                    int nextIdx0 = seq[pos+1];
                    t = startTime + TaskAllocationUtils::getCost(taskIdx0, nextIdx0, amrIdx);
                }
            }
        }
        int cutPos = -1;
        if (cutPosTime != -1 && cutPosDist != -1) cutPos = std::min(cutPosTime, cutPosDist);
        else if (cutPosTime != -1) cutPos = cutPosTime;
        else if (cutPosDist != -1) cutPos = cutPosDist;
        if (cutPos != -1) {
            // 至少保留一个任务：若截断位置在首位，则保底保留第一个
            int safeCut = std::max(1, cutPos);
            for (size_t p=safeCut; p<seq.size(); ++p) {
                repairedResult.unallocatedTaskIds.push_back(seq[p]);
            }
            seq.erase(seq.begin()+safeCut, seq.end());
            repairLogStream << "AMR" << (amrIdx+1) << " detour阈值触发（"
                            << (cutPosTime!=-1? "时间" : "")
                            << (cutPosTime!=-1 && cutPosDist!=-1? "+" : "")
                            << (cutPosDist!=-1? "距离" : "")
                            << ")，截断于位置" << (safeCut+1) << "；";
        }
    }

    // -------------------------- 6. 为空闲AMR进行兜底分配（可选） --------------------------
    // 先补全未分配列表：将当前未出现在任意AMR序列中的任务计为未分配
    {
        std::vector<char> assigned(taskNum, 0);
        for (const auto& seq : amrTasks) for (int t : seq) if (t>=0 && t<taskNum) assigned[t]=1;
        std::vector<char> unflag(taskNum, 0);
        for (int t : repairedResult.unallocatedTaskIds) if (t>=0 && t<taskNum) unflag[t]=1;
        for (int t=0; t<taskNum; ++t) if (!assigned[t] && !unflag[t]) repairedResult.unallocatedTaskIds.push_back(t);
    }
    (void)fillIdleAmrs(amrTasks, repairedResult.unallocatedTaskIds);

    // -------------------------- 7. 重建编码向量（对应MATLAB的new_x） --------------------------
    // 调用通用工具类编码：amrTasks → code（0开始任务ID，TaskAllocationUtils::kCodeSeparator 分隔）
    repairedResult.code = TaskAllocationUtils::deParseSolution(amrTasks);
    // 更新AMR任务列表和修复日志
    repairedResult.amrTasks = amrTasks;
    repairedResult.repairLog = repairLogStream.str();
    // 重新计算修复后的总成本
    repairedResult.totalCost = TaskAllocationUtils::calculateTotalCost(
        tot_amr_num, tot_task_num, repairedResult.code
    );

    return repairedResult;
}
