#ifndef TASK_REACHABILITY_FILTER_H
#define TASK_REACHABILITY_FILTER_H

#include <vector>
#include <string>
#include "data/Task.h"
#include "data/MapInfo.h"
#include "algorithm/base/TaskAllocatorBase.h"
#include "data/Amr.h"

class ShortestPathUpdater;

namespace TaskReachabilityFilter {

struct FilterResult {
    std::vector<Task> filteredTasks;          // 参与调度的任务
    std::vector<int> filteredToOriginal;      // 过滤后索引 -> 原始索引
    std::vector<int> originalToFiltered;      // 原始索引 -> 过滤后索引（无则-1）
    std::vector<int> forcedTaskIndices;       // 被判定不可达的原始索引
    std::vector<std::string> forcedReasons;   // 与原始索引对齐的不可达原因描述
};

FilterResult FilterTasksByReachability(
    const std::vector<Task>& tasks,
    const MapInfo& mapInfo,
    const std::vector<Amr>& candidateAmrs);

AllocationResult RemapAllocationToOriginal(
    const AllocationResult& filteredResult,
    const std::vector<int>& filteredToOriginal,
    size_t originalTaskCount);

void AppendForcedTasks(AllocationResult& result,
                       const std::vector<int>& forcedTaskIndices);

std::vector<int> InsertForcedTasks(
    AllocationResult& result,
    const std::vector<int>& forcedTaskIndices,
    const std::vector<int>& availableToFull,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& originalTasks,
    ShortestPathUpdater& pathUpdater,
    const MapInfo& mapInfo);

std::vector<double> ComputeChainDurationsSec(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& originalTasks,
    ShortestPathUpdater& pathUpdater,
    const MapInfo& mapInfo);

}  // namespace TaskReachabilityFilter

#endif  // TASK_REACHABILITY_FILTER_H
