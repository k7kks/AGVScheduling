// agv_cluster_scheduling/include/common/AllocationReachability.h
#ifndef ALLOCATION_REACHABILITY_H
#define ALLOCATION_REACHABILITY_H

#include <vector>
#include <string>
#include "algorithm/base/TaskAllocatorBase.h"
#include "data/Amr.h"
#include "data/Task.h"
#include "data/MapInfo.h"
#include "algorithm/ShortestPathUpdater.h"

// 对任务分配结果进行“硬性可达性”校验与修复：
// - 逐 AMR 校验 AMR 起点→任务起点、任务内部起点→终点 的可达性（基于 ShortestPathUpdater）
// - 移除不可达任务后，尝试在所有 AMR/所有位置按最小增量重新插入
// - 未能重新插入的任务合入 unallocatedTaskIds（自动去重）
// - 返回修复后的结果（更新 amrTasks/code/unallocatedTaskIds/repairLog/totalCost）
// 注意：amrList 应为本次参与分配的可用 AMR 列表（与 result 中的顺序对应）
AllocationResult SanitizeAllocationResult(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo& mapInfo,
    const ShortestPathUpdater& pathUpdater
);

#endif // ALLOCATION_REACHABILITY_H

