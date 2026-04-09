#ifndef TASK_ALLOCATOR_BASE_H
#define TASK_ALLOCATOR_BASE_H

#include <vector>
#include <string>
#include "TaskAllocationUtils.h"

// 任务分配结果结构体（所有算法通用）
struct AllocationResult {
    std::vector<std::vector<int>> amrTasks;  // 每个AMR的任务列表（0开始）
    std::vector<int> code;                   // 编码向量（0开始任务ID，TaskAllocationUtils::kCodeSeparator 分隔）
    std::string repairLog;                   // 修复日志（若有）
    double totalCost;                        // 总成本（由calculateTotalCost计算）
    std::vector<int> unallocatedTaskIds;     // 未能分配的任务（0开始索引，原taskList索引）
};

// 任务分配算法基类（所有算法继承此接口）
class TaskAllocatorBase {
public:
    virtual ~TaskAllocatorBase() = default;

    // 修改纯虚函数签名
    virtual AllocationResult allocate(
        const std::vector<Amr>& amrList,
        const std::vector<Task>& taskList
    ) const = 0;

    // 同步修改repairInvalidDuties函数参数（移除类型矩阵）
    virtual AllocationResult repairInvalidDuties(
        const AllocationResult& result,
        const std::vector<Amr>& amrList,
        const std::vector<Task>& taskList
    ) const;

};

#endif // TASK_ALLOCATOR_BASE_H
