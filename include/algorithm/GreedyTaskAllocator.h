#ifndef GREEDY_TASK_ALLOCATOR_H
#define GREEDY_TASK_ALLOCATOR_H

#include "algorithm/base/TaskAllocatorBase.h"
#include "data/Amr.h"
#include "data/Task.h"
#include <vector>

class GreedyTaskAllocator : public TaskAllocatorBase {
public:
    // 新的函数签名：输入AMR列表、任务列表和成本矩阵
    AllocationResult allocate(
        const std::vector<Amr>& amrList,
        const std::vector<Task>& taskList
    ) const override;
};

#endif // GREEDY_TASK_ALLOCATOR_H
