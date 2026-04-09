// agv_cluster_scheduling/include/algorithm/PostaTaskAllocator.h
#ifndef POSTA_TASK_ALLOCATOR_H
#define POSTA_TASK_ALLOCATOR_H

#include "algorithm/base/TaskAllocatorBase.h"
#include <vector>
#include <random>
#include <optional>

// POSTA 分配算法实现（支持可选初始解）
class PostaTaskAllocator : public TaskAllocatorBase {
public:
    PostaTaskAllocator();

    // 与 Greedy 算法相同的接口
    AllocationResult allocate(
        const std::vector<Amr>& amrList,
        const std::vector<Task>& taskList
    ) const override;

    // 可选：设置初始解（编码向量，使用 TaskAllocationUtils::kCodeSeparator 分隔，任务编号为 0..N-1）
    void setInitialSolution(const std::vector<int>& code);

    // 可选：配置时间限制（秒）和候选数量 SE
    void setTimeLimit(double seconds);
    void setCandidateCount(int se);
    void setBanShuffle(bool ban);
    // 固定随机数种子（如未显式设置，则沿用环境变量或默认值）
    void setRandomSeed(uint32_t seed);

    // 配置：优先级惩罚项（a 系数 + 曲线/列表，索引1..10，0位忽略）
    void setPriorityPenaltyCoeff(double a) { priorityPenaltyCoeff_ = a; }
    void setPriorityPenaltyCurve(const std::vector<double>& curve) { priorityPenaltyCurve_ = curve; }

private:
    // 可变配置（使用 mutable 以便在 const allocate 中访问）
    mutable std::optional<std::vector<int>> initialCode_;
    mutable double timeLimitSec_;
    mutable int SE_;
    mutable bool banShuffle_;
    mutable std::optional<uint32_t> fixedSeed_;

    // 优先级惩罚参数
    mutable double priorityPenaltyCoeff_;
    mutable std::vector<double> priorityPenaltyCurve_; // 约定：长度>=11，索引1..10有效
};

#endif // POSTA_TASK_ALLOCATOR_H
