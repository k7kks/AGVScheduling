#ifndef COST_MATRIX_GENERATOR_H
#define COST_MATRIX_GENERATOR_H

#include <vector>
#include <map>
#include "data/Amr.h"
#include "data/Task.h"
#include "algorithm/ShortestPathUpdater.h"


// 成本矩阵生成器（专注于type_match_matrix和cost_matrix的生成）
class CostMatrixGenerator {
public:

    // 生成成本矩阵（对应MATLAB的generate_cost_matrix及前后逻辑）
    static std::vector<std::vector<std::vector<double>>> generateCostMatrix(
        const std::vector<Amr>& amrs,
        const std::vector<Task>& taskSet,
        const ShortestPathUpdater& shortestPathUpdater,
        const MapInfo& mapInfo
    );

    // 扩展版：允许覆盖AMR起点节点与初始时间偏移（毫秒）。
    // 若 startNodeOverride.size()==amrs.size() 且值!=-1，则使用该节点ID作为起点；
    // 若 startTimeOffsetMs.size()==amrs.size()，则在AMR→任务的初始成本上加上该偏移（近似先占用时间）。
    static std::vector<std::vector<std::vector<double>>> generateCostMatrix(
        const std::vector<Amr>& amrs,
        const std::vector<Task>& taskSet,
        const ShortestPathUpdater& shortestPathUpdater,
        const MapInfo& mapInfo,
        const std::vector<int>& startNodeOverride,
        const std::vector<double>& startTimeOffsetMs
    );

    // 检查任务是否有效（类型匹配）
    static bool isTaskValid(Amr amr, Task task);

private:
    // 生成agent_cost（AMR到任务的初始成本）
    static std::vector<std::vector<double>> generateAgentCost(
        const std::vector<Amr>& amrs,
        const std::vector<Task>& taskSet,
        const ShortestPathUpdater& shortestPathUpdater
    );

    // 生成task_cost（任务间的转移成本）
    static std::vector<std::vector<std::vector<double>>> generateTaskCost(
        const std::vector<Amr>& amrs,
        const std::vector<Task>& taskSet,
        const ShortestPathUpdater& shortestPathUpdater
    );

};

#endif // COST_MATRIX_GENERATOR_H
