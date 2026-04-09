// agv_cluster_scheduling/include/message/ResultPublisher.h
#ifndef RESULT_PUBLISHER_H
#define RESULT_PUBLISHER_H

#include <string>
#include <vector>
#include <optional>
#include "common/RabbitMQConfig.h"
#include "common/SchedulingRequest.h"
#include "algorithm/base/TaskAllocatorBase.h"
#include "data/Amr.h"
#include "data/Task.h"
#include "data/MapInfo.h"
#include "path_planning/PathPlanningHelper.h"

class StaticPathTable;

// 长连接版本：可复用 RabbitMQ 连接的结果发布器
class ResultPublisher {
public:
    explicit ResultPublisher(const RabbitMQConfig& cfg);
    ~ResultPublisher();

    ResultPublisher(const ResultPublisher&) = delete;
    ResultPublisher& operator=(const ResultPublisher&) = delete;

    // 设置结果落盘目录（为空表示关闭）
    void setDumpDirectory(const std::string& directory);

    std::optional<std::vector<PathPlanningHelper::AmrPlanInfo>> publish(
        const AllocationResult& result,
        const std::vector<Amr>& amrList,
        const std::vector<Task>& taskList,
        const MapInfo* mapInfo,
        long calculation_time_ms = 0,
        double optimization_score = 0.0,
        const std::vector<std::string>& constraints_violated = {},
        const SchedulingRequest* scheduling_request = nullptr,
        const StaticPathTable* staticTable = nullptr,
        int status_code = 0
    );

private:
    struct Impl;
    Impl* impl_;
    std::string dump_directory_;
    bool dump_enabled_;
};

// 将分配结果封装为JSON并通过RabbitMQ发送
// env 覆盖项：
//   ASSIGN_RESULT_EXCHANGE (默认 AlgoToDispExchange)
//   ASSIGN_RESULT_QUEUE    (默认 AlgoToDispQueue)
//   ASSIGN_RESULT_ROUTING_KEY (默认 AssignmentTaskResponse)
// 返回：路径规划结果（若有）或空
std::optional<std::vector<PathPlanningHelper::AmrPlanInfo>> PublishAllocationResult(
    const RabbitMQConfig& cfg,
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo* mapInfo,
    long calculation_time_ms = 0,
    double optimization_score = 0.0,
    const std::vector<std::string>& constraints_violated = {},
    const SchedulingRequest* scheduling_request = nullptr,
    const StaticPathTable* staticTable = nullptr,
    int status_code = 0
);

#endif // RESULT_PUBLISHER_H
