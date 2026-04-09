// SchedulingRequest.h
#ifndef SCHEDULING_REQUEST_H
#define SCHEDULING_REQUEST_H

#include <string>
#include <vector>

// 说明：用于承载一次调度请求的元数据（请求级信息，不属于单个Task）
struct SchedulingRequest {
    // 基本标识
    std::string messageId;          // 对齐字段：schedulingRequestId
    std::string requestTimestamp;   // ISO8601 字符串
    int requestType = -1;           // 请求类型：0=批量，1=单任务，2=紧急调度

    // 规模与超时
    int taskNumber = 0;             // 批量请求的任务数量
    int timeoutMs = 2000;              // 算法计算超时时间

    // 算法选择（可选）
    std::string allocationAlgorithm; // 例：posta / greedy / mlp

    // 触发上下文
    std::string triggerContext;     // 触发类型（AGV空闲/新增任务/周期检查等）
    std::vector<std::string> triggerAgvId; // 当触发类型与AGV相关时携带设备ID

    // 批量配置
    int batchConfig = -1;           // 单次最多处理任务数量上限（-1 表示未设置）

    // 环境上下文
    int mapId = 0;                  // 本次请求目标地图ID（0 表示未指定）
    std::string mapVersion;         // 地图版本
    int systemLoad = 0;             // 系统任务积压
    double systemLoadFactor = 0.0;  // 负载因子（0-1）
};

#endif // SCHEDULING_REQUEST_H
