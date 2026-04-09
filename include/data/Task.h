#ifndef TASK_H
#define TASK_H

#include <string>
#include <vector>
#include "SubTask.h"

// 先声明Point类，用于Task类中的任务点集合
class Point;

class Task {
private:
    int priority;                 // 任务优先级
    int taskType;                 // 任务类型（数值型，-1 表示未知/未设置）
    int expectedCompletionTime;   // 期望完成时间（保留旧字段，单位与历史一致）
    std::string messageId;        // 消息Id，用于反馈结果时校验
    std::vector<std::string> deviceIds; // 可分配设备编号
    int timeout;                  // 分配超时时间，ms
    // 子任务：单点作业（sequence/pointType/location/point/duration/constraints/maxSpeed）
    std::vector<SubTask> subTasks;
    bool load;                    // 是否负载，表示是空载任务还是负载任务
    int mapId;                    // 地图Id，表示在那个地图上进行分配   `
    int minBatteryLevel = 0;      // 任务要求的最低电量（百分比，0-100），0表示不限制

    // 时间戳/时间信息（ISO8601，按需填写）
    std::string expectedStartTimeISO;       // 预计开始时间（字符串）
    std::string expectedCompletionTimeISO;  // 预计完成时间（字符串）
    std::string createTimestampISO;         // 任务创建/触发时间戳（若有）

public:
    // 构造函数，按新数据结构初始化字段
    Task(
        int priority = 0,
        int taskType = -1,
        int expectedCompletionTime = 0,
        const std::string& messageId = "",
        const std::vector<std::string>& deviceIds = {},
        int timeout = 0,
        const std::vector<SubTask>& subTasks = {},
        bool load = false,
        int mapId = 0,
        int minBatteryLevel = 0,
        const std::string& expectedStartTimeISO = "",
        const std::string& expectedCompletionTimeISO = "",
        const std::string& createTimestampISO = ""
    ) : priority(priority), taskType(taskType), expectedCompletionTime(expectedCompletionTime),
        messageId(messageId), deviceIds(deviceIds), timeout(timeout), subTasks(subTasks),
        load(load), mapId(mapId), minBatteryLevel(minBatteryLevel),
        expectedStartTimeISO(expectedStartTimeISO), expectedCompletionTimeISO(expectedCompletionTimeISO),
        createTimestampISO(createTimestampISO) {}

    // -------------------------- Getter方法 --------------------------
    int getPriority() const { return priority; }
    int getTaskType() const { return taskType; }
    int getExpectedCompletionTime() const { return expectedCompletionTime; }
    const std::string& getMessageId() const { return messageId; }
    const std::vector<std::string>& getDeviceIds() const { return deviceIds; }
    int getTimeout() const { return timeout; }
    // 新增：获取/设置 SubTask 列表
    const std::vector<SubTask>& getSubTasks() const { return subTasks; }
    void setSubTasks(const std::vector<SubTask>& v) { subTasks = v; }
    bool isLoad() const { return load; }
    int getMapId() const { return mapId; }
    int getMinBatteryLevel() const { return minBatteryLevel; }
    const std::string& getExpectedStartTimeISO() const { return expectedStartTimeISO; }
    const std::string& getExpectedCompletionTimeISO() const { return expectedCompletionTimeISO; }
    const std::string& getCreateTimestampISO() const { return createTimestampISO; }
    // 兼容旧接口：将“任务参考节点”定义为第一个子任务的 point.nodeId
    int getStartId() const {
        for (const auto& st : subTasks) {
            int nid = st.getPoint().getNodeId();
            if (nid >= 0) return nid;
        }
        return subTasks.empty() ? -1 : subTasks.front().getPoint().getNodeId();
    }
    int getEndId() const {
        for (auto it = subTasks.rbegin(); it != subTasks.rend(); ++it) {
            int nid = it->getPoint().getNodeId();
            if (nid >= 0) return nid;
        }
        return subTasks.empty() ? -1 : subTasks.front().getPoint().getNodeId();
    }
    // 便捷方法：获取第一个子任务的预计作业时间
    std::int64_t getFirstOpDurationMs() const { return subTasks.empty() ? 0 : subTasks.front().getEstimatedDurationMs(); }

    // -------------------------- Setter方法 --------------------------
    void setPriority(int priority) { this->priority = priority; }
    void setTaskType(int taskType) { this->taskType = taskType; }
    void setExpectedCompletionTime(int expectedCompletionTime) { this->expectedCompletionTime = expectedCompletionTime; }
    void setMessageId(const std::string& messageId) { this->messageId = messageId; }
    void setDeviceIds(const std::vector<std::string>& deviceIds) { this->deviceIds = deviceIds; }
    void setTimeout(int timeout) { this->timeout = timeout; }
    // 不再保留 taskPoints/segmentDurationsSec
    void setLoad(bool load) { this->load = load; }
    void setMapId(int mapId) { this->mapId = mapId; }
    void setMinBatteryLevel(int v) { this->minBatteryLevel = std::max(0, std::min(100, v)); }
    void setExpectedStartTimeISO(const std::string& v) { this->expectedStartTimeISO = v; }
    void setExpectedCompletionTimeISO(const std::string& v) { this->expectedCompletionTimeISO = v; }
    void setCreateTimestampISO(const std::string& v) { this->createTimestampISO = v; }

    // -------------------------- 显示任务信息 --------------------------
    void displayInfo() const {
        printf("任务信息:\n");
        printf("  优先级: %d\n", priority);
        printf("  任务类型: %d\n", taskType);
        printf("  期望完成时间: %d\n", expectedCompletionTime);
        printf("  消息Id: %s\n", messageId.c_str());
        printf("  可分配设备数量: %zu\n", deviceIds.size());
        printf("  分配超时时间: %d ms\n", timeout);
        printf("  子任务段数量: %zu\n", subTasks.size());
        printf("  是否负载: %s\n", load ? "是" : "否");
        printf("  地图Id: %d\n", mapId);
        
        if (!expectedStartTimeISO.empty()) printf("  预计开始: %s\n", expectedStartTimeISO.c_str());
        if (!expectedCompletionTimeISO.empty()) printf("  预计完成: %s\n", expectedCompletionTimeISO.c_str());
        if (!createTimestampISO.empty()) printf("  创建时间: %s\n", createTimestampISO.c_str());
    }
};


#endif // TASK_H
