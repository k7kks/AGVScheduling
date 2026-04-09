#ifndef SUBTASK_H
#define SUBTASK_H

#include <string>
#include <vector>
#include <cstdint>
#include "data/Point.h"

// 子任务：单个作业点（不再是起止对）
class SubTask {
private:
    int sequence;                       // 顺序
    int pointType;                      // 操作类型：PICKUP / DROPOFF / CHARGE 等（整型编码）
    std::string location;               // 区域编号（无分区则为""）
    Point point;                        // 位置（x,y,angle,nodeId）
    std::int64_t estimatedDurationMs;   // 预计作业时间（毫秒）
    std::vector<std::string> pathConstraints; // 需绕行区域列表
    int maxSpeed;                       // 子任务最大行驶速度限制
    std::string subTaskId;              // 子任务唯一标识（用于跨能力联动）

public:
    SubTask(int sequence = 1,
            int pointType = -1,
            const std::string& location = "",
            const Point& point = Point(),
            std::int64_t estimatedDurationMs = 0,
            const std::vector<std::string>& pathConstraints = {},
            int maxSpeed = 0,
            const std::string& subTaskId = std::string())
        : sequence(sequence), pointType(pointType), location(location), point(point),
          estimatedDurationMs(estimatedDurationMs), pathConstraints(pathConstraints), maxSpeed(maxSpeed),
          subTaskId(subTaskId) {}

    int getSequence() const { return sequence; }
    int getPointType() const { return pointType; }
    const std::string& getLocation() const { return location; }
    const Point& getPoint() const { return point; }
    std::int64_t getEstimatedDurationMs() const { return estimatedDurationMs; }
    double getEstimatedDurationSeconds() const { return static_cast<double>(estimatedDurationMs) / 1000.0; }
    const std::vector<std::string>& getPathConstraints() const { return pathConstraints; }
    int getMaxSpeed() const { return maxSpeed; }
    const std::string& getSubTaskId() const { return subTaskId; }

    void setSequence(int v) { sequence = v; }
    void setPointType(int v) { pointType = v; }
    void setLocation(const std::string& v) { location = v; }
    void setPoint(const Point& v) { point = v; }
    void setEstimatedDurationMs(std::int64_t v) { estimatedDurationMs = v; }
    void setPathConstraints(const std::vector<std::string>& v) { pathConstraints = v; }
    void setMaxSpeed(int v) { maxSpeed = v; }
    void setSubTaskId(const std::string& v) { subTaskId = v; }
};

#endif // SUBTASK_H
