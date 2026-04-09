#ifndef AMR_H
#define AMR_H

#include <vector>
#include <string>
#include <queue>
#include <algorithm>
#include "Point.h"  // 引入Point类
#include "Task.h"   // 引入Task类，用于任务队列

// AMR运行状态定义
enum AmrStatus {
    // 任务状态枚举（状态上报消息）：
    // 0=空闲,1=作业中,2=任务正常完成,3=任务失败,4=任务未执行,5=任务暂停,10=不接受任务
    AMR_STATUS_IDLE          = 0,
    AMR_STATUS_WORKING       = 1,
    AMR_STATUS_FINISHED      = 2,
    AMR_STATUS_FAILED        = 3,
    AMR_STATUS_NOT_EXECUTED  = 4,
    AMR_STATUS_PAUSED        = 5,
    AMR_STATUS_REJECTED      = 10
};

class Amr {
private:
    // 任务队列及进度相关
    std::queue<Task> taskQueue_;        // 任务队列
    double currentTaskProgress_;        // 当前任务执行进度（0-1，默认0）
    int priorityThreshold_;             // 优先级阈值（只执行高于该值的任务，默认1）
    std::string taskId;                 // 当前任务id（与队列中当前任务关联）
    int taskProgressPercent;            // 任务进度（百分比，与currentTaskProgress_联动）
    int taskStatus;                     // 任务状态，枚举
    int disThresholdMm_ = 50000;        // 可用判定阈值（mm），缺省15m
    int maxTimeAheadMs_ = 600000;       // detour阈值（毫秒），缺省600000ms（10分钟）
    int maxDetourDistanceMm_ = 0;       // 绕路距离阈值（mm），0 表示不限制

    // 路径与位置相关
    Point nextDestinationPoint;         // 下一个目标点(地图的节点)
    std::vector<Point> curTrailPoints;  // 当前路径段
    std::string curArea;                // 所属区域
    int x;                              // X坐标值，mm
    int y;                              // Y坐标值，mm
    int angle;                          // 角度值（屏幕右为x正方向，上为y正方向，逆时针0~359度）

    // 设备基本信息
    std::string deviceId;               // 设备编号
    int deviceType;                     // 设备类型
    int mapId;                          // 当前所属地图编号
    int deviceHeight;                   // 设备高度，mm
    int deviceWidth;                    // 设备宽度，mm
    int deviceLength;                   // 设备长度，mm

    // 设备状态信息
    bool connection;                    // 是否在线
    int updateTime;                     // 更新时间
    int errorCode;                      // 错误代码
    int speed;                          // 速度，mm/s
    int batteryLevel;                   // 电量百分比，%
    int endurance;                      // 续航时间，小时
    bool load;                          // 是否负载

    // 运动参数相关
    int maxLinearVelocity;              // 最大线速度，mm/s
    int linearAcceleration;             // 直线加速度，mm/s^2
    int linearDeceleration;             // 直线减速度，mm/s^2

    // 新增：来自外部状态的可选字段
    int currentNodeId_ = -1;            // 当前所在节点ID（若提供）
    double estimatedDurationSec_ = 0.0; // 当前任务估计剩余时长（秒），用于展示/诊断
    // 新增：电量上下限阈值（百分比0-100，默认上限80，下限20）
    double batteryUpperLimitPct_ = 80.0; // 目标电量（充电结束目标）
    double batteryLowerLimitPct_ = 20.0; // 最低电量（触发充电）

public:
    // 构造函数，初始化列表顺序与成员声明顺序严格一致（避免警告）
    Amr(
        // 任务队列相关默认值
        int priorityThreshold = 1,
        // 新数据字段默认值
        std::string taskId = "",
        int taskProgressPercent = 0,
        int taskStatus = 0,
        Point nextDestinationPoint = Point(),
        std::vector<Point> curTrailPoints = {},
        std::string curArea = "",
        int x = 0,
        int y = 0,
        int angle = 0,
        std::string deviceId = "",
        int deviceType = 0,
        int mapId = 0,
        int deviceHeight = 0,
        int deviceWidth = 0,
        int deviceLength = 0,
        bool connection = false,
        int updateTime = 0,
        int errorCode = 0,
        int speed = 0,
        int batteryLevel = 100,
        int endurance = 0,
        bool load = false,
        int maxLinearVelocity = 0,
        int linearAcceleration = 0,
        int linearDeceleration = 0
    ) : taskQueue_(),  // 容器类建议显式默认初始化
        currentTaskProgress_(0.0),
        priorityThreshold_(priorityThreshold),
        taskId(taskId),
        taskProgressPercent(taskProgressPercent),
        taskStatus(taskStatus),
        nextDestinationPoint(nextDestinationPoint),
        curTrailPoints(curTrailPoints),
        curArea(curArea),
        x(x),
        y(y),
        angle(angle),
        deviceId(deviceId),
        deviceType(deviceType),
        mapId(mapId),
        deviceHeight(deviceHeight),
        deviceWidth(deviceWidth),
        deviceLength(deviceLength),
        connection(connection),
        updateTime(updateTime),
        errorCode(errorCode),
        speed(speed),
        batteryLevel(batteryLevel),
        endurance(endurance),
        load(load),
        maxLinearVelocity(maxLinearVelocity),
        linearAcceleration(linearAcceleration),
        linearDeceleration(linearDeceleration) {}

    // -------------------------- 任务队列相关 Getter/Setter --------------------------
    // 获取任务队列长度
    size_t getTaskQueueSize() const { return taskQueue_.size(); }
    // 获取当前任务进度（0-1）
    double getCurrentTaskProgress() const { return currentTaskProgress_; }
    // 获取优先级阈值
    int getPriorityThreshold() const { return priorityThreshold_; }

    // 设置当前任务进度（0-1，自动同步到百分比进度）
    void setCurrentTaskProgress(double progress) {
        currentTaskProgress_ = std::max(0.0, std::min(1.0, progress));
        taskProgressPercent = static_cast<int>(currentTaskProgress_ * 100);  // 同步到百分比
    }
    // 设置优先级阈值
    void setPriorityThreshold(int priorityThreshold) { priorityThreshold_ = priorityThreshold; }

    // -------------------------- 任务队列操作方法 --------------------------
    // 添加任务到队列（只添加优先级高于阈值的任务）
    void addTask(const Task& task) {
        if (task.getPriority() >= priorityThreshold_) {
            taskQueue_.push(task);
            // 若队列原本为空，添加后将第一个任务设为当前任务id
            if (taskQueue_.size() == 1) {
                taskId = task.getMessageId();
            }
        }
    }

    // 获取当前任务（队列首元素）
    Task getCurrentTask() const {
        if (!taskQueue_.empty()) {
            return taskQueue_.front();
        } else {
            return Task();  // 空任务（type=-1）
        }
    }

    // 取走所有排队任务（清空队列并返回任务数组）

    // 完成当前任务（移除队列首元素，更新状态）
    Task completeCurrentTask() {
        Task completedTask;
        if (!taskQueue_.empty()) {
            completedTask = taskQueue_.front();
            taskQueue_.pop();
            currentTaskProgress_ = 0.0;  // 重置进度
            taskProgressPercent = 0;
            // 更新当前任务id（若队列非空，设为下一个任务id；否则清空）
            taskId = taskQueue_.empty() ? "" : taskQueue_.front().getMessageId();
        }
        return completedTask;
    }

    // 清空任务队列
    void clearTaskQueue() {
        while (!taskQueue_.empty()) {
            taskQueue_.pop();
        }
        currentTaskProgress_ = 0.0;
        taskProgressPercent = 0;
        taskId = "";  // 清空当前任务id
    }

    // 取走所有排队任务（清空队列并返回任务数组），不影响当前任务
    std::vector<Task> stealAllQueuedTasks() {
        std::vector<Task> out;
        while (!taskQueue_.empty()) { out.push_back(taskQueue_.front()); taskQueue_.pop(); }
        return out;
    }

    // 仅保留队列中的第一个任务（下一待执行任务），其余任务回滚到列表
    std::vector<Task> stealQueuedTasksExceptFirst() {
        std::vector<Task> out;
        if (taskQueue_.empty()) return out;
        // 保留第一个
        Task keep = taskQueue_.front();
        taskQueue_.pop();
        // 回滚其余
        while (!taskQueue_.empty()) { out.push_back(taskQueue_.front()); taskQueue_.pop(); }
        // 仅保留第一个
        taskQueue_.push(keep);
        return out;
    }

    // -------------------------- 新数据字段 Getter/Setter --------------------------
    std::string getTaskId() const { return taskId; }
    Point getNextDestinationPoint() const { return nextDestinationPoint; }
    int getNextDestinationPointId() const { return nextDestinationPoint.getNodeId(); }
    const std::vector<Point>& getCurTrailPoints() const { return curTrailPoints; }
    std::string getCurArea() const { return curArea; }
    int getTaskProgressPercent() const { return taskProgressPercent; }
    int getUpdateTime() const { return updateTime; }
    int getErrorCode() const { return errorCode; }
    std::string getDeviceId() const { return deviceId; }
    int getDeviceType() const { return deviceType; }
    int getMapId() const { return mapId; }
    bool isConnection() const { return connection; }
    int getX() const { return x; }
    int getY() const { return y; }
    int getAngle() const { return angle; }
    int getSpeed() const { return speed; }
    int getTaskStatus() const { return taskStatus; }
    // AMR状态便捷判断（基于任务状态枚举）
    // 空闲：taskStatus==0 或 2，且 taskId 为空才视为可分配
    bool isIdle() const {
        return taskId.empty() &&
               (taskStatus == AMR_STATUS_IDLE ||
                taskStatus == AMR_STATUS_FINISHED);
    }
    // 作业中：仅 taskStatus==1
    bool isWorking() const { return taskStatus == AMR_STATUS_WORKING; }
    // 移动中：等价于“作业中”
    bool isMoving() const { return taskStatus == AMR_STATUS_WORKING; }
    // 不接受任务：仅 taskStatus==10
    bool isRejectingTasks() const { return taskStatus == AMR_STATUS_REJECTED; }
    // 不可用：失败/未执行/暂停/不接受任务（不参与任务分配）
    bool isUnavailable() const {
        return taskStatus == AMR_STATUS_FAILED ||
               taskStatus == AMR_STATUS_NOT_EXECUTED ||
               taskStatus == AMR_STATUS_PAUSED ||
               taskStatus == AMR_STATUS_REJECTED;
    }
    // 可用判定阈值（mm）
    int getDisThresholdMm() const { return disThresholdMm_; }
    void setDisThresholdMm(int v) { disThresholdMm_ = v; }
    // detour阈值（秒）
    int getMaxTimeAheadMs() const { return maxTimeAheadMs_; }
    void setMaxTimeAheadMs(int v) { maxTimeAheadMs_ = v; }
    // 绕路距离阈值（mm）
    int getMaxDetourDistanceMm() const { return maxDetourDistanceMm_; }
    void setMaxDetourDistanceMm(int v) { maxDetourDistanceMm_ = v; }
    int getBatteryLevel() const { return batteryLevel; }
    int getDeviceHeight() const { return deviceHeight; }
    int getDeviceWidth() const { return deviceWidth; }
    int getDeviceLength() const { return deviceLength; }
    int getEndurance() const { return endurance; }
    bool isLoad() const { return load; }
    int getMaxLinearVelocity() const { return maxLinearVelocity; }
    int getLinearAcceleration() const { return linearAcceleration; }
    int getLinearDeceleration() const { return linearDeceleration; }

    // 新增 Getter/Setter
    int getCurrentNodeId() const { return currentNodeId_; }
    void setCurrentNodeId(int v) { currentNodeId_ = v; }
    double getEstimatedDurationSec() const { return estimatedDurationSec_; }
    void setEstimatedDurationSec(double v) { estimatedDurationSec_ = v; }
    // 新增：电量阈值 Getter/Setter
    double getBatteryUpperLimitPct() const { return batteryUpperLimitPct_; }
    double getBatteryLowerLimitPct() const { return batteryLowerLimitPct_; }
    void setBatteryUpperLimitPct(double v) {
        if (v < 0.0) v = 0.0;
        if (v > 100.0) v = 100.0;
        batteryUpperLimitPct_ = v;
    }
    void setBatteryLowerLimitPct(double v) {
        if (v < 0.0) v = 0.0;
        if (v > 100.0) v = 100.0;
        batteryLowerLimitPct_ = v;
    }

    void setTaskId(const std::string& taskId) { this->taskId = taskId; }
    void setNextDestinationPoint(const Point& nextDestinationPoint) { this->nextDestinationPoint = nextDestinationPoint; }
    void setCurTrailPoints(const std::vector<Point>& curTrailPoints) { this->curTrailPoints = curTrailPoints; }
    void setCurArea(const std::string& curArea) { this->curArea = curArea; }
    void setTaskProgressPercent(int progress) { 
        taskProgressPercent = std::max(0, std::min(100, progress));
        currentTaskProgress_ = taskProgressPercent / 100.0;  // 同步到0-1进度
    }
    void setUpdateTime(int updateTime) { this->updateTime = updateTime; }
    void setErrorCode(int errorCode) { this->errorCode = errorCode; }
    void setDeviceId(const std::string& deviceId) { this->deviceId = deviceId; }
    void setDeviceType(int deviceType) { this->deviceType = deviceType; }
    void setMapId(int mapId) { this->mapId = mapId; }
    void setConnection(bool connection) { this->connection = connection; }
    void setX(int x) { this->x = x; }
    void setY(int y) { this->y = y; }
    void setAngle(int angle) { this->angle = angle; }
    void setSpeed(int speed) { this->speed = speed; }
    void setTaskStatus(int taskStatus) { this->taskStatus = taskStatus; }
    // setDisThresholdMm 已在前面定义，避免重复
    void setBatteryLevel(int batteryLevel) { this->batteryLevel = std::max(0, std::min(100, batteryLevel)); }
    void setDeviceHeight(int deviceHeight) { this->deviceHeight = deviceHeight; }
    void setDeviceWidth(int deviceWidth) { this->deviceWidth = deviceWidth; }
    void setDeviceLength(int deviceLength) { this->deviceLength = deviceLength; }
    void setEndurance(int endurance) { this->endurance = endurance; }
    void setLoad(bool load) { this->load = load; }
    void setMaxLinearVelocity(int maxLinearVelocity) { this->maxLinearVelocity = maxLinearVelocity; }
    void setLinearAcceleration(int linearAcceleration) { this->linearAcceleration = linearAcceleration; }
    void setLinearDeceleration(int linearDeceleration) { this->linearDeceleration = linearDeceleration; }

    // -------------------------- 显示AMR信息（含任务队列） --------------------------
    void displayInfo() const {
        printf("AMR 设备编号: %s\n", deviceId.c_str());
        printf("  设备类型: %d, 所属地图ID: %d, 所属区域: %s\n", deviceType, mapId, curArea.c_str());
        printf("  在线状态: %s, 错误代码: %d, 最后更新时间: %d\n", 
               connection ? "在线" : "离线", errorCode, updateTime);
        printf("  坐标: (%d, %d) mm, 角度: %d°  |  速度: %d mm/s\n", x, y, angle, speed);
        printf("  运动参数: 最大线速度=%d mm/s, 加速度=%d mm/s, 减速度=%d mm/s\n",
               maxLinearVelocity, linearAcceleration, linearDeceleration);
        printf("  设备尺寸: 长%d × 宽%d × 高%d mm  |  负载状态: %s\n",
               deviceLength, deviceWidth, deviceHeight, load ? "负载" : "空载");
        printf("  电量: %d%%, 续航时间: %d 小时\n", batteryLevel, endurance);
        printf("  任务状态: %d  |  当前任务ID: %s  |  任务进度: %d%%\n",
               taskStatus, taskId.c_str(), taskProgressPercent);
        printf("  任务队列: 共%d个任务  |  优先级阈值: %d\n", 
               static_cast<int>(taskQueue_.size()), priorityThreshold_);
        printf("  下一个目标点:\n");
        nextDestinationPoint.displayInfo();
        printf("  当前路径段点数: %zu\n", curTrailPoints.size());
    }
};

#endif // AMR_H
