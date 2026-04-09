# AGV系统细节问题分析文档

## 问题1：两台车的简单场景是否没问题？

### 1.1 简单场景定义

```
场景：两台AGV（AGV_A、AGV_B）执行不相交的任务

AGV_A 任务：1→2→3→4
AGV_B 任务：5→6→7→8

地图：
┌─────────┐     ┌─────────┐
│ Node 1  │─────│ Node 2  │
└─────────┘     └─────────┘
    │               │
    ▼               ▼
┌─────────┐     ┌─────────┐
│ Node 3  │─────│ Node 4  │
└─────────┘     └─────────┘

┌─────────┐     ┌─────────┐
│ Node 5  │─────│ Node 6  │
└─────────┘     └─────────┘
    │               │
    ▼               ▼
┌─────────┐     ┌─────────┐
│ Node 7  │─────│ Node 8  │
└─────────┘     └─────────┘
```

### 1.2 执行流程

```
时间轴：

t=0s: Fast阶段
  AGV_A: 规划 1→2→3→4，预约 1,2,3,4，发送轨迹 1→2→3→4
  AGV_B: 规划 5→6→7→8，预约 5,6,7,8，发送轨迹 5→6→7→8

t=1s: AGV_A 到达节点2，AGV_B 到达节点6
  AGV_A: 释放节点1，新增预约节点5（不存在或不需要）
  AGV_B: 释放节点5，新增预约节点9（不存在）

t=2s: AGV_A 到达节点3，AGV_B 到达节点7
  AGV_A: 释放节点2，继续执行
  AGV_B: 释放节点6，继续执行

t=3s: AGV_A 到达节点4，AGV_B 到达节点8
  AGV_A: 完成任务，释放所有预约
  AGV_B: 完成任务，释放所有预约

结果：✅ 完全没问题！两台车互不干扰
```

### 1.3 结论

**✅ 两台车的简单场景完全没问题**，因为：
1. 任务不相交，无预约冲突
2. 预约和释放机制正常工作
3. 轨迹发送和执行顺序正确

---

## 问题2：细节问题分析

### 2.1 静止判定机制

**配置**（第47行）：
```bash
STATUS_STATIC_SPEED_EPS="${STATUS_STATIC_SPEED_EPS:-20}"   # 静止判定速度阈值（mm/s）
```

**静止判定逻辑**：

根据代码分析，静止判定涉及以下几个时间点：

#### 2.1.1 `staticSince` - 静止开始时间

**位置**：第6280-6284行
```cpp
long long staticMs = 0;
if (rt.staticSince != std::chrono::steady_clock::time_point{}) {
    staticMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - rt.staticSince).count();
}
const bool staticLong = (staticMs >= majorStuckMs);  // majorStuckMs = 2000ms
```

**含义**：
- 记录AGV开始静止的时间
- 当静止时间 >= `REPLAN_MAJOR_STUCK_MS`（默认2000ms）时，触发Major重规划

#### 2.1.2 `stuckSince` - 卡住开始时间

**位置**：第6878-6898行（状态更新处理）

```cpp
// 当AGV位置不在预期轨迹上时
if (it == trimmed.end()) {
    // AGV不在预期路径上 → 卡住
    rt.stuckSince = now;
    rt.stuckNodeId = curNodeId;
}
```

**含义**：
- 记录AGV偏离预期轨迹的时间
- 用于检测AGV是否真的被卡住

#### 2.1.3 `blockedSince` - 被阻塞开始时间

**位置**：第5834-5842行（重规划失败处理）

```cpp
if (plannedOk) {
    rt.blockedSince = std::chrono::steady_clock::time_point{};  // 清除
} else {
    // 规划失败 → 被阻塞
    rt.blockedSince = now;
}
```

**含义**：
- 记录规划失败导致AGV被阻塞的时间
- 用于判断是否需要更激进的重规划

### 2.1.4 静止判定的完整流程

```
AGV状态更新（收到status消息）
    ↓
检查AGV速度是否 <= STATUS_STATIC_SPEED_EPS（20mm/s）
    ↓
    ├─ 速度 > 20mm/s：AGV在运动
    │   └─ 清除 staticSince
    │
    └─ 速度 <= 20mm/s：AGV静止
        └─ 如果 staticSince 未设置
            └─ 设置 staticSince = now
        └─ 计算静止时间 = now - staticSince
            └─ 如果 静止时间 >= 2000ms
                └─ 触发 Major 重规划
```

### 2.1.5 问题分析

**问题A：速度阈值可能不准确**
```
STATUS_STATIC_SPEED_EPS = 20mm/s

场景：AGV以19mm/s的速度缓慢移动
结果：被判定为静止，触发重规划
问题：实际上AGV在移动，不应该重规划
```

**建议**：
- 增加速度阈值（例如50mm/s）
- 或者同时检查位置变化

**问题B：静止时间阈值固定**
```
REPLAN_MAJOR_STUCK_MS = 2000ms

场景1：AGV在节点停留1秒（正常作业）
结果：不触发重规划 ✓

场景2：AGV在节点停留2.1秒（可能被阻塞）
结果：立即触发重规划
问题：可能过于敏感，导致频繁重规划
```

**建议**：
- 增加阈值（例如5000ms）
- 或者根据任务类型动态调整

---

### 2.2 资源预约释放机制

#### 2.2.1 预约释放的三种方式

**方式1：正常释放（状态更新时）**

**位置**：第6885-6888行
```cpp
auto it = std::find(trimmed.begin(), trimmed.end(), curNodeId);
if (it != trimmed.end()) {
    for (auto jt = trimmed.begin(); jt != it; ++jt) {
        reservations.release(*jt, entry.deviceId);  // 释放已通过的节点
    }
    trimmed.erase(trimmed.begin(), it);
}
```

**流程**：
```
AGV状态更新：currentNode = 3
预约轨迹：1→2→3→4→5

1. 找到节点3在轨迹中的位置
2. 释放节点1、2的预约
3. 保留节点3、4、5的预约
4. 更新轨迹：3→4→5
```

**方式2：空闲释放（任务完成时）**

**位置**：第6844-6873行
```cpp
if (idleReleaseNow) {
    std::vector<int> keepNodes;
    keepNodes.push_back(curNodeId);
    reservations.releaseAllByOwnerExceptSet(entry.deviceId, keepNodes);  // 释放所有，只保留当前节点
    reservations.tryReserve(curNodeId,
                             entry.deviceId,
                             NodeReservationTable::HoldReason::WAITING_POINT,
                             "idle",
                             std::chrono::milliseconds(0));
}
```

**流程**：
```
AGV任务完成，进入空闲状态
预约轨迹：3→4→5

1. 释放所有预约（3、4、5）
2. 只保留当前节点（例如5）
3. 标记为 WAITING_POINT（等待点）
4. 预约TTL = 0（立即过期）
```

**方式3：Super阶段释放（重规划时）**

**位置**：第4996-5006行
```cpp
if (stage == ReplanStage::SUPER) {
    std::vector<int> keepNodes;
    keepNodes.push_back(startNodeId);
    reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);  // 释放所有，只保留起点
    rt.reservedNodes = keepNodes;
}
```

**流程**：
```
触发Super重规划
预约轨迹：2→3→4→5→6

1. 释放所有预约（2、3、4、5、6）
2. 只保留当前节点（例如2）
3. 重新规划新路径
4. 预约新路径
```

#### 2.2.2 预约释放的问题

**问题A：TTL过期导致预约自动释放**

```
配置：RESERVE_COMMITTED_TTL_MS = 0（已提交节点TTL为0）
      RESERVE_TTL_MS = 15000（预约节点TTL为15秒）

场景：AGV在节点3停留超过15秒

时间轴：
t=0s:   预约轨迹：1→2→3→4→5（TTL=15秒）
t=15s:  节点1的预约TTL过期 → 自动释放
t=15s:  节点2的预约TTL过期 → 自动释放
t=15s:  节点3的预约TTL过期 → 自动释放
t=15s:  节点4的预约TTL过期 → 自动释放
t=15s:  节点5的预约TTL过期 → 自动释放

问题：AGV仍在节点3，但预约已全部释放！
      其他AGV可以进入节点3 → 碰撞！
```

**根本原因**：
- TTL是固定的15秒
- 如果AGV停留超过15秒，预约失效
- 系统无法区分"正常停留"和"被阻塞"

**建议修复**：
```cpp
// 改进的TTL计算
int calculateDynamicTTL(const Amr& agv, int nodeIndex, int totalNodes) {
    // TTL应该根据AGV到达该节点的预计时间动态计算
    // 而不是固定的15秒

    double distanceToNode = calculateDistance(agv.currentNode, nodeIndex);
    double timeToNode = distanceToNode / agv.speed;

    // TTL = 到达时间 + 停留时间 + 缓冲
    int ttl = static_cast<int>(timeToNode * 1000) + 5000 + 5000;  // +5秒停留 +5秒缓冲

    return ttl;
}
```

**问题B：状态更新延迟导致预约不同步**

```
配置：网络延迟 = 500ms

场景：
t=0s:   AGV_A 发送状态：currentNode=2
t=0.5s: 系统收到状态（延迟500ms）
        释放节点1，保留节点2、3、4、5

问题：如果状态消息丢失会怎样？
t=1s:   AGV_A 实际位置：节点3
        AGV_A 尝试发送状态，但网络故障
        消息丢失！

t=1.5s: 系统仍认为AGV_A在节点2
        预约仍为：2、3、4、5

t=2s:   AGV_B 规划路径：2→3→4
        尝试预约节点2、3、4
        全部被AGV_A预约！
        AGV_B 无法执行 → 阻塞

t=3s:   AGV_A 状态消息恢复
        发送状态：currentNode=3
        系统更新：AGV_A在节点3
        释放节点2，保留节点3、4、5

t=3.5s: AGV_B 重新规划
        预约节点2、3、4 → 成功

结果：AGV_B被延迟1.5秒
```

**根本原因**：
- 预约表依赖状态消息更新
- 网络延迟或消息丢失导致不同步

**建议修复**：
```cpp
// 实现状态同步检查
class StatusSyncChecker {
private:
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> lastStatusTime_;
    static constexpr auto STATUS_TIMEOUT = std::chrono::seconds(5);

public:
    bool isStatusStale(const std::string& agvId) {
        auto it = lastStatusTime_.find(agvId);
        if (it == lastStatusTime_.end()) {
            return true;  // 从未收到状态
        }

        auto age = std::chrono::steady_clock::now() - it->second;
        return age > STATUS_TIMEOUT;  // 状态超过5秒未更新
    }

    void handleStaleStatus(const std::string& agvId,
                          NodeReservationTable& reservations) {
        if (isStatusStale(agvId)) {
            // 状态过期，释放所有预约
            reservations.releaseAllByOwner(agvId);

            // 记录警告
            log_warning("Status stale for AGV: " + agvId);
        }
    }
};
```

---

## 问题3：任务取消时的预约释放

### 3.1 当前系统的问题

**问题描述**：
```
场景：上层系统（业务系统）取消了AGV_A的任务

当前状态：
- AGV_A 正在执行任务：1→2→3→4→5
- 预约轨迹：2→3→4→5
- 预约TTL：15秒

上层系统发送取消消息：
{
  "messageId": "cancel_task_001",
  "taskId": "task_A",
  "deviceId": "agv_001",
  "action": "cancel"
}

问题：
1. 调度系统收到取消消息后，应该做什么？
2. 预约如何释放？
3. AGV如何停止执行？
4. 其他AGV如何知道节点已释放？
```

### 3.2 预约释放的三个关键问题

#### 3.2.1 问题：预约何时释放？

**当前机制**：
- 预约通过TTL自动过期（15秒）
- 或者通过状态更新手动释放
- 或者通过空闲释放手动释放

**问题**：
- 如果任务被取消，预约不会立即释放
- 需要等待TTL过期（最多15秒）
- 其他AGV在这15秒内无法使用这些节点

**示例**：
```
t=0s:   AGV_A 预约轨迹：2→3→4→5（TTL=15秒）
t=5s:   上层系统取消任务
        调度系统收到取消消息
        但预约仍然有效！

t=5.1s: AGV_B 规划路径：2→3→4
        尝试预约节点2、3、4
        全部被AGV_A预约！
        AGV_B 无法执行

t=15s:  AGV_A的预约TTL过期
        预约自动释放

t=15.1s: AGV_B 重新规划
        预约节点2、3、4 → 成功

结果：AGV_B被延迟10秒
```

#### 3.2.2 问题：如何通知AGV停止执行？

**当前机制**：
- 调度系统无法直接通知AGV停止
- AGV继续执行已发送的轨迹
- 直到收到新的轨迹或状态更新

**问题**：
- AGV可能继续执行已取消的任务
- 导致资源浪费

**示例**：
```
t=0s:   发送轨迹给AGV_A：1→2→3→4→5
t=5s:   上层系统取消任务
        调度系统发送取消消息
        但AGV_A已经收到轨迹，继续执行

t=6s:   AGV_A 到达节点2
t=7s:   AGV_A 到达节点3
t=8s:   AGV_A 到达节点4
t=9s:   AGV_A 到达节点5
        任务完成

结果：AGV_A执行了已取消的任务
```

#### 3.2.3 问题：其他AGV如何知道节点已释放？

**当前机制**：
- 其他AGV通过定期重规划检查预约
- 如果预约释放，重规划会成功
- 但需要等待重规划周期（200ms）

**问题**：
- 其他AGV无法立即知道节点已释放
- 需要等待下一个重规划周期
- 导致延迟

**示例**：
```
t=0s:   AGV_A 预约轨迹：2→3→4→5
        AGV_B 规划路径：2→3→4
        预约失败，等待重规划

t=0.2s: 第一个重规划周期
        AGV_B 重新规划
        预约仍然失败（AGV_A预约仍有效）

t=5s:   上层系统取消AGV_A的任务
        调度系统释放AGV_A的预约

t=5.2s: 第一个重规划周期
        AGV_B 重新规划
        预约成功！

结果：AGV_B被延迟200ms
```

### 3.3 建议的解决方案

#### 方案1：立即释放预约

**实现**：
```cpp
// 在调度系统中添加任务取消处理

class TaskCancellationHandler {
private:
    NodeReservationTable& reservations_;
    RobotDataRepository& repo_;

public:
    void handleTaskCancellation(const std::string& taskId,
                               const std::string& deviceId) {
        // 1. 立即释放AGV的所有预约
        reservations_.releaseAllByOwner(deviceId);

        // 2. 清除AGV的运行时状态
        repo_.clearDynamicPlan(deviceId);
        repo_.updateLastSentRoute(deviceId, {});

        // 3. 记录日志
        log_info("Task cancelled: taskId=" + taskId +
                 ", deviceId=" + deviceId +
                 ", reservations released");

        // 4. 发送通知给其他系统
        notifyTaskCancellation(taskId, deviceId);
    }

private:
    void notifyTaskCancellation(const std::string& taskId,
                               const std::string& deviceId) {
        // 发送消息给业务系统、AGV等
        json notification;
        notification["messageId"] = generate_uuid();
        notification["type"] = "task_cancelled";
        notification["taskId"] = taskId;
        notification["deviceId"] = deviceId;
        notification["timestamp"] = iso8601_utc_now();

        // 发布到MQ
        publishNotification(notification);
    }
};
```

**优点**：
- 预约立即释放
- 其他AGV可以立即使用这些节点
- 响应快速

**缺点**：
- 需要修改系统架构
- 需要处理AGV已经在执行的轨迹

#### 方案2：发送停止轨迹

**实现**：
```cpp
// 向AGV发送空轨迹或停止轨迹

class StopTrailPublisher {
public:
    void publishStopTrail(const std::string& deviceId) {
        json stopTrail;
        stopTrail["messageId"] = generate_uuid();
        stopTrail["deviceId"] = deviceId;
        stopTrail["trailPoints"] = json::array();  // 空轨迹
        stopTrail["action"] = "stop";
        stopTrail["timestamp"] = iso8601_utc_now();

        // 发送给AGV
        algoPublisher_.sendTrailResponse(stopTrail);

        log_info("Stop trail sent to AGV: " + deviceId);
    }
};
```

**优点**：
- AGV收到停止信号后立即停止
- 清晰的停止语义

**缺点**：
- 需要AGV支持停止轨迹
- 需要等待AGV收到消息

#### 方案3：标记任务为已取消

**实现**：
```cpp
// 在任务中添加取消标记

class TaskCancellationTracker {
private:
    std::unordered_set<std::string> cancelledTasks_;
    std::mutex mutex_;

public:
    void markTaskCancelled(const std::string& taskId) {
        std::lock_guard<std::mutex> lk(mutex_);
        cancelledTasks_.insert(taskId);
    }

    bool isTaskCancelled(const std::string& taskId) {
        std::lock_guard<std::mutex> lk(mutex_);
        return cancelledTasks_.find(taskId) != cancelledTasks_.end();
    }

    void handleCancelledTask(const std::string& taskId,
                            const std::string& deviceId,
                            NodeReservationTable& reservations) {
        if (isTaskCancelled(taskId)) {
            // 任务已取消，释放预约
            reservations.releaseAllByOwner(deviceId);

            // 清除取消标记
            std::lock_guard<std::mutex> lk(mutex_);
            cancelledTasks_.erase(taskId);
        }
    }
};
```

**优点**：
- 简单易实现
- 不需要修改AGV通信

**缺点**：
- 预约释放延迟
- 需要定期检查

### 3.4 推荐的完整解决方案

**综合方案**：结合方案1和方案2

```cpp
class ComprehensiveTaskCancellationHandler {
private:
    NodeReservationTable& reservations_;
    RobotDataRepository& repo_;
    AlgoPublisher& algoPublisher_;

public:
    void handleTaskCancellation(const std::string& taskId,
                               const std::string& deviceId) {
        // 步骤1：立即释放预约
        reservations_.releaseAllByOwner(deviceId);
        log_info("Reservations released for AGV: " + deviceId);

        // 步骤2：清除运行时状态
        repo_.clearDynamicPlan(deviceId);
        repo_.updateLastSentRoute(deviceId, {});
        log_info("Runtime state cleared for AGV: " + deviceId);

        // 步骤3：发送停止轨迹给AGV
        json stopTrail;
        stopTrail["messageId"] = generate_uuid();
        stopTrail["deviceId"] = deviceId;
        stopTrail["trailPoints"] = json::array();  // 空轨迹
        stopTrail["action"] = "stop";
        stopTrail["timestamp"] = iso8601_utc_now();
        algoPublisher_.sendTrailResponse(stopTrail);
        log_info("Stop trail sent to AGV: " + deviceId);

        // 步骤4：发送通知
        json notification;
        notification["messageId"] = generate_uuid();
        notification["type"] = "task_cancelled";
        notification["taskId"] = taskId;
        notification["deviceId"] = deviceId;
        notification["timestamp"] = iso8601_utc_now();
        publishNotification(notification);

        log_info("Task cancellation completed: taskId=" + taskId +
                 ", deviceId=" + deviceId);
    }
};
```

**执行流程**：
```
上层系统发送取消消息
    ↓
调度系统接收取消消息
    ↓
立即释放AGV的所有预约
    ↓
清除AGV的运行时状态
    ↓
发送停止轨迹给AGV
    ↓
发送通知给其他系统
    ↓
其他AGV可以立即使用释放的节点
```

---

## 总结

### 问题1：两台车的简单场景
✅ **完全没问题**，预约和释放机制正常工作

### 问题2：细节问题

| 细节 | 问题 | 建议 |
|------|------|------|
| 静止判定 | 速度阈值可能不准确 | 增加阈值或同时检查位置变化 |
| 静止时间 | 阈值固定，可能过于敏感 | 增加阈值或动态调整 |
| TTL过期 | 固定15秒，可能导致预约失效 | 动态计算TTL |
| 状态延迟 | 网络延迟导致预约不同步 | 实现状态同步检查 |

### 问题3：任务取消时的预约释放

**推荐方案**：综合方案
1. 立即释放预约
2. 清除运行时状态
3. 发送停止轨迹给AGV
4. 发送通知给其他系统

**关键点**：
- 预约必须立即释放，不能等待TTL过期
- 需要通知AGV停止执行
- 需要通知其他系统任务已取消
