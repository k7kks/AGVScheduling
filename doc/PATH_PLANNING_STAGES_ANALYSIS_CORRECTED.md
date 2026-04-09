# AGV路径规划三阶段分析文档（修正版）

## 背景说明

在AGV集群调度系统中，路径规划分为三个阶段：**Fast**、**Major**、**Super**。

**核心约束**：
- 发送给机器人的轨迹（Trail）**必定会被执行**
- 轨迹中**只包含已预约的节点**
- 如果某个节点被占用，轨迹会被**截断到最后一个成功预约的节点**
- 不能发送未预约的节点

---

## 三个阶段定义

### 1. Fast阶段（快速规划）

**定义**：默认的快速路径规划阶段

**特点**：
- 规划时间短（<100ms）
- 基于静态地图和当前AGV位置
- **不考虑其他AGV的预约**（`avoidHeldNodesInPlanning = false`）
- 先规划完整路径，后截断

**流程**：
```
1. 规划完整路径：1→2→3→4→5
2. 尝试预约轨迹节点：
   - 节点1：预约成功 ✓
   - 节点2：预约成功 ✓
   - 节点3：预约成功 ✓
   - 节点4：预约成功 ✓
   - 节点5：被AGV_B占用 ✗ → 截断
3. 发送轨迹：1→2→3→4（只包含已预约的节点）
```

**问题**：
- 可能与其他AGV冲突
- 轨迹可能被截断

**代码位置**：第5395-5397行
```cpp
// Fast replans should not consider other AGVs' reservations during planning
std::vector<int> emptyBlocked;
const std::vector<int>& congestionBlocked = avoidHeldNodesInPlanning ? blockedVec : emptyBlocked;
```

---

### 2. Major阶段（主要规划）

**定义**：当机器人被阻塞时触发的重规划阶段

**触发条件**（第6274-6292行）：
- 机器人静止时间 > `REPLAN_MAJOR_STUCK_MS`（默认2000ms）
- 且距离上次MAJOR尝试已过冷却时间

**特点**：
- 规划时间中等（100-500ms）
- **在规划时考虑其他AGV的预约**（`avoidHeldNodesInPlanning = true`）
- 允许更长的路由（`allowLongerRoute = true`）
- 尝试绕过被占用的节点

**流程**：
```
场景：Fast阶段发送了 1→2→3→4，但机器人在节点2停留超过2秒

1. 检测到机器人被阻塞（静止时间 > 2000ms）
2. 触发Major重规划
3. 规划新路径，避开被占用的节点：
   - 节点1：当前位置
   - 节点2：被占用 ✗ → 避开
   - 尝试绕路：1→6→7→4
4. 预约新路径：
   - 节点1：预约成功 ✓
   - 节点6：预约成功 ✓
   - 节点7：预约成功 ✓
   - 节点4：预约成功 ✓
5. 发送新轨迹：1→6→7→4
```

**关键特性**（第5252-5254行）：
```cpp
if (stage == ReplanStage::MAJOR || stage == ReplanStage::SUPER) {
    rt.lastMajorAttempt = now;  // 记录尝试时间
}
```

**问题**：
- 如果绕路也被占用，仍然会失败
- 失败次数过多会触发Super阶段

---

### 3. Super阶段（超级规划）

**定义**：全局优化和死锁恢复的规划阶段

**触发条件**（第6274-6292行）：
- Major阶段失败次数 >= `REPLAN_SUPER_AFTER_MAJOR_FAILS`（默认3次）

**特点**：
- 规划时间长（500ms-2s）
- **释放所有预约**（除了当前节点）
- 重新开始规划
- 允许使用临时目标（temp goal）
- 全局优化

**流程**（第4996-5006行）：
```
场景：Major阶段失败3次，触发Super

1. 释放所有预约（除了当前节点）
   - 释放之前的 1→6→7→4 预约
   - 只保留当前节点1的预约

2. 重新规划路径
   - 现在可以使用之前被占用的节点
   - 例如：1→2→3→4→5（如果现在可用）

3. 预约新路径：
   - 节点1：预约成功 ✓
   - 节点2：预约成功 ✓
   - 节点3：预约成功 ✓
   - 节点4：预约成功 ✓
   - 节点5：预约成功 ✓

4. 发送新轨迹：1→2→3→4→5
```

**代码**（第4996-5006行）：
```cpp
if (stage == ReplanStage::SUPER) {
    std::vector<int> keepNodes;
    keepNodes.push_back(startNodeId);
    if (!deviceId.empty()) {
        reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);  // 释放所有预约
        reservations.tryReserve(startNodeId,
                                 deviceId,
                                 NodeReservationTable::HoldReason::WAITING_POINT,
                                 "super_release",
                                 std::chrono::milliseconds(0));
        rt.reservedNodes = keepNodes;
        committedPrefix = keepNodes;
        repo.updateLastSentRoute(deviceId, keepNodes);
    }
}
```

---

## 核心问题分析

### 问题1：轨迹截断导致路径改变

**场景**：
```
时间轴：
t=0s:   Fast阶段规划路径：1→2→3→4→5
        预约节点：1,2,3,4（节点5被占用）
        发送轨迹：1→2→3→4

t=2.5s: 机器人在节点2停留超过2秒
        触发Major重规划

t=2.5s: Major阶段规划新路径：1→2→3→4→6→5
        预约节点：1,2,3,4,6（节点5仍被占用）
        发送轨迹：1→2→3→4→6

问题：轨迹从 1→2→3→4 变成 1→2→3→4→6
      这违反了commit约束吗？
```

**分析**：
- **不违反**！因为：
  1. 第一个轨迹 `1→2→3→4` 已经发送并执行
  2. 机器人会执行这个轨迹
  3. 当机器人到达节点4时，会收到新轨迹 `1→2→3→4→6`
  4. 新轨迹包含了旧轨迹的所有节点（1→2→3→4）
  5. 只是在节点4之后添加了新的节点6

**关键约束**：
- 已发送的轨迹必须被执行
- 新轨迹可以在已执行的轨迹基础上扩展
- 但不能改变已执行部分的路径

---

### 问题2：预约窗口与轨迹长度

**配置**（第34-35行）：
```bash
TRAIL_MAX_POINTS="${TRAIL_MAX_POINTS:-10}"      # 轨迹最大点数
TRAIL_MIN_POINTS="${TRAIL_MIN_POINTS:-2}"       # 轨迹最小点数
```

**场景**：
```
配置：TRAIL_MAX_POINTS=4（轨迹最多4个节点）
      RESERVE_COMMITTED_TTL_MS=15000（已提交节点TTL 15秒）

时间轴：
t=0s:   Fast阶段规划路径：1→2→3→4→5→6→7→8
        截取前4个节点：1→2→3→4
        预约节点：1,2,3,4（TTL=15秒）
        发送轨迹：1→2→3→4

t=1s:   AGV_A 到达节点2
        释放节点1的预约
        新增预约节点5
        当前预约：2,3,4,5

t=2s:   AGV_A 到达节点3
        释放节点2的预约
        新增预约节点6
        当前预约：3,4,5,6

t=3s:   AGV_A 到达节点4
        释放节点3的预约
        新增预约节点7
        当前预约：4,5,6,7

t=4s:   AGV_A 到达节点5
        释放节点4的预约
        新增预约节点8
        当前预约：5,6,7,8

结果：轨迹窗口始终保持4个节点，动态向前滑动
```

**优点**：
- 预约窗口动态滑动
- 后续节点逐步被保护
- 不会出现"后续节点无保护"的问题

---

### 问题3：状态更新与轨迹同步

**场景**：
```
时间轴：
t=0s:   发送轨迹：1→2→3→4
        预约节点：1,2,3,4

t=1s:   AGV_A 发送状态消息：currentNode=2
        系统收到状态消息

t=1.1s: 系统处理状态更新（第6878-6898行）
        找到节点2在轨迹中的位置
        释放已通过的节点：1
        截断轨迹：2→3→4
        更新预约：释放节点1，保留2,3,4

结果：轨迹和预约始终同步
```

**代码**（第6878-6898行）：
```cpp
auto it = std::find(trimmed.begin(), trimmed.end(), curNodeId);
if (it != trimmed.end()) {
    size_t releasedCount = static_cast<size_t>(std::distance(trimmed.begin(), it));
    for (auto jt = trimmed.begin(); jt != it; ++jt) {
        reservations.release(*jt, entry.deviceId);  // 释放已通过的节点
    }
    trimmed.erase(trimmed.begin(), it);  // 截断轨迹
}
```

---

### 问题4：预约失败导致轨迹截断

**场景**：
```
时间轴：
t=0s:   Fast阶段规划路径：1→2→3→4→5
        尝试预约：
        - 节点1：预约成功 ✓
        - 节点2：预约成功 ✓
        - 节点3：预约成功 ✓
        - 节点4：被AGV_B占用 ✗ → 停止

        已预约节点：1,2,3

t=0.1s: 检查已预约节点数是否满足最小要求
        如果 3 >= TRAIL_MIN_POINTS(2)：
            发送轨迹：1→2→3
        否则：
            跳过响应，等待下一个周期

结果：轨迹被截断到最后一个成功预约的节点
```

**代码**（第8085-8104行）：
```cpp
if (!commitOk) {
    if (committed.size() < committedRequired) {
        // 已预约节点数 < 最小要求 → 跳过响应
        skipTrailResponse = true;
        trimmed.clear();
    } else {
        // 已预约节点数 >= 最小要求 → 返回已预约部分
        trimmed = std::move(committed);
    }
}
```

---

### 问题5：三个阶段的轨迹演变

**完整场景**：
```
初始状态：
- AGV_A 需要执行任务：1→2→3→4→5→6→7→8
- AGV_B 占用节点5

时间轴：

t=0s: FAST阶段
  规划：1→2→3→4→5→6→7→8
  预约：1,2,3,4（节点5被占用，截断）
  发送：1→2→3→4

t=1s: AGV_A 到达节点2
  释放节点1
  新增预约节点5（仍被占用）
  当前预约：2,3,4

t=2s: AGV_A 到达节点3
  释放节点2
  新增预约节点5（仍被占用）
  当前预约：3,4

t=3s: AGV_A 到达节点4
  释放节点3
  新增预约节点5（仍被占用）
  当前预约：4

t=4s: AGV_A 在节点4停留超过2秒
  触发MAJOR重规划

t=4s: MAJOR阶段
  规划：避开节点5，尝试绕路
  规划结果：1→2→3→4→6→7→8
  预约：4,6,7,8（节点5被避开）
  发送：4→6→7→8

t=5s: AGV_A 到达节点6
  释放节点4
  新增预约节点8
  当前预约：6,7,8

t=6s: AGV_A 到达节点7
  释放节点6
  新增预约节点9（如果有）
  当前预约：7,8

t=7s: AGV_A 到达节点8
  释放节点7
  当前预约：8

t=8s: AGV_A 完成任务
  释放节点8
  当前预约：空

结果：
- Fast阶段：1→2→3→4
- Major阶段：4→6→7→8
- 完整执行：1→2→3→4→6→7→8（避开了节点5）
```

---

## 关键机制总结

### 1. 轨迹发送机制

**原则**：
- 轨迹中**只包含已预约的节点**
- 预约失败时**立即截断**
- 截断后的轨迹**必须满足最小长度要求**

**代码流程**（第8068-8104行）：
```
for each node in planned_path:
    if tryReserve(node) succeeds:
        add to committed
    else:
        break  // 立即停止，不继续

if committed.size() >= min_required:
    send committed
else:
    skip response
```

### 2. 预约管理机制

**预约类型**：
- `RESERVED_PATH`：轨迹中的路径节点
- `WAITING_POINT`：机器人当前等待点
- `TEMP_GOAL`：临时目标点

**预约生命周期**：
```
1. 规划时：为轨迹节点预约
2. 执行时：机器人逐步通过节点
3. 更新时：释放已通过的节点，新增后续节点
4. 完成时：释放所有预约
```

### 3. 三阶段协调机制

**阶段转换**：
```
FAST (默认)
  ↓
  如果 静止时间 > REPLAN_MAJOR_STUCK_MS
  ↓
MAJOR (重规划)
  ↓
  如果 失败次数 >= REPLAN_SUPER_AFTER_MAJOR_FAILS
  ↓
SUPER (全局优化)
```

**关键差异**：
| 阶段 | 规划时考虑预约 | 允许更长路由 | 释放预约 |
|------|--------------|-----------|--------|
| FAST | 否 | 否 | 否 |
| MAJOR | 是 | 是 | 否 |
| SUPER | 是 | 是 | 是（除当前节点） |

---

## 环境变量配置

| 变量名 | 默认值 | 功能 |
|--------|--------|------|
| `TRAIL_MAX_POINTS` | 10 | 轨迹最大点数 |
| `TRAIL_MIN_POINTS` | 2 | 轨迹最小点数 |
| `RESERVE_COMMITTED_TTL_MS` | 0 | 已提交节点的TTL（毫秒） |
| `RESERVE_TTL_MS` | 15000 | 预约节点的TTL（毫秒） |
| `REPLAN_MAJOR_STUCK_MS` | 2000 | 触发MAJOR的静止时间（毫秒） |
| `REPLAN_SUPER_AFTER_MAJOR_FAILS` | 3 | 触发SUPER的失败次数 |

---

## 可能的问题和解决方案

### 问题A：轨迹频繁变化

**现象**：
- 轨迹在Fast、Major、Super阶段频繁改变
- 机器人收到多个不同的轨迹

**原因**：
- 预约冲突导致轨迹截断
- 重规划阶段改变路径

**解决方案**：
1. 增加预约窗口大小（`TRAIL_MAX_POINTS`）
2. 增加预约TTL（`RESERVE_COMMITTED_TTL_MS`）
3. 优化任务分配，减少冲突

### 问题B：轨迹过短

**现象**：
- 发送的轨迹只有2-3个节点
- 机器人频繁收到新轨迹

**原因**：
- 后续节点被占用
- 预约失败导致截断

**解决方案**：
1. 改进任务分配算法，减少冲突
2. 增加预约窗口大小
3. 使用Major/Super阶段进行重规划

### 问题C：机器人被阻塞

**现象**：
- 机器人在某个节点停留很长时间
- 无法继续执行任务

**原因**：
- 后续节点被其他AGV占用
- Major/Super重规划失败

**解决方案**：
1. 检查是否有死锁（两个AGV互相阻塞）
2. 使用Super阶段释放预约，重新规划
3. 考虑使用临时目标（temp goal）

---

## 总结

**核心设计**：
- 轨迹中**只包含已预约的节点**
- 预约失败时**立即截断**
- 三个阶段**逐步优化**路径

**优点**：
- 保证轨迹可执行性
- 避免冲突
- 灵活应对环境变化

**缺点**：
- 轨迹可能较短
- 频繁重规划
- 可能出现死锁

**最佳实践**：
1. 合理配置预约窗口大小
2. 优化任务分配算法
3. 监控重规划频率
4. 实现死锁检测和恢复
