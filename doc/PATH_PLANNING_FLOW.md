# 路径规划与资源预约（节点级）

本文描述当前工程的“节点级避障 + 资源预约”路径规划链路：规划不引入时间窗概念，而是通过共享的节点占用/预约表（`NodeReservationTable`）实现避障与并发协调。

核心代码位置：
- `src/tools/external_receiver.cpp`（动态规划、预约/释放、临时目标、Trail 截断）
- `src/path_planning/AStarPathFinder.cpp`（支持 `bannedNodes` 的 A*）
- `src/path_planning/StaticPathTable.cpp`（可选静态表）
- `src/message/ResultPublisher.cpp`（结果发布与落盘裁剪）

---

## 1) 关键数据结构

### 1.1 NodeReservationTable（共享并发数据）

`NodeReservationTable` 以 `nodeId` 为 key，记录：
- 被谁占用/预约（owner）
- 占用原因（例如等待点/预约窗口/临时目标）
- TTL 与周期性清理

规划时通过 `snapshotHeldNodes(deviceId)` 获取“除自己以外”的被占用节点快照，作为 A* 的 `bannedNodes` 输入，从而实现节点级避障。

### 1.2 每车运行态（RuntimeState）

每台 AGV 维护：
- `targets`：目标点队列（通常对应子任务点；到一个就 `pop_front()`）
- `reservedNodes`：当前预约窗口（最多 N 个节点）
- `blockedSince` / `tempGoalNodeId`：长时间规划失败后的临时目标机制

---

## 2) RobotPathRequest：动态规划到“下一个子任务点”

入口：`external_receiver` 收到 `RobotPathRequest` 后，会对该 AGV 做一次“动态规划 + 预约窗口更新”，核心逻辑在 `compute_dynamic_plan_to_next_target(...)`。

主要流程（概念化）：

1. **读取当前状态**：`startNodeId`（当前节点）与 `committedNextNodeId`（状态流里的 `nextDestinationPoint`，表示承诺边）。
2. **释放身后节点**：把 `reservedNodes` 中位于 `startNodeId` 之前的节点释放（避免预约窗口“拖尾”）。
3. **到点弹栈**：若 `startNodeId == targets.front()`，说明到达目标点，弹出并进入下一个子任务点。
4. **保底占用当前节点**：始终 `tryReserve(startNodeId, WAITING_POINT, ttl)`，避免并发窗口中当前位置丢失。
5. **构造 bannedNodes**：从 `NodeReservationTable` 取“其他车占用节点”快照作为 `bannedNodes`。
6. **规划路径**：优先静态表（若启用），否则 A*（携带 `bannedNodes`）。规划失败时会进入“等待→临时目标”机制（见下文）。
7. **保留承诺边**：如果状态流表明车正处于 `start -> committedNext` 的运动中，规划结果会强制以该边作为第一跳（避免频繁抖动/反复改道）。
8. **计算预约长度**：`reserveBudget = compute_reserve_budget_nodes(taskPriority, batteryLevel, deviceType)`，并受 `RESERVE_*` 环境变量约束。
9. **预约窗口**：沿规划出的 `plannedRoute` 顺序 `tryReserve`，直到达到 `reserveBudget` 或遇到已被他人占用的节点；得到 `newReserved`。
10. **释放旧预约**：将旧 `reservedNodes` 中不属于 `newReserved` 的节点逐个 `release`，再更新 `reservedNodes=newReserved`。
11. **返回路径**：对外返回的 path 只包含“本次预约到的窗口”（也就是允许前进的那段），车走到窗口末端后会继续请求下一段。

---

## 3) RobotTrailRequest：按 subTaskId 截断 Trail

`RobotTrailRequest` 可选字段：`subTaskId`（推荐格式：`TASK_003#2`）。

处理逻辑：
- 若请求带 `subTaskId`，`external_receiver` 会尝试在该车的缓存计划中定位对应子任务节点，并将 Trail 截断到该节点（含该节点）。
- 若未携带或找不到该子任务，则 Trail 默认按 `TRAIL_MAX_POINTS` 控制返回长度。

---

## 4) 临时目标机制（规划失败自救）

当多次规划失败且等待超过阈值（阈值 = 2 * REPLAN_TICK_MS）时：
- 为该车选择一个“临时目标点”（优先 `TEMP_GOAL_CANDIDATES` 指定的候选，否则从地图里选一个最近的可用节点）。
- 预约与返回路径会以临时目标为导向，使车辆先离开拥堵/死锁区域；
- 同时系统仍持续尝试规划到真正的下一个子任务点，一旦可达就切回。

---

## 5) 关键环境变量

详见 `doc/ENV_VARS.md`，与本链路强相关的包括：
- 静态表：`SKIP_STATIC_TABLE`、`PLANNER_STATIC_TABLE`
- 预约：`RESERVE_TTL_MS`、`RESERVE_CLEANUP_INTERVAL_MS`、`RESERVE_BASE_NODES`、`RESERVE_MIN_NODES`、`RESERVE_MAX_NODES`
- 临时目标：`TEMP_GOAL_CANDIDATES`（阈值由重规划周期推导）
- 重规划：`REPLAN_TICK_MS`
- Trail：`TRAIL_MAX_POINTS`

---

## 附：预约长度如何计算（节点数）

当前实现位于 `src/tools/external_receiver.cpp` 的 `compute_reserve_budget_nodes()`，逻辑为：

- 读取环境变量：
  - `RESERVE_MIN_NODES`（默认 2）
  - `RESERVE_MAX_NODES`（默认 20）
  - `RESERVE_BASE_NODES`（默认 4）
- 计算分数并四舍五入为节点数：
  - `score = RESERVE_BASE_NODES`
  - `score += max(taskPriority, 0) * 0.3`
  - `score += clamp(batteryLevel, 0..100) * 0.05`
  - `if (deviceType > 0) score += 2.0`
  - `reserveBudget = round(score)`，再 clamp 到 `[RESERVE_MIN_NODES, RESERVE_MAX_NODES]`

其中：
- `taskPriority` 来自本轮调度请求中任务的优先级（按车的“下一个子任务点”关联的 taskId 取值）。
- `batteryLevel` / `deviceType` 来自状态流（字段 `batteryLevel` / `agv_type` 或 `deviceType`）。
