# AGV 集群调度 – 架构与数据流

本文介绍当前的整体架构、数据流、核心数据结构以及各可执行程序的作用。内容已包含：新消息格式（candidateTasks）、子任务建模、以毫秒为单位的成本矩阵、POSTA 的优先级惩罚、诊断输出，以及按批量大小（maxBatchSize）的筛选。

## 总览

- 系统在地图图上的最短路基础上，构建“时间”维度的任务分配。
- 节点间移动时间：最短路距离（mm）/ 全局最大速度 `globalMaxSpeed`（mm/s）。
- 任务内生时间：子任务段的动作时间（estimatedDuration，毫秒）与段间移动时间之和。
- 采用 POSTA 进行分配（可开启“优先级惩罚”）；保留 Greedy 作为基线对照。

## 可执行程序

| 程序 | 路径 | 作用 | 输入 | 输出 |
|---|---|---|---|---|
| external_receiver | `src/tools/external_receiver.cpp` | 监听外部交换机/队列，执行调度 + 路径规划并发布结果。 | RabbitMQ（外部请求/状态/地图） | 控制台输出 + 结果消息 |
| external_task_sender | `src/tools/external_task_sender.cpp` | 发送调度请求 `AssignmentTaskRequest`。 | 本地 JSON/默认样例 | 发往 `EXT_EXCHANGE`/`EXT_QUEUE` |
| external_status_sender | `src/tools/external_status_sender.cpp` | 发送机器人状态 `SendRobotStatusInfos`。 | JSON 文件 | 发往 `EXT_EXCHANGE`/`EXT_QUEUE` |
| external_config_sender | `src/tools/external_config_sender.cpp` | 发送机器人配置 `SendRobotConfigInfos`。 | JSON 文件 | 发往 `EXT_EXCHANGE`/`EXT_QUEUE` |
| external_map_sender | `src/tools/external_map_sender.cpp` | 发送地图 `SendMapInfo`。 | `MAP_FILE` | 发往 `EXT_EXCHANGE`/`EXT_QUEUE` |
| external_path_request_sender | `src/tools/external_path_request_sender.cpp` | 触发全局路径请求 `RobotPathRequest`。 | — | 发往 `EXT_EXCHANGE`/`EXT_QUEUE` |
| external_trail_request_sender | `src/tools/external_trail_request_sender.cpp` | 触发轨迹请求 `RobotTrailRequest`。 | — | 发往 `EXT_EXCHANGE`/`EXT_QUEUE` |

## 新消息格式（candidateTasks）

顶层字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `schedulingRequestId` | string | 请求ID（日志展示用，可选）。 |
| `requestType` | int / string | 请求类型。整型枚举：0=批量，1=单任务，2=紧急调度，默认 -1=未知；兼容旧字符串值。 |
| `triggerContext` | string / {type:string} | 触发上下文描述；推荐直接传字符串，兼容 `{ "type": "..." }` 形式。默认空字符串。 |
| `timeoutMs` | number | 任务超时（用于 Task 构造）。 |
| `batchConfig` | int / {maxBatchSize:number} | 批量大小上限；整型形式直接表示 `maxBatchSize`，默认 -1=不限。 |
| `candidateTasks[]` | array | 任务列表。 |

`candidateTasks[]` 每项：

| 字段 | 类型 | 含义 | 内部映射 |
|---|---|---|---|
| `taskId` | string | 任务ID | Task.messageId |
| `priority` | number | 优先级（越大越高） | Task.priority |
| `minBatteryLevel` | number 0–100 | 最低电量（0 表示不限制） | Task.minBatteryLevel |
| `expectedStartTime` | string ISO8601 | 用于批量排序（早的在前） | 排序键 |
| `bindRobotId` | string | 指定执行的 deviceId（如 `AGV01`）；非空时忽略 `agvRequirements` | 绑定约束（仍参与分配） |
| `agvRequirements[]` | string[] | 可执行的 deviceId 列表；空表示不限制 | Task.deviceIds |
| `subTasks[]` | point[] | “点”列表，系统转换为段 i→i+1 | 转为 `SubTask` 段 |
| `subTasks[i].estimatedDuration` | number ms | 点的动作时间 | 作为“到达该点的段”的动作时间 |
| `subTasks[i].point.nodeId` | string | 地图 nodeId | 段起止节点 |

说明：
- `bindRobotId` 非空时，忽略 `agvRequirements`，任务绑定到指定 AGV，并参与分配用于排序/成本计算。
- `agvRequirements` 为空或缺失 ? 所有 AMR 可执行（不做 deviceId 过滤）。
- 段动作时间采用“下一点”的 `estimatedDuration`（i+1，毫秒），表示到达该点后执行的动作时间。

## 内部数据结构

- Task（`include/data/Task.h`）
  - 字段：`priority`、`taskType`、`messageId`、`deviceIds`（字符串白名单，空=不限制）、`timeout`、`subTasks`（段）、`minBatteryLevel`、`mapId`。
  - 起止节点：分别取 `subTasks.front().startNodeId` / `subTasks.back().endNodeId`。
- SubTask（`include/data/SubTask.h`）
  - 字段：`sequence`、`pointType`（枚举：0=PICKUP,1=DROPOFF,2=WAYPOINT,3=CHARGE,4=STANDBY,5=MAINTENANCE, -1=UNKNOWN）、`point`、`estimatedDuration`（毫秒）等。
- Amr（`include/data/Amr.h`）
  - 关键：`deviceId`、`deviceType`、`batteryLevel`、`endurance`（小时）、`maxTimeAheadMs`（毫秒）、`nextDestinationPointId`、`x/y`。

## 成本矩阵与约束

- 模块：`src/algorithm/CostMatrixGenerator.cpp`
- 形状：`[taskNum+1][taskNum][amrNum]`（逻辑结构，实际为稀疏存储，单位：毫秒）
  - 最后一行（索引 `taskNum`）：AMR→任务 t 的“进入成本” = 行驶时间(AMR→t) + 任务 t 的总作业时长（所有子任务 estimatedDuration 之和，毫秒）。
  - 其他行：i→j 的“转移成本” = 行驶时间(i→j) + 任务 j 的总作业时长。
- 行驶时间计算（近似）
  - 距离：闵氏距离，默认 p=1（曼哈顿 `|dx|+|dy|`）。
  - 速度：`min(起点节点.maxSpeed, 终点节点.maxSpeed, amr.maxLinearVelocity)`，任一为 0 则回退 `map.info.maxSpeed`。
  - 说明：为提速采用 Top‑K 稀疏近似（见“性能参数”）；如某任务对所有 AMR 均未覆盖，则对其做兜底覆盖计算。
- 合法性过滤（`isTaskValid`）
  - AMR 优先级阈值
  - 不可用状态（失败/未执行/暂停/不接受任务）直接过滤
  - 任务最低电量（`minBatteryLevel`）
  - 设备ID白名单 `deviceIds`（空=不限制）
- 首段时间门限（`maxTimeAheadMs`）
  - 不在矩阵阶段置 INF；在“结果修复/截断”阶段判断（见下文）。

## 最短路径

- 模块：`src/algorithm/ShortestPathUpdater.cpp`
- 启发式：闵氏距离，默认 p=1（曼哈顿，单位 mm）。
- `ShortestPathUpdater` 返回秒；成本矩阵阶段统一换算为毫秒（通常用近似距离/速度，不必每对都跑最短路）。

## detour 截断与修复

- 模块：`src/algorithm/base/TaskAllocatorBase.cpp`（repairInvalidDuties 最后阶段）。
- 规则：
  - 时间阈值：`maxTimeAheadMs`（毫秒）。当某任务“开始时间”≥阈值时触发截断位置。
  - 距离阈值：`maxDetourDistanceMm`（mm）。按任务起点间的闵氏距离（默认曼哈顿）累加，达到阈值触发截断位置。
  - 截断策略：取两者最早触发位置；至少保留 1 个任务（若触发位置为首位，则从第 2 个任务开始截断）。
  - 续航约束：在 detour 前独立检查，超出的任务会被移除/标记未分配。
- 默认值与环境变量参见 `doc/ALLOCATION_PARAMETERS.md`（`ALLOC_MAX_TIME_AHEAD_SEC` / `ALLOC_MAX_DETOUR_MM` 以及对应的 `EXT_*` 覆盖项）。

## 分配算法

- POSTA（`src/algorithm/PostaTaskAllocator.cpp`）
  - 额外目标项：优先级惩罚 `a * start_time * priority_weight[priority]`。
  - 提供接口：`setPriorityPenaltyCoeff(a)`、`setPriorityPenaltyCurve([...])`。
  - 搜索算子：swap/shift/sym。
- 关键调优参数（`ALLOC_POSTA_*`、`ALLOC_PRI_COEFF` 等）同样在 `doc/ALLOCATION_PARAMETERS.md` 中列出默认值和配置方式。
- Greedy（`src/algorithm/GreedyTaskAllocator.cpp`）
  - 简单基线实现（未改动）。

## MLP 分配器接入接口

`external_receiver` 在 `allocationAlgorithm=mlp` 时会调用 `MlpTaskAllocator`。检测方式为
`__has_include("algorithm/MlpTaskAllocator.h")`，因此必须提供该头文件并参与编译。

### 必要接口

- 头文件路径：`include/algorithm/MlpTaskAllocator.h`
- 类名：`MlpTaskAllocator`（建议继承 `TaskAllocatorBase`）
- 方法签名：
  - `AllocationResult allocate(const std::vector<Amr>& amrList, const std::vector<Task>& taskList) const`

### 输入语义

- `amrList`：来自 `TaskPolicy::selectAvailableAmrs` 的可用 AMR 列表（已按状态过滤），顺序必须保留。
- `taskList`：经过“不可达预筛 + 批量裁剪”的任务列表，索引 0..N-1 对应当前轮分配任务。

### 输出语义（AllocationResult）

- `amrTasks`：`amrTasks[i]` 为第 i 个 AMR 的任务序列，任务索引为 0 开始。
- `code`：可选编码向量，任务为 1 开始、`0` 分隔；若 `amrTasks` 为空则使用 `code` 解析。
- `unallocatedTaskIds`：未分配任务的 0 开始索引列表（必须覆盖所有未出现在 `amrTasks` 的任务）。
- `totalCost`：可选；后续会被 `SanitizeAllocationResult` 重新计算。
- `repairLog`：可选调试信息。

### 约束一致性（MLP 侧必须自行保证）

MLP 分配绕过 `CostMatrixGenerator`，因此以下约束不会被自动注入，需在 MLP 内部处理：

- 优先级阈值：`amr.getPriorityThreshold() > task.getPriority()` 视为不可分配。
- 最低电量：`task.getMinBatteryLevel() > amr.getBatteryLevel()` 视为不可分配。
- 设备白名单：`task.getDeviceIds()` 非空时，必须包含 `amr.getDeviceId()`。
- 非紧急任务仅允许空闲 AMR（规则与 `CostMatrixGenerator::isTaskValid` 一致）。
- 充电任务隔离：若某 AMR 存在“绑定的充电任务”（`taskType==3` 且 `deviceIds` 包含该 AMR），
  则该 AMR 对非充电任务应视为不可分配（与成本矩阵规则保持一致）。

### 成本与可达性

- `external_receiver` 在 `mlp` 模式下不会生成成本矩阵，但会调用：
  - `TaskAllocationUtils::sparseReset(taskNum, amrNum)`
  - `TaskAllocationUtils::setGlobalSpeedMmPerSec(...)`
  - `TaskAllocationUtils::setMapInfoPtr(...)`
- 若 MLP 需要依赖 `TaskAllocationUtils::getCost()` 或希望 `totalCost` 为有限值，
  则需自行用 `TaskAllocationUtils::sparsePut(prevOrTaskNum, curIdx, amrIdx, value_ms)` 填充稀疏成本。
- `SanitizeAllocationResult` 会再次做可达性校验与修复，因此 MLP 输出的 AMR/任务索引必须严格对应输入列表。

### 编译接入

- 新增实现文件：`src/algorithm/MlpTaskAllocator.cpp`
- 将其加入 `CMakeLists.txt` 中 `ALGO_CORE_SOURCES` 列表，确保参与编译链接。

## 批量筛选（maxBatchSize）

- 在 `external_receiver` 解析完所有 `candidateTasks` 后：
  1) 先按 Priority 降序；
  2) 再按 expectedStartTime 升序（ISO 字符串按字典序即时间顺序）。
- 仅取前 `batchConfig.maxBatchSize` 个进入 POSTA（缺省或≤0 表示取全部）。

## 诊断与未分配原因

- `external_receiver` 打印：
  - 每台 AMR 的任务列表
  - 未分配任务及原因：
    - 设备/类型/优先级限制
    - 路径不可达
    - 首段时间超阈（maxTimeAheadMs）
    - 续航不足（endurance）
    - 调度冲突/成本劣势
- 设定环境变量 `ALLOC_DIAG=1` 可打印更详细的 AMR 概览与“AMR→任务首段”诊断；另外，receiver 会打印“无任务 AMR × 未分配任务”的最小首段成本（便于定位为何没被分配）。

## 性能参数（环境变量）

| 变量 | 默认 | 说明 |
|---|---|---|
| `ALLOC_TOPK_RATIO` | 0.5 | Top-K 稀疏比例（`round(taskNum * ratio)`，最小 1） |
| `ALLOC_TOPK` | -1 | Top-K 上限（>0 时覆盖 `ALLOC_TOPK_AGENT/ALLOC_TOPK_NEXT`） |
| `ALLOC_TOPK_AGENT` | auto | AMR→Task 的 Top-K 覆盖 |
| `ALLOC_TOPK_NEXT` | auto | Task→Task 的 Top-K 覆盖 |
| `ALLOC_TOPK_COVERAGE` | 1 | 无人覆盖任务兜底（1 开启，0 关闭） |
| `ALLOC_WORKERS` | 4 | 最短路 worker 数量 |

## RabbitMQ 与配置

通用环境变量：

| 变量 | 默认值 | 含义 |
|---|---|---|
| `AMQP_HOST` | 127.0.0.1 | 服务器地址 |
| `AMQP_PORT` | 5672 | 端口 |
| `AMQP_USER` | guest | 用户名 |
| `AMQP_PASS` | guest | 密码 |
| `AMQP_VHOST` | / | 虚拟主机 |

external_*_sender / external_receiver：

| 键 | 默认 |
|---|---|
| `EXT_EXCHANGE` | `DispToAlgoExchange` |
| `EXT_QUEUE` | `DispToAlgoQueue` |
| `EXT_ROUTING_KEY` | `AssignmentTaskRequest` |
| `EXT_BINDING_KEY` | `#` |
| `EXT_TIMEOUT_SEC` | 30 |

诊断：

| 变量 | 默认 | 作用 |
|---|---|---|
| `ALLOC_DIAG` | 未设置 | 设置为非 0/`true` 时启用详细诊断输出 |

## 端到端流程（external_receiver）

1) 等待/接收地图（`SendMapInfo`）并初始化最短路模块；地图未就绪时拒绝调度请求。
2) 缓存 `agvStatusList` 并据此构造 AMR 列表；若缺失状态则直接输出未分配结果。
3) 解析 candidateTasks → 内部 Task：
   - 点列表 → 段（i→i+1），段动作时间来自下一点 `estimatedDuration`。
   - `bindRobotId` 非空时任务绑定到指定 AGV，并参与分配用于排序/成本计算；否则 `agvRequirements` → `Task.deviceIds`（字符串白名单，空=不限制）。
   - 设置 `Task.minBatteryLevel`。
4) 按优先级与预计开始时间排序，并按 `maxBatchSize` 截断。
5) 生成“毫秒”为单位的成本矩阵：
   - AMR→t = 行驶时间(AMR→t) + service(t)；i→j = 行驶时间(i→j) + service(j)。
   - 距离默认用曼哈顿，速度取三者最小；Top-K 稀疏近似 + 兜底覆盖。
6) 运行 Greedy + POSTA；修复不可达/续航/detour 截断；打印结果与未分配原因。
7) 发布结果并生成路径规划信息（如静态表可用）。

## 与早期版本的差异

- 使用新消息格式 `candidateTasks`，不再使用旧的 `amrs`/`tasks`。
- 内部 `Task` 仅保留 `subTasks`（段），移除旧的 `taskPoints`。
- 成本矩阵统一使用毫秒；最短路返回秒。
- `deviceIds` 为空表示不限制，否则按白名单过滤。
- 新增 `minBatteryLevel` 与 `batchConfig.maxBatchSize`（批量）。
- 移除 `include/message/` 与 `src/message/` 下的旧消息类型，直接使用 JSON 解析。

## 可扩展性方向

- 从独立队列接入外部 AMR 信息，替换本地静态 AMR。
- 将分配结果与诊断落库或写至文件。
- 将优先级惩罚、`maxTimeAheadMs`、`globalMaxSpeed` 等参数化为配置或环境变量。



