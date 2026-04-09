# 调度与路径规划数据流说明

本文仅关注**输入/输出数据结构**，总结当前工程在典型链路中的 JSON 格式，并给出样例，方便对接与联调。

涵盖的链路：

- 外部链路：`external_task_sender` / `external_status_sender` / `external_config_sender` / `external_map_sender` → `external_receiver`（→ MQ 结果/路径队列）
- 旧版 `allocation_sender/allocation_receiver` 已下线，文档仅保留外部链路。

当前外部链路使用同一套请求格式（新格式 `candidateTasks`）。调度结果、路径响应、轨迹响应会根据不同路由键分开发送，方便业务方独立订阅。

### MQ 路由概览

| 场景 | 路由键（默认） | 说明 |
|------|----------------|------|
| 业务 → 算法：调度请求 | `AssignmentTaskRequest` | 主请求，携带 candidateTasks。队列通常绑定 `#`。 |
| 业务 → 算法：机器人状态 | `SendRobotStatusInfos` | 批量 AMR 状态更新。 |
| 业务 → 算法：机器人配置 | `SendRobotConfigInfos` | 设备档案/尺寸等。 |
| 业务 → 算法：地图更新 | `SendMapInfo` | 地图 JSON 或差分。 |
| 业务 → 算法：全局路径 | `RobotPathRequest` | 单设备路径任务。 |
| 业务 → 算法：轨迹控制点 | `RobotTrailRequest` | 单设备轨迹任务。 |
| 算法 → 业务：调度结果 | `AssignmentTaskResponse` | ResultPublisher 输出（含 path_plan）。 |
| 算法 → 业务：全局路径结果 | `RobotPathResponse` | `build_path_response_body` 输出的路径段。 |
| 算法 → 业务：轨迹结果 | `RobotTrailResponse` | `build_trail_response_body` 输出的控制点。 |

### RobotTrailRequest / RobotTrailResponse

- 请求字段：
  - `messageId` (string)：请求唯一标识，响应会原样回显。
  - `deviceId` (string)：目标 AGV 设备号。
  - `subTaskId` (string，可选)：限定轨迹只返回到指定子任务节点。建议格式为 `任务ID#子任务序号`（例如 `TASK_003#2` 表示任务 `TASK_003` 的第 2 个子任务）。若提供此字段，响应轨迹会在该子任务起点处截断；缺省时仍按 `TRAIL_MAX_POINTS` 环境变量（默认 10）控制节点数量。
- 响应体：结构与以往一致，由 `build_trail_response_body()` 决定，仅新增上述截断逻辑，不额外带回 `subTaskId`。

> `ASSIGN_RESULT_*` 环境变量控制调度结果交换机/队列；Path/Tail 请求的响应使用相同交换机，但以各自路由键区分。

---

## 1. 调度请求（输入消息）

### 1.1 顶层结构

由 `external_task_sender` 发送，`external_receiver`  消费。

```jsonc
{
  "schedulingRequestId": "REQ_20251105_001",
  "requestTimestamp": "2025-11-05T10:30:00.000Z",
  "requestType": 0,
  "taskNumber": 2,
  "triggerContext": "新增任务触发",
  "triggerAgvId": [],
  "batchConfig": 50,
  "timeoutMs": 50000,
  "environmentContext": {
    "mapVersion": "1",
    "systemLoad": 3,
    "systemLoadFactor": 0.65
  },
  "candidateTasks": [ /* 任务列表，见 1.2 */ ],
  "agvStatusList": [ /* AMR 状态列表，见 1.3 */ ]
}
```

字段说明：

- `schedulingRequestId` (string)：请求 ID，用于日志与追踪，可选。
- `requestTimestamp` (string, ISO8601)：请求时间戳；若填写，会触发接收端时间可信性校验。可通过环境变量 `TS_MAX_SKEW_SEC`（或链路专用的 `ALLOC_TS_MAX_SKEW_SEC` / `EXT_TS_MAX_SKEW_SEC`）调整最大允许偏差，默认 0.5 秒。
- `requestType` (int/string)：请求类型；推荐使用整型枚举（0=批量，1=单任务，2=紧急调度，默认 -1=未知）。兼容旧字符串值（“批量”等）并自动映射。
- `taskNumber` (number)：任务数量（可选，接收端不依赖此字段驱动逻辑）。
- `triggerContext` (string/object)：触发上下文；推荐直接传字符串，兼容 `{ "type": "<text>" }`。
- `triggerAgvId[]` (string[])：触发相关的 AGV ID 列表，可为空。
- `batchConfig` (int/object)：批量限制；整型形式直接表示 `maxBatchSize`，默认 -1=不限。兼容 `{ "maxBatchSize": xxx }`。
- `timeoutMs` (number)：调度时间预算（毫秒），用于 Task 构造和 POSTA 时间切片。
- `environmentContext` (object)：环境信息（如地图版本、系统负载），主要用于日志。
- `candidateTasks[]` (array)：待分配任务列表（详见 1.2）。
- `agvStatusList[]` (array)：当前 AGV 状态列表（详见 1.3）。

补充说明：若短时间内收到多条 `candidateTasks` 调度请求且调度空闲，接收端会**只保留最新一条**进入分配/路径规划，其余请求将直接返回结果并设置 `status=404`（`metadata.constraints_violated` 包含 `stale_request`）。若调度正忙，新的请求会被直接丢弃并返回 `status=404`（`metadata.constraints_violated` 包含 `busy`）。

### 1.2 candidateTasks[] – 任务列表

单个任务示例：

```jsonc
{
  "taskId": "TASK_1",
  "priority": 8,
  "expectedStartTime": "2025-11-05T10:30:00.000Z",
  "expectedCompletionTime": "2025-11-05T10:50:00.000Z",
  "minBatteryLevel": 30,
  "bindRobotId": "AGV05",
  "taskType": "CHARGE",                // 可选：字符串或数字（3=充电）
  "subTasks": [
    {
      "subTaskId": "TASK_1#1",
      "sequence": 1,
      "pointType": 0,                  // 0=PICKUP,1=DROPOFF,2=WAYPOINT,3=CHARGE,4=STANDBY,5=MAINTENANCE
      "location": "ABC12345",
      "point": {
        "x": 179640,
        "y": 221475,
        "angle": 0,
        "nodeId": "25"
      },
      "estimatedDuration": 90000,      // 毫秒
      "pathConstraints": [],           // 可选：路径约束标签
      "maxSpeed": 100                  // 可选：段最大速度
    },
    {
      "subTaskId": "TASK_1#2",
      "sequence": 2,
      "pointType": 1,
      "location": "DEF67890",
      "point": {
        "x": 185000,
        "y": 230000,
        "angle": 90,
        "nodeId": "156"
      },
      "estimatedDuration": 150000,
      "pathConstraints": ["slow_zone"],
      "maxSpeed": 70
    }
  ],
  "agvRequirements": ["AGV05"]         // 可执行的 AGV deviceId；空=不限制
}
```

映射关系（到内部 `Task` 结构）：

- `taskId` → `Task.messageId`
- `priority` → `Task.priority`
- `minBatteryLevel` → `Task.minBatteryLevel`
- `expectedStartTime` → 排序键（高优先/早开始先进入计算）
- `taskType` → `Task.taskType`（数值或根据字符串映射；如 `"CHARGE"` → `3`）
- `bindRobotId` → 指定执行的 deviceId；非空时忽略 `agvRequirements`，任务绑定到指定 AGV，但仍进入分配用于顺序/成本计算
- `agvRequirements[]` → `Task.deviceIds`（字符串白名单，推荐提供 `deviceId`，如“AGV01”；空=不限制）
- `subTasks[]`：
  - 顺序决定任务路径段的链；系统将点 i→i+1 转为内部 `SubTask` 段；
  - `subTasks[i].subTaskId` → 子任务唯一标识，推荐格式 `taskId#序号`，用于后续轨迹查询（`RobotTrailRequest.subTaskId`）；
  - `subTasks[i].point.nodeId` → 段起止节点；必须是地图中的 nodeId；
- `subTasks[i].estimatedDuration`（毫秒）：到达该点后的作业时间，进入成本矩阵。

`pointType` 取值与含义：

| 值 | 含义 | 说明 |
|---|---|---|
| 0 | PICKUP | 取货 / 呼叫 |
| 1 | DROPOFF | 送货 |
| 2 | WAYPOINT | 经过点/导航辅助 |
| 3 | CHARGE | 充电 |
| 4 | STANDBY | 待命 |
| 5 | MAINTENANCE | 维护 |

如仍使用字符串，系统会自动映射到以上枚举；未识别时落入 `-1=UNKNOWN`。

### 1.3 agvStatusList[] – AGV 状态列表

单个 AGV 状态示例：

```jsonc
{
  "deviceId": "AGV01",
  "mapId": 1,
  "curArea": "ABC12345",
  "connection": true,
  "x": 179640,
  "y": 221475,
  "angle": 0,
  "speed": 0,
  "nodeId": "0",
  "nextDestinationPoint": null,
  "curTrailPoints": [],
  "taskId": "",
  "taskStatus": 0,
  "taskProgress": 0,
  "estimatedDuration": 0,
  "batteryLevel": 85,
  "endurance": 102,
  "load": false,
  "errorCode": 0,
  "updateTime": 1730883000,
  "agv_type": 1
}
```

字段要点（映射到内部 `Amr`）：

- `deviceId` → `Amr.deviceId`
- `mapId` → `Amr.mapId`
- `x`, `y`, `angle` → 当前世界坐标与姿态
- `nodeId` → 当前所在节点 ID（字符串形式），在内部会转为整数 nodeId 或用于 `findNearestNodeId` 近邻搜索。
- `nextDestinationPoint` / `curTrailPoints` → 当前/未来路径，内部用于估算“当前行程还需要多久”。
- `taskId` / `taskStatus` / `taskProgress` → 当前任务状态。
  - `taskStatus` 枚举：0=空闲, 1=作业中, 2=任务正常完成, 3=任务失败, 4=任务未执行, 5=任务暂停, 10=不接受任务
- `estimatedDuration` → 当前任务预计剩余时间（秒，仅用于展示/诊断）。
- `batteryLevel` → 当前电量百分比。
- `endurance` → 剩余续航（小时），直接写入 `Amr.endurance`（不做换算）。
- `load` → 是否有载。
- `agv_type` (number，可选) → 设备类型编号；若缺失则回退到 `deviceType` 字段或 0。

---

## 2. 调度结果（输出消息）

调度完成后，`external_receiver` 通过 `ResultPublisher` 将结果发送到结果队列：

- 交换机：`ASSIGN_RESULT_EXCHANGE`（默认 `AlgoToDispExchange`，`fanout` / 持久化）
- 队列：`ASSIGN_RESULT_QUEUE`（默认 `AlgoToDispQueue`，持久化，与既有 MQ 配置保持一致）
- 路由键：`ASSIGN_RESULT_ROUTING_KEY`（默认 `AssignmentTaskResponse`，用于标识结果类型）

数据结构包含两部分：

1. 任务分配结果（所有链路共有）
2. 路径规划结果（如果提供了地图和静态表，则自动填充 `path_plan`）

> `RobotPathResponse` / `RobotTrailResponse` 也走同一交换机，但使用各自路由键；二者的数据结构由 `build_path_response_body` / `build_trail_response_body` 输出，保持与调度结果解耦。

### 2.1 分配结果主结构

```jsonc
{
  "agv_assignments": [
    {
      "agv_id": "AGV01",
      "current_position": { "x": 179640, "y": 221475, "nodeId": "25" },
      "assigned_tasks": [
        {
          "task_id": "TASK_1",
          "task_sequence": 1,
          "task_type": "MATERIAL_TRANSPORT",   // 或 CHARGE / STANDBY / ...
          "priority": 8,
          "expected_start_time": "2025-11-05T10:30:00.000Z",
          "expected_completion_time": "2025-11-05T10:50:00.000Z",
          "pickup_point": { "x": 179640, "y": 221475, "nodeId": "25" },
          "delivery_point": { "x": 185000, "y": 230000, "nodeId": "156" },
          "material_info": {
            "material_id": "",
            "quantity": 0,
            "container_type": ""
          },
          "charge_parameters": {
            "min_battery_level": 20,
            "target_battery_level": 80,
            "max_charge_time": 0
          }
        }
      ]
    }
  ],
  "unassigned_tasks": ["TASK_3", "TASK_4"],
  "allocation_metadata": {
    "calculation_time": 1234,
    "optimization_score": 0.0,
    "constraints_violated": []
  },
  "path_planning_metadata": { /* 若启用路径规划，则存在 */ }
}
```

说明：

- `agv_assignments[]`：每个 AGV 的任务链。
  - `agv_id`：对应请求中的 `deviceId`。
  - `current_position`：基于 `Amr` 的当前位置推导出来的点（x/y/nodeId）。
  - `assigned_tasks[]`：
    - `task_id`：原始 `Task.messageId`。
    - `task_sequence`：在该 AGV 的任务链中的顺序，从 1 开始。
    - `task_type`：根据 `Task.taskType` 和子任务点类型推断（`MATERIAL_TRANSPORT`、`CHARGE` 等）。
    - `pickup_point` / `delivery_point` / `target_point`：对不同类型任务抽取的关键点（含 `x`/`y`/`nodeId`），方便业务方直接消费。
    - `charge_parameters`：对充电任务，根据 AMR 的上下限电量填充。
- `unassigned_tasks[]`：
  - 未能分配的任务 ID 列表（字符串形式）；若任务没有 messageId，则回退为 `"TASK_<index>"`。
- `allocation_metadata`：
  - `calculation_time`：总计算时间（毫秒）。
  - `optimization_score`：预留评分指标（目前为 0）。
  - `constraints_violated`：未满足的约束列表（目前为空或简单文本）。
  - 其它调度请求字段（如 `request_type`、`request_timestamp`）不会再输出，避免与文档不一致。

### 2.2 路径规划结果（path_plan）

当 `ResultPublisher` 收到非空 `MapInfo` 且静态表可用时，会在每个 `agv_assignments[*]` 下追加 `path_plan` 字段：

```jsonc
{
  "agv_assignments": [
    {
      "agv_id": "AGV01",
      "current_position": { "x": 179640, "y": 221475, "nodeId": "25" },
      "assigned_tasks": [ /* 同上 */ ],
      "path_plan": {
        "segments": [
          {
            "task_id": "TASK_1",
            "step_type": "PICKUP",         // PICKUP / DELIVERY / TARGET
            "sub_task_id": "TASK_1#1",
            "sub_task_sequence": 1,
            "from_node": 25,
            "to_node": 100,
            "reachable": true,
            "source": "STATIC_TABLE",      // STATIC_TABLE / MIXED_PLANNING / PURE_ASTAR / UNREACHABLE
            "distance_mm": 12345.0,
            "time_sec": 12.3,
            "nodes": [25, 30, 40, 100],
            "planning_time_ms": 0.8
          },
          {
            "task_id": "TASK_1",
            "step_type": "DELIVERY",
            "sub_task_id": "TASK_1#2",
            "sub_task_sequence": 2,
            "from_node": 100,
            "to_node": 156,
            "reachable": true,
            "source": "STATIC_TABLE",
            "distance_mm": 23456.0,
            "time_sec": 23.4,
            "nodes": [100, 120, 140, 156],
            "planning_time_ms": 0.7
          }
        ],
        "total_distance_mm": 35801.0,
        "total_time_sec": 35.7,
        "all_reachable": true
      }
    }
  ],
  "path_planning_metadata": {
    "map_file": "config/south_20260107.json",
    "static_table_file": "config/south_20260107_all.bin"
  }
}
```

- `segments[*].sub_task_id` / `segments[*].sub_task_sequence`：与输入 `subTasks` 中的具体子任务一一对应，用于在 `RobotTrailRequest` 中指定 `subTaskId` 时定位该段终点。

生成规则（核心逻辑）：

- 起点：
  - 初始为 `current_position.nodeId`（若不可用，则由 AMR 的 `x/y` 最近节点推导）。
  - 每规划完一段后，将 `currentNode` 更新为该段的终点。
- 对于每个已分配任务：
  - 如果 `task_type == "MATERIAL_TRANSPORT"`：
    - 使用 `pickup_point.nodeId` 和 `delivery_point.nodeId`；
    - 生成两段：
      - `step_type="PICKUP"`：`currentNode` → `pickupNode`
      - `step_type="DELIVERY"`：`pickupNode` → `deliveryNode`
  - 否则：
    - 使用 `target_point.nodeId`；
    - 单段：`step_type="TARGET"`：`currentNode` → `targetNode`
- 每一段调用 `GlobalPathPlanner::planGlobalPath(fromNode, toNode)`：
  - 若起止节点无效，标记为 `reachable=false` 且 `source="INVALID_NODE"`；
  - 否则根据静态表和 A* 结果填充 `reachable`、`source`、`distance_mm`、`time_sec`、`nodes`、`planning_time_ms`。
- `total_distance_mm` / `total_time_sec` 为所有段的距离/时间累加；
- `all_reachable` = 所有段 `reachable` 的逻辑与。

---

## 3. 小结：端到端数据流

### 3.1 外部链路（external_*_sender → external_receiver）

1. **external_*_sender**：
   - `external_task_sender` 发送调度请求（`AssignmentTaskRequest`）。
   - `external_status_sender` / `external_config_sender` / `external_map_sender` 分别发送状态、配置与地图。
   - `external_path_request_sender` / `external_trail_request_sender` 触发路径/轨迹请求。
   - 发往：
     - `EXT_EXCHANGE`（默认 `DispToAlgoExchange`）/
     - `EXT_QUEUE`（默认 `DispToAlgoQueue`）/
     - `EXT_ROUTING_KEY`（默认 `AssignmentTaskRequest`，fanout 下仍建议将队列绑定为 `#`）。
2. **external_receiver**：
   - 消费请求 → 解析 `candidateTasks` / `agvStatusList` → 构造内部 `Task` / `Amr`；
   - 调用 `CostMatrixGenerator` + `GreedyTaskAllocator` / `PostaTaskAllocator` 生成 `AllocationResult`；
   - 调用 `ResultPublisher` → 输出结构如第 2 节（带 `path_plan`）。

旧版 `allocation_sender/allocation_receiver` 已下线。

业务方只需对接：

- **请求格式**：1.1–1.3；
- **结果格式**：2.1 + 2.2；

即可在外部链路上复用同一套协议。
