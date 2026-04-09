# 环境变量总览（ENV）

本文汇总本工程会读取的环境变量，按模块分组，方便联调/排障/仿真调参。

优先级（从高到低）：
1) 程序命令行参数（如 `simulation_v2/sim_runner_v2.py --vis-port ...`）
2) 环境变量（`export XXX=...`）
3) `config/network.env` 中的默认值
4) 程序内部默认值（C++/Python）

> 推荐做法：先 `source config/network.env`，再按需 `export` 覆盖。

---

## 1) RabbitMQ 连接

| 变量 | 默认值 | 说明 |
|---|---|---|
| `RABBITMQ_URL` | `amqp://${AMQP_USER}:${AMQP_PASS}@${AMQP_HOST}:${AMQP_PORT}/%2F` | 单一连接 URL（若设置则优先生效）。默认 `%2F` 表示 vhost `/`。如需自定义 vhost，建议直接设置该变量。 |
| `AMQP_HOST` | `127.0.0.1` | RabbitMQ 主机。 |
| `AMQP_PORT` | `5672` | RabbitMQ 端口。 |
| `AMQP_USER` | `gms` | 用户名。 |
| `AMQP_PASS` | `123456` | 密码。 |
| `AMQP_VHOST` | `/` | vhost（仅在部分组件/工具中使用；若你设置了非 `/`，请同步设置 `RABBITMQ_URL`）。 |

---

## 2) MQ 通道与 Routing Key（业务/算法/交通）

这些变量默认由 `config/network.env` 提供，所有值都可在运行前覆盖。

### 2.1 外部调度请求通道（Disp → Algo）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `EXT_EXCHANGE` | `DispToAlgoExchange` | 外部请求交换机。 |
| `EXT_EXCHANGE_TYPE` | `fanout` | 交换机类型。 |
| `EXT_QUEUE` | `DispToAlgoQueue` | 外部请求队列。 |
| `EXT_ROUTING_KEY` | `AssignmentTaskRequest` | 外部请求 routing key。 |
| `EXT_BINDING_KEY` | `#` | 队列绑定 key（通配）。 |
| `EXT_TIMEOUT_SEC` | `30` | 消费侧等待超时（秒）。 |
| `EXT_MESSAGE_TTL_MS` | （未设置） | 如设置且 >0，声明队列时会带 `x-message-ttl`。 |
| `EXT_QUEUE_MAX_PRIORITY` | （未设置） | 如设置且 >0，声明队列时会带 `x-max-priority`（常用 10）。 |

### 2.2 分配结果通道（Algo → Disp）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `ASSIGN_RESULT_EXCHANGE` | `AlgoToDispExchange` | 分配结果交换机。 |
| `ASSIGN_RESULT_QUEUE` | `AlgoToDispQueue` | 分配结果队列。 |
| `ASSIGN_RESULT_ROUTING_KEY` | `AssignmentTaskResponse` | 分配结果 routing key。 |
| `ASSIGN_RESULT_BINDING_KEY` | `#` | 队列绑定 key。 |
| `ASSIGN_RESULT_TTL_MS` | （未设置） | 如设置且 >0，声明队列时会带 `x-message-ttl`。 |

### 2.3 Algo publisher（路径/轨迹/心跳等，Algo → Disp）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `ALGO_PUBLISH_EXCHANGE` | `AlgoToDispExchange` | Algo publisher 交换机。 |
| `ALGO_PUBLISH_QUEUE` | `AlgoToDispQueue` | Algo publisher 队列。 |
| `ALGO_PUBLISH_BINDING_KEY` | `#` | 队列绑定 key。 |
| `ALGO_PUBLISH_TTL_MS` | （未设置） | 如设置且 >0，声明队列时会带 `x-message-ttl`。 |
| `ALGO_INIT_ROUTING_KEY` | `AlgoServiceInitialize` | Algo 初始化消息 routing key。 |
| `ALGO_HEARTBEAT_ROUTING_KEY` | `AlgoServiceHeartbeat` | Algo 心跳 routing key。 |
| `ALGO_RUNINFO_ROUTING_KEY` | `SendRunInfo` | 运行信息 routing key。 |
| `ALGO_ALERT_ROUTING_KEY` | `SendAlertInfo` | 告警 routing key。 |
| `ALGO_PATH_RESPONSE_ROUTING_KEY` | `RobotPathResponse` | 路径响应 routing key。 |
| `ALGO_TRAIL_RESPONSE_ROUTING_KEY` | `RobotTrailResponse` | 轨迹响应 routing key。 |
| `TRAIL_PUBLISH_INTERVAL_MS` | `0` | 周期性 trail 推送间隔（毫秒）；<=0 表示不做周期推送（默认关闭，仅响应 `RobotTrailRequest`）。 |
| `ALGO_SERVICE_NAME` | （程序内部默认） | Algo 服务名（用于心跳/标识；仅部分链路使用）。 |

### 2.4 Traffic path 通道（Algo → Traffic）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `TRAFFIC_PATH_EXCHANGE` | `AlgoToTrafficExchange` | Traffic path 交换机。 |
| `TRAFFIC_PATH_QUEUE` | `AlgoToTrafficQueue` | Traffic path 队列。 |
| `TRAFFIC_PATH_ROUTING_KEY` | `SendPathInfo` | routing key。 |
| `TRAFFIC_PATH_BINDING_KEY` | `#` | 绑定 key。 |
| `TRAFFIC_PATH_TTL_MS` | （未设置） | 如设置且 >0，声明队列时会带 `x-message-ttl`。 |
| `TRAFFIC_PATH_PUBLISH_INTERVAL_MS` | `100` | dynamic traffic path 周期推送间隔（毫秒）；<=0 表示不做周期推送。 |

### 2.5 业务侧地图/状态与请求 routing key

| 变量 | 默认值 | 说明 |
|---|---|---|
| `MAP_ROUTING_KEY` | `SendMapInfo` | 地图消息 routing key。 |
| `STATUS_ROUTING_KEY` | `SendRobotStatusInfos` | 状态消息 routing key。 |
| `PATH_REQUEST_ROUTING_KEY` | `RobotPathRequest` | 路径请求 routing key。 |
| `TRAIL_REQUEST_ROUTING_KEY` | `RobotTrailRequest` | 轨迹请求 routing key。 |

### 2.6 可选：仿真 Pose 流（Algo → Sim/Other）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `ENABLE_SIM_POSE_STREAM` | `0` | 1=开启 Pose 流（默认关闭）。 |
| `SIM_POSE_INTERVAL_MS` | `200` | Pose 发布周期（毫秒）。 |
| `SIM_POSE_EXCHANGE` | `AlgoSimExchange` | 交换机。 |
| `SIM_POSE_EXCHANGE_TYPE` | `fanout` | 交换机类型。 |
| `SIM_POSE_QUEUE` | `AlgoSimQueue` | 队列。 |
| `SIM_POSE_ROUTING_KEY` | `SimPose` | routing key。 |
| `SIM_POSE_BINDING_KEY` | `#` | binding key。 |

---

## 3) 一键仿真：`simulation_v2/run_all.sh`

### 3.1 MQ 隔离（避免和其他人/其他进程冲突）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `SIM_USE_SHARED_MQ` | `0` | 0=默认隔离（用 per-run 独立 exchange/queue）；1=使用 `config/network.env` 的共享配置。 |
| `SIM_SESSION` | `$(date +%s)` | 隔离模式下用于生成 exchange/queue 后缀。 |

隔离模式下会强制覆盖：
- `EXT_EXCHANGE/EXT_QUEUE`
- `ASSIGN_RESULT_EXCHANGE/ASSIGN_RESULT_QUEUE`
- `ALGO_PUBLISH_EXCHANGE/ALGO_PUBLISH_QUEUE`（对齐到 ASSIGN_RESULT）

### 3.2 地图与规模

| 变量 | 默认值 | 说明 |
|---|---|---|
| `MAP_FILE` | `config/grid_20x20.json` | 仿真使用的地图 JSON。 |
| `SIM_AGV_COUNT` | `50` | 生成 AGV 数量。 |
| `SIM_TASK_COUNT` | `SIM_AGV_COUNT * SIM_TASK_MULTIPLIER` | 启动时一次性生成的初始任务数。 |
| `SIM_TASK_MULTIPLIER` | `3` | 任务倍数（当 `SIM_TASK_COUNT` 未显式设置时生效）。 |
| `SIM_SEED` | `$(date +%s)` | 随机种子。 |
| `SIM_MAP_VERSION` | `${MAP_VERSION:-sim-map}` | Map version 字符串（业务消息里带）。 |
| `SIM_MAP_ID` | `0` | Map ID（状态消息里带）。 |

### 3.3 仿真节拍与请求频率

| 变量 | 默认值 | 说明 |
|---|---|---|
| `SIM_STEP_INTERVAL` | `0.1` | 仿真 tick 秒数。 |
| `SIM_PATH_INTERVAL` | `0` | 全量路径请求周期（0=关闭）。 |
| `SIM_TRAIL_INTERVAL` | `1.0` | 全量轨迹请求周期。 |
| `SIM_ASSIGNED_PATH_INTERVAL` | `0` | 已接单 AGV 路径请求周期（0=关闭）。 |
| `SIM_ASSIGNED_TRAIL_INTERVAL` | `0.5` | 已接单 AGV 轨迹请求周期。 |

### 3.4 任务持续注入（降低空闲率）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `SIM_LOOP_TASKS` | `1` | 1=持续发送新任务；0=只发启动时那一批。 |
| `SIM_LOOP_MODE` | `any-idle` | `any-idle`/`all-idle`/`timer`（触发策略）。 |
| `SIM_LOOP_TASK_MIN_COUNT` | `ceil(agv/4)` | 每次注入任务的最小数量。 |
| `SIM_LOOP_TASK_MAX_COUNT` | `ceil(agv/2)` | 每次注入任务的最大数量。 |
| `SIM_LOOP_TASK_MIN_DELAY` | `0.3` | 两次注入之间最小延迟（秒）。 |
| `SIM_LOOP_TASK_MAX_DELAY` | `0.8` | 两次注入之间最大延迟（秒）。 |
| `SIM_TASK_PRIORITY_MIN` | `1` | 生成任务的最小优先级。 |
| `SIM_TASK_PRIORITY_MAX` | `9` | 生成任务的最大优先级；默认不产生紧急任务（≥10）。 |
| `SIM_FILTER_OCCUPIED_TASKS` | `1` | 1=生成候选任务时，过滤“终点在当前占用节点”的任务。 |
| `SIM_FILTER_TASK_OVERLAP` | `1` | 1=同一批候选任务中，终点节点去重（避免不同 AGV 的任务点重合）。 |

### 3.5 Web 可视化

| 变量 | 默认值 | 说明 |
|---|---|---|
| `SIM_NO_VIS` | `0` | 1=禁用 Web 可视化（仍跑仿真）。 |
| `SIM_V2_VIS_HOST` | `127.0.0.1` | Web 服务 bind host。 |
| `SIM_V2_VIS_PORT` | `18080` | Web 服务端口。 |

### 3.6 构建与运行细节

| 变量 | 默认值 | 说明 |
|---|---|---|
| `SIM_BUILD` | `0` | 1=运行前编译 `external_receiver` 到 `build_local/`。 |
| `BUILD_JOBS` | `9` | `cmake --build -j` 并行度（仅 `SIM_BUILD=1` 时使用）。 |
| `PYTHON_BIN` | `python3` | Python 解释器。 |
| `SIM_BIND_TASKS` | `1` | 1=生成任务时 round-robin 绑定到设备（`agvRequirements`）。 |
| `SIM_BIND_TASKS_RATIO` | `1.0` | `SIM_BIND_TASKS=1` 时，保留绑定的任务比例（0..1）。 |
| `SIM_RESTART_ON_EXIT` | `0` | 1=仿真退出后自动重启。 |

### 3.7 区域划分（可视化/拥堵）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `CONGESTION_REGION_MODE` | `graph` | 区域划分模式：`grid`（按坐标网格）、`graph/cluster/partition`（按拓扑扩散）。 |
| `CONGESTION_REGION_GRID` | `0` | 网格边数；<=0 则自动估算。 |
| `CONGESTION_REGION_GRID_MIN` | `0` | 自动估算的最小网格边数（<=0 不限制）。 |
| `CONGESTION_REGION_GRID_MAX` | `0` | 自动估算的最大网格边数（<=0 不限制）。 |
| `CONGESTION_REGION_COUNT` | `0` | 仅 `graph/cluster/partition` 生效；<=0 时默认用 `grid*grid`。 |
| `CONGESTION_REGION_TARGET_NODES` | `25` | 自动估算时“每个区域期望的可通行节点数”；越大区域越少。 |
| `CONGESTION_REGION_SOFT` | `0` | 区域软上限（<=0 按 AGV 数估算）。 |
| `CONGESTION_REGION_HARD` | `0` | 区域硬上限（<=0 按 AGV 数估算）。 |

---

## 4) 仿真运动学参数：`simulation_v2/sim_runner_v2.py`

这些变量用于“连续速度 + 转弯停车 + 转向耗时”的仿真运动模型（也可用 CLI 参数覆盖）。

| 变量 | 默认值 | 说明 |
|---|---|---|
| `SIM_DRIVE_MODE` | `trail` | `trail`=仅按 RobotTrailResponse 驱动；`path`=忽略 trail，仅按 RobotPathResponse 驱动。 |
| `SIM_MAX_SPEED_M_S` | `0` | 0=使用地图 `maxSpeed`；>0 覆盖最大速度（m/s）。 |
| `SIM_ACCEL_M_S2` | `0.5` | 加速度上限（m/s²）。 |
| `SIM_DECEL_M_S2` | `0.5` | 减速度上限（m/s²）。 |
| `SIM_TURN_SLOWDOWN` | `0.7` | 转弯限速强度（0..1，仅对“非停车转弯”节点限速有意义）。 |
| `SIM_TURN_MIN_FACTOR` | `0.3` | 180° 转弯的最小速度系数。 |
| `SIM_TURN_STOP_ANGLE_DEG` | `1` | 方向变化 ≥ 该角度时：停车并原地转向。 |
| `SIM_TURN_RATE_DEG_S` | `90` | 原地转向角速度（deg/s）。 |
| `SIM_TURN_MIN_TIME_S` | `0.2` | 原地转向最小耗时（秒）。 |
| `SIM_TRAIL_END_STOP` | `1` | trail 驱动时是否把“最后一个 trail 点”当作必须停车点：1=末点 `v_end=0`（更安全但窗口滚动时会轻微减速）；0=把末点当作 lookahead（更平滑；若末点恰好是当前 subTask 目标点仍会停车）。 |
| `SIM_TRAIL_MIN_DRIVE_POINTS` | `2` | trail 点数 <= 该值时触发刹停逻辑（仿真侧）；`simulation_v2/run_all.sh` 默认设置为 3。 |
| `SIM_TRAIL_BRAKE_WINDOW_M` | `0` | trail 总长度 < 该阈值时按比例降低 `v_max` 以提前减速（单位 m）；0=自动按 `2 * (v_max^2 / 2a)` 计算阈值。 |
| `SIM_SUBTASK_REACH_TOL_MM` | `50` | 子任务点判定“到达”的距离阈值（mm）；距离超过该值时不会消费/移除子任务点。 |

> 注：`simulation_v2/run_all.sh` 为了更平滑的滚动窗口演示，默认会把 `SIM_TRAIL_END_STOP` 设为 `0`（如需安全停靠语义可手动覆盖为 `1`）。

---

## 5) external_receiver 启动与日志：`script/start.sh`

### 5.1 日志滚动（防止单文件过大）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `DEBUG_DIR` | `debug/` | 日志与落盘目录（相对路径会自动转为绝对）。仿真默认会覆盖为 `simulation_v2/debug/`。 |
| `RECEIVER_LOG_ROTATE_SEC` | `300` | 日志滚动周期（秒）。 |
| `RECEIVER_LOG_KEEP` | `30` | 最多保留多少个滚动文件。 |
| `RECEIVER_LOG_STDOUT` | `1` | 1=同时输出到控制台；0=只写文件。 |
| `RECEIVER_LOG_PREFIX` | `external_receiver` | 日志文件名前缀（`<prefix>_YYYYmmdd_HHMMSS.log`）。 |

### 5.2 external_receiver 日志详细度（推荐默认关闭）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `RECEIVER_LOG_EACH_MESSAGE` | `0` | 1=打印每条收到消息的 size/routing_key，以及（若 payload 含字段）`messageId/deviceId/subTaskId`（量很大，谨慎开启）。 |
| `RECEIVER_LOG_EACH_MESSAGE_STATUS` | `0` | 1=在 `RECEIVER_LOG_EACH_MESSAGE=1` 时也打印状态消息（`SendRobotStatusInfos`/可识别的状态数组）；默认关闭避免状态流刷屏。 |
| `RECEIVER_LOG_STATUS_SUMMARY` | `0` | 1=打印状态处理的 summary（如 `[Status] updated ...` / `[Status] cached ...` / `agvStatusList count=...`）；状态流频繁，默认关闭。 |
| `RECEIVER_LOG_STATUS_DETAIL` | `0` | 1=额外打印 task payload 中 `agvStatusList` 的每车明细（非常长）。 |
| `RECEIVER_LOG_ROUTE_SUMMARY` | `0` | 1=打印 path/trail 响应的 summary；同时 trail 请求会额外打印一行 `[TrailReq]`（含 subTaskId）。 |
| `RECEIVER_LOG_TRAIL_POINTS` | `0` | 1=打印每条 `RobotTrailResponse` 的所有点（nodeId/x/y）。 |
| `RECEIVER_LOG_TRAIL_POINTS_MAX` | `200` | `RECEIVER_LOG_TRAIL_POINTS=1` 时最多打印多少个点（防止极端超长日志）。 |
| `RECEIVER_LOG_ROUTE_DETAIL` | `0` | 1=打印 path/trail 的逐点细节 + trail 截断/锚点调试信息（非常长）。 |
| `RECEIVER_LOG_TRAFFIC_SUMMARY` | `0` | 1=在 stdout 打印 traffic path summary（写入 `external_receiver.log`）。 |
| `RECEIVER_LOG_TRAFFIC_JSON` | `0` | 1=在 stdout 追加 traffic JSON（同时包含 summary）。 |
| `RECEIVER_LOG_TRAFFIC_DETAIL` | `0` | 1=打印 traffic path 的节点明细（nodeId/x/y，含 summary）。 |

### 5.2 结果落盘位置（external_receiver）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `RESULT_DUMP_DIR` | `DEBUG_DIR` | `allocation_result_*.json` 落盘目录。 |
| `RESULT_DUMP_KEEP_MAX` | `30` | `allocation_result_*.json` 最大保留数量（<=0 表示不裁剪）。 |
| `TRAFFIC_DUMP_KEEP_MAX` | `0` | `traffic_path_*.json` 最大保留数量（落盘目录同 `RESULT_DUMP_DIR`；<=0 表示不落盘）。 |
| `MAP_CACHE_FILE` | `${RESULT_DUMP_DIR}/received_map.json` | 接收的地图缓存路径。 |

### 5.3 external_receiver 行为开关（常用）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `TS_MAX_SKEW_SEC` | `100000000`（脚本默认） | 允许的时间戳偏差（秒）。调试可放大；严格联调请设回较小值（如 0.5）。 |
| `SKIP_STATIC_TABLE` | `1`（脚本默认） | 1=跳过静态表（纯 A*）；0=启用静态表/混合规划。 |
| `REPLAN_INTERVAL_MS` | `1000` | 兼容旧配置保留；当前节流固定使用 `REPLAN_TICK_MS`。 |
| `REPLAN_TICK_MS` | `200` | 动态重规划执行周期（毫秒）；同时作为固定的重规划间隔/等待阈值基准。 |
| `REPLAN_MAJOR_STUCK_MS` | `2000` | 连续静止达到该阈值时触发“大重规划”（A* 会避让预约/硬阻塞）；<0 禁用。 |
| `REPLAN_SUPER_AFTER_MAJOR_FAILS` | `3` | 大重规划连续不可达次数达到该值后触发“超级重规划”（临时目标点）。 |
| `REPLAN_FAST_HELD_PENALTY_MS` | `1000` | FAST 阶段 A* 若路径经过他车预约节点，则每次经过增加的时间惩罚（毫秒）。用于弱规避他车预约；MAJOR/SUPER 不使用。 |
| `REPLAN_FIRST_HOP_COOLDOWN_MS` | `2000` | 第一跳换邻居冷却（毫秒）。同一车在同一 start 节点内，未过冷却不会切换到其他邻居；<=0 禁用。 |
| `REPLAN_FIRST_HOP_ENABLE` | `0` | 是否允许“第一跳改邻居”解死锁；0=关闭。 |
| `REPLAN_FIRST_HOP_WAIT_MS` | `0` | 需要静止达到该时长后才允许“第一跳改邻居”（毫秒）；0=不等待。 |
| `REPLAN_TEMP_GOAL_BAN_MS` | `10000` | 临时目标点黑名单保留时长（毫秒）。达到临时目标或临时目标不可达时加入黑名单；<=0 禁用。 |
| `STATUS_STATIC_POS_EPS_MM` | `10` | 静止判定的坐标阈值（mm）。 |
| `STATUS_STATIC_SPEED_EPS` | `0.01` | 静止判定的速度阈值（mm/s）。 |
| `STATUS_STATIC_YAW_EPS_DEG` | `-1` | 静止判定的角度阈值（deg）；<0 表示不检查转向。 |
| `PLANNER_STATIC_TABLE` | `config/south_20260107_all.bin` | 静态表路径。 |
| `DEBUG_INCLUDE_PATH_RESULT` | `1`（脚本默认） | 1=在落盘 JSON 中包含规划结果字段（用于可视化/排障）。 |

---

## 6) 外部 sender 工具的输入文件

这些变量主要被 `external_*_sender` 二进制读取（便于离线/手动联调）。

| 变量 | 默认值 | 说明 |
|---|---|---|
| `MAP_FILE` | 见工具内部/文档 | `external_map_sender` 读取并发送的地图 JSON。 |
| `STATUS_PAYLOAD_FILE` | 见工具内部/文档 | `external_status_sender` 读取并发送的状态 JSON。 |
| `CONFIG_PAYLOAD_FILE` | 见工具内部/文档 | `external_config_sender` 读取并发送的配置 JSON。 |
| `CONFIG_ROUTING_KEY` | （程序内部默认） | 配置消息 routing key（少数环境需要覆盖）。 |

---

## 7) Path/Trail 请求与返回长度

| 变量 | 默认值 | 说明 |
|---|---|---|
| `PATH_REQ_WAIT_MS` | `2000` | `RobotPathRequest/RobotTrailRequest` 等待缓存计划的窗口（毫秒）。 |
| `TRAIL_MAX_POINTS` | `10` | 未提供 `subTaskId` 时，trail 返回的最大节点数。 |
| `TRAIL_MIN_POINTS` | `2` | trail 最少返回多少个控制点（节点数）。用于兼容“控制侧不接受 0/1 点或 start==end 的轨迹”的场景；不足时会尝试补一个邻接节点或补一个极小的 epsilon 位移点。设为 `1` 可恢复旧行为。 |
| `TRAIL_WAIT_SPEED_EPS` | `1` | 当 trail 仅剩 1 个点时，速度小于等于该阈值（mm/s）视为等待并跳过响应；大于该值会尽量返回最小 2 点轨迹。 |
| `TRAIL_DEGENERATE_USE_NEIGHBOR` | `0` | 1=当轨迹退化（仅 1 个节点）且需要补点时，优先补一个真实邻接节点（nodeId 不同）；0=默认仅补 epsilon 位移点（nodeId 不变，更适合“停住但必须 ≥2 点”的控制器）。 |
| `TRAIL_DEGENERATE_SPEED_MM_S` | `0` | 当触发 `TRAIL_MIN_POINTS` 补点时，将所有点的 `speed/maxSpeed` 强制覆盖为该值（默认 0=停住）。 |
| `TRAIL_DEGENERATE_EPS_MM` | `1` | 当地图邻接关系不可用时，用于构造“epsilon 位移点”的偏移量（mm），避免 2 点完全重合。 |
| `PATH_REQUEST_MESSAGE_ID` | （工具内部默认） | `external_path_request_sender` 的 messageId 覆盖。 |
| `PATH_REQUEST_DEVICE_ID` | （工具内部默认） | `external_path_request_sender` 的 deviceId 覆盖。 |
| `TRAIL_REQUEST_MESSAGE_ID` | （工具内部默认） | `external_trail_request_sender` 的 messageId 覆盖。 |
| `TRAIL_REQUEST_DEVICE_ID` | （工具内部默认） | `external_trail_request_sender` 的 deviceId 覆盖。 |
| `TRAIL_REQUEST_SUBTASK_ID` | （未设置） | 若设置，trail 会优先截断到该子任务点（含该节点）。 |

---

## 8) 高级调参（分配/规划/预约/临时目标）

这部分变量较多，通常只在性能调优或排障时使用。推荐先看：
- `doc/ALLOCATION_PARAMETERS.md`

### 8.1 分配算法/成本矩阵（POSTA/Greedy/最短路）

常见项（详解与默认值见 `doc/ALLOCATION_PARAMETERS.md`）：
- `ALLOC_TIMEOUT_MS`
- `ALLOC_SEED`
- `ALLOC_WORKERS`
- `ALLOC_TOPK` / `ALLOC_TOPK_RATIO` / `ALLOC_TOPK_AGENT` / `ALLOC_TOPK_NEXT` / `ALLOC_TOPK_COVERAGE`
- `ALLOC_POSTA_SE` / `ALLOC_POSTA_SWEEPS` / `ALLOC_PRI_COEFF`
- `ALLOC_DIAG` / `ALLOC_COST_SPARSE`
- `SCHED_WORKERS`

### 8.2 Detour / 链路截断

| 变量 | 默认值 | 说明 |
|---|---|---|
| `EXT_MAX_TIME_AHEAD_SEC` | `-1` | 覆盖 detour 时间窗（秒）；<0 表示不覆盖，沿用内部默认。 |
| `EXT_MAX_DETOUR_MM` | `-1` | 覆盖 detour 距离阈值（mm）；<0 表示不覆盖。 |

### 8.3 资源预约/节点占用（共享数据）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `RESERVE_TTL_MS` | （程序内部默认） | 预约记录 TTL（毫秒）。 |
| `RESERVE_CLEANUP_INTERVAL_MS` | （程序内部默认） | 预约清理周期（毫秒）。 |
| `RESERVE_BASE_NODES` | （程序内部默认） | 预约长度基准值（节点数）。 |
| `RESERVE_MIN_NODES` | （程序内部默认） | 最小预约长度（节点数）。 |
| `RESERVE_MAX_NODES` | （程序内部默认） | 最大预约长度（节点数）。 |

### 8.4 临时目标（规划失败/长时间等待）

| 变量 | 默认值 | 说明 |
|---|---|---|
| `TEMP_GOAL_CANDIDATES` | （程序内部默认） | 临时目标候选数量/范围（实现相关）。 |
| `TEMP_GOAL_NEAR_MM` | `8000` | 临时目标优先从主目标附近选择的半径（mm）；<=0 表示关闭“靠近目标”的偏好。 |

### 8.5 其他常见开关

| 变量 | 默认值 | 说明 |
|---|---|---|
| `STATUS_HOLD_TTL_MS` | （程序内部默认） | 状态保持/过期阈值（毫秒）。 |
| `SPU_DISABLE_PREFILTER` | （程序内部默认） | 预筛阶段是否禁用最短路（SPU）相关逻辑。 |
| `EXT_PRINT_UNALLOC_CAUSE` | （程序内部默认） | 打印未分配原因（调试用）。 |
| `DEBUG_SANITIZE` | （程序内部默认） | 调试输出脱敏/裁剪（调试用）。 |

---

## 9) 其他（系统环境）

这些不是本工程自定义变量，但会影响部分组件行为：
- `DISPLAY` / `WAYLAND_DISPLAY`：GUI/可视化相关（本仓库的 `simulation_v2` 为 Web 可视化，不依赖它们）。
- `MPLBACKEND`：Matplotlib 后端选择（仅 legacy/开发场景可能用到）。
