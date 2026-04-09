# 运行说明与参数一览

目前仅保留“业务调度链路”：`external_receiver` + 多个专用 external sender。
旧版 `allocation_sender/allocation_receiver` 已下线，所有联调都应通过外部链路完成。

---

## 0. 编译

如果你把工程目录从 A 拷贝/移动到 B，旧的 `build/` 里的 CMake 产物可能仍然写着 A 的绝对路径，直接 `make -j` 会出现 `can't cd to .../build`。

推荐通过仿真一键脚本启动（`simulation_v2/run_all.sh`；需要自动编译时设置 `SIM_BUILD=1`）：

```bash
bash simulation_v2/run_all.sh
```

或手动：

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j9
```

业务侧模拟工具拆分为多个二进制，分别对应不同的业务 → 算法消息：

| 可执行文件（build/bin） | 作用 | RabbitMQ 路由键 |
|------------------------|------|----------------|
| `external_task_sender` | 发送 `AssignmentTaskRequest`（任务批量触发） | `AssignmentTaskRequest` |
| `external_status_sender` | 推送实时 `SendRobotStatusInfos` | `SendRobotStatusInfos` |
| `external_config_sender` | 推送/更新 `SendRobotConfigInfos` | `SendRobotConfigInfos` |
| `external_map_sender` | 发送地图 JSON（`SendMapInfo`） | `SendMapInfo` |
| `external_path_request_sender` | 触发获取全局路径 | `RobotPathRequest` |
| `external_trail_request_sender` | 触发获取轨迹控制点 | `RobotTrailRequest` |

推荐的启动方式：
- 仅启动 `external_receiver`（含日志滚动与落盘目录）：`bash script/start.sh`
- 启动 `external_receiver` + 在线仿真/可视化：`bash simulation_v2/run_all.sh`

> 下文所有命令均假定在工程根目录执行，二进制位于 `build/bin/`。

---

## 1. 业务调度链路：external_receiver + external_*_sender

| 环境变量          | 默认值           | 说明                         |
|------------------|-----------------|------------------------------|
| `EXT_EXCHANGE` | `DispToAlgoExchange` | 目标交换机（`fanout`） |
| `EXT_QUEUE` | `DispToAlgoQueue` | 队列名（声明并绑定） |
| `EXT_ROUTING_KEY` | `AssignmentTaskRequest` | 调度请求路由键 |

#### 1.2.2 地图与 AMR/任务来源

地图文件（external_map_sender）：

| 环境变量        | 默认值                               | 说明                                        |
|----------------|-------------------------------------|---------------------------------------------|
| `MAP_FILE` | `config/south_20260107.json` | 读取地图 JSON 并发送；未设置时，`PathUtils` 会在当前目录、父/祖先目录、`../..` 等常见根路径，以及（若定义）`$AGV_SCHED_ROOT(/config)` 下拼接并递归搜索同名文件。 |
| `AGV_SCHED_ROOT` | 无（可选）                         | 若设置，则把 `<root>` 与 `<root>/config` 加入搜索根列表，方便部署在固定前缀目录。 |

AMR 与任务数据：

- `external_task_sender`：优先读取 `script/generated_test_payload1.json`（若不存在则使用内置样例；可通过 `AGV_SCHED_ROOT` 搜索）。
- `external_status_sender`：读取 `STATUS_PAYLOAD_FILE`（默认 `script/status_simple_warehouse.json`）。
- `external_config_sender`：读取 `CONFIG_PAYLOAD_FILE`（默认内置样例）。

随机数种子：

| 环境变量    | 默认值 | 说明                       |
|------------|--------|----------------------------|
| `ALLOC_SEED` | `1337` | 随机数种子，可用于复现场景 |

---

## 1. 外部调度链路：external_receiver

### 1.1 external_receiver

- 可执行文件：`build/bin/external_receiver`
- 作用：监听外部业务交换机/队列（默认 topic），消费外部系统发来的调度请求，跑完整分配流程（Greedy + POSTA），打印结果，并使用 `ResultPublisher` 长连接版本将分配结果发布到结果队列。
- 典型用法：

```bash
cd build
./bin/external_receiver
```

该程序设计为**长时间运行**的守护式消费者：

- RabbitMQ 连接与消费侧 channel 在启动时建立，异常退出前保持。
- 结果发布连接由 `ResultPublisher` 管理，同样是持久连接，失败时自动重连。

#### 1.1.1 RabbitMQ 连接参数

通过 `RabbitMQConfigFromEnv({})` 读取：

| 环境变量    | 默认值      | 含义                     |
|------------|------------|--------------------------|
| `AMQP_HOST`| `localhost`| RabbitMQ 主机地址        |
| `AMQP_PORT`| `5672`     | 端口                     |
| `AMQP_USER`| `guest`    | 用户名                   |
| `AMQP_PASS`| `guest`    | 密码                     |
| `AMQP_VHOST`| `/`       | vhost                    |

#### 1.1.2 外部请求通道配置

在 `src/tools/external_receiver.cpp` 中：

| 环境变量          | 默认值                    | 说明                           |
|------------------|--------------------------|--------------------------------|
| `EXT_EXCHANGE`   | `DispToAlgoExchange`     | 外部调度请求交换机             |
| `EXT_QUEUE`      | `DispToAlgoQueue`        | 请求队列名                     |
| `EXT_BINDING_KEY`| `#`                      | 队列绑定键（fanout/通配所有消息） |
| `EXT_EXCHANGE_TYPE` | `fanout`              | 交换机类型                     |
| `EXT_TIMEOUT_SEC`| `30.0`                   | `amqp_consume_message` 等待超时 |
| `EXT_MESSAGE_TTL_MS` | 未设置               | 外部请求消息 TTL（毫秒，>0 时才声明 `x-message-ttl`） |
| `EXT_QUEUE_MAX_PRIORITY` | 未设置          | 队列优先级上限（>0 时才声明 `x-max-priority`，常用值 10） |

队列声明策略：

- 先被动声明队列：`amqp_queue_declare(..., passive=1)`。
- 若不存在或不兼容，则主动创建，若 `EXT_QUEUE_MAX_PRIORITY>0` 则附带 `x-max-priority`（常见取值 10）；仅在 `EXT_MESSAGE_TTL_MS>0` 时附带 `x-message-ttl`，否则保持与远端已存在队列一致：

```cpp
entries[0].key = "x-max-priority";
entries[0].value.kind = AMQP_FIELD_KIND_U8;
entries[0].value.value.u8 = 10;
```

算法端默认不会加载本地地图，必须等待业务方发送 `SendMapInfo`。收到后会用 `mapData` 重建 `MapInfo` 与内部 A*，并将原始 JSON 缓存在 `debug/received_map.json`（可通过 `MAP_CACHE_FILE` 覆盖），以便其他模块复用。地图到达之前收到的调度请求会直接拒绝。

当前版本支持按 `mapId` 维护多个独立的运行时上下文：每张地图有各自的状态缓存、计划缓存、动态预约与重规划逻辑。`SendMapInfo` 的地图重载会异步构建新快照，构建成功后再切换到新地图上下文，避免阻塞 MQ 消费线程。`RobotPathRequest` / `RobotTrailRequest` 可显式携带 `mapId`；若未携带，则会根据 `deviceId` 最近一次状态中的 `mapId` 自动路由。对外响应仍按原有 `messageId` 关联，不额外依赖 `mapId` 做匹配。

当动态地图导致区域重新划分、区域硬上限下降时，系统采用“**区内存量豁免，区外新进入禁止**”策略：已经位于该区域内的 AGV 不会因为新上限而被强制驱离；只有尝试从其它区域进入该超限区域的路径会被区域硬约束阻止。

#### 1.1.3 调度与路径请求的异步化

`external_receiver` 现在在主线程中只负责消费 MQ、解析消息，并把调度 payload 投递到内部的 `SchedulingTaskQueue`。该队列会根据 CPU 核心数（或 `SCHED_WORKERS` 环境变量）启动多个工作线程，每个线程独立调用 `processSchedulingMessage()` 完成“任务分配 → 路径规划 → 结果发布/缓存”的完整流程：

| 环境变量        | 默认值 | 说明                                                       |
|----------------|--------|------------------------------------------------------------|
| `SCHED_WORKERS`| `std::thread::hardware_concurrency()` | 调度工作线程数量。<=0 时回退到硬件线程数（至少 1）。 |

这样做的好处：

- **调度与路径查询并行**：`RobotPathRequest` / `RobotTrailRequest` 在主线程即时处理，直接从 `RobotDataRepository` 的最新计划缓存中取数据，不再等待调度流程结束。
- **隔离单次耗时**：即使某一个调度请求很大，也只占用一个 worker，不会阻塞 MQ 消费或其它请求。
- **计划复用**：每次调度完成都会把 `path_planning_results` 写回缓存，供异步的路径/轨迹查询复用；`PATH_REQ_WAIT_MS` 控制等待缓存刷新最长时间，默认 2000ms。

#### 1.1.4 AMR 预测参数

用于控制 AMR 最远可规划时间与最大绕路距离：

| 环境变量                 | 默认值 | 说明                                  |
|-------------------------|--------|---------------------------------------|
| `EXT_MAX_TIME_AHEAD_SEC`| `-1`   | 覆盖 `Amr.maxTimeAheadMs`，以秒配置，内部×1000；<0 表示使用默认600000ms |
| `EXT_MAX_DETOUR_MM`     | `-1`   | 覆盖 `Amr.maxDetourDistanceMm`；<0 表示使用默认策略 |

在外部 payload 中，`agvStatusList[].endurance` 按小时读取并直接写入 `Amr.endurance`（不做换算）；`batteryLevel` 若 ≤0 则回退为 100。

#### 1.1.5 调度请求超时与 POSTA 参数

调度参数与内部单机场景一致：

| 环境变量           | 默认值 | 说明                               |
|-------------------|--------|------------------------------------|
| `ALLOC_WORKERS`   | `4` | 最短路计算线程数；外部紧急预测阶段若未设该变量，会用 2 线程做临时估算 |
| `ALLOC_POSTA_SE`  | `30`   | POSTA 迭代步数上限                |
| `ALLOC_POSTA_SWEEPS` | `10`| POSTA sweep 次数                  |
| `ALLOC_PRI_COEFF` | `0.02` | 优先级惩罚系数                    |
| `ALLOC_TOPK_RATIO`| `0.5`  | Top-K 稀疏近似比率                |
| `ALLOC_TOPK`      | `-1`   | Top-K 上限（>0 时生效，否则自动） |
| `ALLOC_TOPK_AGENT`| `-1`   | 每个 AMR 的 Top-K 数量            |
| `ALLOC_TOPK_NEXT` | `-1`   | 后继任务 Top-K 数量               |

请求中的 `timeoutMs` 决定调度时间预算：

- 先跑 Greedy 得到初始解，记录耗时 `greedyElapsed`。
- 剩余 POSTA 时间 `remainSec = max(0.01, timeoutMs/1000.0 - greedyElapsed - 0.01)`。

#### 1.1.6 结果发布（持久连接）

`external_receiver` 在 `main()` 中创建一个长生命周期的 `ResultPublisher`：

```cpp
RabbitMQConfig cfg = RabbitMQConfigFromEnv({});
ResultPublisher resultPublisher(cfg);
```

每次分配完成后调用：

```cpp
bool ok = resultPublisher.publish(res, availableAmrs, taskList, &mapInfo,
                                  calc_ms, opt_score, constraints);
```

`ResultPublisher` 内部行为：

- 第一次 publish 时建立 RabbitMQ 连接 + channel 1，并声明结果交换机/队列并绑定；
- 之后复用同一连接和 channel 发送所有结果；
- 若 `amqp_basic_publish` 返回错误，则关闭连接，下一次 publish 会自动重连。

结果通道配置：

| 环境变量                  | 默认值                | 说明                      |
|--------------------------|----------------------|---------------------------|
| `ASSIGN_RESULT_EXCHANGE` | `AlgoToDispExchange` | 结果交换机（`fanout`）    |
| `ASSIGN_RESULT_QUEUE`    | `AlgoToDispQueue`    | 结果队列（持久化） |
| `ASSIGN_RESULT_ROUTING_KEY` | `AssignmentTaskResponse` | 结果路由键           |
| `ASSIGN_RESULT_BINDING_KEY` | `#`                | 队列绑定键               |
| `ASSIGN_RESULT_TTL_MS`   | 未设置               | 结果消息过期时间（毫秒，>0 时才声明 `x-message-ttl`） |

> 静态路径表仍需要通过 `PLANNER_STATIC_TABLE` 指定本地文件；即便地图通过 `SendMapInfo` 下发，也必须保证静态表与地图保持一致。

---

### 1.2 external_*_sender

为方便逐条调试不同业务报文，模拟工具拆分为多条命令：

| 可执行文件 | 默认路由键 | 描述 |
|------------|-----------|------|
| `external_task_sender` | `AssignmentTaskRequest` | 发送调度任务批量请求（可从 `script/generated_test_payload1.json` 读取自定义样例） |
| `external_status_sender` | `SendRobotStatusInfos` | 发送机器人状态集（200 ms 监测可通过循环脚本实现） |
| `external_config_sender` | `SendRobotConfigInfos` | 发送/更新机器人配置（可通过 `CONFIG_PAYLOAD_FILE` 填充自定义 JSON） |
| `external_map_sender` | `SendMapInfo` | 读取 `MAP_FILE` 指定的地图 JSON，包装后发送 |
| `external_path_request_sender` | `RobotPathRequest` | 发送获取全局路径的请求（`PATH_REQUEST_DEVICE_ID` 指定设备） |
| `external_trail_request_sender` | `RobotTrailRequest` | 发送获取轨迹控制点的请求（`TRAIL_REQUEST_DEVICE_ID` 指定设备，`TRAIL_REQUEST_SUBTASK_ID=TASK_001#2` 可选，仅返回对应子任务节点前的轨迹） |

所有 sender 共享以下 MQ 参数（若不设置则采用 `RabbitMQConfig` 默认）：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `EXT_EXCHANGE` | `DispToAlgoExchange` | 目标交换机 |
| `EXT_QUEUE` | `DispToAlgoQueue` | 监听队列 |
| `EXT_BINDING_KEY` | `#` | 队列绑定键 |
| `EXT_EXCHANGE_TYPE` | `fanout` | 交换机类型 |
| `EXT_MESSAGE_TTL_MS` | 未设置 | 消息 TTL（>0 时才声明 `x-message-ttl`） |
| `EXT_QUEUE_MAX_PRIORITY` | 未设置 | 队列优先级上限（>0 时才声明 `x-max-priority`） |

> `external_task_sender` 支持从 `script/generated_test_payload1.json` 读取调度请求；其余 sender 可通过 `*_PAYLOAD_FILE` 指定自定义 JSON。

若希望“一键”执行初始化 + 持续任务 + 在线可视化，直接运行：

```bash
bash simulation_v2/run_all.sh
```

如需手动分步联调，请按需运行 `build/bin/external_*_sender`（配置 → 地图 → 状态 → 任务），并保证与 `external_receiver` 使用相同的 exchange/queue/routing_key。

---

## 3. 快速启动组合示例

### 3.1 业务链路自测

```bash
cd build
# 终端1：长时间运行外部 receiver
../script/start.sh

# 终端2：发送业务模拟消息
# 按需单独运行 external_*_sender（配置->地图->状态->任务）：
./bin/external_config_sender
./bin/external_map_sender
./bin/external_status_sender
./bin/external_task_sender
```

`external_receiver` 会在控制台打印调度结果、未分配原因，并通过 `ResultPublisher` 长连接发布结果，可用其他服务订阅 `ASSIGN_RESULT_*` 队列进行联调。

