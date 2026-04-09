**目的**
- 统一管理外部联调（前端 Node.js ↔ 本项目 C++ 接收器）所需的网络/RabbitMQ 参数。
- 与 `a/前端测试/server.js` 的默认值保持对齐，减少环境不一致导致的联调问题。

**快速开始**
- 编辑并加载环境文件：
  - `config/network.env` 已提供默认值，与前端保持一致。
  - 运行前在终端执行：`source config/network.env`。

**关键参数**
- RabbitMQ 服务端：
  - `AMQP_HOST`、`AMQP_PORT`、`AMQP_USER`、`AMQP_PASS`、`AMQP_VHOST`
  - 也支持单一 URL：`RABBITMQ_URL`（如 `amqp://guest:guest@192.168.1.10:5672/%2F`）。
    - 若设置 `RABBITMQ_URL`，会覆盖 `AMQP_*` 参数。
- 地图文件（external_map_sender 读取）：
  - `MAP_FILE` 指向 JSON 地图；external_receiver 必须先收到 `SendMapInfo` 才能调度。
- 外部（前端）通道：必须与前端一致
  - `EXT_EXCHANGE=DispToAlgoExchange`
  - `EXT_EXCHANGE_TYPE=fanout`
  - `EXT_QUEUE=DispToAlgoQueue`
  - `EXT_ROUTING_KEY=AssignmentTaskRequest`
  - `EXT_BINDING_KEY=#`（队列绑定键，默认通配所有消息）
  - `EXT_TIMEOUT_SEC=30`
  - `EXT_MESSAGE_TTL_MS`（可选，默认不设置 TTL）
  - `EXT_QUEUE_MAX_PRIORITY`（可选，默认不声明 `x-max-priority`，需时可设为 10）
- 内部通道（旧版 `allocation_*`）已下线，`ALLOC_*` 仅保留兼容，不再由当前代码使用。

**与前端代码的映射**
- 前端 `a/前端测试/server.js` 默认：
  - `EXCHANGE_NAME=DispToAlgoExchange`
  - `QUEUE_NAME=DispToAlgoQueue`
  - `ROUTING_KEY=AssignmentTaskRequest`
  - 交换机类型：`fanout`
- 本项目外部接收器 `external_receiver` 读取：
  - `EXT_EXCHANGE` ⇔ `EXCHANGE_NAME`
  - `EXT_EXCHANGE_TYPE`（与前端定义一致）
  - `EXT_QUEUE` ⇔ `QUEUE_NAME`
  - `EXT_ROUTING_KEY` ⇔ `ROUTING_KEY`
  - `EXT_BINDING_KEY`（绑定键，默认 `#` 通配 fanout 消息）
  - `AMQP_*` 或 `RABBITMQ_URL` 指向与前端相同的 RabbitMQ 实例。

**注意事项**
- 交换机类型必须一致：若前端声明 `fanout`，接收端也需声明为 `fanout`；否则会出现 RabbitMQ 的 “inequivalent arg 'type'” 错误。
- `agvStatusList.endurance` 单位：接收端按“小时”直接写入 `Amr.endurance`（不做换算）；若前端提供分钟需提前换算。
- 路由键确认：本接收器会在日志中打印收到的 `routing_key`，便于核对是否与前端一致（请求链路应为 `AssignmentTaskRequest`，fanout 下绑定键默认设置为 `#`）。

**运行示例**
- 前端（另一台机器或本机）：
  - 设置 `RABBITMQ_URL` 指向同一 RabbitMQ；或使用默认 `amqp://localhost`。
  - 运行 `node a/前端测试/server.js`，在网页中提交任务。
- 本项目接收端：
  - 在同一网络下，确保能访问对方的 RabbitMQ 主机（`AMQP_HOST`）。
  - 在终端执行：
    - `source config/network.env`
    - `TIMEOUT_SEC=30 MAP_FILE=/path/to/map.json ./build/bin/external_receiver`

**故障排查**
- 如果收不到消息：
  - RabbitMQ 管理界面检查 Exchange/Queue/Bindings 是否与参数一致。
- 确认前端 `EXCHANGE_NAME/ROUTING_KEY` 与接收端 `EXT_EXCHANGE`/发布端 `EXT_ROUTING_KEY` 一致（接收端绑定默认 `EXT_BINDING_KEY=#`）。
  - 查看接收端日志的 `routing_key` 是否为预期的 `AssignmentTaskRequest`。
