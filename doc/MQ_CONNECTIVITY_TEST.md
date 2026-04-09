# MQ 联调连通性快速自测

使用步骤假设你已 `source config/network.env`，并编译好二进制。

## 1. 检查 MQ 连接与队列存在性
```bash
cd /path/to/project
# 默认检测 EXT_QUEUE（DispToAlgoQueue），仅验证连接/队列
# 若想避免影响线上消费，添加 --mirror 使用临时队列
python3 script/mq_network_check.py --queue "$EXT_QUEUE" --count 0 --timeout 5
```
- 看到 “[MQ] 队列 'xxx' 可用” 即连接正常。
- 如需检查结果队列（分配结果队列），用 `--queue "$ASSIGN_RESULT_QUEUE"`。

## 2. 逐个信号验证发送 + MQ 收到
在新终端运行 receiver（或保持线上服务），此终端用脚本监听指定 routing key 对应的队列。

### 2.1 监听业务队列（默认 EXT_QUEUE）
```bash
# 推荐使用镜像临时队列监听，避免影响业务 receiver
python3 script/mq_network_check.py --mirror --count 1 --show-body \
  --skip-routing SendRobotStatusInfos
```
保持运行，另开终端按顺序发送：

1) 配置：
```bash
./build/bin/external_config_sender
```
- 预期：监听端显示 routing_key=SendRobotConfigInfos，summary 为 “机器人配置批次，数量=...”

2) 地图：
```bash
./build/bin/external_map_sender
```
- 预期：routing_key=SendMapInfo，summary 含 mapId/name/nodes/edges 与 `config/south_20260107.json` 一致。

3) 车辆状态（默认 `script/status_simple_warehouse.json`，可用 STATUS_PAYLOAD_FILE 覆盖）：
```bash
./build/bin/external_status_sender
```
- 预期：routing_key=SendRobotStatusInfos，summary 数量=7（或你的覆盖文件数量）。状态消息顶层现已直接使用数组 `[ {...} ]`，但 receiver 同时兼容旧的 `{ "robotStatusInfos": [...] }` 结构。

4) 任务请求（默认 `script/generated_test_payload1.json`；如需自定义可替换该文件内容）：
```bash
./build/bin/external_task_sender
```
- 预期：routing_key=AssignmentTaskRequest，summary 显示 task 数。

### 2.2 监听结果队列（ASSIGN_RESULT_QUEUE）
在 receiver 跑完分配后，监听结果：
```bash
python3 script/mq_network_check.py --queue "$ASSIGN_RESULT_QUEUE" --count 1 --show-body
```
- 预期：routing_key=AssignmentTaskResponse，json 包含 `agv_assignments` 与（若开启）`path_planning_results`。

## 3. 常见问题
- 连接失败/队列不存在：确认 `AMQP_*`/`RABBITMQ_URL` 与联调 MQ 一致，队列是否已声明。
- 收不到消息：确认 sender/receiver 使用同一 exchange/queue/routing_key；`simulation_v2/run_all.sh` 默认启用 per-run 隔离（独立 exchange/queue），如需使用共享队列请设置 `SIM_USE_SHARED_MQ=1`。
- 状态/地图不匹配导致不可达：确保 `STATUS_PAYLOAD_FILE` 节点 ID 与发送的地图一致。默认状态文件已与 `config/south_20260107.json` 对齐。

## 4. 仿真姿态流（Sim Pose Stream）
若需要观察算法内部预测的 AGV 轨迹，可开启“姿态流”：

1. 在 `config/network.env` 中设置
   ```bash
   export ENABLE_SIM_POSE_STREAM=1
   export SIM_POSE_INTERVAL_MS=200            # 发送周期，默认 200ms
   export SIM_POSE_EXCHANGE=AlgoSimExchange   # MQ exchange
   export SIM_POSE_QUEUE=AlgoSimQueue         # MQ queue
   export SIM_POSE_ROUTING_KEY=SimPose        # routing key
   ```
2. 启动 `external_receiver` 后，额外线程会周期性地
   - 从 `RobotDataRepository` 中取出已缓存计划（若存在）；
   - 基于地图节点位置生成简化姿态：`(x, y)` 为计划首节点坐标，`yaw` 指向下一节点（若存在），`velocity` 当前固定为 0；
   - 对没有计划的 AGV，回退到最新状态上报的坐标/航向。
3. MQ payload（JSON）示例：
   ```json
   {
     "deviceId": "AGV05",
     "timestamp": 1764750127668,
     "x": 25000.0,
     "y": 5000.0,
     "yaw": 1.57,
     "velocity": 1200.0,
     "nodeId": "458",
     "source": "plan"
   }
   ```
   `source=status` 表示当前 AGV 尚无计划，直接使用状态上报。

该流适合仿真/可视化联调，生产环境可保持 `ENABLE_SIM_POSE_STREAM=0` 关闭。
