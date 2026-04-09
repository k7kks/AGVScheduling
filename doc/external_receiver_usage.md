# external_receiver 使用指南

本文说明如何启动 `external_receiver`、发送业务消息，以及如何请求路径/轨迹。

> 建议先阅读 `doc/ENV_VARS.md`，里面按模块汇总了所有环境变量。

## 1) 推荐：一键在线仿真（含可视化）

```bash
bash simulation_v2/run_all.sh
```

该脚本会：
- 启动 `external_receiver`（带日志滚动、结果落盘目录）。
- 生成地图/状态/任务并通过 MQ 发送。
- 在线 Web 可视化（`simulation_v2`）。

## 2) 仅启动 external_receiver（带日志滚动）

```bash
source config/network.env
bash script/start.sh
```

`script/start.sh` 默认：
- `SKIP_STATIC_TABLE=1`（跳过静态表，纯 A* + 动态禁用节点）
- 日志按时间滚动（见 `RECEIVER_LOG_ROTATE_SEC` / `RECEIVER_LOG_KEEP`）
- 输出与落盘目录默认在 `DEBUG_DIR`（未设置时为 `debug/`；仿真默认覆盖为 `simulation_v2/debug/`）

## 3) 手动发送业务消息（分步联调）

按顺序运行（每条消息都走 `EXT_EXCHANGE/EXT_QUEUE`）：

```bash
./build/bin/external_config_sender
./build/bin/external_map_sender
./build/bin/external_status_sender
./build/bin/external_task_sender
```

## 4) 请求路径 / 轨迹（Path / Trail）

路径请求（RobotPathRequest）：

```bash
EXT_ROUTING_KEY=RobotPathRequest ./build/bin/external_path_request_sender
```

多地图场景下可额外传入 `PATH_REQUEST_MAP_ID=<mapId>`，显式指定请求应命中的地图上下文；若不传，算法端会按 `deviceId` 最近一次状态里的 `mapId` 自动路由。

轨迹请求（RobotTrailRequest，可选按子任务截断）：

```bash
EXT_ROUTING_KEY=RobotTrailRequest ./build/bin/external_trail_request_sender
```

多地图场景下可额外传入 `TRAIL_REQUEST_MAP_ID=<mapId>`，显式指定轨迹请求应命中的地图上下文；若不传，算法端会按 `deviceId` 最近一次状态里的 `mapId` 自动路由。

`RobotTrailRequest` 请求体字段：
- `messageId`、`deviceId`
- 可选 `mapId`
- 可选 `subTaskId`：当传入时，`external_receiver` 会尝试在该 AGV 的缓存计划中定位对应子任务节点，并把返回的 trail 截断到该节点（含该节点）；若找不到该子任务或未传入，则按 `TRAIL_MAX_POINTS` 控制返回节点数量。
