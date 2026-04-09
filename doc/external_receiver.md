# external_receiver 输出与落盘说明

`external_receiver` 会：
- 消费业务侧调度请求（`AssignmentTaskRequest`）。
- 发布分配结果（`AssignmentTaskResponse`）。
- 缓存每台车的计划，并响应 `RobotPathRequest` / `RobotTrailRequest`。

## 1) 日志滚动（推荐用脚本启动）

推荐用 `script/start.sh` 启动，它会把 `external_receiver` 的 stdout/stderr 按时间滚动写入 `DEBUG_DIR`：

```bash
source config/network.env
bash script/start.sh
```

关键变量：
- `DEBUG_DIR`：日志与落盘目录（默认 `debug/`；仿真默认覆盖为 `simulation_v2/debug/`）
- `RECEIVER_LOG_ROTATE_SEC`：日志滚动周期（默认 300 秒）
- `RECEIVER_LOG_KEEP`：最多保留的 log 文件数（默认 30）

## 2) 分配结果落盘（allocation_result_*.json）

当 `RESULT_DUMP_DIR` 非空时，`AssignmentTaskResponse` 会落盘为 `allocation_result_YYYYMMDD_HHMMSS.json`：
- `RESULT_DUMP_DIR`：落盘目录（默认跟随 `DEBUG_DIR`）
- `RESULT_DUMP_KEEP_MAX`：最多保留的结果文件数量（仅裁剪 `allocation_result_*.json`，默认 20；<=0 表示不裁剪）

> 若你只需要在线可视化，建议直接使用 `simulation_v2/run_all.sh`；它会把 `DEBUG_DIR/RESULT_DUMP_DIR` 默认指向 `simulation_v2/debug/`，避免污染根目录。

## 3) traffic path 落盘与日志

traffic JSON dump 位置与分配结果一致（`RESULT_DUMP_DIR`），通过数量控制是否开启：

- `TRAFFIC_DUMP_KEEP_MAX`：最多保留的 JSON 文件数量（<=0 表示不落盘）

traffic log 直接写到 stdout（即 `external_receiver.log`），按需开启：
- `RECEIVER_LOG_TRAFFIC_SUMMARY`：1=打印 traffic summary
- `RECEIVER_LOG_TRAFFIC_JSON`：1=在 summary 后追加完整 JSON
- `RECEIVER_LOG_TRAFFIC_DETAIL`：1=打印 traffic 节点明细（nodeId/x/y）

文件命名：
- `traffic_path_*.json`（JSON dump）
