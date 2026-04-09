# Demo

真实数据驱动演示链路固定为：

1. `demo/source_data/` 中的真实历史任务导出
2. `demo/prepare_real_inputs.sh` 生成仿真输入
3. `demo/record_real_session.sh` 跑仿真并录制 session
4. `demo/serve_replay.sh` 回放录制结果

说明：

- `simulation_v2/` 只保留仿真与回放引擎。
- `demo/inputs/` 和 `demo/sessions/` 都是产物目录，不纳入版本管理。
- 回放页面只允许顺序播放和倍速快进，不允许拖动跳播。
