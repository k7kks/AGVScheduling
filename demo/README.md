# Demo

真实数据驱动演示链路固定为：

1. `demo/source_data/` 中的真实历史任务导出
2. `demo/prepare_real_inputs.sh` 生成仿真输入
3. `demo/record_real_session.sh` 跑仿真并录制 session
4. `demo/serve_replay.sh` 回放录制结果

推荐直接使用：

```bash
bash demo/record_real_session.sh
bash demo/serve_replay.sh a4_presentation_demo
```

导演版固定场景演示：

```bash
bash demo/record_director_session.sh
bash demo/serve_replay.sh director_demo
```

说明：

- `simulation_v2/` 只保留仿真与回放引擎。
- `demo/inputs/` 和 `demo/sessions/` 都是产物目录，不纳入版本管理。
- 回放页面只允许顺序播放和倍速快进，不允许拖动跳播。
- `demo/record_real_session.sh` 现在默认使用演示参数录制，输出固定覆盖到 `demo/sessions/a4_presentation_demo/`。
- `demo/record_director_session.sh` 会生成固定场景的导演版 demo，输出到 `demo/sessions/director_demo/`。
- 默认参数是 `30 AGV` 的真实数据演示子集（默认 `80 tasks` 上限）。
- 默认还会把演示时长封顶到 `180s` 模拟时间，避免个别死锁循环拖长录制。
