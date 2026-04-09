# 导演版回放

演示入口已经迁移到仓库根目录下的 `demo/`：

```bash
bash demo/prepare_real_inputs.sh
bash demo/record_real_session.sh
bash demo/serve_replay.sh
```

底层工具仍保留在 `simulation_v2/`：

- `sim_runner_v2.py`：在线仿真与录制
- `replay_tool.py`：回放 bundle 构建与服务
- `replay_web/`：回放前端

当前回放页按演示要求做成了“顺序播放”模式：

- 支持播放 / 暂停 / 回到开头 / 倍速快进
- 不支持时间轴拖动、章节跳转、手动步进
