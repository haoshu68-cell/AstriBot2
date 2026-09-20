# 仿真验收证据

- `task18_events.jsonl` / `task18_state.json`：抓放及故障后已放置恢复；包含原故障记录。
- `task19_events.jsonl` / `task19_state.json`：旧进程式 Gazebo 同步超时，保载停止。
- `simulation_camera_audit.json`：仿真传感器采样；不是实机相机审计。
- `warehouse_transfer.json` / `camera_rgbd_transport.yaml`：最终场景和虚拟相机参数。

完整日志在 `/tmp/codex_transport_20260919`；`scene20/launch.log` 和 `skills/launch20.log`
为此轮实际启动日志。`demo/transport_demo.mp4` 为实际图像录像。

- `task20_events.jsonl` / `task20_state.json`：最终完整连续成功的实跑记录。
