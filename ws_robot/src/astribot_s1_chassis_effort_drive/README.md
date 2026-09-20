

## 可选静止轮位保持

`idle_position_hold` 默认 false，保留原零轮速控制。仿真需要模拟驻车阻力时可显式开启；`idle_position_kp` 默认 3.0 Nm/rad，允许 0–10，力矩仍受 wheel_effort_limit_nm 限制。节点仅在四轮速度均小于 0.05 rad/s、位置完整且反馈新鲜时捕获参考；收到非零目标或反馈失效后释放，下一次停止重新捕获。该设置不改变任何真机 SDK 模式，不保证抵消所有轮地滑移或机械碰撞。验收与边界见 docs/CANCEL_AND_STATIONARY_HOLD_20260919.md。
