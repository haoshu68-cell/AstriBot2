# 18:14 恢复与首场准备失败记录

继续用户已批准的单物体正常抓取、搬运、放置主线，不扩大场景矩阵。原 14:10、15:10、15:28、16:28 时间记录保留；协作窗口模型连接中断期间未完成验收。本记录不重置同一问题的历史计时。

## 已核实的现场

- 原始目录：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene01_world92`。
- 导航栈达到 ready；准备脚本的第一次停止窗口在 15 秒内未成立，报 `IDLE_HOLD_AND_MEASURED_STOP_REQUIRED`。
- `fixtures/result.json` 中 `operations=[]`、`created_models=[]`、`motion_commands_sent=0`。完整父 Action 尚未开始，不能声称抓取或搬运失败，也不能声称完成。
- `result.json` 为失败且 `cleanup_complete=true`；`stack/session.json` 为 stopped，`remaining_owned_pids=[]`。这是本会话进程回收证据，不是物理停稳证明。
- 旧准备结果在 guard 尚未获得时错误记录 `stop_window_valid=true`；该字段不作为验收证据。

## 当前直接排查

私有准备脚本复用 `verify_kinematic_inventory.setup_sample`，要求 `/cmd_vel` 在 300 ms 内持续新鲜。当前平滑器冷启动无输入不发布，body-to-world 的 watchdog 在从未接收命令时不发布、过期时仅发一次零。这使旧验证要求与新架构的空闲输出契约不一致。但首场缺失原始采样，尚不能断言它是唯一首因。

M2 保持唯一仿真及私有脚本写者，先补实际 odom、命令、执行器状态与接收时效，使用新的 case 目录在原条件下确认缺项。停止阈值、轨迹 2.5 倍时间缩放与速度/加速度 0.1 均不调整，不添加零命令发布者，不恢复底盘末级保护。根任务只读核对契约并统一交付；M1 只在存在执行器直接缺陷时介入；M5 等待正式主场景视频。

下一放行仍要求同一父任务的 PICK、载荷导航、PLACE、权威 EMPTY、独立 PlanningScene 读回、实际停稳、子动作终态、资源释放和可播放录屏。编译与孤立协议证据不能替代该结果。
