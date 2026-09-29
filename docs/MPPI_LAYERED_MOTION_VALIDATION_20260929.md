# MPPI 分层碰撞检查：现有数据运动验证（2026-09-29）

## 结论

**机器人在新 critic 启用的情况下实际前行了约 0.802 m；真实记录回放证明碰撞候选被拒绝，但整段导航未通过。**

目标前方 0.8 m。到达附近后，Arrival 终点微调报 `PATH_TRACKING/REFINEMENT_NO_PROGRESS`，NavigateToPose 以 ABORTED（6）结束。因此返回和 90° 转向没有执行，不能记为完整往返成功，也不能把本次结果写成双臂避障整体验收通过。

本次使用当前仓库世界、已有 Gazebo 分层地图插件、实际固定姿态包络和 SLAM 数据，没有添加环境障碍或接入新相机地图链。地图来源是 `gazebo_collision_geometry`，不代表相机感知能力。

## 成功标准与结果

| 项目 | 结果 | 证据边界 |
|---|---|---|
| 新 critic 被实际控制器加载 | 通过 | `/proc` 加载路径与控制器参数读回 |
| MPPI 启用分层检查时产生实际行走 | 通过 | FOLLOW 运行、157 条 MPPI 可视化消息、SLAM 位移 |
| 固定双臂前提 | 观测支持 | 14 个臂关节反馈范围最大约 4.57e-13 rad；只代表此次仿真反馈 |
| 实际轨迹在记录地图中无碰撞 | 回放通过 | 357 个 SLAM 样本及相邻扫掠，碰撞数 0 |
| 指向现有障碍的候选被新 critic 拒绝 | 隔离回放通过 | 空闲候选代价 0，碰撞候选代价 infinity、权重 0 |
| 实际障碍边界触发机器人改道/停车 | 未证明 | 本次行走没有产生可归因于新 critic 的在线碰撞拒绝记录 |
| 0.8 m 前行、返回、90° 转向完整成功 | 不通过 | 前行终点微调失败，后续未执行 |
| 中止后停稳及资源释放 | 通过本次观测 | SLAM 位姿窗口、零速度命令和 HoldResources 终态 |
| 接触/力、动态障碍、相机输入、真机 | 未验证 | 没有外推结论 |

## 运行身份与输入

- 实现分支：`codex/fixed-posture-whole-body`；运行 critic 来自该分支隔离安装，功能代码未改动、未合并。
- 自有会话：`mppi_motion_20260929`；domain 93；partition `astribot_mppi_motion_20260929`；独占仿真租约。
- 控制器 PID 1200319 加载 `/home/yjh/WorkSpace/astribot_whole_body_nav/runs/whole_body_nav/install/astribot_s1_path_tracking/lib/libastribot_s1_mppi_critics.so`。
- 参数读回 `FollowPath.inner.critics` 第一项为 `WholeBodyCollisionCritic`。底盘 `idle_position_hold=true`、`idle_position_kp=3.0` 已读回。
- 现有仓库世界 SHA256：`5aa81c3ace012bebfd30e8a21fd6a6e202f63bcc1e373ddde165d10e4981808a`。
- 通过既有 Gazebo 插件生成五层地图，没有修改世界几何；地图版本 `b05aeb31f80201ba658354c89f68c4276cb2da66cd22972ed71668c5b7264f32`。
- 导航阶段高度几何 hash 唯一：`f7d0834beab3943e7f4cfe19acc39f9120541027b04c3255cae443e83247f49a`。
- 实际空载观察、MoveIt 空附着状态、真实 HoldResources 和五消费者准入均通过后，才下发导航目标。

## 实际运动结果

导航模拟时间 655.759～691.476 s，活动阶段记录 357 个 `/slam/pose` 样本，全部在 map 坐标系。

- 最大相对起点净位移：0.801648 m。
- 终态位置距目标：0.001599 m。
- 终态朝向误差：2.145379°。
- 控制器原因：`PATH_TRACKING/REFINEMENT_NO_PROGRESS`；随后 Controller patience exceeded、NavigateToPose ABORTED。
- 中止后观测 7 个 SLAM 样本，墙钟跨度 0.629 s；窗口最大锚点净位移约 1.31e-7 m，最短角展开后的净角偏差约 5.04e-8 rad，最后速度命令为零。

这些极小的停稳/关节数值是本次仿真输出，不代表真机精度。位置与停稳计算只用 `/slam/pose`，未用 odom/TF 代替监测。无 SLAM 样本的情况不计停稳。

## 真实数据回放的评分与轨迹检查

从已经关闭的 rosbag 提取原始 CDR：地图、有效固定包络、SLAM 位姿与同期 `/clock`。保留消息内容，不刷新包络时间戳；在隔离 domain 94 回放原时钟，加载同一真实 pluginlib critic。

两条诊断候选由本次数据生成：一条保持实际空闲位姿；另一条从该位姿指向记录地图中最近的占据格 `(-0.675, 1.275)`。这条候选只用于检查评分，**没有下发给机器人，也不是从机器人实际 MPPI 批次中抽取的候选**。

结果：`stationary_cost=0`、`obstacle_rollout_cost=inf`、`failed_batch=0`，即保留可行候选并将碰撞候选权重置零。此处障碍格属于低层，因此这项实际数据测试不能单独证明“底盘可过、仅双臂碰撞”的专门场景。

另用同一分层几何核检查实际记录的 357 个 SLAM 样本及相邻扫掠，碰撞数为 0。活动阶段地图版本和固定几何 hash 均唯一，支持该固定快照回放；它不替代物理接触测量，也不是独立于被测几何核的碰撞 oracle。

## 排障与保留的失败

1. 首次脚本要求空闲时持续有 cmd_vel，当前控制链为空闲事件式命令，未发送目标即停在前置检查。保留 `navigation/result.json`。
2. 此次专用验证脚本采用已有 `measured_stop` 的事件式命令选项：没有历史命令时以实际 SLAM 窗口确认空闲；一旦有命令仍检查最后命令为零。没有补发零命令来伪造连续观测。保留 `validation_script.diff`。
3. 旧部署 task_arbiter 在接收包络后崩溃。它原构建使用的 NavigationEnvelopeV2 头缺少高度字段，与加载的消息库不一致。未取得崩溃栈，不能把根因写成已完全定位。以当前源代码和本分支消息/EnvelopeEvidence 在隔离目录重编译，ownership_test 1/1 通过；替换自有会话中已退出的进程后，准入及导航正常开始。
4. 替换进程由独立启动句柄持有，记录了启动身份，并在专用验证器中列为本任务拥有的额外根；没有修改共享源码或共享安装。保留原失败、构建日志、参数、身份和验证器副本。
5. 最后仍发生上述终点微调失败，本次没有修改到位阈值或控制逻辑来将失败记为成功。

## 录像、原始证据与收尾

证据目录：`/home/yjh/WorkSpace/astribot_whole_body_nav/runs/mppi_motion_20260929/`。

- [50 秒实际行走录屏](../runs/mppi_motion_20260929/mppi_actual_motion.mp4)：原速，1280×800、15 fps，截取实际 RViz 屏幕；已注明终点失败。
- [完整原始录屏](../runs/mppi_motion_20260929/navigation_retry_screen.mp4)。RViz 的 `/optimal_trajectory` 显示未收到独立话题，不能作为所选轨迹证据；真实 MPPI 可视化记录在 bag 的 `/trajectories`。
- [导航结果](../runs/mppi_motion_20260929/navigation_aligned/result.json)、[运动与双臂分析](../runs/mppi_motion_20260929/motion_analysis.json)、[评分与实际路径回放结果](../runs/mppi_motion_20260929/replay_motion_result.log)。
- [rosbag 元数据](../runs/mppi_motion_20260929/motion_retry_bag/metadata.yaml)：含 1688 条 SLAM 位姿、365 条分层地图、13754 条包络、26368 条消费者确认、157 条 MPPI MarkerArray、1971 条 cmd_vel。数量是整包统计，不能当成活动阶段样本数。
- [会话日志](../runs/mppi_motion_20260929/stack/session.log)、[会话清单](../runs/mppi_motion_20260929/stack/session.json)、[环境](../runs/mppi_motion_20260929/env.bash)、[地图来源](../runs/mppi_motion_20260929/height_map_source.json)。
- 同目录的验证脚本、`replay_critic.cpp`、`extract_replay.py`、`analyze_motion.py` 和构建日志保留复现入口。复现需要重新核实 domain、资源及包解析；录制期间未读取正在写入的 SQLite。

导航已终止、SLAM 停稳观测通过、保持资源已释放；自有补充进程和仿真均已退出，supervisor 最终 `remaining_owned_pids=[]`，独占性能租约已释放。

下一步验收缺口：解决终点微调失败，并在现有障碍附近获得新 critic 的在线候选拒绝/实际改道证据，再验证仅高层碰撞场景。当前不合并开发主线。
