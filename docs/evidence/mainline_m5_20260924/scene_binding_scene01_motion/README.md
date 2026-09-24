# scene_binding_scene01：短暂真实臂运动，视频与相机证据有缺口

原场次：`/home/yjh/WorkSpace/astribot_validation/M1_scene_binding_20260924/scene_binding_scene01`。本目录仅离线读取已经结束的场次，不启动 ROS/Gazebo/GPU，不重复控制根因诊断。

## 可确认的结果

租约原始记录显示六个控制器各有 child submission、response、terminal：左右臂、左右夹爪、头、躯干；response 与 terminal UUID 一一对应。六个 terminal 都是 `success=false/result_code=5`。父结果为 `EXECUTION_GUARD_UNHEALTHY`，`resources_released=true`。

真实左臂 JTC 反馈确认运动发生，未只依赖下发记录：

| ROS 时间 | 已记录事实 |
|---|---|
| 38.26 s | 左臂仍保持初态 |
| 38.28 s | reference 开始变化 |
| 38.297 s | 父反馈首次 EXECUTING_FIRST_MTC_STAGE |
| 38.30 s | actual 开始明显变化 |
| 38.639 s | 父反馈首次 EXECUTION_GUARD_UNHEALTHY |
| 38.64 s | reference 切换到固定值，actual 达到最大初态位移附近 |
| 38.66 s 起 | actual 向固定 reference 收敛 |
| 39.45 s | JTC 记录结束，仍有约 1.19e-5 rad/s 的微小速度 |

347 条 JTC 记录覆盖 32.18–39.45 s；源 header stamp 与探针记录 ros_s 在此数据中相等。左臂第 1 关节最大变化 **0.088494 rad（约 5.07°）**，全左臂最大观测速度 **0.544723 rad/s**，最大跟踪误差 **0.051330 rad**。末样本最大跟踪误差约 **1.071e-6 rad**。这些数值证明短暂运动及停止后的收敛，不证明已到原 PREGRASP 目标或严格零速。

“明显变化”仅用 `1e-6 rad` 排除本记录的数值抖动，不是新的控制或安全阈值。执行→guard 父反馈窗口为 **342 ms 仿真时间 / 355.220 ms steady 时间**。

原始 journal 的 `issued_at` 是租约字段，不能当事件时刻。例如 child 记录均为 38.113 s，但之前的重查记录已明确保存 38.202/38.252 s。报告仅按 journal 顺序确认子任务链，运动时间采用 JTC 与父反馈；不从租约字段制造执行起点。

`envelopes=[]`、`acks=[]`，没有 Hold 确认。六个 child response/terminal **不是六个包络 ACK**。因此完整 PREGRASP→保持→六 ACK 未通过，完整 PICK/PLACE 未验收。

## 录像与 JTC 的对齐边界

原视频：`/home/yjh/WorkSpace/astribot_validation/M1_scene_binding_20260924/scene_binding_scene01/first_stage.mp4`；对应 `first_stage.json`。127 帧，10 fps，播放 12.7 s；记录器墙钟窗口 21.357669 s。固定帧率播放秒数不等于仿真或现实执行时间。

实际解码的第 117→118 帧能看到左臂/夹爪的小幅姿态变化，方向与 JTC 的变化相容。但以下状态更新时累计编码帧数都为 **118**：

- WAITING_FOR_EXECUTION_GUARD：Unix 1790223260.835004；
- EXECUTING_FIRST_MTC_STAGE：Unix 1790223260.8842328；
- EXECUTION_GUARD_UNHEALTHY：Unix 1790223261.2395139。

按记录器 `status_cb` 保存计数、`overview_cb` 写帧的语义，等待→guard 回调之间 **404.510 ms**，执行→guard 回调之间 **355.281 ms** 没有新增编码帧；第 118 帧已经显示故障状态。它证明记录器观察空档，**不能转换成 raw RGB-D gap、相机源龄或上游停发原因**。后续写入帧仍可能是执行期间采集的排队图像，因此也不能断言运动时刻完全没有图像。

视频没有逐帧图像源 stamp、接收 steady time；head 小窗使用缓存图像且没有保存其 stamp。因而只确认前后姿态变化和状态顺序相容，**逐帧图像/JTC 精确对齐与连续运动画面覆盖未验证**。

- `before_after_excerpt.mp4`：原帧 110–126 的 17 帧前后对比片段，保持 10 fps、播放 1.7 s，仅重新编码，没有插帧。不是连续运动验收片段。
- `frame_117.png`、`frame_118.png`、`frame_126.png`：原视频直接解码图像。
- `left_joint_timing.png`、`left_jtc.csv`：真实 JTC 曲线与数据。虚线是父反馈观察时刻，不是 guard 原始事件时刻。

## 相机与 guard 缺证

本场交付 87 个文件，没有 40 话题 observer 的 manifest/events、逐帧 timing/health 或 bag；实际 runner 也未启动该 observer。M2 进一步确认没有场外副本。`capture_owner` 是进程归属记录工具，不是相机采集器。

因此，运动窗口内六相机 raw/ProjectionHealth、采集源龄、接收 gap、点云源龄均为 **NOT_MEASURED**。不能以没有日志告警推断持续健康。

本场动作前 head 快照的确有效：RGB/depth/info 同为 **28.900 s**，health 为 `OK/CAMERA_READY/valid=true`，历史 age **35 ms**、sync skew 0、报告频率 20 Hz。它比执行父反馈早 **9.397 s**，不能顶替运动期间证据。6.677900 s 采集中 RGB/info 各 112 条、depth 101 条，仅保存聚合数据和一张完整快照，不能还原缺失观察的具体 gap 或原因。

日志里头、腹相机各有 5 条 **动作前** STALE 告警，最大 steady age 分别 **296.921/299.397 ms**，告警 capture stamp 为 13.950、20.050、20.100、20.150、20.200 s。告警条数不是 raw gap 次数。最后一批累计点云处理统计也早于执行窗口约 2.05 s，不用于运动期间的源龄判断。

`/transport/execution_guard/status` 原始消息没有记录。现有 `EXECUTION_GUARD_UNHEALTHY`、`RESOURCE_CLOCK_RESET`、`HOLD_CANCELED` 是父级/资源状态的原因字符串，**不足以证明原始 guard 原因或真实时钟回退**。此缺口保留，不重复 M2 的控制诊断。

两份 native PlanningScene CDR 各 211820 bytes，作用是场景证据，不含相机逐帧健康，不能补上述缺口。收尾身份记录显示 12 个唯一进程均退出；MoveGroup 卸载 `-11` 单列，因此不是全节点干净退出通过。

## 文件与复核

`motion_video_report.json` 保存运动、事件与视频范围；`recorded_events.json` 保存规范化事件，`camera_evidence_gaps.json` 保存目录清单、动作前健康及原始 STALE 行号。`verification.json` 记录来源/产物哈希。

从项目根目录运行 `python3 docs/evidence/mainline_m5_20260924/scene_binding_scene01_motion/analyze_recorded_motion.py` 可离线重算图表、片段与 JSON。没有修改运行时、降低阈值或将未测项标为通过。下一场采集接线单独交给 M2 在其有归属的会话中启动。
