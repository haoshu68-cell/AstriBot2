# M5 六相机运动/覆盖验证准备（2026-09-24）

本轮从 09:53 +08 开始，仅做离线准备。修改范围为独立 `tools/vision` 验证脚本和本目录；没有启动 ROS 节点、仿真、GPU 推理或真机，没有修改共享运行时、启动参数或 install。M2 当前拥有仿真时段；完整运动采集等待总调度交接和 M1 实际动作。

本轮成功标准：当前六相机接口有源码依据；被动元数据采集器与离线分析器可离线自测；缺样本、冻结时钟、旧帧心跳及采集失败不会冒充通过；运动/ROI/故障缺口、输入和时段明确。**离线准备完成不等于 M5 运动验收通过。**

## 最新证据与旧问题的关系

- 旧的 484–582 ms 是 9 月 22 日下午非独占窗口的原始深度接收间隔。该窗口曾有 2–3 套仿真，RTF 约 0.542；并发是已证实的干扰因素，唯一根因未证实，详见 [旧分析](../../DEPTH_RECEIVE_GAP_ANALYSIS_20260922.md)。
- 后续 `runs/task_chain_20260922_gpu_recovery/wrist_01/capture/` 已有独占 1800 秒静止测试：RTF 0.994273，时钟最大接收间隔 119.226 ms，四路原始深度没有超过 250 ms 的接收间隔。CUDA 已改成受监督的持久子进程；不能重复把旧线程方案当当前实现，见 [恢复报告](../../RGBD_CUDA_PROCESS_RECOVERY_20260922.md)。两轮实现与负载并不完全相同，不据此量化“单独去掉并发”的收益。
- 对应 `projection_02/` 是 1640 秒处理健康与恢复；`navigation_gate_02/` 是 360 秒静止保护。它们不能代替持续运动、TF 故障、相机覆盖或停车距离。
- [9 月 23 日相机基线](../joint_acceptance_20260923/camera/README.md) 确认六相机加载和模型一致性；[正常抓放录像](../../NORMAL_GRASP_VIDEO_20260923.md) 确认兼容流程的实际抓取/携带/放置。二者没有完整六相机逐帧运动时序，不能拼接成 M5 通过。

## 当前接口

唯一相机预设：`ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml`。本轮源文件哈希见 `source_hashes.json`；其相机预设和安装参考与 9 月 23 日基线一致。运行时还须由会话所有者核实实际 resolved launch、overlay、二进制与参数，源码哈希不能替代实际加载证据。

| 相机 | 当前 raw 接口后缀 | 仿真采样配置 | optical frame |
|---|---|---|---|
| head_rgbd | image / depth_image / camera_info | 640×360，20 Hz | astribot_s1/astribot_head_link_2/head_rgbd_sensor |
| torso_rgbd | image / depth_image / camera_info | 640×360，20 Hz | astribot_s1/astribot_torso_link_4/torso_rgbd_sensor |
| left_wrist_rgbd | image / depth_image / camera_info | 640×320，20 Hz | astribot_s1/astribot_arm_left_link_7/left_wrist_rgbd_sensor |
| right_wrist_rgbd | image / depth_image / camera_info | 640×320，20 Hz | astribot_s1/astribot_arm_right_link_7/right_wrist_rgbd_sensor |
| head_stereo_left | image_raw / camera_info | 400×300，5 Hz | astribot_s1/astribot_head_link_2/head_stereo_left_sensor |
| head_stereo_right | image_raw / camera_info | 400×300，5 Hz | astribot_s1/astribot_head_link_2/head_stereo_right_sensor |

全部 raw 前缀为 `/camera/raw/<camera>/`，总计 16 个流。双目目前不提供 depth 或 ProjectionHealth，不能要求不存在的接口。

`topics.json` 当前共列 40 个订阅，显式添加 native `/transport/hold_executor/status` 后 41 个。导航约束改为前置决策后，移除已撤销的 `/cmd_vel_policy_input`，诊断改为 `/navigation_policy/constraint_state`，新增上游 `/navigation_policy/arm_speed_limit` 百分比源；当前与历史场次虽同为 41 话题，清单不同，历史证据保持原样。精确启动、Action 前订阅验收、停止语义、M3 有界 head raw 录制及视频逐帧时钟方案见 [CAPTURE_HANDOFF.md](CAPTURE_HANDOFF.md)：

- 16 个 raw 流、`/clock` 为基础采样必需项。四路 RGB-D 的接收间隔预算为 250 ms；双目只统计间隔和存在性，没有擅自规定时效阈值。
- 四路 `/perception/camera_health/<camera>` 与 `/perception/projection_health/<camera>`；订阅 QoS 为 reliable + transient_local。头腹健康默认 expected_rate_hz 仍为 10，不能用健康有效替代 20 Hz 采样验证。
- 头腹 `/camera/<camera>/points_raw`、`/camera/<camera>/points`，双腕 `/manipulation/camera/<camera>/points`。双腕处理链须有消费者有效租约；raw 可用不等于双腕处理已激活。
- `/tf`、`/tf_static`、`/joint_states`、`/odom`、`/cmd_vel`、`/navigation_policy/arm_speed_limit`、`/navigation_policy/constraint`、`/navigation_policy/constraint_state`。
- `/navigation_policy/arm_speed_limit` 为 `nav2_msgs/msg/SpeedLimit`，既有 `kind=context` 原样保留 header、percentage、speed_limit；该专用话题约定 `percentage=true`、`0%=HOLD`，不采用原生 Nav2 SpeedLimit 的零值“无限制”语义。上游 arm_chassis_speed_coupling 只发布百分比，不输出 Twist；由 NavigationConstraint 消费后形成最终 MotionConstraint。采样存在本身不能证明该约束已被消费。
- 新增必需 `/transport/execution_guard/status`，typed `ExecutionGuardStatus` 原样保留 reason/stamp/joint_stamp/base_stamp；它没有 header，顶层 source_ns=null 不表示丢失消息内时钟。
- 阶段源必须显式传入。当前 native `/transport/hold_executor/status` 为 String JSON；兼容 `/transport/status` 只用于对应旧流程。task/request/lease/resource_epoch/hold_id/phase 依实际原始状态或所有者事件文件关联，不能从速度和时间顺序猜阶段。

可选项只是允许采样器在未激活链路中运行；它们缺失仍会显示 MISSING。**采样器的 PASS 仅表示必需话题存在与显式接收间隔预算满足，不能作为运动、健康、控制或覆盖通过。** 完整动作窗口须另查所用相机处理健康、有效租约、TF 和控制证据。

## 本轮新增工具与指标口径

`tools/vision/capture_m5_observer.py` 为被动订阅验证脚本；无控制发布者、服务调用或 Action 调用。`--check-config` 不导入/初始化 ROS。真正采集前核对 session.json 中 domain/partition，拒绝覆盖已有输出，记录输入文件哈希、会话副本、环境和明确阶段源。

每条记录保留：topic/kind/camera、源采集 stamp、frame、回调入口 monotonic/wall 时间和最近收到的 `/clock`。图像/点云只落元数据，其他消息保留字段；后台写入队列有界，溢出、写入失败、超时退出或中断均标无效并显式报错。采集时仍有 DDS 反序列化开销，后续性能判断须核对观察器开启的增量影响。

`tools/vision/analyze_m5_capture.py` 输出各流现实接收 P50/P95/P99/max、源 stamp 间隔、重复/回退、初始及末尾静默、超预算次数、ROS 龄期及未来样本，分位数采用 `(n−1)×p` 线性插值。默认发现预热 2 秒，完整样本数仍保留。`ros_now_ns=0` 表示尚无时钟，龄期不可计算。

健康消息使用 capture_stamp 和 source_epoch/processing_epoch 识别同一采集；重复心跳不会刷新该帧的现实年龄。应用 pending_depth 为采样值，不能推成 DDS 队列占用。处理 published_stamp 的 ROS 龄期不能冒充现实端到端延迟。跨机器 monotonic 不可直接相减。

当前 `/navigation_constraint` 的 `constraint_state` 为 String JSON，含 proposal/constraint epoch/sequence、deadline、`constraint_publish_start_ns`、projection_ready/reason、loop_wall_dt_s 等字段，以既有 `kind=context` 原样保留。它不发布 Twist 或 protection envelope ACK，诊断不能作为速度输出/制动证明；`/cmd_vel` 由当前 body-to-world 唯一路径直接输出，arm coupling 已移到上游百分比源。诊断默认不是每控制周期发布，本轮不改变频率。无 stamp/ID 的 Twist 不能通过“最近一条”配对制造因果控制延迟。旧场次 `protection_state` 和其 `output_publish_start_ns` 仍按旧契约解释；旧 `verify_navigation_projection_gate.py` 含末级速度所有权及零输出判据，不适用于本次新架构采证，未纳入本场正常采集入口。

工具明确标 `NOT_MEASURED`：控制因果时延、实际运动验收、ROI/三维覆盖、图与所有权、DDS 队列。TF 消息保存不等于消费者在采集时刻查 TF 成功。点云 width×height 不是有效深度点数。

## 分阶段验证矩阵

| 顺序/场景 | 所需输入与判断 | 本轮状态 |
|---|---|---|
| S0 M2 静止有载接线 | 16 raw + clock、实际订阅存在、正确 overlay/类型；仅观察 30 秒即可查接线 | NOT_RUN；等所有者交接，不算运动覆盖 |
| S1 M1 正常抓放/携带 | 300 秒被动窗口，动作前后各至少 20 秒；M1 阶段与请求 ID、实测关节/odom、所用链路健康/TF | NOT_RUN；M1 实际动作是前置 |
| S2 腕部目标 ROI 与携带遮挡 | 同一采集 stamp 的 RGB/Depth/Info、实测 TF、独立目标几何/位置或标注；按抓取/携带阶段统计可见比例和有效深度 | WAITING_INPUT；缺独立评估几何与数值阈值，不能假定覆盖百分比 |
| S3 低障碍/高障碍 | 由场景所有者放置已知几何，覆盖正前/侧向/臂载荷遮挡；输入 raw 深度、过滤前后点云、实际 costmap 和碰撞包络 | NOT_RUN；需冻结障碍尺寸/位置和判据 |
| S4 稀疏云与 VoxelLayer | 先确定实际启用层；对范围/高度边界内外和观测遮挡逐项比较原始点、过滤点、标记/清除格 | NOT_RUN；默认 local 为 ObstacleLayer，VoxelLayer 是显式候选 |
| S5 源重启/TF 缺失 | 正常链通过后，由资源所有者做有界注入，记录源 epoch、处理 epoch、捕获 stamp、invalid/recovery 和旧帧拒绝 | NOT_RUN；本工具不发信号/重启、不注入 TF |
| S6 完整运动负载 | 同一冻结配置、独占主机窗口，包含动作/导航/推理；长窗口指标与故障恢复分开 | NOT_RUN；本轮 300 秒不能替代此前要求的完整 30 分钟运动压力 |

当前 local 高度/范围源码参考：头部高度 0.05–2.0 m、标记距离 0.2–4.0 m；腹部高度 0.05–1.8 m、距离 0.2–3.0 m。Voxel 参数 origin_z=0、z_resolution=0.15、z_voxels=16，网格高度 2.4 m 不等于相机有效覆盖。实际采样前须核对运行时参数及插件选择。global 默认 scan-only，不把 local 相机结果推成 global 三维覆盖。

复用 `tools/vision/voxel_contract_check/check.cpp` 的离线插件边界检查；它不是渲染障碍覆盖。`sim_pose_capture.py` 的紫色目标 mask 只适用于对应夹具，不是任意物体标注。`audit_camera_reference_mounts.py` 的静态姿态及顶层 profile 结果不能代替 canonical 预设运动覆盖。需要图像 ROI 时单独使用现有快照工具，在已授权窗口按明确目标采样；新元数据工具不重复保存全部图像。

独立物体位置只用于评估，禁止回流给模型作为推理输入。低/高障碍、ROI 阈值未冻结前，不输出完整覆盖 PASS。

## 交接所需信息和执行入口

总调度须提供目标会话 session.json、已核对查询环境及完整 overlay、当前单一仿真所有者、M1 实际动作窗口、明确阶段 topic/type 或事件文件、动作资源和对象 ID、所需消费者激活信息。阶段接口不必等待统一 topic 冻结；按实际所有者文件关联即可。

首轮完整窗口为 **300 秒**，至少覆盖动作前 20 秒、完整正常动作、动作后 20 秒；动作超过 260 秒则事先选择更长窗口，脚本上限 600 秒。先保留正常流程，再逐项增加复杂故障。M2 静止数据只检查接线，不被计入运动覆盖。

以下是交接后的调用模板；本轮未执行采集命令。先由所有者提供并检查查询环境，不能仅 source 基础安装而漏掉自定义消息 overlay。可选订阅也需要其消息类型能导入。

```bash
python3 tools/vision/capture_m5_observer.py \
  --topic-config docs/evidence/mainline_m5_20260924/topics.json \
  --session-json "$M5_SESSION_JSON" --output "$M5_CAPTURE_DIR" \
  --seconds 300 --phase-topic /transport/hold_executor/status \
  --phase-events "$M5_OWNER_EVENTS" \
  --reference ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml

python3 tools/vision/analyze_m5_capture.py \
  --capture "$M5_CAPTURE_DIR" --output "$M5_ANALYSIS_JSON"
```

`--phase-events` 仅在确有所有者文件时传入；文件保持原始 schema/ID。输出目录与分析 JSON 必须是新路径。所有者还须保存实际发布者/QoS/参数、Gazebo 与构建负载清单；本工具不证明主机独占或调用者拥有动作权限。提前 SIGINT 的退出码为 130，完整窗口标 INCOMPLETE，不能改记 300 秒 PASS。

离线验证入口：

```bash
python3 -m unittest discover -s tools/vision -p 'test_*m5*.py' -v
python3 tools/vision/capture_m5_observer.py \
  --topic-config docs/evidence/mainline_m5_20260924/topics.json \
  --check-config --phase-topic /transport/hold_executor/status
```

实际测试日志与文件哈希在本目录。后续恢复入口即上述 S0/S1 交接；本轮没有用旧数据填充新采集 manifest。
