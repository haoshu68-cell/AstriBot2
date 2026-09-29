# M2 / M3 采集接线交接

2026-09-24。本文件和配置仅离线准备；M2 拥有启动、采集、停止权，实际场次等待总调度放行。没有在本轮启动 ROS、Gazebo、GPU 或模型。前场缺证已记录在 [scene_binding_scene01_motion/README.md](scene_binding_scene01_motion/README.md)，M2 确认该场没有启动 observer，也没有场外副本。

验收分三层：配置/脚本离线验证；本场真实进程/订阅/接收证明；覆盖实际动作或实际模型输入。前一层不能替代后一层。本文件不改变机器人产品准入规则。

## M2 运动场：40 个固定订阅，加 native 阶段共 41 个

入口为 `tools/vision/capture_m5_observer.py`，配置为本目录 `topics.json`。新增必需 `/transport/execution_guard/status`，类型 `astribot_transport_msgs/msg/ExecutionGuardStatus`，reliable/volatile/depth 10。现有 `kind=status` 已原样保存 typed guard，不需要新处理分支。guard 没有 header，顶层 `source_ns=null` 正常；原始原因和时钟在 `row.data.reason/stamp/joint_stamp/base_stamp`。分析输出应读 `status_observations[].data.reason`，外层 `reason` 是解析结果。

native 阶段接口为 `/transport/hold_executor/status`、`std_msgs/msg/String`，实际 publisher reliable/transient_local/depth 1、50 ms 周期发布。observer 的 reliable/volatile 订阅能收到后续消息，不请求启动前缓存。

导航约束迁移后，诊断订阅为 `/navigation_policy/constraint_state`、`std_msgs/msg/String`、reliable/volatile/depth 10，以已有 `kind=context` 原样保存；删除已撤销的 `/cmd_vel_policy_input`。新节点 `/navigation_constraint` 只发布前置约束及诊断，不是 `/cmd_vel` 发布者，也不提供 protection envelope ACK；本配置不推断消费者 ACK 集合。新增 `/navigation_policy/arm_speed_limit` 后，M2 私有 transfer 配置和精确话题计数预检须同步为当前 40 + 1；计数再次为 41 不代表清单与历史相同，须核新配置哈希。旧场次 manifest/分析不改，录像工具继续使用已验源码；迁移前哈希快照见 [navigation_constraint_update/before_snapshot.json](navigation_constraint_update/before_snapshot.json)。

约束节点的底盘坐标速度意图来自 `/cmd_vel_nav_body_raw`，实际余速来自 odom；最终 `/cmd_vel` 是当前 body-to-world 唯一路径输出，不能直接解释为底盘坐标意图或约束节点输出。arm_chassis_speed_coupling 已改为上游百分比源，禁止输出 Twist。M5 本轮保存已有最终速度及新增百分比元数据，不新增意图流采集。旧 `tools/vision/verify_navigation_projection_gate.py` 要求末级独占速度输出和非空零 Twist，属于旧故障注入验证；本次不迁移、不运行，旧结果不作为新架构验收。

新增 `/navigation_policy/arm_speed_limit` 类型为 `nav2_msgs/msg/SpeedLimit`，reliable/volatile/depth 10，`kind=context` 保留完整消息和 header 源时间；该专用话题的 `percentage=true`、`0%=HOLD` 契约不同于原生 Nav2 SpeedLimit 的零值“无限制”语义。本场启用耦合时须保存实际发布者/订阅者和样本，再与最终 `/navigation_policy/constraint`、constraint_state 的时间与版本核对；不能只按最近一帧或话题存在断言因果和消费成功。它沿用可选上下文采样规则，缺样本仍报 MISSING，不构成上游能力通过。

在 M2 已确认的实际 overlay 和 session 环境中，沿用 runner 的 `child()` 创建、跟踪该进程；不使用另一个 domain 或仅 source 基础 ROS 的临时环境。下列参数可直接传给现有 `child()`，`--output` 必须尚不存在：

```bash
: "${M2_SESSION_JSON:?当前会话 session.json 绝对路径}"
: "${M5_RUN_DIR:?已存在的本场证据目录绝对路径}"
python3 -B /home/yjh/WorkSpace/astribot_sdk_ros2/tools/vision/capture_m5_observer.py \
  --topic-config /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/topics.json \
  --session-json "$M2_SESSION_JSON" --output "$M5_RUN_DIR/m5_observer" \
  --seconds 300 --warmup-sec 2 \
  --phase-topic /transport/hold_executor/status --phase-type std_msgs/msg/String \
  --reference /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml
```

输出保存为本场 `m5_observer.log`。所有者保存 PID、start_ticks、boot_id、exe、命令、限定的 domain/partition 环境及源码/配置哈希，停止前重新核对身份。不得按进程名字批量终止。

**发 Action 前**，现有 runner 应保存并核对：

1. 同一 observer 的进程身份仍有效；至少已观察 20 秒；没有同名第二节点。manifest 的环境、session 与实际会话一致，清单恰为当前 40 条配置 + 明确 native 阶段；guard required/type/QoS 正确，`phase_source` 正确，无 `error`。
2. 实际 `/m5_passive_observer` 订阅含全部 41 条及正确类型；保存原始订阅拓扑。保存 guard、native 阶段、constraint_state、arm_speed_limit 和本场所用相机的发布者节点、类型、QoS，确认属于本场；constraint_state 发布者应为 `/navigation_constraint`，不能沿用旧末级节点名。manifest 在 ROS 初始化前已经写入，因此存在文件本身不代表订阅建立。
3. 原始事件已出现 `/clock`、guard、native 阶段。本场运动相机的 raw 和所用 health/projection 状态也须按本场观测目标单列存在性与当前 epoch；可选话题没有数据应明示，不能包装成全链路完成。文件有缓冲，不能用刚启动时的文件大小替代拓扑。
4. 将该检查快照保存到 Action 结果，即使其后 Action 失败也保留。保留真实 task/context/phase/clock epoch，不以 issued_at 租约字段充当执行事件时间。

如用命令留存拓扑，仅在本场已授权环境执行：

```bash
ros2 node info /m5_passive_observer > "$M5_RUN_DIR/m5_observer_topology.txt"
ros2 topic info /transport/execution_guard/status --verbose > "$M5_RUN_DIR/m5_guard_topology.txt"
ros2 topic info /transport/hold_executor/status --verbose > "$M5_RUN_DIR/m5_phase_topology.txt"
```

**停止窗口**：动作/取消/实测停止/资源释放收尾完成后至少继续观察 20 秒。若总调度窗口允许，自然到达 300 秒，退出 0 才满足本工具完整窗口条件。若本轮短场结束后由所有者发 SIGINT/SIGTERM，工具会 finally 排空写队列（最多 10 秒）并落最终 manifest，明确 `completed=false/interrupted=true`、退出 130；分析标 INCOMPLETE。可以单独核验实际动作前后覆盖，但不能改写成完整 300 秒 PASS。外部等待应给 ROS shutdown 和最多 10 秒排空留余量，observer 单独给 20 秒；超时/强杀必须保留为采集失败，不能吞掉或覆盖 Action 原始结果。保留实际 returncode 和终态 manifest 的 error/writer_error/writer_dropped/ended_*；进程已退出或外层 cleanup_complete 都不能替代这些检查。

```bash
python3 -B /home/yjh/WorkSpace/astribot_sdk_ros2/tools/vision/analyze_m5_capture.py \
  --capture "$M5_RUN_DIR/m5_observer" --output "$M5_RUN_DIR/m5_analysis.json"
```

## M3 静止场：9 个头部话题，原始数据限时 3 秒

**最新场次决定**：本次第一段 Hold 场只录 7 条基线 head raw，专用 cloud 和专用 ProjectionHealth 明确 `NOT_ACTIVATED`，不为该场新增 source/projector。下面 9 条完整方案供以后专用链实际激活的 M3 场使用；本场从命令中删除 `/manipulation/single_box/head_rgbd/points` 与 `/perception/projection_health/single_box/head_rgbd`，其余 7 条 QoS 不变（YAML 多出的两项不创建订阅）。raw 3 秒录制及 flush 必须在发动作前完成。未录专用链不阻止本场 ARM 任务，但绝不能称完整 M3 输入证据。

当前采集处于 READY PREGRASP 静止窗口，不发模型。原始数据用于寻找该静止窗口内的同刻候选快照；没有实际 source/model 消费记录时，不将它称为模型输入。以后模型场必须以 M3 实际消费的 `capture.stamp_ns` 选样本。

复用 rosbag2 录原始 CDR，禁止 `-a`、正则全相机或无限续录。`sim_pose_capture.py` 不保存专用 cloud/ProjectionHealth，不能单独满足该证据契约。只录以下 9 条，QoS 文件为本目录 `head_raw_qos.yaml`：

| 话题 | 类型 | 请求 QoS |
|---|---|---|
| `/camera/raw/head_rgbd/image` | sensor_msgs/msg/Image | best_effort / volatile / depth 10 |
| `/camera/raw/head_rgbd/depth_image` | sensor_msgs/msg/Image | 同上 |
| `/camera/raw/head_rgbd/camera_info` | sensor_msgs/msg/CameraInfo | 同上 |
| `/manipulation/single_box/head_rgbd/points` | sensor_msgs/msg/PointCloud2 | 同上 |
| `/perception/camera_health/head_rgbd` | astribot_perception_msgs/msg/CameraHealth | reliable / transient_local / depth 10 |
| `/perception/projection_health/single_box/head_rgbd` | astribot_perception_msgs/msg/ProjectionHealth | 同上 |
| `/tf` | tf2_msgs/msg/TFMessage | best_effort / volatile / depth 100 |
| `/tf_static` | tf2_msgs/msg/TFMessage | reliable / transient_local / depth 100 |
| `/clock` | rosgraph_msgs/msg/Clock | best_effort / volatile / depth 100 |

专用 cloud 必须是 M3 的 decimation=1 全像素 FLOAT32 XYZ、point_step=16 来源；不能换用导航 pointcloud。M2/M3 先确认专用 projector/所需租约已由其所有者激活，保存实际参数、publisher 身份和各话题 endpoint QoS；本采集命令不启动 projector/source/模型或运动。publisher 不存在或 QoS 不兼容就记录缺证，不启动空采集假装成功。

以下由 M2 runner 跟踪 timeout 及其 rosbag 子进程身份、开始/结束 steady/wall 时间和日志。这里的 `timeout` 是 **3 秒总录制进程窗口**，包含发现时间；超时先发 SIGINT，另给 10 秒收尾，最多约 13 秒。Humble 的 `--max-bag-duration` 只切分文件，不能用作总时限。

```bash
: "${M3_RUN_DIR:?已存在的本场证据目录绝对路径}"
if timeout --signal=INT --kill-after=10s 3s \
  ros2 bag record --storage sqlite3 --max-cache-size 104857600 \
  --qos-profile-overrides-path /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/head_raw_qos.yaml \
  --output "$M3_RUN_DIR/head_raw_bag" \
  /camera/raw/head_rgbd/image \
  /camera/raw/head_rgbd/depth_image \
  /camera/raw/head_rgbd/camera_info \
  /manipulation/single_box/head_rgbd/points \
  /perception/camera_health/head_rgbd \
  /perception/projection_health/single_box/head_rgbd \
  /tf /tf_static /clock \
  > "$M3_RUN_DIR/head_raw_bag.log" 2>&1; then
  M3_BAG_EXIT=0
else
  M3_BAG_EXIT=$?
fi
printf '%s\n' "$M3_BAG_EXIT" > "$M3_RUN_DIR/head_raw_bag.exit"
ros2 bag info "$M3_RUN_DIR/head_raw_bag" > "$M3_RUN_DIR/head_raw_bag_info.txt"
```

典型正常限时退出码为 124；它只说明 timeout 到期，不代表 bag 验收通过。必须另查日志无写入错误、最终 metadata/数据库可读、九话题实际计数、停止/flush 完成；137/强杀、空 topic、无完整同刻 tuple 均为缺证。不能把 runner 的通用 `run()` 当成功零退出封装来隐藏 124。输出目录必须新建且未存在。3 秒窗口包含 ROS/DDS 发现，不能保证收到足够消息；录制启动后须在进程仍存活时保存本次 recorder 的九订阅及兼容端点，不能只查 publisher 存在。检查耗尽窗口或本场缺 tuple 就明确失败，由调度决定后续时段，不自动延长或循环重录。

640×360、20 Hz、RGB8 + float32 depth + 16 字节点云的纯 payload 约 106 MB/s，3 秒约 318 MB，加数据库/其他消息开销；以实际录制值为准。额外订阅/磁盘写入会增加负载，因此该短 bag 不用于宣称无观测干扰的长期性能结果。不传 `--use-sim-time`：保留 bag 记录时间和源 header 的区别，并单独记录 `/clock`；bag 记录时间不是 steady 接收时间。

未来 M3 静止 source/模型场的外部 steady 元数据使用现有 observer 和 `head_metadata_topics.json`，20 秒自然结束。该清单现为 **14 条**：上列 9 条头部输入，加以下 5 条控制状态。图像/点云仍仅记元数据，其余消息由现有采集器完整保留字段；不新增采集器分支。

| 新增控制话题 | 类型 | 现有 kind / QoS |
|---|---|---|
| `/navigation/arm_hold` | astribot_navigation_msgs/msg/ArmHoldStatus | status / reliable、volatile、depth 10 |
| `/navigation/envelope_applied` | astribot_navigation_msgs/msg/EnvelopeApplyStatus | status / reliable、volatile、depth 10 |
| `/odom` | nav_msgs/msg/Odometry | odom / sensor：best_effort、volatile、depth 5 |
| `/cmd_vel` | geometry_msgs/msg/Twist | twist / sensor：best_effort、volatile、depth 5 |
| `/navigation/execution_status` | astribot_navigation_msgs/msg/NavigationExecutionStatus | status / latched：reliable、transient_local、depth 4 |

这 5 条均为本次静止模型场所需记录，`required=true`，不擅自设置周期 gap 预算。ArmHold/ACK 有 header，字段原样保存；NavigationExecutionStatus 没有 header，其原时间在 `data.stamp`，顶层 `source_ns=null` 正常。Twist 无源 stamp，不能伪造；接收 steady/wall 和最近 `/clock` 由外部 observer 记录，与 source 内部接收时间区分。ACK 的 session/consumer/envelope_epoch/installed_geometry_hash/applied/reason 全部保留，不能加不存在的 mode 字段。

NavigationExecutionStatus 是事件流，不能要求每个周期刷新；latched depth 4 只保留有限历史，不能宣称拿到全部既往事件；没有任务时也可能根本没有事件。若只在前 2 秒接收历史事件，通用分析器热身后可能标 MISSING，必须同时检查 `samples_full_window` 与原事件，不能把该标签等同全场无数据，也不能仅凭缓存存在宣称当前准入成立。准入、有效期及参考身份以 M3 工装/源契约核验为准；元数据不是安全门控的替代品。

**执行顺序尚待 M2/M3 实现，当前不能直接倒序启动**：两个模型 server 参数依赖 probe 先写的 `registration.json`，所以“server ready 再启动现有 probe”还不是可执行流程。总调度正协调“登记→server ready→bag 发现→单次新 capture”的验证门控；本轮不实现登记阶段拆分，也不声称模型已经预热。最终 runner 应在真正模型加载/冷启动完成后，再让 3 秒 head raw 窗口覆盖那一次实际输入 capture T；metadata 应先完成 14 条订阅发现。raw 不必覆盖整个推理输出等待，发现或实际新 capture 耗尽窗口则明确缺证，不自动延长或循环重录。**3 秒 raw 采集预算与原输入 5 秒有效期是不同边界**；不延长原 source TTL、不重盖 stamp、不把旧 capture 重新变为有效。entry/final 快照与全窗口元数据互相补充，不能据此宣称整个窗口一定没有丢失或失效。

它与当前 41-topic observer 使用相同节点名，**不得两者并行启动**；由 M2 选择本场配置。20 秒只是元数据，不延长原始 bag；若实际模型操作超出 20 秒，明确该窗口覆盖不足，不推成完整模型窗口证明。命令保持：

```bash
python3 -B /home/yjh/WorkSpace/astribot_sdk_ros2/tools/vision/capture_m5_observer.py \
  --topic-config /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/head_metadata_topics.json \
  --session-json "$M2_SESSION_JSON" --output "$M3_RUN_DIR/head_metadata" \
  --seconds 20 --warmup-sec 2
```

若本场已运行当前 41-topic observer，可复用其现有 raw/TF/clock、odom/cmd_vel 元数据；它没有 M3 专用 cloud/ProjectionHealth 及上述 3 个 typed 控制状态订阅，不能冒充完整 14-topic 模型场元数据。独立 observer 回调时间也不等于 bag recorder 或 source 内部消费时间。九头部 raw 3 秒命令与 QoS 不变；新增控制状态只进入 20 秒元数据清单。此前 `head_capture_config_verification.json` 对应旧九话题配置哈希，保留作历史验证，不冒充本次新版验证。

**同刻离线核验**：保留 bag 与配置/session/参数/身份/日志/退出码的 SHA256，关联实际 capture T 和 task/context/epoch。RGB、depth、CameraInfo、专用 cloud 需按精确 header stamp/frame 配对，不取最近帧、不重盖时间戳；检查尺寸、布局、无效深度与 NaN 原样保留。health 的 header 是发布/心跳时刻，不能强制冒充采集 T；原样核对 `capture_stamp`、source_epoch、processing_epoch、epoch_first_capture_stamp、valid_until 与 M3 消费契约。TF 必须覆盖 T，若插值应明确记录。只为选中的一个 T 导出原始输入和必要 TF/health/clock 上下文用于复核，不把离线文件重新冒充在线有效输入。raw 文件/图像、ROS 源 stamp、外部接收 steady、source 内部接收 steady 是不同证据层。

## 每帧录屏源 stamp 方案

现有 `tools/sim/record_transport_demo.py` 仅固定 10 fps 写收到的 overview 图像，stage_frames 保存状态回调时的累计帧数；缓存 head 没有保留原 stamp。前场 EXECUTING→guard 故障的 355 ms 内没有新增编码帧，视频只支持前后姿态变化，不能称连续完整动作录像。

最小方案是在当前录像脚本旁增加 JSONL sidecar，每次实际 `writer.write` 对应一行：零基 frame_index、overview 原 header stamp/frame_id、overview 回调入口 monotonic_ns/wall_ns、写入前后 monotonic_ns、缓存 head 原 header stamp/frame_id 及其接收 monotonic_ns/wall_ns、当前 status 序号和状态回调时间、原始 task/context/phase。没有 head 或源 stamp 不可用时明确 null/原因，不拿当前时间补成源 stamp。如需 ROS 接收时钟，显式订阅 `/clock` 并记录最近值及其接收时刻；不能拿节点默认系统时钟标成仿真 ROS 时间。

状态切换另留原始事件，关联 status 序号；逐帧记录的是被叠加的最近状态，不能宣称与图像采集同步。关闭时保存 sidecar 行数与实际可解码视频帧数并核验相等，再按 overview 源 stamp、steady 接收间隔分别统计缺口/重复/回退，按 head stamp 标出缓存画面龄期。固定10fps播放时间不等于真实运动耗时；不插帧、不用状态事件替代图像。总调度已批准此验证脚本最小改动，本轮离线实施和验证材料见 [video_frame_timing](video_frame_timing/)，没有新现场录像。
