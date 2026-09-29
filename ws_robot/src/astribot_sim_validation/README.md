# 上位机仿真验收

只用于 ROS domain 213、localhost、Gazebo partition `astribot_operator_validation_213`。先完成仿真功能与异常场景验收，再讨论真机；本包没有硬件启动入口。

## 组件与责任

- `simulation_validation.launch.xml`：复用仓库的 Gazebo/感知/基线地图，关闭导航自动启动。
- `simulation_probe`：C++ 就绪检查，15 秒内实测 clock、joint_states、odom、scan_from_cloud 的样本和时间推进，并查询 8 个 ros2_control 控制器是否 active。时间重复刷新不会伪装为推进。不是完整导航/抓放验收。
- `navigation_validation.launch.xml`：第二阶段启动现有 Nav2、上位机网关和地图管理，使用 `fixed_v2 + p5`（自动通道识别）。独立地图目录 `/tmp/astribot_validation_213/map_catalog`，不会复用默认 catalog。P4 需要人工通道文件，不能直接省略文件使用。
- `transport_session`：C++ 会话管理服务 `/simulation_transport/start`、`cancel`，状态 `/simulation_transport/status`。仅启动自己持有的 transport_task 子进程组，不自行发送底盘或机械臂指令。
- `transport_fault_acceptance`：C++ 阶段触发异常验收，按本次 run 的账本触发附着后取消或带载移动时停止续租，同时验证停稳、载荷和恢复门禁。
- `transport_acceptance`：C++ 上位机验收客户端；申请控制权、续租、提交搬运、等待结果、释放控制权，支持 `cancel_after_sec` 注入取消。

搬运执行复用已有 `astribot_s1_transport`，保留其资源锁、MoveIt/MTC、控制器、包络和载荷事务。该既有执行器目前仍为 Python；本轮没有新建 Python 采集或控制脚本。前期纯 C++ ArmExecutionTransaction 尚未替换执行器内部事务，不能据此宣称执行层已全部迁移到 C++。

`transport_session` 的只读 `navigation_geometry_mode` 参数默认为 `fixed_v2`，写入本次 scenario.json，并显式传给执行器的 `--navigation-geometry-mode`。只写 JSON 不够：既有执行器的命令行默认值会覆盖 JSON。该参数同时进入关键参数录制清单。默认 launch 两端使用同一种模式；诊断时自行选择 legacy 必须同步调整导航端，不能混用。

世界/感知入口也显式启用 fixed_v2，使用标准感知入口的 2.2 m overhead 过滤上限与附着自过滤。C++ 会话在派发前检查唯一、新鲜、完整且附着状态已确认的几何状态；参数正确不等于真实视场无盲区。这里的导航策略 `p5` 与项目交付阶段 P5 不是同一概念，不代表交付阶段已完成。

## 启动

必须先构建并加载与源码一致的依赖。不要混用缺少 RobotGeometryState、相机配置或 robot_geometry 的旧 install。建议独立 build/install，保留构建日志和加载的前缀。

所有终端须加载同一 install 及 `tools/setup_mtc_humble.sh`，并设置：

```bash
export ROS_DOMAIN_ID=213 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=astribot_operator_validation_213
export GZ_PARTITION=astribot_operator_validation_213
export IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
```

依次启动；长驻 launch 各自使用终端，先确认 domain 213 没有别的验收实例：

```bash
ros2 launch astribot_sim_validation simulation_validation.launch.xml map_yaml_path:=/absolute/maps/warehouse_baseline.yaml
ros2 run astribot_sim_validation simulation_probe --ros-args -p report_path:=/tmp/readiness.json
# readiness.ready == true 后继续
ros2 launch astribot_sim_validation navigation_validation.launch.xml
ros2 launch astribot_s1_transport transport_skills.launch.py
ros2 launch astribot_s1_transport transport_support.launch.py
ros2 launch astribot_sim_validation transport_session.launch.xml output_root:=/tmp/new_validation_session
```

确认 Nav2 生命周期、MoveIt 与相机就绪后，通过 RViz 工作站的“开始仿真搬运任务”按钮操作，或运行：

```bash
ros2 run astribot_sim_validation transport_acceptance --ros-args -p report_path:=/tmp/transport_result.json
# 独立的新场景中注入取消：
ros2 run astribot_sim_validation transport_acceptance --ros-args -p cancel_after_sec:=2.0 -p report_path:=/tmp/cancel_result.json
```

取消用例预期结果需结合账本判定：客户端只对完整搬运 SUCCEEDED 返回 0；取消返回非零不是“取消处理失败”。必须检查账本 CANCELED/FAULT、stop_error、载荷事实。客户端 `max_wait_sec` 默认 900 秒（允许 0–3600 秒之间的有限正值），超时释放租约会触发会话管理请求取消，报告 UNKNOWN，不能解释为已停稳。

## 恢复边界

- 会话启动前要求控制权、空闲导航、推进的仿真时钟、唯一 clock/odom/joint_states 来源、没有外部 transport_task。
- 搬运派发前还要求 `/lifecycle_manager_navigation/is_active` 的新鲜成功响应；无服务、未全部active、响应超时或超过2秒未确认时返回 `SIM.NAVIGATION_NOT_READY`，不创建run或子进程。状态消息增加 `navigation_ready`。检查使用非阻塞请求，最多一个待处理请求，1秒超时清理并使旧响应失效；不把Gazebo/控制器probe通过当作Nav2已激活。
- 通过网关启动的搬运 pending/运行/状态未知期间，网关拒绝新导航/探索/换图；执行器内部仍走原有导航仲裁。
- 租约/控制会话变化、网关失联或时钟停滞会向**自己创建的**进程组请求取消。
- `runtime.json` 在派发前 fsync 落盘；每次保存 scenario.json 和独立 ledger。只有子进程成功退出且账本 SUCCEEDED + PLACED + 无 attachment 才显示成功。
- 失败、异常退出及节点重启进入 RECOVERY_REQUIRED；不自动重试、不删除台账、不自动松爪。检查并重建仿真世界后才可用新的 output_root 开始独立实验。换目录不是同一现场任务的恢复手段。
- 普通 launch 失败也可能留下部分进程，重试前必须核查本次进程组、单一发布者和目录锁。禁止全局 pkill 或清理其他 ROS domain。

## 场景与证据

默认场景来自已有 `warehouse_transfer.json`。`config/warehouse_transfer_grid_aligned.json` 只调整离台/送达目标 Y 到 0.025 m，用于对照栅格边界导致的预测朝向问题；没有缩小 footprint 或降低碰撞门槛。它不是原场景算法问题已修复的证明。

`config/warehouse_transfer_clearance.json` 将取放工位侧向增加 6 cm；相机 `transport_support.launch.py scenario:=...` 必须使用同一文件。实跑结果为 PREGRASP GOAL_STATE_INVALID。两个对照配置均保留为失败复现实验，不作为已验收的默认运行方案。

夹持为 Gazebo **运动学附着**，不是接触摩擦/夹持力/滑落的物理证明。日志、录包与参数快照使用既有 spdlog/diagnostics_recorder。本轮验证报告见 `docs/SIMULATION_OPERATOR_CHAIN_20260919.md`。

## 附着后取消与移动中失联验收

每个用例使用新建且就绪的隔离仿真世界和新的会话目录。取消后载荷仍在机器人上，不能通过换 output_root 继续同一现场。先启动 diagnostics_recorder，使用仓库 recorder.yaml 记录参数、实际约束、整机几何和载荷附着状态，再运行一个用例：

```bash
# 抓取附着并开始收臂到行走姿态时取消。
ros2 run astribot_sim_validation transport_fault_acceptance --ros-args \
  -p use_sim_time:=true -p fault_mode:=cancel \
  -p fault_stage:=TRANSPORT_POSTURE -p minimum_speed_mps:=0.0 \
  -p minimum_joint_speed_radps:=0.05 \
  -p report_path:=/tmp/attached_cancel.json

# 另建世界：已附着并实际移动 >= 0.02 m/s 后停止续租。
ros2 run astribot_sim_validation transport_fault_acceptance --ros-args \
  -p use_sim_time:=true -p fault_mode:=lease_loss \
  -p fault_stage:=TRANSPORT -p minimum_speed_mps:=0.02 \
  -p report_path:=/tmp/moving_lease_loss.json

# 另建世界：带载实际行走时主动取消。
ros2 run astribot_sim_validation transport_fault_acceptance --ros-args \
  -p use_sim_time:=true -p fault_mode:=cancel \
  -p fault_stage:=TRANSPORT -p minimum_speed_mps:=0.02 \
  -p report_path:=/tmp/moving_cancel.json
```

此客户端只在**预期异常处理全部通过**时返回 0，不同于普通完整搬运客户端。它不发布速度，也不直接控制机械臂；通过上位机网关派发仿真任务和取消。报告包含注入时账本、实测速度、关键参数、最终账本、停稳证据及错误原因。

判定要求：本次账本 CANCELED/USER_CANCEL、stop_error 空，载荷 ID/版本/附着链接不变；会话 motion_blocked=true、子进程已退出；底盘线速度 <= 0.005 m/s、角速度 <= 0.01 rad/s、所有关节速度 <= 0.03 rad/s；源时间推进的停稳窗口至少 0.5 s，各条件同时保持 1 s。消息来源唯一、源时间不超前且年龄 <= 0.3 s；最终 cmd_vel 六轴均为零。完整几何仍包含载荷，Gazebo 附着位置误差加外接球半径乘旋转误差 <= 6 mm。失联用例还必须确认原控制会话失效。冻结时钟、重复旧包和仅收到取消 ACK 都不构成通过。

`max_wait_sec` 默认 300，终态后的验证限时 15 s。报告中的 `rejected_samples` 是未同时满足条件的观测次数，不代表机器人故障次数。自定义物体须配置 `object_id` 并同步修改录包话题；默认录包清单的物体为 transport_box_01。这些测试只证明运动学附着场景，不证明夹持力或滑落安全。


异常验收可选 minimum_joint_speed_radps（默认0，最大1）：大于0时，只在新鲜完整关节状态的左臂关节最大速度达到该值后注入；报告保留 measured_arm_speed_radps，避免阶段刚进入、尚未执行就取消而误认已覆盖运动中断。设置0保留原阶段触发行为。取消 capability 和仿真静止保持证据见 docs/CANCEL_AND_STATIONARY_HOLD_20260919.md。

## 静止保持对照实验

按用户 2026-09-25 要求，标准仿真及主线的 `idle_position_hold` 已固定开启，启动时须读回 true，`idle_position_kp` 保持 3.0。下方仅保留历史对照实验记录，不能作为当前主线启动配方自动执行：

```bash
ros2 service call /omni_effort_drive_node/set_parameters rcl_interfaces/srv/SetParameters \
  '{parameters: [{name: idle_position_hold, value: {type: 1, bool_value: true}}, {name: idle_position_kp, value: {type: 3, double_value: 10.0}}]}'
# 必须确认两个结果均 successful=true。
# 替代上面的 skills 启动命令，不要同时启动第二个执行服务：
ros2 launch astribot_s1_transport transport_skills.launch.py \
  mtc_velocity_scaling:=0.03 mtc_acceleration_scaling:=0.03
```

增益单位 Nm/rad，允许范围 0–10；输出仍服从轮端力矩限幅。该选项不绕过碰撞、关节跟踪或底盘漂移保护。采集使用功能包中的 diagnostics_recorder，记录参数和轮端力矩；明确停止录包成功后再打开 SQLite。不要把该仿真参数用于真机 SDK，也不要据一次成功声称可靠性验收完成。

## 可选物理接触诊断

在本包隔离环境与世界就绪后启动 `ros2 launch astribot_sim_validation contact_evidence.launch.py`。
它加载 Gazebo C++ `ContactEvidence`，只为 `astribot_s1` 的左臂/夹爪碰撞体请求接触数据，
不写位姿、速度、力或控制器命令；不在正常机器人启动入口自动加载。
`/simulation/left_arm_contacts` 为 `ros_gz_interfaces/msg/Contacts`，进入既有 recorder 白名单。
每10 ms仿真时间汇总接触对及位置；桥接后的消息时间为汇总窗口末尾，不能当成每个接触的精确起始时间。
在Ignition原始消息header中核对observed_collision_count大于0，空接触消息本身不证明成功监测了碰撞体。
世界重建后需重新启动；关闭桥接launch不会卸载已加入当前世界的诊断插件。
该组件适用于定位规划模型/物理模型差异，接触记录不等于夹持力或安全认证。

## 真实建图验收入口

`mapping_validation.launch.xml map_name:=<唯一名称>` 替代基线世界入口，启动真实
Voxel-SLAM、概率栅格、探索协调器和存图会话；探索默认暂停，导航仍单独启动。
不要与 `simulation_validation.launch.xml` 同时启动，以免重复世界、地图和 TF。
构建依赖必须包含 `astribot_s1_mapping`，基线地图模式通过并不能证明该建图依赖已安装。

确认物理、时钟、TF、地图推进和 Nav2 全部 active 后，使用探索的 resume/pause/cancel
接口验收。pause 不结束 SLAM；cancel 等待自己的导航终态和新鲜停稳证据后存图。
只有 `/mapping_session/status` 为 SAVED、匹配的 manifest 和地图/关键帧校验通过才算保存成功。
默认目录 `/tmp/astribot_validation_213/slam_sessions/<map_name>`，可显式覆盖 save_path。
空地图、不可达前沿和未知区域不能按“建图完成”处理；暂停环境下的存图成功也不等于自主探索覆盖验收。
该入口开启 `require_fixed_envelope`；目前必须先由合法的上层任务所有者完成
姿态保持和包络握手，才允许探索选点。r25暴露的通用探索保持会话尚未接通，
此入口当前可验收暂停/取消存图，不能据此宣称已支持自主遍历。不得发布伪造保持确认绕过此门槛。
