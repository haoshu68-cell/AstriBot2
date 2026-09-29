# 剩余 Python 运行时与 C++ 迁移边界：2026-09-21 盘查

**最新更新：**已完成导航仲裁、臂底盘耦合以及 map→odom / 跨域地图中继 C++ 迁移，对应生产 Python 入口删除；当前剩余 **79 个**自有运行时 Python 文件，另有 **1 个**已有 C++ 的旧标定入口待验证退役。累计 **23 个**历史模块仅用于验证，不随生产包安装。全项目去 pybind 仍未完成。策略 observer 字符串修复已记录在 `f4feccdf`，14/14 CTest、54组配对ROS场景（108项）和1600性能帧通过；JSON整数/字符串门槛已补齐，Stamp域和失败时机、完整controller及闭环验收仍待完成。32个策略Python文件仍计入79个运行时文件。

**这是源码与现有安装目录的只读盘查快照，不是迁移完成或运行验收结论。** 用户要求最终去 pybind、验证通过后不留兼容入口。本文不把关闭构建选项、删除 console_scripts 或 C++ 数学核心存在写成达成该目标。

盘查生成时间：`2026-09-21T13:44:07.695284+00:00`（UTC，上海时间加 8 小时）。工作区多人并行修改；精确版本以 JSON 中每个文件的 SHA256 为准，不能只以 Git HEAD 复现。

证据：[python_runtime_inventory_latest.json](evidence/pybind_removal_20260921/python_runtime_inventory_latest.json)；复现工具：[inventory_python_runtime.py](../tools/migration/inventory_python_runtime.py)。盘查脚本只读；迁移与验证另有 Git 和实验记录。盘查不启动/停止 ROS/Gazebo，不连接 SDK，不写共享安装目录。[本次快照与前次差异](evidence/pybind_removal_20260921/runtime_audit_stage6b_strings/README.md)。

盘查对象是**当前共享工作树**，会包含其他任务尚未提交的文件（例如 camera_calibration_postprocess）。主任务使用 `codex/pybind-migration-20260921` 记录本任务修改，并未收录其他任务全部改动；该记录分支的全树统计因此可以与本文盘查不同。本文不是该分支的全树可复现清单。

## 1. 数量定义与范围

| 分类 | 文件数 | 定义 |
|---|---:|---|
| 当前自有 Python 运行时 | 79 | 从当前源码 console entry 或明确 tools 生产根可达；含纯核心、ROS 适配、持续监督及日志适配，每路径计一次 |
| 已有 C++，仍声明 Python 运行入口 | 1 | transport camera_calibration_postprocess，主 launch 已用 C++，旧 console entry 仍在 |
| 已 C++ 的验证参考 | 23 | 17 个集中历史模块、3 个仲裁/耦合模块和3个定位/地图模块；无生产调用闭包，仅验证目录，不安装 |
| 可选例程运行支持 | 2 | tools/joy_tools.py 与 transport/vla_examples.py；不计生产默认路径 |
| 启动/配置/构建 | 43 | launch/setup、地图出生点启动校验、源资产生成等 |
| 包内测试/诊断/采集分析 | 187 | 包括 ROS 诊断记录器、joint_map_probe、vla_probe/replay；即使启动了 ROS 也不当生产迁移数 |
| tools 验证/采集/分析/准备 | 83 | 逐文件 AST 保留在 JSON；不因文件使用 rclpy 或控制实验装置就计入生产运行时 |
| 纯包标记 | 12 | 只有空体或 docstring 的 __init__.py |
| SDK 层 | 20 | 厂家分发与本地适配混合，单独核对，不混入自有 ROS 运行时数 |
| 厂家 SDK examples | 27 | 演示/测试调用者，不当作 27 个生产节点 |

本快照共解析 477 个 Python 源文件，无 AST 语法错误。**79 是文件数，不是 79 个节点；另 1 个旧入口应退役。** 79 内含 72 个 ROS 包业务/适配文件、5 个 tools 监督/停止模块、2 个日志适配。

当前 setup.py 合计 **20 个 Python console entry**：13 个尚需迁移的运行入口、1 个已有 C++ 的旧入口、5 个验证/记录入口、1 个启动校验入口。入口数与依赖库文件数不可相加。源码盘查范围覆盖 ws_robot/src/astribot*、tools、astribot_sdk、examples；生成 build/install/log/runs 不计源码副本。外部 third_party、ws_robot/third_party、ws_robot/deps、Livox/AWS 子模块以及 astribot_msgs/local 生成消息不计自有迁移数。

## 2. 方法与限制

工具只解析 AST，不 import 被盘查模块。读取 setup(...) 的 console_scripts，解析字面量 Node(package/executable/condition)，构建绝对/相对/本地脚本导入图；从 console 与已核对 shell/launch 的 supervisor 根求闭包。JSON 对每文件保存角色、类/公开 API、ROS 接口调用表达式和行号、SDK 调用、参数、依赖/调用者、绑定 import、C++ 状态、优先级、迁移边界、验证要求与阻塞。分组内每文件继承下列状态/边界；JSON 保存完整字段。

可选函数内 import 保守算可达；不等于每个分支实际执行。observation_adapters 和 vla_server 支持动态 import，工具不会把任意用户插件猜成仓库已有能力。launch 的动态辅助函数、shell/subprocess 拼接调用经人工核对，不把字面量 Node 表当完整实时节点图。现有 install 仅看文件/shebang/ELF，不 source、不执行；旧 wrapper 存在不等于今天可成功启动。

## 3. 当前运行时逐项边界

| 迁移单元 | Python 文件 | 优先级 |
|---|---:|---|
| SDK 桥接与执行 | 19 | P0 |
| 策略 observer/controller 与纯核心 | 32 | P1 |
| 导航任务仲裁 | 0（本轮已迁移） | 已完成隔离验证 |
| 默认臂底盘耦合 | 0（本轮已迁移） | 已完成隔离验证 |
| 定位地图适配与操作 | 2 | P2 |
| 仍被调用的几何库 | 1 | P1 |
| 社会导航仿真输入 | 4 | P2 |
| 搬运 RGB-D 模板观测 | 2 | P1 |
| 搬运事务与 ROS 后端 | 8 | P1 |
| VLA 协议与推理服务 | 4 | P2 |
| tools 持续监督与停止所有权 | 5 | P2 |
| 运行日志 Python 适配 | 2 | P3 |

### SDK 桥接与执行（P0）

生产调用链：桥接启动→bridge_container；state_bridge.launch；hardware_sensors→chassis_odom_node。

C++ 状态：C++ bridge_runtime/chassis_math 已有核心。ROS 容器、消息适配、单 SDK 会话与写准入仍为 Python；不能据核心存在声称原生桥接完成。

ROS/SDK 接口：FollowJointTrajectory；DispatchWaypoints；SetGripper；Twist/scan→底盘；enable/disable/reset_leash；JointState/Odometry/TF；SessionPort→厂家 API。

迁移边界、验证与阻塞：保持 heartbeat/control rights、filtered/direct、stop/hold、限位、leash、取消与保持载荷；先 fake-port 完整写轨迹，再隔离 ROS，最后厂家 SDK/真机。厂家原生接口契约是阻塞项。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [arm_bridge_core.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/arm_bridge_core.py) | 机械臂轨迹执行核心。**不依赖 rclpy、不依赖厂商 SDK。** | SDK: self._session.get_joints_position_limit; self._session.get_current_joints_position; self._session.set_joints_position | arm_traj_bridge_node |
| [arm_traj_bridge_node.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/arm_traj_bridge_node.py) | 机械臂轨迹桥接节点（Gate 3）。**薄适配层** —— 执行逻辑全在 ArmTrajExecutor。 | service（参数/类型见 JSON）; ActionServer（参数/类型见 JSON）; service（参数/类型见 JSON） | bridge_container |
| [arm_traj_math.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/arm_traj_math.py) | 机械臂桥接的纯函数核心：限位自检、轨迹插值、收敛判据、路点整形。 | Python API: ArmConfigError, LimitCheckResult, assert_limit_order, cross_check_limits, check_within_limits | arm_bridge_core, arm_traj_bridge_node |
| [bridge_container.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/bridge_container.py) | 桥接容器进程（方案 S-1：单进程、单 SDK 会话、多 ROS 节点）。 | Python API: main | console:bridge_container |
| [callback_layout.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/callback_layout.py) | 回调组布局：每个桥接节点有哪些回调组，以及执行器该开几个线程。 | Python API: CallbackLayoutError, check_group_names, make_groups, executor_thread_count | arm_traj_bridge_node, bridge_container, chassis_cmd_bridge_node |
| [chassis_bridge_core.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_bridge_core.py) | 底盘桥接的控制核心（状态机）。**不依赖 rclpy、不依赖厂商 SDK。** | SDK: self.session.get_current_joints_position; self.session.set_joints_position; self.session.get_current_joints_position | chassis_cmd_bridge_node |
| [chassis_cmd_bridge_node.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_cmd_bridge_node.py) | 底盘控制桥接节点（Gate 5）。**薄适配层** —— 控制逻辑全在 ChassisBridgeCore。 | subscription（参数/类型见 JSON）; publisher（参数/类型见 JSON）; service（参数/类型见 JSON） … | bridge_container |
| [chassis_feedback.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_feedback.py) | SDK 指令位置误差 leash 与只读位姿诊断。 | Python API: LeashState, validate_leash_config, check_leash, leash_recover_command, effective_thresholds | chassis_bridge_core, chassis_odom_source |
| [chassis_integrator.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_integrator.py) | 底盘位置积分的纯函数核心（不依赖 rclpy / 不依赖厂商 SDK，可离线单测）。 | Python API: ChassisConfigError, wrap_angle, PoseFrameIntegrator, SdkPoseHistory, local_pose_displacement | chassis_bridge_core, chassis_feedback, chassis_odom_source |
| [chassis_odom_node.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_odom_node.py) | 发布 `odom` 话题与 `odom → astribot_torso_base` TF。**纯只读，不含任何运动指令。** | publisher（参数/类型见 JSON）; TransformBroadcaster（参数/类型见 JSON） | console:chassis_odom_node |
| [chassis_odom_source.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_odom_source.py) | SDK 底盘位姿/速度 → odom 的**纯逻辑**层（不 import rclpy / 不 import 消息类型）。 | Python API: OdomSourceError, OdomSample, OdomStats, ChassisOdomSource | chassis_odom_node |
| [gripper_core.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/gripper_core.py) | 夹爪服务的核心逻辑。**不依赖 rclpy**，只依赖 SessionPort。 | SDK: self.session.set_joints_position; self.session.get_current_joints_position; self.session.get_current_joints_position | arm_traj_bridge_node |
| [gripper_math.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/gripper_math.py) | 夹爪命令空间与弧度的换算。**纯函数，不依赖 rclpy、不依赖厂商 SDK。** | Python API: GripperConfigError, clamp_cmd, is_cmd_in_range, cmd_to_rad, rad_to_cmd | arm_traj_bridge_node, gripper_core |
| [loop_timing.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/loop_timing.py) | Bounded wall-time diagnostics, independent of the ROS control clock. | Python API: LoopTiming | chassis_bridge_core |
| [ports.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/ports.py) | 桥接与外部世界之间的端口（Port）抽象，以及供测试用的替身（Fake）。 | Python API: SessionPort, PosePort, ClockPort, FakeClock, FakePose | ros_ports, sdk_session |
| [ros_ports.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/ros_ports.py) | ROS 侧端口实现（TF 位姿源、ROS 时钟）与状态上报。 | TransformListener（参数/类型见 JSON）; publisher（参数/类型见 JSON） | arm_traj_bridge_node, chassis_cmd_bridge_node |
| [sdk_session.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/sdk_session.py) | 厂商 SDK 会话适配器：把 ``Astribot`` 实例包装成 SessionPort。 | SDK: self._bot.get_desired_joints_position; self._bot.get_current_joints_position; self._bot.get_current_joints_velocity | bridge_container, chassis_odom_node |
| [state_bridge_node.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/state_bridge_node.py) | 把部件关节反馈映射为 /joint_states，沿用 bridge.yaml 的顺序和单位换算。 | publisher（参数/类型见 JSON）; subscription / | console:state_bridge_node |
| [write_gate.py](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/write_gate.py) | 写通路准入（WriteGate）与环境变量守卫。纯逻辑，不依赖 rclpy / SDK。 | Python API: is_simulation_mode, GateDecision, evaluate_write_gate, validate_pose_source_target_combo, is_astribot_log_enabled | arm_traj_bridge_node, chassis_cmd_bridge_node, sdk_session |

### 策略 observer/controller 与纯核心（P1）

生产调用链：navigation.launch→policy_controller；已安装 policy_observer；PolicyNode 继承/复用 observer。

C++ 状态：最终保护、包络、速度坐标转换、scan 适配已有直接 C++；32 个此组文件仍被生产 Python 入口导入。已有 native 数学不能替代完整 P2/P3/P4/P5/H2 决策适配。

ROS/SDK 接口：LaserScan/Odometry/OccupancyGrid/Path/CameraInfo/观测→融合/风险；MotionConstraint、SensorHealthArray、NavigationPolicyStatus；PlanCandidate/ResolveRoute；start maneuver/corridor 接口。

迁移边界、验证与阻塞：保留采集期限/epoch、候选预算、路径所有权、窄通道阶段、启动转向/退出、社会策略、取消屏障；每阶段固定输入回放和隔离 Nav2 闭环。rclcpp observer 候选仍缺 Stamp 边界闭环；完整 controller 尚缺。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [behavior.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/behavior.py) | Episode-based behavior selection, independent of ROS and controller internals. | Python API: Selection, requires_stop, YieldPolicy | corridor, policy_node, route_coordinator, social_behavior 等（完整见 JSON） |
| [candidate_safety.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/candidate_safety.py) | Conservative candidate sweep, including slower motion and continued waiting. | Python API: candidate_clearance | route_coordinator |
| [candidate_variants.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/candidate_variants.py) | Bounded, endpoint-preserving alternatives; every result needs full validation. | Python API: lateral_variants | route_coordinator |
| [continuous_sweep.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/continuous_sweep.py) | Bound complete constant-twist intervals against swept obstacle boxes. | Python API: motion_clearance | candidate_safety, protection, risk, start_maneuver |
| [contracts.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/contracts.py) | Immutable SI-unit contracts; image-space geometry is explicitly separate. | Python API: ErrorCode, ContractError, require, finite, label | corridor, corridor_adapter, corridor_detection, execution_context 等（完整见 JSON） |
| [corridor.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/corridor.py) | Straight-corridor admission and local passage permits, independent of ROS. | Python API: angle, Corridor, Passage, CorridorPolicy | corridor_adapter, corridor_detection, fixed_corridor |
| [corridor_adapter.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/corridor_adapter.py) | Map annotations and fused metric observations feed the corridor domain policy. | publisher /navigation/passage_assessment | policy_node |
| [corridor_detection.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/corridor_detection.py) | Generate straight-passage candidates from known occupied map boundaries. | Python API: Grid, detect_corridors | corridor_adapter |
| [execution_context.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/execution_context.py) | World identity and transport conversion, independent of ROS executors. | Python API: ExecutionContext, to_wire | observer_node, policy_node |
| [fixed_corridor.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/fixed_corridor.py) | Asymmetric admission for an actually held posture. No implicit home fallback. | Python API: FixedCorridorPolicy | corridor_adapter |
| [fusion.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/fusion.py) | Conservative observation association. Missing observations are not free space. | Python API: expanded_covariance, translate, prediction_model, fitted_velocity, ConservativeFusion | observer_node, world_geometry |
| [motion_geometry.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/motion_geometry.py) | Body-frame constant-twist envelope shared by prediction and final protection. | Python API: stopping_horizon, body_pose, sampling_margin | behavior, continuous_sweep, protection, risk |
| [observation_adapters.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/observation_adapters.py) | Replaceable observation adapters; no motion or planning authority. | Python API: rotate, VisionAdapter, PointCloudBoxAdapter, adapter_class, make_adapter | observer_node |
| [observer_node.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/observer_node.py) | ROS observation adapter; this node has no command or planning publishers. | TransformListener（参数/类型见 JSON）; publisher /navigation_policy/observation; subscription（参数/类型见 JSON） … | console:policy_observer, corridor_adapter, policy_node, start_maneuver_adapter |
| [obstruction_retry.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/obstruction_retry.py) | Retry only after persistent geometric change, independently of track IDs. | Python API: geometry_snapshot, equivalent, ObstructionRetry | route_coordinator |
| [path_evidence.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/path_evidence.py) | Path-bound collision evidence; stale data never grants distant-risk release. | Python API: path_identity, PathEvidence, PathAssessment, assess_path | policy_node, route_coordinator, start_maneuver_adapter |
| [planning_session.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/planning_session.py) | P3 request ownership and budgets. This module grants no motion authority. | Python API: PlanningBudget, PlanningRequest, PlanningBudgetExhausted, PlanningSession | route_coordinator |
| [policy_node.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/policy_node.py) | Stage P2 adapter: shared observations, one leased constraint output. | subscription /navigation_policy/path_blocked; subscription /navigation_policy/path_risk; publisher /navigation_policy/proposed_constraint … | console:policy_controller |
| [ports.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/ports.py) | Dependency-inversion ports. Implementations belong to later validated stages. | Python API: Prediction, PredictionModel, TrackedObstacle, WorldSnapshot, ObservationAdapter | fusion, social_adapter |
| [profile.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/profile.py) | Explicitly scoped simulation parameters; hardware evidence is a separate input. | Python API: Profile | observer_node |
| [protection.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/protection.py) | Final planar command restriction; a restriction cannot create motion. | Python API: scan_usable, costmap_clearing_ranges, swept_point_collision, CommandRestriction | observer_node |
| [risk.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/risk.py) | Swept planar-envelope risk for conservative simulation transport profiles. | Python API: RobotState, Risk, path_position, remaining_path, box_clearance | observer_node, policy_node, social_behavior |
| [robot_envelope.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/robot_envelope.py) | Shared bounds for policy and independent protection; fixed simulation defaults remain unchanged. | publisher /navigation/envelope_applied; subscription /navigation/envelope_v2; subscription /navigation/robot_envelope | observer_node |
| [route_coordinator.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/route_coordinator.py) | P3 asynchronous candidate arbitration; the behavior tree alone commits paths. | client /path_tracking/plan_candidate; service /navigation_policy/resolve_route | policy_node |
| [sensor_health.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/sensor_health.py) | Acquisition-time health and directional depth coverage, independent of ROS. | Python API: SensorHealthRegistry, scan_coverage, movement_directions, CameraCalibrationRegistry, coverage_allows_motion | observer_node, policy_node |
| [social_adapter.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/social_adapter.py) | Validated social observations restrict the existing policy, never publish Twist. | publisher /social_navigation/behavior_status; subscription /social_navigation/observed_agents; service /navigation_policy/resolve_route | policy_node |
| [social_behavior.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/social_behavior.py) | Behavior-level social scoring; hard risk remains the existing envelope predictor. | Python API: SocialDecision, interaction_cost, SocialPolicy | social_adapter |
| [start_maneuver.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/start_maneuver.py) | Full-turn admission and bounded x-minus exit; no normal tracking control. | Python API: Maneuver, ManeuverScene, StartManeuverPolicy | start_maneuver_adapter |
| [start_maneuver_adapter.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/start_maneuver_adapter.py) | Bind start maneuvers to a live controller request, path and world generation. | subscription /path_tracking/start_maneuver_request; publisher /navigation_policy/start_maneuver | policy_node |
| [stop_reference.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/stop_reference.py) | Use historical chassis peak excursion as a conservative simulation prior. | Python API: apply_stop_reference, describe_stop_reference | profile, social_adapter |
| [swept_geometry.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/swept_geometry.py) | Shared footprint separation from uncertainty-expanded swept obstacle boxes. | Python API: bounds_many, clearance_many, footprint_axes, obstacle_bounds, footprint_clearance | candidate_safety, continuous_sweep, risk, world_geometry |
| [world_geometry.py](../ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/world_geometry.py) | Shared immutable array view of explicit and parametric obstacle predictions. | Python API: PredictionRows, prediction_rows, has_predictions, prediction_count, final_prediction | candidate_safety, corridor_adapter, observer_node, obstruction_retry 等（完整见 JSON） |

### 导航任务仲裁（已迁移）

生产调用链：navigation.launch 唯一选择 `astribot_s1_task_arbiter_native/task_arbiter_cpp`，旧 Python console/module 已删除。两个 Nav2 action 类型、三个请求来源、反馈/结果转发、任务状态 topic 均保留。

取消 ACK、取消拒绝和新任务 handover 超时都不释放旧后端；只在后端终态后交出控制权。保持启动时读取 handover_timeout_s 的既有行为，没有新增整机调度协议。

安装后 77/77 协议/入口检查、ownership CTest 1/1；五组配对各 500 个假 Nav2 即时任务成功。中位 submit→terminal 23.583→3.957 ms，p95 24.886→6.245 ms。未验证真实 Nav2、实测停稳、重启恢复或硬件。

源码：[task_arbiter_node.cpp](../ws_robot/src/astribot_s1_task_arbiter_native/src/task_arbiter_node.cpp)；[完整证据](evidence/pybind_removal_20260921/task_arbiter/README.md)；[冻结测试参考](../ws_robot/src/astribot_s1_navigation_policy/test/reference/task_arbiter_node.py)。本项不再计入剩余 Python 运行时。

### 默认臂底盘耦合（已迁移）

生产调用链仍为 navigation.launch 默认 enable_arm_chassis_coupling=true → arm_chassis_coupling.launch；同名 `arm_chassis_speed_coupling_node` 安装为 C++ ELF，launch/config 逐字节保持。生产 Python 包、console 和数学实现已删除，参考仅在 test/reference，不安装。

保留 Twist/JointState/TF、原参数消费时机、EMA、精确时效边界、缓存、异常处理与已记录的非有限值行为。独立复查修复了告警限流时钟差异，使其与 Python 一样使用系统时间。

10,027 核心输入、30 ROS 用例（合计 31 pytest、2 CTest）和安装后 3 项 ROS smoke 通过。四组交替实验 8,000/8,000 输出关联成功；单核 CPU 中位 5.40%→0.85%，RSS 65.90→24.11 MiB，p95 0.715→0.357 ms；最大单条延迟 3.784→3.933 ms，不能称所有尾延迟都改善。未验证 Gazebo/实机或长期泄漏。

源码：[arm_chassis_speed_coupling_node.cpp](../ws_robot/src/astribot_s1_dynamics_coupling/src/arm_chassis_speed_coupling_node.cpp)；[完整证据与继承缺陷](evidence/pybind_removal_20260921/dynamics_coupling/README.md)；[冻结测试参考](../ws_robot/src/astribot_s1_dynamics_coupling/test/reference/README.md)。本项 2 个文件不再计入剩余 Python 运行时。

### 定位地图适配与操作（P2，剩余 2 文件）

`map_odom_tf_node` 和 `map_domain_relay` 已由新包 `astribot_s1_perception_native` 的同名 C++ ELF 提供。两处 launch 只选择 native 包；原 console 和 3 个生产模块均删除，冻结 Python 仅在 test/reference，不安装。保留单个 map→odom 写者、原 SE(2) / 高度差、源时效、ROS 时钟/缓存/启动看门狗，以及双域单向完整地图转发、QoS、参数和首图超时语义。

已验证范围、原始失败、性能和继承限制分别见 [TF 证据](evidence/pybind_removal_20260921/map_odom/README.md)、[中继证据](evidence/pybind_removal_20260921/map_relay/README.md)；[干净安装证据](evidence/pybind_removal_20260921/perception_delivery/install_audit.json)。这不等于完整 SLAM / Nav2 或跨机硬件验收。

剩余生产调用链：perception_slam 可选巡游；slam_session CLI。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [autonomous_patrol_node.py](../ws_robot/src/astribot_s1_perception/astribot_s1_perception/autonomous_patrol_node.py) | 不依赖 Nav2 的反应式自主巡游 | scan/odom→Twist | console:autonomous_patrol_node |
| [slam_session.py](../ws_robot/src/astribot_s1_perception/astribot_s1_perception/slam_session.py) | Voxel session 操作接口 | /voxel_slam/keyframe_pose_array、/odom、/voxelslam 参数与保存确认 | console:slam_session |

后续迁移须保持巡游控制权与故障停止、保存完整性和确认；巡游需要独立闭环。slam_session 是操作接口，不能当验证脚本排除。

### 仍被调用的几何库（P1）

生产调用链：policy fixed_corridor、robot_envelope、swept_geometry→polygon。

C++ 状态：geometry_state 与 geometry_core 已 C++；polygon.py 仍是生产 Python 消费者接口。

ROS/SDK 接口：无独立 ROS 入口；凸包、膨胀、包含、投影、geometry_hash 等 Python API。

迁移边界、验证与阻塞：保留非矩形/偏置载荷、float32 canonical 顺序与 hash、保守距离；迁完所有调用者才能删 _geometry_native。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [polygon.py](../ws_robot/src/astribot_s1_robot_geometry/astribot_s1_robot_geometry/polygon.py) | Convex polygons and filled-polygon/box distances in metres; no ROS dependency. | Python API: hull, validate, transform, inflate, contains | fixed_corridor, robot_envelope, swept_geometry |

### 社会导航仿真输入（P2）

生产调用链：observe.launch→hunav_truth_adapter/social_observer；warehouse 通过辅助函数启动 scenario_provider；policy.social_adapter 消费 contracts。

C++ 状态：物理 proxy 等 C++ 不覆盖 Python 的观测校验、TF 和场景服务。

ROS/SDK 接口：HuNav AgentArray→SocialAgentArray；/social_navigation/observed_agents；GetAgents /get_agents。

迁移边界、验证与阻塞：保持 simulation truth 来源标记、帧/时效/样本顺序和无运动权限；HuNav fixture 回放/隔离验证，不升级为真实人检测。scenario.py 同时含 launch 资产生成和持续 GetAgents 服务，因此计入。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [contracts.py](../ws_robot/src/astribot_s1_social_navigation/astribot_s1_social_navigation/contracts.py) | Validation independent of ROS executors and transport. SI units throughout. | Python API: stamp_ns, covariance_valid, validate_sample, SampleOrder | social_adapter, hunav_adapter, observer_node |
| [hunav_adapter.py](../ws_robot/src/astribot_s1_social_navigation/astribot_s1_social_navigation/hunav_adapter.py) | Simulation truth adapter. Deliberately strips behavior, desired speed and goals. | publisher /simulation/social_agents_truth; subscription（参数/类型见 JSON） | console:hunav_truth_adapter |
| [observer_node.py](../ws_robot/src/astribot_s1_social_navigation/astribot_s1_social_navigation/observer_node.py) | Validate and transform observations; never publish motion or planning requests. | TransformListener（参数/类型见 JSON）; publisher /social_navigation/observed_agents; publisher /social_navigation/observation_status … | console:social_observer, social_adapter |
| [scenario.py](../ws_robot/src/astribot_s1_social_navigation/astribot_s1_social_navigation/scenario.py) | Optional HuNav scene preparation; retain the existing warehouse and physics. | service /get_agents | console:social_scenario_provider |

### 搬运 RGB-D 模板观测（P1）

生产调用链：transport_support.launch→camera_observer→rgbd。

C++ 状态：C++ RGB-D/YOLO/health 节点已存在，但其强类型消息不证明取代旧 orange-box JSON 观察契约。

ROS/SDK 接口：RGB/Depth/CameraInfo+采集时刻 TF→/transport/object_observation 与 /transport/camera_health。

迁移边界、验证与阻塞：保留配帧、stride/encoding、标定 epoch、深度尺度与 stale 拒绝；图像回放加搬运消费者验证。当前 transport_support 仍用 Python camera_observer。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [camera_node.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/camera_node.py) | RGB-D observation adapter for the controlled simulation transport scenario. | TransformListener（参数/类型见 JSON）; publisher /transport/object_observation; publisher /transport/camera_health … | console:camera_observer |
| [rgbd.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/rgbd.py) | Simulation template detector: orange box, aligned RGB-D, calibrated pinhole. | Python API: localize_orange_box, project_component | camera_node |

### 搬运事务与 ROS 后端（P1）

生产调用链：已安装 transport_task→RosBackend→TransportTask；启动支持节点和服务来自 transport_support。

C++ 状态：C++ MTC planner、execution guard、geometry 已有；8 个此组文件仍承担完整事务、世界/载荷/计划账本、ROS 适配。

ROS/SDK 接口：PlanSkill/PlanManipulation、ExecuteTrajectory、FollowJointTrajectory、NavigateToPose/ThroughPoses；planning scene、attach state、execution guard、SetFixedEnvelope、ArmHoldStatus、/transport/cancel。

迁移边界、验证与阻塞：保持任务资源/计划单次使用、MTC 控制边界、取消后清理、实际 attach/detach 确认、载荷版本/hold/source TTL；完整 pick/carry/place 与单臂故障、失联、取消验证。SourceInbox 仍引用 geometry binding。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [core.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/core.py) | Transport transaction, independent of ROS; all evidence comes from a backend. | Python API: TaskFailure, Canceled, ResourceLease, ObjectRecord, Ledger | geometry, plan_guard, rgbd, ros_backend 等（完整见 JSON） |
| [corridor_route.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/corridor_route.py) | Explicit straight-corridor intent and independently sampled traversal proof. | Python API: CorridorRoute, TraversalWitness | core, ros_backend |
| [fixed_hold.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/fixed_hold.py) | Measured settling before committing a fixed navigation posture. | Python API: ns, StableGeometryReference | ros_backend |
| [geometry.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/geometry.py) | Conservative carried footprint from the live URDF collision primitives and FK. | Python API: box_corners, stl_bounds, collision_bounds, collision_primitive_matrix | ros_backend |
| [payload_sync.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/payload_sync.py) | Simulation attachment identity and capture-time confirmation contract. | Python API: physical_parent, confirmed_state | ros_backend |
| [plan_guard.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/plan_guard.py) | Single-use ordered plan contract; deliberately independent of ROS. | Python API: stage_name, PlanGuard | ros_backend |
| [ros_backend.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/ros_backend.py) | ROS/MoveIt implementation. Gazebo payload is a kinematic surrogate, not a force grasp. | subscription /model/; client /transport/execution_guard/set; subscription /transport/execution_guard/status … | console:transport_task |
| [source_inbox.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/source_inbox.py) | Bounded ROS payload storage; source-time selection is owned by C++. | Python API: SourceInbox | ros_backend |

### VLA 协议与推理服务（P2）

生产调用链：transport.ros_backend→vla_ros/contract/policy；已安装 vla_policy_server。

C++ 状态：没有找到等价 C++ HTTP/协议适配；现有是模型无关协议与 reference adapter，未证明训练模型闭环。

ROS/SDK 接口：astribot.vla/1 JSON HTTP request/proposal；RGB-D 观察；策略建议经搬运任务 owner 审核。

迁移边界、验证与阻塞：保留 size/timeout/TTL、帧/标定/对象版本与 proposal 限权，验证畸形/超时/过期回复和任务取消；不能让模型直接写执行器。外部动态 adapter 需显式 C++ 插件/协议边界。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [vla_contract.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/vla_contract.py) | Versioned, ROS-independent policy boundary. All outputs are proposals, never commands. | Python API: fail, digest, numbers, validate_config, validate_capabilities | ros_backend, vla_policy, vla_ros, vla_server |
| [vla_policy.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/vla_policy.py) | Policy SDK and bounded HTTP client; inference has no ROS/controller dependencies. | Python API: PolicyAdapter, ReferencePolicy, HttpPolicy, PolicySession | vla_ros, vla_server |
| [vla_ros.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/vla_ros.py) | Observation/proposal adapter inside the existing transport task owner. | subscription（参数/类型见 JSON） | ros_backend |
| [vla_server.py](../ws_robot/src/astribot_s1_transport/astribot_s1_transport/vla_server.py) | Inference-only adapter host: no ROS imports, world creation or actuator access. | Python API: make_server, main | console:vla_policy_server |

### tools 持续监督与停止所有权（P2）

生产调用链：launch_sim_stack.sh/sim_stack.launch→sim_stack_supervisor；run_deployed.sh→hardware_exploration；stop_robot_tasks.sh→robot_task_control。

C++ 状态：没有找到等价 C++ supervisor；这些文件混合启动说明与持续会话/进程/停止逻辑，不能按 tools 目录统一排除。

ROS/SDK 接口：PID/start-time/lock/session 生命周期；ROS lifecycle 与 enable；stop/cancel/Twist/odom；ROS_DOMAIN_ID 与 Gazebo partition。

迁移边界、验证与阻塞：将持续监督部分迁出时保留会话所有权、pidfd、防 PID 复用、信号、日志和停止顺序；启动描述可留 Python。只测自有进程。sim_stack_probe/hardware_readiness 为有界启动就绪校验，明确排除。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [hardware_exploration.py](../tools/robot/hardware_exploration.py) | Supervise hardware sensors, manual navigation, or autonomous exploration. | TransformListener（参数/类型见 JSON）; client /chassis_cmd_bridge/enable; subscription /mapping_session/status … | 上表 shell/launch/监督器调用 |
| [hardware_sensors.py](../tools/robot/hardware_sensors.py) | Deployment descriptions and read-only readiness checks for the hardware sensor chain. | subscription（参数/类型见 JSON） | hardware_exploration, robot_task_control |
| [robot_task_control.py](../tools/robot/robot_task_control.py) | Stop navigation, then SLAM/lidar using captured PID identities; retain robot body drivers. | subscription /odom; publisher（参数/类型见 JSON）; client（参数/类型见 JSON） | hardware_exploration |
| [sim_isolation.py](../tools/sim_isolation.py) | Opt-in identities for concurrent copies of the canonical simulation world. | Python API: is_stack_process, SimulationIsolation | sim_stack_supervisor |
| [sim_stack_supervisor.py](../tools/sim_stack_supervisor.py) | Own one simulation session; start navigation only after measured data is ready. | Python API: process_identity, owned_descendants, live_owned, finish_descendants, arguments | 上表 shell/launch/监督器调用 |

### 运行日志 Python 适配（P3）

生产调用链：Python 节点/SDK logger 和两个 supervisor。

C++ 状态：C++ spdlog 后端已存在，Python 通过 ctypes C ABI 接入，不是 pybind。

ROS/SDK 接口：Logger/SessionLog/ProcessOutput→libastribot_spdlog.so；session.log 与轮转文件。

迁移边界、验证与阻塞：业务运行时迁走后，允许启动/验证工具保留此适配；无需为去自有 pybind 把允许保留的 Python 启动链重写。守住 fork、轮转、flush、会话路径。

| 文件 | 角色/公开核心 | 接口摘录 | 当前生产调用者 |
|---|---|---|---|
| [__init__.py](../ws_robot/src/astribot_logging/astribot_logging/__init__.py) | spdlog-backed Python diagnostics; importing this module has no side effects. | Python API: log_level, log_directory, SpdlogHandler, get_logger, configure_stdlib_logger | sim_stack_supervisor, output, map_odom_tf_node, slam_session 等（完整见 JSON） |
| [output.py](../ws_robot/src/astribot_logging/astribot_logging/output.py) | File-only spdlog sinks for launch and captured process output. | Python API: FileLog, SessionLog, SessionHandler, ConsoleOnlyLaunchHandler, LaunchFileHandler | hardware_exploration, sim_stack_supervisor |

## 4. 已有 C++ 的源码与旧入口

**下表 17 文件已经移到显式验证目录。**最初盘查时它们虽不在生产调用闭包，仍会被 Python 安装器打包；后续清理已修正该问题，并删除 main/console/launch 兼容入口。现在生产源码导入和全新安装都找不到这 17 个模块，历史参考只由验证脚本启用，详见 [验证记录](evidence/pybind_removal_20260921/reference_retirement/README.md)。

| 参考文件 | 已有 C++ 对应 | 处置边界 |
|---|---|---|
| [omni_effort_drive_node.py](../tools/migration/python_reference/astribot_s1_chassis_effort_drive/omni_effort_drive_node.py) | astribot_s1_chassis_effort_drive_native/omni_effort_drive_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [arm_reach_metric.py](../tools/migration/python_reference/astribot_s1_navigation/arm_reach_metric.py) | astribot_trajectory_bridge_native/chassis_math; arm_speed_limiter_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [arm_speed_limiter_node.py](../tools/migration/python_reference/astribot_s1_navigation/arm_speed_limiter_node.py) | astribot_trajectory_bridge_native/arm_speed_limiter_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [cmd_vel_body_to_world_node.py](../tools/migration/python_reference/astribot_s1_navigation/cmd_vel_body_to_world_node.py) | astribot_s1_navigation_policy_native/cmd_vel_body_to_world_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [posture_monitor_policy.py](../tools/migration/python_reference/astribot_s1_navigation/posture_monitor_policy.py) | astribot_s1_navigation_policy_native/cmd_vel_body_to_world_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [cmd_vel_math.py](../tools/migration/python_reference/astribot_s1_navigation_policy/cmd_vel_math.py) | astribot_s1_navigation_policy_native/cmd_vel_body_to_world_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [control_time.py](../tools/migration/python_reference/astribot_s1_navigation_policy/control_time.py) | astribot_s1_navigation_policy_native/navigation_math ControlTime; final_protection_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [costmap_scan_node.py](../tools/migration/python_reference/astribot_s1_navigation_policy/costmap_scan_node.py) | astribot_s1_navigation_policy_native/costmap_scan_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [envelope_node.py](../tools/migration/python_reference/astribot_s1_navigation_policy/envelope_node.py) | astribot_s1_navigation_policy_native/envelope_coordinator_cpp + fixed_envelope_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [fixed_envelope.py](../tools/migration/python_reference/astribot_s1_navigation_policy/fixed_envelope.py) | astribot_s1_navigation_policy_native/fixed_envelope_core | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [fixed_envelope_node.py](../tools/migration/python_reference/astribot_s1_navigation_policy/fixed_envelope_node.py) | astribot_s1_navigation_policy_native/fixed_envelope_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [protection_node.py](../tools/migration/python_reference/astribot_s1_navigation_policy/protection_node.py) | astribot_s1_navigation_policy_native/final_protection_cpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [scan_occupancy.py](../tools/migration/python_reference/astribot_s1_navigation_policy/scan_occupancy.py) | astribot_s1_robot_geometry/geometry_core scan_occupied_cells; observer uses native functions directly | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [model.py](../tools/migration/python_reference/astribot_s1_robot_geometry/model.py) | astribot_s1_robot_geometry/geometry_core robot_model.hpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [node.py](../tools/migration/python_reference/astribot_s1_robot_geometry/node.py) | astribot_s1_robot_geometry/geometry_state | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [projection_contract.py](../tools/migration/python_reference/astribot_s1_robot_geometry/projection_contract.py) | astribot_s1_robot_geometry/geometry_state projection contract | 已移到显式验证目录；生产源码与干净安装均不可导入 |
| [state.py](../tools/migration/python_reference/astribot_s1_robot_geometry/state.py) | astribot_s1_robot_geometry/geometry_core joint_snapshot.hpp | 已移到显式验证目录；生产源码与干净安装均不可导入 |

另有 **transport/camera_calibration_postprocess.py**：warehouse_sim.launch 当前选择 astribot_s1_perception_components/camera_calibration_postprocess；transport/setup.py 仍声明同名 Python console entry。应完成当前标定回放/协议验证后删除旧入口并排除该参考模块的生产安装。

**不能一同删除** policy/protection.py（observer 导入 scan_usable）、geometry/polygon.py（fixed_corridor、robot_envelope、swept_geometry 仍消费）。bridge 的 _Python* 类与 facade 与真正 ROS 生产消费者同文件，不能因 C++ probe 通过就整文件列作 oracle。control_time.py 仅由旧 protection_node/测试导入；scan_occupancy.py 的生产 observer 已直接调用 native scan 函数，因此这两项属于上表参考源码。

## 5. 现有安装残留：未清理、未启用

`ws_robot/install` 发现 **10 个**当前源码不再声明的 Python 可执行：omni_effort_drive_node、arm_speed_limiter_node、cmd_vel_body_to_world_node、costmap_scan_adapter、envelope_coordinator、final_protection、cloud_to_grid_node、livox_fusion_node、livox_preprocess_node、slam_adapter_node。根目录 `install` 发现前 **6 个**同类旧入口。两处重复不能当成 16 项源码工作量。

这两套安装没有在本盘查中清理、覆盖或 activate。最终验收必须使用全新 build/install/log 目录，禁止让已有安装或 PYTHONPATH 把旧 .so/模块遮蔽回来；旧入口缺失、包安装清单、ELF 依赖和实际启动命令必须一起验证。将当前源码直接覆盖到共享 install 不等于移除旧文件。

## 6. SDK、examples、第三方边界

astribot_sdk 下 20 个 Python 文件单列，当前桥接从 sdk_session 动态进入 Astribot→AstribotInterface/ROS wrapper/robotics library。astribot_rclcpp_py_proxy.py 硬导入厂家 pybind 扩展；现场同时有 CPython 3.10 与 3.8 的对应 .so。JSON 还列出 24 个 SDK .so（含其他命名库）；文件数和 ELF 名称都不能证明每个二进制使用 pybind，也不能从导出符号推导可安全调用的 C++ 类 ABI。

下一阶段需要厂家可验证 C++ 头文件/库或官方等价协议，以及控制权/心跳、单位、过滤/直通、waypoint、stop/hold、WBC 与反馈契约。替换前仍必须承认 SDK 层 Python/二进制绑定依赖；不把 ROS 上已有厂家状态话题当成拥有等价控制权。

27 个 examples 是厂家 SDK 调用样例：含关节/笛卡尔操作、手柄控制、WBC、图像/雷达/音频、轨迹回放和 stop。它们可以作为接口事实与离线回放输入；不能算作需要上线的 27 个 C++ 服务。tools/joy_tools.py 是这些例程的 Joy 支持，依赖外部 astribot_ros_middleware；transport/vla_examples.py 是可通过动态 adapter 指定的 reference 推理样例，不代表训练模型。二者列为可选运行支持，默认生产图外。

第三方目录、Livox/AWS 子模块、生成 ROS 消息以及发行版 rclpy 内部绑定分别管理。允许 Python 启动/验证意味着发行版 rclpy._rclpy_pybind11 仍可能存在；自有绑定清理不能通过删除第三方整个目录实现。

## 7. 建议下一批与删除门禁

1. 17 个参考模块已移出生产安装，完成关联回归与干净安装检查，见 [参考模块退役证据](evidence/pybind_removal_20260921/reference_retirement/README.md)。仍须验证并退役 transport 旧标定入口；不得移除 live policy/geometry/bridge 消费者。
2. task_arbiter 与 arm_chassis_speed_coupling 已完成本轮迁移、差分与短时配对实验，并删除生产 Python 实现；真实 Nav2/驱动整链和长期稳定性仍需独立验收。后续继续处理下方剩余模块，不将本批局部通过当作全项目完成。
3. 同步推进完整 observer/controller 与 transport 消费者：覆盖所有 P2-P5/H2/非 Home 阶段、MTC/载荷/hold/取消，随后删 navigation/geometry 自有绑定。只切数学函数或只测简单走廊不满足门禁。
4. SDK 桥接先拿到原生端口契约，再把单会话 ownership、ROS action/service 与 C++ core 连起来；厂家/真机验收未完成不得报告全项目去 pybind 成功。
5. tools 监督器、地图 TF/跨域/social/VLA 协议再分单元迁移。日志 ctypes 适配可随允许保留的 Python 启动/验证保留；它不阻塞“去自有 pybind”。

每批最终状态必须同时满足：仅一个生产实现/入口；不存在 Python fallback、impl 选择开关或未声明兼容 alias；生产包不安装旧参考模块；清洁安装无旧绑定/旧可执行；静态与离线、隔离仿真、真机证据分别列明。源码盘查不替代任何运动验收。

## 8. 复现与本次验证

```bash
python3 tools/migration/inventory_python_runtime.py --root "$PWD" \
  --output docs/evidence/pybind_removal_20260921/python_runtime_inventory_latest.json
```

盘查脚本自身只解析源码、入口和导入关系，不执行生产模块。最新快照 471 文件无 AST 错误、20 console targets 均可定位，23 个验证参考无生产导入者。各项迁移和参考模块退役的离线/隔离 ROS 验证单独链接在上文，不属于盘查脚本的能力。原始 85 文件快照仍保存在 `python_runtime_inventory.json`，82 文件阶段留在 Git `1b2c33a4`，当前 79 文件快照为 `python_runtime_inventory_latest.json`。没有 Gazebo 闭环、真机或长时稳定性验收。

下一阶段完整策略入口/绑定删除边界见 [策略迁移施工边界](CPP_POLICY_MIGRATION_NEXT_20260921.md)；单独替换 observer 不能删除 navigation 绑定，geometry 还需完成搬运消费者。
