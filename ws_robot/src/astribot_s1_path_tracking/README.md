# 路径跟踪与终点精调

## 当前代码链路

```text
目标位姿
  → ExactGoalPlanner：Smac 路径规划，末点保留原始目标坐标
  → ArrivalController：内层 MPPI/RPP 跟踪 → 接近限速 → 终点 XY/yaw 精调
  → ArrivalGoalChecker：同时校验误差与停稳
  → Nav2 速度平滑 → 底盘
```

| 文件 | 职责 |
|---|---|
| `src/exact_goal_planner.cpp` | 使用 Smac 搜索，消除栅格中心取代用户目标产生的终点偏差 |
| `src/arrival_controller.cpp` | 内层控制器生命周期、定位来源、精调、碰撞及不可达处理 |
| `src/curvature_speed_limit_critic.cpp` | MPPI 曲率与侧向加速度代价 |
| `src/curvature_speed_math.cpp` | 曲率、前視距离及制动包络计算 |
| `include/.../cost_accumulate.hpp` | 原地累加代价，保留 Nav2 分配的对齐缓冲区 |
| `plugins.xml` / `critics.xml` | Nav2 插件导出 |

ThreePhaseController 已按清理前备份恢复，ArrivalController 继承它。
捕获区外复用原起步对齐、内层 MPPI/RPP 跟踪和接近限速；捕获区内由既有精调逻辑接管。
独立 ThreePhaseController 插件也保留，原终点旋转策略与 GoalChecker 配合使用。

| 原核心参数 | MPPI | RPP |
|---|---:|---:|
| `align_start_enabled` / `align_goal_enabled` | true / true | true / true |
| `zero_vy_in_follow` | false | true |
| `align_kp` / `align_max_vel` / `align_floor_vel` | 1.0 / 0.6 / 0.05 | 1.0 / 0.6 / 0.05 |
| `align_tolerance` | 0.02 rad | 0.05 rad |
| `start_min_angle` / `start_heading_lookahead` | 0.2 rad / 0.5 m | 0.2 rad / 0.5 m |
| `align_timeout` / `follow_timeout` | 40 s / 0 | 35 s / 0 |
| `new_goal_epsilon` / `new_attempt_gap` | 0.25 m / 0.5 s | 0.25 m / 0.5 s |
| `approach_enabled` | true | false |
| `approach_dist` / `approach_v_min` | 1.5 m / 0.05 m/s | 同默认值 |
| `align_inertia_enabled` | true | true（默认） |
| `align_coast_lag` / `align_coast_decel` / `align_settled_wz` | 0.03 s / 7 rad/s² / 0.02 rad/s | 同默认值 |

`fallback_xy_tolerance` 和 `fallback_yaw_tolerance` 仍保留原值。
精调模式使用 ArrivalGoalChecker 的严格容差；原终点对齐参数不取代 arrival 参数。
`start_min_angle` 只判断是否触发起步旋转。触发后，须达到 `align_tolerance`，
并在启用惯性补偿时满足 `align_settled_wz`，才进入跟踪；不能再次用触发阈值提前退出。
初始误差不足触发阈值时仍直接跟踪。原生 PathAngleCritic 和上述参数值保持不变。

普通弯道降速候选未通过完整路线的横向误差对照，已恢复原曲率评分实现与参数。
当前 MPPI 配置开启 PathAlignCritic 方向评分，将其启用进度门槛从 20 点降到 6 点，
并通过 velocity_smoother 将平移正加速度限制为 0.5 m/s²。
仿真启动脚本默认限速 0.35 m/s；跟踪总时长不参与质量评价。
同路线两轮的改善与尚未改善项见 [验证记录](../../../docs/HEADING_CONVERGENCE_VALIDATION.md)。

## 标准角点跟踪

`corner_turn_enabled` 打开后，标准折角按有序路段执行：当前段跟踪 → 角点接近 →
持续停稳 → 转向与位置保持 → 持续停稳 → 下一段。连续曲率跟踪继续使用原控制链。
当前段的起步朝向前瞻不跨越尚未完成的角点。

- 支持范围由原参数 `corner_min_angle`～`corner_max_angle`、`corner_min_segment` 定义。
  短首段、间距不足的反向双角和配置外掉头明确拒绝，并记录原因；同向短斜接保留原检测方式。
- 接近入口取原 `corner_approach_distance` 与“当前实测速度的既有制动预算＋捕获半径”的较大值。
  Arrival 复用 `arrival.braking_model`；`position_hold` 的实测残余位移曲线不外推标定范围。
  独立 ThreePhase 插件仍使用原距离入口；不能据此宣称它具有 Arrival 制动模型。
- `CORNER_APPROACH` 保持零角速度及原接近限速；进入捕获区后输出零指令，复用
  `arrival.settle_time` / `arrival.pose_timeout` 的位姿窗口。重复、乱序、过期、跳变或持续漂移不构成停稳。
  转向完成也须同时满足原位置/角度/速度条件和新源帧停稳窗口。
- `CORNER_STATE` 日志明确区分 `APPROACH`、`SETTLING_BEFORE_TURN`、`TURNING`、
  `SETTLING_AFTER_TURN`、`RECOVERING`、`REANCHOR_SETTLING`；旧高层相位话题仍兼容策略层。
  故障注入应匹配真实子状态，不能把 `FOLLOW` 解释为位置恢复。

任务代次由同一 controller_server 内的 `PolicyProgressChecker::reset()` 提供。
`setPlan()` 暂存路径，首个控制周期在 Nav2 完成任务复位后提交。
因此开启角点模式必须配套 `astribot_s1_path_tracking::PolicyProgressChecker`，MPPI/RPP 配置已统一。
同一代次等价刷新保持游标、相位及停稳/超时计时；替换路线统一停稳并重定位当前路段，
该停稳屏障也覆盖倒退脱离和走廊运动。拒绝的暂存路径保持拒绝，不能在失败容忍重试时恢复旧路线。
路径源时间的独立高水位不会被零时间戳等价刷新清除；新任务代次可明确使用缓存路径。
`new_attempt_gap` 仅保留给关闭角点且没有执行代次所有者的旧接口，不能判断当前角点任务身份。

`nav_msgs/Path` 不携带必经点语义。近终点角点仍按原终端保留距离交给 Arrival，记录
`CORNER_TERMINAL_HANDOFF`，不能算作已完成角点；必须经过/转向的任务须拆成显式航点目标。
旋转和位置保持继续使用原足迹扫掠与策略约束；只有当前 costmap 足迹确实包含双臂及载荷，
才具备对应完整外形证据。legacy 与 fixed_v2 不作等价验收。

本次 C92 修复仅完成离线编译、对象级回归和简化接近段回放；尚未运行新候选的 Gazebo/RViz
或真机验证。控制器代次与 Nav2 action 调用顺序、动态中断、完整外形、多角度重复及 A/B
跟踪质量仍待集成验收，不能将单个终点成功或离线测试通过等同于 C 阶段通过。

## 到位判定

默认同时满足以下条件，连续保持 0.6 秒后返回成功：

- 平面欧氏距离 `hypot(dx, dy) ≤ 0.03 m`。
- 最短航向误差 `abs(remainder(dyaw, 2π)) ≤ 0.02617993877991494 rad`，即 1.5°。
- 实测平移速度 ≤ 0.01 m/s、角速度 ≤ 0.01 rad/s。
- 当前足迹可通行，定位和本拍结果有效。

GoalChecker 不锁存位置达标状态。位置漂移后须重新修正，过期报告不能返回成功。
规划器必须配套使用 ExactGoalPlanner；仅把 Smac 的 tolerance 设为 0 仍可能返回栅格中心，
其量化偏差足以超过 3 cm。

## 定位来源

`FollowPath.arrival.source` 在节点配置时选择：

| 来源 | 输入 | 默认话题 |
|---|---|---|
| `nav2_pose` | Nav2 的导航 TF 基座位姿 | 由 controller_server 传入 |
| `slam_pose` | `PoseWithCovarianceStamped` | `/slam_toolbox/pose` |
| `vision` | `PoseStamped` | `/arrival/vision/pose` |
| `mark` | `PoseStamped` | `/arrival/mark/pose` |

用 `arrival.pose_topic` 覆盖外部话题。消息必须表示**导航基座在 header.frame_id 中的位姿**，
时间戳为采集时间，四元数有效，且该 frame 与路径坐标系有 TF。项目基座是
`astribot_torso_base`。视觉/Mark 的识别、置信度筛选、外参标定由上游模块负责。
例如已知标记位姿时：`T_map_base = T_map_mark × inverse(T_camera_mark) × inverse(T_base_camera)`。
不自动切换定位来源，避免在一次到位过程中改变测量基准。

## 参数与接线

MPPI/RPP 两份导航配置均使用同一对控制器和检查器：

```yaml
FollowPath:
  plugin: astribot_s1_path_tracking::ArrivalController
  inner_controller_plugin: nav2_mppi_controller::MPPIController
  arrival:
    source: nav2_pose
    capture_radius: 0.30
    pose_timeout: 0.5
    refine_timeout: 45.0
    total_timeout: 300.0
    progress_timeout: 15.0
    settle_time: 0.6
    kp_xy: 0.5
    kp_yaw: 0.8
    max_linear_speed: 0.06
    max_angular_speed: 0.15
  inner: {}  # 使用导航配置中的完整 MPPI/RPP 参数
precise_goal_checker:
  plugin: astribot_s1_path_tracking::ArrivalGoalChecker
  xy_goal_tolerance: 0.03
  yaw_goal_tolerance: 0.02617993877991494
```

进入接近区后以车体系全向速度修正位置与姿态，速度按模长限制。需要全向底盘。
控制器和自定义 critic 的参数在 configure 阶段读取；改配置后重新启动导航。
`setSpeedLimit` 仍即时生效并传递给内层；按 Nav2 约定，数值 0 表示取消限速。

## 不可达处理

异常通过 `PATH_TRACKING/<原因>` 的 PlannerException 进入 Nav2 的失败/停止链路：

| 原因 | 条件 |
|---|---|
| INVALID_PATH / INVALID_POSE / INVALID_VELOCITY | 无效路径、坐标、四元数或速度 |
| POSE_UNAVAILABLE / POSE_STALE / NAVIGATION_POSE_STALE | 定位缺失或过期 |
| TF_UNAVAILABLE / POSE_DISAGREEMENT | 坐标变换失败，或外部定位超出精调区域 |
| REFINEMENT_BLOCKED | 足迹预测碰撞、未知、越界或地图失效 |
| NO_MOTION_PROGRESS | 跟踪阶段持续无实测运动 |
| REFINEMENT_NO_PROGRESS | 精调阶段联合误差持续无改善 |
| REFINEMENT_TIMEOUT / GOAL_TIMEOUT | 精调或目标尝试超过预算 |

同目标周期重规划不会重置预算。Humble Controller 接口不提供 Action UUID，因此同目标
重新尝试以控制拍空档超过 0.5 秒识别；紧密重试可能共用预算。
碰撞检查使用导航位姿注册的局部代价地图，检查当前速度与指令速度的 1 秒足迹扫掠。
它判定局部不可执行，不证明目标在全局永久不可达。

起点检查、告警和恢复编排由 [`astribot_s1_navigation_recovery`](../astribot_s1_navigation_recovery/README.md) 负责，在最终目标路径规划前完成；需要恢复时，通过 `DeparturePlanner` / `DepartureController` 执行退出，实测停稳并复检 READY 后才继续目标规划与跟踪。ArrivalController 中的旧起步请求/回复和自动倒退分支已移除。普通 ALIGN 保留实时碰撞检查与原转向控制律，FOLLOW、MPPI/RPP、到位精调及现有政策约束保持原职责。恢复原因与阶段由 `/navigation/start_alarm` 记录。

## 构建与仿真

```bash
cd ws_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select astribot_s1_path_tracking astribot_s1_navigation
source install/setup.bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim launch_gazebo:=true headless:=false use_rviz:=true \
  controller_plugin:=mppi exploration:=false
```

本轮一次性验证程序及原测试备份放在 `/tmp/astribot_cleanup_validation`，不安装进运行包。
仿真验证不能替代真实 SLAM/视觉噪声、外参和底盘制动条件下的实机验收。

### Ground-truth 仿真精度档位（2026-09-14）

`tools/launch_sim_stack.sh --mode baseline` 自动启用 `simulation_precision`；独立 navigation launch 可传 `arrival_precision_profile:=simulation_precision`，要求 `use_sim_time:=true`。参数集中在 `astribot_s1_navigation/config/arrival_precision_sim.yaml`，验收为欧式 2 mm / 0.1°；SLAM 仿真和硬件默认 `standard`，保留原验收阈值。

末端 XY/yaw 分别以 `stop_tolerance_ratio` 进入零指令保持，超过 `resume_tolerance_ratio` 才恢复修正。`settle_drift_ratio` 限制连续稳定窗口的位姿漂移；源时间必须推进。精度档位用 `translation_yaw_tolerance` 在平移中保留粗航向控制，XY 保持后再完成精细航向对准，最终始终按严格容差验收。`min_linear_speed` / `min_angular_speed` 默认 0，仅用于已标定执行模型的低速摩擦补偿，仍服从速度上限与碰撞检查。

停稳依据所选定位源的位姿窗口，Nav2 位姿来自 SLAM 时也适用。每个轴只收集该轴输出零指令之后的位姿，至少 3 个不同时间戳，时间跨度达到 `settle_time`；采用窗口最小二乘速度和首尾平均速度的较大值，与 Checker 的 `stopped_linear_velocity` / `stopped_angular_velocity` 比较。XY 包围盒对角线及解缠 yaw 范围不得超过 `settle_drift_ratio` × 相应到位容差。非零指令、倒退/异常时间戳、数据中断清除证据。COAST 与到位确认共用该窗口，Checker 接受控制器的短时有效确认，不再叠加单帧 odom 速度判定。主动纠偏制动及碰撞预测仍使用原速度反馈，探索层的观测驻留不变。

`ARRIVAL_METRICS` 保留原始 `speed_mps` / `wz_radps`，新增 `stop_source=pose_window`、`stop_xy/yaw`、`stop_span_xy/yaw_s`、`stop_speed_mps` / `stop_wz_radps`、`stop_drift_m/rad`，便于区分单帧速度噪声、真实漂移与证据不足；仍按原采样频率记录。

实测来源、精度预算和限制见仓库 `docs/CHASSIS_CALIBRATION_AND_ARRIVAL_PRECISION_20260914.md`。精度档位不等于真机已验证能力。

运行日志默认以 `FollowPath.metrics.sample_hz=2.0` 采样横向误差和精调误差，`ARRIVAL_REACHED` 每次稳定到位立即记录。`metrics.terminal_exclusion_radius=0.5` 仅标记行进段统计范围。完整公式、字段和查看命令见仓库 `docs/PATH_TRACKING_METRICS.md`；诊断参数不影响控制/到位判定，修改后重新配置导航插件生效。


### 真机到位制动

真机与仿真精度档共用导航包的 `arrival_motion.yaml`，正常 FOLLOW / ALIGN_START 使用相同基线参数。真机 SLAM 判据为 3 cm / 1.5°，仿真真值定位为 2 mm / 0.1°。

`arrival.linear_braking_time` / `angular_braking_time` 是终点的执行器响应适配，默认 0 保持原行为；硬件档为 1.5 s。非零时，用误差减去当前速度对应的预测余移提前减速；预测越点或方向反转先输出零，等待连续新位姿时间戳下的停稳，再按当前误差修正。XY 方向在目标坐标系比较，避免机器人转动造成体坐标方向误判。重复或倒序反馈不能推进停稳计时，新目标或重新配置清除制动状态。

进展指标和到位确认都计入残余运动裕量，避免运动中过零被误判为最佳收敛、或刚进入容差就结束后继续漂出。制动转换记录 `ARRIVAL_COAST`，精调指标仍按 2 Hz 输出。模型与 Gazebo 结果不替代真机负载、地面和定位扰动下的复测。

低速平移的 SLAM/厂家反馈同窗对比与有限补偿由 `SlipMonitor` / `LinearSlip` 提供，集成在 `ArrivalController`。真机配置当前为 `monitor`，保持基线指令；可选的 `compensate` 需要真实测量时间有效。参数、输入坐标要求及验收步骤见 [低速打滑补偿说明](../../../docs/LINEAR_SLIP_COMPENSATION_20260917.md)。


### 标准角点候选的重规划与漂移边界（2026-09-22，待阶段验收）

候选仍沿用 `CORNER_APPROACH → ALIGN_CORNER → FOLLOW`。等价路径按有序顶点和入/出射方向匹配，允许共线重采样；同目标旧时间戳路径拒绝接管。几何改变须具有较新的源时间，且新路径起点接近最新机器人位姿，实际停稳后再重新对齐。已完成角点之前的路径不再交给内层跟踪器；跳过终端短角点不会删除未走过的前段。自交路径的角点投影仅在当前有序路段内计算。

转向采用既有角速度和余转控制，并在角点附近增加有死区的小幅位置反馈：默认 1 cm 死区，平移修正上限为 `corner_approach_speed / 2`（当前 0.04 m/s）；偏离超过 4 cm 暂停转向，恢复到 2 cm 内后继续。恢复范围外拒绝继续；转向完成须同时满足角度、实际速度与角点位置条件。完整剩余转向、纠偏指令及实测速度均检查机身填充轮廓扫掠。这里的 4 cm / 2 cm 来自现有 `corner_capture_radius`，没有变更速度、制动、平滑和终点阈值配置。

角点路径执行中的显著位姿/时钟不连续先于终点接管检查，异常交给 Nav2 失败停车链。它不替代定位质量判定，也不构成新的设备指令通道。Humble 的控制接口仍缺少 Action UUID；紧密重试的执行身份限制仍适用，不能把源时间和路径几何描述成完整事务协议。

接近段已停住而内层指令低于现有最小接近速度时，使用指向角点的有限纠偏，并检查该指令及实测运动的完整机身扫掠。角点接近和转向中的平移纠偏共同遵守外部绝对/百分比限速及策略限速；启用角点时也初始化内层额定速度，不依赖社会策略或打滑模块是否启用。

闭环仍有待修问题：近 180° 回头路径可能因欧式距离近而提前进入终点精调，漏走回头点；部分物理传送式位姿故障由碰撞保护而非不连续检测拦截。当前候选不可据此宣称完整路线和故障边界通过。

当前阶段、源/运行库哈希、单元与仿真证据见 `docs/evidence/unified_navigation_resume_20260921/STATUS.md`。候选未部署真机，不以局部通过代替 C1–C4 的完整回归。


## 固定姿态整机预测碰撞

`WholeBodyCollisionCritic` 将固定高度包络放到 MPPI 底盘 rollout 上，并检查相邻位姿扫掠；不预测关节运动、不做逐候选 FK。部分候选碰撞时设为无限代价，使其 MPPI 权重为零；全候选碰撞时设置批次失败并保持被丢弃批次的数值有限，避免归一化产生 NaN。地图、包络缺失或运动许可撤销/过期直接失败，没有降级到仅底盘几何。

固定包络模式的 Arrival 最终输出复核相同高度几何，包括平移、转向和精调。当前沿用一秒指令/实测速度扫掠，不能据此宣称停车包络已验收。全局二维保守包络保持原职责，因此新增高度检查不自动扩大可通行空间。

独立分支实现与分层证据见 `docs/FIXED_POSTURE_WHOLE_BODY_NAV_20260928.md`。
