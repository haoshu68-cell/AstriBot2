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
