# 代码包完整索引

核对日期：2026-09-29。扫描范围仅为 `ws_robot/src/*/package.xml`：**49 个包，其中 47 个 AstriBot 包、2 个第三方包**。没有把 install/share 中的重复清单、deps、测试快照或其他 worktree 计入。包的主归属是阅读索引，不表示包只能被该模块使用。

架构与状态见[总图](ARCHITECTURE_REFERENCE.md)和[模块细化](MODULE_ARCHITECTURE.md)。每个包恰好出现一次，名称取自 package.xml 而非目录猜测。

| 模块 | 包 / 源码目录 | 构建或入口文件 |
|---|---|---|
| [M01 机器人模型](MODULE_ARCHITECTURE.md#m01) | [astribot_s1_description](../../ws_robot/src/astribot_s1_description) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_description/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_s1_description/package.xml) |
| [M02 仿真和设备输入](MODULE_ARCHITECTURE.md#m02) | [astribot_s1_gazebo_bringup](../../ws_robot/src/astribot_s1_gazebo_bringup) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_gazebo_bringup/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_s1_gazebo_bringup/package.xml) |
| [M02 仿真和设备输入](MODULE_ARCHITECTURE.md#m02) | [astribot_s1_chassis_effort_drive](../../ws_robot/src/astribot_s1_chassis_effort_drive) | [setup.py](../../ws_robot/src/astribot_s1_chassis_effort_drive/setup.py)、[README.md](../../ws_robot/src/astribot_s1_chassis_effort_drive/README.md) |
| [M02 仿真和设备输入](MODULE_ARCHITECTURE.md#m02) | [astribot_s1_chassis_effort_drive_native](../../ws_robot/src/astribot_s1_chassis_effort_drive_native) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_chassis_effort_drive_native/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_s1_chassis_effort_drive_native/package.xml) |
| [M02 仿真和设备输入](MODULE_ARCHITECTURE.md#m02) | [livox_ros_driver2](../../ws_robot/src/livox_ros_driver2) | [CMakeLists.txt](../../ws_robot/src/livox_ros_driver2/CMakeLists.txt)、[README.md](../../ws_robot/src/livox_ros_driver2/README.md) |
| [M02 仿真和设备输入](MODULE_ARCHITECTURE.md#m02) | [aws_robomaker_small_warehouse_world](../../ws_robot/src/aws-robomaker-small-warehouse-world) | [CMakeLists.txt](../../ws_robot/src/aws-robomaker-small-warehouse-world/CMakeLists.txt)、[README.md](../../ws_robot/src/aws-robomaker-small-warehouse-world/README.md) |
| [M03 感知和时序](MODULE_ARCHITECTURE.md#m03) | [astribot_s1_perception](../../ws_robot/src/astribot_s1_perception) | [setup.py](../../ws_robot/src/astribot_s1_perception/setup.py)、[package.xml](../../ws_robot/src/astribot_s1_perception/package.xml) |
| [M03 感知和时序](MODULE_ARCHITECTURE.md#m03) | [astribot_s1_perception_components](../../ws_robot/src/astribot_s1_perception_components) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_perception_components/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_perception_components/README.md) |
| [M03 感知和时序](MODULE_ARCHITECTURE.md#m03) | [astribot_s1_perception_native](../../ws_robot/src/astribot_s1_perception_native) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_perception_native/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_perception_native/README.md) |
| [M03 感知和时序](MODULE_ARCHITECTURE.md#m03) | [astribot_sensor_sync](../../ws_robot/src/astribot_sensor_sync) | [CMakeLists.txt](../../ws_robot/src/astribot_sensor_sync/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_sensor_sync/README.md) |
| [M04 SLAM与地图](MODULE_ARCHITECTURE.md#m04) | [astribot_s1_slam](../../ws_robot/src/astribot_s1_slam) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_slam/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_slam/README.md) |
| [M04 SLAM与地图](MODULE_ARCHITECTURE.md#m04) | [astribot_s1_mapping](../../ws_robot/src/astribot_s1_mapping) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_mapping/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_mapping/README.md) |
| [M05 地图资产与禁行区](MODULE_ARCHITECTURE.md#m05) | [astribot_map_manager](../../ws_robot/src/astribot_map_manager) | [CMakeLists.txt](../../ws_robot/src/astribot_map_manager/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_map_manager/README.md) |
| [M05 地图资产与禁行区](MODULE_ARCHITECTURE.md#m05) | [astribot_navigation_zones](../../ws_robot/src/astribot_navigation_zones) | [CMakeLists.txt](../../ws_robot/src/astribot_navigation_zones/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_navigation_zones/package.xml) |
| [M06 导航任务与探索](MODULE_ARCHITECTURE.md#m06) | [astribot_s1_task_arbiter_native](../../ws_robot/src/astribot_s1_task_arbiter_native) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_task_arbiter_native/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_task_arbiter_native/README.md) |
| [M06 导航任务与探索](MODULE_ARCHITECTURE.md#m06) | [astribot_route_executor](../../ws_robot/src/astribot_route_executor) | [CMakeLists.txt](../../ws_robot/src/astribot_route_executor/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_route_executor/README.md) |
| [M06 导航任务与探索](MODULE_ARCHITECTURE.md#m06) | [astribot_s1_exploration](../../ws_robot/src/astribot_s1_exploration) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_exploration/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_exploration/README.md) |
| [M06 导航任务与探索](MODULE_ARCHITECTURE.md#m06) | [astribot_autonomy_core](../../ws_robot/src/astribot_autonomy_core) | [CMakeLists.txt](../../ws_robot/src/astribot_autonomy_core/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_autonomy_core/README.md) |
| [M07 策略与上肢限速](MODULE_ARCHITECTURE.md#m07) | [astribot_s1_navigation_policy](../../ws_robot/src/astribot_s1_navigation_policy) | [setup.py](../../ws_robot/src/astribot_s1_navigation_policy/setup.py)、[README.md](../../ws_robot/src/astribot_s1_navigation_policy/README.md) |
| [M07 策略与上肢限速](MODULE_ARCHITECTURE.md#m07) | [astribot_s1_navigation_policy_native](../../ws_robot/src/astribot_s1_navigation_policy_native) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_navigation_policy_native/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_navigation_policy_native/README.md) |
| [M07 策略与上肢限速](MODULE_ARCHITECTURE.md#m07) | [astribot_s1_social_navigation](../../ws_robot/src/astribot_s1_social_navigation) | [setup.py](../../ws_robot/src/astribot_s1_social_navigation/setup.py)、[README.md](../../ws_robot/src/astribot_s1_social_navigation/README.md) |
| [M07 策略与上肢限速](MODULE_ARCHITECTURE.md#m07) | [astribot_s1_dynamics_coupling](../../ws_robot/src/astribot_s1_dynamics_coupling) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_dynamics_coupling/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_s1_dynamics_coupling/package.xml) |
| [M08 规划跟踪与恢复](MODULE_ARCHITECTURE.md#m08) | [astribot_s1_navigation](../../ws_robot/src/astribot_s1_navigation) | [setup.py](../../ws_robot/src/astribot_s1_navigation/setup.py)、[package.xml](../../ws_robot/src/astribot_s1_navigation/package.xml) |
| [M08 规划跟踪与恢复](MODULE_ARCHITECTURE.md#m08) | [astribot_s1_path_tracking](../../ws_robot/src/astribot_s1_path_tracking) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_path_tracking/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_path_tracking/README.md) |
| [M08 规划跟踪与恢复](MODULE_ARCHITECTURE.md#m08) | [astribot_s1_navigation_recovery](../../ws_robot/src/astribot_s1_navigation_recovery) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_navigation_recovery/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_navigation_recovery/README.md) |
| [M09 附件与整机几何](MODULE_ARCHITECTURE.md#m09) | [astribot_s1_payload_state](../../ws_robot/src/astribot_s1_payload_state) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_payload_state/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_payload_state/README.md) |
| [M09 附件与整机几何](MODULE_ARCHITECTURE.md#m09) | [astribot_s1_robot_geometry](../../ws_robot/src/astribot_s1_robot_geometry) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_robot_geometry/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_robot_geometry/README.md) |
| [M10 操作规划](MODULE_ARCHITECTURE.md#m10) | [astribot_s1_manipulation](../../ws_robot/src/astribot_s1_manipulation) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_manipulation/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_manipulation/README.md) |
| [M10 操作规划](MODULE_ARCHITECTURE.md#m10) | [astribot_s1_moveit_config](../../ws_robot/src/astribot_s1_moveit_config) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_moveit_config/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_s1_moveit_config/package.xml) |
| [M10 操作规划](MODULE_ARCHITECTURE.md#m10) | [astribot_s1_transport_mtc](../../ws_robot/src/astribot_s1_transport_mtc) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_transport_mtc/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_transport_mtc/README.md) |
| [M11 抓取及物体感知](MODULE_ARCHITECTURE.md#m11) | [astribot_s1_manipulation_perception](../../ws_robot/src/astribot_s1_manipulation_perception) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_manipulation_perception/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_manipulation_perception/README.md) |
| [M11 抓取及物体感知](MODULE_ARCHITECTURE.md#m11) | [astribot_graspnet_runtime](../../ws_robot/src/astribot_graspnet_runtime) | [CMakeLists.txt](../../ws_robot/src/astribot_graspnet_runtime/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_graspnet_runtime/README.md) |
| [M11 抓取及物体感知](MODULE_ARCHITECTURE.md#m11) | [astribot_object_pose_core](../../ws_robot/src/astribot_object_pose_core) | [CMakeLists.txt](../../ws_robot/src/astribot_object_pose_core/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_object_pose_core/README.md) |
| [M12 搬运和资源](MODULE_ARCHITECTURE.md#m12) | [astribot_s1_transport](../../ws_robot/src/astribot_s1_transport) | [setup.py](../../ws_robot/src/astribot_s1_transport/setup.py)、[README.md](../../ws_robot/src/astribot_s1_transport/README.md) |
| [M12 搬运和资源](MODULE_ARCHITECTURE.md#m12) | [astribot_s1_transport_native](../../ws_robot/src/astribot_s1_transport_native) | [CMakeLists.txt](../../ws_robot/src/astribot_s1_transport_native/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_s1_transport_native/README.md) |
| [M13 SDK桥接](MODULE_ARCHITECTURE.md#m13) | [astribot_trajectory_bridge](../../ws_robot/src/astribot_trajectory_bridge) | [setup.py](../../ws_robot/src/astribot_trajectory_bridge/setup.py)、[README.md](../../ws_robot/src/astribot_trajectory_bridge/README.md) |
| [M13 SDK桥接](MODULE_ARCHITECTURE.md#m13) | [astribot_trajectory_bridge_native](../../ws_robot/src/astribot_trajectory_bridge_native) | [CMakeLists.txt](../../ws_robot/src/astribot_trajectory_bridge_native/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_trajectory_bridge_native/README.md) |
| [M14 操作台](MODULE_ARCHITECTURE.md#m14) | [astribot_operator_backend](../../ws_robot/src/astribot_operator_backend) | [CMakeLists.txt](../../ws_robot/src/astribot_operator_backend/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_operator_backend/README.md) |
| [M14 操作台](MODULE_ARCHITECTURE.md#m14) | [astribot_operator_station](../../ws_robot/src/astribot_operator_station) | [CMakeLists.txt](../../ws_robot/src/astribot_operator_station/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_operator_station/README.md) |
| [M15 日志验证与依赖](MODULE_ARCHITECTURE.md#m15) | [astribot_logging](../../ws_robot/src/astribot_logging) | [CMakeLists.txt](../../ws_robot/src/astribot_logging/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_logging/package.xml) |
| [M15 日志验证与依赖](MODULE_ARCHITECTURE.md#m15) | [astribot_sim_validation](../../ws_robot/src/astribot_sim_validation) | [CMakeLists.txt](../../ws_robot/src/astribot_sim_validation/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_sim_validation/README.md) |
| [M15 日志验证与依赖](MODULE_ARCHITECTURE.md#m15) | [astribot_eigen_vendor](../../ws_robot/src/astribot_eigen_vendor) | [CMakeLists.txt](../../ws_robot/src/astribot_eigen_vendor/CMakeLists.txt)、[README.md](../../ws_robot/src/astribot_eigen_vendor/README.md) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_navigation_msgs](../../ws_robot/src/astribot_navigation_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_navigation_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_navigation_msgs/package.xml) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_bridge_msgs](../../ws_robot/src/astribot_bridge_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_bridge_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_bridge_msgs/package.xml) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_slam_msgs](../../ws_robot/src/astribot_slam_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_slam_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_slam_msgs/package.xml) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_perception_msgs](../../ws_robot/src/astribot_perception_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_perception_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_perception_msgs/package.xml) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_payload_msgs](../../ws_robot/src/astribot_payload_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_payload_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_payload_msgs/package.xml) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_transport_msgs](../../ws_robot/src/astribot_transport_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_transport_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_transport_msgs/package.xml) |
| [M16 领域接口](MODULE_ARCHITECTURE.md#m16) | [astribot_operator_msgs](../../ws_robot/src/astribot_operator_msgs) | [CMakeLists.txt](../../ws_robot/src/astribot_operator_msgs/CMakeLists.txt)、[package.xml](../../ws_robot/src/astribot_operator_msgs/package.xml) |

## 工作区之外与非包目录

| 目录 | 职责与边界 |
|---|---|
| [astribot_sdk](../../astribot_sdk) | 厂家 SDK 接口及原生依赖；真机使用对应 ARM64 安装，不是 ws_robot 自研节点包 |
| [astribot_msgs](../../astribot_msgs) | 厂家消息；内含 share 安装布局，不重复计包 |
| [astribot_config](../../astribot_config) | 厂家模型/设备/运动配置来源 |
| [tools](../../tools) | 启动、部署、采集、验证、模型准备；不是全部都可当生产入口 |
| [config](../../config)、[maps](../../maps)、[routes](../../routes) | 应用配置、地图、路线资产；需绑定会话和版本 |
| [ws_robot/third_party](../../ws_robot/third_party)、[third_party](../../third_party) | 数值及厂家依赖；按实际构建清单核对架构和版本 |
| [examples](../../examples)、[chassis_benchmark](../../chassis_benchmark) | SDK 示例与专项测量，不等于经整机仲裁的正式业务入口 |
| build / install / log（根目录及 ws_robot） | 构建/安装/日志，不是源码主归属；不得覆盖正在使用的库 |
| runs / docs/evidence | 仿真、实验与证据；runs 的本地绝对路径可能不随 Git/部署复制 |

第三方子模块本轮已有修改状态，本次未修改、重置或更新它们。没有把包描述中的历史“已实现”直接作为能力证明；各模块以当前源码和对应验收记录为准。
