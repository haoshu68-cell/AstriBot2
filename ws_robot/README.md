# Astribot S1 ROS2 工作空间

本工作空间包含轮式双臂机器人的仿真、感知、探索、导航、机械臂规划和 SDK 桥接。当前底盘为 X 型布局全向轮，基座坐标系为 `astribot_torso_base`，底盘接收车体系速度。

当前版本的职责、启动步骤、运行参数和能力边界见 [架构、仿真及真机手册](../docs/manuals/README.md)。

## 目录与模块

| 目录 | 内容 |
|---|---|
| `src/` | 19 个 ROS 包的源码；含 17 个项目包、2 个第三方子模块 |
| `build/` | colcon 构建缓存，不代表当前仍存在的源码模块 |
| `install/` | 当前运行安装空间；增量构建可能遗留旧文件 |
| `log/` | 构建和测试日志 |
| `maps/` | 历史建图产物；实际地图以启动参数指定路径为准 |

完整状态、依赖与退役项见 [ws_robot 模块清单](../docs/WS_ROBOT_MODULE_AUDIT_20260915.md)。

| 分组 | 包 |
|---|---|
| 公共接口与日志 | `astribot_bridge_msgs`、`astribot_navigation_msgs`、`astribot_logging` |
| 模型与仿真 | `astribot_s1_description`、`astribot_s1_gazebo_bringup`、`astribot_s1_chassis_effort_drive` |
| 感知与探索 | `astribot_autonomy_core`、`astribot_s1_perception`、`astribot_s1_perception_components`、`astribot_s1_exploration` |
| 导航与控制 | `astribot_s1_navigation`、`astribot_s1_navigation_policy`、`astribot_s1_path_tracking`、`astribot_s1_dynamics_coupling` |
| 双臂与真机桥接 | `astribot_s1_manipulation`、`astribot_s1_moveit_config`、`astribot_trajectory_bridge` |
| 第三方 | `aws_robomaker_small_warehouse_world`、`livox_ros_driver2` |

感知两个包并不重复：Python 包负责统一 SLAM 的启动、会话操作与 TF 分解；Voxel 内部完成标准点云融合与自滤，C++ components 负责多高度切片。旧 `astribot_s1_autonomy` 兼容包已删除；组合调试入口迁入探索包，见 [迁移说明](../docs/AUTONOMY_PACKAGE_RETIREMENT_20260917.md)。

## 构建

以下命令从仓库根目录执行，在未加载厂商 ROS 环境的新终端构建。不要在机器人运行期间删除或替换其安装空间。

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
bash ws_robot/src/astribot_s1_perception/scripts/prepare_livox_driver2.sh
bash tools/robot/build_slam_dependencies.sh
cd ws_robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DGTSAM_DIR="$PWD/deps/gtsam/lib/cmake/GTSAM"
```

真机依赖和部署流程见 [真机操作手册](../docs/manuals/HARDWARE_OPERATIONS.md)。真机构建脚本跳过 Gazebo 世界和仿真 bringup，构建项目内 Livox 驱动、SLAM 和统一 Eigen 依赖。新 SLAM 包为 `astribot_s1_slam`、`astribot_s1_mapping`、`astribot_slam_msgs`；第三方 GTSAM 源码位于 `third_party/gtsam`，Eigen 唯一源码由 `astribot_eigen_vendor` 管理。

## 启动

从仓库根目录启动受监督仿真：

```bash
bash tools/launch_sim_stack.sh --mode baseline
```

该入口按顺序检查 Gazebo 数据、TF 和 Nav2 生命周期，保存会话日志；不自动发送目标。`baseline` 使用固定地图与仿真真值定位，默认策略阶段为 `off`。策略验证状态以 [窄通道手册](../docs/NARROW_PASSAGE_REBUILD_20260914.md) 为准。

建图、探索、定位和底层 launch 参数见 [导航说明](src/astribot_s1_navigation/README_NAVIGATION.md)。仅查看模型可使用：

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
ros2 launch astribot_s1_gazebo_bringup rviz.launch.py
```

感知/探索入口位于 `astribot_s1_perception_components` 和 `astribot_s1_exploration`；组合入口为 `ros2 launch astribot_s1_exploration autonomy_bringup.launch.py`。感知包的独立 `perception_slam_bringup.launch.py` 还保留反应式巡游功能；统一 Nav2 总入口明确关闭它，避免另一条速度源参与导航。

## 数据与控制归属

```text
双雷达/IMU → Voxel-SLAM（进程内自滤、同步、估计）→ 栅格与一次切片
人工目标/探索任务 → 任务仲裁 → 导航策略与规划 → 路径跟踪及到位精调
    → 速度平滑与机械臂动态限速 → 独立最终防护（启用策略时）
    → 仿真力矩驱动 / 真机 SDK 桥接
```

路径跟踪保留 ThreePhase 的起始对齐、MPPI/RPP 跟踪与接近控制，ArrivalController 完成末端精调。标准到位上限为 3 cm / 1.5°；精度档以实际加载的目标检查器参数为准。视觉、SLAM 和 Mark 接口属于在用扩展契约，不按“当前仿真未启用”删除。

## 文档入口

- [整体架构](../docs/ARCHITECTURE.md)
- [模块状态与残留清理](../docs/WS_ROBOT_MODULE_AUDIT_20260915.md)
- [感知与 SLAM](src/astribot_s1_perception/README_PERCEPTION.md)
- [导航与跟踪](src/astribot_s1_navigation/README_NAVIGATION.md)
- [双臂规划](src/astribot_s1_manipulation/README.md)
- [桥接](src/astribot_trajectory_bridge/README.md)
- [统一日志](../docs/LOGGING.md)
- [窄通道设计及当前验证阶段](../docs/NARROW_PASSAGE_REBUILD_20260914.md)

`build/`、`install/`、`log/` 和生成地图均属于本地工作产物。清理旧安装文件应依据当前安装清单及依赖核对；保留有效兼容入口、地图和验证证据，不以目录名称判断模块是否废弃。
