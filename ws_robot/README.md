# Astribot S1 轮式双臂机器人 × AWS 仓储仿真世界 — 操作指引

本工作空间把仓库自带的 Astribot S1（麦克纳姆全向轮底盘 + 4自由度升降躯干 + 2自由度头部 +
左右各7自由度机械臂 + 双 Livox Mid-360 激光雷达）接入 `aws-robomaker-small-warehouse-world`
（ros2 分支）仓储场景，运行在 ROS2 Humble + Gazebo(Ignition/Gz Sim) 上，并集成了 SLAM Toolbox
建图/纯定位能力。

设计细节与决策依据见实施方案：`/home/yjh/.claude/plans/synthetic-snuggling-pizza.md`；
故障排查见 [`src/astribot_s1_gazebo_bringup/README_TROUBLESHOOTING.md`](src/astribot_s1_gazebo_bringup/README_TROUBLESHOOTING.md)（仿真/底盘/机械臂）
和 [`src/astribot_s1_perception/README_PERCEPTION.md`](src/astribot_s1_perception/README_PERCEPTION.md)（双雷达感知/SLAM/硬件分支）。

## 1. 依赖安装

系统需要 Ubuntu 22.04 + ROS2 Humble（`astribot_sdk_ros2/install.sh` 已装好基础环境）。
仿真额外需要以下 apt 包（本机实测 `ros-humble-ros-gz-sim` 依赖 `libignition-gazebo6`，
即 **Ignition Gazebo Fortress**，是 ROS2 Humble 官方源下的默认新版 Gazebo 搭配）：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  liburdfdom-tools \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-tf2-sensor-msgs \
  ros-humble-message-filters
```

双 Livox Mid-360 实体硬件分支（`livox_ros_driver2`）额外的依赖安装、源码编译步骤见
[`src/astribot_s1_perception/README_PERCEPTION.md`](src/astribot_s1_perception/README_PERCEPTION.md#1-依赖安装)，
只用仿真的话可以跳过。

## 2. 工作空间结构

```
astribot_sdk_ros2/
└── ws_robot/
    └── src/
        ├── aws-robomaker-small-warehouse-world/   # git submodule（ros2分支，官方仓库原样）
        ├── livox_ros_driver2/                      # git submodule（硬件分支用，仿真不需要跑它）
        ├── astribot_s1_description/                # 机器人描述（xacro/urdf + meshes + rviz）
        ├── astribot_s1_gazebo_bringup/              # 仿真bringup（launch + 控制器yaml + 环境脚本）
        └── astribot_s1_perception/                 # 双雷达点云感知 + SLAM Toolbox 建图/定位
```

`aws-robomaker-small-warehouse-world`、`livox_ros_driver2` 都是以 **git submodule** 形式接入的
（而不是直接拷贝源码），首次拉取本仓库或子模块内容为空时执行：

```bash
cd astribot_sdk_ros2
git submodule update --init --recursive
```

## 3. 编译

```bash
cd astribot_sdk_ros2/ws_robot
rosdep update
rosdep install --from-paths src --ignore-src -r -y   # 按 package.xml 声明自动装齐依赖

# -DROS_EDITION=ROS2 -DDISTRO_ROS=humble 是 livox_ros_driver2 的 CMakeLists 需要的参数
# （用来区分 ROS1/ROS2、区分发行版），对其它包无影响（只会多两条无害的 CMake 警告）。
# 只用仿真、不需要硬件分支的话，见 astribot_s1_perception/README_PERCEPTION.md 里
# "跳过硬件分支单独编译"的命令。
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

## 4. 运行仿真

```bash
# 默认：small_warehouse 世界 + astribot_s1（挂雷达+相机）+ RViz2 一起打开
ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py

# 常用可调参数示例：
ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py \
  world_name:=no_roof_small_warehouse \
  robot_name:=astribot_s1 \
  spawn_x:=1.0 spawn_y:=-2.0 spawn_z:=0.10 spawn_yaw:=1.57 \
  use_lidar:=true use_camera:=true \
  use_rviz:=false
```

验证仿真是否正常（另开终端，记得先 `source install/setup.bash`）：

```bash
ros2 control list_controllers        # 5 个控制器应全部 active
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.1}, angular: {z: 0.0}}" --once   # 全向底盘应同时前移+侧移
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom astribot_torso_base
```

给左臂发一条测试轨迹：

```bash
ros2 topic pub /arm_left_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [astribot_arm_left_joint_1, astribot_arm_left_joint_2, astribot_arm_left_joint_3,
astribot_arm_left_joint_4, astribot_arm_left_joint_5, astribot_arm_left_joint_6, astribot_arm_left_joint_7],
points: [{positions: [0.3, -0.5, 0.0, 1.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}" --once
```

## 5. 只看模型/不开仿真（离线核对 URDF、TF）

```bash
ros2 launch astribot_s1_gazebo_bringup rviz.launch.py
```
会额外打开 `joint_state_publisher_gui`，可以拖滑条手动摆关节，检查每一节是否按预期转动。

## 6. 常用参数怎么改

| 需求 | 怎么改 |
|---|---|
| 出生位置/朝向 | launch 参数 `spawn_x` / `spawn_y` / `spawn_z` / `spawn_yaw` |
| 出生高度下限 | 由轮子碰撞体半径决定，见 `astribot_s1_description/config/collision_overrides.yaml` 里的 `spawn_height` 小节；换了轮子尺寸要重新算 |
| 换仓储世界 | launch 参数 `world_name`（`small_warehouse` / `no_roof_small_warehouse`） |
| 关闭雷达/相机 | launch 参数 `use_lidar:=false`（双 Mid-360 一起关） / `use_camera:=false` |
| 双 Mid-360 安装位姿 | `astribot_s1_description/urdf/astribot_s1.xacro` 里的 `livox_left_xyz`/`livox_left_rpy`/`livox_right_xyz`/`livox_right_rpy`（默认左前+右后对角安装，见 `astribot_s1_perception/config/livox_extrinsics.yaml` 的文档化记录） |
| 机器人重名 | launch 参数 `robot_name`（同时决定 Gazebo 实体名与传感器话题前缀 `/model/<robot_name>/...`） |
| 机械臂关节限位/力矩/速度 | `astribot_s1_description/urdf/astribot_s1_arm.xacro` 里对应 `<limit .../>`（已与 `astribot_arm_left/right.yaml` 里的原始 URDF 数值保持一致，改动前建议先确认是否也要同步改 `astribot_config` 里的原始文件，本方案未改动原始文件） |
| 控制器增益/更新频率 | `astribot_s1_gazebo_bringup/config/astribot_s1_controllers.yaml` |

## 7. 双雷达感知 + SLAM 建图/定位

```bash
# 仿真 + 建图（顺带拉起仓储世界仿真）
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true
```

完整用法、建图/定位切换、地图保存加载、硬件分支联调清单见
[`src/astribot_s1_perception/README_PERCEPTION.md`](src/astribot_s1_perception/README_PERCEPTION.md)。

## 8. 关于 git

本次已把 `astribot_sdk_ros2` 初始化为本地 git 仓库（`git init`，仅本地提交，未配置/推送任何远程），
`aws-robomaker-small-warehouse-world`、`livox_ros_driver2` 均以 submodule 形式纳入。
`.gitignore` 已有的 `*build` / `install` / `log` 规则会自动排除 `ws_robot/{build,install,log}`
等 colcon 产物目录。
