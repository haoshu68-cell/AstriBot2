# Astribot S1 轮式双臂机器人 × AWS 仓储仿真世界 — 操作指引

本工作空间把仓库自带的 Astribot S1（**X 型布局全向轮(omni)** 底盘 + 4自由度升降躯干 + 2自由度头部 +
左右各7自由度机械臂 + 双 Livox Mid-360 激光雷达）接入 `aws-robomaker-small-warehouse-world`
（ros2 分支）仓储场景，运行在 ROS2 Humble + Gazebo(Ignition/Gz Sim) 上，集成了 SLAM Toolbox
建图/纯定位能力，以及 Nav2 自主导航（发目标点、自动规划避障路径）。

> ⚠️ **底盘构型的命名已于 2026-08-21 纠正。** 四个轮关节的 rpy 是 ±2.35619 / ±0.785398
> （即 ±135° / ±45°），轮轴指向车体对角线 —— 这是 **X 型布局全向轮(omni)**，
> **不是**"轮轴沿 ±y、辊子 45°"的麦克纳姆轮。权威出处：
> [`src/astribot_s1_chassis_effort_drive/config/omni_effort_drive_params.yaml`](src/astribot_s1_chassis_effort_drive/config/omni_effort_drive_params.yaml)
> 文件头 + `astribot_s1_torso_wheel.xacro` 的轮关节 origin。
> 本文件与 `README_PERCEPTION.md` 第 10 节已改，但**仓库里仍有其它地方留着
> "麦克纳姆轮"字样**（如上面第 19 行那条链接的显示文字），指的是同一台机器人，
> 不要当成两种底盘。按麦轮的运动学去推导会得到错的逆解矩阵。

设计细节与决策依据见实施方案：`/home/yjh/.claude/plans/synthetic-snuggling-pizza.md`；
故障排查见 [`src/astribot_s1_gazebo_bringup/README_TROUBLESHOOTING.md`](src/astribot_s1_gazebo_bringup/README_TROUBLESHOOTING.md)（仿真/底盘/机械臂）、
[`src/astribot_s1_perception/README_PERCEPTION.md`](src/astribot_s1_perception/README_PERCEPTION.md)（双雷达感知/SLAM/硬件分支/自主巡游/麦克纳姆轮bug修复）
和 [`src/astribot_s1_navigation/README_NAVIGATION.md`](src/astribot_s1_navigation/README_NAVIGATION.md)（Nav2导航集成）。

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

## 7.1 自主巡游建图（"轮子转、车不走/一动就倒"bug 已修复）

上面的建图命令默认会顺带启动一个反应式自主巡游节点（`autonomous_patrol_node`），
让机器人边走边转、自动扩大 SLAM 覆盖范围，不需要人工遥操作：

```bash
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true autonomous_patrol:=true   # 默认就是true
```

只想手动遥操作、不想自主巡游节点抢着发 `/cmd_vel` 时，加 `autonomous_patrol:=false`。

底盘曾经存在"轮子转、身体不平移"以及"平移过程中身体会倾倒"两个真实 bug，
均已定位并修复（4 个独立根因：轮子小尺寸下各向异性摩擦失效、`MecanumDrive`/
`VelocityControl` 摩擦力冲突、双雷达自撞点被误判成障碍物、车体系/world系速度混用），
单实例实测连续巡游 22+ 分钟零次异常。完整排查过程、根因、修复代码位置见
[`src/astribot_s1_perception/README_PERCEPTION.md`](src/astribot_s1_perception/README_PERCEPTION.md#10-自主巡游节点autonomous_patrol_node与全向轮平移bug的完整修复记录)
第10节。

## 7.2 Nav2 自主导航（发目标点，自动规划路径避开货架）

```bash
# 建图 + 导航同时进行（推荐用 mppi 发挥全向底盘能力）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true controller_plugin:=mppi

# 预建图 + SLAM定位 + 导航
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=localization launch_gazebo:=true \
  map_file_name:=/绝对路径/my_map controller_plugin:=mppi
```

不跑 `nav2_map_server`/`amcl`（SLAM Toolbox 自己在两种模式下都会发布 `/map` +
`map->odom` TF），底盘用 gz-sim `VelocityControl` 插件按 world 系解释速度，接了
一个专门的 `cmd_vel_body_to_world_node` 做车体系→world系转换（否则会重现"原地
打转不挪窝"的旧问题）。两种模式、机械臂展开限速均已实测跑通，完整设计取舍/踩坑
记录见 [`src/astribot_s1_navigation/README_NAVIGATION.md`](src/astribot_s1_navigation/README_NAVIGATION.md)。

## 7.3 自主环境探索 + 建图（机器人自己选点，不用发目标）

§7.2 是"你发目标点、Nav2 去执行"。这一节是**机器人自己决定去哪** ——
拉起探索协调器，它跑前沿搜索选下一个观测点、校验路径、调 Nav2 过去、
到位后再选下一个，直到探完。

```bash
cd <仓库根>/ws_robot
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  exploration:=true controller_plugin:=mppi
```

**只有这两个参数是非默认的**，别的都不用写：

| 参数 | 默认 | 为什么要显式给 |
|---|---|---|
| `exploration` | `false` | 默认关，避免协调器和你手动发的目标抢 |
| `controller_plugin` | `rpp` | `rpp` 是任务书要求的非全向退化行为；`mppi` 才能发挥全向底盘 |

其余 `env:=sim`、`mode:=mapping`、`launch_gazebo:=true`、`use_rviz:=true`、
`scan_source:=slice_scan`、`headless:=false` 都已经是默认值；
`map_source`/`localization` 由 `src/astribot_s1_perception/config/map_source.yaml`
给成 `sim_slam`/`slam`。想防配置漂移就全写出来：

```bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true \
  exploration:=true map_source:=sim_slam localization:=slam \
  controller_plugin:=mppi scan_source:=slice_scan \
  headless:=false use_rviz:=true
```

这条会一起拉起：Gazebo(仓储世界) + RViz + 多层切片感知 + slam_toolbox 建图 +
Nav2 + 探索协调器。起来后机器人自主推进，**不需要你发任何目标**。

### 起来没起来：一行就能看

```bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1     # ← 必须，见下
ros2 topic echo /exploration/state --once --field data
```

约 40~60s 后应看到 `state=NAVIGATING`。这一行里有全部诊断字段
（`bootstrap=a/b`、`path_pts`、`replan_policy`、三个失败计数器…），
逐字段读法见手册 [§2.7](../docs/technical_operations_manual.md)。

### 四个会让你白等的坑（都实测踩过）

1. **`headless:=true` 不要用。** 它不是"省资源的选项" —— `gz_ros2_control`
   在加载期取 `robot_description` 时死锁，物理**一步都不走**。
   最快判据是数日志行数：卡住时停在 4 行左右，正常 250+。

2. **查询必须带 `ROS_DOMAIN_ID=25`。** launch 侧固定 25（`warehouse_sim.launch.py`
   的默认值会覆盖你 shell 里的 export）。不带就是"节点全都 Node not found"，
   看着像系统没起来。确认实际值：
   ```bash
   tr '\0' '\n' < /proc/$(pgrep -f exploration_coordinator_node | head -1)/environ | grep DOMAIN
   ```

3. **`ros2 topic list` 只回 2 个话题时，先清 daemon，不是系统没起来。**
   `ros2` 守护进程按 domain 缓存节点图，换过 domain 或上一轮被强杀之后会一直
   返回一份几乎空的图。实测同一时刻：带 daemon **2** 个 → `--no-daemon` 4 个 →
   `ros2 daemon stop` 之后 **80** 个。
   ```bash
   ros2 daemon stop      # 下次调用自动重启
   ```

4. **启动前确认上一个实例真的停了。** 两个实例会抢同一个 Gazebo 和
   `/joint_states`，而**两边都不报错**。清场方法见
   [手册 §6.7](../docs/technical_operations_manual.md) —— 一句话版：
   **在 launch 的终端按 Ctrl-C**；要用命令清就只用 `pkill -x <comm 名>`，
   而且**必须先杀 `ros2`（launch 父进程）再杀子进程** ——
   顺序颠倒的话父进程会把带 respawn 的节点重新拉起来，和新起的一套凑成两份。
   **绝不要 `pkill -f 'ros2 launch|…'`**（它会连发起它的 shell 一起杀，
   实测：`pgrep -x ruby` 数到 3 个而 `pgrep -f ruby` 数到 4 个，
   多的那个就是命令行里提到了 ruby 的 shell 自己）。

   启动后**先确认没有重复实例**，任何一项 >1 就是有两套栈在打架：
   ```bash
   for c in ros2 ruby controller_serv exploration_coo omni_effort_dri; do
     printf '%-18s %s\n' "$c" "$(pgrep -c -x $c)"
   done
   ```

### 存地图

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/绝对路径/my_map'}"
```

存出来是 `.data` + `.posegraph`（**位姿图**），不是 `.pgm` + `.yaml`。
加载回去用 §7.2 的 `mode:=localization map_file_name:=/绝对路径/my_map`（不带扩展名）。

### 人工干预

```bash
ros2 service call /exploration_coordinator_node/pause  std_srvs/srv/Trigger
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger
```

`state=PAUSED` 且 `auto_resume` 已用尽时，先看 `nav_fail`：
若为 `0/3`（从没导航失败），那是探索**自然结束**（地图里只剩零散前沿格），不是被困。

状态机、双层校验口径、跟踪期换路径判据、冷启动自举的完整设计见
[`src/astribot_s1_autonomy/README.md`](src/astribot_s1_autonomy/README.md)；
排查手册见 [`docs/technical_operations_manual.md`](../docs/technical_operations_manual.md) §8.5。

## 8. 关于 git

本次已把 `astribot_sdk_ros2` 初始化为本地 git 仓库（`git init`，仅本地提交，未配置/推送任何远程），
`aws-robomaker-small-warehouse-world`、`livox_ros_driver2` 均以 submodule 形式纳入。
`.gitignore` 已有的 `*build` / `install` / `log` 规则会自动排除 `ws_robot/{build,install,log}`
等 colcon 产物目录。
