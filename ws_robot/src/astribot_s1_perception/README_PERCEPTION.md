# astribot_s1_perception —— 双 Livox Mid-360 感知 + SLAM Toolbox 建图/定位

Astribot S1 轮式双臂机器人的点云感知与 2D SLAM 栈：双 Mid-360 点云预处理、时间同步融合、
2D 投影，SLAM Toolbox 建图/纯定位两种模式，区分仿真（Ignition Gazebo 近似仿真）与实体硬件
（livox_ros_driver2）两套启动分支。设计依据见实施方案
`/home/yjh/.claude/plans/synthetic-snuggling-pizza.md`。

## 1. 依赖安装

### 1.1 仿真分支（apt 直接装，本机已验证可用）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-tf2-sensor-msgs \
  ros-humble-message-filters \
  python3-numpy
```

### 1.2 硬件分支（livox_ros_driver2，无 ROS2 apt 包，必须源码编译）

livox_ros_driver2 依赖 Livox 官方的 **Livox-SDK2** C++ 库，需要单独编译安装（不属于
ROS2 工作空间，装到系统 `/usr/local` 下）：

```bash
# 1) 编译安装 Livox-SDK2（本仓库已实测走通这一步）
git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
cd /tmp/Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig

# 2) livox_ros_driver2 作为 submodule 已经在 ws_robot/src/livox_ros_driver2，
#    首次拉取/更新子模块：
cd <astribot_sdk_ros2>/ws_robot
git submodule update --init --recursive

# 3) livox_ros_driver2 官方仓库没有现成 package.xml/launch，需要跑一次准备脚本
#    （不要用官方自带的 build.sh——它会顺手 rm -rf ws_robot/build|install，
#    把本工作空间其它包已经编译好的产物全部删掉，细节见脚本内注释）：
bash src/astribot_s1_perception/scripts/prepare_livox_driver2.sh
```

## 2. 工作空间准备与编译

```bash
cd <astribot_sdk_ros2>/ws_robot
git submodule update --init --recursive   # 含 aws warehouse world + livox_ros_driver2
bash src/astribot_s1_perception/scripts/prepare_livox_driver2.sh   # 见上一节

# 注意：-DROS_EDITION=ROS2 -DDISTRO_ROS=humble 这两个 cmake 参数是
# livox_ros_driver2 的 CMakeLists 用来区分 ROS1/ROS2、区分发行版的，
# 对本仓库其它 ament_cmake 包无影响（只会多两条无害的 "unused variable" CMake 警告）。
# 本仓库已实测这一条命令能把 astribot_s1_description / astribot_s1_gazebo_bringup /
# aws_robomaker_small_warehouse_world / livox_ros_driver2 / astribot_s1_perception
# 五个包一起编译通过。
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

如果你不需要硬件分支（只用仿真），可以跳过 1.2/`prepare_livox_driver2.sh`，
只编译仿真相关的包：

```bash
colcon build --symlink-install --packages-up-to astribot_s1_perception \
  --packages-skip livox_ros_driver2
```

## 3. 新增包目录树

```
astribot_s1_perception/
├── package.xml / setup.py / setup.cfg / resource/
├── astribot_s1_perception/
│   ├── livox_preprocess_node.py   直通/半径滤波/地面剔除，左右雷达各一份实例
│   └── livox_fusion_node.py       时间同步 + tf2转公共系 + 拼接
├── launch/
│   ├── sim_perception.launch.py       仿真/硬件共用的感知节点图（use_sim_time区分）
│   ├── hardware_perception.launch.py  硬件分支入口，直接 include 上面那个文件
│   ├── hardware_livox.launch.py       硬件分支：robot_state_publisher + 双livox_ros_driver2
│   ├── slam_mapping.launch.py         SLAM Toolbox 建图模式
│   ├── slam_localization.launch.py    SLAM Toolbox 纯定位模式
│   └── perception_slam_bringup.launch.py  【顶层入口】env:=sim|hardware, mode:=mapping|localization
├── config/
│   ├── pointcloud_filter_params.yaml
│   ├── livox_fusion_params.yaml
│   ├── pointcloud_to_laserscan_params.yaml
│   ├── livox_extrinsics.yaml           左右雷达安装位姿文档化记录（权威数据源是xacro）
│   ├── mapper_params_online_async.yaml
│   ├── mapper_params_localization.yaml
│   └── MID360_config_left.json / MID360_config_right.json   硬件分支driver配置(IP占位)
├── rviz/perception_slam_view.rviz
├── scripts/prepare_livox_driver2.sh
└── maps/   建图后 /slam_toolbox/serialize_map 保存的地图放这里（.gitkeep占位，不提交具体地图）
```

同时改动了两个既有包：
- `astribot_s1_description/urdf/astribot_s1_sensors.xacro`、`astribot_s1.xacro`：
  原来的单个通用 2D 雷达换成两个 `astribot_s1_livox_mid360` 宏实例
  （`livox_mid360_left` 左前、`livox_mid360_right` 右后，见下方安装位姿说明）。
- `astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py`：
  bridge 配置从桥接单路 LaserScan 改成桥接双路 PointCloud2；另外补了两处联调时才发现的
  仿真专属修复，见下面"仿真踩坑记录"一节。

## 4. 双 Mid-360 安装位姿

**左前 + 右后**对角安装在 `astribot_torso_base` 上（用户指定，对角布局扩大合并视场、
减少互相遮挡的盲区）：

| | 挂载位置 | xyz (m) | rpy (rad) |
|---|---|---|---|
| `livox_mid360_left` | 左前 | 0.28, 0.18, 0.10 | 0, 0, 0.4 |
| `livox_mid360_right` | 右后 | -0.28, -0.18, 0.10 | 0, 0, 2.74 |

权威数据源是 `astribot_s1_description/urdf/astribot_s1.xacro` 里的 4 个 xacro:arg
（`livox_left_xyz`/`livox_left_rpy`/`livox_right_xyz`/`livox_right_rpy`），
`config/livox_extrinsics.yaml` 只是同一份数值的文档化记录，改了安装位置请两边同步。

## 5. 一键启动命令

```bash
# 仿真 + 建图（顺带拉起仓储世界仿真）
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true

# 建好图之后保存地图（basename不带扩展名，会生成 <name>.data + <name>.posegraph）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/绝对路径/my_map'}"

# 仿真 + 纯定位（加载刚保存的地图）
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=sim mode:=localization launch_gazebo:=true \
  map_file_name:=/绝对路径/my_map

# 实体机器人 + 建图（需要先完成第1.2节的硬件分支依赖安装）
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=hardware mode:=mapping

# 只看点云/建图效果，不需要重新起一遍Gazebo（假设仿真已经在跑）
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=false
```

## 6. 已实测验证的结果（不是纸面设计）

仿真分支全链路本仓库已经实际跑通并逐项核对：
- `ros2 topic hz /livox/lidar_left /livox/lidar_right`：稳定 ~9.5-10Hz；
- `ros2 topic hz /livox/left/cloud_filtered /livox/right/cloud_filtered`：预处理节点正常出数据；
- `ros2 topic hz /livox/fused_points`：融合节点正常出数据；
- `ros2 topic hz /scan`：投影节点正常出数据；
- `ros2 topic echo /map --once`：SLAM Toolbox 建图模式正常产出栅格地图并持续扩大；
- `ros2 run tf2_ros tf2_echo map odom` / `tf2_echo odom astribot_torso_base`：TF 链路完整不断裂；
- 用 `/slam_toolbox/serialize_map` 保存地图后，切到 `mode:=localization` 重新启动，
  日志里能看到 "Load From File ... .posegraph" / "Finished serializing Mapper/Dataset"，
  `/map` 尺寸和刚保存的一致，`tf2_echo map astribot_torso_base` 有输出——纯定位流程走通。
- 硬件分支：Livox-SDK2 已实际 `sudo make install` 成功；`livox_ros_driver2` 已实际
  `colcon build` 编译通过（`ros2 pkg executables livox_ros_driver2` 确认可执行文件名是
  `livox_ros_driver2_node`，和 `hardware_livox.launch.py` 里写的一致）——**编译验证过**，
  但因为没有真实 Mid-360 硬件接入，无法验证真实联机取数据这一步，
  详见下面"硬件分支联调清单"。

## 7. 仿真踩坑记录（联调时实际遇到、已修复）

这些坑在开发过程中真实踩到过，写在这里避免以后重复趟坑：

1. **Gazebo 的 Sensors 系统插件缺失**：`aws_robomaker_small_warehouse_world` 的 world
   文件一个 `<plugin>` 标签都没有，gz-sim 会退回默认系统插件列表（只有
   Physics/UserCommands/SceneBroadcaster），**没有 Sensors 系统**——机器人身上所有
   camera/gpu_lidar 传感器（包括双 Mid-360、包括头部相机）都发布不出任何数据。
   因为不能改 aws 官方 world 文件，改成把 `gz-sim-sensors-system` 插件挂在我们自己
   机器人模型的 `<gazebo>` 标签下（见 `astribot_s1.gazebo.xacro`），全局生效。
2. **gz-sim 传感器消息的 frame_id 是长路径**：不是简单的链接名，而是
   `<model>/<parent_link>/<sensor_name>` 这种全路径字符串（比如
   `astribot_s1/astribot_torso_base/livox_mid360_left_sensor`），TF 树里没有这个
   frame，点云没法直接 tf2 变换。用 `static_transform_publisher` 做零位姿别名，
   把这个长 frame 名字挂到真正的传感器 link 下面（见 `warehouse_sim.launch.py`）。
3. **`read_points_numpy` 遇到混合类型字段会崩溃**：gz-sim 的 gpu_lidar 点云除了
   x/y/z(float32) 还带 intensity(float32) 和 ring(uint16)，`read_points_numpy` 要求
   "消息里所有字段"统一数据类型，遇到混合类型直接 assert 崩溃退出。改用通用的
   `read_points()`（结构化数组，逐字段类型独立），用完再手动拼成 (N,3) 的 float 数组
   （见 `livox_preprocess_node.py`/`livox_fusion_node.py`）。
4. **`use_sim_time` 不能手动 declare_parameter**：rclpy 的 Node 基类会自动声明这个
   标准参数，节点代码里再手动 `declare_parameter('use_sim_time', ...)` 会抛
   `ParameterAlreadyDeclaredException`，通过 launch 的 parameters 字典传值即可，
   不需要（也不能）显式声明。
5. **数组类型的 launch 参数不能直接从 LaunchConfiguration 动态注入**：
   `map_start_pose` 是 `vector<double>` 类型，LaunchConfiguration 解析出来永远是字符串，
   塞进 Node 的 parameters 覆盖字典会被当成字符串类型、和期望类型对不上导致启动报错。
   数组类型参数目前只能走静态 yaml 生效，要改就直接编辑
   `config/mapper_params_localization.yaml`。

## 8. 硬件分支联调清单（未接入真实硬件，尚未验证的部分）

已经完成并验证过的：Livox-SDK2 编译安装、livox_ros_driver2 源码编译通过。
接入真实 Mid-360 后还需要现场核对：

1. **网络配置**：`config/MID360_config_left.json` / `MID360_config_right.json` 里的
   `host_net_info`(网卡IP) 和 `lidar_configs[].ip`(雷达设备IP) 都是占位值，
   必须按现场实际网络环境改——典型走线方式是两台 Mid-360 各用一张独立网卡
   （或者一张网卡配两个IP别名），和雷达出厂标签上的默认IP（通常是192.168.1.1xx段）
   在同一网段。
2. **两个 frame_id 不能混**：`hardware_livox.launch.py` 里给左右两个
   `livox_ros_driver2_node` 各自传了独立的 `frame_id` 参数
   （`livox_mid360_left`/`livox_mid360_right`），启动后用
   `ros2 topic echo /livox/lidar_left --once` 核对 header.frame_id 是否正确，
   TF 断裂/雷达点云对不上号首先查这里。
3. **`xfer_format` 必须是 2**：不用默认的 1（Livox 自定义 CustomMsg），否则下游
   `livox_preprocess_node` 收到的不是标准 PointCloud2，会直接解析失败。
4. **时间同步**：确认没有 `/clock` 仿真时钟残留干扰（硬件分支 `use_sim_time` 全部是
   false），两台雷达的系统时钟要同步（NTP/chrony），否则
   `livox_fusion_node` 的 `sync_slop_sec` 时间窗对不上，融合点云会持续丢帧。

## 9. 故障排查清单（对应任务书要求的异常场景）

| 现象 | 排查方向 |
|---|---|
| 两台雷达 frame_id 冲突、TF 树断裂 | `ros2 run tf2_tools view_frames` 看是否有孤立分支；确认 xacro 里两个 Mid-360 的 `side` 参数没有重复 |
| 仿真雷达话题收不到数据 | 先查 `gz-sim-sensors-system` 插件是否被正确加到 `<gazebo>` 里（见第7节第1点）；再查 `ros2 topic hz` 和 `ign topic -l` 是否对得上 |
| 点云时间戳错乱 | 确认仿真分支/硬件分支的 `use_sim_time` 设置正确（仿真true、硬件false）；检查 `/clock` 是否正常发布 |
| SLAM建图漂移严重、地图重影 | 调整 `pointcloud_filter_params.yaml` 里的 `ground_z_min`/`height_max` 高度切片范围；调整 `livox_fusion_params.yaml` 的 `sync_slop_sec` |
| 雷达模型在Gazebo里穿墙 | 检查 `astribot_s1_sensors.xacro` 里 Mid-360 link 的 collision 标签是否存在（本方案已配 collision，正常不会穿墙） |
| livox_ros_driver2 启动报错/端口占用 | 确认没有两个 driver 实例用了同一个 IP/端口；检查系统是否已经装好 Livox-SDK2（`ldconfig -p \| grep livox`） |
| 定位模式加载地图失败 | 确认 `map_file_name` 是绝对路径、不带扩展名；确认对应的 `.data`/`.posegraph` 两个文件都存在 |
| 点云过载、CPU占用过高 | 调大 `astribot_s1_sensors.xacro` 里 gpu_lidar 的采样间隔(降低samples)，或调大 `pointcloud_filter_params.yaml` 的 `voxel_size` |

## 10. 自主巡游节点（`autonomous_patrol_node`）与全向轮平移bug的完整修复记录

用户反馈"底盘移动时机器人的身体会倾倒"后排查出的 **4 个独立真实 bug**，均为实测复现，
详细技术记录见 `astribot_s1.gazebo.xacro`（bug 1/2）和 `autonomous_patrol_node.py`
文件头部注释（bug 3/4）：

1. 各向异性摩擦近似方案在本机轮子半径(0.08m)下失效 → 改用 `VelocityControl` 系统插件
   直接设定车身速度，`MecanumDrive` 只做轮子视觉转动。
2. `MecanumDrive` 独立计算的轮速（假设真实摩擦驱动）与 `VelocityControl` 强制的车身
   速度不一致，只要轮子还有摩擦系数，这个差异就通过轮地接触力矩反馈到车身
   z/roll/pitch，几十秒到一两分钟内缓慢累积成卡死的倾斜姿态 → 四个轮子
   `mu1`/`mu2` 清零，两个插件彻底解耦。
3. 双 Mid-360 打在机器人自己身上的固定结构（0.10~0.25m 距离，稳定聚簇），被巡游
   节点误判成"到处都是障碍物/被困角落"，只会原地自转 → `pointcloud_filter_params.yaml`
   的 `range_min` 由 0.15 提高到 0.35。
4. 巡游节点把车体系下的"最开阔方向"角度直接当 world 系角度发给 `/cmd_vel`，但
   `VelocityControl` 插件二进制里的组件类型是 `WorldLinearVelocityCmdTag`
   （用 `strings libignition-gazebo6-velocity-control-system.so` 核实），说明它按
   world 系解释速度——车身同时还在自转，车体系角度每周期漂移，导致推力方向随之转动，
   一整圈平均抵消，车子"一直打转不挪窝" → `autonomous_patrol_node.py` 新增订阅
   `/odom` 拿当前航向角，用 `world_angle = best_angle(车体系) + current_yaw` 换算后
   再计算 vx/vy。

同时新增了两层工程纵深防御（不是对上述根因的替代）：
- 基于 `/odom` 的安全监控：z 高度/roll/pitch 超出阈值（默认 `max_height_deviation=0.06`,
  `max_tilt_rad=0.12`≈7°）时强制持续下发零速度并报错，不依赖任何单一修复"包治百病"。
- 碰撞安全扇区跟随实际移动方向（`best_angle`）而不是车体固定正前方，外加不分方向的
  全向最近距离紧急阈值 `critical_stop_distance`（默认0.4m）。

**最终验证**：单实例连续运行 22+ 分钟（`autonomous_patrol:=true`），z 全程稳定在
0.1342m 标称值，roll/pitch 全程 0，安全监控零触发，`/map` 栅格从 274x232 增长到
274x282（真实新增覆盖）。

### 10.1 联调环境踩坑（调试工具本身的问题，不是仿真/机器人 bug）

反复启停 launch 调试时，`pkill -f astribot_sdk_ros2/ws_robot` 这类按工作空间路径
匹配的杀进程命令**杀不掉** `ros_gz_bridge/parameter_bridge`——它的二进制在系统路径
`/opt/ros/humble/lib/ros_gz_bridge/` 下。这次调试一度累积 6-7 个僵尸 bridge 进程
同时抢占同一个 `ROS_DOMAIN_ID`，表现为 `tf2_buffer: Detected jump back in time`
刷屏、`ros2 topic echo` 超时拿不到消息——排查时务必用 `ps -ef | grep parameter_bridge`
单独确认，不能只信"按工作空间路径杀干净了"。

长时间高频反复启停还会在 `/dev/shm/fastrtps_*` 下累积大量共享内存分段，最终导致
`RTPS_TRANSPORT_SHM Error: Failed init_port ... open_and_lock_file failed`（正常
一次性使用不会遇到）。应对：导出下面这份只用 UDPv4、禁用 SHM 的 profile：

```bash
cat > /tmp/disable_shm.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_only" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_transport</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_shm.xml
```

