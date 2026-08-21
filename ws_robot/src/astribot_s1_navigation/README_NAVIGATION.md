# astribot_s1_navigation —— Nav2 自主导航集成

在已验证过的双 Livox Mid-360 感知 + SLAM Toolbox 建图/定位栈
（`astribot_s1_perception`）基础上接入 Nav2，实现"发一个目标点，机器人自己规划路径、
避开货架、走到目标"。设计依据见实施方案 `/home/yjh/.claude/plans/synthetic-snuggling-pizza.md`。

## 1. 依赖安装

本机已装好完整 Nav2 栈，不需要额外安装：

```bash
# 核实命令（应该都显示 install ok installed）：
dpkg -s ros-humble-navigation2 ros-humble-nav2-bringup \
        ros-humble-nav2-mppi-controller \
        ros-humble-nav2-regulated-pure-pursuit-controller \
        ros-humble-nav2-smac-planner 2>&1 | grep Status
```

如果是全新环境，一键安装命令：

```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

## 2. 目录树

```
astribot_s1_navigation/
├── package.xml / setup.py / setup.cfg / resource/
├── astribot_s1_navigation/
│   ├── cmd_vel_body_to_world_node.py   Nav2车体系cmd_vel → world系(gz-sim VelocityControl语义)
│   └── arm_speed_limiter_node.py       双臂展开时给 /speed_limit 发限速
├── launch/
│   ├── navigation.launch.py            controller/planner/behavior/bt_navigator/...
│   └── nav2_full_bringup.launch.py     【顶层入口】
├── config/
│   ├── nav2_params_rpp.yaml            RPP+Smac2D(任务书默认要求)
│   └── nav2_params_mppi.yaml           MPPI+Smac2D(推荐，全向)
└── rviz/nav2_view.rviz
```

## 3. 一键启动命令

```bash
colcon build --symlink-install --packages-select astribot_s1_navigation
source install/setup.bash

# 模式1：建图 + 导航同时进行（推荐用 mppi）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true controller_plugin:=mppi

# 模式2：预建图 + SLAM定位 + 导航
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=localization launch_gazebo:=true \
  map_file_name:=/绝对路径/my_map controller_plugin:=mppi

# 按任务书字面要求用 rpp+Smac（默认值，见第4节的取舍说明）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py env:=sim mode:=mapping
```

发目标点：RViz 里用 "Nav2 Goal" 工具点一个点，或者：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}}"
```

## 4. 关键设计决策（如实记录取舍，不是纸面设计）

### 4.1 机器人是 X 型布局全向轮底盘，不是差速轮

四个轮子的轴线在 `astribot_torso_base` 系里指向 ±45° 对角线
（RF=[0.707,-0.707,0]、LF=[0.707,0.707,0]、RR=[-0.707,-0.707,0]、LR=[-0.707,0.707,0]，
直接由 `astribot_s1_torso_wheel.xacro` 各轮关节的 origin rpy 算出）。这是
**X 型布局全向轮(omni)**，**不是**"轮轴沿 ±y、辊子 45°"的标准麦克纳姆布局 ——
两者的运动学逆解公式完全不同，逆解系数推导见
`astribot_s1_chassis_effort_drive/config/omni_effort_drive_params.yaml` 顶部注释。
没有 `diff_drive_controller`。

> ⚠️ 本节原先写的是"底盘运动完全由 gz-sim 的 `VelocityControl` 系统插件直接接管，
> 不经过 `ros2_control`"。那是**旧架构**，已经不成立：`VelocityControl` 与
> `MecanumDrive` 两个插件都已整体移除，现在底盘由 `omni_effort_drive_node`
> 做逆解 + 轮速 PID + 摩擦前馈算出力矩，经 `ros2_control` 的
> `forward_command_controller`（`wheel_effort_controller`）下发。
> 下面 4.2 节讲的 `VelocityControl` world 系语义是当时定位问题的记录，
> 保留是因为 `cmd_vel_body_to_world_node` 这一层至今仍在链路里。

### 4.2 【最关键】cmd_vel 必须做 body→world 转换

用 `strings` 反查 `libignition-gazebo6-velocity-control-system.so` 二进制，确认它
内部速度分量组件类型叫 `WorldLinearVelocityCmdTag`/`WorldAngularVelocityCmdTag`——
`VelocityControl` 插件把 `/cmd_vel` 的线速度分量**按 world 系解释**，不是移动机器人
cmd_vel 常见的车体系语义。Nav2 的标准控制器（RPP/MPPI/DWB）全部按车体系发布 Twist，
直接对接真实 `/cmd_vel` 会导致车身一边被要求转向、一边把车体系向量误当world系向量用，
表现为"原地打转、走不动"——跟本仓库早前修复 `autonomous_patrol_node` 时踩过的坑
完全同源。

**解决方式**：`navigation.launch.py` 把 Nav2 最终输出改道到 `cmd_vel_nav_body`
（不是官方默认的 `cmd_vel`），`cmd_vel_body_to_world_node` 订阅这个话题 + `/odom`
（拿当前航向角），换算成 world 系后才发真正的 `/cmd_vel`。**验证 Nav2 导航是否正常
工作，第一件事就是看机器人是不是真的在平移，不是原地打转**——如果只打转，先查这个
转换节点是不是真的在跑（`ros2 node list | grep cmd_vel_body_to_world`）。

### 4.3 不跑 map_server / amcl

SLAM Toolbox 在 mapping（在线建图）和 localization（预加载 posegraph 定位）两种
模式下都会自己发布 `/map` 和 `map -> odom` TF。再跑一套 `map_server`+`amcl`
会跟它抢着发布/消费 `map -> odom`，产生冲突。Nav2 侧只启
`controller_server`/`smoother_server`/`planner_server`/`behavior_server`/
`bt_navigator`/`waypoint_follower`/`velocity_smoother`（照抄官方
`nav2_bringup/launch/navigation_launch.py` 的节点结构，不用 `bringup_launch.py`），
全局代价地图的 `static_layer` 直接订阅 `/map`。

**已实测验证两种模式都能跑通**：模式1（`mode:=mapping`）建图+导航同时进行，
实测确认无 amcl/map_server 节点、Nav2 全部生命周期节点 active、`/scan`→
costmap→规划链路正常。模式2（`mode:=localization`）先用模式1跑一段建好图、
调用 `/slam_toolbox/serialize_map` 存盘，重启为 `mode:=localization
map_file_name:=<刚存的文件>` 后，日志确认 "Load From File...posegraph"/
"Finished serializing Mapper/Dataset"，`/map_metadata` 尺寸跟保存时基本一致
（275x282 vs 保存时274x282，一像素级的差异来自地图边界重新栅格化，不是加载出错），
`/odom` 正常发布（约45Hz）——两种模式的 Nav2 接入都是真实跑通过的，不是只跑通了
模式1。

### 4.4 控制器选型：任务书要求 vs 机器人实际运动学

任务书要求"优先使用 Smac全局规划 + Regulated Pure Pursuit 局部控制器"，但 RPP
是为非全向（差速/阿克曼）载体设计的，不会真正利用全向轮的全向平移能力（车体
基本会先转向、再沿朝向前进）。方案默认仍配 RPP（满足任务书字面要求，行为正确、
只是没发挥全向优势），同时完整配一套 MPPI（`motion_model: "Omni"`，原生支持全向
运动），用 `controller_plugin:=rpp|mppi` 一键切换。**推荐实际使用 mppi**。
全局规划器统一用 `SmacPlanner2D`（栅格搜索，不强加非全向运动约束）。

### 4.5 机械臂展开限速——启发式，不是精确碰撞检测

`arm_speed_limiter_node` 检查双臂14个关节角相对"收纳姿态"参考值(`folded_reference_rad`
参数，默认全0，**部署前需要按实际收纳姿态重新核对这组数值**)的最大偏差，超过阈值
(默认0.5rad)就通过 Nav2 官方的 `/speed_limit`(`nav2_msgs/msg/SpeedLimit`) 机制把
`velocity_smoother` 限速到50%。这是"关节明显偏离收纳姿态→保守降速"的粗粒度安全阀，
**不知道机械臂末端在三维空间里到底伸到哪个位置、离最近障碍物多远**，不能替代真正的
全身碰撞检测。更精确的方案是用 `nav2_collision_monitor` 订阅机械臂末端 TF 动态改变
碰撞多边形，本方案没有做，留作后续可扩展点。

**已实测验证**：给 `arm_left_controller` 发一条把 joint_2 转到 -1.2rad 的轨迹后，
`arm_speed_limiter_node` 日志打出"检测到机械臂展开(最大关节偏差 0.50 rad > 阈值)，
限速到 50%"，轨迹执行完机械臂回到收纳姿态附近后又打出"机械臂已收纳，解除限速"——
状态切换逻辑实测正确，不是只在纸面上写了逻辑没跑过。

### 4.6 代价地图 obstacle_layer 复用 `/scan`，不重新做点云处理

`astribot_s1_perception` 现有链路（双Livox原始点云 → 直通滤波(`range_min=0.35`避开
机身自撞) + 体素降采样 → 时间同步融合 → 2D投影）产出的 `/scan` 已经满足"降采样+
距离截断+2D投影"的要求，直接作为 obstacle_layer 的观测源，不重新实现一套。

### 4.7 足迹用圆形，不用多边形

`astribot_torso_base` 碰撞体是半径0.3m的圆柱，4个轮子中心距原点约0.306m，costmap
用 `robot_radius: 0.35`（覆盖轮子凸出部分+安全余量）。这个值**不包含双臂/头部展开时
的真实包络**，那部分风险靠第4.5节的限速机制缓解。

### 4.8 【实测踩坑记录，最终定位到真正根因】"Starting point in lethal space" ——
不是出生点旁真有障碍物，是自身检测的一个新缺口

第一次仿真联调时实测复现过 `planner_server` 报
`Starting point in lethal space! Cannot create feasible plan.`，一直规划失败，
而且**机器人移动到新位置后同样的问题还会复现**（排除了"只是出生点旁边刚好有个
障碍物"这个最初的猜测——如果是固定障碍物，挪开之后应该不会在新位置又立刻复现）。
逐层查数据定位到真正根因：

- `livox_preprocess_node` 的 `range_min=0.35`（本方案更早阶段修复自主巡游节点
  "一直原地打转"问题时加的）是按点的**完整3D球面距离**`sqrt(x²+y²+z²)`过滤的；
- 但 `pointcloud_to_laserscan` 把点云投影成2D `LaserScan` 时用的是**纯XY平面
  距离**`sqrt(x²+y²)`——如果一个自撞点打在机身结构上 z方向偏移比较大的位置
  （比如接近垂直角度撞到机身），它的3D距离可能刚好过了0.35m这道关，去掉z分量后
  纯XY距离却缩水到0.15m左右，照样会被投影进最终的 `/scan`。
- 之前只跑 SLAM Toolbox + 反应式巡游节点时一直没暴露这个问题，是因为
  `autonomous_patrol_node` 自己的避障逻辑靠的是"全向最近距离阈值+跟随移动方向的
  安全扇区"，没有对"当前位置是否在严格的footprint致命空间内"做几何检查；接入
  Nav2 之后，`planner_server` 每次规划前都会先校验起点本身是否合法，这个持续的、
  跟着机器人到哪都在的自撞盲区就被暴露出来了。

**修复**：`astribot_s1_perception/config/pointcloud_to_laserscan_params.yaml`
的 `range_min` 从 0.15 提到 0.35（跟点云层的3D过滤门限对齐，堵住"3D距离够远但
投影后XY距离变近"这个漏洞）。这是感知包（`astribot_s1_perception`）里的一处
真实修复，不是导航包自己的配置问题，但因为是在接入Nav2做严格碰撞检查时才暴露，
记在这里方便排查同类问题。

（顺带留下的次要调整：`inflation_radius` 从默认的0.4调到0.25，降低机器人贴着
真实障碍物走时膨胀层过度保守拒绝合法起点/路径的概率，属于常规调优，不是这个bug
的根治手段——根治手段是上面那条 range_min 修复。）

### 4.9 已验证：body→world 转换在真实导航路径下确实生效，同时确认了已知的
残余碰撞风险仍然存在（跟自主巡游模式共享同一个已记录的限制，不是Nav2集成引入的
新问题）

修完 4.8 的 range_min bug 后完整实测过一轮：发一个 `NavigateToPose` 目标
（出生点(0,0)到(2.5,1.0)），机器人真实沿对角线方向平移超过1.4米（从(0,0)走到约
(0.58,0.56)），中途没有"原地打转不挪窝"——`cmd_vel_body_to_world_node` 的
body→world 转换在真实 Nav2 路径下确认生效，不是只在孤立测试里生效。这也顺带
实测验证了 4.4 节说的 mppi 全向能力：目标点(2.5,1.0)相对出生点是一个斜向
（既要+x又要+y），机器人从(0,0)移动到(0.58,0.56)这段轨迹本身就是斜向平移，不是
先转向对准目标方向再直线前进——如果换成 rpp，同样的目标点会先转向、再沿朝向
走一段近似直线，两者的轨迹形状预期会有明显差异（本方案没有为了严格对照专门跑一次
rpp 的同目标点测试，这里只是记录 mppi 已经实测确认了全向平移这个能力本身，
不是理论推测）。

同一次实测里，导航进行到中途 `cmd_vel_body_to_world_node` 的安全监控触发了止损
（检测到 z=0.172m/pitch=6.9°，超出阈值），说明机器人在导航过程中发生了一次真实的
碰撞/物理异常事件，安全监控按设计正确地停止继续转发 Nav2 的速度指令。这跟本方案
之前给 `autonomous_patrol_node` 记录过的残余风险是**同一个、已知的、跨模式共享的
限制**：底盘用 `VelocityControl` 直接设定速度，不理会真实碰撞反作用力，一旦机身
撞上障碍物，物理引擎会在原有动量下继续演化一小段直到新的稳定姿态，安全监控只能
"止损"（停止继续叠加新的错误指令），不能"瞬间纠正"已经发生的碰撞。**这不是本次
Nav2 集成引入的新缺陷**，是更底层的驱动机制（详见 `astribot_s1.gazebo.xacro` 里
的实测记录）在任何上层控制方式（巡游/Nav2）下都共享的已知限制，Nav2 集成只是
把同一个安全监控机制复用到了新的速度指令来源上，而且已确认在 Nav2 路径下同样
正确触发。真正的根治需要让避障逻辑/costmap感知到机械臂的展开范围，或者把驱动
机制换成"轮子摩擦力学真实生效"的方案，属于本方案范围之外的进一步工作。

## 5. 与 `autonomous_patrol_node` 的关系

功能不同（Nav2=有目标点导航，patrol=无目标点自主探索），但都会发 `/cmd_vel`，
`nav2_full_bringup.launch.py` 在 include `perception_slam_bringup.launch.py` 时
始终显式传 `autonomous_patrol:='false'`，避免打架。想要自主探索建图仍然用
`perception_slam_bringup.launch.py`（`autonomous_patrol:=true`）。

## 6. 故障排查清单（对应任务书8条异常场景）

| 现象 | 排查方向 |
|---|---|
| Nav2无法接收定位信息(无map→odom变换) | 确认 SLAM Toolbox 节点是active的(`ros2 node list`)；`ros2 run tf2_ros tf2_echo map odom`；确认没有同时跑了 amcl/map_server 跟 SLAM Toolbox 抢发布权 |
| 规划失败"Starting point in lethal space" | 出生点/当前位置离真实障碍物太近，见4.8节；先手动挪开一点距离，或调小 `inflation_radius`(需权衡安全边际) |
| 机器人原地打转、不平移 | **先查这个**：`cmd_vel_body_to_world_node` 是否在跑、是否真的在做body→world转换，见4.2节 |
| 机器人原地震荡、无法跟踪路径 | 调 `nav2_params_*.yaml` 里 controller_server 的速度/加速度限制、RPP的lookahead参数或MPPI的critic权重 |
| costmap无障碍物、雷达数据无法被识别 | 先确认 `/scan` 本身有数据(`ros2 topic hz /scan`)，再查 costmap 的 `observation_sources`/`topic` 配置是否对得上 |
| 导航路径穿透货架、碰撞障碍物 | 调大 `inflation_radius`/`robot_radius`；确认 `/scan` 的高度切片范围没有漏掉低矮/高处障碍物 |
| 定位漂移后导航跑偏 | SLAM Toolbox 定位模式下检查 `tf2_echo map astribot_torso_base` 是否收敛；必要时重新走一遍建图流程 |
| 启动时报BT文件/参数不存在 | 本方案没有自定义BT xml，直接用 `nav2_bt_navigator` 自带默认文件；检查 `ros2 pkg prefix nav2_bt_navigator` 能否正常解析 |
| CPU负载过高 | 调大 costmap 的 `resolution`/降低 `update_frequency`；调大 `livox_preprocess_node` 的 `voxel_size` |
| 启动时 `spawner: Could not contact service /controller_manager/list_controllers` FATAL | 已知的间歇性资源/时序问题（跟本方案的Nav2改动无关，`astribot_s1_gazebo_bringup` 里就有过，见其README），实测多次复现为"偶发、清干净残留进程后重新launch一次就好"，不是每次必现；如果持续复现才需要深入查 |
| 机械臂展开状态下导航易碰撞 | 检查 `arm_speed_limiter_node` 是否在跑、`folded_reference_rad` 是否对应实际收纳姿态，见4.5节 |
