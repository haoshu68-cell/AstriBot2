# astribot_s1_manipulation — 双臂运动规划栈

Astribot S1 的 MoveIt2 运动规划能力包。四块能力：OMPL 规划器扩展、双臂闭链约束、
碰撞/奇异检测、轨迹节拍优化。配套配置在 `astribot_s1_moveit_config`。

---

## ⚠️ 先读这几条（都是实测踩过的坑，不知道会白花很多时间）

### 1. 本机器人的「全零构型」是奇异构型，不能从它起步规划

两条臂的 `joint_4` 是肘关节，限位 `[0.0, 2.4]`。**`joint_4 = 0` 就是肘部完全伸直**，
即教科书上的奇异构型。实测该构型 `sigma_min = 0.0077`，低于本工程阈值 `0.02`，
奇异检测会（正确地）拒绝任何以它为起点的轨迹：

```
attempt 1/3: singular configuration at waypoint 0/12
             (sigma_min=0.00771057 below threshold 0.02) -> discarding solution
```

而 `joint_state_publisher` 默认把所有关节发成 0，SRDF 里的 `home` 也是全零。
所以：**规划前必须先让机器人离开全零构型**。SRDF 的 `ready` 姿态就是为此准备的
（`joint_4 = 1.0`，实测 `sigma_min = 0.116`，余量充足）。
`demo.move_to_ready_first: true` 会自动做这件事。

### 2. `planning_attempts` 必须是 1，否则 Informed RRT* / BIT* 必定失败

这是一条真实的 **OMPL ↔ MoveIt 不兼容**，不是配置写错。

`planning_attempts > 1` 时 MoveIt 走 `ompl::tools::ParallelPlan` 并行跑多个规划器实例，
而 MoveIt 的目标是用 `GoalLazySamples`（懒惰目标采样线程）表达的。
informed 采样类规划器在创建采样器时要求"至少已有 1 个起点和 1 个目标状态"，
并行分支下目标线程常常一次都没跑完，于是：

```
Debug:   Stopped goal sampling thread after 0 sampling attempts
Error:   Exception thrown during ParallelPlan::solveMore:
         PathLengthDirectInfSampler: There must be at least 1 start and
         and 1 goal state when the informed sampler is created.
Warning: Goal sampling thread never did any work.
```

MoveIt 把它翻译成一句毫无信息量的 `Unable to solve the planning problem`，
看起来像"这个规划器不行"或"目标不可达"，极难定位。

设 `planning_attempts: 1` 后走单规划器路径，日志出现 `Using informed sampling.`
并正常出解。需要多次尝试请用本包自己的 `max_replan_attempts`
（每次是完整一轮"规划 + 全套校验"，不走 ParallelPlan）。

### 3. 换回官方 OMPL 插件会让 BIT* / Informed RRT* 静默失效

MoveIt2 Humble 官方的 `ompl_interface/OMPLPlanner` 只注册了 25 个规划器，实测
三个必需规划器里**只有 `RRTstar`**：

```bash
strings /opt/ros/humble/lib/libmoveit_ompl_interface.so \
  | grep -oE "geometric::[A-Za-z_]+" | sort -u    # 25 个，无 BITstar / InformedRRTstar
```

而 OMPL 1.7 本体是有的（`informedtrees/BITstar.h`、`rrt/InformedRRTstar.h`）。
本包的 `astribot_s1_manipulation/OmplPlannerExtension` 插件用 MoveIt 公开 API
把它们补注册进去（不改任何系统库源码）。

**关键风险**：`ompl_planning.yaml` 里写一个 MoveIt 不认识的规划器名时，
MoveIt **不报错**，而是静默回退到该组默认规划器。所以切换规划器后一定要看日志：

```
[astribot_s1_manipulation.ompl_extension]: registered extra OMPL planners: 25 -> 30 total
[astribot_s1_manipulation.ompl_extension]:   [OK]      geometric::RRTstar
[astribot_s1_manipulation.ompl_extension]:   [OK]      geometric::BITstar
[astribot_s1_manipulation.ompl_extension]:   [OK]      geometric::InformedRRTstar
[moveit.ompl_planning...]: Planner configuration 'arm_left[BITstarConfig]'
                           will use planner 'geometric::BITstar'
```

最后那行是"真的在跑 BIT*"的唯一证据。

---

## 编译

```bash
cd ~/WorkSpace/astribot_sdk_ros2/ws_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
    astribot_s1_moveit_config astribot_s1_manipulation
source install/setup.bash
```

跑单元测试（不需要仿真、不需要 move_group）：

单元测试已从仓库删除；历史回归不作为当前可执行入口。

---

## 启动

### 只验证规划（不开 Gazebo，最快）

需要有 `/robot_description` 和 `/joint_states`。开三个终端：

```bash
# 终端 1：机器人模型
xacro $(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/urdf/astribot_s1.xacro \
      robot_name:=astribot_s1 > /tmp/astribot_s1.urdf
ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p robot_description:="$(cat /tmp/astribot_s1.urdf)" -p use_sim_time:=false

# 终端 2：关节状态。注意不要用默认的全零（见上面第 1 条），
# 要么用 GUI 手动拖离全零，要么发一个 ready 姿态。
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# 终端 3：move_group + demo
ros2 launch astribot_s1_manipulation planning_demo.launch.py use_sim_time:=false
```

### 全链路仿真（能真的看到机器人动）

```bash
# 终端 1：仿真（含 Gazebo、ros2_control、4 个 JointTrajectoryController）
ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py

# 终端 2：move_group + demo，并把轨迹下发执行
ros2 launch astribot_s1_manipulation planning_demo.launch.py execute:=true
```

MoveIt 通过既有的 `arm_left_controller` / `arm_right_controller` / `torso_controller`
下发轨迹（都是 `JointTrajectoryController`，100Hz），不需要新增任何控制器。

### 只起 move_group（自己写客户端时）

```bash
ros2 launch astribot_s1_moveit_config move_group.launch.py use_rviz:=true
```

---

## 如何切换 OMPL 规划器

三种方式，优先级从高到低：

```bash
# 1) 命令行（最常用）
ros2 launch astribot_s1_manipulation planning_demo.launch.py planner_id:=BITstarConfig

# 2) 改 yaml：astribot_s1_manipulation/config/manipulation_params.yaml
#    planner_id: "InformedRRTstarConfig"

# 3) 代码里按请求覆盖：SingleArmPlanRequest::planner_id / ClosedChainPlanRequest::planner_id
```

可用的**配置名**（不是 OMPL 类名，别写混）：

| 配置名 | 实际规划器 | 说明 |
|---|---|---|
| `RRTstarConfig` | `geometric::RRTstar` | 官方已注册，渐进最优 |
| `BITstarConfig` | `geometric::BITstar` | **本包插件注册**，批量启发式，收敛快 |
| `InformedRRTstarConfig` | `geometric::InformedRRTstar` | **本包插件注册**，椭球启发采样 |
| `RRTConnectConfig` | `geometric::RRTConnect` | 非最优但快，作对比基线 |

另外插件还注册了 `geometric::ABITstar` / `AITstar` / `SORRTstar`，想用的话在
`ompl_planning.yaml` 里照样加一段 `type:` 即可。

每个规划器的参数在 `astribot_s1_moveit_config/config/ompl_planning.yaml`，
各自独立一段。**参数名写错不会报错但也不生效** —— 本包插件会检出并打 WARN：

```
planner 'arm_left/arm_left[InformedRRTstarConfig]' has no parameter named 'focus_search'
(value 'true' ignored); check the key spelling against OMPL's declareParam names
```

（OMPL 原生行为是静默忽略未知参数，所以这条 WARN 是本包特意加的。）

---

## 如何修改闭链约束

全部在 `config/manipulation_params.yaml` 的 `closed_chain:` 段。

### 闭链的数学定义

两臂夹持同一刚体 ⇒ 两个 TCP 的相对位姿恒定：

```
T_rel = T_L^{-1} · T_R = const
```

残差按下式算，**位置与姿态量纲不同，分开设阈值，不能加成一个标量**：

```
E   = T_L^{-1} · T_R · T_rel^{-1}      理想为单位矩阵
e_p = ||translation(E)||               单位 m
e_r = |angle-axis(rotation(E))|        单位 rad
```

### 常用改法

```yaml
closed_chain:
  # 换哪条臂当 leader（物体偏右时用右臂当 leader 更容易规划）
  leader_group: "arm_left"
  follower_group: "arm_right"

  # 换夹具/换 TCP 只改这两行，不用动 SRDF
  leader_tcp_link: "astribot_arm_left_tool_link"
  follower_tcp_link: "astribot_arm_right_tool_link"

  # true  = 从当前状态捕获 T_rel（推荐：先手动把两臂摆到夹持位姿再规划，
  #         夹持几何由实际摆位决定，不需要任何人去量尺寸）
  # false = 用下面两行显式给定（适合已知物体尺寸的标定场景）
  capture_from_current_state: true
  relative_translation: [0.0, -0.4, 0.0]      # m
  relative_rotation_xyzw: [0.0, 0.0, 0.0, 1.0]

  # 残差阈值。放宽会让夹持物体承受更大内力，收紧会让 IK 更容易被拒
  position_tolerance: 0.005        # m
  orientation_tolerance: 0.02      # rad
```

### 闭链返回 ALREADY_AT_GOAL 是什么意思

不是失败。leader 每个关节与目标的偏差都在 `already_at_goal_tolerance_rad`
（默认 1e-3 rad）以内时，本包直接返回 `ALREADY_AT_GOAL`、不输出轨迹、也不重试。

判断"要不要当失败处理"请用 `!succeeded() && !noActionNeeded()`，
不要只看 `!succeeded()` —— `succeeded()` 的语义是"有一条合法轨迹可以执行"，
已经到位时它理应为 false。

这个码是实测加出来的：起点==目标时 OMPL 返回一条"2 个相同状态、代价 0.00"的
退化路径（日志 `Found an initial solution with a cost of 0.00` +
`changed from 2 to 2 states`），加密后有效路点数 < 2。原先这被判成
`PLANNER_FAILED` 并重试 3 次（每次必然拿到同一条退化路径，白烧 1.5s），
最后报 `RETRIES_EXHAUSTED`，消息还是
`leader path has fewer than 2 waypoints after densification` ——
把"已经到位了"说成"规划器坏了"，排查方向完全错。

### 闭链规划失败时怎么查

demo 会打印 `follower IK 失败点数`。按这个数字判断：

| 现象 | 处置 |
|---|---|
| IK 失败点数多（>轨迹一半） | follower 目标超出可达空间。换 leader、缩小 leader 运动幅度，或调 `ik_attempts`（**不要**调大 `ik_timeout`，单次 IK 要么很快收敛要么真无解） |
| 错误码 `CLOSED_CHAIN_RESIDUAL_TOO_LARGE` | IK 收敛到了容差外的解。适度放宽 `position_tolerance`，或减小 `densify_max_joint_step` 让相邻点更近（IK 种子更好） |
| 错误码 `SINGULAR_CONFIGURATION` | 闭链构型把某条臂推到奇异。换起始姿态，或适度降低 `singularity.min_singular_value`（但别低于 0.01） |
| 错误码 `SELF_COLLISION` | 两臂在闭链约束下互撞。改 `T_rel`（夹持更宽/更窄），或换 leader |

---

## 如何看节拍指标 / 怎么调节拍

### 指标从哪来

demo 每条轨迹都会打印：

```
  总运动时长: 0.381 s
  最大关节速度: 3.9132 rad/s (astribot_arm_right_joint_1)
  最大关节加速度: 21.6085 rad/s^2 (astribot_arm_right_joint_1)
  速度利用率峰值: 46.6% (越接近 100% 说明节拍越紧)
  轨迹合法性: 合法
优化: 节拍对比: 时长 0.707s -> 0.381s (缩短 46.1%) | ... | 优化后合法性: 合法
  已采纳 TOTG 优化，节拍缩短 46.1%
```

「速度利用率」= 实际峰值速度 / 关节硬限位。这是判断"还有多少压缩余量"的核心指标：
远小于 100% 说明还能更快，接近 100% 说明已经贴着限位。

### 节拍优化怎么工作

同一条几何路径分别做两次时间参数化，取更快且**合法**的：

- **baseline = IPTP**（`IterativeParabolicTimeParameterization`）：保守、鲁棒，作对比分母
- **优化 = TOTG**（`TimeOptimalTrajectoryGeneration`）：时间最优，把限位真正跑满

优化不改路径形状，只重新分配时间戳，所以不会引入新的碰撞风险。

**硬契约**：优化结果由 `trajectory_metrics` 独立复核限位，
超限 / 参数化失败 / 反而更慢 → **回退 baseline 并告警**，绝不输出非法轨迹。
连 baseline 都超限 → 直接返回 `JOINT_LIMIT_VIOLATION`，**不输出任何轨迹**。

### 调参手册

```yaml
time_optimizer:
  # 想更快：把这两个往 1.0 推。但真机建议留 10% 余量给跟踪误差，
  # 否则控制器一滞后就超限报警
  optimized_velocity_scaling: 0.90
  optimized_acceleration_scaling: 0.90

  # 拐角抹圆容差。给大了节拍更好但会偏离原路径（有碰撞风险，
  # 因为碰撞是在原路径上校验的）；给 0 会让拐角速度掉到 0，节拍反而更差
  totg_path_tolerance: 0.1

  # 这条很关键：OMPL 路径常含几乎重合的点，不合并会让 TOTG
  # 在这些点上算出巨大速度（除以接近 0 的距离）
  totg_min_angle_change: 0.001
```

### ⚠️ 加速度限位是估算值，上真机前必须标定

URDF 里**只有** velocity/effort，**没有加速度限位**（实测确认）。
`astribot_s1_moveit_config/config/joint_limits.yaml` 里的加速度是按
`a_max = v_max / ramp_time`（`ramp_time = 0.35s`）推导的，
**这是工程起始估算值，没有实机辨识数据支撑**。

TOTG 会按这个上限压缩节拍。如果它偏大，生成的轨迹控制器根本跟不住
（表现为跟踪误差告警、抖动）。上真机前用单关节阶跃响应标定：
给一个已知幅值的位置阶跃，测实际达到满速所需时间，用它替换 `ramp_time`。
偏保守时把 `ramp_time` 调大（= `a_max` 调小），不用改代码。

---

## 实测结果：mobile_transport 搬运 → 导航 → 搬运（Gazebo 全链路执行，2026-08-20）

完整移动作业流程：起始位置取货 → 底盘导航 → 目标位置放货。

```bash
# 终端1：仿真 + nav2（定位模式）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
    slam_mode:=localization map_file_name:=<ws>/ws_robot/maps/warehouse_full

# 终端2：move_group + 流程编排
ros2 launch astribot_s1_manipulation planning_demo.launch.py \
    scenarios:=mobile_transport execute:=true nav_goal:="0.995,1.712,-1.673"
```

### 六步序列一行都没改

`buildTransportSteps()` 的六步天然可分成「取货 1~3 / 放货 4~6」，
导航就插在第 3 步(抬起)之后。第 4 步"移动到 B 上方"是**体系内**的横移，
底盘挪没挪都成立 —— 所以 `mobile_transport` = `transport` + 一段导航，
步骤定义、取货点、物体尺寸、`grasp_z_offset`、姿态全部复用 `transport_*`。

### 物体坐标系：体系就够，不需要 AttachedCollisionObject

物体作为碰撞体发布在 `astribot_torso_base`（随机器人移动的体系）。
导航途中机械臂**保持抬起姿态不动**，物体与机器人的相对位姿恒定，
体系坐标严格成立，物体自然跟着走。
**如果以后要在导航途中收臂，这个前提就破了**，必须改成把物体附着到 TCP link
（`AttachedCollisionObject` + `touch_links`）。

### 世界位移：物体真的被搬走了（数值自洽到 1mm）

RRT\* 那一轮实测：

```
取货后：物体 map=(0.909, 5.397, 0.761)  底盘 map=(0.463, 5.594)
放货后：物体 map=(1.136, 1.673, 0.761)  底盘 map=(0.918, 1.908)
世界位移：物体 (0.227, -3.724) 距离 3.731m | 底盘 (0.455, -3.686) 距离 3.714m
```

自洽性验算（这一步是必要的 —— 否则"体系里换了个数"和"世界里真搬走了"分不清）：
终点实测 yaw = −1.913 rad，把体系放置点 B=(0.149, 0.284) 按该 yaw 旋转：

```
x' = 0.149·cos(−1.913) − 0.284·sin(−1.913) = 0.217
y' = 0.149·sin(−1.913) + 0.284·cos(−1.913) = −0.236
```

与实测的 (物体−底盘) = (0.218, −0.235) 吻合到 1mm。取货端同理（0.446, −0.197
对 0.446, −0.195）。所以物体的世界位移确实 = 底盘位移 + 体系内 (B−A) 位移。

### 三求解器在这个流程上的横向验证

先用 RRT\* 调通，再在这个已验证基准上换求解器（导航目标在起点/终点之间来回换，
否则第二轮起机器人已经在目标上、导航段等于没跑）：

| planner | 结果 | 机械臂步数 | 导航 | 规划总耗时 | 全程最差 σ | 物体世界位移 |
|---|---|---|---|---|---|---|
| `RRTstarConfig` | PASS | 6/6 | SUCCEEDED 50.5s | 3.218s | 0.1113 | 3.731m |
| `BITstarConfig` | PASS | 6/6 | SUCCEEDED 79.4s | 0.185s | 0.1143 | 3.928m |
| `InformedRRTstarConfig` | PASS | 6/6 | SUCCEEDED 54.9s | 3.227s | 0.1111 | 3.727m |

规划耗时的分档仍然是 `optimization_budget_sec` 配置造成的，不是规划器快慢
（理由见上一节）。导航耗时差异与求解器无关 —— 那是底盘的事。

### ⚠️ 实测发现：不收臂让底盘只能跑 15% 速度

3.71m 走了 50.5s，平均 0.076m/s。路径跟踪诊断节点把原因定位得很干净：

```
正常 | 路径在(73点) | raw 21.0Hz/0.704 -> smooth 0.704 -> preCpl 0.704 -> cmd 0.106
                   | 轮速峰值 1.099rad/s | 实速 0.104m/s
```

MPPI 输出 0.704m/s 一路原样传到 `preCpl`，在**臂-底盘耦合限速**这一级被砍到
0.106 —— 比值 0.106/0.704 = **0.151**，正好是
`arm_chassis_coupling_params.yaml` 里的 `min_speed_scale: 0.15`。
整段导航 25 个采样全是"正常"，0 次卡滞，所以这不是走不动，是**按设计被限速**。

限速公式（`arm_chassis_speed_coupling_node.py`）：

```
extension_ratio = max over 14 joints of min(1, |pos − folded_reference| / extension_full_rad)
scale = 1 − activity · (1 − min_speed_scale)
```

搬运姿态下至少有一个关节偏离参考 ≥ `extension_full_rad`(1.2)，
`extension_ratio` 饱和成 1.0，于是 scale 掉到下限 0.15。

**顺带暴露一个跨包的设计不一致**（未修，记录在此）：
`folded_reference_rad` 默认是 14 个 0，而本机器人的**全零构型就是奇异构型**
（`joint_4=0` 即肘部完全伸直，实测 σ_min=0.0077，见本文开头第 1 条）。
也就是说 `scale=1.0` 只在一个**永远不该使用的构型**上取得：
连 SRDF 的 `ready` 姿态(`joint_4=1.0`)都会得到
`ratio = 1.0/1.2 = 0.83`、`scale ≈ 0.29`。
换句话说**任何可用的机械臂构型下底盘都在被限速**。
要么把 `folded_reference_rad` 改成真实的收纳姿态，要么放大
`extension_full_rad` —— 但两者都要重新做倾覆余量评估，所以这里只记录不动手。

实用结论：追求节拍时应该在导航前把臂收到接近 `folded_reference_rad` 的姿态。
本 demo 刻意不收臂，是为了让体系坐标严格成立（见上），代价就是 6.6 倍的时间。

### 边界（如实说明）

- 导航途中伸出的手臂 + 物体**超出了代价地图那个 0.42m 外接八边形足迹**，
  nav2 看不到它们。所选目标点周边净空 1.65m、路径全程开阔，
  所以实测没出问题，但这一点**没有被验证过**，换到窄环境要重新评估。
- 导航目标点不是随便取的：净空 1.65m（`distance_transform_edt` 实测）在 MPPI
  足迹代价饱和阈值 1.62m 之上，且下发前用 `ComputePathToPose` 预检过可达。
  换点必须重做这两项检查，否则容易把"目标本来就不可达"误读成"局部规划器走不动"。
- 仍然是**纯运动学演示**（本机无夹爪），物体不会在 Gazebo 里被夹住跟着走。


## 实测结果：transport 搬运场景 + 四求解器横向验证（Gazebo 全链路执行，2026-08-20）

先把 `transport` 场景在一个求解器上调通，再拿这个**已验证成功的基准**去跑其余
自定义求解器。命令（只换 `planner_id`，其它全不动）：

```bash
ros2 launch astribot_s1_manipulation planning_demo.launch.py \
    scenarios:=transport execute:=true planner_id:=<Config名>
```

四轮结果（每轮 6 步全部规划 + 真实下发执行，grep `[transport][summary]` 即得）：

| planner | 注册来源 | 结果 | 规划总耗时 | 轨迹总节拍 | 全程最差 σ |
|---|---|---|---|---|---|
| `RRTConnectConfig` | MoveIt 官方 | PASS 6/6 | 0.220s | 2.179s | 0.1150 |
| `BITstarConfig` | **本工程插件** | PASS 6/6 | 0.221s | 1.483s | 0.1119 |
| `RRTstarConfig` | MoveIt 官方 | PASS 6/6 | 3.159s | 1.421s | 0.1110 |
| `InformedRRTstarConfig` | **本工程插件** | PASS 6/6 | 3.196s | 1.552s | 0.1143 |

每轮都核对过 move_group 打印的**实际**规划器名（`will use planner 'geometric::XXX'`），
不是只看配置名 —— 名字写错时 MoveIt 会静默回退，见前面第 3 条。
Informed RRT* 还额外确认了 `Using informed sampling.` 确实出现。

### 这张表怎么读（不要读成"BIT* 比 RRT* 快 14 倍"）

规划耗时的分档**是配置造成的，不是规划器能力差异**：

- `RRTstarConfig` / `InformedRRTstarConfig` 配了 `optimization_budget_sec: 0.5`，
  即首解之后再优化 0.5s。6 步 × 约 0.53s ≈ 3.16s，完全对得上。
- `BITstarConfig` **刻意不配**这个键（理由见 `ompl_planning.yaml`：BIT* 有自己的
  批次收敛判据，配 0.5s 窗口只会把 0.015s 拖成 0.5s）。所以它停在首解。

所以横向可比的只有"同样停在首解"的那两个：

- **RRTConnect 0.220s / 节拍 2.179s** vs **BIT\* 0.221s / 节拍 1.483s**
  —— 规划开销一样，BIT\* 的**首解**节拍就好 32%。这是 BIT\* 真正的价值所在。
- RRT\* 花 6×0.5s 的额外优化换到 1.421s，只比 BIT\* 的免费首解好 4%。
  也就是说在这个任务上，那 0.5s/步的优化窗口性价比很低。

### σ 几乎不随规划器变化

四轮最差 σ 全落在 0.111~0.115，差异远小于选点造成的差异（见下）。
说明在这个任务里**奇异余量由 A/B 选点决定，不由规划器决定** ——
想要更大的奇异余量应该去调 `transport_pick_xyz`，调规划器没用。

### A/B 两点是扫出来的，不是取出来的

`transport_probe` 场景（`scenarios:=transport_probe`，不动机器人，只出表）
用与真跑**同一套判据**扫了 80 个候选，50 个可用。关键几行：

```
(0.249, 0.464, 0.861) 抓取 σ=0.0091 κ=198  -> 不可用   <- 先前第2步失败的点
(0.249, 0.384, 0.811) 抓取 σ=0.0021 κ=876  -> 不可用   <- 几乎精确奇异
(0.149, 0.464, 0.911) 抓取 σ=0.1116        -> 可用，选作 A
(0.149, 0.284, 0.911) 抓取 σ=0.1598        -> 可用（全场最佳），选作 B
```

规律：**z 越低、y 越大 = 手臂越伸展 = σ 越小**，一路掉进奇异。

### 调通 transport 一共踩了三个独立的坑

三个都不是"参数调不对"，而是三个不同层面的机制，且**每一个的报错都指不到根因**：

1. **腕部碰撞球**：TCP(`tool_link`) 无碰撞几何，但它与 `link_7`(sphere r=0.05)、
   `link_6`(cylinder r=0.04) 的原点**完全重合**（URDF 里两个关节 origin 都是 `0 0 0`）。
   TCP 离物体顶面 0.03m 时腕部球已嵌入 0.02m。报错：`RETRIES_EXHAUSTED`。
   现在 `planning_demo_node` 会用 `RobotModel` 现算这个下限并带数字拒绝。
2. **奇异**：改对高度后 IK 有解、不碰撞，但被本工程的奇异监视器否掉
   （`sigma_min=0.0161 < 0.02`）。报错还是 `RETRIES_EXHAUSTED` —— 与上一个坑
   长得一模一样，这就是为什么必须有 `transport_probe` 把三项分开报。
3. **没等静止**：控制器报 `successfully finished` 时手臂还在收敛，
   紧接着规划下一步就拿到一个移动中的起点，0.53s 后下发时真实关节已经走远：
   `Invalid Trajectory: start point deviates ... expected: -0.963636, current: -1.02968`
   （0.066rad > 默认 `allowed_start_tolerance` 0.05），报出来是 `MoveItErrorCode=-4`。
   修法是 `execution.settle_*` 一族参数（执行后等静止），
   **不是**放大 `allowed_start_tolerance`（那等于把错误起点合法化）。

### 边界（如实说明）

- 本机**无夹爪关节**（两臂止于 `link_7`，之后只有无碰撞体的 `tool_link`，
  SRDF 刻意不声明 `end_effector`）。所以这是**纯运动学演示**：
  动作序列、碰撞校验、轨迹执行都是真的，物体是规划场景里的真实碰撞体并参与避障，
  但物体不会在 Gazebo 里被夹住跟着走。
- `transport_probe` 的碰撞场景只含机器人自身 + 那个物体，不含 Gazebo 环境。
  对本 demo 够用（物体是唯一环境障碍），但不能当成"真跑一定不碰"的证明。


## 实测结果（Gazebo 仿真，全链路执行，2026-08-21）

一次 `ros2 launch astribot_s1_manipulation planning_demo.launch.py execute:=true`
（3 个场景 + 2 个预备动作 + 3 规划器对比）的完整输出摘要：

```
预备动作 [arm_left  -> ready]: SUCCESS   trajectory executed successfully on group 'arm_left'
预备动作 [arm_right -> ready]: SUCCESS   trajectory executed successfully on group 'arm_right'

---------- 场景 [single_arm_named] 结果 ----------
错误码: SUCCESS (成功), 尝试次数: 1
trajectory executed successfully on group 'arm_left'

---------- 场景 [closed_chain] 结果 ----------
错误码: SUCCESS (成功), 尝试次数: 1
闭链残差(全轨迹最差): 位置 0.000293 m, 姿态 0.000366 rad -> 满足约束
trajectory executed successfully on group 'dual_arm'

---------- 规划器对比 ----------
  RRTstarConfig            成功 | 时长 0.119s | 最大速度 1.5891 rad/s | 最差 sigma_min 0.1369
  BITstarConfig            成功 | 时长 0.136s | 最大速度 1.0821 rad/s | 最差 sigma_min 0.1369
  InformedRRTstarConfig    成功 | 时长 0.136s | 最大速度 1.0809 rad/s | 最差 sigma_min 0.1369

demo 全部场景执行完毕，退出码 0
```

同一次运行的 move_group 侧规划耗时（`allowed_planning_time` 是 5.0s）：

| 规划器 | 优化前 | 优化后 |
|---|---|---|
| RRTstarConfig | 5.001 s | 0.512 ~ 0.558 s |
| InformedRRTstarConfig | 5.001 s | 0.512 s |
| BITstarConfig | 0.015 s | 0.014 s（本来就没问题，刻意不配优化窗口）|

以及日志噪声：

| 现象 | 优化前 | 优化后 |
|---|---|---|
| `PathLengthDirectInfSampler: There must be at least 1 start ...` | 13 条 / 次 InformedRRT* 规划 | 0 |
| `unknown goal response, ignoring...` | 6 ~ 13 条 / 次运行 | 0 |
| 每跑一次 demo 泄漏的 move_group 进程 | +1 | 0 |

执行侧的物理验证（闭链场景，`ros2 topic echo /joint_states`）：

```
arm_left : +0.3999 +0.6999 +0.3999 +1.3000 +0.1000 +0.0000 +0.0000   <- 就是下发的 leader 目标
arm_right: +0.8249 +0.8929 +0.4244 +1.4926 +0.1156 +0.0767 +0.4265   <- 没人直接下发过
```

`arm_right` 这组值不是任何地方配置的目标，它完全由闭链约束经逐点 IK 投影算出 ——
这是"两臂真的被同一个相对位姿约束着"的直接证据。

> 尚未独立验证的一项：**执行过程中间路点**的 T_rel。
> 现在的证据链是「最终时间参数化轨迹的每个路点残差 <= 0.29mm」+「执行终点与
> 规划终点一致到 1e-4 rad」。中间过程另外用 TF 采样验证的尝试没成功
> （采样脚本读到的 `astribot_torso_base -> astribot_arm_left_tool_link`
> 全程不变，与 `/joint_states` 明显矛盾，怀疑与本仓库已知的动态 TF 读取问题
> 同源，未定位）。要补这一项，建议在 demo 节点内部用 PlanningSceneMonitor
> 的当前状态直接算残差，而不是从外部读 TF。

---

## 实测结果（bare 环境，无 Gazebo，2026-08-20）

```
---------- 场景 [closed_chain] 结果 ----------
错误码: SUCCESS (成功), 尝试次数: 1
关节轨迹: 9 点, 关节数 14
笛卡尔轨迹(leader TCP): 9 个位姿
笛卡尔轨迹(follower TCP): 9 个位姿
  总运动时长: 0.381 s
  速度利用率峰值: 46.6%
  轨迹合法性: 合法
  已采纳 TOTG 优化，节拍缩短 46.1%
闭链残差(全轨迹最差): 位置 0.000293 m, 姿态 0.000366 rad -> 满足约束
奇异余量(全轨迹最差): sigma_min=0.116389, 条件数=15.34

---------- 规划器对比 ----------
  RRTstarConfig            成功 | 时长 0.236s | 6 点 | 速度利用率 25.7%
  BITstarConfig            成功 | 时长 0.236s | 6 点 | 速度利用率 25.7%
  InformedRRTstarConfig    成功 | 时长 0.236s | 6 点 | 速度利用率 25.7%
```

闭链残差 0.29mm / 0.00037rad，远优于 5mm / 0.02rad 的阈值 —— 这是
"逐点 IK 投影"相比"两条独立单臂规划"的直接差别（后者残差会一路漂移到厘米级）。

---

## 工具：重新生成 SRDF 的碰撞关闭列表

SRDF 里 42 条 `disable_collisions` 不是手填的，是推导 + 实测的产物：
27 条相邻对由 URDF 树确定性推导，另 15 条由实测得到。

漏了会怎样：起始构型直接判自碰撞，OMPL 报
`There are no valid initial states!` → `Unable to solve the planning problem`，
现场看起来像"规划器不行"，实际是起点非法、规划器根本没机会跑。

重新生成：

```bash
ros2 run astribot_s1_manipulation self_collision_pair_generator 10000
```

输出到 stdout（不带日志前缀，可直接复制）。三类结果：
- `ALWAYS` → `reason="Default"`，**必须关**
- `NEVER` → `reason="Never"`，关掉纯为省开销
- `有时碰撞` → **绝对不要关**，必须留给运行时检测（只以注释形式列出）

---

## 代码结构

```
include/astribot_s1_manipulation/
├── error_codes.hpp                 统一错误码 + isRetryable（区分该不该重试）
├── singularity_monitor.hpp         雅可比 SVD 奇异检测（sigma_min + 条件数双判据）
├── closed_chain_constraint.hpp     T_rel / 残差 / follower IK 投影
├── collision_validator.hpp         自碰撞 + 臂-底盘 + 环境碰撞，分别返回错误码
├── trajectory_metrics.hpp          节拍量化 + 限位合法性判定
├── trajectory_time_optimizer.hpp   IPTP vs TOTG + 超限回退
├── optimizing_planner_wrapper.hpp  ★ 渐进最优规划器的收敛策略包装（header-only 模板）
└── dual_arm_planner.hpp            对外总接口（单臂 / 闭链 / 执行）
src/
├── ompl_planner_extension.cpp      ★ 注册 BIT* / Informed RRT* 的 PlannerManager 插件
├── planning_demo_node.cpp          demo：四个场景 + 全量指标输出
└── self_collision_pair_generator.cpp  SRDF 碰撞对生成工具
```

规划流程（`planClosedChain`）：

```
捕获 T_rel  →  leader 单臂 OMPL 规划(7维)  →  加密路径  →  逐点 follower IK 投影
   →  逐点校验(闭链残差 / 碰撞 / 奇异)  →  合并成 14 维轨迹
   →  时间参数化 + 节拍优化  →  在最终轨迹上复核闭链残差  →  输出
```

最后那步"复核"不能省：TOTG 会重采样路径点，插出来的新点未必满足闭链约束。

## 排查手册：规划很慢 / 日志刷 ERROR / 执行报 -7

### 规划每次都恰好用满 allowed_planning_time

RRT* / Informed RRT* 是**渐进最优**规划器，只在 PTC 成立或解的代价达到
optimization objective 的 cost threshold 时才返回。MoveIt 默认用
`PathLengthOptimizationObjective`、threshold = 0（日志里那句
`Seeking a solution better than 0.00000`），永远达不到 —— 唯一出口就是超时。

本工程在 `ompl_planning.yaml` 里给这类规划器配了扩展键
`optimization_budget_sec`（首解出现后再优化多少秒），由
`OptimizingPlannerWrapper` 实现。默认 0.5s，实测把 5.001s 压到 0.51s。
置 0 或删掉该键 = 退回原生行为。

**不要**给 BIT* 配这个键：它有自己的批次收敛判据，实测 0.014s 就返回，
配了反而会被拖到 0.5s。

MoveIt 官方也有 `termination_condition: CostConvergence[w,eps] | ExactSolution
| Iteration[n]` 可用（本包原样透传），但治不了这个场景：CostConvergence 要靠
**发现更好的解**推进计数，首解即最优的查询计数停在 1，永不收敛；
ExactSolution 一有可行解就停，等于放弃渐进最优性。细节见
`include/astribot_s1_manipulation/optimizing_planner_wrapper.hpp` 文件头。

### 日志刷 `PathLengthDirectInfSampler: There must be at least 1 start and and 1 goal state`

informed 采样器构造时就要求 pdef 里已有 >=1 个目标状态，而 MoveIt 的目标是
`ob::GoalLazySamples`（后台线程陆续产出）。线程刚 start 还没排上 CPU 时状态数为 0，
于是抛异常，并沿 MoveIt 的整条 request-adapter 链层层重抛（实测 13 条 / 次规划）。
它会自愈，但真正的规划失败会被埋在这堆 ERROR 里看不见。

解法：`InformedRRTstarConfig` 里配 `goal_state_wait_sec: 0.1`，
包装层在委托 `solve()` 前有界等待第一个目标状态。实测 13 条 -> 0 条。

### 日志刷 `unknown goal response, ignoring...`，执行返回 MoveItErrorCode=-7

**先数一下 move_group 进程有几个**：

```bash
ps aux | grep -c '[m]ove_group --ros-args'
```

大于 1 就是这个原因。多个 move_group = 多组同名 action server
（`move_action` / `execute_trajectory`）互相抢答，客户端匹配不上自己的 goal；
更糟的是**其中一个 move_group 把轨迹发给了控制器、机器人真的动了，另一个稍后
才做起点校验**，此时机器人已经离开规划起点，于是报

```
Invalid Trajectory: start point deviates from current robot state more than 0.05
joint 'astribot_arm_left_joint_1': expected: 0.399907, current: 0.267065
```

并返回 ABORTED / MoveItErrorCode=-7 (CONTROL_FAILED)。极易被误判成"执行链路有
并发 bug"，实际只是进程泄漏。

泄漏来源：demo 是一次性任务，跑完就退出，但 launch 里其他节点没有退出条件，
`ros2 launch` 就一直挂着。`planning_demo.launch.py` 已经加了
`OnProcessExit -> Shutdown`，demo 节点退出即关闭整个 launch。
手写 launch 时务必照做，否则每跑一次泄漏一个 move_group。

清理：

```bash
ps aux | grep -E "[m]ove_group --ros-args|[p]lanning_demo" | awk '{print $2}' | xargs -r kill
```

（注意别用 `pkill -f <工作区路径>`：move_group 的可执行文件在
`/opt/ros/humble/lib/` 下，按工作区路径匹配抓不到它。）

---

## 错误码

| 错误码 | 含义 | 可重试 |
|---|---|---|
| `SUCCESS` | 成功，且已通过限位/碰撞/奇异/闭链四项校验 | — |
| `INVALID_INPUT` | 入参非法（空组名、维度不符、目标超限位） | 否 |
| `PLANNING_GROUP_NOT_FOUND` | SRDF 里没这个组 | 否 |
| `NOT_CONFIGURED` | 没 initialize 就调规划接口 | 否 |
| `PLANNER_FAILED` | OMPL 无解或超时 | **是** |
| `IK_FAILED` | follower IK 无解（闭链目标超出可达空间） | **是** |
| `CLOSED_CHAIN_RESIDUAL_TOO_LARGE` | 闭链残差超阈值 | **是** |
| `SINGULAR_CONFIGURATION` | 轨迹含奇异构型 | **是** |
| `SELF_COLLISION` / `ENVIRONMENT_COLLISION` | 碰撞 | **是** |
| `TIME_PARAMETERIZATION_FAILED` | IPTP/TOTG 都失败 | 否 |
| `JOINT_LIMIT_VIOLATION` | 轨迹超硬限位（此时**不输出轨迹**） | 否 |
| `RETRIES_EXHAUSTED` | 重试用尽 | 否 |
| `EXCEPTION_CAUGHT` | 捕获到第三方库异常（已记日志，不上抛） | 否 |

失败时 `PlanResult::trajectory` 一定为空 —— 任何失败都不会输出半成品轨迹。
