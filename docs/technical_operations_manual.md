# Astribot S1 双臂轮式机器人 —— 完整技术操作手册

> 归档版本：2026-08-20 · 分支 `chassis-effort-drive` · 基线提交 `8a3c3a9`
>
> 本手册是**全栈总入口**：架构、数据流、算法与参数、部署、配置、运行、就绪校验、故障定位。
> 深水区细节不在本手册内重复，而是逐处给出定位链接（见 §0.2 文档地图）。

---

## 0 · 使用本手册之前

### 0.1 适用范围与真值源规则

本手册覆盖 `ws_robot/` 下 13 个包构成的完整栈：感知 → 建图 → 自主决策 → 导航 → 运动规划 → 厂商 SDK 桥接。
不覆盖厂商 SDK 内部实现（`astribot_sdk/`，闭源二进制）。

**真值源规则（与 `sim_real_alignment.md §0` 同一条，全栈适用）：**

| 事实类别 | 唯一真值源 | 不可作为依据的来源 |
|---|---|---|
| SDK 接口行为 | `examples/` 下的源码 | 厂商文档描述、README 叙述 |
| 关节限位 / 原点 | 厂商 **per-part** yaml 的 `model` 字段 | `whole_body_*.urdf`（SDK 运行时并不加载）|
| 数值阈值 | 对应的 `config/*.yaml` | 代码里的 `declare_parameter` 默认值、注释里的历史值 |
| 几何 / 碰撞 | 活的 URDF（xacro 展开后）| 硬编码常量、注释中的旧尺寸 |

厂商 ship 了**四份互不一致**的模型。抄错的后果已实测：躯干速度限位宽 3.3 倍（不安全方向）、
左臂 `joint_3` 白丢 1.7 rad 行程。`astribot_s1_description` 里有哨兵测试挡这个错误 —— 但它只守 URDF，
见 §5.4 与附录 B 的未闭合项。

### 0.2 文档地图：什么问题该查哪一份

本手册是总入口，深水区不重复。**数字只在一处维护**，其余地方给链接 —— 复制过的数字一定会漂。

| 你的问题 | 查这里 |
|---|---|
| 全栈怎么跑起来、坏了怎么查 | **本手册** |
| 某个设计为什么是这样、某个方案为什么被否 | [`sim_real_alignment.md`](sim_real_alignment.md) |
| 实机（Orin / aarch64）怎么部署 | [`real_robot_deployment.md`](real_robot_deployment.md) |
| 多层切片 / SLAM / Nav2 参数逐项 | [`astribot_s1_perception/README_PERCEPTION.md`](../ws_robot/src/astribot_s1_perception/README_PERCEPTION.md)、[`README_NAVIGATION.md`](../ws_robot/src/astribot_s1_navigation/README_NAVIGATION.md) |
| 探索协调器状态机与验收 | [`astribot_s1_autonomy/README.md`](../ws_robot/src/astribot_s1_autonomy/README.md) |
| OMPL 规划器 / 闭链约束 | [`astribot_s1_manipulation/README.md`](../ws_robot/src/astribot_s1_manipulation/README.md) |
| 桥接层接口与硬边界 | [`astribot_trajectory_bridge/README.md`](../ws_robot/src/astribot_trajectory_bridge/README.md) |

⚠️ 已知**文档滞后于代码**的位置（读旧文时要提防），逐条见 §5.4 与附录 B。

### 0.3 本手册中"已验证"的含义

三档，混用会出事，所以逐处标注：

| 标记 | 含义 | 可信度 |
|---|---|---|
| ✅ **实测** | 在活的系统上跑出来，有数字 | 可作为判据 |
| 🟡 **仅离线** | 单元测试通过，未在真后端跑过 | 不能推断在线行为 |
| ⬜ **未验证** | 代码存在，从未运行 | 不可作为依据 |

后端也必须一起标注 —— **本栈全部在线数据来自仿真（Gazebo 或厂商 MuJoCo），真机侧为零。**
仿真通过的结论不能外推到真机，理由见 [`real_robot_deployment.md §0.2`](real_robot_deployment.md)。

---

## 1 · 系统整体架构

### 1.1 分层视图

```
┌─ 决策层 ────────────────────────────────────────────────────────┐
│  exploration_coordinator_node   前沿探索协调（严格时序 + 未知区禁行） │
│  planning_demo_node             搬运 / 移动搬运场景编排              │
└───────────────┬─────────────────────────────┬────────────────────┘
                │ NavigateToPose              │ MoveGroup / 桥接
┌───────────────▼──────────────┐  ┌───────────▼────────────────────┐
│ 导航层  Nav2                  │  │ 规划层  MoveIt2 + OMPL          │
│  planner / controller / bt    │  │  move_group、闭链约束、奇异监视  │
└───────────────┬──────────────┘  └───────────┬────────────────────┘
                │ /cmd_vel_pre_arm_coupling   │ JointTrajectory
┌───────────────▼──────────────┐              │
│ 限速层（两级串联，见 §3.4）    │              │
│  arm_speed_limiter_node       │              │
│  arm_chassis_speed_coupling   │              │
└───────────────┬──────────────┘              │
                │ /cmd_vel                    │
┌───────────────▼─────────────────────────────▼────────────────────┐
│ 执行层（二选一，由 launch 环境决定，不由代码 if-else 决定）          │
│  A. Gazebo    omni_effort_drive_node → wheel_effort_controller    │
│  B. 厂商 SDK  astribot_trajectory_bridge → MuJoCo / 真机           │
└──────────────────────────────────────────────────────────────────┘
        ▲
┌───────┴──────────────────────────────────────────────────────────┐
│ 感知层  雷达 → 预处理 → 融合 → 自滤 → 多层切片 → /scan_from_cloud   │
│         另一条单层链 → /scan → slam_toolbox → /map + map→odom      │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 两套工作空间与两套后端

**两套工作空间，别 source 错：**

| 路径 | 内容 | 状态 |
|---|---|---|
| `ws_robot/install/` | **本栈 13 个包** | ✅ 唯一正确的 overlay |
| `install/`（仓库顶层）| 只有 `astribot_s1_manipulation`、`astribot_s1_navigation` | ⚠️ 2026-08-21/24 的**陈旧残留**，不要 source |
| `ws_robot/src/astribot_s1_description/urdf/install/` | 嵌套在源码树里的构建产物 | ⚠️ 误同步产物，见 §8.2 |

**两套执行后端，同一份上层代码：**

| 后端 | 谁提供物理 | 谁提供 `/joint_states` | 用途 |
|---|---|---|---|
| **Gazebo（ros2_control）** | Ignition/GZ | `joint_state_broadcaster` | 导航、感知、探索、SLAM |
| **厂商 MuJoCo（SDK）** | 厂商仿真 | `astribot_trajectory_bridge` | 控制通路对齐、夹爪、真机预演 |

⚠️ **两个后端不能同时开** —— 都发 `/joint_states`，两个发布者并存时 TF 抖动而**两边都不报错**。

### 1.3 包清单与职责边界

| 包 | 可执行 | 职责 |
|---|---|---|
| `astribot_bridge_msgs` | （接口）| 桥接层 msg/srv 定义，全栈最底层依赖 |
| `astribot_s1_description` | （模型）| URDF/xacro **模型真值源** + 限位一致性哨兵测试 |
| `astribot_s1_perception` | `livox_preprocess_node`、`livox_fusion_node`、`map_domain_relay`、`map_start_cell_check`、`autonomous_patrol_node` | 雷达预处理/融合、SLAM 配置、地图来源仲裁 |
| `astribot_s1_autonomy` | `pointcloud_slice_scan_node`、`frontier_explorer_node`、`exploration_coordinator_node` | 多层切片+自滤、前沿检测、探索协调 |
| `astribot_s1_navigation` | `cmd_vel_body_to_world_node`、`arm_speed_limiter_node`、`path_tracking_diagnostics_node` | Nav2 配置、体系→世界系转换、限速第一级、路径跟踪诊断 |
| `astribot_s1_dynamics_coupling` | `arm_chassis_speed_coupling_node` | 限速第二级：按机械臂**水平伸展**缩底盘速度 |
| `astribot_s1_chassis_effort_drive` | `omni_effort_drive_node` | Gazebo 侧力矩驱动（X 型全向轮，**非麦轮**）|
| `astribot_s1_moveit_config` | （配置）| SRDF、OMPL、kinematics、`joint_limits.yaml` |
| `astribot_s1_manipulation` | `planning_demo_node`、`self_collision_pair_generator` | MoveIt 客户端、搬运场景、闭链/奇异工具 |
| `astribot_trajectory_bridge` | `bridge_container`、`state_bridge_node`、`joint_map_probe` | 厂商 SDK 桥接（臂/底盘/夹爪）+ 关节顺序探针 |
| `astribot_s1_gazebo_bringup` | （launch/配置）| Gazebo 世界、`ros_gz` 桥、四组控制器 |
| `aws-robomaker-small-warehouse-world` | （资源）| 仿真仓储世界 |
| `livox_ros_driver2` | `livox_ros_driver2_node` | Livox MID360 驱动（硬件分支，⬜ 仅编译验证）|

### 1.4 sim / real 切换的唯一开关

**硬规则：业务代码内部不允许出现 `if sim / else real`。** 切换只由 launch 参数与环境变量完成。

| 维度 | 开关 | 取值 |
|---|---|---|
| 仿真 / 硬件雷达 | `env:=` | `sim` \| `hardware` |
| 建图 / 定位 | `mode:=` | `mapping` \| `localization` |
| 地图来源 | `map_source.yaml` → `map_source` | `sim_slam` \| `real_file` \| `real_live` |
| `map→odom` 提供者 | `map_source.yaml` → `localization` | `slam` \| `ground_truth` |
| 控制器 | `controller_plugin:=` | `mppi` \| `rpp` |
| 桥接目标 | `bridge_bringup.launch.py` → `target:=` | 见 §6.6 |
| 时钟 | `use_sim_time` | 仿真 `true` / 实机 `false` |

`map_source` × `localization` **只有三种合法组合**，其余在启动时被显式拒绝（不是静默降级）：

| 组合 | 用途 |
|---|---|
| `sim_slam` + `slam` | **探索算法只能在这个组合下开发** —— 静态地图里没有前沿 |
| `real_file` + `ground_truth` | 在真机产出的地图上验证规划/导航/搬运 |
| `real_live` + `ground_truth` | 同上，地图跟着真机实时更新 |

被拒的两种及原因：`sim_slam + ground_truth` 会让一个子帧有两个父源，位姿反复跳而**症状看起来像"定位漂移"**，极难归因；
`real_* + slam` 下扫描匹配要求仿真几何与地图一致，Gazebo 跑 AWS 仓库而地图来自真实场地，**定位必然发散**。

---

## 2 · 端到端数据流向

### 2.1 感知链：两颗雷达 → 多层切片 → `/scan`

```
[sim]  /livox_mid360_left/points ─┐        [hw]  /livox/lidar_left ─┐
       /livox_mid360_right/points ┘              /livox/lidar_right ┘
                    │
                    ▼  livox_preprocess_node ×2
       range_min 0.35（**3D 球面距离** √(x²+y²+z²)）、ground_z_min -0.18
                    │
       /livox/left/cloud_filtered  +  /livox/right/cloud_filtered
                    │
                    ▼  livox_fusion_node
              /livox/fused_points
                    │
                    ▼  pointcloud_slice_scan_node   ← 自滤在这里，全栈唯一一处
         ┌──────────┴───────────┐
         ▼                      ▼
  /livox/cloud_self_filtered   /scan_from_cloud（4 层切片跨层取最近）
         │                            │
         ▼  pointcloud_to_laserscan   ▼
      /scan（**单层** 0.05~0.6 m）   Nav2 costmap
         │
         ▼  slam_toolbox
     /map + TF map→odom
```

⚠️ **这条链最容易误解的一点：多层切片只进了 Nav2 代价地图，没进 SLAM。**
`slam_toolbox` 订阅的是单层 `/scan`。后果：矮托盘（离地 5~25 cm）和悬空横梁
**在代价地图里有、在 SLAM 地图里没有**。这是刻意的（见附录 B-1），不是缺陷遗漏。

**两处 `range_min` 必须同为 0.35，口径却不同** —— 这是本仓库花了几层才定位的一个坑：
`livox_preprocess_node` 按**完整 3D 球面距离**过滤，而 `pointcloud_to_laserscan` 投影成 2D 时
按**纯 XY 平面距离**。一个点 3D 距离刚过 0.35 m，去掉 z 分量后 XY 距离可能只剩约 0.15 m，
照样投进 `/scan`。表现为规划器持续报 `Starting point in lethal space`，且这个自撞 blob
**跟着机器人走** —— 看起来像 SLAM 地图里烧进了幽灵点，实际是机器人实时把自己当障碍。

### 2.2 建图与定位链

| 环节 | 提供者 | 关键值 |
|---|---|---|
| `/scan` | `pointcloud_to_laserscan` | `min_height 0.05`、`max_height 0.6`、`range_min 0.35`、`range_max 20.0`、`angle_increment 0.0087`（0.5°，约 720 束）|
| `/map` + `map→odom` | `slam_toolbox` | `resolution 0.05`、`max_laser_range 20.0`、`base_frame astribot_torso_base` |
| `odom→astribot_torso_base` | Gazebo / SDK | —— |

`mapping` 与 `localization` 两份配置的**唯一功能差异**只有三项：`mode`、`map_file_name`、`map_start_pose`。
其余匹配器参数逐字相同。

⚠️ **`map_file_name` 不是栅格地图。** 它是 `slam_toolbox` 自己序列化的位姿图基础文件名（不带扩展名），
对应 `<name>.data` + `<name>.posegraph` 一对文件，**不是** `nav2_map_server` 那种 `.pgm` + `.yaml`。

### 2.3 决策链：前沿探索协调器

```
/map ──► frontier_explorer_node ──► 前沿簇 + 候选站位点
                                          │
global_costmap ────────────────────────────┤（第二层校验）
                                          ▼
                              exploration_coordinator_node
                                  严格时序：同时只有一个目标在飞
                                          │ NavigateToPose
                                          ▼
                                        Nav2
```

**双图分工是硬规则，不能互换：`/map` 找前沿，`global_costmap` 判可站。**
换数据源时距离阈值也要跟着换口径 —— 实测过的三个错值：`0.42`（重复计足迹）、
`0.0`（导航超时）、`0.25`（= `xy_goal_tolerance`，正确）。

### 2.4 导航链：Nav2 → 两级限速 → 底盘执行

这条链的话题名经过多次重映射，**不看 launch 无法还原**，而中间任何一环缺失都表现为"机器人不动"：

```
controller_server（内部 cmd_vel，车体系）
        │ 重映射
        ▼
/cmd_vel_nav_body_raw
        │ velocity_smoother
        ▼
/cmd_vel_nav_body（仍是车体系）
        │ cmd_vel_body_to_world_node  ← 车体系 → world 系换算，**必须存在**
        ▼
/cmd_vel_pre_arm_coupling        （enable_arm_chassis_coupling:=false 时这里直接就是 /cmd_vel）
        │ arm_chassis_speed_coupling_node  ← 限速第二级
        ▼
    /cmd_vel
        │
        ├─[Gazebo]  omni_effort_drive_node → /wheel_effort_controller/commands
        └─[SDK]     astribot_trajectory_bridge → 底盘积分下发
```

**为什么必须有 `cmd_vel_body_to_world_node`：** Nav2 标准控制器按**车体系**发布，
而底盘的 gz-sim `VelocityControl` 插件按 **world 系**解释 `/cmd_vel`。直接对接会重现
"原地打转不挪窝"这个 bug。所以"机器人只自转不平移"的**第一个**排查项永远是这个节点。

**两级限速是两套独立机制，但效果相乘：**

| 级 | 节点 | 机制 | 所在包 |
|---|---|---|---|
| 第一级 | `arm_speed_limiter_node` | 发 Nav2 `/speed_limit`（`percentage=true`），二值判断 | `astribot_s1_navigation` |
| 第二级 | `arm_chassis_speed_coupling_node` | 直接缩放 `/cmd_vel` 分量，连续 | `astribot_s1_dynamics_coupling` |

⚠️ 实测 `0.50 × 0.15 = 0.075` —— **两层串联叠乘**。两个包各有一层，
只改一个会看不到效果，这曾被误判为"限速没生效"。

### 2.5 操作链：MoveIt2 → 桥接 → SDK

### 2.6 夹爪链

### 2.7 状态与诊断回流

### 2.8 全局时序与时钟

---

## 3 · 各模块关键算法与参数

### 3.1 感知：多层点云切片与自滤

**算法（`pointcloud_slice_scan_node`，C++）：**
1. 融合点云变换到 `base_frame`（`astribot_torso_base`）；TF 超时 `0.05 s` 即丢帧等 TF 恢复。
2. 时间一致性检查：`max_cloud_age_sec 0.30`、`tf_time_tolerance_sec 0.10` —— 防时间错位造成**假障碍**。
3. 降采样与去噪：`voxel_leaf_size 0.03`、统计离群点 `outlier_mean_k 12` / `outlier_stddev_mul 2.0`。
4. **自滤**：按 URDF 链生成胶囊体（圆柱 + 两端半球），剔除落在体内的点。
5. **四层高度切片**，每层独立成 2D 扫描后**跨层取最近距离**，合成 `/scan_from_cloud`。

**四层定义（z 相对 `astribot_torso_base`）：**

| 层 | `z_min` | `z_max` | 离地范围 | `min_points` | `max_range` | 目的 |
|---|---|---|---|---|---|---|
| `low_obstacle` | −0.03 | 0.17 | 约 5~25 cm | 2 | 6.0 | 矮托盘、地面杂物 |
| `main_nav` | 0.17 | 0.60 | 25~68 cm | —— | —— | 主导航层：墙面、货架立柱 |
| `torso_high` | 0.60 | 1.10 | 68~118 cm | —— | —— | 躯干高度障碍 |
| `overhead` | 1.10 | 1.55 | 118~163 cm | —— | —— | 悬空横梁 |

`low_obstacle` 的 `min_points: 2` 是刻意放宽的：这一层的误报是"幽灵障碍物"会无谓挡路，
而地面反光点恰好集中在这一层。

**自滤链（`chain_names`，6 条）：**

| 链 | `radius` |
|---|---|
| `footprint`（竖直圆柱，非胶囊）| 0.42（`z_min −0.20` / `z_max 0.05`）|
| `torso` | 0.20 |
| `head` | 0.18 |
| `arm_left` / `arm_right` | 0.15 |
| `gripper_left` / `gripper_right` | 0.10（**几何算出来的**：逐 link 量碰撞几何顶点到 link 轴的距离）|

`radius` 调参原则（yaml 原话）：**宁可略大也不要过小** —— 漏剔会让机械臂在 costmap 里变成障碍；
但 `radius` 超过 `0.3` 会开始吃掉手臂旁边的真实障碍物。

⚠️ **改 URDF 必须同步改 `chain_names`/`chains`。** 夹爪曾整个不在自滤链里，后果链条是：
机器人把指尖当障碍 → `Starting point in lethal space` → 探索 **0 次派发**。
且 SLAM 地图**有记忆**，幽灵点必须重新建图碾掉，改完配置不会自动消失。

**故障策略（不静默失败）：** `min_valid_points: 20` —— 剔除自身点后不足该数，
本帧判为**不可信**，而不是判为"前方无障碍"。`invalid_input_policy: hold_last`。

### 3.2 自主：前沿检测与路径校验

**探索协调器的两条硬规则：**
1. **同时只有一个目标在飞** —— 状态序列必须是
   `IDLE → GEN_NEXT_POINT → VALIDATING → NAVIGATING → ARRIVED → GEN_NEXT_POINT → …`，
   两个相邻 `NAVIGATING` 段之间**必须**有 `ARRIVED`。
2. **绝不踏入未知区** —— 每次派发前必须先出现 `双层校验通过`。

**双层校验（`/map` 找前沿，`global_costmap` 判可站）：**

| 参数 | 值 | 约束 |
|---|---|---|
| `arrival_xy_tolerance` | 0.30 | 必须 ≥ `xy_goal_tolerance` |
| `goal_clearance_radius` | 0.25 | **必须等于** Nav2 `xy_goal_tolerance`（0.25）|
| `goal_unknown_clearance_radius` | **0.0** | 地图分辨率 0.05 时**必须为 0**，见下 |
| `use_costmap` | true | 必须与 `allow_unknown: false` 同向 |

⚠️ **`goal_unknown_clearance_radius` ≥ 地图分辨率会按定义否掉每一个前沿候选**（实测 6775/6784），
`dispatched` 恒为 0，机器人永远不动。这个症状离根因有三层远。

### 3.3 导航：代价地图与控制器

| 参数 | 值 | 依据 |
|---|---|---|
| `robot_radius` | **0.42** | 真实底盘外接半径 0.386 m（轮心离轴 0.306 + 轮球半径 0.080）+ 余量 |
| `inflation_radius` | **0.65**（两份配置四处一致）| 必须 > 足迹外接半径 0.42 |
| `xy_goal_tolerance` | 0.25 | 与探索器 `goal_clearance_radius` 同口径 |
| 地图分辨率 | 0.05 | —— |

两份配置 `nav2_params_mppi.yaml` / `nav2_params_rpp.yaml` 的差异只在控制器插件及其专属参数；
代价地图几何两份相同。**MPPI 发挥全向能力更好**，是推荐值。

⚠️ **MPPI 足迹代价在窄通道里饱和**：`consider_footprint: true` 时窄于 1.62 m 的通道内代价恒为 253、
零梯度（实测占可行域 35%）。改 `false` 无收益已回退 —— **这张地图上根本测不出差别**，
不要据此结论推广到别的场地。

### 3.4 臂-底盘耦合限速

**两个维度各自独立换算成 0~1 活跃度，取更严格者**（不做加权平均，避免掩盖某一维的剧烈运动），
再换算成限速系数。

**维度一：展开幅度（C1 已修正度量）**

```
activity = clamp( (reach − reach_folded_m) / (reach_full_m − reach_folded_m), 0, 1 )
scale    = 1.0 − activity × (1.0 − min_speed_scale)
```

| 参数 | 值 | 依据 |
|---|---|---|
| `extension_metric` | `horizontal_reach` | 旧值 `joint_deviation` 已证伪，保留仅供一键回退 |
| `reach_folded_m` | 0.42 | = Nav2 `robot_radius`：臂还在足迹内时规划器已算进去了 |
| `reach_full_m` | 0.8865 | = 实测全工作空间最大水平伸展 |
| `min_speed_scale` | 0.15 | 限速系数下限 |
| `reach_update_period_sec` | 0.05 | TF 查询节流 20 Hz |

**为什么旧度量必须废弃（数据全部由活的 URDF 采样，非估算）：**
- 全 0 姿态（原当"收纳基准"）实际水平伸展 **0.4205 m 且肘部完全伸直**，却因偏差 = 0 而完全不限速；
- 真实收纳姿态伸展仅 **0.3532 m**，反而被限到系数下限；
- 4000 次全工作空间采样：伸展 0.8865 m 时偏差 3.062，伸展 0.1997 m 时偏差 3.079 ——
  **偏差几乎相同，伸展差 4.4 倍**，说明旧度量根本不携带伸展信息。

修正效果：`ready` 姿态限速系数从 0.292 → **> 0.85**，导航耗时 78.1 s → **13.4 s**。
测试里有一条哨兵 `test_old_metric_was_anti_correlated_on_measured_poses`，
专门在有人把默认度量改回去时提醒他旧问题依然存在。

**维度二：运动速率（C1 未改动）** —— `velocity_full_rad_s: 2.0`，取各关节**最大值**而非求和。
手臂高速运动的反作用力矩是独立且合理的约束，与"参考姿态"无关。

**降级行为：** `joint_state_timeout_sec 0.5` 超时 → `degraded_scale 0.3`（保守限速，不是放行）。

### 3.5 底盘力矩驱动与运动学

### 3.6 运动规划：OMPL 与闭链约束

**三条会浪费大量时间的前提（都是实测踩过的）：**

**① 全零构型是奇异构型，不能从它起步规划。**
`joint_4 = 0` 是肘部**完全伸直**，实测 `sigma_min = 0.0077`（阈值 0.02）。
而 `joint_state_publisher` 的默认值和 SRDF 的 `home` 命名状态**恰好都坐在那里**。
所以规划必须从 `ready` 起步；这也是 `allow_singular_start` 与 `move_to_ready_first`（默认 `true`）存在的原因。

**② `planning_attempts` 必须是 1。** > 1 会触发 `ParallelPlan`，
informed 采样器在惰性目标线程产出状态前就抛异常，而 MoveIt 只报
`Unable to solve the planning problem` —— 看不出真因。

**③ MoveIt2 Humble 的官方 OMPL 插件里没有 BIT\* / Informed RRT\*。**
它只注册 25 个规划器。yaml 里写一个**未注册**的名字会**静默回退到组默认**，
所以"我在跑 BIT\*"完全可能实际在跑 RRTConnect。

**验证方法（必须两条都看）：** `move_group` 日志里插件的 `[OK]/[MISSING]` 注册行，
**以及**每次请求的 `planning request: group=… planner_id=…` 行。

**比较规划器耗时前先看优化预算。** "BIT\* 快 14 倍"是 `optimization_budget_sec` 配置差异造成的假象；
在同样"停在首解"的条件下，它真正的优势是**首解节拍好 32%**。
另外 RRT\*/Informed RRT\* 这类优化型规划器**总是用满** `allowed_planning_time`，
检测首解要用 `best cost REAL` 进度属性，**不能用** `pdef->hasExactSolution()`。

**切换规划器：** 命令行 `planner_id:=BITstarConfig`（最常用）、
`manipulation_params.yaml` 的 `planner_id`、或按请求覆盖 `SingleArmPlanRequest::planner_id`。

### 3.7 TCP 与夹爪几何（抓取余量的可信下限）

⚠️ **`tool_link` 没有碰撞几何，但它与 `link_7` 的 `r = 0.05` 球**原点完全重合**。**
所以"TCP 贴着物体"在几何上必然碰撞，而它报的 `RETRIES_EXHAUSTED`
与"奇异被否"是**完全相同的错误码** —— 光看错误码分不出这两件事。

**夹爪存在一处 7.0 mm 的两源不确定度**（MJCF `0.04125, 0, 0.036379` vs SDF `0.036, 0, 0.031749`，
当前取 MJCF）。这意味着**指尖沿抓取方向有约 7 mm 不确定度**：
**抓取余量小于这个数时，结论不可信。** 测试里 `test_l11_cross_source_delta_is_pinned` 把它钉住了。

厂商没有任何单一文件同时给出夹爪的几何和耦合关系，本包混用
MJCF（几何/耦合）+ `whole_body_with_gripper.sdf`（碰撞 mesh/基座装法），
所以模型测试校验的是"能证明没抄错"的性质（TCP 偏置、平行夹爪不变量 `R(−y,q)·R(+y,q) = I`、
两源交叉校验、腕部质量守恒、夹爪确实进了碰撞模型），**不是逐字节比对**。

### 3.8 轨迹桥接：状态机与写入闸门

### 3.9 夹爪：命令空间与标定

---

## 4 · 环境部署操作

### 4.1 系统前置条件

### 4.2 Python 依赖与版本钉子

### 4.3 厂商 SDK 环境：`env.sh` 逐项说明

### 4.4 编译

### 4.5 实机部署差异

---

## 5 · 配置修改指引

### 5.1 配置文件总索引

**硬规范：所有阈值/频率/超时一律放 yaml，禁止硬编码。**

| 文件 | 管什么 |
|---|---|
`astribot_s1_autonomy/config/pointcloud_slice_scan_params.yaml` | 四层切片 z 范围、自滤链与半径、滤波、看门狗 |
`astribot_s1_autonomy/config/frontier_explorer_params.yaml` | 前沿检测、候选采样、权重 |
`astribot_s1_autonomy/config/exploration_coordinator_params.yaml` | 状态机时序、双层校验口径 |
`astribot_s1_navigation/config/nav2_params_mppi.yaml` / `_rpp.yaml` | 代价地图、控制器 |
`astribot_s1_perception/config/pointcloud_to_laserscan_params.yaml` | 单层 `/scan`（喂 SLAM）|
`astribot_s1_perception/config/mapper_params_online_async.yaml` / `_localization.yaml` | slam_toolbox |
`astribot_s1_perception/config/map_source.yaml` | 地图来源 × 定位来源（三种合法组合）|
`astribot_s1_perception/config/autonomous_patrol_params.yaml` | 巡游（与 Nav2 互斥）|
`astribot_s1_dynamics_coupling/config/*.yaml` | 臂-底盘耦合限速 |
`astribot_s1_moveit_config/config/*.yaml` | SRDF、OMPL、kinematics、`joint_limits.yaml` |
`astribot_s1_manipulation/config/manipulation_params.yaml` | 规划器选择、场景坐标 |
`astribot_trajectory_bridge/config/arm_bridge.yaml` | 桥接状态机阈值、夹爪 |

### 5.2 高频修改场景

| 想改什么 | 改哪里 | 必须同步 |
|---|---|---|
| 让 Nav2 看到更矮/更高的障碍 | 切片层 `z_min`/`z_max` | 记住 **SLAM 看不到多层**，只有 costmap 看得到 |
| 换机械臂/夹爪/新增 link | URDF | **`self_filter.chain_names` + `chains`**，否则机器人把自己当障碍 |
| 放宽/收紧目标容差 | `xy_goal_tolerance` | `goal_clearance_radius`（相等）、`arrival_xy_tolerance`（≥）|
| 底盘更保守 | `inflation_radius` | 必须 > `robot_radius` 0.42 |
| 臂展开时更早限速 | `reach_full_m` 往小调 | —— |
| 换 OMPL 规划器 | `planner_id` | `planning_attempts` 必须为 1；**核对注册行** |
| 换地图来源 | `map_source.yaml` | 只有三种合法组合；`real_live` 需 `ROS_LOCALHOST_ONLY=0` |

在线改（大部分阈值支持，切片节点会在下一帧**原子地**整套换上，失败则保留上一份有效配置）：
```bash
ros2 param set /pointcloud_slice_scan_node slices.low_obstacle.z_max 0.20
ros2 param set /frontier_explorer_node search.weight_gain 10.0
```
**例外，必须重启**：话题名与 `watchdog_period_sec`（需要重建订阅/定时器）—— 节点会显式拒绝。

### 5.3 改配置时必须同步的耦合项

这九条违反了会让系统坏掉，且**症状离根因很远**：

1. **`astribot_torso_base` 是全栈根 frame。** 写 `base_link` 会让 TF 永久失败，只有 WARN。
2. `validator.goal_clearance_radius` **必须等于** controller 的 `xy_goal_tolerance`（0.25）。
3. `arrival_xy_tolerance`（0.30）**必须 ≥** `xy_goal_tolerance`（0.25）。
4. 地图分辨率 0.05 时 `goal_unknown_clearance_radius` **必须为 0.0**。
5. `validation.use_costmap: true` 与 `allow_unknown: false` **必须同向**。
6. **`/map` 找前沿，`global_costmap` 判可站** —— 绝不互换。
7. 自滤只存在于 `pointcloud_slice_scan_node` 一处；改 URDF 必须镜像到它的链配置，
   且 **SLAM 地图有记忆**，旧幽灵点要重新建图碾掉。
8. 每个 `IncludeLaunchDescription` **必须显式传 `params_file`** 并包 `GroupAction(scoped=True)`。
9. `scan_source` 是**一个**开关：改了 Nav2 话题却没起切片节点 = 代价地图零障碍。

### 5.4 禁止修改项与已知不一致

**禁止：**
- ❌ 改厂商 SDK 源码；❌ 用软链接绕 ABI 冲突
- ❌ 在业务代码里写 `if sim / else real`
- ❌ 直接申请 `sdk_high_control_rights=true`（开发阶段只读安全模式优先）
- ❌ 省掉底盘 leash 牵引绳保护；❌ 静默吞异常
- ❌ 复用已证伪的旧度量（关节角偏差作为伸展度量）
- ❌ 改 MoveIt2/OMPL 系统库源码；❌ 硬编码 DH 参数/关节极限

**已知不一致（本次核出，注释滞后于代码；除第 1 条外均无行为影响）：**

| # | 位置 | 不一致 | 影响 |
|---|---|---|---|
| **1** | `moveit_config/config/joint_limits.yaml:46-65` vs `astribot_s1_torso_wheel.xacro:270-362` | 躯干 `max_velocity: 6.0` vs URDF `velocity="1.8"` = **3.33 倍** | ⚠️ **有行为影响且在不安全方向**。该 yaml 挂在 `robot_description_planning` 下（`move_group.launch.py:100`），**覆盖** URDF。哨兵测试只守 URDF，没守这份 yaml |
| 2 | `frontier_explorer_params.yaml:10` | 声明 `robot_radius = 0.35`，实际 Nav2 是 0.42 | 第 55/70 行的 `0.35` 是**活值**，比真实底盘包络松 0.07 m |
| 3 | `pointcloud_slice_scan_params.yaml:118` | 注释称"地面在 −0.080"，而权威推导是 **−0.095**（`−0.015 − 0.08`）| 贴地层下沿比设计意图高 1.5 cm，5~6.5 cm 高的矮障碍看不到 |
| 4 | `collision_overrides.yaml` | 记 `default_spawn_z: 0.10`，launch 实际默认 **0.15**；记轮子碰撞是 `cylinder`，xacro 已是 `sphere` | 记录失真 |
| 5 | `manipulation_params.yaml` | 关节限位清单仍是旧 `whole_body` 那套 | 注释误导 |
| 6 | `manipulation_params.yaml` | `transport_probe` 的 IK 预算**只在代码默认值里**，不在 yaml | ⚠️ 该文件自称"唯一的数值来源"，照它调参**没有任何效果** |
| 7 | 耦合包 README | 倾覆余量把四角都当前轴 `0.2163`，实际后轮 `0.21134675` | 结论位移很小，但 `0.42+0.2163=0.64` 阈值源于前轴数 |
| 8 | `nav2_params_rpp.yaml` §4.8 | 称 `inflation_radius` 降到 0.25 | 实际两份均为 **0.65** |
| 9 | `moveit_controllers.yaml` 头注释 | 少数了夹爪 ×2 与 `wheel_effort_controller` | 功能列表本身正确 |
| 10 | `astribot_s1_manipulation/README.md` | `mobile_transport` 段与 2026-08-25 代码相反；仍称"本机无夹爪关节" | 方法部分仍有效，坐标是旧腕部模型下测的 |

---

## 6 · 运行操作说明

### 6.1 启动矩阵

**只用一条命令时选这一行。** 顶层入口（会把下层一起拉起来）标 ★。

| launch 文件 | 包 | 拉起什么 |
|---|---|---|
| ★ `nav2_full_bringup.launch.py` | navigation | Gazebo + 感知 + SLAM + Nav2 (+ 可选探索协调器) |
| ★ `perception_slam_bringup.launch.py` | perception | Gazebo + 感知 + SLAM（不含 Nav2）|
| ★ `planning_demo.launch.py` | manipulation | `move_group` + 场景编排（可 `execute:=true`）|
| ★ `bridge_bringup.launch.py` | trajectory_bridge | 厂商 SDK 桥接容器 |
| `warehouse_sim.launch.py` | gazebo_bringup | 仅 Gazebo + 世界 + 控制器 + `ros_gz` 桥 |
| `navigation.launch.py` | navigation | 仅 Nav2 + 两个适配节点 |
| `autonomy_bringup.launch.py` | autonomy | 感知切片 + 前沿决策（同容器）|
| `exploration_coordinator.launch.py` | autonomy | 仅协调器（要求 Nav2 + SLAM 已在跑）|
| `slice_scan.launch.py` / `frontier_explore.launch.py` | autonomy | 单独起切片 / 单独起决策 |
| `sim_perception.launch.py` / `hardware_perception.launch.py` | perception | 仿真 / 硬件雷达感知链 |
| `slam_mapping.launch.py` / `slam_localization.launch.py` | perception | 建图 / 定位 |
| `map_provider.launch.py` | perception | 地图来源仲裁（三种合法组合，见 §1.4）|
| `arm_chassis_coupling.launch.py` | dynamics_coupling | 限速第二级（单独调试用）|
| `omni_effort_drive.launch.py` | chassis_effort_drive | 底盘力矩驱动（单独调试用）|
| `move_group.launch.py` | moveit_config | 仅 `move_group` |
| `rviz.launch.py` | gazebo_bringup | RViz |

⚠️ **别手工同时起两个顶层入口** —— 会出现两个 Gazebo / 两个 `/joint_states` 发布者，
而两边都不报错（§8.2）。

### 6.2 仿真：Gazebo 栈

### 6.3 仿真：厂商 MuJoCo 栈

### 6.4 建图 / 导航 / 自主探索

```bash
# 建图 + 导航（推荐：MPPI 更能发挥全向底盘能力）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true controller_plugin:=mppi

# 预建图 + SLAM 定位 + 导航
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=localization launch_gazebo:=true \
  map_file_name:=/abs/path/my_map controller_plugin:=mppi

# 自主探索（协调器 + Nav2 一起）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py exploration:=true

# 只要感知 + SLAM
ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true
#   接管已在跑的 Gazebo： launch_gazebo:=false
#   硬件雷达分支：        env:=hardware
```

**存地图（注意是位姿图，不是栅格图）：**
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/abs/path/my_map'}"     # 基础名不带扩展名 → <name>.data + <name>.posegraph
```

**手动发目标 / 干预探索：**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}}"

ros2 topic echo /exploration/state
ros2 service call /exploration_coordinator_node/pause  std_srvs/srv/Trigger
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger
```

⚠️ `autonomous_patrol_node` 与 Nav2 **都发 `/cmd_vel`**，
所以 `nav2_full_bringup` 始终强制 `autonomous_patrol:='false'`。别手工同时起。

**RViz 调试（`rviz/autonomy_debug.rviz`，Fixed Frame 必须是 `map`）：**
`slice_<layer>` ×4 逐层确认有点、`self_filtered` 灰点须始终贴在 `self_filter_capsule` 橙线上、
两个预置 LaserScan（红 `/scan_from_cloud` vs 蓝 `/scan`，后者默认关）用于并排比对层覆盖。
三个最有用的读法：候选点扎堆 → 调 `adaptive_sample_gain`/`max_samples_per_cluster`；
红球太多 → 约束过紧，读 `reject_reason` 直方图；紫球连成墙 → 缩 `visit_penalty_radius`。

### 6.5 运动规划与搬运

```bash
# 只验证规划（不开 Gazebo，最快）
#   终端 2 的关节状态：不要用默认全零（§3.6 第 ① 条），要拖离全零或发 ready 姿态
ros2 launch astribot_s1_manipulation planning_demo.launch.py

# 全链路（能真的看到机器人动）
ros2 launch astribot_s1_manipulation planning_demo.launch.py \
  scenarios:=mobile_transport execute:=true      # ← 需先起 nav2_full_bringup

# 换规划器
ros2 launch astribot_s1_manipulation planning_demo.launch.py planner_id:=BITstarConfig
```

**重新生成 SRDF 自碰撞对：**
```bash
ros2 run astribot_s1_manipulation self_collision_pair_generator 10000
```
只粘 `ALWAYS`/`NEVER` 结果，**绝不要粘 "sometimes" 对**。

### 6.6 桥接层运行

### 6.7 停机与清场

```bash
# ⚠️ 不要用 pkill -f <本命令里出现过的模式>——会杀掉自己的 shell（exit 144）
pkill -x gz; pkill -x ruby
pkill -f 'ros2 launch' ; sleep 1
# ⚠️ 按工作空间路径 kill 会漏掉 /opt/ros/humble 下的二进制，僵尸会污染测试
pgrep -af 'parameter_bridge|nav2_|slam_toolbox|move_group|robot_state_publisher'
```
清场后必须复查：`pgrep -af 'gz sim|mujoco'` 为空、`/joint_states` 无发布者。

---

## 7 · 系统就绪校验

### 7.1 离线校验：测试套件

**全部 683 条，✅ 于 2026-08-20 实跑全绿。**

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash          # ← 必须，否则少 20 条且报「no tests collected」
```

| 包 | 数量 | 覆盖 |
|---|---|---|
| `astribot_trajectory_bridge` | **425** | 桥接状态机、写入闸门、夹爪数学/控制、底盘积分、SDK 环境 |
| `astribot_s1_autonomy`（gtest）| **84** | 切片投影 16、自滤 11、前沿搜索 17、路径校验 18、代价地图适配 22 |
| `astribot_s1_manipulation`（gtest）| **51** | 闭链 8、夹爪 16、优化型规划器 9、奇异监视 10、轨迹优化 8 |
| `astribot_s1_perception` | 43 | 起点栅格校验等 |
| `astribot_s1_description` | 31 | **限位/原点与厂商 per-part 模型一致性** + 夹爪模型 14 |
| `astribot_s1_dynamics_coupling` | 27 | 伸展度量、C1 回归哨兵 |
| `astribot_s1_navigation` | 22 | —— |

⚠️ **`pytest test/` 在桥接包里会报 `no tests collected`，而不是报错。**
真因：`test_status_code_map.py` 用了模块级 `pytest.importorskip`，在 pytest 6.2.5 下
**整场收集被中断**。逐文件跑是 405 条，source 了 overlay 后跑整目录才是 425 条 ——
差的 20 条正是需要 `astribot_bridge_msgs` rosidl 产物的那些。

C++ 侧：
```bash
colcon test --packages-select astribot_s1_autonomy
colcon test-result --test-result-base build/astribot_s1_autonomy --verbose
```

**几条哨兵测试的用途（改配置前先读）：**

| 测试 | 挡住的具体历史错误 |
|---|---|
| `test_not_using_whole_body_limits` | 退回 `whole_body_with_wheel.urdf` 限位（躯干速度宽 3.3 倍，不安全方向）|
| `test_old_metric_was_anti_correlated_on_measured_poses` | 把耦合度量改回关节偏差 |
| `TwoGridDeadlock_*` / `ClearanceCaliber_*` | 探索双图校验的死锁与口径错误 |
| `test_l11_cross_source_delta_is_pinned` | 夹爪 7 mm 两源不确定度被悄悄"修好" |

### 7.2 环境校验

```bash
# 1. 只有一个仿真实例（两个会抢 domain，症状是 TF 时间跳变）
pgrep -af 'gz sim|ign gazebo|mujoco' | grep -v grep

# 2. 只有一个 /joint_states 发布者
ros2 topic info /joint_states --verbose | grep -c 'Node name'

# 3. domain 一致（不带它会「节点全都 Node not found」）
echo "$ROS_DOMAIN_ID"

# 4. numpy 钉子没被踩
python3 -c "import numpy; assert numpy.__version__=='1.21.5', numpy.__version__"

# 5. 时钟源（仿真必须 true，实机必须 false）
ros2 param get /pointcloud_slice_scan_node use_sim_time
```

⚠️ 清理残留进程时，**按工作空间路径 kill 会漏掉 `/opt/ros/humble` 下的二进制**
（`parameter_bridge`、`nav2_*`），留下的僵尸会污染测试结果。

### 7.3 在线校验：分层探针

仓库内自带四个工具，**每个验证的是不同的一件事**，不要互相替代：

| 工具 | 验证什么 |
|---|---|
| `ros2 run astribot_trajectory_bridge joint_map_probe` | **部件划分与关节顺序**。⚠️ 顺序必须**探**而不能**读**，这一步不通过就不要往下走 |
| `ros2 run astribot_s1_perception map_start_cell_check` | 起点栅格是否可站（`Starting point in lethal space` 的第一手判据）|
| `ros2 run astribot_s1_navigation path_tracking_diagnostics_node` | 区分"机器人不动"的六类原因 |
| `ros2 run astribot_s1_autonomy scan_slice_debug.py` | 终端里看四层切片，不用开 RViz |

**逐层判据（按此顺序，前一层不过不要查后一层）：**

```bash
# L1 时钟
ros2 topic hz /clock
# L2 感知
ros2 topic hz /livox/fused_points /scan_from_cloud /scan
# L3 建图 + TF
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom astribot_torso_base
# L4 导航链（中间任何一环断掉都表现为「机器人不动」）
ros2 topic hz /cmd_vel_nav_body_raw /cmd_vel_nav_body /cmd_vel_pre_arm_coupling /cmd_vel
# L5 限速真的在动
ros2 topic echo /cmd_vel_pre_arm_coupling   # 与 /cmd_vel 比对
# L6 探索时序
ros2 topic echo /exploration/state | grep -o 'state=[A-Z_]*' | uniq
```

### 7.4 就绪判据总表

| # | 判据 | 通过标准 |
|---|---|---|
| 1 | 离线测试 | 683 全绿 |
| 2 | 单仿真实例、单 `/joint_states` 发布者 | 各为 1 |
| 3 | `/clock` 在走 | 非零频率（恒 0 见 §8.2）|
| 4 | 三条感知话题 | `/livox/fused_points` 约 9.5~10 Hz；`/scan_from_cloud`、`/scan` 有输出 |
| 5 | 四个切片层 | **每层都有点**（永久空 = z 范围写错）|
| 6 | 自滤跟随 | 灰点始终贴在橙色胶囊上；胶囊随臂移动（不动 = TF 没工作）|
| 7 | TF 链完整 | `map→odom`、`odom→astribot_torso_base` 均不断 |
| 8 | 导航链四段 | 四个话题都有流量 |
| 9 | 探索时序 | 相邻 `NAVIGATING` 之间必有 `ARRIVED`；每次派发前必有 `双层校验通过` |
| 10 | 关节顺序探针 | 通过（**未通过禁止继续**）|

---

## 8 · 常见问题定位排查

### 8.1 排查总则

本项目已经**两次**因为跳过这一步而写出错误的根因，并让它扩散进文档、使排查停止。所以：

> **断言根因之前，先跑那条能证伪它的命令。**

两个反面案例：判断"GUI 把 server 饿死"—— 一条 `nproc` 就否证了（28 核，负载 188% 饿不死）；
判断"pinocchio ABI 冲突卡住在线验证"—— 一条 `ldd` 就否证了（全部解析成功，零 `not found`）。

**还有一条同等重要的：**

> **验证机器人真的动了，不只是指令对。**

e2e 检查只比对 `dispatched_cmd` 等于只验证了自己。曾因此让"要求半开、实际冲到全闭合（99.998）"
这个真缺陷被判为"✔ 通过"并打印"全部通过"。**成功路径必须比对实测位置。**

### 8.2 启动类

| 症状 | 真因 | 确认方法 |
|---|---|---|
| 节点全都 `Node not found` | 查询 shell 没带 `ROS_DOMAIN_ID`（全栈统一 25，与厂商 `env.sh` 一致）| `ROS_DOMAIN_ID=25 ros2 node list` |
| 参数全是默认值、改 yaml 没反应 | **`launch` 的 `params_file` 共享上下文泄漏**：第一个 include 占用共享名，后续节点静默加载错 yaml（`/**:` 文件加载无报错，全部回落声明默认值）| 每个 `IncludeLaunchDescription` 显式传 `params_file`，并包 `GroupAction(scoped=True)` |
| TF 查询永远失败，只有 WARN | 用了 `base_link` —— **本机没有这个 frame**，根 frame 是 `astribot_torso_base`。切片节点的代码默认值恰好是 `base_link`，params_file 没加载上就会掉进去 | `ros2 run tf2_tools view_frames` |
| 重复 action server：`unknown goal response`、`MoveItErrorCode=-7`、`Invalid Trajectory`、`PREEMPTED` | launch **漏了 shutdown handler**，泄漏出第二个 `move_group`。看着像并发 bug | **先数进程**：`pgrep -af move_group` |
| `spawner: Could not contact service /controller_manager/list_controllers` FATAL | 已知的间歇性时序问题，与业务改动无关 | 清干净残留进程后重启 |
| Gazebo 所有话题有发布者但零消息，协调器被 THROTTLE 静音 | 物理不步进，`sim time` 恒为 0。**真因未定**，曾误判为"GUI 饿死 server"（28 核已证伪）；headless 只是规避手段 | `ros2 topic hz /clock` |

### 8.3 通信类

| 症状 | 真因 | 确认方法 |
|---|---|---|
| 同机同域两个节点互相发现不了 | `ROS_LOCALHOST_ONLY=1` 与 `=0` 的 DDS 参与者**互相发现不了**（同机同域也不行）| 比对两侧 `env \| grep LOCALHOST` |
| 设了 `ROS_LOCALHOST_ONLY=1` 但流量还在网卡上 | Fast DDS 的 `interfaceWhiteList` XML **会静默击败它**；Gazebo 的 `ign-transport` 要另设 `IGN_IP` | `tcpdump` 看业务网卡 |
| `incompatible QoS` 告警（`joint_space_command`）| **两端都在厂商 SDK 内部**：SDK 自己的 RELIABLE 订阅收不到自己的 BEST_EFFORT 发布。仿真侧兼容，指令实测 100% 落地、误差 0.0 mm | 无需处理；不要去改本栈 QoS |
| `tf2_buffer: Detected jump back in time` | 两个仿真实例抢同一个 `ROS_DOMAIN_ID` | 数进程 |

### 8.4 感知与建图类

| 症状 | 真因 | 确认方法 |
|---|---|---|
| `Starting point in lethal space`，且障碍 blob **跟着机器人走** | ①两处 `range_min` 口径不同（3D 球面 vs 纯 XY，见 §2.1）；②某个 link 不在自滤链里（夹爪曾整个缺失）| RViz 看 `self_filtered` 灰点是否始终贴在橙色胶囊上 |
| 机器人静止时已知区只有约 1.4 m 半径 | **Karto 从不主动碾自由空间**：`max_laser_range 20.0 == scan range_max 20.0` → 可碾区间 `[threshold, maxRange)` **为空** → 无回波方向永远碾不出自由空间（实测 723 束仅 245 束有回波）| 见附录 B-1；修它要同时改三处 |
| 巡游节点一直原地自转、走不出去 | 雷达一直"看见自己"，最开阔方向逻辑看到"到处都很近"，误判被困角落。**不是撞货架** | 看自滤是否生效 |
| 某个切片层永远空 | 该层 z 范围写错 | RViz `slice_<layer>` ×4 逐层确认都有点 |
| 改了自滤配置，幽灵障碍还在 | **SLAM 地图有记忆**，必须重新建图碾掉 | 重开一张图 |

### 8.5 导航与探索类

| 症状 | 真因 | 确认方法 |
|---|---|---|
| 机器人只自转、不平移 | **第一个要查的**：`cmd_vel_body_to_world_node` 是否在跑并真的在换算（车体系 vs world 系）| `ros2 node list \| grep body_to_world` |
| 探索 `dispatched` 恒为 0，机器人永不移动 | `goal_unknown_clearance_radius ≥ 地图分辨率` → 按定义否掉**每一个**候选（实测 6775/6784）| 看 `reject_reason` 直方图 |
| 底盘爬行 2 cm/s + MPPI 求解失败 + 控制器掉频，三个看似无关的症状 | **同一个根因：参数泄漏。** `omni_effort_drive_node` 收到了协调器的 yaml → `control_period_sec 0.5` 把 100 Hz 循环变成 1.85 Hz，自身 PID/摩擦参数静默回落到已知发散的默认值（`pid_kp=2.0` 而稳定需 < 1.0；`friction_viscous_nm_s=0.02` 而正确值 1.0，**小 50 倍**）→ 轮 PID 发散成 ±15 N·m bang-bang、净偏航力矩 60 N·m、车体 3.3 rad/s 自转（**完全没有 `cmd_vel`**）→ 3.3 超过 MPPI `wz_max=2.0` → `Optimizer fail` ×94 → 净进展 2 cm/s → `Failed to make progress` → 协调器 PAUSED | 修复后实测：1.85 → **95 Hz**、净偏航力矩 60 → **−0.098 N·m**、`Optimizer fail` 94 → **0**、力矩饱和告警 1548 → **0** |
| Nav2 以约 0.085 m/s 下发但底盘不动，最终 `Failed to make progress` | 力矩驱动全向底盘的**静摩擦地板**。协调器把它当普通导航失败处理 | 属底盘控制层问题，见附录 B-3 |
| 限速改了没效果 | **两级限速串联叠乘**（`0.50 × 0.15 = 0.075`），两个包各一层 | §2.4 |
| 导航特别慢（实测曾 78.1 s）| 臂-底盘耦合用了已证伪的关节偏差度量，`ready` 都只有 scale ≈ 0.29 | §3.4；C1 修正后 13.4 s |

### 8.6 运动规划类

| 症状 | 真因 | 确认方法 |
|---|---|---|
| `Unable to solve the planning problem`（无更多信息）| `planning_attempts > 1` + informed 规划器 → `ParallelPlan` 抛异常 | 设成 1 |
| "我在跑 BIT\*"但性能像 RRTConnect | Humble 官方 OMPL 插件**没有** BIT\*/Informed RRT\*，未注册名字**静默回退组默认** | 查 `[OK]/[MISSING]` 注册行 + 每请求的 `planner_id=` 行 |
| 从默认姿态规划总失败 | 全零 = `joint_4=0` = 肘部完全伸直 = 奇异（`sigma_min 0.0077`）| 从 `ready` 起步 |
| `RETRIES_EXHAUSTED` | 可能是"TCP 贴着物体必然碰撞"（`tool_link` 与 `link_7` 的 `r=0.05` 球同心），**与"奇异被否"同码** | §3.7 |
| `start point deviates`（0.066 > 0.05），上层只看到 `MoveItErrorCode=-4` | **控制器报完成时手臂还在收敛**，下一步规划拿到移动中的起点 | 下发前等真正静止 |
| 机器人越限后每条轨迹都被拒、连开回来都不行 | 每个路点都查限位，而**路点 0 就是当前位置** | 路点 0 是测量值不是目标，需留恢复通路 |
| 张口静默算成一半、抓取角错一倍且零报错 | `setJointGroupPositions` **漏掉组外 mimic**（SRDF 夹爪组只含主动关节）| 用 `setJointPositions` |
| 探针扫了两轮全是假阳性 | **探针判据与真跑不一致**：一轮物体不在场景里，一轮不看 `sigma_min` | 判据少一项就在那一项上系统性假阳 |

### 8.7 桥接与 SDK 类

| 症状 | 真因 | 确认方法 |
|---|---|---|
| `No module named 'meta'` | 实际缺的是 **`filterpy`**，错误名极具误导性 | 装时必须 `--no-deps` 以保住 numpy 1.21.5 |
| SDK import 失败 | 三层原因：`robotics_library_py.so` 与同名包互相遮蔽（预热可解）、proxy 用 **cwd 相对路径**、`env.sh` 指的 `astribot_ros_middleware_py` 目录缺失（只在真机上有）| 逐层排除，**不是** pinocchio 冲突 |
| 取消后手臂漂 0.045~0.37 rad | **保持位置必须持续重发**：`set_joints_position` 发一次抓不住正在运动的关节。SDK 的 `desired` 已等于所发值却仍会漂 | 只能看**实际位置**，持续重发后实测 0.0000 |
| 要求半开，夹爪冲到全闭合 | 中间开度**单次下发**会冲到底 | 必须持续重发 |
| 夹爪方向反了 | **`0` = 张开、`100` = 闭合**（与直觉相反，三处独立来源印证）| §3.9 |
| `set_effector_max_force` 没效果 | 仿真下是**空操作** | 力控在仿真中不可验证 |
| IK 报成功但姿态不对 | `get_inverse_kinematics` **恒返回 `flag=True`**（连零位解都报成功）、只迭代一小步、会停在局部最优后缓慢劣化 | **唯一判据是 FK 回验** |
| 场景物体的全局坐标对不上 | SDK 的 `world` 系在**会话启动时以当前底盘位姿重置**，无法表达全局坐标 | IK 目标用 `chassis` 系 |
| 底盘停车距离比预期远一倍 | 分两段：积分器只走"惯性 + 减速尾巴"，SDK 的 `control_way='filter'` 还会再收敛约等量一段 | 判据只能压在积分器上；三个量混一起会误判 |

### 8.8 测量可信度类

这一类的危险在于：**数据看起来正常，结论是错的。**

| 症状 | 真因 | 确认方法 |
|---|---|---|
| 跟踪误差出现高度可复现的 0.327 假尖峰；读数漂出硬限位 | **长跑的仿真会退化**：`j6 = 1.04` 而 MuJoCo `range = ±0.78` —— **物理上不可能** | 重启仿真后全消失（peak 回到 0.0347~0.0639）。⚠️ 曾据此差点把坏基线拟合进生产配置 |
| `pytest test/` 报 `no tests collected` | 模块级 `pytest.importorskip` 在 pytest 6.2.5 下**中断整场收集** | 见 §7.1：必须先 source overlay |
| 隔离实验所有组 `peak=0.0000` | 忽略了 `start()` 的返回值 → 零样本，而脚本仍打印了自信的判词 | 加有效性门；**脚本里不要写自动判词** |
| 端口契约测试恒通过 | `hasattr` 对"继承了会抛异常的基类实现"的子类**永远为真** | 改成检查函数对象身份，并加探测器自测 |
| 结论比数据说得多 | 本阶段发生过三次（"惯性尾巴无害"漏了幅度、"三组都没恶化"、"机制是底盘位置本身"）| **故障注入后必须先确认注入真的生效** |

**自伤类脚本 bug（会让"启动失败"的假结论成立）：**
`pkill -f` / `pgrep -f` 匹配到自己的命令行（杀掉自己的 shell，exit 144）；
`source env.sh | grep` 在子 shell 里跑导致 export 全丢；
`set -u` + ROS `setup.bash` 静默退出；
脚本内 `rm -f "$LOG"` 而 stdout 已重定向到该文件；
Python stdout 缓冲在超时时丢输出；两个仿真实例同时在跑。

---

## 附录 A · 测试清单

见 §7.1。总计 **683 条，2026-08-20 实跑全绿**：
桥接 425、自主 84、操作 51、感知 43、描述 31、耦合 27、导航 22。

## 附录 B · 已知边界与未验证项

**⬜ 真机侧数据为零。** 本栈全部在线结论来自仿真（Gazebo 或厂商 MuJoCo）。

| # | 边界 | 状态 |
|---|---|---|
| 1 | **SLAM 从不主动碾自由空间** | 已定位未修。静止时已知区仅约 1.4 m 半径（245/723 束有回波）；一动起来多姿态并集填到约 75% 自由，所以平时不显形。修它要**同时**改三处（slam 指向 `/scan_from_cloud`、`max_laser_range` 降到 `range_max` 以下如 12.0、新增一档无回波值）。刻意不修：把多层切片喂给 Karto 做 scan matching **有退化风险**（跨层取最近距离产出的不是一致水平切面，而 Karto 是 scan-to-map，几何一致性比物理真实更重要），这条**没有验证过** |
| 2 | **机械臂碰撞包络对规划器不可见（C1b，未实现）** | 实测臂水平伸展可达 0.8865 m，而 `robot_radius` 0.42 —— **0.47 m 未建模包络**。限速**不缩小**包络。正解是 `nav2_collision_monitor` 做位姿相关动态足迹 |
| 3 | 力矩驱动全向底盘有**静摩擦地板** | Nav2 有时以约 0.085 m/s 下发而底盘不动 → `Failed to make progress`。协调器按普通导航失败处理 |
| 4 | 手臂点自滤**无法在本仿真里端到端验证** | 上游 `range_min 0.35` 在点到达切片节点前就切掉了臂上的点：左臂从收纳摆到大幅伸展，落在左臂胶囊内的点数**恒为 0**（躯干胶囊内稳定 664 点 / 降采样后 131）。改由 `test_self_filter.cpp` 覆盖；要在线验证需把上游 `range_min` 降到约 0.1 并重调 `min_points` |
| 5 | `VelocityControl` **忽略碰撞反作用力** | 安全监控只能"止损"（停止下发），不能纠正正在发生的碰撞。巡游与 Nav2 同此，非 Nav2 集成引入 |
| 6 | 硬件雷达分支**仅编译验证** | `MID360_config_{left,right}.json` 的 `host_net_info` / `lidar_configs[].ip` 是占位值，需现场实测填入 |
| 7 | **底盘移动会恶化手臂跟踪 2~5 倍** | 基线 0.08 → 0.16~0.42。数据**双峰**（0.16@35% / 0.42@85%），**机制未定**。已排除移动余波与 desired-actual 分叉。这解释了搬运里手臂 ABORTED。⚠️ **机制查清前不改** `max_tracking_error_rad`（现 0.10）|
| 8 | "真的把物体夹起来"**未验证** | 被两条堵住：IK 的 `flag` 恒为真（只能 FK 回验）、SDK 的 `world` 系随会话重置 |
| 9 | 夹爪力控在仿真中**不可验证** | `set_effector_max_force` 是空操作 |
| 10 | `WBC_SYNC` 出处**未定位** | 全仓搜索零命中 |
| 11 | 底盘闭环**未测** | MuJoCo 侧没有 SLAM |
| 12 | 双臂并发、plan A `move_joints_waypoints` | ⬜ 从未运行 |
| 13 | **Gazebo 的 mimic 有 7.8° 稳态误差** | 支持但不精确（从动关节到 0.794 而非 0.93）。看 `/joint_states` **永远发现不了**，须反解从动关节角度 |
| 14 | 厂商 SDK **没有任何地图接口** | 25 个 msg + 7 个 srv 里没有 `OccupancyGrid`、没有 `GetMap`。真机对外只给两颗雷达点云 + 底盘 `[x, y, theta]`。所谓"真机地图"是我们自己的 slam_toolbox 跑在真机雷达上 |

## 附录 C · 术语与坐标系约定

| 术语 | 含义 |
|---|---|
| `astribot_torso_base` | **全栈根 frame。本机没有 `base_link`。** |
| 地面 z | **−0.095**（相对 `astribot_torso_base`，推导：轮关节 z 偏移 −0.015 − 轮球半径 0.08）。**不是 −0.129**，也不是切片 yaml 注释里的 −0.080。odom/Gazebo 的静态高度 0.1292 **不是**地面偏移，因为 world z=0 不是地板 |
| 出生高度 | `min_spawn_z 0.095`，launch 默认 `spawn_z 0.15` |
| 底盘构型 | **X 型全向轮（OMNI），不是麦克纳姆轮**（命名已于 2026-08-21 清理，旧文档仍有"麦轮"字样）|
| 夹爪命令空间 | **`0` = 张开，`100` = 闭合**（与直觉相反）。`rad = 0.0093 × cmd`，实测残差 0.0005 / 回差 0.0000 / 系数吻合 0.02% |
| `map_source` / `localization` | 两条正交轴：谁提供 `/map` / 谁提供 `map→odom`。仅三种合法组合 |
| 位姿图 vs 栅格图 | `slam_toolbox` 存的是 `.data` + `.posegraph`，**不是** `.pgm` + `.yaml` |
