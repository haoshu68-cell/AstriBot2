# astribot_s1_autonomy

Astribot S1 自主感知与探索。两个 rclcpp 组件，一个包：

| 模块 | 组件类名 | 干什么 |
|---|---|---|
| **感知** | `astribot_s1_autonomy::PointcloudSliceScanNode` | 3D 点云**多层高度切片**投影成 2D LaserScan，靠 TF 实时剔除底盘/双臂自身点，供 Nav2 costmap 使用 |
| **决策** | `astribot_s1_autonomy::FrontierExplorerNode` | 占据栅格上做**边界遍历 + 前沿点自适应采样**，输出探索目标位姿 |

两个模块之间没有直接依赖，可以单独启动。决策模块**只发目标位姿**，不发速度、不含 Nav2 客户端。

---

## ⚠️ 先看这个：本机器人没有 base_link

整机根坐标系是 **`astribot_torso_base`**（运行时 TF 实测确认，Nav2 的 `robot_base_frame` 也是它）。
配置里所有 `z` 值都是**在这个坐标系下的高度，不是离地高度**：

```
地面 ≈ z = -0.129 m        (实测静止位姿 z = 0.1292)
离地高度 = z + 0.129
```

把 `base_frame` 写成 `base_link` 会导致 TF 查询永久失败、节点一帧都出不来（只会刷 WARN，不会崩）。

---

## 编译

```bash
cd ~/WorkSpace/astribot_sdk_ros2/ws_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select astribot_s1_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

依赖说明：只用 `pcl_conversions` + 原生 PCL（common/filters），**不依赖 `pcl_ros`**（该包在本机未安装）。
点云的坐标变换由本包自己用 Eigen 仿射变换完成。

---

## 启动

```bash
# 感知 + 决策，装进同一个 component_container（推荐）
ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py

# 顺手开一个预配好全部调试 Marker 的 RViz
ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py use_rviz:=true

# 只要感知
ros2 launch astribot_s1_autonomy slice_scan.launch.py
# 只要决策
ros2 launch astribot_s1_autonomy frontier_explore.launch.py

# 拆成两个独立进程，便于单独 gdb
ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py use_composition:=false

# 让感知直接顶替既有 /scan，Nav2 不用改配置就能吃到多层切片结果
ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py output_scan_topic:=/scan
```

命令行覆盖项留空即表示「用 yaml 里的值」，不会把配置冲掉。

---

## 话题清单

### 感知模块 `pointcloud_slice_scan_node`

| 方向 | 话题 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| 输入 | `/livox/fused_points` | `sensor_msgs/PointCloud2` | SensorData | 双雷达融合点云，可用 `input_cloud_topic` 改 |
| 输入 | `/tf`, `/tf_static` | TF | — | 查 `点云frame → astribot_torso_base`，以及各连杆实时位姿 |
| 输出 | `/scan_from_cloud` | `sensor_msgs/LaserScan` | SensorData | 多层切片融合结果，可用 `output_scan_topic` 改成 `/scan` |
| 输出 | `~/debug_markers` | `visualization_msgs/MarkerArray` | Reliable(1) | 各切片层点云、被剔除的自身点、剔除胶囊体 |

### 决策模块 `frontier_explorer_node`

| 方向 | 话题 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| 输入 | `/map` | `nav_msgs/OccupancyGrid` | **TransientLocal + Reliable** | slam_toolbox 输出。QoS 必须匹配，否则「话题在但收不到图」 |
| 输入 | `/tf`, `/tf_static` | TF | — | 查 `map → astribot_torso_base` 拿机器人位姿 |
| 输出 | `/explore/goal_pose` | `geometry_msgs/PoseStamped` | Reliable(1) | **目标位姿，由外部送进 Nav2** |
| 输出 | `/explore/status` | `std_msgs/String` | Reliable(1) | `EXPLORING / COMPLETE / WAITING_MAP / WAITING_TF / NO_VALID_GOAL \| 详情` |
| 输出 | `/explore/complete` | `std_msgs/Bool` | **TransientLocal** | 探索完成标志，晚启动的订阅者也能拿到 |
| 输出 | `~/debug_markers` | `visualization_msgs/MarkerArray` | Reliable(1) | 前沿块、遍历路径、候选点、选中目标、历史惩罚区 |

---

## 如何驱动机器人（决策模块不含 Nav2 客户端）

按约束，本包不封装 Nav2 客户端。把目标位姿接到 Nav2 的办法：

```bash
# 看一眼探索模块在往哪走
ros2 topic echo /explore/goal_pose --once

# 手动把某个目标喂给 Nav2
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: -1.0}, orientation: {w: 1.0}}}}"
```

要长期自动跑，写一个外部桥接节点：订阅 `/explore/goal_pose` → 调 `NavigateToPose` action，
并在 `/explore/complete == true` 时停止。这个桥接属于「外部编排」职责，
不放在本包内（本包的 Python 只用于调试可视化）。

---

## 参数表

### 感知模块（`config/pointcloud_slice_scan_params.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `input_cloud_topic` | `/livox/fused_points` | 输入点云话题 |
| `output_scan_topic` | `/scan_from_cloud` | 输出 LaserScan 话题 |
| `base_frame` | `astribot_torso_base` | 投影所在本体系（**不是 base_link**） |
| `tf_timeout_sec` | 0.05 | TF 查询超时，超时丢帧等 TF 恢复 |
| `max_cloud_age_sec` | 0.30 | 点云时间戳与当前时刻偏差上限，超出丢帧 |
| `tf_time_tolerance_sec` | 0.10 | 点云与 TF 时间戳偏差上限，防时间错位造成假障碍 |
| `enable_voxel_filter` / `voxel_leaf_size` | true / 0.03 | 体素降采样 |
| `enable_outlier_filter` / `outlier_mean_k` / `outlier_stddev_mul` | true / 12 / 2.0 | 统计离群点滤波 |
| `min_valid_points` | 20 | 剔除自身点后不足该数，本帧判为不可信 |
| `watchdog_period_sec` / `input_timeout_sec` | 0.5 / 1.0 | 输入看门狗 |
| `invalid_input_policy` | `hold_last` | `hold_last` 重发上一帧 / `stop_output` 停发 |
| `publish_markers` / `marker_point_stride` | true / 3 | 调试 Marker 及抽稀 |
| `scan.angle_min` / `angle_max` / `angle_increment` | ±π / 0.0087 | 360°，0.5° 分辨率，约 722 束 |
| `scan.range_min` / `range_max` | 0.35 / 20.0 | 有效距离区间 |
| `scan.no_return_mode` | `range_max` | 无障碍方向填最大距离（需求规定）；`infinity` 可对齐旧行为 |

**多层切片**（`slice_names` + `slices.<名>.*`，改 `slice_names` 即可增删层，至少 2 层，程序强制校验）：

| 层 | z 区间 (torso_base) | 离地高度 | `min_points` | `max_range` | 针对什么 |
|---|---|---|---|---|---|
| `low_obstacle` | −0.08 ~ 0.12 | 5~25 cm | 3 | 6.0 | 托盘、踏板、纸箱底、地面杂物 |
| `main_nav` | 0.12 ~ 0.55 | 25~68 cm | 2 | 20.0 | 货架立柱、墙面，导航主力层 |
| `torso_high` | 0.55 ~ 1.05 | 68 cm~1.18 m | 2 | 12.0 | 桌面、货架横梁（底盘能过但躯干会撞） |
| `overhead` | 1.05 ~ 1.50 | 1.18~1.63 m | 3 | 8.0 | 悬空横梁、门楣（撞头） |

- `min_points`：该角度桶内至少多少点才认账。**贴地层给大一点**，压制地面反光噪点。
- `max_range`：该层的信任距离。贴地层在底盘俯仰时容易把远处地面误判成障碍，所以只信近处。
- 融合方式：同一角度桶**跨层取最小距离**，所以任意高度上最近的障碍都会体现在 scan 上，不存在层间漏检。

**自身点剔除**（`self_filter.*`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `footprint.enabled` / `radius` | true / 0.40 | 底盘足迹圆柱，管底盘和轮子 |
| `footprint.z_min` / `z_max` | −0.25 / 0.05 | 圆柱高度范围 |
| `chain_names` | torso, head, arm_left, arm_right | 连杆链列表 |
| `chains.<名>.frames` | 见 yaml | 按顺序的 TF frame 名，相邻两个连成胶囊体 |
| `chains.<名>.radius` | torso 0.20 / head 0.18 / 双臂 0.15 | 胶囊体半径 |

### 决策模块（`config/frontier_explorer_params.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map_topic` / `goal_topic` | `/map` / `/explore/goal_pose` | |
| `robot_base_frame` | `astribot_torso_base` | |
| `planning_period_sec` | 2.0 | 探索规划周期。太快会在机器人还没动时反复改目标 |
| `map_timeout_sec` | 10.0 | 地图超时告警（仍用旧图规划，不停机） |
| `goal_same_tolerance` | 0.35 | 两次目标距离小于该值视为同一目标 |
| `max_same_goal_count` | 3 | 同一目标连续 3 次 → 加重惩罚并重新全局采样 |
| `max_relaxation_level` / `relaxation_scale` | 3 / 0.6 | 采不到点时逐级放宽约束 |
| `max_consecutive_failures` | 5 | 连续 5 轮失败 → 告警并清空历史惩罚 |
| `visit_history_limit` | 50 | 历史访问记录上限 |
| `search.occupied_threshold` / `free_threshold` | 65 / 25 | 栅格三态判定 |
| `search.obstacle_inflation_radius` | 0.35 | **必须 ≥ 机器人半径**，否则会采到进不去的贴墙点 |
| `search.min_obstacle_cluster_cells` | 3 | 小于该格数的孤立占据斑块视为噪声抹掉 |
| `search.min_frontier_cells` | 12 | 前沿块最小格数（0.05 m/格 → 约 0.6 m） |
| `search.gain_window_radius` | 1.5 | 未知增益统计窗口半径 |
| `search.adaptive_sample_gain` | 0.15 | 采样数 = clamp(ceil(前沿格数 × 该值), min, max) |
| `search.min_samples_per_cluster` / `max_` | 1 / 8 | 上限防止候选扎堆 |
| `search.required_clearance_radius` | 0.35 | 候选点净空半径 |
| `search.min_goal_distance` / `max_goal_distance` | 0.8 / 0（不限） | 过近/过远目标过滤 |
| `search.weight_distance` / `weight_gain` / `weight_visit_penalty` | 1.0 / 6.0 / 4.0 | 代价权重 |
| `search.visit_penalty_radius` | 1.2 | 历史惩罚作用半径 |
| `search.random_seed` | 20260819 | 固定种子，采样可复现 |

代价函数：

```
cost = weight_distance × 距离(m)
     + weight_visit_penalty × 历史访问惩罚
     − weight_gain × 归一化未知增益(0~1)
取 cost 最小者
```

调参直觉：`weight_gain=6` 意味着「增益从 0 涨到满分」值得多走 6 米。
想更贪心地扑向大片未知区 → 加大 `weight_gain`；想先把近处扫干净 → 加大 `weight_distance`。

### 动态调参

全部阈值都能在线改，不用重编译：

```bash
ros2 param set /pointcloud_slice_scan_node slices.low_obstacle.z_max 0.20
ros2 param set /pointcloud_slice_scan_node self_filter.chains.arm_left.radius 0.18
ros2 param set /frontier_explorer_node search.weight_gain 10.0
ros2 param list /pointcloud_slice_scan_node        # 看全部可调参数
```

话题名和 `watchdog_period_sec` 例外——它们要重建订阅/定时器，节点会明确拒绝并提示重启。

---

## RViz 调试配置要点

本包自带 `rviz/autonomy_debug.rviz`，已经把下面所有显示项配好：

```bash
ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py use_rviz:=true
# 或单独开
rviz2 -d $(ros2 pkg prefix astribot_s1_autonomy)/share/astribot_s1_autonomy/rviz/autonomy_debug.rviz
```

**Fixed Frame 必须设成 `map`**（探索模块的 Marker 都在 map 系；感知模块的在
`astribot_torso_base` 系，RViz 会自动用 TF 换算）。

### 感知模块 Marker（`/pointcloud_slice_scan_node/debug_markers`）

| 命名空间 | 显示内容 | 怎么用 |
|---|---|---|
| `slice_low_obstacle` 等 4 个 | 各高度层的点，按层不同色相 | 确认**每层都有点**。若某层恒为 0，说明该层 z 区间配错了 |
| （半透明的层） | `enabled: false` 的层 | 一眼看出哪层没参与融合 |
| `self_filtered` | 被判为机器人自身而剔除的点（灰） | 挥动机械臂时，灰点应始终贴着橙色胶囊 |
| `self_filter_capsule` | 双臂/躯干/头部剔除胶囊体轴线（橙色粗线，线宽=直径） | 胶囊必须跟着手臂动。不动 ⇒ TF 没生效 |

**关键对照技巧**：RViz 配置里预置了两个 LaserScan 显示项——
`多层切片融合 scan`（红，`/scan_from_cloud`）和 `对照-原单层切片 scan`（蓝，`/scan`，默认关闭）。
把两个都打开，能直观看出多层切片多检出了哪些障碍物，尤其是低矮托盘和悬空横梁。

### 探索模块 Marker（`/frontier_explorer_node/debug_markers`）

| 命名空间 | 显示内容 |
|---|---|
| `frontier_cluster` | 通过过滤的前沿连通域，每块一个色调（确认聚类没把不同区域粘在一起） |
| `frontier_rejected` | 被过滤掉的前沿块（暗灰） |
| `frontier_label` | 每块的 `#序号 n=面积 gain=增益`，被拒的还带 `[原因]` |
| `frontier_traversal_path` | **边界遍历路径**：黄线依次连接各前沿块质心 |
| `candidate_valid` / `candidate_rejected` | 候选采样点，绿=通过 / 红=被拒 |
| `selected_goal` | 最终选中的目标位姿，橙色箭头，箭头方向即目标朝向 |
| `visit_history` | 历史访问记录（紫球），即代价函数的惩罚区 |

调参时最有用的三个观察：
1. **候选点是否扎堆**：绿球应沿前沿铺开。全挤在一处 ⇒ 调大 `adaptive_sample_gain` 或 `max_samples_per_cluster`。
2. **红球比例过高**：说明约束太严。看日志里的 `reject_reason` 分布，再决定放宽哪个（净空 / 最小距离 / 膨胀半径）。
3. **紫球是否堵住了出路**：紫球连成片说明惩罚区过大，调小 `visit_penalty_radius` 或 `weight_visit_penalty`。

### 终端诊断脚本（不开 RViz 时）

```bash
ros2 run astribot_s1_autonomy scan_slice_debug.py
```

输出帧率、有效障碍束占比、各扇区最近距离、**各切片层点数柱状图**、探索状态与目标跳变距离，
并在检测到「所有切片层都是 0 点」「目标几乎不动」时直接给出提示。

---

## 排障：几个真实踩过的坑

按出现概率排序，都是本模块开发过程中实际遇到并定位的。

### 1. 机械臂在 costmap 里变成幽灵障碍物（自身点剔除看起来没生效）

先看日志有没有这一条：

```
[ERROR] [tf2_buffer]: Do not call canTransform or lookupTransform with a timeout
        unless you are using another thread for populating data.
```

**症状极具误导性**：`/tf_static` 里的静态变换（雷达→本体）照样查得到，
所以点云投影一切正常、`/scan_from_cloud` 有输出、看起来毫无问题；
但 `robot_state_publisher` 发在 `/tf` 上的**动态**变换（双臂 27 个连杆）100% 查不到，
于是自身剔除形同虚设，日志里只有一条不起眼的
`本帧有 26 个连杆 TF 查询失败`。

**根因**：节点在自己的工作线程里带 timeout 调 `lookupTransform`，
必须让 listener 自带 spin 线程、并显式 `setUsingDedicatedThread(true)`，
两者缺一，带超时的查询就必然失败。本包已修复，代码里有详细注释。

### 2. 组件容器加载失败：`intraprocess communication allowed only with volatile durability`

不要给本包的组件加 `use_intra_process_comms: True`。
两个节点都要用 TF，而 `/tf_static` 必须是 `TRANSIENT_LOCAL`；
探索模块还要以 transient_local 订阅 `/map`、发布 `/explore/complete`。
进程内通信要求全链路 volatile，二者不可兼得。本包的 launch 已默认不开。

### 3. 节点起来了但一帧都不处理

本节点会主动告警并列出检查清单：

```
[WARN] 启动后 12.3s 从未收到任何点云。请依次检查：① 上游话题是否在发 ...
```

最常见的两个原因：上游 livox 链没起来；`use_sim_time` 没设成 true
（仿真里若用系统时钟，点云时间戳校验会把每一帧都判为超时丢弃）。

### 4. 探索模块一直 `WAITING_MAP`

`/map` 是 `TRANSIENT_LOCAL + RELIABLE`。QoS 不匹配时话题看得见、数据收不到。
本包已按匹配 QoS 订阅；如果换了 SLAM 方案，注意保持一致。

### 5. 本仿真链路无法端到端验证「机械臂点被剔除」

上游 `livox_preprocess_node` 配了 `range_min: 0.35`（单雷达坐标系），
而双臂离雷达通常不到 0.35m，**机械臂的点在进入本模块之前就被距离门限截掉了**。

实测证据：把左臂从收纳摆到大幅伸展（`joint_2 → -1.3`、`joint_4 → +1.2`），
落在左臂胶囊体内的点数**恒为 0**；而躯干胶囊体内稳定有 664 点（3cm 体素降采样后 131）。

所以这条能力由**单元测试**验证（`test/test_self_filter.cpp`，用真实机器人几何构造点云，
覆盖「胶囊内必剔」「胶囊外一点点必不剔」「胶囊随连杆位姿移动」三条）。
想在仿真里真正跑通这条链路，需要把上游 `range_min` 调小到 0.1 左右
（代价是会引入更多近距离噪点，需要配合调 `min_points`）。

---

## 单元测试

算法核心（切片投影、自身剔除、前沿搜索）都不依赖 ROS 运行时，可以在无仿真环境直接跑：

```bash
colcon test --packages-select astribot_s1_autonomy
colcon test-result --test-result-base build/astribot_s1_autonomy --verbose
```

| 测试文件 | 用例数 | 覆盖 |
|---|---|---|
| `test_slice_projector.cpp` | 16 | 参数校验、**低矮托盘在单层切片下会漏检而多层不会**、悬空横梁、地面/过高点排除、空结果填 range_max、`min_points` 抗噪、分层 `max_range`、NaN/Inf、角度分桶 |
| `test_self_filter.cpp` | 11 | 点-线段距离（含端点退化、零长退化）、胶囊内/外判定、**剔除区随连杆位姿移动**、TF 丢失时降级、足迹圆柱边界 |
| `test_frontier_search.cpp` | 17 | 空地图/长度不一致/未配置不崩、全explored无前沿、自由-未知交界、**目标不落在障碍内**、**被包围前沿判不可达**、过近目标过滤、**采样数随面积自适应**、候选不扎堆、净空校验、历史惩罚换目标、增益优先、朝向指向质心、小斑块去噪、膨胀防贴墙 |

`colcon build` 与 `colcon test` 均为 **0 warning / 0 failure**。
