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

# ---- 真正自主跑起来：探索协调器（严格时序调度 + 未知区禁行）----
# 要求 Nav2 和 SLAM 已在运行。详见「如何驱动机器人」一节。
ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py
# 或者连 Nav2 一起拉起来（推荐）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py exploration:=true
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

## 如何驱动机器人：探索协调器 `exploration_coordinator_node`

`frontier_explorer_node` 是**候选点建议流** —— 每个规划周期都发一次 `/explore/goal_pose`，
不管上一个目标走到哪了。直接把它接进 Nav2 会出现三个具体问题：提前下发、重叠下发、连续跳点。
（实测过：临时 Python 桥接 20s 内向 Nav2 下发 18 次目标、反复抢占、成功 0 次。）

`exploration_coordinator_node` 是**调度器**，它自己调 Nav2 的 action，是真正驱动机器人的那个：

| | frontier_explorer | exploration_coordinator |
|---|---|---|
| 输出 | `/explore/goal_pose` 建议流 | 直接调 `NavigateToPose` |
| 时序 | 无，每周期都发 | 严格单点推进，抵达并驻留才生成下一个 |
| 未知区 | 只保证目标点不在膨胀障碍内 | 目标点 + 全局路径**逐点**校验 |
| 用途 | 调参、看 Marker | 自主探索建图 |

**两者不要同时启动**：会有两个源往 Nav2 塞目标，正是需求禁止的「重叠下发」。

```bash
# 单独启动协调器（要求 Nav2 和 SLAM 已在跑）
ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py

# 连 Nav2 一起（推荐）：exploration:=true 会自动拉起协调器
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py exploration:=true

# 观察状态机
ros2 topic echo /exploration/state

# 人工干预
ros2 service call /exploration_coordinator_node/pause  std_srvs/srv/Trigger
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger
```

### 状态机

```
                    ┌──────────────────────────────────────────┐
                    │                                          │
                    v                                          │
   ┌────────┐  地图/定位/里程计就绪   ┌───────────────┐         │
   │  IDLE  │ ─────────────────────> │ GEN_NEXT_POINT│         │
   └────────┘                        └───────┬───────┘         │
       ^                                     │ 候选已生成       │
       │                                     v                 │
       │                             ┌───────────────┐         │
       │      候选全被拒，重新采样    │  VALIDATING   │         │
       │      <──────────────────────┤  (等全局路径) │         │
       │                             └───────┬───────┘         │
       │                                     │ 双层校验通过     │
       │                                     v  【唯一下发点】  │
       │                             ┌───────────────┐         │
       │      导航失败，重新选点      │  NAVIGATING   │         │
       │      <──────────────────────┤               │         │
       │                             └───────┬───────┘         │
       │                                     │ Nav2 SUCCEEDED  │
       │                                     v                 │
       │                             ┌───────────────┐         │
       │                             │    ARRIVED    │         │
       │                             │ 实测位置+驻留  │─────────┘
       │                             └───────────────┘  收敛完成
       │
       │  自动恢复(限次)      ┌────────┐        ┌───────────┐
       └──────────────────────│ PAUSED │        │ COMPLETED │
                              └────────┘        └───────────┘
                          导航失败超限 /        地图内已无
                          定位丢失 /            任何前沿格
                          连续无合法候选 /
                          人工 pause
```

**关键区分（写反了会让机器人提前停止探索）**

- 一个前沿格都没有 ⇒ `COMPLETED`（真的探索完了）
- 有前沿格但候选点全不合法 ⇒ `PAUSED`（只是暂时找不到合法通路）

**两个「多余」状态为什么必须存在**

- `VALIDATING`：路径校验要调 `ComputePathToPose`，那是**异步 action**，不能在回调线程里阻塞等结果
  （会把执行器卡死，TF 和地图都收不到）。它是 `GEN_NEXT_POINT` 的子步骤，不是第五个业务阶段。
- `PAUSED`：需要一个「冻结但未结束」的表达。没有它就没法区分"暂时走不动"和"探索完成"。

### 严格时序是怎么保证的

三道锁，缺一道就可能重复下发：

1. **状态锁** —— `allowsGoalGeneration()` 只对 `GEN_NEXT_POINT` 返回 true；
   `dispatchNavGoal()` 只允许从 `VALIDATING` 调用，其余状态直接 ERROR 拦截。
2. **在途标志** —— `std::atomic<bool> nav_goal_in_flight_`，下发前 `compare_exchange`。
   任何并发下发企图都会失败在这里并打 ERROR。
3. **入口统一加锁** —— `state_mutex_` 由三类入口（定时器 / action 回调 / 服务回调）统一持有，
   一次 tick 或一次回调就是一个原子的状态推进。
   *如果每个小函数各自加锁，状态判断和状态修改之间会出现锁间隙，两个线程能各自通过
   「现在是 VALIDATING」的检查然后双双下发。*

**抵达判定不只看 Nav2 的 SUCCEEDED**：Nav2 报成功后进 `ARRIVED`，还要
实测位置偏差 ≤ `arrival_xy_tolerance`、合速度 ≤ `settle_speed`，且**连续满足**
`dwell_time_sec` 才算收敛。速度一旦超限就重新计时 —— 不允许「路过即算抵达」。
Nav2 报成功但实测超差，按导航失败处理（否则「抵达校验」形同虚设）。

### 未知区域禁行的两层校验

| 层 | 检查什么 | 为什么单靠上一层不够 |
|---|---|---|
| 校验1 目标点 | 目标格已知空闲，且 `goal_clearance_radius` 邻域内无未知/占据格 | 只查单格不够：机器人有 0.42m 外接半径，目标格空闲但紧邻未知区时，停过去会有半个身子在未知区里 |
| 校验2 路径 | Nav2 返回的路径**逐段插值采样**，步长 ≤ `resolution/2`，每个采样点都必须已知空闲 | **只查顶点会漏**：Smac 的路径点可能几十厘米一个，一小块未知区正好夹在两个合法顶点中间时会被完全放过 |

单元测试 `SegmentCrossingUnknownIsRejected` 专门钉死第二条：路径只有两个顶点、两个顶点都合法、
中间夹一条 0.15m 宽的未知窄带。只查顶点的实现会判「路径合法」并放行。

配套改了 Nav2 侧：`nav2_params_{mppi,rpp}.yaml` 的 `allow_unknown` 由 `true` 改成 **`false`**。
否则 SmacPlanner2D 会主动穿未知区抄近道，协调器把这种路径整条否掉，
结果是候选一个个被拒、探索原地打转。两层约束方向必须一致。

### 话题与服务

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 订阅 | transient_local + reliable |
| `/odom` | `nav_msgs/Odometry` | 订阅 | 只用速度做驻留判定，取 `hypot(vx, vy)` |
| `/exploration/state` | `std_msgs/String` | 发布 | 单行字段化，每拍一发 |
| `/exploration/complete` | `std_msgs/Bool` | 发布 | transient_local，只在跳变时发 |
| `/exploration/current_goal` | `geometry_msgs/PoseStamped` | 发布 | 已下发目标，仅供 RViz |
| `~/pause` / `~/resume` | `std_srvs/Trigger` | 服务 | 人工冻结 / 重置全部计数器 |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | action 客户端 | 执行导航 |
| `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | action 客户端 | 只取路径做校验，不用于控制 |

`/exploration/state` 的格式（可直接用 grep 抓取做验证）：

```
state=NAVIGATING goal_in_flight=1 goal=(2.31,-0.84) candidate=1/5 dispatched=3 \
succeeded=2 rejected=4 nav_fail=0/3 sample_fail=0/4 auto_resume=0/3
```

### 异常处理对照表

| 场景 | 行为 |
|---|---|
| 导航失败 / 局部无路 / 被困 | 计数累加，达 `max_consecutive_nav_failures` → `PAUSED` + 告警，失败点记入访问历史 |
| 定位丢失（TF 查不到）/ 里程计超时 | 立即取消在途目标 → `PAUSED`，冻结目标发布 |
| 目标未收敛 | 拦截下一轮生成（只有 `GEN_NEXT_POINT` 允许生成） |
| 候选点在未知区 / 路径穿未知区 | 丢弃该候选、记入访问历史、试下一个；全被拒则重新采样 |
| 无合法已知区可前进 | 前沿格为 0 → `COMPLETED` 并发 `/exploration/complete` |
| 地图为空 / 尺寸不自洽 / 超时 | 拦截逻辑并回 `IDLE`，不崩溃 |
| 校验器参数非法 | `PathValidator` 停在未配置态，`isKnownFree()` 一律返回 false（fail-closed，绝不放行） |
| 状态机非法转换 | 取消在途目标 + 清空候选 + 强制回 `IDLE`，打 ERROR |
| 重试 | 全部限次：`max_sample_failures` / `max_consecutive_nav_failures` / `max_auto_resume_attempts`。超限只等人工 `~/resume`，**无死循环重试** |

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

### 探索协调器（`config/exploration_coordinator_params.yaml`）

**时序约束相关**（最需要按现场调的一组）

| 参数 | 默认 | 语义与调参依据 |
|---|---|---|
| `control_period_sec` | 0.5 | 状态机节拍。探索是低频决策，调快只让日志变噪音 |
| `arrival_xy_tolerance` | 0.30 | 抵达位置容差。**必须 ≥ Nav2 controller 的 `xy_goal_tolerance`**，否则 Nav2 报成功而这里判超差，会陷入「成功即失败」的反复重试 |
| `arrival_yaw_tolerance` | 0.40 | 朝向容差，仅 `check_yaw: true` 时生效 |
| `check_yaw` | false | 探索目标的朝向只是「让雷达看向未知区」的建议值，不是任务约束。卡朝向会让机器人原地反复微调、浪费大量时间 |
| `dwell_time_sec` | 1.5 | 稳定驻留时长。足够让 MPPI 末段振荡衰减，又不至于每个点白等太久。**觉得机器人「还在晃就换点了」就调大这个** |
| `settle_speed` | 0.05 | 驻留判定的速度上限(m/s)。全向底盘用合速度 `hypot(vx, vy)`，只看 `vx` 会把纯横移当成已静止 |
| `nav_timeout_sec` | 120.0 | 单目标最长导航时间。本底盘是力矩驱动全向轮，低速段推进偏慢，给太紧会把「还在正常走」误判成被困 |

**未知区禁行相关**

| 参数 | 默认 | 语义与调参依据 |
|---|---|---|
| `validator.goal_clearance_radius` | 0.42 | 目标点净空半径。**取 Nav2 footprint 外接半径**。调小会让机器人停在贴着未知区的位置 |
| `validator.path_sample_step` | 0.0 | 路径段内采样步长，0=自动取 `resolution/2`(0.025m)。**必须 ≤ resolution/2**，否则单格宽的未知缝隙会被整步跨过去 |
| `validator.path_endpoint_tolerance` | 0.5 | 规划终点与请求目标的偏差上限。防止规划器把不可达目标「尽力靠近」后返回半截路径也算通过 |
| `validator.max_samples` | 200000 | 单条路径采样上限。防御性无界保护；超限判**不合法**，绝不因超限而放行 |

**重试与限次**（「禁止死循环重试」的硬边界）

| 参数 | 默认 | 语义 |
|---|---|---|
| `max_candidates_per_cycle` | 5 | 单轮最多校验几个候选（按代价升序），试完仍无合法点就重新采样 |
| `max_sample_failures` | 4 | 连续几轮采不到合法点 → `PAUSED`（注意不是 `COMPLETED`） |
| `max_consecutive_nav_failures` | 3 | 连续几次导航失败 → `PAUSED`（疑似被困） |
| `pause_cooldown_sec` | 10.0 | `PAUSED` 后多久尝试一次自动恢复 |
| `max_auto_resume_attempts` | 3 | 自动恢复上限，超限只等人工 `~/resume` |
| `map_timeout_sec` / `odom_timeout_sec` | 10.0 / 1.0 | 数据超时。里程计超时即视为丢失，冻结目标发布 |

`search.*` 与决策模块同名参数语义完全一致，只有两处刻意不同：
`obstacle_inflation_radius` 和 `required_clearance_radius` 由 0.35 提到 **0.45**（略大于 footprint
外接半径 0.42），保证采出来的点是机器人真能站进去的，而不是贴墙的理论自由格。

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

### 6. 协调器卡在「等待定位就绪: TF 查询失败(map -> base_link)」

**不是** TF 没起来，是**参数文件根本没加载**，`robot_base_frame` 退回了代码里的声明默认值。

根因是 ROS2 launch 的一个共享作用域陷阱：`LaunchConfiguration` **不按 include 层级隔离**，
而 `DeclareLaunchArgument` 的 `default_value` 只在该名字**尚未被设置**时才生效。
于是 `params_file` 这个被所有子 launch 共用的名字会互相串——谁先 include，
谁的默认值就成了后面所有人的值。实测 `slice_scan` 先 include，协调器就拿到了
**切片感知的 yaml**；反过来在 `launch_arguments` 里显式传也会**继续往后泄漏**，
实测 `omni_effort_drive_node` 收到了协调器的 yaml。

之所以极难发现：这些 yaml 都用 `/**:` 通配，**会被正常加载且不报任何错**，
只是里面没有该节点的参数，所有阈值静默退回声明默认值。

正确做法是两件一起做：显式传 `params_file`（治「拿错」）+ `GroupAction(scoped=True)`
把作用域关在组内（治「往后泄漏」）。一条命令就能确诊：

```bash
pgrep -af <node_name> | grep -o "params-file [^ ]*"
```

### 7. 有几千个前沿格，但候选一个都留不下（`dispatched` 恒为 0）

看日志里「目标校验淘汰 N」这一项。如果 `搜索否决 0` 而 `目标校验淘汰` 等于候选总数，
基本就是 `validator.goal_unknown_clearance_radius` 配大了。

**它必须是 0**。格算术：前沿格的定义是「空闲格且 8 邻域含未知格」，未知邻居的格距离
`d_sq = 1`(正交) 或 `2`(对角)；校验按 `d_sq <= (r/resolution)^2` 遍历邻域。
所以只要 `r >= resolution`（0.05m 就是 1 格），`(r/res)^2 >= 1 >= d_sq` 恒成立，
**每一个**前沿候选都会被否——等于要求前沿点不是前沿点。

实测（活地图 277x414 @0.05m）：合法前沿候选 6784 个，通过 **0** 个，
其中 6775 个(99.87%)死在这一条；改成 0 后 6775 个全部通过，立刻开始下发目标。

这个坑栽过两次：第一次和 `goal_clearance_radius` 共用 0.42，
改成 0.10 以为解决了——但 0.10/0.05 = 2 格，仍然 >= 1 格，**数值变了、机制没变**。
逐条原因是 DEBUG 级，排查时要 `log_level:=debug`。

### 8. 底盘只有 2cm/s、MPPI 频繁求解失败、控制器保不住 20Hz

这三个「看起来无关」的现象是**同一个根因**，就是上面第 6 条的参数泄漏，
只不过受害者是 `omni_effort_drive_node`：

1. 它吃到了协调器的 yaml，其中 `control_period_sec: 0.5` 把底盘控制环
   从 100Hz 改成 2Hz（实测 1.85Hz）；
2. 同时它自己的 PID/摩擦参数一条都没加载，全部退回声明默认值——
   恰好是重构前那套**已知会发散**的值（`pid_kp=2.0`，稳定条件要求 `< 1.0`；
   `friction_viscous_nm_s=0.02`，该配 1.0，小了 50 倍）；
3. 于是轮速 PID 发散 → 四轮全部钉在 ±15N·m 限幅 bang-bang → 净旋转力矩 60N·m →
   车身在**没有任何 `cmd_vel`** 的情况下持续自转 3.3rad/s；
4. 这个 3.3rad/s 超出 MPPI 的 `wz_max=2.0`，控制器求解必然失败
   (`Optimizer fail to compute path` ×94)；
5. 车只自转不前进，净进度 ~2cm/s，Nav2 进度检查器判 `Failed to make progress`，
   每个目标都在剩 ~1m 处中止，协调器连续导航失败后进 `PAUSED`。

修好泄漏后实测：控制环 1.85Hz → 95Hz，净旋转力矩 60N·m → -0.098N·m，
`Optimizer fail` 94 → 0，力矩饱和告警 1548 → 0，
直接发 `/cmd_vel vx=0.3` 实测跑出 0.334m/s（`wz` 仅 0.022rad/s，不再自转）。

---

## 验证探索协调器

### 1. 不开仿真的冒烟测试（10 秒）

```bash
ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py use_sim_time:=false
```

预期：打出「探索协调器已启动」+「抵达判定: xy<=... 驻留>=...」，然后每 3s
一条「等待地图就绪: 尚未收到占据栅格地图」。**没有地图就一直停在 IDLE，不崩溃、不下发** —— 这本身就是一条要验证的规则。

另开一个终端确认状态出口和服务都在：

```bash
ros2 topic echo /exploration/state --once
# data: state=IDLE goal_in_flight=0 candidate=0/0 dispatched=0 succeeded=0 ...
ros2 service list | grep exploration_coordinator_node/
# .../pause  .../resume
```

### 2. 全链路仿真验证

```bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py exploration:=true
```

### 3. 三条必须验证的规则，以及怎么验

**规则一：严格单点推进（任意时刻最多一个目标在途）**

```bash
# goal_in_flight 只能是 0 或 1，且两次 dispatched 递增之间必须夹一个 ARRIVED
ros2 topic echo /exploration/state | grep -o 'state=[A-Z_]*' | uniq
```

预期序列形如 `IDLE → GEN_NEXT_POINT → VALIDATING → NAVIGATING → ARRIVED → GEN_NEXT_POINT → ...`。
**判定标准**：序列里绝不出现两个相邻的 `NAVIGATING` 段之间没有 `ARRIVED`；
`dispatched` 每 +1，前面必有一次 `ARRIVED`。

```bash
# 下发次数不该超过「成功数 + 失败数」，超了就说明有重复下发
ros2 topic echo /exploration/state --once | grep -o 'dispatched=[0-9]*\|succeeded=[0-9]*'
```

日志里出现 `检测到并发下发企图` 或 `试图在 ... 态下发目标，已拦截` 说明第 2/1 道锁挡住了
一次真实的重复下发 —— 属于**保护生效**，但需要查为什么会有并发企图。

**规则二：目标与路径都不碰未知区**

```bash
# 每次下发前必有一条「双层校验通过」
ros2 launch ... 2>&1 | grep -E "双层校验通过|路径未知区校验未通过|目标点复检未通过"
```

抓一个已下发目标，回到地图上核对该栅格值：

```bash
ros2 topic echo /exploration/current_goal --once
# 然后在 RViz 里看这个点：必须落在白色(已知空闲)区域，不能贴着灰色(未知)边缘
```

**规则三：异常时暂停而非乱跑**

```bash
# 人工触发：暂停后 dispatched 必须停止增长
ros2 service call /exploration_coordinator_node/pause std_srvs/srv/Trigger
ros2 topic echo /exploration/state --once   # state=PAUSED ... manual_pause=1
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger
```

模拟定位丢失（杀掉 SLAM）→ 预期 `导航中定位丢失` + 取消在途目标 + 转 `PAUSED`，
且 `PAUSED` 期间 `dispatched` 不再增长。

### 4. 已知的环境限制

本仿真底盘是力矩驱动全向轮，低速段存在静摩擦门槛：Nav2 有时会以 ~0.085 m/s 下发速度而底盘
不动，最终 `Failed to make progress`。**协调器对此的处理是把它当普通导航失败**：
计数累加 → 达 `max_consecutive_nav_failures` → `PAUSED` + 告警。
这是底盘控制层的问题，不在协调器职责内；协调器的正确行为就是不卡死、不乱跑、如实报告。

---

## 单元测试

算法核心（切片投影、自身剔除、前沿搜索、未知区校验）都不依赖 ROS 运行时，可以在无仿真环境直接跑：

```bash
colcon test --packages-select astribot_s1_autonomy
colcon test-result --test-result-base build/astribot_s1_autonomy --verbose
```

| 测试文件 | 用例数 | 覆盖 |
|---|---|---|
| `test_slice_projector.cpp` | 16 | 参数校验、**低矮托盘在单层切片下会漏检而多层不会**、悬空横梁、地面/过高点排除、空结果填 range_max、`min_points` 抗噪、分层 `max_range`、NaN/Inf、角度分桶 |
| `test_self_filter.cpp` | 11 | 点-线段距离（含端点退化、零长退化）、胶囊内/外判定、**剔除区随连杆位姿移动**、TF 丢失时降级、足迹圆柱边界 |
| `test_frontier_search.cpp` | 17 | 空地图/长度不一致/未配置不崩、全explored无前沿、自由-未知交界、**目标不落在障碍内**、**被包围前沿判不可达**、过近目标过滤、**采样数随面积自适应**、候选不扎堆、净空校验、历史惩罚换目标、增益优先、朝向指向质心、小斑块去噪、膨胀防贴墙 |
| `test_path_validator.cpp` | 15 | 未知/占据/越界均判不合法、**未配置时一律拒绝(fail-closed)**、目标紧邻未知区被净空规则拒、空地图不崩、空/单点路径拒、**半截路径拒**、顶点落未知拒、**顶点全合法但段间穿未知窄带必须拒**、段间穿占据拒、**自动步长 ≤ resolution/2**、采样超限不误判为合法 |

`colcon build` 与 `colcon test` 均为 **0 warning / 0 failure**（59 个算法用例 + 33 个 lint 用例）。

`test_path_validator.cpp` 里最要紧的一条是 `SegmentCrossingUnknownIsRejected`：
它构造的路径**只有两个顶点、两个顶点所在格都合法**，未知区是夹在正中间的一条 0.15m 竖直窄带。
只检查路径顶点的实现会判定「路径合法」并放行，机器人随后直接穿过未知带 —— 这种漏检
在现场表现为「机器人一头开进没扫过的区域」，事后极难复盘，所以必须能离线钉死。
