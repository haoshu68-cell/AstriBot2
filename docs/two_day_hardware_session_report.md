# S1 实机联调纪要 · 8.31 – 9.1

> `astribot@orin` · aarch64 Jetson Orin · ROS 2 Humble · `ROS_DOMAIN_ID=25` · 分支 `chassis-effort-drive`
>
> 数据采集 2026-09-01 21:20–21:35 · **全程未发送任何速度指令**
>
> 标「已验证」的都有当次会话可复现的实测数字；标「未验证」/「预测」的**请勿当结论使用**。

两天在真机上的工作：把「雷达 → SLAM → 感知 → 导航」这条链第一次完整打通，
并在打通过程中修掉 4 类**会静默骗过判据**的缺陷。

---

## ⚠️ 写在最前 —— 报告成稿时整栈处于降级状态

**9-01 21:26:01，三个依赖厂商 SDK 的进程在同一秒全部退出**
（`state_bridge` / `chassis_odom` / `bridge_container`），死因完全一致：

```
Driver heartbeat timeout detected! Count: 5/5
Driver crashes. Reached timeout threshold (5). Exiting...
```

三者引用的是**同一个** `Last error_code_timestamp: 1788269160.069`。

这正是计划书里编号 **D3** 的那条「厂商心跳失效会杀掉我们所有 SDK 进程」——
从「有记录的已知缺陷」变成了**实证事件**。连带后果：`/scan` 从 9.9 Hz 掉到
**0.56 Hz**，动态避障再次失效。详见 [第三节 P0](#p0--厂商控制驱动下线带走我们全部-3-个-sdk-进程)。

---

## 一、已完成工作概述

两天分成两个性质不同的阶段：**8.31 是「让链路跑起来」**（9 个提交，已全部入库）；
**9.1 是「让链路可信」**（78 个文件改动，尚未提交）。

| 日期 | 主线 | 产出 | 状态 |
|---|---|---|---|
| 8.31 | 生产拓扑首次贯通 + 可视化通路 | SLAM→栅格→nav2 全链验证记录；PC/机器人两套 rviz 方案；底盘写通路饿死缺陷修复 | ✅ 9 个提交已入库 |
| 9.1 | 标定纠偏 + 补齐缺失环节 + 判据加固 | URDF 雷达位姿修正；新建 CustomMsg→PointCloud2 转换节点；QoS 断链修复；A2/A3 时空基准加固；启动脚本阶段⑦ | ⚠️ 78 个文件未提交 |

### 一句话结论

| 问题 | 两天前 | 现在 |
|---|---|---|
| `/scan` 有没有数据（动态避障） | **0 Hz —— 从来没通过** | 曾达 **9.9 Hz**，❌ 现降级 0.56 |
| 雷达安装位姿 | xacro 占位默认值，与实机差 **0.108 m / 68.8°** | ✅ 已按标定修正 + 6 项交叉校验 |
| SLAM 基准离地高度 | 我曾答 0.017 m（错） | ✅ **0.095 m**，三次独立确认 |
| 厂商 CustomMsg → 我们的 PointCloud2 | 缺环节，曾被误判为「格式互斥无解」 | ✅ C++ 转换节点上线，两路 10.00 Hz |
| TF 树完整性 | `map` 与 `odom` **两座孤岛** | ✅ 全连通，78020 次更新 0 跳变 |
| nav2 时钟 | 实机上**恒为 0**（`use_sim_time` 默认 true） | ✅ 已修，六节点均 false |

---

## 二、按模块细分

### M1 · 感知链：CustomMsg → PointCloud2 → 融合 → 自滤 → /scan

`astribot_s1_autonomy`（新建 C++ 节点）· `astribot_s1_perception`

**已做**

- 新建 `livox_custom_to_pc2_node`（C++）：厂商驱动只发 `livox_ros_driver2/msg/CustomMsg`，
  我们整条链吃 `PointCloud2`，中间一直缺这一环。
- 纯逻辑与 ROS 解耦（`livox_custom_convert.cpp` 不含任何 ROS 头），便于单测。
- 逐通道**空回波剔除**：无回波点被雷达设备的外参一起变换，堆成一个固定坐标的密集点团。
- 修复 `livox_preprocess_node` 订阅端 QoS：`RELIABLE` → `qos_profile_sensor_data`。
- `hardware_perception.launch.py` 增加 `left/right_input_topic`，默认指向 `_pc2` 两路。

**已验证**

| 判据 | 实测 | |
|---|---|---|
| 转换节点两路输出频率 | 10.00 / 10.00 Hz | ✅ 与雷达同拍 |
| 空回波剔除比例（节点内统计） | 33.3% / 34.8% | ✅ |
| 同一指标的**独立** Python 探针 | 33.4% / 34.0% | ✅ 交叉验证 |
| 输出点云里的假点团点数 | **0** | ✅ 彻底剔除 |
| `is_dense` / `frame_id` | true / `livox_frame` | ✅ |
| 打通后 `/scan` | **9.9 Hz** | ✅ 首次有数据 |
| 自滤实际剔除量 | 76~80 点/帧（原为 0） | ✅ 生效 |
| 27 条 C++ 单测 | 全过 | ✅ 含变异测试 |

**对比结果与结论**

- **假点团用球面距离门根本挡不掉。** front 那团落在 `(0,0,0)`，back 那团落在
  `(0.001,−0.496,0.084)` —— 距原点 **0.4968 m**，而 `range_min=0.35`。
  back 的团**在门外**，只能按「等于外参平移量」这个特征点剔除。
- **「xfer_format 1-vs-2 互斥，无解」这条旧结论是错的。** 真相是缺一个转换节点。
  旧结论会让人去改驱动配置，方向完全错。
- **QoS 单向不兼容会静默切断整条链。** BEST_EFFORT 发布 + RELIABLE 订阅 = 一帧都收不到，
  全程只有**一条 WARNING**。而链上每个节点都有发布者，所以 `count_publishers` 这个判据
  **恒为真** —— 实测五个话题全 pub=1 / 0 Hz。中段必须按**拍率**验。
- **Python 追不上 CustomMsg。** 反序列化两万点跟不上 10 Hz，静默丢 60~80%。
  同一套系统我先后测出 **6% / 41% / 100%** 三个配对率，前两个都是我探针自己丢帧造成的。
  改 `raw=True` 后才对 —— 这也是转换节点必须用 C++ 的直接依据。

**未验证**

- `/livox/right/cloud_filtered` **8.0 Hz** vs left **10.1 Hz** —— 慢 20%，把融合拖到 8.1 Hz。原因未查。
- 转换节点在长时间运行下的内存/时延漂移。只测了分钟级。

---

### M2 · 传感器标定与 URDF

`astribot_s1_description` · 本机仓库 + 机器人两侧同步

**已做**

- 按厂商标定 + 实测修正两台 MID-360 的安装位姿。
- 补上 `livox_mid360_left → livox_frame` 恒等边（只挂在 left 实例上）。
- 新增 `config/livox_extrinsics.yaml` 记录来源。

```
livox_left_xyz   0.17393  0.16893  0.082    rpy 0 0 -0.800058
livox_right_xyz -0.18120 -0.17733  0.166    rpy 0.001168 -0.009370 2.373036
```

**已验证** —— 六项几何交叉校验（每项都是**独立**来源比对，不是同一个数换算两遍）

| 校验项 | 残差 |
|---|---|
| 两路点云拟合地面的离地高度差 | 2.5 mm |
| 两路地面平面之间的差 | 0.6 mm |
| 两路法向夹角 | 0.43° |
| 两雷达间距（点云 vs URDF） | 7.1 mm |
| 安装高度差 | 0.084 m，两侧一致 |
| 两雷达连线中点 | 5.5 mm |

51 个 link / 34 个 mesh，在机器人上全部解析成功（`mesh_MISSING=0`）。

**对比结果与结论**

- 旧值与实机差 **0.108 m / 68.8°**。原值是 xacro 宏的**占位默认值**，不是标定结果。
- **关键认知：驱动把外参写进雷达设备本身**（`SetLivoxLidarInstallAttitude`），
  所以 `/livox/lidar_back` 的点**已经在 front 系里**，两路 `frame_id` 都是 `livox_frame`。
  上层不能再变换一次。
- 我曾说「两台雷达基本同位同姿」—— **错的**。那是拿设备变换**之后**的点云量的。
  实际是背靠背、相距 0.496 m、偏航约 180°。
- `--symlink-install` 一路链回 `src`，改 URDF **不需重编**，但必须重启 `robot_state_publisher` 才生效。

> **方法论 · 这次最贵的一个教训**
> 我先前用 `tf2_echo` 读到 z=0.100 就当成「实测」，据此答出错误的基准高度。
> 但 **TF 只是把 URDF 里的占位值原样发出来** ——
> **量一个由错值生成的量，不构成对那个值的验证。**

**未验证**

- `chassis_extrinsic` 的 yaw 是旧 URDF 占位值的 **2.0001 倍**（0.800058 vs 0.4），
  是「把 yaw 当半角构造四元数」这个经典 bug 的典型特征。
  **但我无法证明**：平移与 URDF 差 0.106 m 说明不是直接抄的，且六项几何校验全过 ——
  这组数**就算来路可疑，几何上是自洽正确的**。需标定者确认。

---

### M3 · SLAM 接入与时空基准（A2 / A3）

第三方 Voxel-SLAM（独立工作区）· 我们的 `map_odom_tf_node` / `cloud_to_grid_node`

**已做**

- **A3**：`map→camera_init` 恒等边改为构造时即发、走 `/tf_static`（latched）。
  旧代码挂在 `_tick` 里且只发一次动态 TF。
- **A2**：新增 `check_source_age()`，源侧 TF 超龄（`MAX_SOURCE_AGE=1.0s`，时钟偏差容忍 0.05s）就拒绝发布。
- 修复 `cloud_to_grid` 的 TF 查询时刻：用**最新**变换而不是点云 stamp（已提交 `dcc0098`）。
- 更正 `docs/slam_nav_dataflow_and_datum.md` 里我先前写错的基准推导。

**已验证**

- **A2 抓到一次真实事件**，不是构造的测试数据：
  `camera_init→aft_mapped 已经 1.66s 没更新（上限 1.00s）`，并正确拒绝发布。
  之后 78020 次更新里没有复发，陈旧计数稳定在 1。
- **A3 前后对照**：修复前该边实测**不存在**（TF 树是两座孤岛），修复后为**静态**边。
- `map→odom` 连续 **78020** 次更新、**0** 跳变。
- `/map_scan_filtered` **9.995 Hz**；`cloud_to_grid` 投影 `雕刻失败=0`、`切片外=0`、单帧耗时 **0.03 s**。
- `/map` 0.476 Hz 是**节点主动限流**（收 200 帧投影 10 帧），不是故障。

**对比结果与结论**

- **SLAM 基准离地 0.095 m，不是我先前说的 0.017 m** —— 你在 rviz 里
  「SLAM 基准不在地面」的观察是**对的**。三次独立确认。
- **A2 为什么必须存在**：tf2 只在某 frame pair 有**新**数据时才修剪缓冲。
  停更的 pair 会**永久返回最后一条记录**，且 `lookup_transform(..., Time())` 一直「成功」、不报错。
  SLAM 挂掉时会算出一个随机器人移动而**反向漂移**的 `map→odom`，而判据全绿。

**未验证**

- **C4 回环跳变的影响**：历史记录里回环修正最大 0.626 m。本次**测不出来** ——
  机器人静止，78020 次更新里跳变 0 次。要评估必须让机器人走出一个回环。**需发速度**。
- SLAM 侧遗留两处误导（在第三方仓库，我没动）：`save_path: /home/pf/…`（本机无此路径，
  `is_save_map:1` 时会炸）、源码注释「world-frame z 直接就是离地高度」（现已不成立）。

---

### M4 · 导航 nav2

`astribot_s1_navigation`

**已做**

- 实机分支强制 `use_sim_time:=false`，并在启动脚本里新增 `check_nav_clock()` 真查时钟是否前进。
- 阶段⑤判据从「costmap 有没有发布者」改成 `check_tf_edge map odom` —— 真查 TF 边可解。

**已验证**

- 修复后六个 nav2 节点 `use_sim_time` 全为 **False**，时钟正常前进。
- `/scan` 的 QoS 双侧一致：发布端 `BEST_EFFORT`，`local_costmap` 与 `global_costmap`
  订阅端**同为** `BEST_EFFORT` —— **nav2 确实收到了 `/scan`**（今天逐个端点确认）。
- `/map` 双侧 `RELIABLE + TRANSIENT_LOCAL` 一致。
- 规划链在 8.31 已验证：`ComputePathToPose → /plan` 通，全程不发速度。

**对比结果与结论**

- **`use_sim_time` 在 nav2 里默认是 `'true'`**。实机 `/clock` 发布者为 0，
  于是六个节点的时钟**恒为 0、永不前进**，而 costmap 照发、判据照过。
  我的脚本绕过 `nav2_full_bringup.launch.py`（它会正确推导）直调内层 launch，就中招了。
- **一个已实证的假成功**：六阶段全报 `[OK]`、脚本打印「全部阶段完成」，而那一刻
  `map→odom` 和 `map→camera_init` **都不存在** —— nav2 有 costmap 发布者，但根本无法定位。
  根因：判据只查「costmap 有没有发布者」，而 costmap 节点起来就会发，**与能不能定位无关**。

**未验证**

- rviz 的 Navigation 2 面板显示 `Feedback: aborted`。我没发过任何导航目标，来源未查。
- `navigation.launch.py` 的 `use_sim_time` 默认值是否该翻成 `'false'`。已标记，未改 ——
  会改变共享 launch 行为。
- B2 底盘滑行距离。**需发速度**。

---

### M5 · 状态桥与底盘写通路

`astribot_trajectory_bridge`

**已做**

- 修复底盘写通路的**回调饿死**缺陷（已提交 `f23bf84`）。
- 启动脚本新增 `answer_sdk_prompt()`：SDK 的控制权提示会**阻塞在 stdin**，
  只发回车、**绝不发 yes**。

**已验证**

- `/joint_states` **50.1 Hz**（8.31）/ **43 Hz**（9.1），22 个关节：
  躯干 4 + 双臂 7×2 + 夹爪主动 1×2 + 头 2。
- `/odom` **49.96 Hz**，静止状态 `行程=0.000m 跳变=0`，66 万帧无异常。
- **控制权提示阻塞的完整因果链已实测**：状态桥卡在 stdin 约 3 小时 →
  `/joint_states` 0 Hz → 无动态 TF → 自滤剔除 0 点 → `/scan` 不出。

**对比结果与结论**

- **`cmd_vel` 订阅回调被 250 Hz 内环饿死**：两者共用一个互斥回调组，订阅回调
  **一次都执行不到**，机器人静止且**全程无告警**（leash 与看门狗都不报）。
  互斥组数量是 executor 线程数的下限 —— **只拆组不加线程等于没改**。
- **阶段⑥原来在发 `echo yes`**，而按提示文本，`yes` 会**立刻停止机器人当前运动** ——
  这与脚本自己「绝不触碰运动」的前提直接矛盾。已改为只发回车。
- **SDK 根本不暴露轮子转角**：`whole_body` 是 22 DOF、**不含底盘**；底盘在厂商侧是
  3 个虚拟关节 `astribot_chassis_x/y/z_rot`。这一点决定了 M6 那个问题**无法靠改配置解决**。

**未验证**

- A4 使能瞬间 3.3 cm 跳变。你明确说先跳过。
- C7 `name=` remap 让容器内两节点同名，**两个 yaml 全失效**、参数回落代码默认值。未修。
- C8 `StatusReporter` 的 `node_name` 是字面量，与真实节点名不匹配。未修。
- C9 实机 numpy 1.24.0 vs 钉住的 1.21.5。未修。
- **今天新发现**：`cmd_vel_body_to_world_node` 有**两个实例**在跑（PID 1173742 / 3360292），
  同时订阅 `/odom`。这是**写通路**上的重复节点。未处理。

---

### M6 · 机器人模型 `RobotModel: Status: Error`（今天专项查清）

**已做** —— 按「先证再断言」的顺序逐层排除，不猜。

**已验证**

| 假设 | 实测 | 结论 |
|---|---|---|
| mesh 资源找不到 | 34/34 全部解析成功 | ✅ 排除 |
| URDF 解析失败 | 51 link 全部读出 | ✅ 排除 |
| 某些 link 没有 TF | **4 个 / 51 个失败** | ❌ 命中 |

**rviz 自己给出了答案**（把 Displays 面板的 RobotModel 展开后截图，不是我推断的）：
面板里**恰好 4 条**红色错误项，内容为 `No transform from [wheel_LF_L…]`，
四个分别是 `wheel_LF_Link` / `wheel_LR_Link` / `wheel_RF_Link` / `wheel_RR_Link`，
其余 47 个 link 全部 `Transform OK`。

```
URDF 可动关节        36
  ├─ /joint_states 里有   22   ← 躯干4 + 臂7×2 + 夹爪主动1×2 + 头2
  ├─ mimic 从动关节       10   ← RSP 按 URDF mimic 关系算出，TF 正常
  └─ 无任何来源            4   ← wheel_{LF,LR,RF,RR}_Joint  ✗ 无 TF
```

**对比结果与结论**

- **根因：轮关节是真实的 `continuous` 关节、没有 `mimic`，而实机上没有任何数据源
  提供它们的角度。** RSP 收不到就不发这 4 条 TF，rviz 的 `TFLinkUpdater` 对每个
  查不到变换的 link 报 **Error**。
- **这不是配置遗漏，是数据不存在。** SDK 的 `whole_body` 22 DOF 不含底盘，底盘只以
  3 个虚拟关节暴露 —— 轮子的**自转角**厂商侧从来没有这个量。`bridge.yaml` 里已写清。
- **为什么仿真里看不到这个错**：Gazebo 侧 `astribot_s1_ros2_control.xacro` 把 4 个轮关节
  声明为 effort 关节，`joint_state_broadcaster` 会发它们的状态。
  **这是一个纯粹的仿真/实机不对称**。
- **功能影响：目前是纯显示问题。** 已逐个确认下游：自滤的 link 清单
  （`pointcloud_slice_scan_params.yaml`）**不含**任何轮 link；nav2 用足迹多边形而不是 link。

**未验证**

- **MoveIt 会不会因此拒绝规划 —— 这是预测，不是结论。**
  机制清楚（`CurrentStateMonitor` 要求模型内所有 active 关节都出现在 `/joint_states`，
  轮关节是 active），且 URDF 自己在 `astribot_s1_ros2_control.xacro:100-103` 记录了
  同类症状（`The complete state of the robot is not yet known. Missing …`）。
  但 **`move_group` 现在没在跑，我没有测过**。这是待验证的潜在实机阻塞项。

---

### M7 · 可视化通路

**已做**

- 方案 A：机器人侧直接跑 rviz（NoMachine 连物理桌面 `:0`）；
  方案 B：x11vnc 只绑回环 `127.0.0.1:5900`，画面走 ssh 隧道。
- `tools/robot/vnc_view.sh` + 两篇教程文档。
- 启动脚本清掉 ssh 转发过来的 `LC_*`，改 `LC_ALL=C.UTF-8`。
- 选用 `nav2_view.rviz` 前先**剥掉注释再 grep**，确认 `Tools:` 段只有
  `Interact/MoveCamera/Select/SetInitialPose`，**没有 `SetGoal`**、`goal_pose` 出现 0 次 ——
  工具栏里没有任何东西能让机器人动。

**已验证**

- rviz 在机器人 `:0` 上渲染正常，**19 fps**，Fixed Frame `map`，costmap 与机器人模型都出图。
- `/map` 的 QoS 双侧一致 —— rviz 里的地图/代价地图**是真实数据**。

**对比结果与结论**

> **更正我先前的说法**
> 我在展示截图时说白色扇形条纹是「`/scan` 的多层切片投影」。**这是错的。**
> 今天逐个查端点 QoS 发现：`/scan` 发布端 `BEST_EFFORT`，而 **rviz 订阅端是 `RELIABLE`** ——
> **rviz 的 LaserScan 显示一帧都没收到**，日志里有对应的 `incompatible QoS` WARNING
> （`/odom` 同样）。那些条纹来自别的显示项。nav2 侧不受影响（它订阅端就是 BEST_EFFORT）。

- rviz 必须直接 GLX。`ssh -X` 走不通（Ogre 建不出 GL 窗口，100 次重试后 core dump），
  offscreen 也救不了。
- **我连着三次报「rviz 死了」，实际它一直在跑。** 我的进程模式写的是 `lib/rviz2/rviz2`，
  而真实命令行是 `rviz2 -d …` —— 模式根本匹配不上，我把「匹配不到」读成了「进程不存在」。

**未验证**

- rviz 侧 `/scan` / `/odom` 的 QoS 需要在 `.rviz` 配置里显式设成 BEST_EFFORT。已定位，未改。

---

### M8 · 启动脚本与环境

`tools/robot/bringup_stages.sh` —— 六阶段扩到七阶段

**已做**

- 新增**阶段⑦**：转换节点 → 预处理 → 融合 → 切片 → `/scan`，每一步**按拍率**验证。
- 新增 `check_ldd()` / `check_tf_edge()` / `check_nav_clock()` / `answer_sdk_prompt()` / `wait_for_rate()`。
- 阶段④改为 `/odom` 就绪后才启动 `map_odom_tf`。

**已验证** —— 五个启动期环境坑，全部定位并写进脚本

| 症状 | 真因 |
|---|---|
| `Package 'voxel_slam' not found` | SLAM 是独立工作区，不在 `ws_robot` 下 |
| `libmetis-gtsam.so` 缺失（exit 127） | 自建 GTSAM 4.1.0 需**前置** `LD_LIBRARY_PATH` |
| `ModuleNotFoundError: astribot_sdk` | `PYTHONPATH` 缺 `$SDK_ROOT` |
| `ValueError: Invalid robot type` | 厂商 environ 里没有 `ROBOT_TYPE` |
| `TypeError: str expected, not NoneType` | 缺 `ASTRIBOT_SDK_ROOT`，**报错文本里不含任何变量名** |

**对比结果与结论**

- **刻意不 source `$SDK_ROOT/env.sh`**：它会因本机是 `192.168.0.11`（非它认的 `.10`）
  而生成并导出一份 interfaceWhiteList，**覆盖厂商的 `fastdds_udp.xml`**。
  厂商双网卡隔离用的是多播路由 + iptables，不该再叠一层（本项目已吃过亏：
  话题能列出来但 pub 恒 0）。只抄变量，不 source。
- **守卫要数 launcher，且 kill launcher 不杀子进程**：`kill 87653` 之后 RSP `88044`
  被 init 收养继续跑，一直在发旧的占位雷达位姿。
- **厂商的 `ros2` CLI 只有 `bag/daemon/node/param/service`，没有 `topic`**。
  PATH 里它排在 `/opt/ros/humble` 前面，所以 `ros2 topic ...` 会以 `invalid choice` 失败 ——
  排查时容易误读成「话题不存在」。

**未验证**

- Fast DDS **endpoint 级**发现失效（已记录 `927e689`，**未解决**）：
  实测过「节点发现到了但话题 pub 恒 0」。
- 机器人侧 `flake8` / `xmllint` 失败，因为 `src/astribot_s1_autonomy/` 里有别人 8-20
  留下的嵌套 `install/build/log`。不是我的产物，我没删。

---

### M9 · 测试与文档

**已做 / 已验证**

- `test_livox_custom_convert.cpp` —— **27** 条（纯逻辑，不依赖 livox 包）
- `test_source_age.py` —— **28** 条
- `test_map_odom_tf_wiring.py` —— **12** 条
- 文档：实机链路验证记录、DDS 发现失效记录、VNC/rviz 两篇教程、收尾计划书、
  数据链与基准文档（含更正）。

**对比结果与结论**

- **一条变异测试活下来了，证明我的注释是错的**：我写「无符号相减会绕回」，
  实际两补数 uint64 相减再转回 int64 **得到正确的负值**。真正的危险是
  **无符号 → 浮点**（会得到约 1.8e10 秒）。注释已更正，并补了一条算术测试。
- **我自己的一条测试失败，是测试的错不是代码的错**：样本点 `(0.001,0,0)` 距原点 1 mm，
  被默认空回波规则正确剔除。

**未验证**

- **78 个文件未提交**，包含 9.1 的全部工作。本机与机器人两棵树已经分叉。
  这是最大的过程风险。

---

## 三、当前卡点

### P0 · 厂商控制驱动下线，带走我们全部 3 个 SDK 进程

这是**现在就在发生**的故障，不是历史记录。9-01 21:26:01 实测：

```
state_bridge_node     Driver heartbeat timeout 5/5 → Driver crashes. Exiting...
chassis_odom_reader   Driver heartbeat timeout 5/5 → Driver crashes. Exiting...
bridge_container      Driver heartbeat timeout 5/5 → Driver crashes. Exiting...
                      ↑ 三者引用同一个 Last error_code_timestamp: 1788269160.069
```

| 证据 | 实测 |
|---|---|
| 阈值与响应时间 | 5 次心跳超时 ≈ **1.2 s** 后退出 |
| 爆炸半径 | 3 个 SDK 进程全死；**0 个**非 SDK 进程受影响 |
| `/astribot_error_code/control_driver` | 发布者 = **0** |
| `/astribot_*/joint_space_states`（6 个） | 发布者全 = **0** |
| `bridge_container` 日志 | 出现过一次 `Driver recovered` 又立刻复发 —— 是**抖动**而非干净停止 |

**级联后果（逐环实测）**

```
/joint_states  0 Hz  (原 43)
   └→ robot_state_publisher 停发连杆 TF
        └→ 切片节点：「本帧有 36 个连杆 TF 查询失败」「自身剔除 0」(原 76~80)
             └→ /livox/cloud_self_filtered  0.56 Hz  (原 8.7)
                  └→ /scan  0.56 Hz  (原 9.9)   ← 动态避障再次失效
```

SLAM（9.99 Hz）、转换节点（9.2 Hz）、融合（9.5 Hz）、nav2、RSP 全部存活 ——
死的**恰好且仅有**持有 SDK 会话的那三个。

> **最先该排除的可能：你手上那个物理急停按钮。**
> 如果 21:26 前后按下过，控制驱动停止是**完全正常**的表现，这条 P0 就不是缺陷而是预期行为。
> 请先确认这一点，再往下查。
>
> 无论哪种原因：**在厂商控制驱动恢复之前重启我们那 3 个进程是没有意义的** ——
> 它们会在 1.2 s 内以同样的原因再次退出。

### P1 · 已定位、待处理

| # | 问题 | 影响 | 状态 |
|---|---|---|---|
| 1 | `cmd_vel_body_to_world_node` **双实例** | 在**写通路**上。两个节点同时转译 `cmd_vel`，后果未评估 | ❌ 安全相关 |
| 2 | 4 个轮 link 无 TF（`RobotModel Error`） | 当前纯显示；**MoveIt 可能因此拒绝规划（未验证）** | ⚠️ 待决策 |
| 3 | rviz 侧 `/scan` `/odom` QoS 不兼容 | rviz 收不到这两个话题；nav2 不受影响 | ⚠️ 一行配置 |
| 4 | `/livox/right/cloud_filtered` 慢 20% | 把融合从 10 Hz 拖到 8.1 Hz | ⚠️ 未查 |
| 5 | nav2 `Feedback: aborted` | 来源不明，可能是历史遗留 | ⚠️ 未查 |
| 6 | 78 个文件未提交 | 丢失即无法重建；两棵树已分叉 | ❌ 过程风险 |
| 7 | D3 本身无自动重启 | 心跳一抖就静默全停，无人值守时不可用 | ⚠️ 未修 |

### P2 · 需要你授权才能推进

> **铁律 —— 不会被技术前置条件替代**
> **每次使能前、每次发速度前都要与你确认朝向和距离。**
> `/cmd_vel` 无帧、电量足、leash 生效这些**不构成授权**。机器人前方曾被你标为危险方向。

- **B2 底盘滑行距离** —— 停车距离分两段（积分器的惯性尾巴 + SDK `control_way='filter'`
  的再收敛），判据只能压在积分器上。
- **C4 SLAM 回环跳变影响** —— 必须走出一个回环才测得出。与 B2 同一批运动。

---

## 四、后续计划

### 第 0 步 —— 先恢复，再谈别的

1. **确认 21:26 前后是否按过急停**（你回答）。这一步决定后面是「恢复流程」还是「查厂商缺陷」。
2. 确认厂商控制驱动回来：判据是 `/astribot_error_code/control_driver` 发布者 > 0
   **且** `/astribot_*/joint_space_states` 发布者 > 0。
   *不能只看进程在不在 —— 进程活着而话题 pub=0 这个组合本项目实测过。*
3. 再按阶段③④⑥重启我们的 3 个 SDK 进程，并用**拍率**验证 `/joint_states` → `/scan`
   整条链恢复到 9~10 Hz、自滤剔除量回到 76~80 点/帧。

### 第 1 步 —— 不需要授权、可立刻做

| 任务 | 判据 | 依赖 |
|---|---|---|
| 查清 `cmd_vel_body_to_world_node` 双实例的来源 | 找到两个 launcher，确认哪个该留；确认双发对写通路的实际后果 | 无 |
| rviz 配置里把 `/scan` `/odom` 的 Reliability 显式设成 Best Effort | rviz 日志不再出现 `incompatible QoS`，LaserScan 显示有点 | 无 |
| 提交 78 个文件（**等你明确要求**） | 分几个语义清晰的提交；`.coverage` 加 ignore | 你的指令 |
| 查 `/livox/right/cloud_filtered` 慢 20% 的原因 | 找到瓶颈在哪一环并给出实测对比 | 第 0 步 |
| 查 nav2 `Feedback: aborted` 来源 | 能说出是哪个目标、什么时候、为什么 | 第 0 步 |

### 第 2 步 —— 需要一次决策

**4 个轮关节没有数据源，怎么办。** 三条路，我不替你选：

| 方案 | 好处 | 代价 |
|---|---|---|
| **A** 由桥接补发 4 个轮关节的**常量 0** | TF 补齐，rviz 报错消失，顺带挡掉 MoveIt 那个潜在阻塞 | 在状态话题里写了一个**我们并不知道的值**。轮子不会转，视觉上不真 |
| **B** 实机 URDF 里把轮关节改成 `fixed` | 语义最诚实：「实机不建模轮子自转」 | 仿真/实机 URDF 分叉；MoveIt 的 `joint_limits.yaml`/SRDF 还引用这些关节名 |
| **C** 不动 | 零风险 | rviz 一直红；MoveIt 上实机时可能直接被卡住（未验证） |

**先做的应该是验证，不是选方案**：起一次 `move_group`，看它到底会不会因为缺这 4 个关节
而拒绝规划。测出来是「会」，A 就成了必选项；是「不会」，C 完全可以接受。
**这一步不涉及任何运动。**

### 第 3 步 —— 需要你在现场授权

1. B2 滑行距离（需确认朝向和距离）
2. C4 回环影响（与上一条同批运动）
3. A4 使能瞬间 3.3 cm 跳变（你先前说跳过，可随这批一起量）

### 第 4 步 —— 收尾

- C7 / C8 / C9 三个已记录未修的缺陷。**C7 优先** —— 它让两个 yaml 全部失效，
  现在「参数是安全的」这个保证其实来自代码默认值，改 yaml 反而无效。
- D3 加自动重启/看护（先决定策略：是重连还是整栈重起）。
- `navigation.launch.py` 的 `use_sim_time` 默认值是否翻成 `false`。
- 请标定者确认 `chassis_extrinsic` 的 yaw 来路。
- 清理仍引用旧雷达位姿的文档（如 `technical_operations_manual.md:172-173`）。

---

## 附：本文的可信度约定

标「已验证」的都有当次会话可复现的实测数字；标「未验证」/「预测」的**请勿当结论使用**。

本次报告中我主动更正了**两处先前说错的话**（rviz 里的 `/scan`、「rviz 反复退出」），
以及**沿用旧记录的一处错误结论**（`xfer_format` 互斥）。
