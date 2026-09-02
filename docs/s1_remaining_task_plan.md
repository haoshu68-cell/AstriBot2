# S1 实机收尾计划书

> 2026-09-01 · 分支 `chassis-effort-drive` · 六阶段栈已起、TF 树已接通、写通路仍为拒绝态
>
> 所有"实测"数字均来自本机器人当次会话，可复现。标注"需确认"/"无法证明"的条目
> **请勿当作结论使用**。

## 当前一句话状态

SLAM + 感知 + 状态桥 + 里程计 + nav2 + 桥接六个阶段全部起来了，`map→odom` 已连续
78020 次更新、0 跳变，机器人正确定位在地图原点。

但**感知的自滤/切片链一个进程都没在跑**（`/scan` 与 `/livox/cloud_self_filtered`
发布者数均为 0），所以**现在没有动态避障**。这是最大的功能缺口，且它的前置条件
正是 P0 那一条。

---

## P0 — 一条命令，卡住两件事

### 01. 重启 robot_state_publisher，让新雷达位姿与 `livox_frame` 生效

孤儿 RSP `88044`（`ppid=1`，已跑 2h+）还在发**旧的占位雷达位姿**。它原来的父
launcher `87653` 已终止，但它被 init 收养后活了下来。

```bash
# 机器人终端
kill 88044
```

之后由我执行：用新 URDF 起 RSP，并验证 4 条边。**预期值**（已推导并交叉校验过）：

| TF 边 | 期望 xyz | 期望 yaw |
|---|---|---|
| `torso_base→livox_mid360_left` | 0.17393 0.16893 0.082 | −45.84° |
| `torso_base→livox_mid360_right` | −0.18120 −0.17733 0.166 | 135.96° |
| `livox_mid360_left→livox_frame` | 恒等 | 0° |
| `map→odom` | 重启期间会短暂缺失，之后必须恢复 | |

**重启前基线已记录**，好证明改动真的生效而不是我说生效：当前 TF 里读到的是
`0.28 0.18 0.10 / 22.918°`，且 `livox_frame` 查询抛 `LookupException`。

- 依赖：无 ｜ 阻塞：02 03 04
- 风险：重启期间 TF 断几秒，nav2 短暂丢变换；RSP 不涉及任何运动
- 自动模式两次拦下 `kill`，理由正确（那进程不是我起的，整栈 TF 依赖它），不绕

---

## P1 — 感知链：现在等于没有避障

### 02. 把自滤/切片链拉起来并验证输出

用可靠计数法（模式文件与读进程表分两次 shell 调用）实测，以下**全部为 0**：

| 进程 | 数量 | 后果 |
|---|---|---|
| `pointcloud_slice_scan` | 0 | 无 `/scan`、无 `cloud_self_filtered` |
| `livox_fusion` | 0 | 两路点云未融合 |
| `frontier_explorer` | 0 | 无自主探索 |
| `exploration_coordinator` | 0 | 同上 |

这几个节点**不在** `bringup_stages.sh` 的六个阶段里 —— 脚本从设计上就没启动它们，
所以"全部阶段完成"并不意味着感知可用。要么给脚本加阶段⑦，要么明确写清由谁启动。

- 依赖：01（自滤靠 TF 摆放点云，没有 `livox_frame` 就查不到变换）

### 03. 验证假点团被自滤剔掉

无回波点在雷达设备内被外参一起变换，于是堆成一个固定坐标的密集点团：

| 话题 | 占比 | 堆积坐标 | 距原点 |
|---|---|---|---|
| `/livox/lidar_back` | 34.0%（81574/239904） | (0.001, −0.496, 0.084) | 0.496 m |
| `/livox/lidar_front` | 33.4%（80128/240000） | (0, 0, 0) | 0 m |

两处都落在机器人足迹内（footprint 自滤半径 0.42 m），**理论上应当被剔掉** ——
但必须实测确认，因为本项目已栽过一次同类问题（夹爪不在自滤链里，机器人把指尖
当障碍，探索 0 次派发）。

- 依赖：02 ｜ 判据：自滤输出里这两个坐标附近的点数必须为 0

### 04. 重新评估 D1（`/scan` 缺失）的真正原因

原记录写的是"`xfer_format` 1-vs-2 互斥导致无法出 `/scan`"。**这个结论需要修正**：
实测厂商驱动发的是 `livox_ros_driver2/msg/CustomMsg`（`xfer_format=1`），而我们的
切片链吃 `PointCloud2`。所以问题不是"互斥无解"，而是**中间缺一个转换环节**，或者
需要另起一个 `xfer_format=0` 的驱动实例。哪条路可行没有验证过，不要照抄旧结论。

- 依赖：02 ｜ 注意：旧记录有误，先证再改

---

## P1 — 启动脚本的判据太松

### 05. 阶段⑤判据加上 `map→odom` 与 `map→base` 可解

本次实测暴露了一个**假成功**：六个阶段全报 `[OK]`、脚本打印"全部阶段完成"，而
那一刻 `map→odom` 和 `map→camera_init` **都不存在** —— nav2 有 costmap 发布者，
但根本无法定位。

根因是阶段⑤只查 `/global_costmap/costmap` 与 `/local_costmap/costmap` 有没有
发布者。costmap 节点起来就会发，与能不能定位无关。

已做的部分：阶段④已改为在 `/odom` 就绪后才启动 `map_odom_tf`，并新增
`check_tf_edge` 真查 TF 边（而不是查 `/tf` 有没有发布者 —— 那永远 >0）。
阶段⑤的判据还没加。

- 文件：`tools/robot/bringup_stages.sh`

### 06. 进程守卫要数 launcher；kill launcher 不杀子进程

两个已实证的坑：

- 阶段③的守卫只数 `state_bridge_node`。老 launcher 的节点因 SDK 报错死了、
  launcher 还活着，守卫看到 0 就**又起了一个 launcher**。
- `kill 87653`（launcher）之后 RSP `88044` **没有跟着死**，被 init 收养继续跑
  —— 这正是 P0 那条卡住的原因。要用进程组（`kill -- -PGID`）或显式清理子进程。

---

## P2 — 需要发速度才能推进（必须先与你确认朝向和距离）

### 07. B2 滑行距离实测

底盘停车距离分两段（积分器的惯性尾巴 + SDK `control_way='filter'` 的再收敛），
判据只能压在积分器上。**未测**，因为需要真实发速度。

> **铁律**：每次使能前、每次发速度前都要与你确认**朝向和距离**。技术前置条件
> （`/cmd_vel` 无帧、电量足、leash 生效）**不构成授权**。机器人前方曾被你标为
> 危险方向。

### 08. C4 SLAM 回环跳变的影响评估

历史记录里回环修正最大 0.626 m。本次**测不出来**：机器人静止，`map_odom_tf`
78020 次更新里跳变 0 次。要评估必须让机器人走出一个回环。

- 依赖：07 同一批运动

---

## P2 — 已知缺陷，有记录未修

| 编号 | 问题 | 后果 | 状态 |
|---|---|---|---|
| A4 | 使能瞬间 3.3 cm 跳变 | 位置突变 | 你明确说先跳过 |
| C7 | `name=` remap 让容器两个节点同名 | 两个 yaml 全失效，参数回落代码默认值。"不动"的保证其实来自代码默认值，改 yaml 加强安全是反效果 | 未修 |
| C8 | `StatusReporter` 的 `node_name` 是字面量 | 与真实节点名不匹配 | 未修 |
| C9 | 实机 numpy 1.24.0 vs 钉住的 1.21.5 | 版本承诺与现实不符 | 未修 |
| C6 | `led_controller_node` 曾以 13.4 Hz 刷 `/rosout`（91% 流量） | 日志淹没 | **需重新确认**：实测它现在没在跑 |
| D3 | 厂商心跳失效会杀掉我们所有 SDK 进程 | 无自动重启，静默全停 | 未修 |

---

## P2 — 新 SLAM 配置的遗留项（需要标定者确认）

### 10. `chassis_extrinsic` 的 yaw 恰好是旧 URDF 值的 2.0001 倍

配置里的旋转角是 `0.800058 rad`（45.84°），而旧 URDF 的占位 yaw 是 `0.4 rad`
（22.92°）。比值 **2.000146**。这是"把 yaw 当半角构造四元数"这个经典 bug 的
典型特征。

**但我无法证明。** 反证是平移与 URDF 差 0.106 m，说明不是直接抄的；而且
0.800058 ≠ 2×0.4 精确值（差 4e-5），更像某个标定流程的输出。更关键的是：
六条几何交叉校验**全部通过**（离地差 2.5 mm、两路地面差 0.6 mm、间距差 7 mm、
高度差一致、连线中点 5.5 mm），说明这组数就算来路可疑，**几何上是自洽且正确的**。

- 需要：标定者确认这是标定结果还是推导结果 ｜ 紧急度：低（不影响当前正确性）

### 11. 三处会误导后人的地方

- `save_path: /home/pf/SLAM/sessions/` —— 这台机器上没有 `/home/pf`，只有
  `/home/astribot`。`is_save_map: 0` 时不炸，**一旦置 1 就炸**。
- 源码注释 `"world-frame z is used directly as height-above-ground"` **现在是
  误导的**：世界系锚在底盘中心、离地 0.095 m，偏移藏在 `nav_scan_z_min/max` 的
  数值里（`−0.046 = 0.05 − 0.096`、`1.534 = 1.63 − 0.096`）。
- `docs/slam_nav_dataflow_and_datum.md:228` 说"真实 frame 是
  `livox_mid360_left/right`" —— 错的，实测两路 `frame_id` 都是 `livox_frame`。

---

## P3 — 仓库卫生

### 12. 63 个文件未提交

分支 `chassis-effort-drive`，`git status` 63 项改动，**包含本次全部工作**
（dt 修复、回调组重构、A2/A3、URDF 雷达位姿、`livox_frame`、启动脚本）。
另有 `ws_robot/src/livox_ros_driver2` 未跟踪、`ws_robot/.coverage` 应忽略。

还有若干文档仍引用旧雷达位姿（如 `docs/technical_operations_manual.md:172-173`），
建议随提交一并清理。

- 风险：这么多改动未提交，一旦丢失无法重建

---

## 已完成 — 本轮已在实机验证的部分

列在这里是为了划清边界 —— 下面每一条都在实机上验证过，不是"改完了应该没问题"。

### A2 陈旧 TF 检查

已部署并**抓到一次真实事件**：`camera_init→aft_mapped 已经 1.66s 没更新
（上限 1.00s）`，并正确拒绝发布。不是造出来的测试数据。之后 78020 次更新里
没有复发（陈旧计数稳定在 1）。

机制：tf2 只在某 frame pair 有**新**数据时才修剪缓冲，停更的 pair 会永久返回
最后一条记录，且 `lookup_transform(..., Time())` 一直"成功"、不报错。SLAM 挂掉时
会算出一个随机器人移动而反向漂移的 `map→odom`。

### A3 `map→camera_init` 恒等边

改为构造时即发、走 `/tf_static`（latched）。旧代码把它挂在 `_tick` 里且只发一次
动态 TF，里程计没起时这条边压根不存在，晚启动的消费者永远收不到。

实证：修复前实测该边**不存在**（TF 树是两座孤岛），修复后为**静态**边。

### URDF 雷达安装位姿（本机 + 机器人）

原值是 xacro 宏的占位默认值，与实机差 **0.108 m / 68.8°**。已按厂商标定修正并
六项交叉校验。`install/` 是 `--symlink-install` 一路链回 `src`，不需重编。

关键认知：驱动通过 `SetLivoxLidarInstallAttitude` 把外参**写进雷达设备本身**，
所以 `/livox/lidar_back` 的点已经在 front 系里，上层不要再变换一次。

### SLAM 基准高度（修正了我先前的错误答案）

`camera_init` 原点离地 **0.095 m**，不是我先前说的 0.017 m —— 所以你在 rviz 里
"SLAM 基准不在地面"的观察是**对的**。错因：我拿 `tf2_echo` 读到的 z=0.100 当
"实测"，但 TF 只是把 URDF 里的占位值原样发出来。**量一个由错值生成的量，不构成
对那个值的验证。**

### 启动链环境：五个坑已定位并写进脚本

| 症状 | 真因 |
|---|---|
| `Package 'voxel_slam' not found` | SLAM 是独立工作区，不在 `ws_robot` 下 |
| `libmetis-gtsam.so` 缺失（exit 127） | 自建 GTSAM 4.1.0 需前置 `LD_LIBRARY_PATH` |
| `ModuleNotFoundError: astribot_sdk` | `PYTHONPATH` 缺 `$SDK_ROOT` |
| `ValueError: Invalid robot type` | 厂商 environ 里没有 `ROBOT_TYPE` |
| `TypeError: str expected, not NoneType` | 缺 `ASTRIBOT_SDK_ROOT`，报错文本里不含任何变量名 |

**刻意不 source `$SDK_ROOT/env.sh`**：它会因本机是 `192.168.0.11`（非它认的
`.10`）而生成并导出一份 interfaceWhiteList，覆盖厂商的 `fastdds_udp.xml`。
厂商双网卡隔离用的是多播路由 + iptables，不该再叠一层。只抄变量，不 source。
