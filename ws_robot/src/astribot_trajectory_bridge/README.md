# astribot_trajectory_bridge

厂商 SDK 与 ROS2 规划栈之间的唯一桥接层。包含**状态桥接**（`/joint_states`）与
**控制桥接**（底盘 / 手臂 / 夹爪）两部分。

## 控制职责调整（2026-09-14）

桥接层已删除 SLAM 比例校正、期望地图位姿积分及校正量切片叠加。
路径与到点控制由上层 Nav2/ThreePhaseController 负责。SDK 桥接的 x/y/yaw
按下面的逐帧 pose 规则同步积分。`enable_slam_correction` 默认 false；
它只兼容已删除的比例校正开关，不控制新的 pose 基准机制；true 明确拒绝。
旧校正增益参数已从配置中删除。`VelTrace.corr_path` 暂保留兼容字段，固定为零。

2026-09-15 起，底盘速度限幅与加减速限制统一由导航控制器和 velocity_smoother 执行。
桥接已移除 `max_vel_xy`、`max_vel_theta`、`max_accel_xy`、`max_accel_xy_up`、
`max_accel_theta` 及对应限幅函数，不再二次限制上游速度指令。真机和仿真使用导航基线运动参数，
到位运动参数共享 `astribot_s1_navigation/config/arrival_motion.yaml`，
到位阈值通过 `arrival_precision_hardware.yaml` / `arrival_precision_sim.yaml` 区分定位源。

桥接保留 SDK 位置积分、坐标适配、指令超时、扫描时效联锁、位置误差 leash、
使能与身份写入闸门，以及调度停顿时的积分步长保护。上游应发送已经过导航平滑器
处理的速度；直接向桥接输入速度时，不再有桥接层的速度和加速度限幅。
低频 tick 只做位姿健康检查与漂移诊断，不再改写位置指令；其检查不再受旧校正开关影响，
`require_slam_to_enable=true` 时定位丢失的停车闩锁仍有效。

此前删除 SLAM 校正时通过纯 Python 假 SDK/时钟的 700 拍差分回归（与旧 correction=false 指令一致），
并检查定位扰动不叠加位置、位姿丢失、扫描丢失、指令超时、leash、停用、SDK 读失败及
退役参数拒绝。证据在 `/tmp/astribot-bridge-dedup/`；没有启动机器人或发送 ROS 运动指令。
下面为历史验证记录，不代表本轮真机复验。

## 逐帧 pose 三轴积分（2026-09-15）

使能时，读取 SDK **实际** `[x,y,yaw]` 和 TF `map → astribot_torso_base`，建立反馈基准。
正常运动时，`SDK 目标 = 最新反馈基准 + 有界(本帧速度积分 + 速度 × 前瞻时间)`，
yaw 做角度归一化。前瞻用于适配厂家 `set_joints_position(control_way='filter')` 的跟随响应。
桥接每拍按实测 dt 积分三个轴，收到源时间戳严格递增的新 pose 时，三个轴的指令积分
在同一次操作中清零。相同时间戳的数据即使数值不同，也不重复换帧；乱序帧不刷新有效期。
延迟帧从被接收这一拍开始积分，不补算传输期间的历史指令。

SDK 示例按机器人局部方向累加 x/y，不能直接写入 SLAM 的 map 绝对坐标。
映射时将相邻 SLAM pose 的实测位移转换成本体行程，加入 SDK 反馈基准；yaw 仍使用
`wrap(SLAM yaw + 使能时角度零点差)`。帧间转弯按恒定本体 twist 的 SE(2) 对数映射处理，
避免将弧线的弦长当作行程。反馈基准随**测量位移**变化，上一帧未完成的**指令积分**不进入基准。
这个映射遵循仓库现有 SDK 示例口径，尚未通过转向后的真机位移复验；轴向和单位仍需一致。
两帧间快速往返或复杂转向无法仅凭端点恢复。重新使能应在静止时进行；
定位重置后应停用并重新使能，不能把新的地图零点当成真实位移。

启用前瞻的轴收到零速度时，先用 SDK 实际位置一次性撤掉提前量，之后保持该目标，
避免继续追赶旧位置或被 SLAM 静态噪声驱动。换向会丢弃本帧该轴旧方向的积分。

SDK 的 theta 是连续位置坐标。使能种子、历史插值、积分、零速保持及故障恢复均保留
其圈数，不能归一化到 ±π；否则跨界时会向位置接口发送近 2π 的反向阶跃。
SDK leash 使用连续目标与实际坐标之差；SLAM 帧间朝向及导航几何误差仍取最短角差。
原本零速的轴重新起动时，以 SDK 实际位置重新对齐该轴接口基准，避免很小的新指令
释放保持阶段积累的定位偏差；已在运动的其他轴不改基准。
缺失、过期、非有限或时间超前的 pose，以及控制时钟不前进时，三个轴均停止积累；
定位丢失的原有停车状态机继续生效。即使 `require_slam_to_enable=false` 允许无定位使能，
也不能无定位累计指令。SDK 读写失败的拍不提交三个轴的积分。

每秒的桥接速度链日志分别记录目标角变化 `dθ 指令`、厂家实际角变化、输入角速度积分、
`pose换帧`次数、三个轴的`本帧积分=(dx,dy,dtheta)`以及目标相对 SDK 实测位置的
`目标领先峰值`。换帧后的目标变化包含基准更新，
不能当成纯速度积分。

| 前瞻参数 | 初始值 | 含义 |
| --- | --- | --- |
| `pose_preview_xy_sec` | 0.5 s | 平移位置前馈时间，x/y 共用 |
| `pose_preview_theta_sec` | 0.5 s | 角度位置前馈时间 |
| `pose_preview_max_xy_m` | 0.20 m | 相对 pose 反馈基准的平移提前量欧式上界 |
| `pose_preview_max_theta_rad` | 0.34 rad | 相对 pose 反馈基准的角度提前量上界 |

有效提前量上界同时不超过对应 leash 的 95%，为测量误差留出余量；增大 leash
不会自动增大上述前瞻空间上界。前瞻时间设为 0 时，该轴回到此前仅本帧积分、零速保持旧目标的行为。
这些是位置接口的适配参数，不代替导航运动参数。本轮真机原地试验中，0.1 / 0.3 rad/s
输入的实际速度接近指令；0.6 rad/s 短平台实测约 0.55–0.57 rad/s。0.02 rad/s 能产生旋转，
但存在低速波动，区间平均约 0.012–0.014 rad/s。x/y 的前瞻响应仍只有离线验证。
定角度试验和位移指标的适用范围、原始数据见
`runs/bridge_preview_20260915_191703/REPORT.md`；此前纯逐帧积分的诊断见
`runs/hardware_inspect_20260915_190625/REPORT.md`。

指令超时仍逐拍检查；相同超时状态每秒报告一次，收到新指令后重新允许立即报告。

## 历史状态（2026-08-27）

控制桥接已在真实 MuJoCo 后端上建立会话并跑通。取证细节见
`docs/sim_real_alignment.md` §7.10，本文只讲**怎么用**和**已知会踩的坑**。

| 通路 | 状态 |
|---|---|
| 底盘 `chassis_cmd_bridge_node` | ✅ 真后端验证：指令落地跟随率 100.0%（误差 0.0 mm）、leash 触发与冻结、看门狗 0.298 s（阈值 0.30）、`reset_leash` 重取积分种子 |
| 手臂 `arm_traj_bridge_node`（`FollowJointTrajectory`） | ✅ 真后端验证：250 Hz 流式（静止底盘下跟踪误差峰值 0.0041）、越限拒绝、关节名顺序校验、取消保持、越限恢复 |
| 夹爪 `~/set_gripper` 服务 | ✅ 真后端验证：极性、换算、越界拒绝、阻塞时长；**仿真下夹持力不可验收** |
| 状态话题 `/astribot/bridge/status` | ✅ 29 个状态枚举，全程无意外故障位 |
| 离线测试 | ✅ **425 条**，约 15 次故障注入验证非空跑 |
| 底盘闭环（SLAM 校正） | ⛔ MuJoCo 无 SLAM，只跑过纯开环 + leash |
| "真的把物体夹起来" | ⛔ **未验证**，原因见 §7.10.6（IK 的 flag 会对失败报成功 + `world` 系随会话重置） |

## ROS 接口

| 类型 | 名称 | 说明 |
|---|---|---|
| Action | `<arm_group>/follow_joint_trajectory` | 方案 B：周期流式 `set_joints_position`，**可取消** |
| Service | `~/set_gripper`（`SetGripper`） | 对外用 `opening_fraction`（**1.0 = 全张开**），厂商的 0-100 极性翻转关在桥接内部 |
| Service | `~/dispatch_waypoints`（`DispatchWaypoints`） | 方案 A：`move_joints_waypoints`，**阻塞且不可取消**。默认关闭 |
| Service | `chassis ~/enable` / `~/disable` / `~/reset_leash` | 底盘使能与 leash 复位 |
| Topic | `/cmd_vel` → 底盘 | x/y/yaw 按同一 pose 帧同步重置指令积分 + leash |
| Topic | `/astribot/bridge/status` | 结构化故障上报，**不要只看日志**（见下） |
| Topic | `/astribot/chassis/odom_from_sdk` | `frame_id` 刻意是 `sdk_chassis` 而非 `odom` —— 漂移特性未取证前**不要接进 Nav2** |

## 用之前必须知道的六件事

1. **夹爪极性与直觉相反：0 = 张开，100 = 闭合。** 服务接口只收 `opening_fraction`
   （1.0 = 全张开）正是为了让上层不必记住这件事。换算 `rad = 0.0093 × cmd`，
   `cmd=100 → 0.93 rad` 恰为关节上限。别直接把 0-100 当百分比或弧度用。
2. **位置指令必须持续重发。** 单次下发抓不住正在运动的关节：取消轨迹时会继续漂
   0.045~0.37 rad；夹爪要求半开（cmd=50）时会**冲到 99.998 全夹紧**。
   桥接内部已按持续重发实现（HOLDING 相位 / `_stream_to`），但如果你绕过桥接
   直接调 SDK，这个坑还在。
3. **`max_tracking_error_rad: 0.10` 是按静止底盘定的。** 底盘动过之后手臂跟踪误差
   涨 2~5 倍（实测上界 0.42），搬运场景下这个阈值会误触发 abort。机制未定，
   见 §7.10.4 —— 用在搬运里请自行加大并记录依据。
4. **仿真下 `set_effector_max_force` 是空操作**，服务响应的 `force_applied` 会如实
   报 false。**别把仿真里的夹持行为当成力限已验收。**
5. **`rclpy.init()` 必须在建立 SDK 会话之前**，反了会抛
   `Context.init() must only be called once`（报错指向 rclpy，真因是 SDK 已初始化过）。
6. **实验前重启仿真、确认只有一个仿真进程。** 长跑的仿真会退化到读数超出 MuJoCo
   自己的硬限位（物理不可能），整套数据都会是假的 —— 本项目已被这件事骗过一次。

## 这一层在整条链路里的位置

```
上层业务/MoveIt  --(关节名 + 弧度)-->  本桥接  --(部件名 + 厂商量纲)-->  厂商 SDK
```

**它是整条链路上唯一的单位/命名换算点。** 上层只见 MoveIt 的关节名与弧度，
厂商接口只见部件名与它自己的量纲（夹爪还是 0~100 的抽象量），两边都不需要
知道对方的表示法。

## 状态桥接的三条硬边界

1. **不申请高控制权**（`high_control_rights=False`，代码里固定传，不提供参数）。
2. **只发主动关节**（22 个）。夹爪每侧 6 个关节里只有 `joint_L1` 是主动的，
   另外 5 个是 URDF mimic 从动关节，由 `robot_state_publisher` 算。
   桥接也发的话，同一自由度就有两个来源，一旦两边算法有出入就出现无法解释的
   姿态抖动（已实测 Gazebo 侧的 mimic 有 7.8° 稳态误差，正是这类出入）。
3. **读不到状态就退出，绝不发陈旧值**。发陈旧关节角比不发更危险：
   MoveIt 会拿它当规划起点。

## 两个工具，验证的是两件不同的事

不要混为一谈 —— 混了会产生"测试全绿所以映射没问题"的错觉，
而顺序恰恰是最容易错、错了又最难发现的那一项。

| | 验证什么 | 需要后端 |
|---|---|---|
| 清理前的离线回归（9 条） | 关节名在 URDF 里存在、是主动关节而非 mimic、夹爪 scale 与 URDF 限位自洽、无重复、不含底盘 —— 即"我自己有没有说错话" | 否 |
| `joint_map_probe` | 部件内**顺序**与 SDK 一致 —— 即"我说的话与厂商是否指同一台机器" | **是** |

### 顺序为什么必须探而不能读

SDK 的 `get_current_joints_position(names)` 返回**按部件成组的裸数组**，
第 i 个数对应哪个物理关节，SDK 没有任何地方声明；而核心是编译好的
`astribot_function.so`，**读代码得不到答案**。

顺序错了的后果很隐蔽：`/joint_states` 照样发、话题里也有 22 个值、RViz 里
机器人也在动，只是**姿态是错的**，而 MoveIt 会拿这个错姿态当规划起点。

探针的判据是**限位指纹**：臂的 7 个关节限位互不相同
（−3.1/3.1、−1.53/0.46、±3.1、−0.06/2.61、±2.56、±0.76、±1.53），
所以"按 bridge.yaml 顺序从 URDF 取的限位向量"必须与"SDK 返回的限位向量"逐项相等。
两边数据来源完全不同（URDF 展开 vs SDK 运行时），却本该指向同一台机器 ——
这是个不依赖我的假设的独立判据。

判据的边界（探针会自己报告）：限位相同的关节之间**区分不了**。
躯干四关节限位互不相同，可判定；头部两关节若限位相同则该部件报"不可判定"，
而不是假装通过。

## 一个会吞掉所有日志的坑（已在代码里绕开）

厂商 SDK 一被 import 就把**整个进程**的 fd 1/2 重定向到 `/dev/null`
（`astribot_interface.py` 顶部的 `quiet` 分支，用 `os.dup2`）。
它本意是掩掉底层 C 库刷屏，但 `os.dup2` 是进程级的，连带把本节点的
ERROR 日志一起吞了 —— "响亮失败"这条设计会彻底失效：失败了，但没人看得见。

实测：不设 `ASTRIBOT_LOG` 时，节点连不上后端就静默退出（除了
`Exited with failure 1` 一个字都没有）。

本包在 import SDK **之前**就把 `ASTRIBOT_LOG=1` 置上，而不是依赖运维记得 export。

## 环境

全栈统一 `ROS_DOMAIN_ID=25`（厂商 `env.sh` 设的就是 25，真机上 SDK 后端是既有
进程、domain 改不动，所以是本栈迁过去）。
桥接、后端、`robot_state_publisher`、RViz 必须在**同一个 domain**，
否则表现为"节点都在但话题一个都收不到"。

```bash
source /opt/ros/humble/setup.bash
source <repo>/env.sh                 # 设 PYTHONPATH / ROBOT_TYPE=S1 / DOMAIN=25
source <repo>/ws_robot/install/setup.bash
```

## 验证步骤

```bash
# 0. 先确认没有别的 /joint_states 发布者。Gazebo 的 joint_state_broadcaster
#    也发这个话题，两个发布者同时在，TF 会抖而两边都不报错。

# 1. 起厂商 MuJoCo 仿真（或激活真机）
export ASTRIBOT_SIMU_ROOT=<astribot_simulation 路径>
cd $ASTRIBOT_SIMU_ROOT && python3 astribot_simulation.py

# 2. 先跑探针，确认部件划分与关节顺序（**这一步不通过就不要往下走**）
ros2 run astribot_trajectory_bridge joint_map_probe --ros-args \
  --params-file <install>/astribot_trajectory_bridge/config/bridge.yaml \
  -p robot_description:="$(xacro <install>/astribot_s1_description/urdf/astribot_s1.xacro robot_name:=astribot_s1)"

# 3. 起桥接 + robot_state_publisher
ros2 launch astribot_trajectory_bridge state_bridge.launch.py

# 4. 按 Gate 2 验收标准比对
ros2 topic echo /joint_states --once      # 与 SDK get_current_joints_position() 逐关节比
rviz2                                      # 模型姿态应与 MuJoCo 画面一致
```

## 仿真后端的依赖（实测记录）

厂商 `scripts/lite_install/install_mujoco_noconda.sh` **不建议整个跑**：
它会 `pip install numpy==1.22.4` / `setuptools==64.0.0` 并往 `~/.bashrc` 追加两行，
有把现有能跑的 ROS2 栈搞坏的实际风险。

本机按最小增量装的（numpy 全程保持 1.21.5 未动）：

| 包 | 用途 |
|---|---|
| `mujoco==3.2.5` `glfw` `imageio` | MuJoCo 本体 |
| `gymnasium==1.1.1` | `src/astribot_envs/__init__.py` 顶层 import |
| `open3d` | `src/simu_utils/simu_common_tools.py:15` 顶层 import（只在深度图转点云那一个函数里用到，但模块级 import 躲不开）|
| `tabulate` | 示例 101 用 |
| `tf_transformations` | SDK 核心 .so 的硬依赖，**不在 PyPI**，只能 `sudo apt install ros-humble-tf-transformations`（已装） |
| `filterpy` | SDK 的 `whole_body_control.py:3123` 硬依赖。缺它时报的是 `No module named 'meta'`（**完全误导的名字**）。装时必须 `--no-deps`，否则会顶掉本项目钉住的 numpy 1.21.5 |

### 桥接周期与 SDK 耗时

使能期间用单调墙钟采集内环回调间隔、回调总耗时、控制锁等待、SDK 读/写调用耗时。
每约 10 秒在统一 `session.log` 输出一条 `BRIDGE_TIMING`，包括样本数、均值、P50/P95/P99、
最大值及有界缓冲丢样数。停用后保留的最后一个窗口会标明当时状态。
这些观测不改变积分使用的 ROS 时间或速度指令；SDK 调用耗时不等于底盘机械响应延迟。

`LOOP_OVERRUN` 仅统计使能期间，并在原有外环约每秒汇总次数及最大间隔，避免在 250 Hz
回调中逐条发布。停用时仍周期发布 `NOT_ENABLED`。周期统计中的窗口分位数不能直接当作
整段运行的分位数；汇总可使用：

```bash
python3 tools/robot/analyze_bridge_timing.py /path/to/session.log --output timing.json
```
