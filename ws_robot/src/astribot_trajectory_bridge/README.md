# astribot_trajectory_bridge

厂商 SDK 与 ROS2 规划栈之间的唯一桥接层。包含**状态桥接**（`/joint_states`）与
**控制桥接**（底盘 / 手臂 / 夹爪）两部分。

## 当前状态（2026-08-27）

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
| Topic | `/cmd_vel` → 底盘 | 位置积分开环 + leash |
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
