# astribot_trajectory_bridge

厂商 SDK 与 ROS2 规划栈之间的唯一桥接层（仿真-实物对齐方案 Gate 2/3）。

## 当前状态：Gate 2 代码就绪，**在线验证被依赖缺失卡住**

| 项目 | 状态 |
|---|---|
| 状态桥接节点 `state_bridge_node` | ✅ 写完、能构建、无后端时按设计"响亮失败" |
| 映射表 `config/bridge.yaml` | ✅ 22 个主动关节，6 个部件 |
| 离线一致性测试（9 条） | ✅ 全过，且经注入故障验证非空跑 |
| 关节顺序探针 `joint_map_probe` | ✅ 写完，**未运行**（需要 SDK 后端）|
| `/joint_states` 与 SDK 逐关节比对 | ⛔ **未做**：SDK 起不来，见下 |

### 阻塞点：SDK 缺 `tf_transformations`，且只能用 apt 装

```
File "astribot_function.py", line 12, in init astribot_function
ModuleNotFoundError: No module named 'tf_transformations'
```

这不是本桥接的问题 —— 直接跑厂商自己的 `examples/101-get_joint_states.py`
报的是同一个错。该模块**不在 PyPI 上**（`pip install tf-transformations` 报
"No matching distribution found"），只能：

```bash
sudo apt install ros-humble-tf-transformations
```

本机 sudo 需要密码，所以这一步得由你来执行。

**刻意没有做的事**：没有自己写一个 `tf_transformations` 兼容模块糊上去。
那个模块涉及四元数/欧拉角的顺序约定，我的实现与真实实现只要有一处约定不同，
就会静默产出错误的位姿 —— 而这类错误在 `/joint_states` 层面完全看不出来。

装好之后按下面「验证步骤」跑一遍即可。

## 这一层在整条链路里的位置

```
上层业务/MoveIt  --(关节名 + 弧度)-->  本桥接  --(部件名 + 厂商量纲)-->  厂商 SDK
```

**它是整条链路上唯一的单位/命名换算点。** 上层只见 MoveIt 的关节名与弧度，
厂商接口只见部件名与它自己的量纲（夹爪还是 0~100 的抽象量），两边都不需要
知道对方的表示法。换算规则全在 `config/bridge.yaml`，代码里没有数值常量。

## Gate 2 的三条硬边界

1. **不申请控制权**（`sdk_high_control_rights: false`）。这是只读方向的物理边界：
   没有控制权，即使桥接有 bug 也不可能让机器人动。配置层和代码层各有一道检查。
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
| `test/test_joint_map.py`（9 条） | 关节名在 URDF 里存在、是主动关节而非 mimic、夹爪 scale 与 URDF 限位自洽、无重复、不含底盘 —— 即"我自己有没有说错话" | 否 |
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

厂商 `env.sh` 会设 `ROS_DOMAIN_ID=25`（而本仓库 ROS2 栈其它部分用 42）。
桥接、后端、`robot_state_publisher`、RViz 必须在**同一个 domain**，
否则表现为"节点都在但话题一个都收不到"。

```bash
source /opt/ros/humble/setup.bash
source <repo>/env.sh                 # 设 PYTHONPATH / ROBOT_TYPE=S1 / DOMAIN=25
source <repo>/ws_robot/install/setup.bash
```

## 验证步骤（装好 tf_transformations 后）

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
| **`tf_transformations`** | **SDK 核心 .so 的硬依赖，只能 apt，见上** |
