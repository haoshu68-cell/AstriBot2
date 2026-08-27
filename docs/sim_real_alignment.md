# 规划层 / 控制层 仿真-实物对齐方案

> **已定决策（2026-08-20）：全面转投厂商栈，砍掉导航。**
>
> 仿真用厂商 MuJoCo 环境，控制走 astribot_sdk，仿真与实物**由构造对齐**
> （同一个 SDK、同一套 WBC，只有物理引擎不同）。
> 移动作业能力（nav2 / SLAM / 探索 / `mobile_transport` 的导航段）
> ~~退出范围~~ —— **2026-08-25 已推翻，导航回到范围内，方案见 §7。**
> 依据：真机有两颗 Livox（SDK `activate_lidar()`），底盘是完整全向
> （`joint_types: [2,2,1]`）。原判断只对 MuJoCo 后端成立，不对目标系统成立。
>
> **前提（已定）：以 `examples/` 为准。** 示例与其他材料冲突时一律以示例为准。
> 末端为**夹爪**：1 DOF，0~100 无量纲，100=全闭 / 0=全开。
>
> 被否方案与理由见附录 A。差异实测数据见附录 B。

---

## 0 · 真值源规则（这一节必须先读）

厂商 `astribot_config/` **内部就没有单一真值** —— 它 ship 了四份互不一致的模型。
所以"以厂商配置为准"这句话不成立，必须先定"以哪一份为准"。

规则很干净：**以各部件 yaml 的 `model:` 字段指向的东西为准。**
这是 SDK 自己的解析入口，按定义就与真机一致。

| 部件 yaml | `model:` 指向 |
|---|---|
| `astribot_arm_left.yaml` | `model/astribot_arm_left.urdf` |
| `astribot_arm_right.yaml` | `model/astribot_arm_right.urdf` |
| `astribot_torso.yaml` | `model/astribot_torso.urdf` |
| `astribot_head.yaml` | `model/astribot_head.urdf` |
| `astribot_chassis.yaml` | **内联** `model{}` 块 |
| `astribot_gripper_left/right.yaml` | **内联** `model{}` 块 |

**`model/astribot_whole_body*.urdf` 那几份一个都没被引用** —— 它们是给别的用途
（可视化、动力学标定）准备的，不是 SDK 的运行模型。

> ⚠️ 我的 `astribot_s1_description` 恰恰是从 `astribot_whole_body_with_wheel.urdf`
> 抄的（xacro 头部注明了来源，且逐项核对**完全一致**，没有任何漂移）——
> 抄得很忠实，但**抄的是一份 SDK 不使用的模型**。这是 Gate 1 要修的根本问题。

---

## 1 · 架构：只剩一个新增组件

砍掉导航之后架构大幅简化。原本需要 ros2_control 的理由（nav2 要 `/cmd_vel`、
底盘要轮速）全部消失，剩下的只有臂/躯干/夹爪的轨迹执行。

```
┌──────────────────────────────────────────────────┐
│ 业务层 · 对 target 完全无感知                     │
│   定点双臂搬运 / 装配场景                          │
└───────────────────────┬──────────────────────────┘
                        │
┌───────────────────────▼──────────────────────────┐
│ 规划层 · MoveIt2 + OMPL                          │
│   RRT* / BIT* / Informed RRT*（本工程插件）        │
│   碰撞校验 · 奇异点监视 · 时间参数化                │
│   模型直接用厂商 URDF ← 单一真值源                 │
└───────────────────────┬──────────────────────────┘
                        │ FollowJointTrajectory
┌───────────────────────▼──────────────────────────┐
│ astribot_trajectory_bridge  ◀── 唯一新增组件      │
│   把 MoveIt 轨迹转成 SDK 调用                     │
└───────────────────────┬──────────────────────────┘
                        │ astribot_sdk (Python)
              ┏━━━━━━━━━┻━━━━━━━━━┓
    ┌─────────▼────────┐  ┌───────▼──────────┐
    │ MuJoCo 仿真进程   │  │ 真机 192.168.0.10 │
    └──────────────────┘  └──────────────────┘
      同一套 SDK、同一套 WBC —— 对齐由构造保证
```

### 关键性质：切换不是我代码里的参数

SDK 面向 ROS 图工作，**它连的是"当前图上有谁"**。仿真是一个独立进程
（`python3 astribot_simulation.py`），真机是另一端。所以：

- 我不需要实现任何 sim/real 分支，**厂商机制本身就是分叉点**。
- 为满足"启动参数切换"的使用体验，外层包一个 launch 参数即可：
  `target:=sim` 顺带把 MuJoCo 进程拉起来，`target:=real` 不拉。
  **这个参数只被 launch 文件读到，一行代码都不读它。**

### 唯一新增组件的职责

`astribot_trajectory_bridge`：一个 `FollowJointTrajectory` 动作服务端。

| 输入 | 输出 |
|---|---|
| MoveIt 的 `FollowJointTrajectory` goal（关节名 + 时间戳路点） | SDK `move_joints_waypoints(names, waypoints, time_list)` |
| — | 或按控制周期流式 `set_joints_position(names, cmd, control_way="direct", use_wbc=False)` |
| SDK `get_current_joints_position()` 等 | `/joint_states` |

它是 **target 无关的** —— 因为 target 由"哪个后端在跑"决定，桥接不需要知道。

两种下发方式要选一个，见决策 D2。

---

## 2 · 范围变更

### 退出范围（不删代码，停止维护）

> **⚠️ 本表已被 2026-08-25 的决策部分推翻，见 §7。**
> 导航**回到范围内**。下面这 6 个包里，除 `gazebo_bringup` 外全部重新成为主线；
> 依据是：真机**有**两颗 Livox 雷达（SDK `activate_lidar()` +
> `/livox/lidar_front` `/livox/lidar_back`），且底盘是完整全向
> （`joint_types: [2,2,1]`），与我们仿真侧同构。
> 原判断"MuJoCo 无 LiDAR 所以导航没意义"只对 MuJoCo 成立，不对真机成立 ——
> 这是把"仿真后端的能力"误当成了"目标系统的能力"。

| 包 / 能力 | 原因（已过时，保留以便追溯） | 现状 |
|---|---|---|
| `astribot_s1_navigation`（nav2） | MuJoCo 无 LiDAR、无场景、无导航集成 | ↩ 回到主线（§7） |
| `astribot_s1_perception`（slam_toolbox） | 同上，且 SLAM 依赖 `/scan` | ↩ 回到主线（§7.4） |
| `astribot_s1_autonomy`（探索协调器 / 多层切片） | 依赖 Livox 点云 | ↩ 回到主线（真机就是 Livox） |
| `astribot_s1_chassis_effort_drive` | 底盘不再由我驱动 | ⏸ 仍退出：真机底盘吃位置指令，力矩闭环只用于 Gazebo |
| `astribot_s1_dynamics_coupling` | 臂-底盘耦合限速，无底盘即无意义 | ↩ 回到主线，且升级为**前置项**（§7.7 第 1 条） |
| `astribot_s1_gazebo_bringup` | Gazebo 退场 | ⏸ 仍退出（但作为已验证资产保留） |
| `mobile_transport` 场景的导航段 | 退化为 `transport`（定点） | ↩ 保留完整的搬运→导航→放货 |

> **建议只标记不删除。** 这些是已经跑通并有实测记录的资产
> （探索建图 77.5% 自由 / 230.78 m²、四求解器横向验证、路径跟踪诊断节点）。
> 若将来导航需求回来，重建成本远高于保留成本。
> 在各包 README 顶部加一行"⏸ 已退出范围（2026-08-20 决策），见 docs/sim_real_alignment.md"即可。

### 保留并成为主线

| 包 / 能力 | 说明 |
|---|---|
| `astribot_s1_manipulation` | `DualArmPlanner`、碰撞校验、奇异点监视、时间参数化、OMPL 扩展插件（BIT*/Informed RRT* 注册）、`transport` / `transport_probe` 场景 |
| `astribot_s1_moveit_config` | SRDF、kinematics、`ompl_planning.yaml`（含两个扩展键的完整踩坑记录） |
| `astribot_s1_description` | **需重做**：改为引用厂商 URDF，见 Gate 1 |
| `path_tracking_diagnostics_node` | 逐段量速度链路，改造后可用于桥接层的跟踪诊断 |

---

## 3 · 关键决策

### D1 · MoveIt 是否保留 —— ✅ 保留

SDK 只有 FK/IK 和 `move_joints_waypoints`，**没有运动规划器**。
放弃 MoveIt 等于放弃全部规划能力：OMPL 采样规划、碰撞校验、奇异点监视、
TOTG 时间最优参数化，以及已经验证过的三个求解器。

已有实测支撑保留决策（四求解器在 `transport` 上各跑一轮，6/6 全过）：

| planner | 规划总耗时 | 轨迹总节拍 | 最差 σ |
|---|---|---|---|
| RRTConnect（基线，非最优） | 0.220 s | 2.179 s | 0.1150 |
| **BIT\***（本工程插件） | 0.221 s | 1.483 s | 0.1119 |
| RRT\* | 3.159 s | 1.421 s | 0.1110 |
| **Informed RRT\***（本工程插件） | 3.196 s | 1.552 s | 0.1143 |

耗时分档是 `optimization_budget_sec` 配置差异，不是规划器快慢；
同样停首解时 BIT\* 的首解节拍比 RRTConnect 好 32%。

### D2 · 轨迹下发方式

| 选项 | 评价 |
|---|---|
| **a. `move_joints_waypoints()` 整条下发** | ✅ **推荐（第一步）** |
| b. 按周期流式 `set_joints_position(direct, no-WBC)` | ⏩ 需要精确跟踪时再上 |

- **a**：整条轨迹一次交给 SDK，由它自己插值执行。桥接实现最简单，
  且示例 208（轨迹回放）就是这个用法，是厂商验证过的路径。
  代价是**放弃了对执行过程的实时控制**——中途无法修改，
  且 SDK 内部的插值方式与 MoveIt 的时间参数化可能不一致
  （这会成为一个新的对齐点，必须实测比对）。
- **b**：桥接自己按控制周期把 MoveIt 轨迹采样后逐点下发，
  等价于把 `JointTrajectoryController` 的职责搬到桥接里。
  跟踪最可控、与仿真行为最一致，但要自己处理时序、抖动、超时。

> 建议先用 a 打通，在 Gate 3 用实测跟踪误差决定是否需要 b ——
> 不要凭直觉选，这正是"控制层对齐"该拿数据说话的地方。

### D3 · WBC 用还是不用 —— ✅ 本决策下**可以启用**

这一条因转投厂商栈而**发生反转**，是本次决策带来的实质收益。

原方案（保留 Gazebo）里启用 WBC 等于主动制造仿真-实物差异，
因为 WBC 只存在于厂商侧。现在仿真和实物**都走同一个 SDK、同一套 WBC**，
启用它不再破坏对齐。

- 收益：拿到厂商的全身稳定能力（示例 211：推躯干时手臂末端仍保持在目标上）。
- 代价：WBC 会改写关节指令，所以 **MoveIt 规划出的轨迹不等于实际执行的轨迹**。
  碰撞校验、奇异点校验都是在原轨迹上做的，WBC 改写后的构型未被校验。

> **建议：先关闭（`use_wbc=False`），Gate 3 通过后再作为独立课题评估开启。**
> 理由不是对齐（对齐已不受影响），而是**校验有效性**：
> 现在这套代码的碰撞/奇异校验假设"下发的就是执行的"，
> 开 WBC 会打破这个假设。主配置里写明 `use_vendor_wbc: false` 并注明原因。

### D4 · 夹爪 —— ✅ 前提已定，风险已实测解除

**前提（用户已定）：以 `examples/` 为准，末端为夹爪，1 DOF，0~100，100=全闭。**

拉下 MuJoCo 仓库实测后，这条从"最大技术风险"变成"完全自洽"：

| 来源 | 末端形态 | 结论 |
|---|---|---|
| SDK API（示例 103/107） | `effector_left_name: [50.0]`，1 值 | ✅ |
| 示例 106 | 幅度 0~100（100 全闭），速度 0~1000 | ✅ |
| `astribot_gripper_left.yaml` | 单关节 `astribot_gripper_joint`，限位 **0~100**，力矩 145，速度 1000 | ✅ |
| `astribot_whole_body_with_gripper.sdf` | 每侧 6 物理关节 `L1/L11/L2/R1/R11/R2` | ✅ 仿真侧实现 |
| **MuJoCo 仿真** | 同时提供 gripper 与 hand 两套模型，**按需选择** | ✅ 风险解除 |

先前担心的"MuJoCo 只有 BrainCo 灵巧手"**不成立**：
仓库里 `astribot_s1_with_gripper.xml` 与 `astribot_s1_with_hand.xml` 并存，
灵巧手只是可选项。选 gripper 那套即与真机一致。

**MuJoCo 的 1-DOF 抽象是怎么实现的**（`astribot_gripper_left_actuator.xml` 实测）——
这正是我仿真侧要复刻的耦合关系：

```xml
<tendon><fixed name="gripper_left_split">
  <joint joint="astribot_gripper_left_joint_R1" coef="0.5"/>
  <joint joint="astribot_gripper_left_joint_L1" coef="0.5"/>
</fixed></tendon>
<actuator><general tendon="gripper_left_split" ctrlrange="0 100" forcerange="-200 200" .../></actuator>
```

`ctrlrange="0 100"` —— **与 SDK 的 0~100 逐字一致**。闭链由 `equality` 约束闭合：

| 约束 | 关系 |
|---|---|
| `R1 = L1` | 两指同步 |
| `R1 = R2` | — |
| `R11 = −R1` | `polycoef="0 -1 0 0 0"` |
| `L2 = −L1` | `polycoef="0 -1 0 0 0"` |
| `L11 = L1` | `polycoef="0 1 0 0 0"` |

物理关节行程 `range="0 0.93"` rad，对应控制量 0~100
—— **所以 0~100 与关节角的映射是 `100 ↔ 0.93 rad`**，
这顺带解决了原先"0~100 对应多少开口"这个未确认项（关节角层面已确定，
毫米级开口还需要连杆几何换算）。

**对外接口**：MoveIt SRDF 声明 `end_effector` + 一个 `gripper` 规划组，
1 个抽象关节（0~100），`GripperCommand` 驱动。业务层写 `gripper: 100` 就是闭合。
仿真侧用 mimic 按上表的比例展开到 6 个物理关节。


### D5 · 能力矩阵 —— ✅ 按"以 examples 为准"定案

| 能力 | 真机 | 仿真（**按示例定案**） | MuJoCo 仓库文档说的 |
|---|---|---|---|
| 位置控制 | 支持 | 支持 | 支持 |
| 速度控制 | 支持 | ❌ **不支持** | 声称支持 |
| 力矩控制 | 支持 | ❌ **不支持** | 声称支持，但另注 "does not guarantee sim-to-real accuracy" |

示例 106："Joints velocity control is **not supported in simulation and will not
respond to commands**"；示例 210 对力矩控制同样表述。
MuJoCo 仓库文档与之冲突 —— **按已定前提以示例为准，仿真侧一律按不支持处理**。

即使采信 MuJoCo 文档也不改变结论：它自己给力矩控制标了
"does not guarantee sim-to-real accuracy, mainly for simulation purposes"，
对"对齐"这个目标本来就没有价值。

**落地方式**：主配置里 `capabilities.sim` 把 velocity / effort 置 false；
桥接在初始化阶段比对 —— 请求了当前 target 不支持的接口就**拒绝启动**
并打出缺哪一项。静默不响应是最坏的失败形态（代码正常跑、日志干净、机器人不动），
必须换成启动期的响亮失败。

### D6 · `get_joints_position_limit()` 返回顺序 —— ✅ 定为 `(lower, upper)`

三个官方示例表面自相矛盾，但按"以 examples 为准"是可以判定的：

| 示例 | 解包方式 | 之后怎么用 |
|---|---|---|
| 100 | `upper_limit, lower_limit = ...` | **仅打印表头标签**，不做任何比较 |
| 103 | `lower_limit, upper_limit = ...` | `all(cmd > lower)` 且 `all(upper > cmd)` ——语义正确 |
| 210 | `lower_limit, upper_limit = ...` | `any(pos < lower)` 或 `any(pos > upper)` ——语义正确 |

两处**功能性**用法一致且语义自洽，唯一的分歧在示例 100 的装饰性表头。
判定：**返回 `(lower, upper)`，示例 100 的表头标签写错了。**
（Gate 0 仍会顺带用真值复核一次，但不阻塞。）


---

## 4 · 主配置

厂商 `astribot_config/robot_config/astribot_s1/` **就是主配置**，不再另建一套模型描述。
我这边只留一份很薄的桥接配置：

```yaml
# astribot_s1_bringup/config/bridge.yaml —— 唯一的自有配置
target: sim                      # sim | real，只被 launch 读到

backend:
  sim:
    # target:=sim 时由 launch 拉起的后端进程
    launcher: $(env ASTRIBOT_SIM_ROOT)/astribot_simulation.py
    param_file: simulation_mujoco_param.yaml    # 带相机的换 _with_camera
  real:
    launcher: null                # 真机侧不由我们拉起
    require_activation: true      # README："需要先激活机器人"

model:
  # 单一真值源，MoveIt 的 URDF 直接来自这里，不再手写一份
  # 单一真值源 = 各部件 yaml 的 model: 字段指向的东西（见第 0 节）
  # 绝不要指向 whole_body_*.urdf —— 那几份 SDK 一个都不加载
  source_dir: $(env ASTRIBOT_SDK_ROOT)/astribot_config/robot_config/astribot_s1
  resolve_via: part_yaml_model_field

frames:
  # TCP 必须显式：厂商 tool 是法兰再往 −y 0.15m，不是法兰原点
  # 数值来自厂商 yaml 的 effector_to_tool_pose，不手抄
  arm_left_tcp:  {parent: astribot_arm_left_link_7,  xyz: [0.0, -0.15, 0.0]}
  arm_right_tcp: {parent: astribot_arm_right_link_7, xyz: [0.0, -0.15, 0.0]}

bridge:
  dispatch_mode: waypoints        # D2：waypoints | streaming
  streaming_freq: 250             # 仅 streaming 模式生效
  use_vendor_wbc: false           # D3：开启会使碰撞/奇异校验失效，见文档
  control_way: direct

capabilities:                     # D5：**实测填写**，不抄文档
  sim:  {position: null, velocity: null, effort: null}   # Gate 0 后填
  real: {position: true, velocity: true,  effort: true}

safety:
  velocity_scale:     {sim: 1.0, real: 0.3}
  acceleration_scale: {sim: 1.0, real: 0.3}   # 加速度限位是估算值，真机务必压低

network:
  domain_id: 25                   # 全栈统一 25，与厂商 env.sh 一致（本栈原来用 42）
time:
  use_sim_time: {sim: true, real: false}
```

> **一个已经踩过的坑**：每个 `IncludeLaunchDescription` 都必须显式传 `params_file`。
> 第一个 include 会占用共享参数名，后续节点静默加载错误的 yaml——
> `/**:` 结构的文件加载时不报任何错，所有参数悄悄退回声明期默认值。

---

## 5 · 分阶段落地与验收门

### Gate 0 · 装环境 + 实测摸清后端（不写功能代码）

这一步是本决策的**前提验证**。

**已完成部分**（仓库已拉到 `/home/yjh/WorkSpace/astribot_simulation`，311 MB）：

- ✅ 前置条件核实：RTX 4090 / 驱动 580 / Ubuntu 22.04 / Python 3.10.12
  全部满足；磁盘 819 G；GitHub 可达。**仅缺 `git-lfs`**。
- ✅ 夹爪形态实测确认（见 D4）：gripper 与 hand 两套模型并存，
  gripper 侧 `ctrlrange="0 100"` 与 SDK 逐字一致，耦合关系已抄出。
- ✅ 无 LiDAR **从源码确认**（不只是文档）：全仓库 `grep -i
  "rangefinder|lidar|laser|livox|scan"` 在 xml/py/yaml 里零命中
  （仅 `src/sim_assets_tools/process_mujoco_xml.py` 里的目录扫描函数误命中）。
- ✅ 场景资产：只有 `simpleTable`（`simpleTable_asset.xml` + `_body.xml` + 贴图），
  无 warehouse / 房间。**对定点抓取来说这张桌子刚好够用。**
- ✅ 底盘：MuJoCo 里 **4 个轮关节 + 3 个虚拟关节**并存，
  虚拟关节是 `position` 执行器（`kp` 20000/20000/5000），与 SDK 的
  `[x, y, theta]` 位置指令对得上。

**剩余待测**（2026-08-25 复测了环境，情况比原来写的好一些）：

环境现状实测：`mujoco 3.2.5` ✅ 已装、`numpy` 仍是 **1.21.5** ✅ 未被动过、
网格资产 ✅ **已解析**（`left_base_link.STL` 2.5 MB 等，不是 LFS 指针文本），
所以原来写的"仅缺 git-lfs"这条已经不成立 —— `git-lfs` 未装但也**不需要**了。

真正卡住的是 SDK 自己起不来，见 Gate 2 的"阻塞点已经换了一个"：
少一条 PYTHONPATH + 缺 `libdmumps_seq-5.4.so`。下面 5 项全部依赖它先通。

1. **SDK 原样跑通性**（本决策的生死项）：跑 `examples/101`（读状态）、
   `103`（关节运动）、`107`（笛卡尔）。文档给的话题是
   `/astribot_arm_left/joint_space_command`，而 `astribot_msgs` 里是
   `AstribotControlCommand` —— 这两者是不是同一套接口，文档**没有明确承诺**。
2. **能力矩阵复核**：跑 `106` / `210`，确认速度/力矩确实不响应（D5 已按示例定案，
   这一步只是取证）。
3. **FK 一致性**：同一组关节角，比 SDK `get_forward_kinematics()`（示例 204）
   与 MoveIt FK，顺带定掉 B.5 那个基座 rpy 疑点。
4. **速率**：各部件 yaml 写 `frequency: 100`，示例用 `Astribot(freq=250)`，
   实测哪个是实际生效的控制周期（D2 选 streaming 时直接依赖）。
5. **关节命名核对**：MuJoCo 里是 `astribot_chassis_zrot`，
   厂商 yaml 里是 `astribot_chassis_z_rot` —— **少一个下划线**。
   虽然底盘已退出范围，但同类命名差异可能出现在别处，值得扫一遍全表。

> **通过标准**：`examples/101/103/107` 在 MuJoCo 上跑通；
> FK 位置偏差 < 1 mm、姿态偏差 < 0.001 rad；速率有定论。
>
> **若第 1 项不通过，本决策的前提就不成立**，必须回附录 A 重选。


### Gate 1 · 模型改为单一真值源 —— ✅ 已完成

- ✅ 限位改为**第 0 节规则指定的 per-part 模型**。躯干速度 6.0 → **1.8**，
  臂 joint_3 −3.0~1.4 → **±3.1**（B.1 的两个实质风险已解除）。
  只换限位、不动几何与惯量 —— 实测两份模型在关节原点与惯量上**逐项完全一致
  （0 处差异）**，所以没必要动。也不能直接 `xacro:include` 厂商 URDF：
  per-part 文件完全无 mesh，且同样缺 `base_link/link_1/link_2/link_6` 的 collision。
- ✅ 加 `*_tcp_link`（B.2）：法兰再往 −y 0.15 m，SRDF 各组 tip 与
  `manipulation_params.yaml` 的 TCP 全部指向它，**不再用 `tool_link` 当笛卡尔目标**。
- ✅ 补夹爪，但**最终形态与原计划不同**（见下）。SRDF 加了
  `gripper_left/right` 组 + `end_effector` + `open/closed` 命名状态。
- ✅ 一致性测试：`test_joint_limits_parity.py`（4 条，逐关节比对 per-part 文件）
  + `test_gripper_model.py`（27 条）。两者都用**注入故障**验证过非空跑。
- ✅ 控制链路：夹爪主动关节进 `ros2_control` / `joint_state_broadcaster` 白名单 /
  两个 `gripper_*_controller` / `moveit_controllers.yaml` / `joint_limits.yaml`。
  漏掉任何一处，move_group 会报 `Missing astribot_gripper_*_joint_L1` 并且
  **整个规划功能不可用**（不只是夹爪不可用）。

**夹爪建模与 D4 原计划的偏差（必须知道）**：
原计划是"6 物理关节 + 1 抽象关节（0~100）"。实际做成 **6 物理关节，其中
1 个主动（`joint_L1`，弧度 0~0.93）+ 5 个 mimic**，不引入 0~100 的抽象关节。
原因：抽象关节在厂商 yaml 里声明为 prismatic 0~100，若进 URDF 会让该组的
关节空间量纲混进"100 单位 vs 弧度"，而它带来的唯一好处（与厂商 wire format
同单位）本就该由桥接层承担。0~100 ↔ 弧度的换算放在
`astribot_trajectory_bridge`，作为整条链路唯一的单位换算点。
D4 表里的耦合比例经 MJCF `<equality>` 原文核对，**完全正确**：
`R1=L1`、`R2=R1`、`R11=−R1`、`L2=−L1`、`L11=L1`，行程 `100 ↔ 0.93 rad`
（`L1 = 0.0093·cmd`，由 actuator `gainprm 4.65 / biasprm -500` 反推）。
`0 = 张开`（依据 SDK 的 `open_effector` 下发 `0.0`、`close_effector` 下发 `100.0`）。

> **通过标准（已修正 —— 原标准不成立）**
>
> 原标准写的是"重跑 `transport_probe`，可用候选数应**明显增加**
> （臂 joint_3 放开 1.7 rad 之后必然如此）"。**这条判据是错的**：
> 实测同配置连跑三次得到 48 / 46 / 47 —— 该指标本身有 ±2 抖动，
> 与要观测的效果同量级。我拿噪声当信号，而且从没量过它的噪声底。
> 根因是 `probeTcpPose` 只调一次 `setFromIK`（随机重启解算器）且沿用了
> 闭链那套 `ik_timeout: 0.01`。
> 修法不是调参凑数，而是**改语义**：`50ms × 20 次`，任一次拿到合法解即算可达
> —— 点估计换成稳定下界。改后两次复测 **54 / 54 完全一致**，
> 且"IK 无解"稳定在 23（加大预算没减少它 = 这 23 个是真不可达）。
> 连带说明早先 README 里"80 个候选 50 个可用"是个软数字。
>
> 现行标准：
> 1. ✅ 两套一致性测试通过，且经注入故障证明非空跑。
> 2. ✅ 实测 ALWAYS 碰撞对（35 对）**全部**在 SRDF 的 disable 列表里
>    （缺 1 对就会让默认构型判自碰撞 → `There are no valid initial states!`）。
> 3. ✅ move_group 加载零错误：无 `not known to the URDF`、无 `Group is empty`、
>    无 `Missing astribot_gripper_*`。
> 4. ✅ 已重扫 `transport_probe` 并重定 A/B 与 `grasp_z_offset`（见下）。
> 5. ⬜ **未做**：确认躯干速度收紧到 1.8 后 `TrajectoryTimeOptimizer` 节拍变长。

**Gate 1 收尾时暴露的三个连带问题（都不是"重扫一遍"能解决的）**

1. **抓取余量的前置检查静默失效了。** `tcpCoincidentCollisionRadius` 的算法是
   "从 TCP 沿父链上溯，只要关节原点没平移就继续"，前提是 **TCP 与腕部连杆原点重合**
   （旧 TCP = `tool_link`，`tool_joint` 的 xyz 确实是 0）。TCP 改到 `tcp_link`
   （+0.15 m）之后，上溯第一步就终止，函数**恒返回 0**，那道"跑之前拒绝"的检查
   变成永远通过，却还在日志里报告一个看起来合理的下限。**这是我挪 TCP 时引入的回归。**
   已换成 `endEffectorSpan()`：把末端各连杆碰撞体 AABB 变换到 TCP 系、投影到
   真实抓取方向上量包络。
   顺带修正了判据的分工 —— 只保留**上限**（指尖伸出量 `reach`），下限交回碰撞校验：
   - 下限压成标量是错的：`setback` 量的是"TCP 背后最远的几何"，而背后是整条手臂，
     实测 `link_7` 一项就 0.21 m，算出的下限恒为负、永不触发。三维净空本就是
     碰撞检测的本职，而夹爪现在已完整进入碰撞模型。
   - 上限**必须**前置判断，因为碰撞检测查不出来：夹爪够不到物体时不发生任何碰撞，
     规划成功、执行成功，然后夹爪在物体上方闭合到空气里 —— 静默的错误结果。
2. **`grasp_z_offset = 0.15` 超了上限。** 实测 `reach = 0.0525 m`，上限
   = 物体半高 0.06 + 0.0525 = **0.1125**。旧值 0.15 意味着物体顶面在 TCP 前方
   0.09 m 而指尖只伸 0.0525 m —— 两指根本够不到。改为 **0.10**
   （= 物体半高 + 指尖行程一半，两侧都留余量）。
3. **抓取姿态四元数也作废了。** 旧值 `[0.6227,-0.0410,0.2927,0.7245]` 是照抄
   `ready` 位姿的 TCP 姿态得来的 —— 那时 TCP 就是法兰，姿态只要 IK 有解就行，
   因为法兰上没有任何几何。实测它的夹爪接近轴与竖直方向差 **28.6°**，
   斜插下去一根手指扎进物体：重扫的第一轮 **80 个候选全部不可用**，
   失败原因清一色 `environment collision: gripper_left_Link_R11 <-> probe_target`。
   新值按夹爪几何推导而非试出来：接近轴（TCP −y）→ 世界 −z、
   张合轴（TCP x）→ 世界 x，即绕 x 转 +90°，`[0.70710678, 0, 0, 0.70710678]`，
   回代验证与竖直夹角 0.0°。

**重扫结果（新模型 / 新姿态 / 新网格）**

网格也整体平移了：旧网格围着**法兰**取，而实测 `ready` 位姿下
法兰 `(0.1486, 0.3839, 0.8609)` → `tcp_link` `(0.2198, 0.3759, 0.7291)`，
即前移 0.071 m、下移 0.132 m，旧网格套在新 TCP 上是在扫一块偏离可达域 0.13 m 的区域。

| | 结果 |
|---|---|
| 可用候选 | **39 / 80**，两次独立重跑**逐点一致** |
| 选点依据 | 不是"两端点 σ 最高"，而是"A→B 整条 y 走廊全可用"（第 4 步是沿 y 平移）|
| 选中走廊 | `x=0.17, z=0.729`，5/5 全可用，σ 0.0931~0.1458（最差是阈值 0.02 的 **4.7 倍**）|
| 新 A / B | `(0.170, 0.456, 0.729)` / `(0.170, 0.276, 0.729)` |
| `transport` 全流程 | **6/6 SUCCESS**，规划总耗时 4.045 s，轨迹总节拍 3.525 s |

> ✅ **上一轮那个"σ=0.0077 却 PASS"的现象已查清 —— 推测成立。**
> 接 Gazebo 实跑 `execute:=true` 后，六步的最差 σ 变成
> **0.1156 / 0.1377 / 0.1380 / 0.1653 / 0.1658 / 0.1570**，
> 全程最差 σ = 0.1156 = 阈值 0.02 的 **5.8 倍**。
> 也就是说 0.0077 确实是**起始状态**（零位 = `joint_4=0` 肘部完全伸直）带进来的：
> `execute:=false` 时机器人从不移动，六步全都从同一个零位重新起步；
> 一旦真的执行，每步从上一步终点起步，σ 反映的就是真实轨迹。
> 奇异监视器从来没有"失效"，它报的是那条**真的起于奇异位形**的轨迹的最差值 ——
> 而机器人停在零位时，这个起点是无法回避的。

### Gate 1 全栈验证（Gazebo + RViz + 执行）

| 检查项 | 结果 |
|---|---|
| 控制器加载 | `gripper_left_controller` / `gripper_right_controller` 均 **Configured and activated** |
| `/joint_states` | **26** 个关节，含两个夹爪主动关节（5 个 mimic 从动关节正确地不在其中）|
| 夹爪实际动作 | 下发 0.93 rad → 主动关节到位 0.93000，action 返回 `SUCCEEDED` |
| `transport execute:=true` | **6/6 SUCCESS**，无 `deviates from current state` / `Invalid Trajectory` / `ABORTED` |
| 末端落点 | TF 实测 `(0.170, 0.276, 0.929)`，与第 6 步目标**逐位一致**；姿态 `(0.707, 0, 0, 0.707)` = 推导出的俯视抓取姿态 |
| 轨迹节拍 | 1.583 s（不执行时是 3.525 s —— 每步不再从零位长距离起步）|

**新测出的一个 sim/plan 差异：Gazebo 的 mimic 有 7.8° 稳态误差。**
此前记为"未实测"。测法是给主动关节下发 0.93 rad 后，把 Gazebo 内部连杆位姿
（`ign topic /world/default/pose/info`）与 TF（RSP 按 mimic 算的）逐个比：

| 连杆 | Gazebo vs TF | 说明 |
|---|---|---|
| `Link_L11` | 0.21 mm | 父关节是**主动**关节 L1，一致 |
| `Link_R11` | **7.47 mm** | 父关节 R1 是 mimic 关节，不一致 |

用 R11 实测位置反解：`R1(Gazebo) = 0.7940 rad`，mimic 应给 `0.9300 rad`，
差 **−7.79°**（残差 0.21 mm）。0.794 既不是 0（那会差 42.5 mm）也不像自由摆动的
随机值，所以 `gz_ros2_control` **确实在执行 mimic，只是像个带稳态误差的软约束**。

影响范围：MoveIt 侧的 mimic 是精确的（规划与碰撞不受影响，与厂商模型一致）；
Gazebo 侧右指比 MoveIt 认为的少闭合约 15%。当前 `transport` 没有夹取/attach 动作，
不依赖 Gazebo 接触物理，所以不进入结论 —— 但要做夹持力/接触实验前必须先解决。

### Gate 1 后续 · 在自主探索地图上跑真实夹爪搬运 —— ✅ 已完成

把「纯运动学演示」升级成「真开合夹爪 + 物体真 attach」，并且跑在**自主探索
自己产出的那张地图**上（定位模式，不再建图）。

**地图来源可追溯。** 探索是自然终止的，不是崩溃：剩余 366 个前沿格产生 23 个候选点，
全部被"目标点净空半径 0.25 m 内存在占据栅格"淘汰 —— 剩下的 20% 都在货架缝隙里，
机器人没有合法站位。状态机 `PAUSED → 3 次自动恢复 → 停止重试`，
符合"不做无限重试死循环"。覆盖 **243.2 m² / 80.1%**，落盘为
`ws_robot/maps/warehouse_explored_auto.{posegraph,data}` +
`warehouse_explored_auto_grid.{pgm,yaml}`。
定位模式起栈后实测 `/map` 为 289×420、origin (−7.369, −10.464)、已知 80.1%
—— 与落盘完全一致，证明用的是探索图而不是重新建的。

**夹爪张口量程：两条独立路径互相印证。**

| 来源 | 最大张口 |
|---|---|
| 早先按 L11/R11 的 x 区间手算（`[+0.055,+0.087]` / `[-0.075,-0.043]`） | 0.098 m |
| `GripperCommander` 用 FK + 碰撞包络现算 | **0.0975 m** |

两者差 0.5 mm。完全闭合时残留间隙 0.0060 m。
0.06 m 的物体 → 抓取角 **0.4664 rad**（理论张口 0.056 m = 0.060 − 0.004 预紧），
实测稳态角 0.4664 rad，偏差 0.0000。

**开合角不写死**：从 SRDF 的 `open`/`closed` group_state 读。
**抓取宽度不写死**：按抓取姿态四元数把张合轴转到物体系，取长方体在该方向的支撑宽度
——直接取 `size[0]` 只在"张合轴刚好映射到本体系 x"时成立，姿态一改就静默错。

**实测：不收臂导航必然失败；收臂后成功，但没有根治。** 这是本轮最有价值的发现。

| 轮次 | 导航前姿态 | 限速系数(实测) | 导航结果 | 耗时 | 距离 | 臂步数 |
|---|---|---|---|---|---|---|
| 第 2 轮 | 抬着物体不收臂 | **0.15**(下限) | **ABORTED** | 176.7 s | — | 3/6 |
| 第 3 轮 | 收臂到 `ready` | 0.29 | SUCCEEDED | 35.8 s | 2.76 m | **6/6** |
| 第 5 轮 | 收臂到 `ready` | 0.29 | SUCCEEDED | 158.7 s | 2.17 m | **6/6** |
| （复位用，臂未收） | 抬着不收臂 | 0.15 | **ABORTED**(码 6) | 180.6 s | 差 0.29 m 未收敛 | — |

失败时 `controller_server` 连报 7 次 `Failed to make progress`。
根因**不在 nav2**：臂-底盘耦合节点把"抬着物体"判成展开度 1.00，
限速系数落到下限 `min_speed_scale = 0.15`，那个速度下 10 s 走不满进度检查器要求的
`required_movement_radius = 0.5 m`，于是判卡住 → 恢复行为 → 用尽 → 放弃。
表面症状是"局部规划器走不动"，真去调 nav2 参数方向完全错。

**但收臂只是把它从"必然失败"变成"能成功"，没有根治**：第 5 轮虽然 SUCCEEDED，
中途仍触发了 1 次 `Running backup` + 3 次 `Failed to make progress`，靠 nav2 的
恢复行为兜过去，耗时是第 3 轮的 4.4 倍。因为 `ready` 的限速系数也只有 **0.29**
—— 耦合节点拿全零构型（奇异位形，`sigma_min = 0.0077`）当"收纳姿态"基准，
这个基准本身不合理。真正的修法在另一个包里，本轮刻意**没有**在这边绕过它
（既没放宽进度检查器，也没抬高 `min_speed_scale`）。

而**能收臂正是这次改用 attach 换来的**：旧代码物体用体系固定坐标，
一收臂物体就留在原地不动，所以旧注释明确写着"如果以后要在导航途中收臂，
这个前提就破了，必须改成 AttachedCollisionObject"。现在前提解除。

**完整流程实测（第 5 轮，`execute:=true`，起点非目标点，真的走了一段）**：
6/6 步全过，导航 SUCCEEDED 158.7 s，规划总耗时 3.461 s，
全程最差 σ = 0.1156（远高于 0.02 阈值），物体世界位移 **2.194 m**、底盘 2.169 m。

**attach 真的生效了，不是"发了消息就算"。** 从 `move_group` 自己发布的
`/monitored_planning_scene` 旁听（这一步必要：demo 自己打的日志只证明它**发出了**
那条差分，不证明 move_group 收下并挂上了，而这两者差的那一步会让规划器
其实不知道手上有东西——然后照样规划成功）：

```
出现在 robot_state.attached_collision_objects 的消息数 = 5
出现在 world.collision_objects 的消息数              = 3
挂载到的 link = astribot_arm_left_tcp_link
touch_links 数量 = 6
```

（旁听这个话题必须用 `VOLATILE` 订阅。第一次用 `TRANSIENT_LOCAL`，QoS 不兼容、
**一条消息都收不到**，只有一行 WARN 提示 —— 很容易被当成"没有 attach 发生"。）

**边界如实说清**（同时写进运行日志，不靠读者推断）：

- **真的**：夹爪按指令开合（Gazebo 里真在动，主动关节稳态误差 0.0000 rad）；
  物体被 attach 到 TCP，碰撞检测把它当机器人的一部分，规划器必须带着它绕障。
- **不是真的**：Gazebo 里没有该物体的刚体，不产生夹持力。物理夹持受上面那条
  mimic 7.79° 稳态误差与未标定指尖摩擦影响，是独立课题。
- 导航途中伸出的手臂与手上的物体都超出代价地图那个 0.42 m 外接足迹，nav2 看不到它们
  —— 这一条没有因为改用 attach 而改变。

**顺带修掉/查清的三件事**：

1. `setJointGroupPositions()` 只更新**组内** mimic 关节。SRDF 里夹爪组刻意只含主动
   关节，于是 5 个从动关节一个都不动，FK 量出的张口变化率只有真实值的一半、
   反解抓取角错一倍，且**全程零报错**。改用 `setJointPositions(master, ...)`
   （走 `master->getMimicRequests()`）。抓住它的是合成夹爪单测——张口宽度可口算，
   断言能写到 1e-9；拿真实 URDF 只能"跑出来多少断言多少"，这个错会照样绿。
2. 对没挂东西的 link 发 detach **不是**无害空操作，MoveIt 会打
   `[ERROR] Attached body 'xxx' not found`。日志里凭空多一条 ERROR 会误导下次排查，
   改成按标记只在确实挂着时补发。
3. `move_group` 退出时 SIGSEGV（`TrajectoryExecutionManager::~...` →
   `Node::~Node()` → `CallbackGroup::~CallbackGroup()`），**与本次改动无关**：
   两轮都出现，包括流程在导航段就中止、根本没走到收尾的那一轮。
   发生在 SIGINT 之后、所有工作已完成，属 MoveIt 2.5.9 关停期问题。
4. 删掉了死参数 `transport_clearance_margin`：它是下限概念，而下限已整体交给
   逐点碰撞校验，读进来没人用 —— 留着只会让人以为调它有效。

### Gate 2 · 桥接骨架（只读）—— 🟡 代码就绪，在线验证仍被后端起不来卡住

`astribot_trajectory_bridge` 先只做状态方向：SDK 读状态 → 发 `/joint_states`。
暂不接受任何轨迹。

| 项目 | 状态 |
|---|---|
| `state_bridge_node` | ✅ 写完、能构建、无后端时按设计"响亮失败" |
| `config/bridge.yaml` 映射表 | ✅ 6 个部件 / 22 个主动关节 |
| 离线一致性测试（9 条） | ✅ 全过，经注入故障验证非空跑 |
| `joint_map_probe`（顺序探针） | ✅ 写完，**未运行**（需后端）|
| `/joint_states` 与 SDK 逐关节比对 | ⛔ **未做** |

> **通过标准（不变）**：`/joint_states` 与 SDK `get_current_joints_position()`
> 逐关节一致；RViz 里模型姿态与 MuJoCo 画面一致。

#### ⛔ 阻塞点已经换了一个（2026-08-25 复测）

原来的阻塞点 `tf_transformations` **已解除**（已装在
`/opt/ros/humble/lib/python3.10/site-packages/tf_transformations/`）。
把 SDK import 链往下推之后暴露出**两个新的、都在厂商侧**的问题：

**① `env.sh` 少给一条 PYTHONPATH。**
编译好的 `astribot_function.so` 里是**顶层** import：

```
import robotics_library_py.robotics_library_base
```

而 `env.sh` 只把 `SDK_ROOT` 和 `third_party/astribot_ros_middleware_py`
放进 PYTHONPATH，没有放 `astribot_sdk/core/common`。
实测 `importlib.util.find_spec('robotics_library_py')` 返回 `None`。
报错是 `'robotics_library_py' is not a package`，指向"包结构坏了"，
而真实原因只是搜索路径少一条。补上 `astribot_sdk/core/common` 即可解析。

**② 缺一个原生库，仓库里和系统里都没有。**
路径补好之后，`robotics_library_py/__init__.py` → `librobotics_library.so` 缺依赖：

```
ldd librobotics_library.so | grep "not found"
    libdmumps_seq-5.4.so => not found
```

`third_party/drake/lib` 里没有，`/usr/lib` 里也没有，全仓 `find` 零命中。
apt 有现成包（`libmumps-seq-5.4`，候选版本 5.4.1-2），
**需要 sudo，得由你执行**：

```bash
sudo apt install libmumps-seq-5.4
```

这不是桥接的问题——厂商 `examples/101-get_joint_states.py` 在同一条链上失败。
**在这两条解决之前，Gate 2 的在线验证、Gate 0 的 SDK 跑通性验证都无法开始。**

（同样刻意没做：不自己写兼容层糊上去。缺的是数值求解器的原生实现，
不是能用 Python 替代的东西。）

#### 设计要点（三条硬边界）

1. **不申请控制权**（`sdk_high_control_rights: false`）—— 只读方向的物理边界：
   没有控制权，即使桥接有 bug 也动不了机器人。配置层与代码层各一道检查。
2. **只发 22 个主动关节**。夹爪每侧 6 个关节里只有 `joint_L1` 主动，
   另外 5 个 mimic 从动关节由 `robot_state_publisher` 算。桥接也发就成了
   同一自由度两个来源，两边算法一有出入就出现无法解释的姿态抖动
   （Gazebo 侧那 7.8° 稳态误差正是这类出入）。
3. **读不到状态就退出，绝不发陈旧值** —— 陈旧关节角会被 MoveIt 当规划起点。

#### 顺序为什么必须"探"而不能"读"

SDK 的 `get_current_joints_position(names)` 返回**按部件成组的裸数组**，
第 i 个数对应哪个物理关节，SDK 没有任何地方声明；核心是编译好的
`astribot_function.so`，**读代码得不到答案**。
顺序错了的后果很隐蔽：话题格式完全正常、RViz 里机器人也在动，只是姿态是错的。

`joint_map_probe` 的判据是**限位指纹**：臂 7 个关节限位互不相同
（−3.1/3.1、−1.53/0.46、±3.1、−0.06/2.61、±2.56、±0.76、±1.53），
所以"按 `bridge.yaml` 顺序从 URDF 取的限位向量"必须与"SDK 返回的限位向量"逐项相等。
两边数据来源完全不同（URDF 展开 vs SDK 运行时）却本该指向同一台机器 ——
判据不依赖我的假设。Gate 1 的 `test_joint_limits_parity.py` 保证"限位对"，
本探针用"限位对"反推"顺序对"，两条测试是串起来的。

判据边界（探针自己会报）：限位相同的关节之间区分不了。躯干四关节可判定；
头部两关节若限位相同则报"不可判定"，而不是假装通过。

#### 顺带修掉一个会吞掉所有日志的坑

厂商 SDK 一被 import 就把**整个进程**的 fd 1/2 重定向到 `/dev/null`
（`astribot_interface.py` 顶部 `quiet` 分支用 `os.dup2`）。本意是掩底层 C 库刷屏，
但 `os.dup2` 是进程级的，连带吞掉本节点的 ERROR —— "响亮失败"这条设计彻底失效：
失败了，但没人看得见。实测不设 `ASTRIBOT_LOG` 时节点连不上后端会静默退出
（除 `Exited with failure 1` 一个字都没有）。
桥接在 import SDK **之前**就置上 `ASTRIBOT_LOG=1`，不依赖运维记得 export。

#### 环境：domain 号（已统一 25）

原先厂商 `env.sh` 设 `ROS_DOMAIN_ID=25`、本仓库 ROS2 栈其它部分用 42，两边互不可见。
**现已全栈统一 25**（真机上 SDK 后端是既有进程、domain 改不动，只能本栈迁过去）。
桥接/后端/`robot_state_publisher`/RViz 必须同 domain，否则表现为
"节点都在但话题一个都收不到"。

#### 仿真后端依赖（最小增量，numpy 全程未动）

厂商 `install_mujoco_noconda.sh` **不建议整个跑**：它会
`pip install numpy==1.22.4` / `setuptools==64.0.0` 并往 `~/.bashrc` 追加两行。
按最小增量实装：`mujoco==3.2.5` `glfw` `imageio` `gymnasium==1.1.1`
`open3d`（`simu_common_tools.py:15` 模块级 import，躲不开）`tabulate`，
numpy 全程保持 **1.21.5**。剩下的就是那个只能 apt 的 `tf_transformations`。

### Gate 3 · 桥接写通路（D2 + D3）

加 `FollowJointTrajectory` 动作服务端，按 D2 的 waypoints 模式下发。
`use_wbc=false`。**真机首次务必 `velocity_scale: 0.1`、单臂、小幅度、有人守急停。**

> **通过标准**：同一条 MoveIt 轨迹在 MuJoCo 与真机上，
> 关节跟踪误差曲线形状一致、峰值误差同量级；
> 并据此判定是否需要升级到 streaming 模式。

### Gate 4 · 全流程对齐验收

**同一个脚本、同一份配置，只换 `target`，跑 `transport` 全流程**，对比量化指标。
现成工具：`[transport][summary]` 一行汇总 + 改造后的 `path_tracking_diagnostics_node`。

> **通过标准**：6 步全 SUCCESS；两个 target 的轨迹节拍、最差 σ 在容差内一致；
> **业务层代码 diff 为空**（这是"无感知"的唯一硬性证据）。
> 夹爪就位后，`transport` 应从"纯运动学演示"升级为真实抓取。

---

## 6 · 剩余问题总表（2026-08-25 汇总）

按"卡在谁手上"分组，因为这决定了能不能并行推进。

### 6.1 卡在你手上（需要 sudo，我做不了）

| # | 事项 | 命令 / 动作 | 卡住了什么 |
|---|---|---|---|
| A1 | 缺原生库 `libdmumps_seq-5.4.so` | `sudo apt install libmumps-seq-5.4` | ✅ **已解除**（2026-08-25 复测在位） |
| A2 | `/opt/astribot_ros` 目录不存在，SDK 日志层建不出来就 `terminate` | `install.sh:92-100` 那三行（`mkdir -p log`、`mkdir -p robot_config`、`chown -R $USER`） | **Gate 0 全部 5 项 + Gate 2 在线验证 + Gate 3 + Gate 4**。现在的总闸 |

`tf_transformations` 已装、`mujoco 3.2.5` 已装、`libmumps-seq-5.4` 已装、
网格资产已解析、`numpy` 仍 1.21.5 —— 除 A2 之外没有别的装机需求。`git-lfs` 不需要了。

> **注意 Gate 5（§7.6，底盘导航通路）不在这条链上**：它只依赖 SDK 能起来
> （即 A2）和真机/MuJoCo 底盘，不依赖 Gate 3 的臂轨迹下发，可以并行推进。

### 6.2 厂商侧问题（已定位，但要绕过或等厂商修）

| # | 事项 | 现状 | 影响 |
|---|---|---|---|
| B1 | `env.sh` 少一条 PYTHONPATH（`astribot_sdk/core/common`） | 已定位，可在桥接侧补 | SDK import 失败，报错却指向"包结构坏了" |
| B2 | Gazebo 的 mimic 有 7.79° 稳态误差 | 已实测量化 | **物理夹持无法做**；`/joint_states` 里看不见，只能反解从动关节 |
| B3 | 指尖 7 mm 两源不确定度（MJCF vs SDF 的 L11 销位） | 用测试钉住，未解决 | 抓取余量的精度天花板 |
| B4 | 手指惯量是占位值（0.03 kg 配 I=0.001，回转半径 0.18 m，物理上不可能） | 照抄厂商值并标注 | 动力学仿真不可信；纯运动学不受影响 |
| B5 | 左臂基座 rpy 前两位对调（见附录 B.5） | 未定 | 只能靠 Gate 0 的 FK 比对定掉，依赖 A1 |
| B6 | 速度/力矩控制"声称支持实则不支持"（D5） | 已按示例定案 | 需 Gate 0 第 2 项取证，依赖 A1 |

### 6.3 我这边可以立刻做的（不依赖 A1）

| # | 事项 | 为什么值得做 |
|---|---|---|
| C1 | 修臂-底盘耦合的 `folded_reference_rad` 基准 | **当前最痛的一条**：基准是全零奇异构型，任何可用臂姿都被限速；抬着物体导航必然 ABORTED，收臂到 `ready` 也只有 0.29、仍会触发恢复行为。改基准必须重做倾覆余量评估 |
| C2 | 躯干速度收紧到 1.8 后确认 `TrajectoryTimeOptimizer` 节拍变长 | Gate 1 唯一遗留的 ⬜ 项，一次实测即可 |
| C3 | `move_group` 退出时 SIGSEGV | 析构链问题（`TrajectoryExecutionManager::~...` → `CallbackGroup::~...`），与业务无关但每次都刷一屏，掩盖真实错误 |
| C4 | 给已退出范围的包在 README 顶部加"⏸ 已退出范围"标记 | 决策已定但没落到文件上，新人会误读 |

### 6.4 依赖 A1 的按序推进

```
A1 解除
  └─ Gate 0（5 项：SDK 跑通 / 能力取证 / FK 一致性 / 速率 / 命名核对）
       └─ Gate 2 在线验证（joint_map_probe 定顺序 → /joint_states 逐关节比对）
            └─ Gate 3 桥接写通路（FollowJointTrajectory，velocity_scale 0.1 起）
                 └─ Gate 4 全流程对齐（只换 target，业务层 diff 必须为空）
```

### 6.5 范围问题已定：导航回到范围内（2026-08-25）

原来这里记着一个待拍的问题。**已拍：导航回来**，完整方案见 §7。

连带的三处调整：

- §2 的"退出范围"表已标注部分推翻（`navigation` / `perception` / `autonomy` /
  `dynamics_coupling` 回归主线；`chassis_effort_drive` / `gazebo_bringup` 仍退出）。
- **C1（修臂-底盘耦合的 `folded_reference_rad` 基准）从"可选优化"升级为前置项**：
  真机 `joint_max_velocities` 只有 1.0 m/s，0.15 倍就是 0.15 m/s，
  比仿真更容易触发进度检查器。
- 新增 Gate 5（§7.6），可与 Gate 3 并行推进 —— 它只依赖底盘通路，不依赖臂轨迹下发。

### 6.6 A1 已解除，但后面还有一层（2026-08-25 复测）

`libmumps-seq-5.4` 已装（`/usr/lib/x86_64-linux-gnu/libdmumps_seq-5.4.so` 在位），
`libdmumps_seq-5.4.so => not found` 这条消失。import 链继续往下走，撞到第三层：

```
[ast_log_base.cpp] create log config directory failed, path: /opt/astribot_ros/robot_config/log
[ast_log_base.cpp] Failed to create directory: /opt/astribot_ros/log/python3 ... Permission denied
terminate called after throwing an instance of 'spdlog::spdlog_ex'
```

SDK 的 C++ 日志层把路径写死在 `/opt/astribot_ros/`，该目录**根本不存在**，
建不出来就直接 `terminate` 抛异常（不是降级到 stderr）。

这一条**不是缺陷，是厂商安装步骤没跑**——`install.sh:92-100` 自己就写着：

```bash
sudo mkdir -p /opt/astribot_ros/log
sudo mkdir -p /opt/astribot_ros/robot_config
sudo chown -R "$USER:$USER" /opt/astribot_ros
```

所以还需要你执行这三行（或整段跑 `install.sh` 的那一节）。
跑完之后 Gate 0 的 5 项才真正能开始。

---

## 7 · 导航回到范围：技术方案（2026-08-25 定，取代 §2 里"nav2 退出范围"）

### 7.0 结论先行：不用发明，厂商例程里就是官方做法

原来担心的"nav2 输出速度、真机底盘吃位置，中间怎么接"**厂商自己已经给了答案**。
`examples/202-chassis_joy_control_local.py` 与 `203-..._global.py` 做的正是速度→位置积分：

```python
freq = 250.0
astribot = Astribot(freq=freq)
rate = ast_astribot_middleware.Rate(freq)
pos_cmd = astribot.get_desired_joints_position([astribot.chassis_name])[0]
while ok():
    vel = joy_controller.get_vel()
    # 203 版本：先按当前 theta 把体系速度旋到世界系，再积分
    rot = [[cos(th), -sin(th), 0], [sin(th), cos(th), 0], [0, 0, 1]]
    ...
    pos_cmd[i] += vel_world[i] / freq
    astribot.set_joints_position([astribot.chassis_name], [pos_cmd])
    rate.sleep()
```

三个必须照抄的细节（都不是随手写的）：

1. **积分的种子是 `get_desired_joints_position`，不是 `get_current_...`**。
   在"期望值"上累加，跟踪误差就不会反馈进积分器；用"当前值"会随跟踪滞后一路蠕变。
2. **`set_joints_position` 是流式接口**，按 client `freq` 每周期发一次绝对位置设定点。
   与 `move_joints_position`（离线、带 duration、阻塞）是两套东西，导航只能用前者。
3. **local 与 global 的区别就是有没有那个旋转矩阵**。nav2 的 `cmd_vel` 是**体系**的，
   所以必须走 203 的路子。这和我们已有的 `cmd_vel_body_to_world_node` 是同一件事
   （它现在旋转是 OFF，因为力矩底盘吃体系 twist；真机分支要打开）。

### 7.1 底盘真值（`astribot_chassis.yaml`，按 §0 规则这就是真值源）

| 项 | 值 | 对方案的含义 |
|---|---|---|
| `joint_names` | `astribot_chassis_x / _y / _z_rot` | 注意是 `z_rot`（MuJoCo 里是 `zrot`，少个下划线，Gate 0 第 5 项要扫的就是这类） |
| `joint_types` | `[2, 2, 1]`（移动/移动/转动） | **完整全向**：x、y 独立 + 自转，与我们仿真侧的 X 型全向底盘同构，nav2 可以照常输出 `vy` |
| `joint_max_velocities` | `[1.0, 1.0, 2.0]` | **硬钳位**：vx ≤ 1.0 m/s、vy ≤ 1.0 m/s、ω ≤ 2.0 rad/s。nav2 的 `max_vel_*` 必须收进这个盒子里 |
| `joint_*_positions` | ±99999999 | 无位置限位 = 没有越界保护，跑飞了不会有人拦，靠 7.3 的 leash |
| `frequency` | 100 | 与 client 默认 250 不一致，Gate 0 第 4 项要实测哪个生效 |
| `weld_to_base_pose` 注释 | `world to chassis when slam pose is 0` | **关键**：SDK 的底盘 `[x, y, theta]` 是"SLAM 位姿"，即真机上**已经有一套自己的定位** |

### 7.2 坐标系决策：SDK 的底盘位姿当 `odom`，**不**当 `map`

这是整个方案里最容易做错的一步。SDK 那个位姿虽然注释写着 slam，但**不能**直接当
`map` 用：

- 我们的代价地图、前沿探索、`/map` 都建立在自己那套 slam_toolbox 上。
  两套 SLAM 同时声明 `map`，TF 树里就有两个父节点，表现是位姿反复跳。
- 厂商那套位姿的回环/重定位行为不可控也不可观测，它一跳，我们所有costmap 全部错位。

所以：

```
map   ──(slam_toolbox，用融合点云)──▶ odom ──(SDK 底盘位姿)──▶ astribot_torso_base ──▶ ...
```

- **`odom` ← SDK**：`get_current_joints_position([chassis])` 给位姿，
  `get_current_joints_velocity([chassis])` 给 twist，合成 `/odom` + TF `odom→torso_base`。
  它只需要**局部连续**，不需要全局正确 —— 这正是 odom 的定义，也正是厂商位姿能保证的。
- **`map` ← 我们自己的 slam_toolbox**：输入是融合后的点云切片 `/scan`，与仿真侧**完全同一套**。

这样一来 `odom` 之上的所有东西（nav2、探索协调器、代价地图、路径诊断）
**一行都不用改** —— 与 Gate 4 "业务层 diff 为空"是同一个判据。

### 7.3 指令通路：新增 `chassis_cmd_bridge_node`

`nav2 /cmd_vel（体系 Twist, 20Hz）` → 本节点 → `set_joints_position([chassis], [[x,y,θ]])（100~250Hz）`

七个必须项，缺一个都会出事：

| # | 机制 | 不做的后果 |
|---|---|---|
| 1 | **速率解耦**：nav2 20 Hz 发，本节点按 `freq` 高频积分，中间保持"最后一次速度" | 直接按 20 Hz 发位置设定点，底盘会一顿一顿走 |
| 2 | **看门狗**：`cmd_vel` 超过 `cmd_timeout`（建议 0.25 s）未更新 → 速度**立刻归零**（不是保持） | nav2 崩了/网络断了，机器人带着最后一个速度一直跑 |
| 3 | **体系→世界旋转**：用**当前 theta** 旋转（203 的做法） | 不旋转就只在 θ≈0 时正确，转过身之后机器人朝着错误方向走 |
| 4 | **钳位到 yaml 限位**：`[1.0, 1.0, 2.0]`，且加速度/jerk 各自限幅 | 超限的位置设定点会让底层跟不上，直接进入第 5 项的失控场景 |
| 5 | **位置牵引绳（leash）**：`\|cmd − current\|` 超过阈值（建议 0.15 m / 0.2 rad）就**停止积分** | **这是本方案最大的安全隐患**：位置积分是开环的，底盘被挡住/打滑时命令位置一路跑远，障碍一撤销机器人会猛冲过去追赶。厂商例程没有这一项，因为摇杆是人在闭环 |
| 6 | **启动/恢复时重新播种**：每次使能（或 leash 触发恢复）都从 `get_desired_joints_position` 重取种子 | 复用旧的 `pos_cmd` 会让底盘瞬间跳到一个陈旧目标 |
| 7 | **使能开关 + 控制权**：默认不申请控制权；导航使能是显式动作，且与急停互斥 | 桥接一起来就能动底盘，调试期风险太高 |

关于第 5 项再补一句：leash 触发时**只停积分、不清零已有命令**，并往
`/chassis_bridge/status` 报明确状态码（沿用 `PlanErrorCode` 那套风格），
让上层能区分"nav2 让我走但我走不动"和"nav2 没让我走"。这两者在
`Failed to make progress` 里长得一模一样 —— 我们已经在仿真侧被这个坑过一次。

### 7.4 感知通路：真机雷达是两颗，与仿真侧同构

`examples/301-get_lidar_scan.py` 给出确切接口：

```python
astribot.activate_lidar()                      # 必须先激活，否则没数据
node.create_subscription(PointCloud2, '/livox/lidar_front', cb, qos)  # BEST_EFFORT
node.create_subscription(PointCloud2, '/livox/lidar_back',  cb, qos)
```

我们现有链路本来就是"两颗 Livox 预处理 → 融合 → 多层切片 → `/scan`"
（`livox_preprocess_node` ×2 + `livox_fusion_node` + `pointcloud_slice_scan_node`），
所以真机分支**只需要改话题名 + 加一次 `activate_lidar()`**，
`hardware_perception.launch.py` 已经预留了这个分支。

两处必须核对，不能想当然：

1. **外参**：两颗雷达的安装位姿必须从 URDF/yaml 取，不允许写死。
   仿真里的 `livox_front/back` 外参是我们自己设的，真机的以厂商为准。
2. **QoS 必须 BEST_EFFORT**。用 RELIABLE 订阅会 QoS 不兼容、**一条消息都收不到**，
   只有一行 WARN —— 这个坑本轮刚踩过一次（旁听 `/monitored_planning_scene` 时）。

另外 `/scan` 那条链的**自滤必须带上夹爪**，这一条已经在仿真侧修过：
夹爪不在自滤链里会让机器人把自己的指尖当 0.41 m 处的障碍，
SLAM 把幻影烙进地图，然后规划器报 `Starting point in lethal space!`、探索 0 次派发。
真机上同一个坑会以同样方式出现。

### 7.5 代码落点

不新开包，全部落在已有的 `astribot_trajectory_bridge` 里（它已经有只读状态通路）：

| 新增 | 职责 |
|---|---|
| `chassis_odom_node` | SDK 底盘位姿/速度 → `/odom` + TF `odom→astribot_torso_base` |
| `chassis_cmd_bridge_node` | `/cmd_vel` → 7.3 的七道处理 → `set_joints_position` |
| `config/chassis_bridge.yaml` | `freq` / `cmd_timeout` / 三轴限位（**从厂商 yaml 读，不复制数值**）/ leash 阈值 / 使能默认关 |
| 单元测试 | 积分正确性、看门狗归零、钳位、leash 触发与恢复、旋转矩阵在 θ≠0 时的正确性（这条要故障注入：把旋转关掉必须让测试失败） |

复用而不是重写：`cmd_vel_body_to_world_node` 的旋转逻辑、
`path_tracking_diagnostics_node` 的分段量速度能力（用来验证 7.3 每一级的钳位是否真生效）。

### 7.6 分阶段验收（Gate 5，可独立于 Gate 3 推进）

| 阶段 | 内容 | 通过标准 |
|---|---|---|
| 5.1 只读 | `chassis_odom_node` 上线，机器人**手推**或用例程 202 移动 | `/odom` 与 `get_current_joints_position` 逐项一致；RViz 里 TF 连续无跳变 |
| 5.2 开环钳位 | `chassis_cmd_bridge_node` 上线，但**不接 nav2**，手工发 `/cmd_vel` | 三轴钳位、看门狗归零、leash 触发全部可复现；`velocity_scale: 0.1` |
| 5.3 接 nav2（空场地） | 完整栈，单点导航，有人守急停 | 到达 `xy_goal_tolerance` 内；leash 全程不触发；`preCpl→cmd` 比值符合预期 |
| 5.4 全流程 | 定位模式 + `mobile_transport` | 与仿真侧同一份配置只换 `target`；**业务层 diff 为空** |

### 7.7 已知会踩的坑（都是本仓库实测过的，别重新发现一次）

1. **臂-底盘耦合会把底盘限到 15%**，抬着物体导航必然 ABORTED。
   真机上这条同样成立，而且真机的 `joint_max_velocities` 只有 1.0 m/s，
   0.15 倍就是 **0.15 m/s** —— 比仿真更容易触发进度检查器。
   所以 §6.3 的 C1（修 `folded_reference_rad` 基准）从"可选优化"升级为**前置项**。
2. **进度检查器**：`required_movement_radius: 0.5` / `movement_time_allowance: 10.0`
   是按仿真速度定的，真机限速后必须重算，否则会得到"局部规划器走不动"的假结论。
3. **`use_sim_time` 必须关**，真机没有 `/clock`。已经在 `hardware_perception.launch.py` 处理。
4. **`ROS_DOMAIN_ID`**：已全栈统一 **25**（厂商 `env.sh` 就是 25，真机上 SDK 后端
   改不动）。启动前确认查询 shell 也带上 25，否则表现为"节点都在、话题一个都收不到"。
5. **SDK 一 import 就把整个进程 fd 1/2 重定向到 `/dev/null`**，
   桥接必须在 import **之前**置 `ASTRIBOT_LOG=1`，否则"响亮失败"完全失效。

### 7.8 刻意不做

- **不用 `set_joints_velocity` 走底盘**。API 里有这个函数，但 D5 已按厂商示例定案
  "速度控制实际不响应"，Gate 0 第 2 项才是取证。若取证发现底盘速度**确实**可用，
  那 7.3 立刻简化掉第 1、5、6 三项 —— 所以这条取证优先级很高，值得先做。
- **不把厂商那套 SLAM 位姿当 `map`**（理由见 7.2）。
- **不在桥接里做避障**。避障是 nav2 的职责，桥接只做"限幅 + 拒绝执行"，
  不擅自改方向 —— 否则出事时无法归因。

---

## 7.9 · C1 已完成：臂-底盘耦合限速基准修正（2026-08-25 实测验收）

C1 原定的任务是"`folded_reference_rad` 基准配错了，换个真实收纳姿态"。实测取证后
**这个定性是错的**：不是基准配错，而是**度量本身与物理量反相关**，换基准解决不了。

### 7.9.1 证伪旧度量（数据全部由活的 URDF 采样，不含硬编码几何）

| 姿态 | 关节偏差(旧度量) | 水平伸展 | 臂质心水平偏移 |
|---|---|---|---|
| 全0（原"收纳基准"） | 0.0000 | **0.4205 m** | 0.0117 m |
| `ready` | 1.0000 | 0.4790 m | 0.0226 m |
| 实测（左臂作业中） | 2.1104 | 0.4698 m | 0.0202 m |
| 候选收纳（肘部折回） | 2.4000 | **0.3532 m** | **0.0025 m** |

全工作空间随机采样 4000 次：**最大伸展 0.8865 m 时关节偏差 3.062，最小伸展 0.1997 m
时关节偏差 3.079**——偏差几乎相同，伸展差 4.4 倍。旧度量基本不携带伸展信息。

### 7.9.2 倾覆余量重算（放松安全阈值前必须做的那一步）

| 量 | 值 |
|---|---|
| 支撑多边形（4 轮） | xy = (±0.2163, ±0.2163)，临界方向取**边中点 0.2163 m** |
| 整机质量 | 78.595 kg（双臂+夹爪 17.963 kg，占 22.9%） |
| 整机质心（全0） | xy = (−0.0359, +0.0027)，z = **0.4321 m** |
| 最差静态余量 | 质心已偏后 0.036 m → **向后 0.180 m** |
| 倾覆所需加速度 | `a_tip = 9.81 × 0.180 / 0.4321 = 4.09 m/s²` |
| Nav2 指令加速度上限 | `max_accel: 2.5 m/s²`（= 阈值的 61%，**余量只有 39%**） |

**倾覆风险真实存在，C1 没有否掉它。** 但可归因于机械臂姿态的部分很小：整机质心偏移
= 0.2285 × 臂质心偏移 → 全工作空间最差降 **11.0%** `a_tip`，实际用到的姿态只降
**1.5~2.9%**。而修正前的方案为此把底盘速度砍到 **1/7~1/13**。这个不成比例才是 C1
要修的问题。`min_speed_scale` 仍为 0.15（限到多少没放松），只修正了"什么时候该限速"。

### 7.9.3 同一个坏基准在两个包里

`astribot_s1_navigation/arm_speed_limiter_node`（Nav2 官方 `/speed_limit`、二值 50%）
与 `astribot_s1_dynamics_coupling/arm_chassis_speed_coupling_node`（连续系数）**串联
叠乘**：0.50 × 0.15 = 0.075，把 `desired_linear_vel: 0.5` 压到约 0.037 m/s。只改一个
包无效，两个都改了。

### 7.9.4 修正内容

| | 修正前 | 修正后 |
|---|---|---|
| 度量 | 关节角相对全0参考的最大偏差 | 监控连杆相对 `astribot_torso_base` 的**水平伸展**（查 TF） |
| 耦合节点阈值 | `extension_full_rad: 1.2` rad | `reach_folded_m: 0.42` / `reach_full_m: 0.8865` m |
| 静态节点阈值 | `extended_threshold_rad: 0.5` rad | `extended_reach_m: 0.64` m + 0.03 m 迟滞 |
| 旧路径 | — | 保留为 `extension_metric: joint_deviation`，供 A/B 回归与一键回退 |

静态节点阈值 0.64 = 0.42（`robot_radius`）+ 0.2163（支撑多边形倾覆力臂），定位改为
**粗粒度 backstop**；0.42~0.64 的连续调速交给耦合节点，不重复计算同一判据。
（方案原定 0.42，实算发现 `ready`/作业姿态伸展都超过 0.42、50% 那一刀等于常开，
故按上述物理判据抬到 0.64。这是一次额外的阈值放松，依据是 7.9.2 的算术。）

### 7.9.5 实测验收（`real_file` + 完整 nav2 栈 + `mobile_transport execute:=true`）

| 轮次 | 导航耗时 | 底盘位移 | 平均速度 | 恢复行为 | 机械臂 |
|---|---|---|---|---|---|
| 修正前-1 | 35.8 s | 2.76 m | 0.077 m/s | 0 | 6/6 |
| 修正前-2 | 158.7 s | 2.17 m | 0.014 m/s | 4 | 6/6 |
| 修正前-3 | 78.1 s | 2.409 m | 0.031 m/s | 3 | 6/6 |
| **C1 后-1** | **14.0 s** | 2.721 m | **0.194 m/s** | **0** | 6/6 |
| **C1 后-3** | **13.4 s** | 2.777 m | **0.207 m/s** | **0** | 6/6 |

- 导航耗时从 35.8~158.7 s（方差极大、恢复行为频发）收敛到 **13.4~14.0 s（方差
  0.6 s、零恢复）**，且位移更远。
- 现场实测缩放比（直接量 `/cmd_vel_pre_arm_coupling → /cmd_vel` 幅值比，不看日志、
  不信推算）：**138 个样本全部 = 0.909**，与预测的 0.91 一致；修正前该值为 0.150。
- 离线 FK 预测伸展 0.4698 m vs 线上 TF 实测 0.4697 m —— **吻合到 0.1 mm**，取证脚本
  与线上代码路径互相验证。
- 单元测试 49 条（两个包各自独立），3 轮故障注入验证非空转。

**其中第 2 轮不是有效数据点，如实记录**：那一轮起点就在目标点上（底盘位移 0.007 m、
导航 0.1 s），是空操作，不计入。第 3 轮先用 `drive_to_pose.py` 复位到原点才跑。

### 7.9.6 C1b 立项：限速不缩小碰撞包络（未实现）

实测机械臂水平伸展**最大 0.8865 m，比 `robot_radius: 0.42` 多伸出 0.47 m**，这部分
是规划器彻底看不见的真实碰撞风险。原来 `nav2_params_rpp.yaml` 和
`README_NAVIGATION.md` §4.7 都声称这个风险"靠展开限速缓解"，**这句是错的——限速只
降低速度，完全不缩小包络**，两处注释已更正为"未解决"。正确机制是姿态相关的动态足迹
（`nav2_collision_monitor` 订阅机械臂 TF 实时改 footprint），立项 C1b，本次未做。

---

## 7.10 · 控制桥接已打通到 MuJoCo 后端（2026-08-27 实测取证）

`astribot_trajectory_bridge`（底盘 / 手臂 / 夹爪三条通路）已在真实 MuJoCo 后端上
建立会话并跑通。规模：17 个源文件 4709 行、12 个测试文件 3573 行、**425 条离线测试**、
约 15 次故障注入。本节只记**实测取证**与**被证伪的旧判断**，设计理由见包内 README。

### 7.10.1 结论先行：pinocchio 从来不是卡点

文档与代码注释里曾有 5 处声称"libpinocchio 3.7.0 与系统 4.0.0 主版本 ABI 冲突，
阻塞全部在线 Gate"。**这个结论是错的，一条 `ldd` 即可否证**：

    ldd astribot_sdk/core/common/robotics_library_py/_robotics_library_py.so | grep pinocchio
    libpinocchio_default.so.3.7.0 => .../third_party/third_pkg/pinocchio/lib/...   ← 零个 not found

| | soname | 位置 |
|---|---|---|
| 仓库自带 | `libpinocchio_default.so.**3.7.0**` | `third_party/third_pkg/pinocchio/lib` |
| 系统 apt | `libpinocchio_default.so.**4.0.0**` | `ros-humble-pinocchio` |

soname 是**完整字符串精确匹配**，厂商 `.so` 的 `DT_NEEDED` 烧死了 3.7.0，永不加载 4.0.0；
`env.sh` 里 `third_pkg/local_setup.bash` 早已把 3.7.0 挂进 `LD_LIBRARY_PATH`。
"整机对齐到 4.0.0"是**死路**：厂商 `.so` 闭源无法重编；跨主版本 symlink 会让符号对上
而行为静默错乱（3→4 改了模板签名与 `Data` 布局），比链接失败危险；卸系统 4.0.0 牵连 MoveIt。
**正确姿势就是两版按 soname 共存。**

真实卡点是一串打包/依赖问题：

| 层 | 现象 | 真因 | 处置 |
|---|---|---|---|
| 1 | `No module named 'robotics_library_py.robotics_library_py'; 'robotics_library_py' is not a package` | 扩展模块 `robotics_library_py.so` 与同名包目录互相遮蔽 | `sdk_session.prewarm_robotics_library_py()` 先预热一次 |
| 2 | `No module named 'meta'`（**完全误导的名字**） | 缺 pip 包 `filterpy` | `pip install --no-deps filterpy`（`--no-deps` 保护 numpy 1.21.5 钉版，有测试锁死） |
| 3 | `env.sh:17` 指向的 middleware 目录不存在 | 该行**从来是错的**，真实位置在 `third_party/software/astribot_ros_middleware/lib/python3.10/site-packages`，由 `software/setup.bash` 自动挂载 | `env.sh` 改为变量 `ASTRIBOT_MIDDLEWARE_PY`，不存在时**显式报 ERROR**（原先静默忽略，所以这行错活了很久） |

另：**`rclpy.init()` 必须在 `open_session()` 之前**。反了会抛
`RuntimeError: Context.init() must only be called once` —— 报错指向 rclpy，
真因是"SDK 自己已经初始化过了"。两个方向都实测过。

### 7.10.2 厂商 SDK 接口事实（全部实测，不采信文档描述）

| 项 | 实测结果 |
|---|---|
| `chassis_dof` | **3**（S1），`whole_body_dofs = [3,4,7,1,7,1,2]` 共 25 |
| `get_joints_position_limit` | 顺序确认为 **(lower, upper)**，25 个关节全部 `lower ≤ upper`（`examples/100:49` 的解包顺序是反的，是样例 bug） |
| 返回值嵌套 | list-of-lists，**每个部件一个内层 list** |
| `get_robot_mode` | **挂在内层 `astribot_interface` 上，`Astribot` 上没有**；仿真返回 `'simulation'`；真机三模式 `safe`/`professional`/`extremity`，**其它任何值都表示仿真**（源码是 if/elif/elif/else） |
| `get_current_cartesian_pose(frame=...)` | 参数是 **frame**（不是 names），返回**整机所有部件**，顺序同 `whole_body_names`；effector 那一项是 **1 维命令值**，不是位姿 |
| `get_inverse_kinematics` | 返回 `(bool, dict)`，dict 含 **torso + 双臂**（整机 IK）；**`flag` 恒为 True**，包括解就是零位、FK 误差 0.80 m 时；**迭代式**，每次只走一小步；会**停在局部最优后极缓慢劣化**（某目标第 ~100 次触底 0.2634 m，600 次爬回 0.2737 m） |
| `world` 坐标系 | **会话启动时以当前底盘位姿重置**。底盘在 x=+0.3237 时，新建会话里 `world` 与 `chassis` 读数差 **0.0000**，而移动前就存在的会话差 0.3958 m |
| IK 目标坐标系 | **chassis 系**（底盘前移 0.32 m 后同一 world 坐标目标误差 0.3412→0.3419，几乎不变） |
| QoS 告警 | 不兼容的那一对**两端都在 SDK 内部**（它自己 RELIABLE 的 WBC_SYNC 订阅 vs 自己 BEST_EFFORT 的发布）；仿真侧订阅是 BEST_EFFORT，**兼容**。指令实测跟随率 **100.0%、误差 0.0 mm** |
| 仿真侧不提供 | `/tf`（实测 0 条）、弧度关节角 —— 没有第三方来源可交叉核对 |

夹爪（`gripper_math` 已把这些钉成纯函数 + 37 条测试）：

| 项 | 实测结果 |
|---|---|
| 极性 | **0 = 张开，100 = 闭合**，与直觉相反。`open_effector`→0.0（`astribot_client.py:813/817`），`close_effector`→100.0（`836/840`）。本文开头的前提"100=全闭 / 0=全开"与实测**一致**，且 `examples/107` 注释逐字写着 "100 means fully closed, and 0 means fully open" —— 三处独立来源互相印证 |
| 换算 | `rad = 0.0093 × cmd`（= `gainprm 4.65 / -biasprm 500`），`cmd=100 → 0.93 rad` **正好等于关节上限** |
| 线性度 / 回差 | 残差 **0.0005** / 滞环 **0.0000**；实测系数 0.009298 对理论 0.0093，**吻合 0.02%** |
| 越界 −20 / 150 | **夹到边界**（0.0143 / 99.998），不乱走 |
| `open/close_effector` | **阻塞**，尊重 `duration`（实测 0.5s→0.50s、2.0s→2.01s） |
| `set_effector_max_force` | **仿真下是空操作**（`astribot_client.py:1139` `if __in_simulation: return`）→ 服务响应里 `force_applied` 恒为 false。**仿真里夹持力不可验收** |

### 7.10.3 只有真后端才暴露的九个缺陷

离线 296~425 条测试全绿时，下面每一条都**发现不了**。

| # | 现象 | 真因 | 为何离线测不到 |
|---|---|---|---|
| 1 | `AttributeError: 'RosClock' object has no attribute 'handle'` | `self._clock` **遮蔽了 `rclpy.Node` 的内部时钟**（`node.py:210`）。报错指向时钟对象，真因是属性遮蔽 | 只有实例化节点才暴露 |
| 2 | `Astribot` 上没有 `get_robot_mode` | 该方法在内层 `astribot_interface` 上 | `FakeSession` 实现了它 |
| 3 | 写入闸门会**拒绝仿真写入** | 判据只认 `'safe'`，而仿真返回 `'simulation'`。使用者只能去开 `allow_unsafe_mode`，**而那个开关会连带放开真机的 professional/extremity** —— 仿真期的便利变成真机的安全缺口 | 替身默认 `robot_mode='safe'` |
| 4 | 闸门②（声明与实际一致）**恒真** | `_discover_backends()` 返回 `declared_target`，即把"声明"当"发现结果" | 无真后端可发现 |
| 5 | 取消轨迹后手臂继续漂 **0.045~0.37 rad** | 位置指令**单次下发抓不住正在运动的关节**。SDK 的 `desired` 明明已等于所发值 —— **看 desired 永远发现不了**。持续重发则漂移 **0.0000** | 替身"下发即生效"是我自己定义的语义 |
| 6 | 上条的修复**等于没生效** | 节点执行循环写的是 `while phase in (STREAMING, SETTLING)` —— 新增 HOLDING 相位后**直接跳出循环且不报任何错**。改为排除终态 | 白名单在离线路径上恰好够用 |
| 7 | 要求夹爪半开，实际**全夹紧**（cmd=50 → actual 99.998，0.1s 就到 91.5） | 同 #5，中间开度必须持续重发。真机上手指间有东西就是压碎 | 同 #5 |
| 8 | 位姿源异常上报成 `SDK_CALL_FAILED` | 位姿查询**不是 SDK 调用**，报错指向错误子系统会把诊断引向机器人。新增 `POSE_PORT_FAILED = 28`，且**不折进** `SLAM_UNAVAILABLE_OPEN_LOOP`（那会把真 bug 伪装成正常降级） | 替身不会违约 |
| 9 | 机器人越限后**对任何指令都不响应** | `start()` 对每个路点查限位，而**路点 0 就是当前位置** → 每条轨迹都被拒，连"开回来"也被拒。**保护把恢复通路一起堵死了** | 替身不会自己走到非法状态 |

#5/#7 的对照组很关键：**正常完成（DONE）路径不暴露**，因为 `_step_settling` 本来就在
持续重发末点、到 DONE 时关节已静止（停发只掉 0.0047 rad）。**同一个 SDK 特性在正常路径
上无害、在安全路径（取消）上有害。**

#9 的修法：路点 0 是"从哪儿出发"的**测量值**，不是我们挑的目标。放行需三条同时成立 ——
等于当前实测位置（容差 `oob_start_tolerance_rad`）、其余路点合法、终点越限量不比起点更糟，
且必须上报 `LIMIT_VIOLATION`。已在真的越限的机器人上验证（起点 j6=0.7674 越限 → 放行 → 回到合法区间）。

### 7.10.4 新发现：底盘 → 手臂 的反向耦合（机制未定）

§7.9 处理的是**手臂 → 底盘**（伸展时把底盘限到 15%）。搬运串联暴露了**反方向**的耦合。

四因子对照，干净仿真、每组前都归零到同一起点、仿真自洽性检查通过：

| 扰动 | 手臂峰值跟踪误差 | 相对基线 |
|---|---|---|
| 底盘不动（基线） | 0.0659 ~ 0.0799 | — |
| **先移动底盘 0.16 m** | **0.1591 ~ 0.4180** | **+140% ~ +383%** |
| 先闲置 2.5 s | 0.0691 | +5%（噪声） |
| 先操作夹爪 | 0.0672 | +2%（噪声） |

这解释了搬运串联里两段手臂都 ABORTED（峰值 0.3519 / 0.3570，阈值 0.35）。

**机制未定，已排除两条**：

* **不是移动余波** —— 发手臂前底盘残余速度 0.0000 m/s 的样本里照样出现 0.4162；
* **不是 desired/actual 分叉** —— 该差值与峰值**完全不相关**（最差样本差 0.0020，
  基线样本差 1.1037 反而峰值最低）。这个假设是我提的，数据否了它。

数据是**双峰**的，同一底盘位置下两簇并存，所以"位置本身决定"也不成立：

    ~0.16  @ 峰值进度 35%     （等 0 s、等 6 s）
    ~0.42  @ 峰值进度 85%     （等 1 s / 3 s / 8 s + 重新归零）

峰值进度（35% vs 85%）与幅度（2.6 倍）都截然不同，是**两个现象**而非强弱变化。

**对配置的影响**：给手臂定 `max_tracking_error_rad` 必须用**底盘动过之后**的数据。
当前 yaml 默认 **0.10 是按静止基线定的**（静止峰值 0.035~0.064，约 1.6 倍余量，在静止
场景下是对的）。搬运场景实测上界 0.42，留余量需 ≥0.6 —— 但**机制查清前不改生产配置**，
按未理解的现象定阈值与按坏基线定阈值是同一类错误。下一步：扫 0/0.05/0.10/0.20/0.30 m
位移，看峰值-位移曲线是单调（指向静力/模型）还是有跳变。

### 7.10.5 测量纪律（本阶段两次被数据反咬后加的）

**A. 长跑的仿真进程会退化，整套数据都可能是假的。**

| 现象 | 长跑进程（15 h+，期间一度两实例并存） | **重启后** |
|---|---|---|
| 手臂 j6 静止值 | **+0.98 ~ +1.15** | +0.0001 |
| 跟踪误差峰值 | 双峰：0.058~0.072 + **3 次 0.327/0.327/0.328** | 0.0347 ~ 0.0639 |
| 0.15 以上尖峰 | 3 / 16 次 | **0 / 12 次** |

**一票否决的判据：j6 = 1.04 超出了 MuJoCo 自己的硬限位 ±0.78（`jnt_limited=1`）——
物理上不可能。** 假尖峰极具欺骗性：幅度 0.327/0.327/0.328、持续 90/91/93 拍、进度全是 1%，
高度可复现、看着就是确定的物理现象，我据此差点把 `max_tracking_error_rad` 改掉 ——
**那就是把坏基线拟合进生产配置**。

纪律：每轮实验前**重启仿真**、确认**只有一个进程**、数据旁记录仿真已运行时长；
所有涉及机器人状态的脚本都加 `check_sim_sane()`（读数超出 MuJoCo `jnt_range` 即拒绝采数）。

**B. 判据少一项，就在那一项上系统性假阳性。** 本阶段自己犯的四次：

| 错误判据 | 后果 |
|---|---|
| e2e 只比 `dispatched_cmd`，不比 `actual_cmd` | "要求半开、实际 99.998（全夹紧）"被判成 **✔ 通过**，报告还打了"全部通过" |
| 隔离实验不检查 `start()` 返回值 | 四组全报 `峰值=0.0000`（**零采样**），脚本据此打印"三组都没有明显恶化" |
| 把"已收敛 + 同向"当惯性尾巴，漏掉**量级** | 0.354 rad 被判成无害滑行（0.167 rad/s 走 0.354 需 2 s 以上，实测 0.3 s 内到位 —— 是跳变） |
| 端口契约用 `hasattr` | 基类为每个方法提供了 raise 实现，**`hasattr` 永远为真**；删掉 `close_effector` 后测试照样通过。改为判"是否覆盖基类同名函数"，并加**探测器自检** |

由此定的两条：**故障注入后必须先确认注入真的生效**（改前后计数对比）；
**e2e 成功路径必须比对机器人实测位置**，只验证自己算出的数字等于只验证了自己。

**C. 不要在脚本里写自动判词。** 本阶段有三次脚本的结论比数据说得多
（"惯性尾巴无害"、"三组都没恶化"、"机制是底盘位置本身"）。判词把"我预设的两种可能"
当成了穷举，而它打印出来的语气与真结论无异。

### 7.10.6 当前边界：已验证 / 未验证

**已在真后端验证**：

* SDK 会话建立、写入闸门（三条：声明与实测一致 / 反向一致性 / 真机模式）
* 底盘：指令落地（跟随率 100.0%、误差 0.0 mm）、leash 触发与冻结、看门狗
  （0.298 s 上报，阈值 0.30）、`reset_leash` 重取积分种子
* 手臂：250 Hz 流式（静止底盘下跟踪误差峰值 0.0041）、越限拒绝、关节名顺序校验、
  取消保持（修后 3 次全 0.0000 rad）、越限恢复
* 夹爪：极性、换算、越界拒绝、未知夹爪拒绝、阻塞时长、`force_applied` 恒 false
* **接缝**：手臂 Action 执行中并发调夹爪服务成功（cmd=70 → actual 69.593）；
  12/12 组隔离实验证明并发**不恶化**跟踪（峰值 +0.7%/−7.9%/+0.4%，频率四组
  全在 237.6~238.4 Hz）

**未验证 / 做不到**：

* **"真的把 box 夹起来"没有验证。** 笛卡尔抓取被 §7.10.2 两条堵住：IK 的 flag 会
  对失败报成功，且 `world` 系随会话重置，无法把场景物体的全局坐标表达成 IK 目标。
  正解是上层（MoveIt + TF）出关节轨迹、SDK 只执行 —— 与桥接的设计前提一致。
* 夹持力（仿真下空操作）、双臂并发、方案 A `move_joints_waypoints`
  （`enable_waypoints_service` 默认 false，从未真跑）
* 底盘闭环（MuJoCo 无 SLAM，全程 `SLAM_UNAVAILABLE_OPEN_LOOP` 纯开环 + leash）
* `WBC_SYNC` 的出处**未定位** —— 两个仓库、`/opt`、site-packages、所有 `.so`/`.pyc`
  的 `strings` 全搜过零命中。如实记为未知，不编解释。

### 7.10.7 环境侧改动（需要知会）

* `env.sh`：middleware 路径改为变量 `ASTRIBOT_MIDDLEWARE_PY`；新增
  `astribot_sdk/core/common` 到 `PYTHONPATH`（编译过的 `util.py` 用裸名导入，需要这一层）
* 装了 pip 包 `filterpy`（`--no-deps`，numpy 仍为 1.21.5，有测试锁死）
* **仿真仓 `astribot_simulation`（独立仓库）改了两处**，均已备份、可一键回退：
  * 4 个 `.obj` 是未拉取的 git-lfs 指针 → 换成**已存在的 STL**。这 4 个 geom 全是
    `contype=0 conaffinity=0 density=0`（纯视觉），**物理零影响**，只丢贴图。
    装 `git-lfs` 后 `git lfs pull` 即可换回（需要 sudo）。备份 `*.lfs_orig`
  * `simulation_mujoco_param.yaml` 切到 `astribot_s1_for_aloha_with_gripper.xml`
    （带桌子 + `box_red` + 可动底盘）。备份 `*.orig_bak`
* 复用 SDK 已编译的 `astribot_msgs` 供仿真使用：msg/srv 定义逐字节比对，25 个完全相同，
  3 个差异中 2 个仅行尾换行、1 个是**未被使用**的常量（常量不参与序列化，线格式一致）

---

## 附录 A · 被否方案

| 方案 | 结论 |
|---|---|
| **保留 Gazebo + 为真机写 ros2_control 硬件接口** | 已否。可行性验证过（`astribot_msgs` 带 C++ typesupport，`AstribotControlCommand`/`RobotJointState` 可直接收发，无需 Python 进实时环），保留全部导航资产；但需逐条消除附录 B 的 5 处模型差异，且仿真-实物对齐需要人为维护而非由构造保证 |
| **双仿真分工**（Gazebo 管导航、MuJoCo 管操作） | 已否。违反"一套主配置"，两套仿真会漂移 |

MuJoCo 环境的实测能力（决定"全面转投"必然砍掉导航的依据；
以下均为拉下仓库后**从源码确认**，不是只读文档）：

| 我的导航栈依赖 | MuJoCo 环境 |
|---|---|
| Livox 点云 → 多层切片 → `/scan` | **无 LiDAR**。全仓库 xml/py/yaml 内 `grep -i "rangefinder\|lidar\|laser\|livox\|scan"` 零命中（唯一命中是 `process_mujoco_xml.py` 里的目录扫描函数）。RGB/深度/点云/FT/IMU 都有 |
| warehouse 世界 + 货架 | **只有 `simpleTable` 一张桌子**，无 warehouse / 房间 / 货架。（早期我说"只有机器人模型 + 地面"，略有低估——但对定点抓取来说这张桌子刚好够用） |
| nav2 / SLAM / 代价地图 / 里程计 | **无**。底盘虽然在 MuJoCo 里有 4 轮 + 3 虚拟关节的完整建模，但没有任何导航/建图集成，`chassis_kinematics.py` 只做运动学，多个配置直接锁死底盘 |


---

## 附录 B · 差异实测数据

本决策下 **B.1 / B.4 / B.5 自动消失**（不再维护自有模型 / 无底盘），
B.2 / B.3 已在 Gate 1 处理完毕（B.3 的实际严重程度远超原判断，见该节）。保留全部数据备查。

### B.1 关节限位：厂商 ship 了四份互不一致的模型

> **本节是对早期一处错误归因的更正。** 我最初拿 `ws_robot` 与
> `model/astribot_arm_left.urdf` 比，得出"我的模型漂移了、躯干速度宽 3.3 倍不安全"。
> 逐项核对后：**我的模型与 `astribot_whole_body_with_wheel.urdf` 完全一致，
> 一个数都没改** —— xacro 头部那句"未改动任何数值"是真的。
> 真实情况是厂商自己的四份模型互不一致。

左臂与躯干，四份厂商模型 + 我的：

| 关节 | per-part（SDK 实际加载） | whole_body ×3 | dynamic_calib | 我的 ws_robot |
|---|---|---|---|---|
| arm joint_1 | −3.1~3.1 v8.4 | −3.0~3.0 v8.4 | −3.1~3.1 **v20** | −3.0~3.0 v8.4 |
| arm joint_2 | −1.53~0.46 v8.4 | −1.4~0.4 v8.4 | −1.5~0.5 v20 | −1.4~0.4 v8.4 |
| arm joint_3 | **−3.1~3.1** v8.4 | **−3.0~1.4** v8.4 | −3.14~1.57 v20 | −3.0~1.4 v8.4 |
| arm joint_4 | −0.06~2.61 v15 | 0.0~2.4 v15 | −0.02~2.55 v20 | 0.0~2.4 v15 |
| arm joint_5 | −2.56~2.56 **v20** | −2.0~2.0 **v15** | −2.15~2.15 v20 | −2.0~2.0 v15 |
| arm joint_6 | −0.76~0.76 v16.8 | −0.6~0.6 v16.8 | −0.78~0.78 v20 | −0.6~0.6 v16.8 |
| arm joint_7 | −1.53~1.53 v16.8 | −1.4~1.4 v16.8 | −1.55~1.55 v20 | −1.4~1.4 v16.8 |
| torso joint_1 | −0.04~1.5 **v1.8** | 0.0~1.4 **v6.0** | −0.06~1.5 **v20** | 0.0~1.4 v6.0 |
| torso joint_2 | −2.3~0.06 v1.8 | −2.3~0.0 v6.0 | −2.4~0.03 v20 | −2.3~0.0 v6.0 |
| torso joint_3 | −0.4~2.3 v1.8 | 0.0~2.3 v6.0 | −0.03~2.4 v20 | 0.0~2.3 v6.0 |
| torso joint_4 | −1.2~1.2 v1.8 | −1.5~1.5 v6.0 | −1.6~1.6 v20 | −1.5~1.5 v6.0 |

`whole_body.urdf` / `whole_body_with_wheel.urdf` / `whole_body_with_head.urdf`
三份**完全一致**；`dynamic_calib` 是第三套（所有速度一律 20，明显是标定用的占位值）。
头部两关节在所有版本里都一致（±1.57 / ±1.22，v4.0）。

**风险的方向没变，只是原因变了。** 按第 0 节的规则，SDK 加载 per-part，
所以真机控制栈认为躯干最大速度是 **1.8 rad/s**，而我的仿真按 **6.0** 执行 ——
3.3 倍这个风险是真实的，起因是"我抄了一份 SDK 不使用的模型"，
不是"我改错了数"。同理臂 joint_3 的真值是 **±3.1**，比我用的 −3.0~1.4 宽 1.7 rad，
所以 `transport_probe` 那张可达域图确实偏悲观 —— 这条结论不变。


### B.2 TCP 差 0.15 m —— ✅ Gate 1 已处理

我的 `astribot_arm_left_tool_joint` 是 `xyz = 0 0 0`，`tool_link` 坐在法兰原点上。
厂商 `astribot_arm_left.yaml`：

```yaml
effector_to_tool_pose: [0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 1.0]
```

SDK 的 tool 是法兰再往 **−y 0.15 m**。两个都没错，但不是同一个坐标系——
同名不同物，一条笛卡尔目标两边差 15 厘米且都不报错。

已加 `*_tcp_link` 并把 SRDF 各组 tip 与 manipulation 的 TCP 全部指过去。
`0.15` 有独立几何佐证（不是只信 yaml 一处）：按 MJCF 算指尖胶垫沿抓取方向
覆盖 link_7 系 `y ∈ [−0.144, −0.204]`，`−0.15` 正落在胶垫近端，是合理的夹持参考点。

顺带解释了之前那轮"TCP 嵌进腕部碰撞球"的排查：真机 TCP 离法兰 0.15 m，
本来在球外面，**那个问题只存在于我的模型里**。

### B.3 夹爪缺失 —— ✅ Gate 1 已处理，且比原判断严重得多

原文写的是"风险已解除，剩下的纯粹是实现工作，不再有未知"。
**这个判断偏轻了。** 动手时才发现夹爪缺失的真正代价不在"不能抓"，
而在**碰撞模型缺了一大块**：

厂商自己的碰撞模型（`astribot_whole_body.sdf` 的 link_7 `<collision>`）用的是
`meshes/s1_gripper/obj/opened_gripper.obj`，实测该 mesh 换算到 link_7 系后覆盖

| | x | y（抓取方向） | z |
|---|---|---|---|
| 厂商包络 | −0.068 ~ 0.080 | **−0.1997 ~ −0.048** | ±0.031 |
| 改造前本仓库 | ±0.056（一个球） | **−0.05 ~ 0.05** | ±0.056 |

也就是说：**沿抓取方向少了约 0.15 m，横向少了约 0.148 m，整个夹爪不在碰撞模型里。**
后果不是"规划保守一点"，而是规划出来的轨迹会**用夹爪去撞东西**，且仿真里一切正常。

**连带作废**：此前 `transport` 场景那套抓取余量分析（"下限 = 物体半高 + 腕部外接
半径 0.056 + margin = 0.136"、A/B 点的碰撞结论）都建立在这个缺了 150 mm 的模型上，
**数字全部作废**（方法仍然有效）。σ（奇异性）那部分只取决于手臂 7 个关节，不受影响。

厂商三档腕部碰撞建模，本仓库最终取第 ③ 档：

| | 内容 | 为什么不用 |
|---|---|---|
| ① `whole_body_with_wheel.urdf` | link_7 = sphere r=0.05 | 夹爪完全不建模（改造前抄的就是这份）|
| ② `whole_body.sdf` | 单个 `opened_gripper.obj` | 包络把**两指之间**也填实了，而 TCP 就在指间 → 正常抓取必判碰撞 |
| ③ `whole_body_with_gripper.sdf` | 6 关节 + 每连杆碰撞 mesh | ✅ 张开时两指相距约 105 mm、指间是空的 |

**实现中新发现的两个厂商不一致**（都已记进代码注释并被测试钉住）：
1. `L11/R11` 销轴位置两源差 **7.0 mm**（MJCF `(0.04125,0,0.036379)` /
   SDF `(0.036,0,0.031749)`）。取了 MJCF；依据不强 —— SDF 那个值与 MJCF 里
   被注释掉的 `<connect ... anchor='0.036 0 0.031749'>` 完全相同，而那是**另一处**
   销轴且在 L2 局部系下，怀疑 SDF 转换时搞混了。**指尖沿抓取方向有约 7 mm 不确定度。**
2. 碰撞 mesh **不是精确镜像**，是各自独立简化的（L2 120 顶点/236 面 vs
   R2 121/238；L11 139/274 vs R11 122/240）。运动学侧严格镜像（实测差 0.000000），
   但销轴处"刚好擦到"的接触在两侧判定不同：实测 `L11↔L2` 9.32% vs `R11↔R2` 100%。
   这是 SRDF 里唯一一处**不按采样统计、按机构判断**关掉的碰撞对（那是销轴，
   物理上永远接触；不关会凭空废掉约 9% 的夹爪工作区间）。
   排查中错怪过两个东西：先以为是 L11 预压角两源不一致，改了只从 10.14% 动到 9.32%；
   又以为是 `max_contacts=200` 截断，改成按对数现算后结果一字未变。

### B.4 底盘表示法不同 —— ✅ 本决策下消失

我：4 个真实轮关节 + `/cmd_vel` Twist。
厂商 yaml：3 个虚拟关节 `astribot_chassis_x/y/z_rot`，**位置**指令，
SLAM 世界系，100 Hz，最大速度 `[1.0, 1.0, 2.0]`。
MuJoCo 里两套并存（4 轮 + 3 虚拟，虚拟侧是 `position` 执行器）。
导航退出范围后不再需要桥接。

> 顺带记一个命名差异：MuJoCo 里是 `astribot_chassis_zrot`，
> 厂商 yaml 里是 `astribot_chassis_z_rot` —— **少一个下划线**。
> 底盘已出范围，但同类差异可能出现在别处，Gate 0 应扫一遍全表。

### B.5 左臂基座姿态疑点 —— 仍待 Gate 0 用 FK 定掉

| 来源 | xyz | rpy |
|---|---|---|
| 我的 URDF `arm_left_base_fixed_joint` | 0, 0.06449, 0.02348 | 0, −1.22173, −1.5707963 |
| 厂商 yaml `weld_to_base_pose` | 0, 0.06449, 0.02348 | −1.22173, 0, −1.5707963 |

平移完全一致，rpy 前两位对调。`weld_to_base_pose` 后 3 位的旋转约定
**未从源码确认**，只是按最常见约定读的。

注意这一项**不会**因为 Gate 1 换成 per-part URDF 而自动消失：
per-part 的 `astribot_arm_left.urdf` 是**单臂**模型，
它的根就是臂基座，臂如何焊到躯干末端只写在 yaml 的 `weld_to_base_pose` 里。
所以这个数必须单独核对 —— Gate 0 的 FK 比对是唯一可靠手段。


---

## 附录 C · 顺带解决的存量问题

- **`transport` 的"纯运动学"限制**：Gate 1 已把夹爪补成真实关节并进碰撞模型，
  但**还没加"张开/闭合 + attach 物体"这两步动作**，所以物体仍不会跟着走。
  另外 A/B 点与 `grasp_z_offset` 需按新碰撞模型重扫（旧数字已作废，见 B.3）。
- **臂-底盘耦合限速的跨包不一致**（`folded_reference_rad` 是全零即奇异构型，
  导致任何可用臂姿下底盘都被限速，实测搬运姿态直接掉到 `min_speed_scale: 0.15`，
  3.7 m 走了 50.5 s）：随 `astribot_s1_dynamics_coupling` 退出范围而不再相关，
  但**若导航需求回来必须先修这条**。

---

依据材料：`examples/` 全部 26 个示例、
`astribot_config/robot_config/astribot_s1/`（7 份部件 yaml + 16 份模型文件）、
`astribot_msgs`（21 条消息定义）、`env.sh`、
`ws_robot` 展开后的 URDF 逐关节比对、
以及 `Astribot-Dev/astribot_simulation` 仓库文档。

文中所有数值均为从上述文件直接读取或实测所得；标注"未确认"的部分未做验证。
