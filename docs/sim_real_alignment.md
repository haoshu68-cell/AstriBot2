# 规划层 / 控制层 仿真-实物对齐方案

> **已定决策（2026-08-20）：全面转投厂商栈，砍掉导航。**
>
> 仿真用厂商 MuJoCo 环境，控制走 astribot_sdk，仿真与实物**由构造对齐**
> （同一个 SDK、同一套 WBC，只有物理引擎不同）。
> 移动作业能力（nav2 / SLAM / 探索 / mobile_transport 的导航段）退出范围。
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

| 包 / 能力 | 原因 |
|---|---|
| `astribot_s1_navigation`（nav2） | MuJoCo 无 LiDAR、无场景、无导航集成 |
| `astribot_s1_perception`（slam_toolbox） | 同上，且 SLAM 依赖 `/scan` |
| `astribot_s1_autonomy`（探索协调器 / 多层切片） | 依赖 Livox 点云 |
| `astribot_s1_chassis_effort_drive` | 底盘不再由我驱动 |
| `astribot_s1_dynamics_coupling` | 臂-底盘耦合限速，无底盘即无意义 |
| `astribot_s1_gazebo_bringup` | Gazebo 退场 |
| `mobile_transport` 场景的导航段 | 退化为 `transport`（定点） |

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
  domain_id: 25                   # 厂商 env.sh 固定 25（我原来的栈用 42）
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

**剩余待测**（需要先装 MuJoCo 运行时）：

```bash
sudo apt install git-lfs && git lfs install
cd /home/yjh/WorkSpace/astribot_simulation
git submodule foreach git lfs pull          # 网格资产在 LFS 里
bash scripts/lite_install/install_mujoco.sh # 只装 MuJoCo，不装四个后端
```

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

> ⚠️ **一处尚未查清的现象**：`transport` 每一步报的"最差 σ"都是 **0.0077**，
> 六步一模一样，且**低于**配置的奇异阈值 0.02，结果却是 PASS。
> 0.0077 正是本机零位（`joint_4=0`，肘部完全伸直）的 σ。
> 推测原因：`execute:=false` 时机器人从不移动，每一步都从同一个零位起步，
> 而起始状态是"机器人已经在那儿了"、不该被拒绝。
> 但这需要 `execute:=true` 接 Gazebo 实跑才能确认，**目前只是推测**。
> 加 `move_to_ready:=true` 也仍是 0.0077（不执行就动不了），与该推测一致。


### Gate 2 · 桥接骨架（只读）

`astribot_trajectory_bridge` 先只做状态方向：SDK 读状态 → 发 `/joint_states`。
暂不接受任何轨迹。

> **通过标准**：`/joint_states` 与 SDK `get_current_joints_position()` 逐关节一致；
> RViz 里模型姿态与 MuJoCo 画面一致。

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
