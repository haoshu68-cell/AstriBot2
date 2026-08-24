# 规划层 / 控制层 仿真-实物对齐方案

> **已定决策（2026-08-20）：全面转投厂商栈，砍掉导航。**
>
> 仿真用厂商 MuJoCo 环境，控制走 astribot_sdk，仿真与实物**由构造对齐**
> （同一个 SDK、同一套 WBC，只有物理引擎不同）。
> 移动作业能力（nav2 / SLAM / 探索 / mobile_transport 的导航段）退出范围。
>
> 被否方案与理由见附录 A。差异实测数据见附录 B（其中 3 项因本决策自动消失）。

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

### D4 · 夹爪 —— ⚠️ 存在未解决的错位

这一条因转投而**变得更关键**（定点作业里抓取就是核心），且**新增了一个问题**：

| 来源 | 末端形态 |
|---|---|
| SDK API | 1 DOF，0~100 无量纲，100=全闭；速度 0~1000 |
| `astribot_gripper_left.yaml` | 单关节 `astribot_gripper_joint`，限位 0~100，力矩 145 |
| `astribot_whole_body_with_gripper.sdf` | 每侧 **6 个物理关节** `joint_L1/L11/L2/R1/R11/R2`（连杆机构） |
| **MuJoCo 仿真环境** | 文档写的是 **"BrainCo hand"（灵巧手）** |
| 我的 ws_robot | **不存在** |

前三者自洽（6 关节机构对外抽象成 1 DOF），但**MuJoCo 那边可能装的是另一种末端**。
如果仿真是灵巧手、真机是二指夹爪，那"由构造对齐"这个前提在末端上就不成立。

> **这是本决策下最大的技术风险，必须在 Gate 0 实测确认。**
> 确认方法：起 MuJoCo，`ros2 topic list` + `get_dof()` 看夹爪部件的 DOF 是 1 还是更多。

对外接口仍按 SDK 的 1-DOF 抽象做（MoveIt SRDF 里声明 `end_effector` +
`gripper` 组，`GripperCommand` 驱动），这一点不受上面影响。

### D5 · 能力矩阵 —— ⚠️ 材料自相矛盾

| 能力 | 真机 | SDK 示例说的"仿真" | MuJoCo 仓库说的 |
|---|---|---|---|
| 位置控制 | 支持 | 支持 | 支持（7~14 维，前 7 位置 + 后 7 速度） |
| 速度控制 | 支持 | ❌ "not supported in simulation and will not respond to commands"（示例 106） | 支持（后 7 维即速度，带速度/重力补偿） |
| 力矩控制 | 支持 | ❌ 同上（示例 210） | 支持，但 **"does not guarantee sim-to-real accuracy, mainly for simulation purposes"** |

**示例 106/210 与 MuJoCo 文档直接冲突。** 可能的解释：
示例里的"simulation"指的是另一个更早的仿真后端，或者 MuJoCo 支持是后来加的。

> 不管哪种解释，结论都一样：**能力矩阵必须实测生成，不能抄文档。**
> Gate 0 里逐个接口发指令、看机器人是否响应，把结果写进主配置。
> 静默不响应是最坏的失败形态——代码正常跑、日志干净、机器人不动。

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
  source_dir: $(env ASTRIBOT_SDK_ROOT)/astribot_config/robot_config/astribot_s1

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

这一步是本决策的**前提验证**。全部未确认项都在这里解决。

安装（前置已核实：RTX 4090 / 驱动 580 / Ubuntu 22.04 / Python 3.10.12 全部满足；
GitHub 可达；磁盘 819G 可用。**缺 git-lfs，需先装**）：

```bash
sudo apt install git-lfs && git lfs install
git clone -b main https://github.com/Astribot-Dev/astribot_simulation.git
cd astribot_simulation
git submodule update --init --recursive
git submodule foreach git lfs pull
bash scripts/lite_install/install_mujoco.sh     # 只装 MuJoCo，不装四个后端
```

要测出来的东西（每一项都是"文档说的不算，跑一遍才算"）：

1. **MuJoCo 侧的 ROS 图**：`ros2 topic list` / `ros2 node list`，
   与 SDK 期望的话题对照。文档给的是 `/astribot_arm_left/joint_space_command`，
   而 `astribot_msgs` 里是 `AstribotControlCommand` ——
   **这两者是不是同一套接口，决定 SDK 能否原样跑。**
2. **SDK 原样跑通性**：直接跑 `examples/101`（读状态）、`103`（关节运动）、
   `107`（笛卡尔）。跑通即证明"同一套 SDK"这个前提成立。
3. **能力矩阵**（D5）：跑 `106`（速度）、`210`（力矩），看机器人是否真的动。
4. **夹爪形态**（D4）：`get_dof()` 看夹爪部件维度；`ros2 topic list` 找末端话题。
   是 1 DOF 二指夹爪还是 BrainCo 灵巧手，直接决定末端能否对齐。
5. **FK 一致性**：同一组关节角，比 SDK `get_forward_kinematics()`（示例 204）
   与 MoveIt FK。顺带定掉附录 B.5 那个基座 rpy 疑点。
6. **`get_joints_position_limit()` 返回顺序**：示例 100 按 `upper, lower` 解包，
   示例 103 / 210 按 `lower, upper` —— **三个官方示例自相矛盾**，
   必须从源码或实测确认，否则所有限位校验方向都可能是反的。
7. **速率**：各部件 yaml 写 `frequency: 100`，示例用 `Astribot(freq=250)`。
   实测哪个是实际生效的控制周期（D2 选 streaming 时直接依赖这个）。

> **通过标准**：`examples/101/103/107` 在 MuJoCo 上跑通；
> 能力矩阵三项有明确实测结论；夹爪 DOF 确定；
> FK 位置偏差 < 1 mm、姿态偏差 < 0.001 rad；限位返回顺序有定论。
>
> **若第 1、2 项不通过，本决策的前提就不成立**，必须回到附录 A 重选。

### Gate 1 · 模型改为单一真值源

- `astribot_s1_description` 改为引用
  `astribot_config/robot_config/astribot_s1/model/` 下的厂商 URDF，
  删掉我这边所有重复声明的关节限位与连杆几何。
  这一步自动消灭附录 B 的 B.1（限位不一致）和 B.5（基座 rpy）。
- 补夹爪（按 Gate 0 测出的真实形态），SRDF 加 `end_effector` + `gripper` 组。
- 加 `*_tcp_link`（B.2），把 SRDF 各组 tip 与 `manipulation_params.yaml`
  的 TCP 全部指向它——**不再用 `tool_link` 当笛卡尔目标**。
- 加构建期一致性测试：逐关节比对厂商 yaml/URDF，任何一项不等就编译失败。

> **通过标准**：一致性测试通过；重跑 `transport_probe`，
> 可用候选数应**明显增加**（左臂 joint_3 放开 1.7 rad 之后必然如此）——
> 没变化说明真值源没真正生效。

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

MuJoCo 环境的实测能力（决定"全面转投"必然砍掉导航的依据）：

| 我的导航栈依赖 | MuJoCo 环境 |
|---|---|
| Livox 点云 → 多层切片 → `/scan` | **无 LiDAR**（RGB/深度/点云/FT/IMU 都有，激光全文未提） |
| warehouse 世界 + 货架 | **无任何场景**，资产只有机器人模型 + 地面 `worldbody` |
| nav2 / SLAM / 代价地图 / 里程计 | **全文未提**；底盘仅 `chassis_kinematics.py` 纯运动学，多个配置直接锁死底盘 |

---

## 附录 B · 差异实测数据

本决策下 **B.1 / B.4 / B.5 自动消失**（不再维护自有模型 / 无底盘），
B.2 / B.3 仍需在 Gate 1 处理。保留全部数据备查。

### B.1 关节限位不一致 —— ✅ 本决策下消失

左臂（我的 URDF vs 厂商 `model/astribot_arm_left.urdf`）：

| 关节 | 我的位置限位 | 厂商位置限位 | 我的速度 | 厂商速度 |
|---|---|---|---|---|
| joint_1 | −3.0 ~ 3.0 | −3.1 ~ 3.1 | 8.4 | 8.4 |
| joint_2 | −1.4 ~ 0.4 | −1.53 ~ 0.46 | 8.4 | 8.4 |
| joint_3 | −3.0 ~ **1.4** | −3.1 ~ **3.1** | 8.4 | 8.4 |
| joint_4 | **0.0** ~ 2.4 | **−0.06** ~ 2.61 | 15 | 15 |
| joint_5 | −2.0 ~ 2.0 | −2.56 ~ 2.56 | **15** | **20** |
| joint_6 | −0.6 ~ 0.6 | −0.76 ~ 0.76 | 16.8 | 16.8 |
| joint_7 | −1.4 ~ 1.4 | −1.53 ~ 1.53 | 16.8 | 16.8 |

臂全部是我更保守——安全，但可达域被人为缩小。`joint_3` 丢 1.7 rad 尤其明显：
`transport_probe` 那张"80 个候选 50 个可用"的可达域图在真机上偏悲观。

躯干**相反，且不安全**：

| 关节 | 我的位置限位 | 厂商位置限位 | 我的速度 | 厂商速度 |
|---|---|---|---|---|
| torso_joint_1 | 0.0 ~ 1.4 | −0.04 ~ 1.5 | **6.0** | **1.8** |
| torso_joint_2 | −2.3 ~ 0.0 | −2.3 ~ 0.06 | **6.0** | **1.8** |
| torso_joint_3 | 0.0 ~ 2.3 | −0.4 ~ 2.3 | **6.0** | **1.8** |
| torso_joint_4 | **−1.5 ~ 1.5** | **−1.2 ~ 1.2** | **6.0** | **1.8** |

躯干速度宽 3.3 倍意味着 `optimized_velocity_scaling: 0.90` 跑满的"时间最优"轨迹，
躯干段速度可达真机限位 3 倍，而 Gazebo 会老老实实按 URDF 执行、什么都不报。
头部两关节完全一致（±1.57 / ±1.22，速度 4.0）。

### B.2 TCP 差 0.15 m —— ⚠️ Gate 1 仍需处理

我的 `astribot_arm_left_tool_joint` 是 `xyz = 0 0 0`，`tool_link` 坐在法兰原点上。
厂商 `astribot_arm_left.yaml`：

```yaml
effector_to_tool_pose: [0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 1.0]
```

SDK 的 tool 是法兰再往 **−y 0.15 m**。两个都没错，但不是同一个坐标系——
同名不同物，一条笛卡尔目标两边差 15 厘米且都不报错。

顺带解释了之前那轮"TCP 嵌进腕部碰撞球"的排查：真机 TCP 离法兰 0.15 m，
本来在球外面，**那个问题只存在于我的模型里**。

### B.3 夹爪缺失 —— ⚠️ Gate 1 仍需处理，且风险升级

见决策 D4。本决策下新增了"MuJoCo 可能装的是 BrainCo 灵巧手"这一未确认项。

### B.4 底盘表示法不同 —— ✅ 本决策下消失

我：4 个真实轮关节 + `/cmd_vel` Twist。
厂商：3 个虚拟关节 `chassis_x/y/z_rot`，**位置**指令，SLAM 世界系，100 Hz，
最大速度 `[1.0, 1.0, 2.0]`。导航退出范围后不再需要桥接。

### B.5 左臂基座姿态疑点 —— ✅ 本决策下消失（仍在 Gate 0 顺带定掉）

| 来源 | xyz | rpy |
|---|---|---|
| 我的 URDF | 0, 0.06449, 0.02348 | 0, **−1.22173, 0** 后接 −1.5707963 |
| 厂商 yaml `weld_to_base_pose` | 0, 0.06449, 0.02348 | **−1.22173, 0**, −1.5707963 |

平移完全一致，rpy 前两位对调。`weld_to_base_pose` 后 3 位的旋转约定
**未从源码确认**，只是按最常见约定读的。改用厂商模型后此项自动消失，
但 Gate 0 的 FK 比对仍应顺带验证——它是"两套模型到底哪套可信"的直接证据。

---

## 附录 C · 顺带解决的存量问题

- **`transport` 的"纯运动学"限制**：Gate 1 补上夹爪后消失，可做真实抓取。
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
