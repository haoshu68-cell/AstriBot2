---
name: robot-task-intake-execution
description: 在本仓库设计或修改机器人任务接收、准入、排队、优先级、资源冲突、执行监督、取消暂停恢复和结果回传时使用。把外部请求收敛为可追踪的任务事务，再交给导航、双臂、感知和设备执行层。
---

# 机器人任务接收与执行

这个技能补足“任务从哪里进入，以及怎样从请求走到可验证结果”的架构边界。它与
[`robot-task-orchestration`](../robot-task-orchestration/SKILL.md) 配合：本技能负责整机任务
生命周期和执行监督，后者负责搬运/抓取/放置的阶段事务。它不能把当前导航
`TaskArbiter` 写成已经存在的整机任务总线；先以源码和运行入口核对当前能力。

按任务范围读取：

- 核对当前入口与能力时查 [项目源码索引](../astribot-architecture-design/references/project-map.md)，再核对实际节点和源码。
- 涉及抓取、放置或运输阶段时才读取 [搬运事务](../robot-task-orchestration/SKILL.md) 及相关运输 README；普通任务排队无需加载搬运方案。
- 设计取消、结果提交或重启恢复时读取 [状态转换与事件偏序](references/task-transitions.md)。
- 进入实现时遵循 [项目语言约定](../robot-runtime-cpp/SKILL.md)。

## 1. 先区分任务入口与执行入口

当前仓库至少有人工导航、路线、探索和运输等入口；它们不能因为都使用 Action
就被视为同一层。设计新入口时画清下面的边界：

```text
Request sources
  operator / route / exploration / transport / remote API / scheduler
    -> Task Gateway（接收、鉴权、规范化、幂等去重）
    -> Admission & Scheduler（能力、资源、优先级、期限、租约）
    -> Task Arbiter / Mission Executor（每项任务的唯一执行者、冲突资源仲裁、取消、恢复）
    -> child Action adapters（Nav2 / MoveIt-MTC / gripper / head / perception）
    -> Execution protection（限位、时效、制动、急停、设备桥）
    -> observed result（反馈、终态、实测停稳、账本提交）
```

- Task Gateway 只接受和规范化请求，不发布 `/cmd_vel`、关节命令或厂家协议。
- Admission 决定“能不能接”；Executor 决定“按什么步骤执行”；设备桥和最终保护
  决定“当前这一拍是否允许运动”。
- 子 Action 的成功只表示对应子执行端报告成功；整机任务还要等待必要的实测确认
  和状态提交。
- 同一个长任务只能有一个执行所有者。多个入口可以提交请求，但不能并行写同一
  资源，也不能绕过仲裁器直接调用后端。

## 2. 规范化任务信封

不要让每个入口定义一套私有字段。进入调度器前把请求转换为 `TaskEnvelope`，至少
包含：

```text
task_id / request_id / parent_task_id
source / requester / priority / preemption_policy
task_type / arguments / required_capabilities
resource_set / world_context / map_epoch / scene_epoch / envelope_epoch
created_at / deadline / max_runtime / retry_budget
idempotency_key / cancel_policy / resume_policy
```

- `task_id` 标识业务任务；`execution_id` 标识一次执行尝试；重试不能覆盖原事件。
- 为幂等键定义 requester/租户、操作类型和保留期等作用域，并绑定规范化 payload 摘要。
  同作用域同键同 payload 返回已有任务/结果；同键不同 payload 返回 `IDEMPOTENCY_CONFLICT`。
  终态后重放不能重新驱动机器人；查询、取消与重试使用各自明确定义的请求语义。
- `world_context` 要引用世界快照、定位来源、时间/坐标、标定和能力版本；过期上下文
  只能重新快照或拒绝，不能靠延长超时继续执行。
- 资源集合至少能表达底盘、左臂、右臂、躯干、头部、夹爪、PlanningScene、导航任务、
  RobotEnvelope 和人工控制权。资源名字要稳定，不能用日志文本代替锁。
- 优先级只能在准入与仲裁层生效。抢占要冻结旧任务的新步骤，按取消/暂停协议处置并
  记录原因；只有冲突资源满足交接条件后才授予新执行者，不能覆盖执行器的最后命令。

## 3. 生命周期与状态机

分别建模执行阶段、业务结果和资源处置，避免把它们串成一条必经流程。下面是
设计骨架，名字可映射到现有接口；不是对当前已实现状态的声明：

```text
RECEIVED -> NORMALIZING -> VALIDATING
                         ├─ 校验失败 -> FINALIZING(REJECTED)
                         └─ 校验通过 -> QUEUED -> ADMITTED -> RESERVED -> PREPARING -> EXECUTING
QUEUED / ADMITTED         └─ 取消 -> FINALIZING(CANCELED；无下游副作用时)
EXECUTING                ├─ 子步骤完成 -> 下一步骤 / VERIFYING -> FINALIZING(候选结果)
                         ├─ 暂停 -> PAUSE_REQUESTED -> PAUSED -> RESUMING -> PREPARING
                         └─ 取消/抢占 -> CANCEL_REQUESTED -> STOPPING -> VERIFYING -> FINALIZING
任何已产生副作用的阶段   └─ 故障或物理状态未知 -> RECOVERY_REQUIRED（隔离相关资源）
FINALIZING               └─ 结果持久化 -> 业务终态；资源交接单独提交并记录
```

- `ADMITTED` 是本轮准入通过，`RESERVED` 是所需资源已获得；排队后需重新准入，未持有
  必需资源不得执行副作用。读写资源与依赖决定冲突，不要求整机永远只有一个活动任务。
- `cancel requested/ACK`、子 Action 终态、实测安全状态、业务结果持久化和资源交接是不同
  事件。记录源时间、接收时间及关联/epoch；ACK、终态和反馈可能乱序，不强求到达全序。
- 业务结果区分 `SUCCEEDED/FAILED/CANCELED/PREEMPTED/REJECTED`，包含原因、证据、
  最后已知物理状态及 `resource_disposition`；结果提交不表示资源已经可用。
- 结果可先持久化为 `release_pending` 或 `quarantined`。旧执行者已终止或被有效隔离、
  必要实测确认有效、结果和资源交接可恢复地提交后，才允许冲突资源的新授权。
  中间阶段/暂停交接使用持久化检查点，不必等待整项任务终态。
  不能要求“先释放再有结果”同时又要求“先有结果再释放”。
- 物理状态未知或落盘失败时保留相关资源隔离，允许最终保护继续停止/保载；独立只读
  或确无资源/依赖冲突的任务按各自准入执行。`RECOVERY_REQUIRED` 不等于空闲或自动重试。
- 暂停需确认安全状态，明确保留/释放哪些资源以及恢复重验条件。详细转换、竞态和
  持久化失败规则见 [状态转换与事件偏序](references/task-transitions.md)。

## 4. 准入与调度顺序

在创建子 Action 之前按以下顺序检查实际能力依赖；与本任务无关的地图、定位或运动检查标明不适用，不阻塞纯去重、只读查询等工作：

1. 请求格式、来源权限、能力类型和幂等键；
2. 当前模式、设备健康、时间是否推进、定位/地图/场景/标定是否新鲜；
3. 资源冲突、已有活动任务、人工控制权和抢占策略；
4. 目标/路径/物体/区域约束，以及 `RobotEnvelope`、禁行区和安全边界；
5. 截止时间、总预算、子步骤超时和允许的有限重试；
6. 失败后的安全停留点、补偿步骤和人工接管入口。

准入结果要可解释：返回 `accepted`、`queued` 或 `rejected`，并附稳定的 reason code、
检查的能力/资源/epoch 和下一步建议。`queued` 不得被调用者误读成已经开始执行。

推荐的调度规则是“资源可行性优先，随后比较优先级和等待时间，再应用显式抢占策略”。
同优先级任务保持先到先服务；探索/覆盖任务可以被人工、故障处理或受保护的运输阶段
暂停，但不得用无界重试饿死其他任务。任何抢占都应生成父子关联事件。

## 5. 执行监督与子任务适配

Executor 不实现导航控制律或机械臂轨迹算法，而是把已准入任务拆成有界步骤：

```text
precondition snapshot
  -> plan/validate
  -> submit child action
  -> consume feedback and timeout
  -> observe completion
  -> commit step result
  -> refresh snapshot
  -> next step or terminal/recovery
```

- 每个步骤绑定 `task_id / execution_id / step_id / context_id`，并记录输入 epoch、发出
  时间、反馈时间、结果时间和取消状态。
- Nav2、MTC、夹爪和头部执行器分别使用其已有 Action/Service；任务层不把多个后端
  合并成一个无法取消的同步调用。
- 提交前重新检查起点、资源租约、模式、上下文有效期和执行器状态；反馈停滞、时钟
  回退、状态过期或版本不一致时停止继续发新步骤。
- 回调只入队或更新状态，不能在 ROS executor 线程执行无界规划、阻塞等待或重试循环。
  长规划放到受控 worker；实时保护仍在执行侧。
- 子步骤的 `SUCCEEDED`、目标状态变化和真实物理确认分开。例如 Nav2 到站后还要等
  停稳；抓取 action 完成后还要核对关节、夹爪、附着物和载荷账本。

## 6. 取消、暂停、恢复与故障

取消和恢复是任务协议的一部分，不是 CLI 的附加按钮：

- 取消先冻结新步骤，再向当前子 Action 发 cancel，等待结果或超时进入受控停止；持续
  读取相关速度/关节/附着状态。按同一权威状态机处理取消与成功竞争，先决定并持久化
  业务结果及资源处置，再完成资源交接；无法确认安全时保留隔离，不用取消 ACK 代替停稳。
- 暂停要定义可暂停边界。导航通常在安全停留点暂停；夹持中、附着状态不确定或设备
  故障时优先保载和人工确认，不自动松爪。
- 恢复必须重新获取世界快照、资源租约、地图/场景/包络和能力状态；旧计划和旧上下文
  默认失效，只能从已确认的检查点继续。
- 重试按步骤和原因设置上限，指数等待不能替代失败预算。相同目标反复生成新
  `task_id` 逃避预算要拒绝并报警。
- 任务失败时保存最后一个已知安全状态和未决物理状态；不删除账本、不伪造
  `PLACED`/`STOPPED`，不因为进程重启就把未知状态当成空闲。

故障处置优先级为：最终保护/急停 > 设备故障隔离 > 保持载荷或安全停留 > 任务取消/恢复
决策 > 普通任务重排。这个技能不替代
[`robot-mission-boundary-safety`](../robot-mission-boundary-safety/SKILL.md) 的脱困、报警
和区域边界规则，也不替代
[`ros2-control-hardware-safety`](../ros2-control-hardware-safety/SKILL.md) 的硬件保护。

## 7. 接口与持久化

- 长时间、有反馈、可取消的整机任务使用 Action；短查询、准入检查或配置提交使用
  Service；当前任务状态、能力和事件摘要使用有界 Topic。
- Action feedback 是过程观测，result 是一次执行结果；不能把 feedback 的“接近目标”
  当成到站或任务成功。
- 状态事件和结果落盘时至少写入 `task_id`、`execution_id`、`step_id`、状态、原因、
  时间/时钟域、资源租约、上下文/epoch、来源和版本。事件追加写，状态快照原子替换。
- 重启后对未完成执行、未完成交接或证据已过期的资源进入 `UNKNOWN/RECOVERY_REQUIRED`，
  核对下游及账本；已持久化结果保持不可重复执行。检查新 execution/lease epoch，拒绝旧
  结果改变当前状态，不能仅凭最后一条内存状态继续执行。
- 对外 API 必须定义版本、超时、取消语义、幂等行为、结果有效期和错误码；不要通过
  改变字符串文案隐式改变调度语义。

## 8. 模块内实现与验证

设计取消/抢占竞争、幂等、租约过期和重启恢复验收时，读取 [多场景与边界验证](../robot-scenario-boundary-validation/SKILL.md) 中的 SC-TASK、SC-LEASE、SC-TRANSPORT 场景族；报告成功前把终态、停稳和资源交接证据分别关联。

- 用无 ROS 的核心实现 `TaskEnvelope`、状态机、资源租约、调度比较、重试预算和事件模型；
  用 ROS 适配层连接 Action/Service/Topic 和现有后端，语言统一遵循 [项目约定](../robot-runtime-cpp/SKILL.md)。
- 任务入口、Executor、资源管理器、事件/状态存储和后端适配器分开；不要让一个节点
  同时拥有接收、规划、底盘控制和设备协议写入权。
- 对本次修改的协议覆盖正常成功、拒绝、排队取消、同键不同 payload、优先级/抢占、
  资源冲突、迟到 ACK/结果、取消竞争、超时、重启、落盘失败、旧 epoch 和停止未确认；
  具体输入与预期见 [协议场景表](references/task-transitions.md#协议验证场景)。
- 隔离仿真验证 Action 终态与实测停稳分离、任务状态持久化和恢复边界；整栈仿真验证
  Nav2/MoveIt/桥接组合；真机只在完成设备/急停/负载/制动准入后验证。
- 交付报告必须区分“请求已接收”“任务已准入”“子步骤成功”“整机终态成功”和
  “实测安全状态确认”，并关联原始事件和版本。详见
  [`robotics-validation-evidence`](../robotics-validation-evidence/SKILL.md)。

## 9. 当前项目的落点与限制

当前 `TaskArbiter` 主要拥有导航目标仲裁；`/exploration/*`、路线入口和运输入口仍有
各自边界。把它们统一成整机任务入口是后续设计，不应在文档或报告中写成已经完成。

设计新任务模块时，优先复用已有导航仲裁、`astribot_navigation_msgs`、运输账本、
`RobotEnvelope`、日志关联 ID 和执行保护；先增加影子事件/适配器，再迁移现有入口。不得
为了统一 API 复制一套速度控制器、绕过 SDK 使能/时效/停车门槛，或让新任务客户端直接
向 `/cmd_vel`、关节控制器或厂家 SDK 写命令。
