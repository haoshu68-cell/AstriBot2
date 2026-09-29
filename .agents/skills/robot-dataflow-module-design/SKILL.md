---
name: robot-dataflow-module-design
description: 在本仓库设计或审查机器人软件数据流、模块边界、接口契约、依赖图、QoS、数据生命周期和模块内实现时使用。把感知、世界模型、任务决策、规划、执行、设备反馈和诊断组织成可追踪的数据与控制流。
---

# 机器人数据流与模块设计

这个技能专门处理“数据怎样在模块之间流动，以及模块应当负责什么”。它与
[`mobile-dual-arm-architecture`](../mobile-dual-arm-architecture/SKILL.md) 的整机控制权视图、
[`perception-world-model`](../perception-world-model/SKILL.md) 的观测/世界快照、
[`robot-task-intake-execution`](../robot-task-intake-execution/SKILL.md) 的任务生命周期互补，
不替代它们。

按变更范围读取：

- 核对当前包、节点或启动入口时查 [项目源码索引](../astribot-architecture-design/references/project-map.md)，再读取受影响源码；历史评审只作为设计背景。
- 涉及整机执行权、感知有效性或任务生命周期时，分别读取上面对应的技能，不默认加载全部。
- 需要确认 ROS 接口或 QoS 细节时查 [接口语义](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html) 与 [QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。
- 进入实现时遵循 [项目语言约定](../robot-runtime-cpp/SKILL.md)。

## 1. 按影响范围区分数据流和控制权

数据流回答“事实、意图、命令、反馈和证据经过哪些模块”；控制流回答“谁拥有写入、
取消、停稳确认和状态提交权”。二者不能合并成一张只标节点名称的图。局部接口可用
契约表表达；跨模块架构再画对应视图，不强制每次重画整机。

```text
Sensor / SDK feedback
  -> Adapter
  -> Observation + provenance
  -> Quality gate / fusion
  -> WorldSnapshot / capability state
  -> Task / policy decision
  -> Plan / constraints / envelope
  -> Child Action
  -> Controller / velocity smoothing
  -> Device adapter
  -> measured feedback
  -> observation, evidence and state commit
```

对每条箭头标明：生产者、消费者、数据类别、方向、频率/触发条件、QoS、时间语义、
frame、版本/epoch、失效条件、是否允许丢弃和是否能触发运动。没有这些字段的箭头只是
概念连接，不能作为接口设计。

## 2. 按数据语义划分接口

不要按“所有东西都发 Topic”或“所有事情都做 Service”设计。至少区分：

| 数据类别 | 推荐接口 | 典型内容 | 当前项目例子 |
|---|---|---|---|
| 连续观测流 | 有界 Topic | scan、joint、odom、图像、健康采样 | `/scan_from_cloud`、关节与定位反馈 |
| 连续命令流 | 有界 Topic | 速度、控制目标；携带或在受控入口绑定授权与时效 | 既有 `/cmd_vel` 控制链 |
| 版本化快照 | Topic 或有界查询 Service | WorldSnapshot、能力、包络、场景摘要 | 地图/定位/RobotEnvelope 状态 |
| 短查询/提交 | Service | 准入检查、配置、包络提交、状态查询 | `SetRobotEnvelope` 类接口 |
| 可取消长任务 | Action | 导航、规划、抓取、运输、探索 | Nav2 和 MTC Action |
| 任务事件/审计 | 追加事件流或持久化记录 | accepted、preempted、stop confirmed、result | `events.jsonl`、任务状态 |
| 诊断/解释 | 结构化状态 + 文本说明 | reason code、source、建议动作 | `health_reason`、失败码 |

- Topic 可以承载观测、命令或事件；通信形式本身不授予执行权。连续命令契约必须定义
  唯一授权 writer、资源/lease epoch、有效期、旧命令拒收、失联/超时 watchdog 及安全动作。
  原始消息无这些字段时，由受控入口和执行侧维护等价关联；不能假定消息自带租约。
- 命令只通过既有仲裁、Nav2 控制器、速度平滑和设备适配链；任务层仍不得直接发布速度、关节或 SDK 命令。
  观测和诊断订阅者不能因读到命令而获得写入权。
- Service 成功的含义由契约限定，不能自动等价于物理完成；Action result 必须和 feedback、
  实测状态分开。
- 高频流必须有边界、丢帧/过期计数和消费策略；不能用无限队列掩盖回调阻塞。
- 将需要跨模块决策的字符串 JSON 逐步收敛为带版本的强类型消息；文本保留为诊断补充。

## 3. 模块卡片与责任边界

新增模块或改变状态/副作用所有权时，按需复制 [模块卡模板](assets/module-card.md)；
新增或修改跨模块接口时使用 [接口契约模板](assets/interface-contract.md)。对不受影响的字段
标注“不涉及”及原因，不为局部改动虚构新模块或重写整机设计。

责任边界至少回答：

- 谁创建数据，谁验证数据，谁融合数据，谁将数据投影到另一个表示空间；
- 谁拥有状态机，谁可以改变状态，谁只能观察；
- 谁产生规划候选，谁批准执行，谁发出设备命令，谁可以最终阻止命令；
- 谁负责超时，谁负责取消，谁确认停稳，谁提交任务/对象/载荷账本；
- 模块删除、重启或数据过期后，下游是等待、降级、停车、恢复还是人工接管。

当前项目的建议映射：

```text
astribot_s1_perception / perception_components
  -> ObservationAdapter + quality/provenance
astribot_s1_slam / mapping
  -> localization/map revisions and frames
astribot_navigation_msgs / bridge_msgs / slam_msgs
  -> domain contracts, not a universal message dump
navigation_policy / exploration / TaskArbiter
  -> decision, candidate and navigation-task ownership
MoveIt/MTC / transport
  -> manipulation plan, scene and object/attachment transaction
path_tracking / Nav2 / navigation constraints
  -> motion admission, speed/stop decisions and bounded motion execution
trajectory_bridge / SDK / Gazebo effort
  -> device adaptation and measured feedback
logging / evidence tools
  -> observation only; never a second controller
```

这些是架构映射，不代表每个模块已经实现了这里的全部契约。

## 4. 数据生命周期与版本

任何可能影响运动的事实都要说明生命周期：

```text
created -> timestamped -> transformed -> quality-gated
  -> published/cached -> consumed
  -> expired/revoked -> diagnosed/archived
```

最小元数据包括：

```text
source_id / source_epoch
sample_stamp / receive_stamp / clock_epoch
frame_id / transform_chain / calibration_revision
schema_version / map_revision / localization_revision
scene_revision / envelope_epoch / task_id
quality / covariance / valid_until / health_reason
```

- 版本不是装饰字段。地图、定位、标定、PlanningScene、载荷、包络或任务上下文变化时，
  依赖它们的计划必须重新验证或失效。
- 生产者发布“当前值”时必须定义是否允许迟到样本覆盖新样本；通常以源时间和 epoch
  双重判断，不能只比较 ROS 接收顺序。
- 缓存必须写明一致性策略：latest-valid、snapshot-at-time、transactional commit 或
  append-only event。不同策略不能混用而不标注。
- `tf2` 查询成功只代表变换可计算；不能单独证明定位质量、标定有效、对象身份或数据
  新鲜度。

## 5. QoS、时间和背压

对每个 Topic/Action/Service 记录以下契约：

```text
reliability / durability / history / depth
deadline / lifespan / liveliness
source timestamp / receive timestamp / ROS or wall clock
expected update rate / max age / queue bound / drop policy
```

- 传感器流优先采用与发布端匹配的 sensor-data QoS；状态、版本、确认和故障使用可靠、
  有界的 QoS。先检查兼容性，不靠默认 depth 猜测。
- callback 只做轻量接收、序号检查、时间/质量门控和有界入队；融合、点云、规划和报表
  使用可测的 worker 或组件执行。
- 背压策略必须明确是丢旧、丢新、降采样、暂停生产者还是触发降级；不能让内存增长
  代替安全策略。
- 仿真使用 `/clock` 时要同时处理时钟停滞和回退；真机要区分源时间与墙钟接收延迟。

## 6. 依赖图与模块拆分

每张图先声明箭头含义，不把下列关系混为同一种依赖：

1. **源码/构建依赖**：`依赖者 -> 被依赖的库/接口`；检查头文件、链接、包依赖及禁止环；
2. **运行时调用**：`调用者 -> 被调用端口/实现`；核心可调用其自身定义的抽象端口；
3. **消息流**：`生产者 -> 消费者`；标注观测/命令/事件及其契约；
4. **运行/资源关系**：启动和生命周期先决条件、租约持有者、资源申请/归还者；
5. **证据先决条件**：哪些检查支持本次主张，哪些运行记录尚未取得。

模块拆分遵循：

- 按变化原因拆包：设备驱动、领域算法、接口定义、组合启动和验证工具不要因目录方便
  混在一起；
- 源码依赖为 `ROS/SDK adapter -> core/contracts`；抽象端口归核心或稳定 contracts 所有。
  核心不能反向依赖 ROS 节点、厂家 SDK 或 bringup；组合入口负责注入具体适配器；
- 一个模块只拥有一种主要副作用；规划模块不能同时成为底盘写入者，日志模块不能成为
  运动授权者；
- 允许多个消费者读取同一数据，但一个写入状态或设备命令只能有明确唯一所有者；
- 同一领域消息放在稳定的 contracts 包，避免产生“万能消息包”或跨域循环依赖；
- 将配置、启动、算法和执行适配拆开时，保留反向 launch 依赖、插件 ABI 和旧配置兼容
  清单，不能只按文件移动判断完成。

## 7. 模块内实现模板

典型结构如下；箭头只表示源码依赖，不表示消息或调用顺序：

```text
ROS adapter           -> domain core / contracts
backend adapter       -> core-owned ports / contracts
backend adapter       -> Nav2 / MoveIt / SDK libraries
core                  -> stable domain contracts（不依赖 concrete adapters）
```

组合入口负责装配核心与具体适配器，启动文件负责进程组合。运行时可由
`core -> 抽象端口 -> 注入的 backend adapter` 发起请求；回调、worker、
executor 与有界队列属于执行模型，另行标注线程和队列所有者，不当作必经模块链。

- 领域核心保存不变量、状态转移、版本门控、调度比较、几何/约束和故障判定；应能脱离
  ROS 做单元测试。
- ROS 适配层负责消息转换、QoS、生命周期、线程模型、Action 取消和参数快照；不要在
  callback 中执行无界阻塞。
- backend adapter 负责协议、能力差异、反馈质量和错误码；上层不直接拼厂家命令。
- 实现语言遵循 [项目语言约定](../robot-runtime-cpp/SKILL.md)，适配层不因“薄”而自动获得语言例外。
- 事件、指标和日志通过关联 ID 读取状态；它们不能通过订阅控制话题形成第二条控制链。

## 8. 设计和验证输出

跨模块主张或接口变更的边界验收使用 [robot-scenario-boundary-validation](../robot-scenario-boundary-validation/SKILL.md)；检查方案中“数据存在→能力成立”的推断时使用 [robot-argument-audit](../robot-argument-audit/SKILL.md)。

输出与影响范围对应，复用既有图表并标明本次变化：

- 局部接口变更：受影响接口契约、两端责任、兼容/失败行为及对应验证；
- 新模块或模块拆分：模块卡、源码依赖与装配关系、状态/资源所有权、线程和故障边界；
- 跨模块运动链或整机架构：补数据流和控制权视图、上下文字典、依赖/故障传播及证据矩阵。

从实际变更选择验证：消息/版本转换、QoS 兼容性、队列上限、迟到样本、重复事件、重启、
取消竞争、上下文失效、命令时效/watchdog 和单一写入者。说明未覆盖项及其对结论的影响；
无需为纯局部改动生成全部视图。不能用“Topic 存在”“节点启动”“Action 返回成功”推断
数据流已闭环或机器人已经安全执行。

## 9. 当前项目限制

当前项目已经有领域消息包、导航仲裁、SLAM/mapping/perception 分层、MoveIt/MTC 运输
接口、设备桥和统一日志，但仍需逐步补齐跨模块数据字典、端到端 provenance、消息版本
兼容策略和整机任务/世界/执行三者的统一上下文。

新增模块时先复用已有 `astribot_navigation_msgs`、`astribot_bridge_msgs`、
`astribot_slam_msgs`、`RobotEnvelope`、任务/事件关联 ID 和现有控制链。不要因为追求“统一
数据流”而把二维 costmap、三维 PlanningScene、对象账本、设备反馈或日志合成一个无界
全局状态；它们应通过对象身份、时间、坐标、版本和提交事件建立可审计联系。
