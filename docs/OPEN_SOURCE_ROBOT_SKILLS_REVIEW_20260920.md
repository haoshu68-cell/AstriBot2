# 轮式双臂机器人开源架构与 Agent Skills 检索整理

日期：2026-09-20。目标：检索能帮助“软件架构设计、模块交互、模块内实现和验证”的开源机器人框架与 Agent Skills，并把可迁移的决策规则整理为当前 AstriBot ROS 2 Humble 项目的仓库技能。

2026-09-22 增补：[方案评审、论证链与多场景验证接入](ROBOT_REVIEW_SKILLS_INTEGRATION_20260922.md)。包含 Doubao 公开镜像来源/许可边界、当前代码映射和可复用模板；不把公开可读的镜像直接标作已核实开源许可。

这不是对互联网的穷尽证明，也不是对任何框架的安全认证。检索结果按公开仓库/官方文档的当前页面整理；版本、分支和 ROS 发行版仍需在实际构建前核对。

## 1. 检索结果和可复用模式

| 来源 | 公开协议/形态 | 对架构和模块实现最有用的部分 | 当前项目采用方式 |
|---|---|---|---|
| [ROS 2 interfaces](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html) | 官方文档 | topic 适合连续流，service 适合短请求，action 适合带反馈/取消的长任务 | 把整机任务、MTC 规划、导航执行区分为 Action/Service/Topic，不把长任务塞进同步 service |
| [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | 官方文档 | sensor-data 的时效优先、QoS 兼容性、deadline/liveliness 事件 | 在感知世界快照技能中要求记录 QoS、采样/接收时刻和丢帧/过期原因 |
| [Nav2](https://docs.nav2.org/) | Apache-2.0 开源框架/官方文档 | 生命周期节点、行为树编排、模块化 task server、planner/controller/behavior 插件 | 保留 Nav2 为导航执行端；任务层只编排和仲裁，不复制速度控制器 |
| [ros2_control Humble](https://control.ros.org/humble/doc/getting_started/getting_started.html) | Apache-2.0 开源框架/官方文档 | Controller Manager—Resource Manager—Hardware Interface；`read/update/write`；pluginlib；接口互斥 | 用于执行所有权和硬件边界审计；不把当前厂家 SDK 桥未经核对地改写成 ros2_control |
| [ros2_control Controller Manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html) | 官方文档 | 控制器/硬件生命周期、实时循环、切换和错误停用 | 固化控制器切换、限位、watchdog、取消和实测停稳的证据要求 |
| [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor) | BSD-3-Clause | 基于 PlanningScene 的层级 stages、顺序/候选/并行容器、可诊断的多阶段规划 | 复用现有 `/transport/plan_manipulation`；MTC 只产生分阶段计划，不取得底盘或设备控制权 |
| [TIAGo++ robot](https://github.com/pal-robotics/tiago_dual_robot) | Apache-2.0 | 双臂移动平台把 description、controller configuration、bringup 分开，URDF/Xacro 参数化配置 | 作为包/启动/控制配置的参考，不复制其硬件参数或声明当前项目已具备相同能力 |
| [Autoware architecture](https://docs.autoware.org/pr-803/design/autoware-architecture-v1/) | Apache-2.0 生态/官方文档 | sensing、map、localization、perception、planning、control、vehicle interface 的分层和可替换模块 | 为当前项目补充“感知/定位/规划/控制/设备接口/监控”视图；映射到 Nav2、SLAM、MoveIt、桥接，不引入车辆 HD map 或 Cyber RT |
| [Autoware Vehicle Interface](https://autowarefoundation.github.io/autoware-documentation/latest/design/autoware-architecture/vehicle/) | 官方架构设计 | 车辆特定协议适配、能力诊断、自治/人工控制模式和状态反馈 | 约束底盘/双臂/夹爪 SDK 适配层，任务与规划不直接依赖厂家协议 |
| [Apollo software architecture](https://github.com/ApolloAuto/apollo/blob/master/docs/14_Others/Apollo_5.5_Software_Architecture.md) | Apache-2.0 开源平台/架构文档 | Planning、Control、CanBus、Guardian 等模块的消息边界与安全命令分离 | 只借鉴控制权和消息依赖审查；不直接移植 Apollo 的 Cyber RT 或道路场景状态机 |
| [BehaviorTree.CPP ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) | 开源 C++ 行为树库 | action 节点、ports/blackboard、可组合行为 | 仅用于 Nav2/任务编排层；高频控制仍由现有控制器/保护链负责 |
| [OOMWOO](https://github.com/makerspet/oomwoo) | 开源 ROS2 真空机器人设计 | 模块接口先行、仿真优先、CPU/MCU 分层、硬安全独立于 Linux/ROS2、接口文档作为模块契约 | 提炼为区域任务、定位丢失恢复、脱困和报警技能；不假设当前项目已有独立 MCU 安全控制器 |
| [OOMWOO I/O firmware](https://github.com/makerspet/oomwoo-io-firmware) | 开源 MCU 固件设计 | 把硬安全 cutoff、过流、CPU watchdog 放在确定性实时层；上层贡献代码不能饿死安全层 | 作为未来 SDK/硬件安全审查的参考；当前项目继续使用已有桥接/最终保护，未宣称有同等 MCU 证据 |
| [Valetudo](https://github.com/tarik02/valetudo) | Apache-2.0 本地化扫地机控制层 | 本地控制、地图/任务抽象、REST/MQTT 适配和设备能力边界；公开错误类型覆盖卡住、悬空、堵塞和被困 | operator/远程入口只提交任务、暂停或查询，不能绕过 TaskArbiter 和设备保护 |
| [OpenMower](https://github.com/ClemensElflein/OpenMower) | 开源割草机软硬件 | 多区域边界、障碍/天气暂停、抬升或碰撞急停、RTK/设备状态 | 提炼 keep-in/keep-out、暂停恢复、硬停止、脱困预算和报警锁存；不移植 RTK 或割草策略 |
| [OpenMowerNext](https://github.com/jkaflik/OpenMowerNext) | Apache-2.0 ROS2 端口 | ROS2 化割草栈的模块迁移和能力演进边界 | 作为 ROS2 户外任务参考，不把 Jazzy 项目当作 Humble 兼容证明 |
| [Robotics Agent Skills](https://github.com/arpitg1304/robotics-agent-skills) | Apache-2.0 Agent Skills | lifecycle、QoS、有限缓冲、时间同步、watchdog、bringup、测试和安全路由 | 吸收其决策规则，重新写成当前项目约束；不直接拷贝其文件或声称已验证其示例 |
| [ROS 2 Skill](https://github.com/adityakamath/ros2-skill) | Apache-2.0 Agent Skill | profile-first + live introspection、preflight、resolve→act→verify | 只借鉴探查前置和执行后核验；当前项目的运动控制仍受本仓库硬件与栈操作技能约束 |
| [ROS 2 Copilot Skills](https://github.com/wimblerobotics/ros2-copilot-skills) | Apache-2.0 Agent Skills 集合 | 按 ROS2 core、Nav2、BT、SLAM、感知、硬件、仿真、URDF 分类并按需加载 | 采用“按任务触发、避免加载万能技能”的组织方式；不将其 158 项直接并入仓库 |

## 2. 当前项目对照

当前仓库已经具备或已有文档约束的能力：

- `TaskArbiter`、Nav2/策略/跟踪和独立最终保护；
- `astribot_s1_manipulation`、MoveIt 配置、`astribot_s1_transport` 和 `astribot_s1_transport_mtc`；
- `RobotEnvelope`/`SetRobotEnvelope`、双 costmap 确认和版本字段；
- SLAM、mapping、perception、消息按领域分包；
- C++ 运行时优先、统一 spdlog、仿真/真机操作和路径跟踪指标手册。

仍需要在后续工程工作中持续固化的边界：

- 统一的任务接收、准入、排队、优先级、资源租约、取消/暂停/恢复和结果回传链路；
- 跨感知、世界模型、任务、规划、执行和设备反馈的数据字典、QoS、时间/坐标和版本生命周期；
- 统一持有底盘、双臂、躯干、夹爪和场景资源的整机任务层；
- 抓持/附着/运输/包络/导航/放置的真实状态事务；
- 物体、载荷、标定、定位和场景版本组成的共享世界快照；
- 规划成功、执行成功、实测停稳、附着/释放确认的区分；
- 控制器/SDK 写入所有权、取消屏障和真机安全证据；
- 静态、离线、整栈仿真和真机验收不互相越级。

这些是“需要用技能约束后续开发”的边界，不表示每项都缺少任何实现。具体当前状态以仓库源码和架构审查为准。

## 3. 已整理到仓库的项目技能

| 技能 | 触发场景 | 主要固化内容 |
|---|---|---|
| [`robot-design-review`](../.agents/skills/robot-design-review/SKILL.md) | 方案可行性、硬伤、需求和风险评审 | 只读诊断、方案取舍、共识/分歧求证、产品/行业/安全适用性按需检查 |
| [`robot-argument-audit`](../.agents/skills/robot-argument-audit/SKILL.md) | 设计或报告论证链、隐含假设、证据外推 | 主张/前提/证据/推理/成立范围、反例、事实与推断分开 |
| [`robot-scenario-boundary-validation`](../.agents/skills/robot-scenario-boundary-validation/SKILL.md) | 多场景、边界、故障、恢复、负载、迁移验收 | 场景目录和卡片、独立判据、时序竞争、覆盖追踪、功能/安全/性能分开 |
| [`astribot-architecture-design`](../.agents/skills/astribot-architecture-design/SKILL.md) | 架构设计总入口、架构评审、重构规划、接口演进 | 路由子技能、七类架构视图、四条控制边界、契约字段、决策与证据状态 |
| [`mobile-dual-arm-architecture`](../.agents/skills/mobile-dual-arm-architecture/SKILL.md) | 架构设计、包拆分、模块交互、接口审查 | 控制权、架构视图、消息边界、版本/epoch、现有与建议分离 |
| [`robot-dataflow-module-design`](../.agents/skills/robot-dataflow-module-design/SKILL.md) | 数据流、模块边界、接口契约、QoS、依赖图、数据生命周期、模块内实现 | 数据流/控制流双视图、模块卡片、数据语义、provenance、背压、依赖环和证据矩阵 |
| [`robot-task-intake-execution`](../.agents/skills/robot-task-intake-execution/SKILL.md) | 任务接收、准入、排队、优先级、资源冲突、执行监督、取消/暂停/恢复、结果回传 | TaskEnvelope、生命周期状态机、幂等与抢占、子 Action 监督、停止确认、持久化与恢复 |
| [`robot-task-orchestration`](../.agents/skills/robot-task-orchestration/SKILL.md) | 搬运、抓取、放置、运输姿态、Nav2+MoveIt/MTC | 资源租约、分阶段事务、MTC/PlanningScene/账本分离、包络握手、取消屏障 |
| [`ros2-control-hardware-safety`](../.agents/skills/ros2-control-hardware-safety/SKILL.md) | 控制器、SDK 桥、硬件 bringup、实时执行 | Controller Manager/Resource Manager/硬件接口所有权、read/update/write、限位和真机门槛 |
| [`perception-world-model`](../.agents/skills/perception-world-model/SKILL.md) | SLAM、点云、相机、标记、对象、导航/操作场景融合 | ObservationAdapter→WorldSnapshot、坐标/时间/标定/QoS/质量/版本、二维与三维边界 |
| [`autonomy-stack-architecture`](../.agents/skills/autonomy-stack-architecture/SKILL.md) | 自主系统分层、车辆/设备接口、控制模式、系统监控 | 融合 Autoware/Apollo 的分层经验，映射到当前 ROS2/Nav2/MoveIt/SDK |
| [`robotics-validation-evidence`](../.agents/skills/robotics-validation-evidence/SKILL.md) | 回归、仿真、真机报告、指标和日志 | 实现/离线/仿真/真机分层、关联 ID、阶段过滤、成功/失败分离、证据字段 |
| [`robot-mission-boundary-safety`](../.agents/skills/robot-mission-boundary-safety/SKILL.md) | 区域任务、覆盖/探索、禁行边界、脱困、报警、回充/远程控制 | OOMWOO/OpenMower/Valetudo 的接口先行、分级恢复、重试预算、报警锁存与人工接管 |

它们与已有 [`robot-runtime-cpp`](../.agents/skills/robot-runtime-cpp/SKILL.md) 和 [`ros2-stack-ops`](../.agents/skills/ros2-stack-ops/SKILL.md) 互补：前者约束运行时语言和迁移，后者约束共享 ROS2/Gazebo 栈操作。

## 4. 推荐加载组合

| 工作类型 | 技能组合 |
|---|---|
| 只读方案论证与硬伤评审 | `robot-design-review` + `robot-argument-audit` + 受影响领域技能 |
| 边界条件与多场景验收设计 | `robot-scenario-boundary-validation` + `robotics-validation-evidence` |
| 设计整机模块/接口 | `astribot-architecture-design` → `mobile-dual-arm-architecture` + `robotics-validation-evidence` |
| 设计跨模块数据流/依赖图 | `astribot-architecture-design` → `robot-dataflow-module-design` + `mobile-dual-arm-architecture` + `perception-world-model` |
| 设计任务接收与执行总线 | `astribot-architecture-design` → `robot-task-intake-execution` + `robotics-validation-evidence` |
| 实现双臂搬运/抓放 | `robot-task-intake-execution` + `robot-task-orchestration` + `mobile-dual-arm-architecture` + `robot-runtime-cpp` |
| 修改控制器/桥接/硬件启动 | `ros2-control-hardware-safety` + `robot-runtime-cpp`；操作栈时再加 `ros2-stack-ops` |
| 接入相机/点云/SLAM/物体 | `perception-world-model` + `autonomy-stack-architecture` |
| 设计导航/决策/控制/设备接口分层 | `autonomy-stack-architecture` + `mobile-dual-arm-architecture` + `ros2-control-hardware-safety` |
| 做仿真/真机回归与报告 | `robotics-validation-evidence`；探查或清理共享栈时再加 `ros2-stack-ops` |
| 设计探索/覆盖/回充/远程任务 | `robot-task-intake-execution` + `robot-mission-boundary-safety` + `robot-task-orchestration` + `robotics-validation-evidence` |

## 5. 不直接采用的内容

- 不把通用 Agent Skill 仓库的安装脚本或实时控制 CLI 直接放入项目；当前仓库已有设备准入、日志和栈安全边界。
- 不因 TIAGo++、UR 或其他平台的控制器/URDF 参数而改写 AstriBot 设备模型。
- 不把 Nav2 的模块化架构解释成“导航已经拥有双臂任务控制权”。
- 不把 MTC 的规划成功解释成夹持、附着、放置、停稳或真机同步执行已成功。
- 不把扫地机/割草机的“自动脱困”理解成可以无限倒车、旋转或重试；当前项目必须先通过包络、碰撞、定位、停车和资源租约检查。
- 不把报警 UI、REST/MQTT 或 ROS 心跳当作急停、脱困完成或硬件安全已生效的证明。
- 不把论文/示例、离线测试或 Gazebo 真值当成真机安全或外部物理精度验收。
