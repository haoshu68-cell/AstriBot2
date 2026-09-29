# 项目 Skills 整体盘点

日期：2026-09-22。范围：`.agents/skills` 下全部 15 个项目技能及其模板、参考材料和关键源码锚点。本次只读评审技能，新增此报告与静态检查记录，没有修改技能、机器人源码或运行环境。

后续状态：用户已授权整改，技能与文档修正见 [整改记录](SKILLS_REMEDIATION_20260922.md)。下文保留整改前发现和当时行号，不是对修改后版本的重新判断；前后差异已保存于整改证据目录。

## 结论

现有技能覆盖方向合理：整机架构、数据流、任务入口、搬运事务、世界模型、控制安全、脱困报警、论证核查及场景验证已经形成相互补充的体系。当前主要问题是规则正确性、资料时效和调用粒度，而不是技能数量不足。建议先整改再扩充，不继续按应用行业增加平行架构入口。

最应保留的约束是：导航仲裁不冒充整机资源管理、规划结果不授予设备执行权、取消回执不等于停稳、保存不等于应用、候选不等于抓取、仿真不等于真机、预期拒绝不计业务成功。新增评审技能也已正确处理“共识不是事实”“同一评审者双视角不是独立复审”。

下述 P1 表示可能直接误导共享环境的破坏性操作，应优先修正；P2 表示会误导设计、诊断、技能选择或证据判断。严重度评价针对技能指令及其使用风险，不表示已复现机器人运行故障。

## 已确认的问题及整改判据

### F-01 / P1：清理脚本没有在执行点约束进程归属

- 证据：[ros2-stack-ops:74](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:74) 按全机进程名称匹配，仅排除自身祖先和部分工具 shell；[93 行](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:93) 直接对匹配集合发送信号，并删除全局 Fast DDS 共享内存文件。
- 后面的 [归属预检查](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:261) 只扫描 `ros2 launch` 的输出路径，且没有被清理函数强制调用。直接启动的进程、遗留子进程、并发新启动和 PID 复用不能靠该检查完整排除。技能自身也承认会杀掉其他会话。
- 影响：用户只要求清理自己的仿真时，照抄示例可能影响其他任务。排除工具 shell 解决的是误匹配自身，不能证明目标属于当前会话。
- 建议：移除通用名称清理作为默认操作路径；在实际停止入口验证会话、启动身份、进程归属和受控进程树，只停止已确认拥有的资源。归属未知则不停止该资源；不以全局共享内存删除作为每次清理必做步骤。
- 关闭判据 SC-SKILL-OPS：使用模拟进程表验证其他会话、直接启动进程、工具 shell、PID 复用和归属未知均不进入停止集合。静态阅读不足以证明新工具实现安全，后续整改应在隔离夹具中验证；本轮未执行清理脚本。

### F-02 / P2：操作技能的隔离方式和就绪判断已过时

- 证据：[ros2-stack-ops:123](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:123) 及 [301 行](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:301) 声称 domain 钉死为 25、不能用于隔离。当前 [warehouse_sim.launch.py:249](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py:249) 声明可配置 `ros_domain_id`，303 行使用该配置；[supervisor:198](/home/yjh/WorkSpace/astribot_sdk_ros2/tools/sim_stack_supervisor.py:198) 已传入该参数。
- 技能 [137 行](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:137) 从全机第一个名称匹配的节点取环境，未绑定目标会话；149 行没有复制 Gazebo partition。当前 [supervisor:400](/home/yjh/WorkSpace/astribot_sdk_ros2/tools/sim_stack_supervisor.py:400) 已记录包含 domain、partition 和端口的会话环境。
- [204–211 行的诊断示例](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/ros2-stack-ops/SKILL.md:204) 把话题数、约 196% CPU 和 `/stats` 无消息直接映射为是否步进。没有先证明查询对象、partition、统计话题与观测窗口正确，无法作这种判定；一帧统计也不能证明持续推进。
- 建议：保留“从实际目标会话核对环境”的原则，更新为当前启动接口和会话记录；domain 是隔离的一部分，仍需 partition、端口、安装目录和所有权。话题数量与 CPU 历史值仅作背景，使用目标世界统计中的时间/迭代推进及实际数据链判断就绪。
- 关闭判据 SC-SKILL-ENV：双会话、错误 domain/partition、世界暂停、无统计消息的输入应给出不同结论；缺观测时保留未知，不断言物理完全未运行。

### F-03 / P2：数据流技能混淆源码依赖方向与通信语义

- 证据：[robot-dataflow-module-design:177](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-dataflow-module-design/SKILL.md:177) 写“依赖方向从 contracts/纯算法指向适配器”，紧接着又禁止核心依赖 SDK/ROS，前后方向矛盾。190 行开始的箭头也没有标注是调用流、数据流还是源码依赖。
- 正确表达应区分：源码依赖通常为 `adapter → core/contracts`；核心通过自身定义的端口请求后端服务，运行时调用方向可与源码依赖不同。组合入口负责装配。
- 同技能 [61 行](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-dataflow-module-design/SKILL.md:61) 的“Topic 只提供观测”过于绝对。当前 [cmd_vel_body_to_world_node.cpp:77](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/src/cmd_vel_body_to_world_node.cpp:77) 使用 Topic 接收和发布速度命令。合理的不变量是通信形式本身不授予执行权。
- 建议：分别标注编译依赖、调用、消息和资源关系；接口表补“有界命令流”，要求写入者、控制权、失效截止时间与看门狗。任务入口仍不得因此绕过执行保护直接发速度。
- 关闭判据 SC-SKILL-DATAFLOW：用一个无 ROS 的 C++ 核心和 ROS/SDK 适配器的模块设计题，检查依赖无反转；用已有速度 Topic 作为反例，不能为了满足文字规则建议把连续控制改成长任务 Action。

### F-04 / P2：任务状态图和释放屏障存在歧义及顺序冲突

- 证据：[robot-task-intake-execution:73](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-task-intake-execution/SKILL.md:73) 将状态清单画成一条链，容易读成 `REJECTED → QUEUED`，或正常成功必须先经历暂停、取消。
- [86 行](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-task-intake-execution/SKILL.md:86) 要求包含“资源归还、任务终态”的事件按所列顺序记录；92 行又要求终态写入后才能释放租约。未明确区分子 Action 终态与业务任务终态，形成相反的前后要求。
- 建议：用分支状态图或转换表写清事件、前置条件、下一状态与副作用。分别定义子执行终态、实测安全状态、任务结果持久化、资源交接的提交条件；异步回执记录源时间与接收时间，不强制不必要的全序。
- [任务编排:36](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-task-orchestration/SKILL.md:36) 的“不启动下一个任务”也应明确作用于冲突资源及依赖链；独立只读任务不应无条件等待整机所有动作终止。
- 关闭判据 SC-SKILL-TASK：正常成功、准入拒绝、排队取消、执行取消与成功竞争、终态先于取消回执到达、停止状态未知、持久化失败、无资源冲突的并发请求，均有不矛盾的预期。未知停稳状态仍不得重新授予相关运动权限。

### F-05 / P2：必读关系过宽，同一交付要求重复出现

- 证据：[总入口:21](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/astribot-architecture-design/SKILL.md:21) 要求整机问题同时加载两种架构技能，数据流问题固定加载三个技能，任务入口问题固定加载搬运编排。[区域任务技能:8](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-mission-boundary-safety/SKILL.md:8) 默认要求五个前置技能，后者还继续引用其他必读材料。
- 总入口 58 行、整机技能 34 行和数据流技能 211 行都要求成套架构产物。纯任务去重、局部接口调整、整理既有证据不一定需要双臂规划或七张全新架构视图。
- 这是可见的加载和交付要求重叠；本轮没有测量它增加了多少时间或 token，不声称已证明性能损失。
- 建议：每类问题选择一个主技能，其余按实际影响条件加载；共享不变量只有一个维护位置。小改动引用已有视图并只提交受影响差异，跨模块设计才要求完整视图。
- 同时收窄 [证据技能:55](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robotics-validation-evidence/SKILL.md:55) 的“仿真证据完整后”通用门槛：真实运动保留相关仿真和硬件准入要求；只读设备健康检查、标定数据获取等按其实际风险与目的定义前置条件，避免把与任务无关的全栈仿真作为必备条件。
- 关闭判据 SC-SKILL-ROUTING：局部 QoS 设计不无条件加载搬运；请求去重只在涉及搬运事务时扩展；整理既有日志不启动栈、不重做全套验收；整机搬运评审仍覆盖必要领域。

### F-06 / P2：默认资料入口没有跟上当前源码

- 多个技能将 [当前架构参考](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/manuals/ARCHITECTURE_REFERENCE.md:36) 列为先读材料。该文仍写 19 个 ROS 包，71 行链接已不存在的 Python `task_arbiter_node.py`。
- 本轮在 `ws_robot/src` 发现 45 个 `package.xml` 文件，导航仲裁的可读入口为 [C++ 实现](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_task_arbiter_native/src/task_arbiter_node.cpp)。文件计数不等于可构建包数、上线模块数或能力验收。
- 新增的 [项目场景目录](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-scenario-boundary-validation/references/project-scenarios.md:3) 已提示日期、迁移和重新核对，是应推广的做法。不能因为新目录正确就认为所有旧入口也已更新。
- 建议：维护一个精简的“领域→当前源码入口→状态/限制→核对日期”索引，供各技能复用；有日期的设计审查保留为历史决策依据。源码、采用版本、安装产物和历史实验分别核对。
- 关闭判据 SC-SKILL-SOURCE：迁移掉一个旧文件后，路由到现有实现而非断言能力缺失；不仅检查技能直接链接，还检查作为必读事实来源的文档关键源码链接。

### F-07 / P2：Python 例外范围表述不一致

- [robot-runtime-cpp:8](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-runtime-cpp/SKILL.md:8) 将持续运行逻辑优先 C++、Python 主要用于启动和验证，并将薄绑定放在既有适配迁移语境中。
- [区域任务技能:76](/home/yjh/WorkSpace/astribot_sdk_ros2/.agents/skills/robot-mission-boundary-safety/SKILL.md:76) 却广泛允许区域编辑、REST/MQTT 使用 Python；总入口 81 行、数据流 204 行和控制安全 54 行也没有明确“薄适配”的适用边界。容易将历史迁移例外扩张为新运行模块的默认语言选择。
- 建议：以 `robot-runtime-cpp` 为语言政策唯一入口；其他技能只引用。明确新运行时模块、现有绑定、启动/离线验证三种情形，保留必要例外的理由，不因语言统一重写已验证链路。
- 关闭判据 SC-SKILL-LANGUAGE：新区域编辑/任务网关持续运行逻辑采用项目 C++ 约定；已有 Python 绑定可在受限迁移计划内保留；一次性验证脚本不会被要求改成 C++。

### F-08 / P2：技能行为验证仍不足以支持整套技能稳定有效

- [既有验证记录:7](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/review_skills_20260922/validation.md:7) 显示基线已经识别主要证据越级；[32 行](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/review_skills_20260922/validation.md:32) 明确同一评审者先后试评，加载后还补了输入参数，不是盲测或改善率对照。这些诚实限制应保留。
- 已有试评支持新增模板和场景卡的有限可用性，但不能关闭本报告的操作风险、状态机冲突、资料漂移或路由过宽。格式通过也不能关闭这些问题。
- 建议：建立小型固定请求集，覆盖应触发、不应触发、正常受限结论、任务竞争、来源过期、多会话归属和拒绝/成功分母。比较相同输入、相同边界下的决策是否正确，不以标题齐全或措辞匹配评分。
- 关闭判据 SC-SKILL-EVAL：对 F-01～F-07 的关键反例保留输入、实际输出、正确行为及失败样本；可用时做独立评审，服务不可用则明确仅有本地单人检查，不以角色标签代替独立性。具体题数按风险选择，不设为了凑数量的门槛。

## 15 个技能的逐项处置建议

| 技能 | 判断 | 建议处置 |
|---|---|---|
| `astribot-architecture-design` | 必须保留的总入口，当前承担过多正文 | 保留简短路由、基线确认和按影响选择产物；领域约束归各主技能 |
| `mobile-dual-arm-architecture` | 本项目核心，职责成立 | 保留整机控制权、资源与空间表示边界；不重复所有接口细则 |
| `autonomy-stack-architecture` | 借鉴价值存在，和整机技能重复较多 | 从默认必读移为按需行业对照；如后续不再有独立使用场景，再合入整机技能 references，并检查所有调用者 |
| `robot-dataflow-module-design` | 独立价值高，不能只并入总架构 | 修依赖/Topic 语义；保留模块卡、数据生命周期、背压；长模板按需读取 |
| `perception-world-model` | 职责清晰，应保留 | 聚焦时空、质量、版本与消费者一致性；共享字段字典避免多份维护 |
| `robot-task-intake-execution` | 用户明确需要，独立必要 | 修状态图与提交/释放顺序；补具体转换表和崩溃窗口用例，保留幂等与持久化 |
| `robot-task-orchestration` | 与任务入口互补，应保留 | 只在抓放/运输等事务时加载；明确受影响资源，保留规划、实测、账本的区别 |
| `robot-mission-boundary-safety` | 脱困/报警经验适配合理 | 保留恢复预算、episode、锁存与人工接管；减少必读前置，统一语言例外 |
| `ros2-control-hardware-safety` | 独立执行安全边界，应保留 | 按实际设备/控制器层使用；区分首次启动、切换与旧动作取消的前置条件 |
| `robot-runtime-cpp` | 短且必要，适合作为唯一政策入口 | 统一其他技能引用，保持分步迁移及性能/语义边界 |
| `ros2-stack-ops` | 操作不变量必要，现有示例急需整改 | 优先处理 F-01/F-02；历史故障放参考资料，正文引用当前受控操作入口 |
| `robot-design-review` | 适配合理，应保留 | 保持默认只读、按影响选领域、允许无缺陷；小评审只填相关模板部分 |
| `robot-argument-audit` | 与硬伤评审存在交集，但输出不同 | 保留按需入口，核查“为何成立”；不要要求每次硬伤评审都做双人全量流程 |
| `robot-scenario-boundary-validation` | 补足场景、边界和独立判据，应保留 | 保持 NOT_RUN、BLOCKED、INVALID 的分离；对停止事件用偏序及资源范围表达 |
| `robotics-validation-evidence` | 项目交付的基础约束，应保留 | 聚焦“已获得证据能证明什么”；离线整理不必加载操作手册或重跑全链 |

不建议合并“任务入口”和“搬运编排”，也不建议合并“验证方案”和“证据报告”：前者分别回答接收调度与具体事务，后者分别回答如何测与测得了什么。优先收敛的是重复的行业架构入口和无条件依赖。

## 建议的整改顺序与最小补充

1. 先修操作安全与过时隔离说明（F-01/F-02），再修设计规则的实质冲突（F-03/F-04）。
2. 精简总入口及必读关系；统一语言和当前源码索引（F-05～F-07）。无需改动机器人运行逻辑。
3. 在既有技能中按需补三个支持材料：任务状态/事件转换表、接口契约与模块卡模板、技能使用反例集。不要再为这三件事各建一个平行技能。
4. 在任务材料中细化已经要求但尚缺实例的提交边界：请求落盘与提交子动作之间崩溃、相同幂等键不同载荷、迟到结果与资源换代、写盘失败与未知物理状态。数据流材料细化接口兼容、配置生效/回滚和队列预算来源；脱困报警材料细化 episode 合并、告警升级和确认不解除故障。它们是现有职责的深化，不是声称项目完全未考虑这些问题。
5. 用固定反例集复核整改后的行为（F-08），通过后再考虑合并可选行业参考。此次提出建议，尚未实施这些修改。

## 本次验证及其边界

- 实际读取全部 15 个 `SKILL.md`，合计 1514 行，并检查评审/场景模板与关键源码。正文长度仅为盘点信息，不作为技能质量评分。
- 使用现有 `skill-creator` 格式校验器：15/15 通过。
- 扫描技能目录 Markdown 中的 85 处本地链接引用：目标均存在。这是链接出现次数，不是 85 个独立目标；没有验证 Markdown 标题锚点或递归检查所有被引用文档，因此不与 F-06 的旧文档源码断链矛盾。
- 检查记录及所读关键文件哈希见 [static_validation.json](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/skills_audit_20260922/static_validation.json)。工作区存在既有未提交/未跟踪内容，不能仅用 HEAD 代表所读版本。
- 本轮并行复核服务访问失败，最终结论为本地单评审者的静态审查；没有取得新的独立行为试评。旧试评记录仅作为历史材料核对，没有重跑其测试。
- 没有启动、停止或探查 ROS/Gazebo 运行栈，没有硬件操作。本轮没有重新核验全部外部仓库、许可证或标准；不据此宣称它们当前可用或适用。结论限定为本项目技能内容与所读源码之间的关系。
