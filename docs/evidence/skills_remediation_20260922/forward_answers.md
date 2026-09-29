# 冻结技能使用试答

本次仅应用题设与 `/tmp/astribot-skill-fix-forward-96bwc4z3/.agents/skills` 内的技能及必要参考；未读取项目源码、审计报告或其他答案，未联网、未操作 ROS/进程。以下是设计与资料判断，不是测试通过记录。

技能版本：上述冻结目录本次读取版本。将文末资料清单按相对路径排序，逐项形成 `文件 SHA-256 + 两个空格 + 相对路径 + 换行`，所得清单 SHA-256 为 `72f41c08ec4350e5153218814be4716b6d644baa852d315994136e110da6848a`。

## 1. 任务接收去重接口

主技能：`robot-task-intake-execution`。补充：`robot-dataflow-module-design` 的局部接口契约；实施时才需 `robot-runtime-cpp`。不加载抓放编排。

交付接收/去重契约、两端职责、错误与兼容行为及验证用例即可。幂等键限定请求者、操作和保留期，绑定规范化 payload；同键同内容返回已有任务，同键不同内容报冲突，终态重放不再执行。副作用前持久化键到任务映射，失败不启动请求。网关不写运动命令。保留期、并发去重原子性与重启策略尚待接口需求确定，无需重画整机视图。

## 2. 整理导航 CSV

主技能：`robotics-validation-evidence`。补充：新增分析脚本时用 `robot-runtime-cpp`；本题不加载栈操作或重新规划整套验证。

先核对 CSV 来源、版本、采样时间与频率、位姿来源、frame、阶段和结果码，再按固定公式整理。成功到位与取消/失败分开，FOLLOW 与终端阶段分开，注明过滤和样本数；采样极值不扩写成连续最大误差。保留原始文件与结果绝对路径、缺项及未覆盖范围。本题未提供 CSV 和指标公式，只能给流程，不能产生实际数值或声称真机精度。

## 3. 实现语言

主技能：`robot-runtime-cpp`。条件补充：后续设计网关生命周期或区域业务时再选对应领域技能。

区域编辑器、REST 任务网关、持久化服务均优先 C++，因为它们承担常驻功能、接口或状态副作用；不能因 UI、薄适配或脚本名称改选 Python。启动文件与离线验证脚本优先 Python，核心行为单元测试可以用 C++。配置保留已有格式，必要 shell 包装无需重写。现存 Python 或第三方绑定可按兼容边界保留，但不因此默认新增 Python 业务实现。

## 4. 编译依赖与运行调用

主技能：`robot-dataflow-module-design`。补充：`robot-runtime-cpp`；只有实际改变整机控制权才补整机架构技能。

编译箭头表示依赖者→被依赖者：核心→稳定领域 contracts；ROS 适配→核心/contracts 和 ROS 库；SDK 适配→核心拥有的抽象端口/contracts 及厂家 SDK。核心禁止反向依赖 ROS、SDK 实现或 bringup。组合入口依赖并装配各层、注入适配器。运行调用另画为核心→抽象端口→注入的 SDK 适配→设备，反馈回到核心；线程与有界队列另标所有者。具体库、线程预算和接口版本未给定。

## 5. 连续 Twist 命令

主技能：`robot-dataflow-module-design`。补充：`ros2-control-hardware-safety` 核对命令所有权；本题不触发栈操作。

可以保留连续速度的有界 Topic 接口，通信形式不决定执行权限。只允许既有仲裁与控制保护链中获授权的唯一 writer 发布对应命令段；任务入口、规划器、UI 或诊断不能另开写入链。Twist 无租约字段时由受控入口及执行侧绑定授权、epoch、时效，拒收旧命令，并定义断流 watchdog 与安全动作。题设未给实际接线，故不能指定某个当前节点就是合法发布者，也未证明现链路已满足这些契约。

## 6. 拒绝与正常成功路径

主技能：`robot-task-intake-execution`，使用状态转换参考。补充：需要执行验收时再用场景和证据技能。

准入失败：RECEIVED→NORMALIZING→VALIDATING→FINALIZING(REJECTED)→拒绝结果持久化，不创建 child 或运动权限，也不虚构取消、停稳。正常成功：前三步→QUEUED→ADMITTED→RESERVED→PREPARING→EXECUTING→VERIFYING→FINALIZING(SUCCEEDED)→结果持久化；无需经过暂停或取消。实测确认与资源交接单独记录，满足旧执行终止/隔离、安全确认及持久化屏障后才重新授权。实际接口状态名称待映射。

## 7. 取消、成功、ACK、未停与存储失败

主技能：`robot-task-intake-execution`，使用事件偏序参考。补充：`ros2-control-hardware-safety` 约束停止保护。

若取消已由权威状态机受理，先冻结新步骤；迟到 SUCCEEDED 仅记为下游事实，ACK 也不代表停稳。速度非零使安全确认不成立，存储失败使结果提交不成立：进入 RECOVERY_REQUIRED，隔离相关资源及真实依赖，最终保护继续停止/保载，不发布已持久化成功或取消。交接须满足“终态/有效隔离＋安全确认＋持久化→交接→新授权”。允许无冲突只读任务。若取消只是发送而未受理，最终裁决仍需核对；不可按回调到达顺序定输赢。

## 8. 仅清理 A 的文字方案

主技能：`ros2-stack-ops`。补充：仅当涉及真实硬件时用设备安全技能；本题不执行动作。

先取得 A 的会话目录、有效启动句柄/所有者、boot 身份、PID/启动时刻、命令令牌、隔离环境、安装与日志，并交叉核对。停止 A 的接收与活动任务，保留反馈，随后由其已核实 supervisor 退出并检查自有残留。执行点必须重验身份且防 PID 复用；复用 PID 排除。B 无 launch 父进程不构成 A 归属，脱离父进程者也需先前捕获证据。未知对象保留，不全局杀进程或删 DDS。反馈缺失仅能报停车未知；缺少有效身份时不能按陈旧 PID 清理。

## 9. 错误查询环境下的超时

主技能：`ros2-stack-ops`。无默认补充。

不能断言 A 的物理不步进。查询 shell domain=25，既不属于 A 的 61，也不属于 B 的 62；Gazebo transport 又需独立对齐 sim_a partition、发现端口/IP 和实际 world。正确环境下观察一段时间的 stats 时间、iterations 与 paused，另查连续 /clock，再区分暂停、QoS、错话题、窗口不足及生产端故障。四个话题和一次超时只能说明当前观察不足。现有证据应写“尚不能判断 A 是否推进”。

## 10. 可配置 domain 的隔离查询

主技能：`ros2-stack-ops`，使用当前入口参考。补充：进入 launch 修改才需语言技能。

题设源码已证明 25 是默认值，supervisor 传入 61 应经 LaunchConfiguration 生效，不能判断被硬编码覆盖。隔离启动显式指定非空 instance、domain=61 和独立日志，检查身份未冲突，并核对生成的 partition、发现端口及独立安装/overlay。查询从可信目标记录和已确认进程取实际环境，ROS 用 61，Gazebo 用该实例真实 partition/端口；核对 env.sh 未完整恢复 overlay 的限制。参数解析尚不证明运行采用版本或实际就绪。

## 11. 旧手册与 C++ 源码

主技能：`robot-argument-audit`。补充：`astribot-architecture-design` 源码索引用于定位；实际模块接线审查才补数据流技能。

不能判断任务仲裁未实现。“旧 Python 路径不存在→能力不存在”的推断被题设 C++ 源码入口反驳；19 包是旧文档范围，不能作为当前包数。可确认题设支持存在 C++ 候选实现，尚不能仅凭文件名证明其行为、构建、launch 接线、当前安装采用及运行效果。应更新定位依据并按需核对这些层。当前导航仲裁边界也不能外推成整机双臂资源总线；本次只读题设未完成任何运行验证。

## 12. 姿态输出与预期拒绝的分母

主技能：`robotics-validation-evidence`。补充：`robot-argument-audit` 核对“7/7 抓取成功”的推断；若要新增实验才补场景技能。

不能报告 7/7 抓取成功。记录仅支持“2 个场景有姿态输出、5 个场景按预期拒绝”；前者的精度和完整验收条件未知，后者证明对应拒绝防护。两类分别列样本与预期/观测，不能相加充当抓取完成率。没有实际抓持尝试，抓取成功率应标未测试/无有效分母，不写 0/7。要支持抓取成功还缺执行准入、IK/碰撞、夹爪反馈、物体保持及账本证据。

## 所读冻结资料

- `astribot-architecture-design/SKILL.md`
- `astribot-architecture-design/references/project-map.md`
- `astribot-architecture-design/references/skill-use-cases.md`
- `robot-task-intake-execution/SKILL.md`
- `robot-task-intake-execution/references/task-transitions.md`
- `robotics-validation-evidence/SKILL.md`
- `robot-runtime-cpp/SKILL.md`
- `robot-dataflow-module-design/SKILL.md`
- `robot-dataflow-module-design/assets/interface-contract.md`
- `robot-dataflow-module-design/assets/module-card.md`
- `ros2-control-hardware-safety/SKILL.md`
- `ros2-stack-ops/SKILL.md`
- `ros2-stack-ops/references/current-entrypoints.md`
- `robot-argument-audit/SKILL.md`
- `robot-design-review/assets/review-record.md`
