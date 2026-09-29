# 冻结技能应用试评答案

依据仅为本目录下 `.agents/skills/*/SKILL.md` 与题目给定事实。未读取文档审计报告、源码或其他评审答案，未联网、未执行实验或 ROS/进程控制。下文设计不是已实现声明。

## 1. 任务接收去重接口

主技能：`robot-task-intake-execution`；架构路由表要求同时加载 `robot-task-orchestration`，本题其抓放事务内容不适用；接口契约补充 `robot-dataflow-module-design`。交付任务信封、去重键作用域与保留期、重复请求/异参冲突/终态重放规则、唯一接收与存储所有者、并发和存储失败流程及离线验收表。建议相同键同请求返回原任务与结果，异参返回稳定冲突码；创建任务与写入去重记录须原子化。实际存储和现有接口缺信息，保留为待确认。

## 2. 已有导航 CSV 整理

主技能：`robotics-validation-evidence`；其前置技能为 `robot-runtime-cpp`、`ros2-stack-ops`，本次不执行栈操作。先核对 CSV 来源、版本、时间域、frame、位姿来源、采样率和缺失值，再按成功/取消/失败及 FOLLOW/终端阶段分别统计，明确公式、过滤条件、样本量与证据绝对路径。仅整理已有记录无需重新设计全部测试或启动系统；文件未提供，当前不能给数值结论，更不能把历史记录当当前运行验证。

## 3. 语言选择

主技能：`robot-runtime-cpp`；涉及模块边界时补充 `robot-dataflow-module-design`。区域编辑器若为 RViz 插件，其交互、几何和提交逻辑优先 C++；REST 任务网关的持续运行服务及准入/去重逻辑优先 C++；持久化服务与事务、恢复逻辑优先 C++。启动文件、离线验证和数据分析脚本优先 Python。若编辑器是浏览器页面，展示部分可用适合前端的语言，但运行时领域逻辑仍放 C++；现有薄 Python 绑定可渐进迁移，不因此重写已验证链路。

## 4. 编译依赖与运行调用

主技能：`robot-dataflow-module-design`；补充 `mobile-dual-arm-architecture`、`robot-runtime-cpp`，涉及世界事实再补 `perception-world-model`。明确 `A → B` 表示 A 编译依赖 B：核心 → 纯领域接口；ROS 适配 → 核心、领域接口、ROS；SDK 适配 → 领域接口、厂家 SDK；组合入口 → 核心和具体适配。核心不得依赖 ROS、SDK 或启动脚本。运行时则可为 ROS 入口 → 核心 → 注入的领域接口 → SDK 实现 → 设备，反馈反向回传；调用方向不能冒充编译依赖方向。

## 5. 连续 Twist 命令 Topic

主技能：`robot-dataflow-module-design`；命令所有权补充 `ros2-control-hardware-safety`。可保留既有连续速度 Topic；题目已明确它承载命令，应在接口表标为命令流，不能因采用 Topic 就当作纯观测。最终命令边界只允许当前获授权的控制器或仲裁/保护链唯一出口发布。任务网关、规划器、日志工具不能旁路发布；人工控制也需经过模式和控制权交接。契约应补频率、队列上限、时效、丢帧及失联停车；Twist 无源时间戳时须明确接收时效局限。

## 6. 准入失败和正常成功路径

主技能：`robot-task-intake-execution`；报告补充 `robotics-validation-evidence`。准入校验失败：`RECEIVED → NORMALIZING → VALIDATING → REJECTED`，记录稳定拒绝码，不创建子动作、不取得运动权限。正常成功：`RECEIVED → NORMALIZING → VALIDATING → QUEUED → ADMITTED → RESERVED → PREPARING → EXECUTING → 必要实测确认 → 终态持久化 → SUCCEEDED`，最后释放租约。暂停、取消、STOPPING 等是条件分支，正常请求无需走遍；QUEUED 是否允许无等待跳过须在契约明确。

## 7. 取消竞争、未停稳和存储失败

主技能：`robot-task-intake-execution`；补充 `ros2-control-hardware-safety`、`robot-scenario-boundary-validation`。冻结后续步骤，关联 execution/step 身份记录迟到 SUCCEEDED 和取消 ACK；两者均不证明停稳。速度仍非零就继续受控停止和最终保护，不能发布整机成功或完成取消。存储失败又使终态无法可靠提交，应进入 `RECOVERY_REQUIRED`、对外说明结果未决并报警。资源继续受保护占用或显式转交安全所有者，禁止因租约到期自动给新任务；停稳及持久化恢复后再核对、提交和释放。

## 8. 仅清理仿真 A

主技能：`ros2-stack-ops`；隔离边界补充 `robot-scenario-boundary-validation`。目标证据应包含 A 的会话记录、日志、domain/partition、PID 与启动时间、可执行文件和环境，并交叉核对进程树或所属控制组。B 没有 launch 父进程不能视为不存在；被复用 PID 必须排除。每次发信号前重新校验身份，只对已证明归属 A 的进程先温和停止、等待，必要时定点升级。禁止全局模式清理、全局共享内存删除和影响 B 的 daemon 操作；归属无法证明则保留目标。本题仅给流程，不执行。

## 9. 查询环境与物理步进

主技能：`ros2-stack-ops`；证据判定补充 `robotics-validation-evidence`。不能断言 A 不步进。查询 shell 的 domain=25 与 A 不同，ROS 发现结果无效；Gazebo 查询还必须对齐 A 的 partition=sim_a 和网络环境，不能用另一 transport 空间的四个话题或超时代表 A。先证明目标会话身份与查询环境，再在 A 的实际 stats 话题连续观察 iterations/sim_time 是否增长，并关联 server 日志；随后才查 domain=61 的 /clock、TF、生命周期和数据率。

## 10. 参数化 domain 的隔离与查询

主技能：`ros2-stack-ops`；补充 `robot-scenario-boundary-validation`。按题目源码事实，25 是默认值而非不可覆盖常量；supervisor 显式传入 61 后，目标子进程应使用 61，仍需从归属明确的运行进程反读核实参数是否传播。另设 A 专用 Gazebo partition、日志、端口、锁和必要的 build/install 路径。查询 ROS 使用实际 domain 及 RMW/网络配置，查询 Gazebo 使用实际 partition/网络配置；每个查询 shell 分别对齐，不能只依赖父 shell 的 export。

## 11. 文档与 C++ 仲裁源码

主技能：`robot-argument-audit`；实现核对补充 `astribot-architecture-design`、`robotics-validation-evidence`。不能据“19 包”或失效 Python 链接判断仲裁未实现。给定 C++ 文件存在，至少说明文档入口可能过时；也不能反向仅凭文件存在宣布能力完整。应进一步核对 CMake 构建目标、安装产物、launch 入口、接口服务端与消费者、实际仲裁和取消实现及测试记录。本题未提供这些证据，结论保留为“旧链接不能支持未实现主张，C++ 实现范围与接线待核验”。

## 12. 两个姿态输出与五个预期拒绝

主技能：`robotics-validation-evidence`；主张核查补充 `robot-argument-audit`。不能报告 7/7 抓取成功。可报告“2 个场景产生姿态输出，5 个场景按预期拒绝”；只有输出质量和拒绝判据均有证据时，才可另称七个场景响应均符合各自预期。姿态输出不是抓取执行，预期拒绝只证明对应防护。实际抓持未运行，抓取成功率应记未测、无有效分母；仍缺执行、夹爪反馈、物体保持及状态提交证据，不能升格为仿真或真机抓取验收。
