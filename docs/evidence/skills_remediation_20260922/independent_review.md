# 项目技能整改独立只读复核

日期：2026-09-22。复核者为本任务独立子代理，读取当前工作区及 `docs/SKILLS_AUDIT_20260922.md`；未使用其他评估者的答题结果。本报告只核对规则、模板、引用及相关源码事实，不表示独立机器人实验。

## 结论

**无阻断发现。** 在本次范围内，没有找到需要继续修改的实质语义冲突、语言例外扩大、无条件整机产物要求或对现有停止器安全性的错误保证。发现数：0；不为凑数列轻微措辞意见。

## 核查结果

| 关注点 | 观察结果及准确锚点 |
|---|---|
| 语言统一 | `AGENTS.md:3-5` 与 `.agents/skills/robot-runtime-cpp/SKILL.md:8-19` 均按职责要求功能实现优先 C++、脚本与启动文件优先 Python；常驻网关、UI、REST/MQTT、持久化均归功能实现。现有绑定只作为兼容迁移保留，不构成新功能通用例外。其他领域技能引用此入口，未见冲突许可。 |
| 接口与依赖方向 | `.agents/skills/robot-dataflow-module-design/SKILL.md:50-64` 已容纳受控连续命令 Topic，并明确通信形式不授予执行权；`:158-192` 分开源码依赖、运行调用和消息流。`assets/module-card.md:13-15`、`assets/interface-contract.md:22-32` 与正文一致。 |
| 状态与提交偏序 | `.agents/skills/robot-task-intake-execution/SKILL.md:71-99` 和 `references/task-transitions.md:26-42` 分开拒绝、正常成功、取消与暂停路径；`references/task-transitions.md:49-69` 明确 `(T 或具备实现证据的 F) + S + R -> H -> N`，结果可以先持久化为 release_pending，不形成先释放/先结果的相互等待。 |
| 竞态与资源范围 | `task-transitions.md:76-88` 明确取消与完成的单一裁决点、旧 epoch 和幂等落盘；`:71-72`、`.agents/skills/robot-task-orchestration/SKILL.md:35-42`、`.agents/skills/robot-scenario-boundary-validation/SKILL.md:43` 均把隔离约束限定到冲突资源及真实依赖，允许无冲突只读任务继续。 |
| 按需加载与交付 | `.agents/skills/astribot-architecture-design/SKILL.md:8-31` 明确一个主技能及条件补充，局部变更只交受影响差异；整机、区域、数据流、任务入口和证据技能没有再要求局部工作无条件加载整套架构或重跑全链。场景卡的 readiness/execution 分离与正文 `robot-scenario-boundary-validation/SKILL.md:30-38` 一致。 |
| 诊断与停止权限 | `.agents/skills/ros2-stack-ops/SKILL.md:14-31` 禁止通过进程名、domain 或陈旧 PID 猜测停止归属，并禁用现有全局清理工具作为共享环境入口；`:48-58` 将观测不足与暂停/持续推进分开。`references/current-entrypoints.md:12-23` 已披露仿真清单身份不完整、采样子树局限，以及真机工具固定 ROS 端点仍依赖调用环境。正文 `:66` 明确静态修改不证明停止工具实现已修复。 |
| 当前源码入口 | `.agents/skills/astribot-architecture-design/references/project-map.md:3-16` 标明日期、源码范围和安装/运行限制；`docs/manuals/ARCHITECTURE_REFERENCE.md:5` 保留历史主体边界，`:36` 不再使用固定总包数，`:72` 指向 C++ 仲裁入口，`:218` 指向现有 `nav_prob_grid_node.cpp` 并说明旧独立节点已移除。 |

## 实际验证及边界

- 读取了全部 15 个 `SKILL.md`、技能内其他 11 份 Markdown、根 `AGENTS.md` 和架构参考手册，共 28 份范围内 Markdown。
- 独立扫描上述文件的 Markdown 本地链接：155 处引用，缺失目标 0。此项仅验证链接目标存在，不验证所有标题锚点、外部 URL 或递归引用文档的全部事实。
- 静态读取了 `tools/launch_sim_stack.sh`、`tools/sim_isolation.py`、`tools/sim_stack_supervisor.py`、`tools/robot/robot_task_control.py` 的相关实现，以及 `warehouse_sim.launch.py` 的 domain 声明/环境设置。源码支持参考材料关于隔离参数、清单字段、overlay 未完整保存和固定停车端点的限制说明。
- **现有停止器运行安全性仍未验证。** supervisor 仍有 `killpg` 路径和采样后代集合（`tools/sim_stack_supervisor.py:293-294,435-452`）；本复核不证明所有退出竞态、逃逸进程或 PID/进程组复用均已覆盖。文档要求不能代替实现及隔离夹具证据。本轮没有修改或执行这些工具。
- 没有运行固定题集的另一轮盲测，没有重新执行格式校验器，没有测量技能改善率；本报告支持“当前文本未见实质冲突”，不支持“技能无遗漏或稳定性已全面验收”。
- 没有启动、停止或在线探查 ROS/Gazebo，没有进程管理动作、硬件操作、构建或机器人测试。未重新核验全部外部仓库、标准、许可证或安装产物。

## 复核版本

范围内文件按 `AGENTS.md`、排序后的技能 Markdown、架构参考手册顺序，将每个相对路径、NUL、SHA-256、换行串联后的 SHA-256 为：

`5da01cd691fb66ba15ab5005d49a95787b99e1cda9c01f07e48ff08c2b5ef5a5`

关键文件 SHA-256：

| 文件 | SHA-256 |
|---|---|
| AGENTS.md | b55e2509660bbaf3cf1080e484a7d11841de061278194bbed4bf7991675bc5ff |
| .agents/skills/robot-runtime-cpp/SKILL.md | 8955250b01d14821efe7647fbdf9d8a5d5864c17ec245044d192c0f6ed2aed7e |
| .agents/skills/robot-task-intake-execution/references/task-transitions.md | 38213fa01da833360002d79c5fb09fa2badcb78f8e89d5fa66279b9bb879e2bf |
| .agents/skills/ros2-stack-ops/SKILL.md | 4341d7184c1cb62e7b1403da887d4d34853e5cb86a920862dd5eb7e034d58fa1 |
| docs/manuals/ARCHITECTURE_REFERENCE.md | ccf6067cfaeee085ea4f6111ab8251b5763edf7af2a01e7dfedd9fb4510d7bdc |

若这些文件随后变化，本结论只对应上述复核版本；应对新增差异继续复核。
