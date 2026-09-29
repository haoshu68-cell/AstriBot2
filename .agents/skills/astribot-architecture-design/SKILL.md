---
name: astribot-architecture-design
description: 在本仓库需要确定整机架构工作范围、选择领域设计技能，或规划跨模块接口演进和重构时使用。已明确领域的局部工作可直接使用对应技能。
---

# AstriBot 架构设计入口

按本次问题选择一个主技能，再按实际影响补充；表中的补充不是默认必读清单。
涉及实现或语言选择，统一使用 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)：功能实现优先 C++，只有脚本和启动文件类优先 Python。

| 问题 | 主技能 | 何时补充 |
|---|---|---|
| 整机职责、控制权和包边界 | [mobile-dual-arm-architecture](../mobile-dual-arm-architecture/SKILL.md) | 比较自动驾驶行业设计时才读 `autonomy-stack-architecture` |
| 数据流、接口、QoS、依赖和模块实现 | [robot-dataflow-module-design](../robot-dataflow-module-design/SKILL.md) | 影响世界快照时读 `perception-world-model`；改变整机控制权时读整机架构 |
| 任务接收、去重、准入、排队、取消和恢复 | [robot-task-intake-execution](../robot-task-intake-execution/SKILL.md) | 涉及抓放/运输阶段事务时才读 `robot-task-orchestration` |
| 抓取、放置、运输和 Nav2/MoveIt/MTC 编排 | [robot-task-orchestration](../robot-task-orchestration/SKILL.md) | 改变外部接收、调度或持久化契约时读任务入口技能 |
| 感知、SLAM、地图、标定和场景一致性 | [perception-world-model](../perception-world-model/SKILL.md) | 改变执行准入或资源边界时才读相关任务/控制技能 |
| 控制器、SDK、设备桥和执行保护 | [ros2-control-hardware-safety](../ros2-control-hardware-safety/SKILL.md) | 实际启动/探查/停止栈时读 `ros2-stack-ops` |
| 区域任务、脱困、报警与恢复预算 | [robot-mission-boundary-safety](../robot-mission-boundary-safety/SKILL.md) | 按涉及的载荷事务、感知或设备保护补充对应技能 |
| 方案可行性、选型和风险评审 | [robot-design-review](../robot-design-review/SKILL.md) | 论证链有争议时读 `robot-argument-audit`；按领域补充约束 |
| 前提、证据与结论是否相符 | [robot-argument-audit](../robot-argument-audit/SKILL.md) | 需要设计区分实验时读场景技能 |
| 多场景、边界、故障注入和恢复验证设计 | [robot-scenario-boundary-validation](../robot-scenario-boundary-validation/SKILL.md) | 记录实际结果或交付证据时读证据技能 |
| 整理已有日志、指标和验收结论 | [robotics-validation-evidence](../robotics-validation-evidence/SKILL.md) | 仅需重新设计测试时才读场景技能；离线整理不触发栈操作 |
| 启动、查询、停止 ROS2/Gazebo 栈 | [ros2-stack-ops](../ros2-stack-ops/SKILL.md) | 真实硬件运动另需设备保护及硬件准入 |

## 基线与交付粒度

1. 从 [项目源码入口索引](references/project-map.md) 定位本次涉及的源码、launch、接口和消费者，核对工作区版本及实际安装产物。索引与旧设计文档都是定位材料，不是运行能力证明。
2. 局部设计提交受影响接口/模块的差异、理由、失败处理和验证入口；已有架构视图直接引用，不要求重画七张图。跨模块或整机边界改变时，按 `mobile-dual-arm-architecture` 维护相应视图。
3. 写清现状、选择、替代方案、兼容/回退条件、实现与证据范围。按影响检查 launch 依赖、消息/插件兼容、控制写入者、时间/版本、取消与资源交接；不为目录整洁重写已验证控制器。
4. 用户要求论证或验收设计时，关联需求 R、主张 C、发现 F/危险源 H、场景 SC 与证据 E。只使用能改变决策的字段；局部修改不自动变成全库审计或多角色流程。

架构与技能工作完成不等于机器人功能验收。历史检索/取舍可按需查 [开源来源记录](../../../docs/OPEN_SOURCE_ROBOT_SKILLS_REVIEW_20260920.md) 和 [评审技能接入记录](../../../docs/ROBOT_REVIEW_SKILLS_INTEGRATION_20260922.md)，不默认加载全部来源。
修改本组技能时，使用 [固定使用题集](references/skill-use-cases.md) 检查受影响行为；格式通过不能替代使用验证。
