---
name: robotics-validation-evidence
description: 在本仓库为机器人架构、导航、双臂搬运、感知、控制或启动流程做验证、回归和交付记录时使用。严格区分实现状态、静态/离线证据、仿真证据和真机验收，统一关联 ID、阶段、来源和指标口径。
---

# 机器人验证与证据分层

按证据类型取用：导航指标查 [指标口径](../../../docs/PATH_TRACKING_METRICS.md)，会话日志查 [日志说明](../../../docs/LOGGING.md)，实现入口查 [源码索引](../astribot-architecture-design/references/project-map.md)。仅整理已有文件不加载栈操作流程，也不重新构建或运行系统。编写功能/验证脚本时才按 [语言约定](../robot-runtime-cpp/SKILL.md) 选择实现方式。

## 四层结论不可混写

对每项改动分别记录：

1. **实现**：源码、接口、配置和启动入口确实存在；
2. **静态/离线**：编译、类型、依赖、单元、回放、公式或契约检查通过；
3. **隔离/整栈仿真**：真实 ROS2/Gazebo/控制器/传感器链在固定场景运行通过；
4. **真机验收**：真实 SDK、传感器、标定、地面/外部真值、停止距离和安全流程通过。

一层不能自动升级为后一层。孤立探针通过不等于 live SLAM、双臂执行或整机搬运通过；仿真真值也不等于外部物理精度。

## 每条证据的最小字段

```text
robot_id / session_id / task_id / step_id / action_id
source_node / source_topic_or_action / pose_source / frame_id
sample_stamp / receive_stamp / clock_epoch / map_or_scene_revision
calibration_revision / envelope_epoch / controller_or_sdk_state
phase / scenario / expected / observed / reason_code
artifact_path / source_commit_or_hash / environment / operator
```

运行日志统一使用仓库 spdlog/ROS 日志链，并报告实际绝对路径（包括 `session.log` 和 `latest_sim` 指向）；地图、轨迹、JSON/CSV 和日志分别说明，不把其中一种冒充另一种。

## 运动指标口径

- 成功到位与取消/失败目标分开统计；不要用所有 action 结果混合计算误差。
- `FOLLOW`、终端 `REFINE`、`ARRIVAL_COAST` 等阶段分开统计；阶段过滤条件、采样频率、位姿来源和 frame 必须写明。
- 到位误差、横向误差、航向误差、停止漂移和跟踪样本使用固定公式和版本；完成时长不能偷偷替代控制质量。
- 传感器断流、旧帧、时钟回退、取消未终止、包络版本改变和一臂失败要有独立失败原因，不能只记“任务失败”。

## 按主张选择验证层

需要制定覆盖方案时先用 [多场景与边界验证](../robot-scenario-boundary-validation/SKILL.md)，把需求/主张/风险关联到场景和证据；仅整理已有运行记录时不必重新规划所有测试。
每个场景分别记录功能结果、安全不变量、性能预算和证据层。预期拒绝只证明该防护；未运行、阻塞、无效实验与被测失败分开，不把重复包装的测试计数相加。

1. 只整理既有证据：核对来源、版本、原始数据和指标；缺项明确保留，不扩大成重跑任务。
2. 验证实现：按影响做依赖/接口/构建、固定输入、边界或回放检查。
3. 验证 ROS/仿真集成：选择受影响的启动、QoS、TF、控制器、Nav2、MoveIt/MTC、包络或取消链，在有所有权的隔离环境执行。
4. 真实运动：完成该运动所需的离线/仿真验证及硬件准入后，按硬件手册做小范围验收。只读设备健康检查、物理标定采集等按实际风险和目的设置前置条件，不以无关全栈仿真完整性阻塞；涉及运动的采集仍需运动准入。
5. 对未覆盖项标 `not tested` / `blocked` / `requires hardware`；不填造样本、成功率或“无紧急项”。无关层可注明 N/A 及原因。

涉及 ROS2/Gazebo 进程、话题、发布者、启动或清理时，先遵守 [ros2-stack-ops](../ros2-stack-ops/SKILL.md)，并确认共享栈归属；不能以进程数、发布者数或节点存在宣称系统已就绪。

## 交付格式

报告至少包含：改动范围、已实现、已验证、待验收、复现入口、证据文件绝对路径、指标定义、失败样本和未覆盖风险。不要将设计建议、历史记录和当前运行结果混为同一状态。

## 外部参考

- 生产级机器人 agent skills 对生命周期、QoS、边界缓冲、看门狗和测试的归纳：[robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills)
- ROS 2 接口和动作的结果/取消语义：[Interfaces](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html)
- ROS 2 节点图和运行时连接探查：[Understanding ROS 2 nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
