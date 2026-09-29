---
name: autonomy-stack-architecture
description: 在本仓库需要对照 Autoware/Apollo 等自动驾驶架构，评估自主系统分层和设备接口模式能否借用于轮式双臂机器人时使用。普通项目模块设计直接使用对应领域技能。
---

# 自主系统分层与车辆/设备接口

本技能是按需行业对照，不是整机设计的必经入口。仅对受影响领域查 [源码入口索引](../astribot-architecture-design/references/project-map.md)。具体世界快照、设备执行分别由 `perception-world-model`、`ros2-control-hardware-safety` 负责，只有落实到这些领域时再读取。
涉及实现语言时使用 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)。

## 采用分层，但保留机器人特有资源

```text
Sensing / Perception / Localization / Mapping
  -> WorldSnapshot + health + version
Planning / Task Decision / Navigation or Manipulation Skills
  -> trajectory, constraint, action goal
Control / Tracking / Execution Guard
  -> bounded motion command
Vehicle or Device Interface
  -> chassis / arm / gripper / SDK protocol
System Monitoring / Mode / Human Override
  -> capability, fault, safe stop, recovery permission
```

- 传感器、定位和地图输出可被替换，但必须遵守 frame、时间、质量、版本和 QoS 契约。
- 规划输出轨迹或意图；控制器/跟踪器负责把它变成受限命令；设备接口负责协议转换、反馈和控制模式。
- 车辆/设备专用协议放在适配层。不要让导航、任务或感知模块直接依赖厂家 SDK 类型。
- 系统监控和人工接管是独立关注点；它们可以撤销自治权限，但不能删除底层最终保护。
- 对轮式双臂机器人，`vehicle interface` 拆成底盘、双臂、躯干、夹爪等执行接口，由整机任务层管理资源和同步语义。

## 从 Autoware/Apollo 借鉴的内容

- Autoware 的 sensing、map、localization、perception、planning、control、vehicle interface 分层可作为功能视图；其 vehicle-specific interface 负责命令/状态转换和能力诊断。
- Apollo 的消息驱动模块依赖、Planning/Control/CanBus 分离和 Guardian/安全命令分离，可作为控制权审查的参考。
- 两者都是自动驾驶平台；当前项目是 ROS2 Humble 轮式双臂机器人。只吸收职责和接口原则，不复制 Cyber RT、车辆 CAN 语义、HD map 假设或道路场景状态机。

## 领域映射（具体实现以源码索引为准）

| 自主系统层 | 当前项目对应 | 约束 |
|---|---|---|
| Sensing/Perception | `astribot_s1_perception`、`perception_components` | 只发布带来源和质量的观测 |
| Localization/Mapping | `astribot_s1_slam`、`astribot_s1_mapping` | 保持 SLAM、栅格和消息领域分包 |
| Planning/Decision | Nav2、`astribot_s1_navigation_policy`、任务/操作技能 | 不直接写底盘或关节命令 |
| Control/Tracking | `astribot_s1_path_tracking`、dynamics coupling、MoveIt 执行校验 | 保留限速、制动、包络和到位门槛 |
| Device Interface | `astribot_trajectory_bridge`、仿真控制器、厂家 SDK | 单一会话/写入者，反馈和失联语义明确 |
| Monitoring/Override | `TaskArbiter`、health、FinalProtection、operator station | 导航仲裁不宣称拥有整机资源 |

## 内部实现规则

- 实现语言引用 `robot-runtime-cpp`；领域核心与 ROS 适配、调度和消息转换分开。
- 每个层的输入输出定义截止时间、版本、故障码和降级行为；不以空消息、默认参数或“节点存在”表示能力可用。
- 需要长时间运行、反馈和取消的动作使用 Action；短的能力查询/提交使用 Service；高频状态使用有界 Topic。
- 任何控制模式切换、人工接管、失联停车、重定位或包络变化都必须形成可记录的状态转换。

## 外部参考

- Autoware 分层架构：[Architecture overview](https://docs.autoware.org/pr-803/design/autoware-architecture-v1/)
- Autoware 车辆/设备接口：[Vehicle Interface design](https://autowarefoundation.github.io/autoware-documentation/latest/design/autoware-architecture/vehicle/)
- Apollo 模块架构：[Apollo software architecture](https://github.com/ApolloAuto/apollo/blob/master/docs/14_Others/Apollo_5.5_Software_Architecture.md)
