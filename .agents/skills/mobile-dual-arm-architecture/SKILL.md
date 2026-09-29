---
name: mobile-dual-arm-architecture
description: 在本仓库设计或审查轮式双臂机器人软件架构、模块边界、模块交互、接口契约或包拆分时使用。把导航、双臂操作、感知世界、任务资源、执行保护和设备适配放进同一套可追溯架构视图，但不把尚未实现的设计写成现有能力。
---

# 轮式双臂整机架构

先按 [源码入口索引](../astribot-architecture-design/references/project-map.md) 核对受影响领域。语言选择统一遵循 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)。
只有追溯历史拆分决策时才读 [整机审查](../../../docs/WHOLE_ROBOT_ARCHITECTURE_REVIEW_20260917.md) 或 [SLAM 分包决策](../../../docs/SLAM_ARCHITECTURE_REVIEW_20260918.md)；行业方案对照时才加载 `autonomy-stack-architecture`。

## 先定控制权，再画模块

把每条命令链标成唯一所有者：

```text
整机任务/资源协调
  -> 导航技能 -> TaskArbiter -> Nav2/策略/跟踪 -> 速度平滑与坐标适配 -> 设备适配
  -> 操作技能 -> MoveIt/MTC/公共轨迹校验 -> 臂/夹爪执行适配
感知适配 -> 带时间、坐标、标定和质量的 WorldSnapshot -> 导航与 MoveIt
运输状态/RobotEnvelope -> 导航准入、BT、costmap、规划和控制约束
```

- 当前 `TaskArbiter` 只拥有导航 Action；不要把它描述成底盘、双臂、躯干和夹爪的整机仲裁器。
- 按用户 2026-09-24 明确要求，上肢/载荷的停车、限速与不可执行判断均前置于导航规划策略；不设独立整机末级速度门禁。平滑器及底盘输出适配不消费账本、几何版本或包络 ACK，不重复导航准入。底盘设备急停、命令失联和驱动限幅保留各自职责。
- Nav2、MoveIt、轨迹桥和 SDK 各自保留实现边界；任务层编排它们，不在上层复制速度控制器或直接发布设备命令。
- 每个 service/action 只能有一个生产所有者。若已有入口能复用，优先增加适配器、策略或插件，不新增平行入口。
- 任务、路径、地图、定位、包络、时钟、场景和标定都使用显式版本或 epoch。版本不一致必须让计划失效或重新校验，不能默认“最新消息就是可用”。

## 按影响维护架构视图

整机设计使用下列视图检查覆盖；局部变更只更新受影响视图，并引用未变的基线。接口细节交给 `robot-dataflow-module-design`，不重复生成同一套材料。

1. 功能视图：任务、导航、感知、双臂、运输、保护和诊断的职责。
2. 交互视图：topic/service/action、调用方向、QoS、超时、取消、结果和唯一所有者。
3. 控制权视图：谁可以申请、提交、执行、取消和确认运动；规划结果不能直接授予设备控制权。
4. 数据与坐标视图：frame、采集时间、接收时间、时钟域、标定版本、对象/载荷状态。
5. 部署视图：仿真、真机、bringup、控制器、SDK 和 ROS 节点的启动与生命周期。
6. 故障恢复视图：旧任务终态屏障、资源租约、过期计划、重定位、传感器失效、一臂失败和保载/停车策略。
7. 证据视图：静态检查、单元/离线回放、隔离仿真、整栈仿真和真机验收分别能证明什么。

## 包和接口的本项目约束

- `astribot_navigation_msgs`、`astribot_bridge_msgs`、`astribot_slam_msgs` 按领域保留，不创建万能消息包。
- 导航二维 costmap 与 MoveIt 三维 PlanningScene 共享对象身份、时间、坐标和版本，但保留各自空间表示、更新频率和校验器。
- 实现语言由 `robot-runtime-cpp` 统一约定，不因“轻量适配”放宽新功能的语言选择。
- 新接口优先用强类型消息/Action/服务承载机器状态；字符串只作诊断和日志说明。
- `planning_demo_node.cpp` 这类演示流程不能直接升级为生产整机任务层。拆出任务编排、场景适配、执行适配后再复用现有规划能力。

## 架构评审输出

交付中分开写“已实现、已验证、待验收、建议设计”。对每条建议注明：

- 现有代码入口和真正的消费者；
- 新增还是复用的接口；
- 谁获得/不获得控制权；
- 取消、失败、过期和恢复语义；
- 需要的静态、离线、仿真或硬件证据。

不要用包数量、进程数量或“有一个节点/服务”推断整机能力已闭环。

## 外部参考

- ROS 2 接口选型：[topics / services / actions](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html)
- ROS 2 QoS 兼容性：[Quality of Service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- Nav2 模块化行为树与任务服务器：[Nav2 architecture](https://docs.nav2.org/)
- 可借鉴的双臂移动平台分层：[TIAGo++ robot](https://github.com/pal-robotics/tiago_dual_robot)
