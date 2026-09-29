---
name: robot-task-orchestration
description: 在本仓库新增或修改搬运、抓取、放置、运输姿态、导航与双臂任务编排时使用。把任务资源、MoveIt/MTC 规划、Nav2 执行、包络握手、物体账本、取消和实测确认组织成有界事务。
---

# 整机任务与双臂运输事务

先根据 [项目源码索引](../astribot-architecture-design/references/project-map.md) 核对受影响入口。
按需读取：

- 改变运输阶段、MTC 规划或已有服务时，读对应的 [`astribot_s1_transport` README](../../../ws_robot/src/astribot_s1_transport/README.md)、[`astribot_s1_transport_mtc` README](../../../ws_robot/src/astribot_s1_transport_mtc/README.md) 及源码。
- 追溯设计理由时查 [MTC 运输架构方案](../../../docs/MTC_TRANSPORT_ARCHITECTURE_PROPOSAL_20260919.md)，历史方案不替代当前实现。
- 改变整机取消、结果提交或资源交接时读 [任务事件与转换](../robot-task-intake-execution/references/task-transitions.md)。
- 进入实现时遵循 [项目语言约定](../robot-runtime-cpp/SKILL.md)。

## 采用分阶段事务

生产任务按以下边界实现，阶段名称可按现有接口映射：

```text
ADMISSION
  -> RESOURCE_LEASE / 操作 HOLD
  -> SNAPSHOT（物体、关节、底盘、地图、定位、标定、场景、时钟）
  -> PLAN（Pick / Place / transport）
  -> VALIDATE（碰撞、限位、奇异点、时间参数化、包络、后续退出）
  -> EXECUTE（逐段执行和取消屏障）
  -> OBSERVE_CONFIRM（关爪/附着/停稳/释放/退臂的实测证据）
  -> COMMIT_STATE（账本和版本）
  -> NAVIGATE（只交给既有导航仲裁）
  -> PLACE / RELEASE / RECOVER / TERMINAL
```

- 资源至少区分底盘、左臂、右臂、躯干、夹爪、PlanningScene、RobotEnvelope 和导航任务。
- 任务层申请和归还资源；MTC、MoveIt、Nav2 和设备桥只使用被授予的资源。
- 取消请求、取消已受理、底层动作终止、实际停稳和资源归还是不同事件，必须分别记录。
- 旧动作终止/有效隔离、必要实测安全确认和可恢复的结果/资源交接未完成时，不授予
  冲突资源的新运动权限，也不自动恢复旧命令；中间阶段交接用已持久化检查点，不要求
  整项任务终态。隔离范围覆盖载荷、包络、场景等真实依赖；
  独立只读或确无资源/依赖冲突的任务不必等待整机所有动作终止。
- 子 Action 终态、业务结果持久化和资源交接分别记录；业务结果可以标明 `release_pending`
  或 `quarantined`，不能把业务终态等同于资源空闲。异步 ACK/反馈按关联 ID、epoch 和偏序
  处理，不能依赖到达顺序。

## MTC / MoveIt 的正确位置

- MTC 是分阶段、候选和场景假设的规划器；使用 `SerialContainer`、候选/回退和多组规划时，保留每段的前置条件、场景操作和确认屏障。
- 规划副本、实时 PlanningScene、物体/载荷账本是三个状态空间。MTC 的 attach/detach 只是规划假设，不能直接提交真实 `ATTACHED` 或 `PLACED`。
- 计划必须绑定 `task_id/context_id`、对象/场景/地图/定位/标定/包络版本、完整关节状态、底盘位姿和创建时刻。执行前检查实际起点和有效期。
- 释放前必须验证“释放后退臂—收臂”至少有一条合格路径；无解时保持夹持并返回可诊断的失败原因。
- 先复用公共碰撞、奇异点、限位和时间参数化校验；候选排序不能抵消硬约束失败。
- 当前 `/transport/plan_manipulation` 只规划并返回分阶段结果；`/transport/plan_skill` 只返回经过校验的具名/位姿/夹爪轨迹。不要把它们写成直接发关节命令的接口。
- 使用真实组名 `arm_left` / `arm_right`；不要把文档中的 `left_arm` / `right_arm` 当作当前实现。

## 运输准入与包络握手

底盘移动前必须完成：

1. 抓持、附着、抬升和运输姿态的实测确认；
2. 基于实际姿态/载荷提交保守 `RobotEnvelope`，先保持 `transport_ready=false`；
3. `/navigation/set_robot_envelope` 的版本、有效期和两张 costmap 足迹确认；
4. 导航仲裁、BT、规划和控制策略绑定同一包络 epoch；两张 costmap、planner、controller、policy 共五个消费者确认应用，不再要求独立末级保护 ACK；
5. 到站后停稳，再重新获取 Place 场景快照，不把运输前的场景当作终态真值。

包络变更必须是停稳事务。扩大/缩小几何、切换姿态、清除载荷和恢复任务都要记录版本、原因和确认者；不能以默认 home 姿态代替实际双臂/载荷几何。

上肢状态失效由导航准入/BT/控制器撤销后续运动，不在底盘速度输出端重复判断账本或包络；停车命令、平滑器的减速尾段、实际停稳和资源归还是不同证据。变更这一链路后必须重新测量停止过程，不能沿用旧末级速度截断的停止时间。

## 模块内实现建议

- 任务核心：无 ROS 状态机/资源租约/事务对象，单独测试状态转移和版本失效；实现语言遵循上述项目约定。
- ROS 适配：Action server/client、MoveIt/MTC、Nav2 和包络服务放在薄适配层；回调只入队，不在高频线程做阻塞规划。
- 执行适配：每段轨迹前检查起点、场景、时效和控制器状态；完成后读取反馈并提交确认事件。
- 诊断：每个任务/步骤/候选/版本使用同一关联 ID；将计划成功、执行成功、实测停稳和物体确认分开。

## 外部参考

- MTC 的层级阶段与 PlanningScene 传递：[MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)
- Humble MTC 示例：[Pick and place with MTC](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- ROS 2 长任务取消/反馈契约：[Actions](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html)
