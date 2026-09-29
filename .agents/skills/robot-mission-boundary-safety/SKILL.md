---
name: robot-mission-boundary-safety
description: 在本仓库设计区域任务、覆盖/探索、禁行区、地理围栏、回充、远程控制、暂停恢复、电池天气或设备看门狗时使用。吸收扫地机和割草机的任务边界、脱困和报警模式，但映射到当前 ROS2/Nav2/SDK，不假设本项目已有 RTK、充电桩或独立 MCU 安全实现。
---

# 区域任务、脱困与报警安全

先按 [源码入口索引](../astribot-architecture-design/references/project-map.md) 核对本次区域/恢复入口。只加载涉及的领域：观测时效用 `perception-world-model`，携物/释放事务用 `robot-task-orchestration`，设备级停止用 `ros2-control-hardware-safety`，报告已有证据用 `robotics-validation-evidence`。无需为普通报警状态机加载完整整机/行业架构。

## 把区域和任务边界当作一等数据

覆盖、探索、路线、搬运或回充任务至少绑定：

```text
mission_id / task_id / operator_or_source
map_revision / zone_revision / no_go_revision
boundary_frame / geometry / keep_in_or_keep_out
localization_epoch / envelope_epoch / calibration_revision
valid_from / expires_at / pause_reason / recovery_policy
```

- keep-in、keep-out、动态障碍、机械臂/载荷包络和最终停车空间分开建模，不能把一张多边形同时当作全部安全证明。
- 边界、地图、定位、包络或标定版本改变会使候选失效；不能用旧计划继续运动。
- `PAUSED_SAFE` 后不自动恢复旧命令。恢复前重新检查定位、边界、障碍、设备状态、命令租约和任务版本。
- 区域任务的完成由覆盖/任务语义确认，不能由到达一个中间点推断。

## 脱困必须是分级、有界的恢复事务

把“无法继续”至少分成以下类别，并为每类定义动作、预算和终态：

| 类型 | 典型证据 | 允许动作 | 终态/报警 |
|---|---|---|---|
| 短暂阻塞 | 传感器仍新鲜、原地等待安全、障碍可能离开 | 减速、原地等待、一次受限重规划 | 阻塞 episode，恢复后清除；不重复刷屏 |
| 局部困住 | 里程计无进展、局部代价图封堵、存在验证过的退路 | 有界后退/转向/恢复动作，之后重新建图或重定位 | `RECOVERY_RUNNING` → `RECOVERED` 或 `NO_SAFE_MANEUVER` |
| 任务不可达 | 多次候选失败、区域无出口、包络/放置条件无解 | 保持安全姿态，保存上下文，等待人工/回充/重新规划 | `BLOCKED`/`FAILED`，通知一次并带原因 |
| 硬故障 | 急停、抬升/轮离地、碰撞、过流、过热、SDK/心跳失联 | 立即零命令/设备级停机；不执行高层脱困 | `FAULT_LATCHED`，必须按策略人工复位 |

- 每个 episode 有最大持续时间、重试次数和总预算；子步骤不能通过重置计时器绕过预算。
- 脱困动作先经过完整包络、碰撞、定位和停车空间检查；不能把“反向走一小段”当作天然安全。
- 脱困期间保留受影响的底盘、双臂、载荷和包络资源；未确认安全状态和旧执行隔离前，不向冲突任务重新授予这些资源。无资源冲突的只读工作可以继续。
- 任何脱困后要重新获取 WorldSnapshot、路径/场景版本和设备反馈，不能沿用旧候选。
- 定位丢失单独建模：先进入安全保持，只有定位质量恢复、重定位结果稳定且版本重新绑定后，才允许有限次数的原地重定位或恢复动作。
- 自动脱困不能覆盖“设备被卡住、轮子悬空、传感器堵塞、刷头/关节过流、区域无出口”等不同原因；每类原因有不同恢复动作和人工提示。

## 报警不是一条字符串

报警事件至少包含：

```text
alarm_id / severity / reason_code / source
task_id / episode_id / first_seen / last_seen
ack_required / latched / retry_count / recovery_budget
current_motion / pose_source / frame / versions
recommended_action / operator_ack / clear_condition
```

- `INFO`（状态变化）、`NOTICE`（需要关注）、`WARNING`（任务受阻）、`CRITICAL`（必须停机）分级；相同 episode 按策略去重，状态升级仍必须上报。
- 报警清除与恢复权限分开：传感器恢复不等于允许继续运动；硬故障保持锁存直到明确复位。
- 去重键绑定设备/原因/episode，记录合并时间窗和原始次数；不同故障不因文案相同被吞并。确认、升级、恢复与再次发生独立记事件，进程重启不重置重试预算或清除锁存。`operator_ack` 只表示收到提示，不满足物理故障清除条件。
- operator station、日志和机器状态消息可以共享 `alarm_id`，但 UI 不能成为第二个控制器。
- 不以“节点仍在”“收到心跳”或“命令发送成功”代替设备已停机、已脱困或物体已安全的证据。
- 对“短暂阻塞/定位丢失/无障碍恢复/需人工移机/硬故障”使用不同 `reason_code`；同一故障的重复通知要抑制，但恢复、升级和人工确认必须产生事件。
- 进入任何可自动运动的恢复前，先验证急停、碰撞、抬升/轮离地、反馈和设备写入准入；未验证安全输入时保持 `FAULT_LATCHED` 或 `NOT_READY`。

## 本项目的适配方式

- 现有导航阻塞、路线/探索任务、`RobotEnvelope`、最终保护、设备桥接和 operator station 可作为实现入口；新增的是统一的 episode、报警和恢复语义。
- `RobotEnvelope` 与地图区域/局部 costmap/最终保护分别维护，在准入时求交集；包络失效优先进入安全保持，不自动缩小几何放行。
- 双臂携物时，脱困默认保持保载姿态并停止底盘；涉及改变臂姿、放置或释放必须回到 `robot-task-orchestration` 的规划和确认事务。
- 回充、电池、天气和 RTK/地理围栏目前属于候选能力；没有设备接口和验收证据时标记为 `not implemented`。
- 区域编辑、REST/MQTT 服务、恢复和报警实现统一遵循 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)，不设置 Python 薄适配例外。

## 外部参考

- OOMWOO：模块接口、仿真优先、CPU/MCU 分层、硬安全独立于 Linux/ROS2、定位丢失恢复：[Architecture](https://github.com/makerspet/oomwoo/blob/main/docs/ARCHITECTURE.md)、[Software interfaces](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)、[localization recovery notes](https://github.com/makerspet/oomwoo-install)
- OpenMower：多区域、障碍/天气暂停、抬升或碰撞急停：[OpenMower](https://github.com/ClemensElflein/OpenMower)
- OpenMower ROS2 端口：ROS2 化项目的范围和未完成能力：[OpenMowerNext](https://github.com/jkaflik/OpenMowerNext)
- OpenMower 的安全启用门槛：先验证急停传感器，再允许 mower enable：[OpenMowerOS](https://github.com/ClemensElflein/OpenMowerOS)
- Valetudo：本地控制、地图/任务抽象、REST 与 MQTT 适配；其错误类型示例包含激光头堵塞、碰撞条卡住、轮子悬空、刷头堵塞和设备被困：[Valetudo](https://github.com/tarik02/valetudo)、[error taxonomy example](https://github.com/rand256/valetudo/discussions/361)
