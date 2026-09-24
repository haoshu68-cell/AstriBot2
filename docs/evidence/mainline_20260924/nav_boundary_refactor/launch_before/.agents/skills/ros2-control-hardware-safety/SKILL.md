---
name: ros2-control-hardware-safety
description: 在本仓库修改底盘/机械臂/夹爪控制器、ros2_control、厂家 SDK 桥接、硬件启动或运行时安全保护时使用。明确硬件接口、控制器、资源抢占、实时循环和上层规划的所有权，保持仿真与真机的安全门槛。
---

# ros2_control 与设备执行安全

核对本次设备和命令链入口，必要时使用 [源码索引](../astribot-architecture-design/references/project-map.md)。实现语言统一遵循 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)。实际启动、探查或清理栈才读 [ros2-stack-ops](../ros2-stack-ops/SKILL.md)，真实设备接入或运动读 [硬件手册](../../../docs/manuals/HARDWARE_OPERATIONS.md)；纯接口设计不必加载完整运行流程。

## 先做执行所有权审计

确认每个命令接口只有一个生产写入者：

```text
任务/规划：产生目标与轨迹，不写设备接口
控制器：按周期将参考转成命令，声明并占用 command interface
ResourceManager/硬件接口：读状态、写命令、管理生命周期
SDK/厂家桥：设备会话、错误码、时效、急停和刹停语义
独立保护：最新状态、限位、包络、速度/加速度/制动约束和失联停车
```

- 不因目录整理或“开启双臂并行”删除现有锁、控制权或安全门槛。
- 当前厂家 SDK 桥不自动等于 `ros2_control` 硬件插件；先记录真实执行入口，再决定是否迁移。
- 规划器、任务节点、RViz、测试工具和 operator UI 不能绕过桥接/控制器直接同时写同一硬件。
- 仿真控制器、真机 SDK 和最终保护可共用上层任务接口，但不假设时延、错误码、停止距离和反馈质量相同。

## ros2_control 参考实现边界

Humble 中的标准关系是：Controller Manager 通过 Resource Manager 管理硬件组件和控制器；循环按 `read -> update -> write` 运行；控制器通过 pluginlib 提供并声明所需 command/state interfaces。按此模式设计新插件时：

- `on_init/configure/activate/deactivate/cleanup/shutdown/on_error` 的生命周期要完整且可诊断；
- `read()` / `write()` 不做无界分配、阻塞日志或不可控网络等待；
- 控制器只写已声明且被授予的接口；切换控制器要有明确的 stop/start 顺序和超时；
- joint/velocity/effort/acceleration/jerk、包络和 watchdog 约束在正确层生效，不能用放宽限制修复规划失败；
- 时间戳和时钟域明确，仿真 `use_sim_time` 不能被当成真机时钟证明；
- 错误返回应导致受控停用/回退，并让上层获得结构化故障原因。

## 真机变更门槛

涉及真实运动前依次确认：

1. 描述、控制器配置、关节组、校准和限位版本一致；
2. 当前控制器和硬件状态已读取，只有一个合法命令所有者；
3. 起点、反馈新鲜度、动作时效、速度/加速度/制动预算均通过；
4. 若替换、抢占或取消旧运动，已传播停止请求并确认旧执行不能继续发命令、底层终态和实测安全状态；首次启动则核对无旧命令/有效控制者及当前反馈，不虚构一次取消；
5. 急停、SDK 会话、设备写入准入和最终防护仍有效；
6. 仿真/离线检查结果与真机证据分开报告。

语言选择及迁移按 `robot-runtime-cpp`。保持相同消息、坐标、采样时刻、取消和故障语义，先以回放/影子计算比较，再切换执行输出。

## 外部参考

- Humble ros2_control 架构：[Controller Manager / Resource Manager / Controllers](https://control.ros.org/humble/doc/getting_started/getting_started.html)
- Humble Controller Manager 与实时循环：[Controller Manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
- ros2_control 控制器接口互斥说明：[Universal Robots controller usage](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/usage/controllers.rst)
