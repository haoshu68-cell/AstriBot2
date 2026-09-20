# 夹爪物理耦合与关节跟踪偏差（仿真）

本次按“关节6/覆盖失效 → 重复搬运 → 探索存图/多地图”推进。仅操作自有 domain 213、partition `astribot_operator_validation_213`；不下发真机指令。临时证据根目录 `/tmp/astribot-joint6-fix`，尚未作为持久归档。

## 定位证据与责任边界

r21 的 joint6 在 ROS 169.11 s 实际/期望速度均约 0.00721 rad/s，169.13 s 实际速度突然达到 0.62933 rad/s，期望仍约 0.00721。169.25 s 实际 0.100499、期望 0.048823 rad，误差 0.051675 rad。该位置不在 joint6 的 ±0.76 rad 限位处，因此不能套用此前 joint3 硬限位解释。r21 没有接触记录，无法追认其具体碰撞对。

新增只读 C++ Gazebo 接触证据后，r22 复现为 TRANSPORT_POSTURE 的 joint7 跟踪超限：左夹爪 L2 与取料台从 ROS 75.959 s 开始发生接触，84.439 s 保护触发，误差 0.056484 rad。录包中有 864 个 L2/台面接触窗口、8 个 R11/台面接触窗口。失败后物理连杆姿态反算显示：L2 应为 −0.466377 rad，实际约 0.000036 rad，差 0.466413 rad；R1 差 0.343176 rad。此为停机后的快照，不冒充故障时刻的同步测量。

根本配置缺口：URDF 描述了 mimic，但 Gazebo 的 ros2_control hardware info 只注册了主关节。已安装 gz_ros2_control 0.7.20 的运行日志没有加载从动关节。上游 [Humble 实现](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/humble/gz_ros2_control/src/gz_system.cpp) 遍历 hardware info 中的 joints 并读取 mimic/multiplier 参数；仅有 URDF mimic 不能代替该注册。

## 修改

- 在仿真 ros2_control xacro 中注册左右夹爪共 10 个从动关节，master、multiplier 与 URDF 一致。仅导出状态接口，不为从动关节增加独立命令接口；主关节控制器保持原有所有权。
- 修正夹爪注释中“已正确 mimic”的旧推断。没有修改 SLAM 注释或硬件 SDK 控制。
- 可选 C++ `astribot_contact_evidence` 只请求/读取 Gazebo ContactSensorData 并发布证据，不写姿态、力或运动命令。验证 launch 只允许专用域/partition，并通过现有 ros_gz_bridge 接入统一录制器。
- 接触按物理步采集、10 ms 仿真时间窗口合并。ROS 消息时间是窗口结束时刻；每对保留窗口最后一次接触，不能用于精确冲量积分。原生消息的 observed_collision_count 实测为 12；无注册对象时的空消息不能作为无碰撞证明。
- 不改变 0.05 rad 关节跟踪、20 mm 底盘漂移、碰撞或覆盖时效门槛。依然采用 MTC 0.1 rad 规划余量、速度/加速度缩放 0.03、仿真 idle hold=true/kp=10。

## 验证记录

| 轮次 | 结果 | 证据与限制 |
|---|---|---|
| r22，修复前 | RECOVERY_REQUIRED | joint7 超限，接触与夹爪物理耦合错误得到直接记录；不能代替 r21 joint6 的缺失接触证据 |
| r23，修复后 | SUCCEEDED，206.070 s 墙钟 | 45 个账本事件，两个导航目标及完整抓取/放置/解除附着/收臂；PLACED、attachment 空 |
| r24，第二轮启动 | 未派发 | planner_server/get_state 请求失败，生命周期管理器中止；物理和控制器就绪不等于 Nav2 就绪 |

r23 正常关闭录包后分析：17,061 条接触消息无碰撞对，最大 active 关节误差 0.026523 rad，最大底盘平移 0.00001941 m；决策记录没有 coverage_ok=false，约束没有 REQUIRED_COVERAGE_UNAVAILABLE。其他等待原因仍存在，不等于全程无等待。运行日志确认左右夹爪 10 个从动关系已加载。joint_states 未包含从动状态，不能用该话题宣称实际耦合误差已测量。

新增 xacro 回归覆盖启用/禁用夹爪两种情况：从动关系与 URDF 一致、无从动独立命令接口、主关节命令保持；pytest 2 例及 CTest 同一测试套件通过，不重复计数。C++ 插件和验证包隔离构建通过。

覆盖问题的原始优化和决策证据字段见 [覆盖时效修复](COVERAGE_TIMING_REPAIR_20260919.md)。r23 一轮零覆盖失效只表示本轮观察结果；没有控制所有机器负载因素，不能声称残余时效问题在任意负载下消失。

r24 仅重启本轮导航组件后全部 active，保留世界、记录器和会话后再派发；这是启动恢复手段，不是 DDS 超时根因修复。后续结果逐项追加，不删除失败轮次。

### 第二轮结果

r24 搬运 SUCCEEDED，224.300 s 墙钟，45 个账本事件、PLACED/attachment 空。最大 active 关节误差 0.036488 rad，底盘平移 0.00002824 m；38,727 条接触消息无接触对。因包含启动恢复前等待，消息数量不能直接比较运行性能。

任务结束后的 ROS 413.925 s 物理位姿快照，五个左夹爪从动关系最大误差为 5.02e-6 rad。此时主关节接近 0 rad，仅证明收回后的静态耦合；不冒充所有开度、所有运动过程的精度保证。

本轮记录中有一次 coverage_ok=false：ROS 414.698 s，scan capture=414.200 s、valid_until=414.500 s，ACQUISITION_EXPIRED。发生在上述任务结束快照之后、停止录制之前；这次年龄0.498 s确实超过0.3 s预算，保留失效保护。搬运期间没有覆盖失效决策，也没有 REQUIRED_COVERAGE_UNAVAILABLE 约束。r23/r24 是两次完整动作成功，r24 的一次启动失败仍计入启动可靠性问题。

两轮结束均正常关闭录包后读取；r24 按 PID/启动时间清理51个自有进程，remaining为空。其他域的实验栈保留。
