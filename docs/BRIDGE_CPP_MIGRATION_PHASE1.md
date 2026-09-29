# Runtime Python→C++ migration: verified runtime slices, arm/gripper math, navigation math, and wheel control

> 历史迁移分阶段记录。本文中的 Python/cpp 入口选择开关已由后续清理移除；当前入口及未完成项以 [去 pybind 记录](PYBIND_REMOVAL_20260921.md) 为准。旧测量仍绑定当时源码/构建，不能代替当前整栈或真机验收。

第一阶段迁移 `chassis_integrator.py` 的无 ROS、无厂商 SDK 数学核；第二阶段继续
迁移 `PoseFrameIntegrator`、`SdkPoseHistory`，随后把 `ChassisBridgeCore`、夹爪核心、
机械臂执行器和 waypoint dispatcher 做成可回放的 C++ 状态机候选。ROS 节点和 SDK
端口保持不变。所有候选都由显式环境变量选择，默认仍使用 Python oracle；只有同输入
差分、边界/故障注入和隔离性能实验通过，才允许进入后续 ROS 集成 A/B。逐函数 pybind
调用的边界开销在状态机仍为 Python 时可能超过数学计算本身，所以不把单个 kernel 的
微基准写成端到端收益，也不改变启动方式、控制权、消息接口或故障门槛。

最终目标是删除 pybind，而不是把它作为运行时架构。当前 pybind 只承担迁移期的
Python oracle 对照、故障回放和小步 A/B；每当一个完整运行时消费者改为直接链接
C++ 库并完成 ROS/仿真/真机证据，该消费者的 pybind facade 和环境变量开关就删除。

## 已迁移的函数

- `wrap_angle`
- `local_pose_displacement`
- `to_local_velocity`
- `integrate_step_dt` / `integrate_step`
- `pose_error` / `error_magnitude`
- `chassis_feedback.py` 的 `leash_error`、`pose_jump_distance`、`odom_drift`
- `chassis_odom_source.py` 的跳变/累计行程/速度坐标转换状态核
- `measure_tick_dt` 的首拍兜底、时钟回退和最大步长钳位

`measure_tick_dt` 的参数校验和非有限时钟输入仍由 Python 保持；有限输入在
`ASTRIBOT_BRIDGE_NATIVE_KERNELS=1` 时使用 native kernel，返回的 `TickDt` 字段和
原因文本逐项与旧实现一致。

第二阶段的两个状态对象已经提供 C++ 实现，并由同一个 Python 兼容 facade 选择。
旧实现仍保留为差分 oracle；未启用 native 时，运行行为完全使用旧 Python 状态对象。

机械臂桥接的纯计算 slice 也已接入同一 native module：轨迹线性/Hermite 插值、
最大误差和夹爪命令/弧度换算。`ArmTrajExecutor`、action 取消和 SDK 流式状态机仍
保持 Python；完整 golden replay 已加入，但状态机默认仍不切换。

夹爪 `GripperController` 的端口调用核心现在也有独立 C++ 候选，使用同一组
`SessionPort`、clock 和 sleep 注入，不改 ROS service 或写通路准入。Python facade
将结果/事件转换回原有 `GripperResult`/`StatusEvent`，默认仍使用 Python oracle；
只有 `ASTRIBOT_GRIPPER_NATIVE_CORE=1` 才选择 native。

`ArmTrajExecutor` 的完整 `STREAMING → SETTLING → DONE`、取消后的
`HOLDING → CANCELED`、限位加载、跟踪超限和收敛超时状态机也已有 C++ 候选。
它通过 `SessionPort` adapter 保留原有关键字调用（`control_way`、`use_wbc`、
`add_default_torso`），并以 `ASTRIBOT_ARM_NATIVE_CORE=1` 显式选择；默认仍是
Python oracle。阻塞式 `WaypointDispatcher` 也已有同一 adapter 下的 C++ 候选，
但 ROS action 节点和真实后端仍暂不切换。

`ChassisBridgeCore` 的完整内环/外环候选也已接入同一 native module，保留
`enable/disable/reset_leash`、cmd_vel 看门狗、scan/pose 宽限停车、leash、SDK 读写
异常、位姿换帧、漂移诊断、tick/velocity 统计和事件码。`ASTRIBOT_BRIDGE_NATIVE_CORE=1`
只切换离线或独立 A/B 实验中的核心对象；ROS timer、消息转换、端口所有权和默认
停车路径仍由 Python 节点掌握，避免在没有真实消息序列证据时改变控制权。

本轮“可迁移实现”以运行时数值和端口状态机为边界：启动/验证脚本、ROS launch 和
消息转换、SDK/TF/参数适配、任务编排、`fusion/risk` 的观测关联与风险裁决继续由
Python 掌握。这些模块的主要工作是时序、所有权和拒绝策略；把它们直接改成 C++ 会
同时改变接口与安全责任，不能用离线 kernel 基准代替验证。它们只有在 ROS bag
重放、独立 domain A/B 和真机静止/只读证据齐备后，才进入下一轮状态机迁移评审。

导航策略的恒速预测小核也已加入独立的
`astribot_s1_navigation_policy_native` 包：`stopping_horizon`、`body_pose`、
`sampling_margin`、`footprint_axes` 和车体系到 world 系的 `body_to_world_xy`。
`motion_geometry.py`、`swept_geometry.py` 与 `cmd_vel_body_to_world_node.py` 的纯
转换层只在 `ASTRIBOT_NAV_NATIVE_KERNELS=1` 时调用它们，默认保留
Python 路径。`fusion`、`risk`、`protection_node` 的 ROS 时序和完整 footprint clearance
暂不切换，因为它们包含观测时效、保守拒绝和多边形覆盖状态，必须先完成整段录制回放。

为最终移除 pybind，`astribot_s1_navigation_policy_native` 还提供了直接链接
`navigation_math` 静态库的 C++ `cmd_vel_body_to_world_cpp` 节点。它直接拥有
`/cmd_vel` 转换、/odom 有效性、姿态采样止损、命令 watchdog 和零速发布，不经过
Python 或 pybind；非法四元数、body→world 旋转、超时停车和姿态超限单元测试已通过。
`navigation.launch.py` 增加 `cmd_vel_node_impl:=cpp` 的显式 A/B 入口，默认仍为
Python oracle；独立 ROS domain 的消息序列、停车时刻和重启恢复对齐后，才允许切换
默认实现，随后删除该节点的 Python/pybind 适配。这个节点是去掉 pybind 的第一条直接
运行时路径，`nav2_full_bringup.launch.py` 已透传同一开关；后续底盘、保护和任务状态机
沿同一方式迁移。

同一批直接 C++ 节点还加入了 `costmap_scan_cpp`，它复制 LaserScan 元数据、直接调用
`costmap_clearing_ranges` 并在非法扫描时保留“记录错误、不发布该帧”的拒绝语义。
`costmap_scan_node_impl:=cpp` 已从两层导航 launch 透传，默认仍走 Python。该节点和
cmd_vel 节点都能在 `ASTRIBOT_BUILD_PYBIND=OFF` 下编译，因而可以独立做无 pybind
构建和 ROS A/B；只有两条消息序列一致后才删除对应 Python 适配。

机械臂二值限速节点也已有直接 C++ 实现 `arm_speed_limiter_cpp`。它直接订阅
`sensor_msgs/JointState`、查询监控连杆 TF 并发布 `nav2_msgs/SpeedLimit`，保留水平
伸展阈值迟滞、关节偏差回退、TF 缺失/陈旧时按“展开”保守限速、定时重发和
`percentage=true` 的消息契约。`arm_speed_limiter_node_impl:=cpp` 已从两层导航 launch
透传，默认仍是 Python oracle；节点启动参数和无消息启动检查已通过。它与 cmd_vel、
costmap 两条路径一起在 `ASTRIBOT_BUILD_PYBIND=OFF` 下直接链接静态数学库，后续需在
独立 ROS domain 对比相同 JointState/TF 序列、限速消息和 TF 故障恢复后，才可删除
该节点的 Python/pybind 适配。

legacy 包络协调器也已有直接 C++ 节点 `envelope_coordinator_cpp`。它读取与 Python
`Profile` 相同的 JSON（包括 `base_profile` 和历史停止参考），保留仿真时钟/硬件证据
准入、`/odom` 新鲜度与停止门槛、包络字段上下界、epoch 递增、两层 costmap
footprint 的形状/TF/时间确认，以及 `TRANSPORT_READY` 租约发布。`fixed_v2` 明确在
launch 校验阶段拒绝切换，继续由现有 `FixedEnvelopeAdapter` 负责，避免两套几何协议
静默降级。`navigation.launch.py envelope_node_impl:=cpp` 和
`nav2_full_bringup.launch.py` 透传该开关，默认仍为 Python。

`test_envelope_coordinator_cpp_isolated.py` 在独立 ROS domain 回放 clock、静止/运动
odom、双 footprint 和服务请求，覆盖初始双确认、非法缩小、运动中拒绝和静止后重新
提案；`test_envelope_coordinator_ab.py` 用完全相同消息序列比较 Python/C++ 服务结果、
最终几何和 ready 原因，三项服务结果和最终 `TRANSPORT_READY` 均一致。直接节点在
`ASTRIBOT_BUILD_PYBIND=OFF` 下构建，启动时不加载 Python/pybind；这仍是隔离证据，
没有把 Nav2/Gazebo 闭环或硬件验收写成已完成。

native 包增加了 `ASTRIBOT_BUILD_PYBIND=OFF` 构建选项；该模式不查找、不生成
`_navigation_math_native`，直接 C++ 节点和两个 C++ 单元测试仍可独立构建并通过。
这使“去掉 pybind”可以在每个运行时消费者完成后先做无桥接构建，再删除对应 Python
oracle，避免把过渡层误当成最终依赖。

同样的 `ASTRIBOT_BUILD_PYBIND=OFF` 已加入轨迹 native 包；全向轮 native 包已直接从
构建系统移除 pybind 目标和依赖。底盘/机械臂数学静态库、轮控静态库和 `chassis_math`
CTest 在无 pybind 构建中通过。当前轨迹包的夹爪、机械臂和底盘完整
状态机仍以 Python SDK/端口为消费者，不能仅靠关闭模块就宣称已完成迁移；必须先提供
直接 C++ 端口或 C++ ROS 节点，再删除它们各自的过渡 facade。当前 `arm_speed_limiter_cpp`
已满足“直接 C++ ROS 节点”这一阶段条件，但其 Python 默认路径仍需等待消息序列 A/B
和静止硬件验证。

`ConservativeFusion.snapshot()` 的逐轨迹派生部分已有独立的 geometry native kernel，
现在可用 `ASTRIBOT_FUSION_NATIVE_SNAPSHOT=1` 选择；观测 ingest、关联、free-space
确认、epoch 清理和风险评估仍由 Python 掌握。该开关只在完整 snapshot differential
replay 通过后使用，默认关闭。

200 条轨迹的 7 次隔离回放中，Python snapshot 为 0.002167 s，native facade 为
0.002171 s，轨迹数量和中心 checksum 完全一致；对象重建成本抵消了 kernel 收益，
所以该开关当前用于差分而不是性能承诺。

导航保护层现在增加了可选的 `CommandRestriction` native 状态对象，以及
`scan_usable`、`costmap_clearing_ranges` 两个无状态安全核。Python 仍保留拒绝路径、
公开的 `output/recovering` 字段和非有限输入回退；如果调用方手动修改
`recovering`，native facade 会在下一拍同步该状态。保护对象在非有限输入后持续使用
Python，直到显式 `reset`，避免两套限制状态重新合流。

无动态多边形 footprint 时，`swept_geometry.clearance_many` 的矩形批处理也可通过
`ASTRIBOT_NAV_NATIVE_KERNELS=1` 调用 native；多边形 footprint、连续 sweep 递归和
风险评估仍保持 Python，避免把保守包络或对象所有权边界混入单个 kernel。矩形
`continuous_sweep.motion_clearance` 的有界 11 层区间递归已加入同一 native 包；
wrapper 只在有限、顺序正确且使用矩形 footprint 时切换，其他输入回到 Python
验证路径。`risk.py` 的路径投影和剩余路径构造也增加了批量 native kernel；世界模型
关联、风险裁决和多边形 footprint 仍保持 Python。

`scan_occupancy.py` 的 `occupied_cells` 和 `angular_box_free` 也加入同一导航 native
包。Python 继续保留空角点、非有限点、分辨率和角度增量的异常边界；仅在有限输入
下切换，native 结果再按原有 cell ID/角度盒布尔契约返回。由于逐次 Python→C++ 的
列表转换当前更慢，这两个核默认仍走 Python，native 开关只用于差分和后续批处理
整体迁移。

`posture_monitor_policy.py` 的姿态超限、采样窗口、退化源判定和状态文本也加入
导航 native 包。它不读取 TF、不发布零速度，节点仍负责 ROS 订阅和止损动作；native
只提供同输入策略结果。由于当前每次姿态判定跨越 pybind 边界，200,000 次混合调用
的 Python 为 0.187525 s、native facade 为 1.939433 s（慢 934.22%，checksum
`2.000000000000e5`），因此默认仍使用 Python，只有把姿态窗口与节点状态一起下沉后
才有性能切换依据。可复现实验脚本为
`ws_robot/src/astribot_s1_navigation/test/benchmark_posture_monitor.py`。

保护节点使用的 `control_time.py` 仿真时钟 watchdog 也已提供 C++ 状态对象候选，
保留时钟回退、仿真暂停、采样时间线 floor、命令 floor 和 wall freshness 语义。
200,000 次 advance/accepts/fresh/command_fresh 混合回放中，Python 为 0.327646 s，
native facade 为 0.494983 s（慢 51.07%，checksum `8.260119999200e7`）。因此继续
保持显式 native 选择，必须等保护节点整段状态迁移后再评估端到端收益。可复现实验脚本
为 `ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_control_time.py`。

路径候选和路径证据也已下沉为无 ROS native slice：`candidate_variants.lateral_variants`
保留端点和 C2 taper，`path_evidence.assess_path` 保留时间/epoch/未知障碍/距离预算
裁决。5 万条 24 点路线变体回放中，Python 为 1.613463 s、native facade 为
3.878120 s（慢 140.36%，checksum `2.195999999998e7`）；20 万条路径证据回放中，
Python 为 0.309216 s、native facade 为 0.516977 s（慢 67.19%，checksum
`7.002704142853e5`）。两者暂不默认启用，避免把边界转换开销带入 250 Hz/策略周期。
实验脚本分别为 `benchmark_lateral_variants.py` 和 `benchmark_path_evidence.py`。

`behavior.YieldPolicy` 的 HOLD/SLOW/CONTINUE、clear confirmation、clock rewind、
episode 和 blocked budget 状态机，以及 `execution_context.py` 的 goal/map/localization/
path version 状态也已有 native 候选。同输入状态回放通过；10 万次 YieldPolicy
回放 native facade 慢 27.71%，10 万次 ExecutionContext 回放慢 30.24%，所以仍以
Python 为默认实现。对应脚本为 `benchmark_yield_policy.py` 和
`benchmark_execution_context.py`。

`sensor_health.py` 的 `scan_coverage`、`movement_directions` 和
`coverage_allows_motion` 也加入了同一 native 包。Python 仍负责 `BearingCone`/
`Vec3` 合同、非法对象和非有限输入边界；native 只接收已确认的数值列表，并把扁平
结果还原为原有不可变对象。随机扫描、旋转/平移方向和覆盖缺口回放通过，边界异常
仍走 Python。20,000 次固定回放中，扫描覆盖 Python 为 1.646037 s、native facade
为 2.364730 s（慢 43.66%）；方向判定慢 132.01%，完整覆盖判定慢 136.29%，三个
checksum 分别保持 `4.400000e5`、`6.000000e4`、`0`。可复现实验脚本为
`ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_sensor_health.py`；
当前逐次边界转换没有性能切换依据，默认继续使用 Python。

`planning_session.py` 的请求所有权状态也已有 `PlanningSessionState` C++ 候选，覆盖
每目标请求预算、episode deadline、挂起请求复用、时钟回退锁定、响应 lease 和 retire。
Python 保留 `PlanningBudget`、`PlanningRequest`、`Stamp`/`Version` 合同与异常类型，
native 只承接内部状态转移；`ASTRIBOT_NAV_NATIVE_KERNELS=1` 才启用。固定 20,000
个新目标的 activate/request/response/retire 回放 checksum 为 `200218894`，Python
为 0.168116 s，native facade 为 0.178163 s（慢 5.98%）。规划请求状态的差分和
时钟回退故障注入已通过，但 ROS route coordinator、规划器服务和路径提交仍未切换。
可复现实验脚本为 `benchmark_planning_session.py`。

全向底盘的 `_WheelLoop` 和四轮逆运动学也已加入独立的
`astribot_s1_chassis_effort_drive_native` 包。Python `omni_effort_drive_node.py` 现在
保留为纯 Python oracle；该包不再构建或安装 wheel pybind 模块。
每轮 PID 的积分、抗积分饱和、导数时间门控、摩擦前馈死区、限幅和 reset 语义均保留，
逆运动学的非法轮半径仍返回四个零目标。控制节点的 ROS 发布、反馈看门狗、控制器
所有权和停车路径在 Python oracle 中保持不变。

在此基础上，`omni_effort_drive_cpp` 已直接接管同一组 ROS 话题和 100 Hz 控制循环：
保留 `/cmd_vel` 超时将目标置零、`/joint_states` 超时将四轮 effort 置零、轮速限幅、
PID 积分抗饱和、摩擦前馈、可选 idle position hold、跟踪误差告警和四个调试话题。
`omni_effort_drive.launch.py node_impl:=cpp` 与 `warehouse_sim.launch.py`
的 `effort_drive_node_impl:=cpp` 提供显式 A/B，默认仍使用 Python。无 pybind 构建直接
生成该节点；Python/C++ 独立 ROS domain 消息回放已验证首拍及连续 effort 序列一致，
四轮符号一致，停止指令后目标归零，反馈超时后四轮 effort 全零。默认切换前仍需补
齐速度限幅、idle hold、跟踪误差和参数异常场景的逐拍 A/B。

只读的 `chassis_odom_source.py` 也已加入轨迹 native 包的可选状态核：跳变距离、
跳变计数/历史、累计行程、连续 yaw 归一化和 world→body 速度转换均在同一个
`ChassisOdomSource` 对象内完成。Python 仍负责输入长度/`None` 错误边界、`OdomSample`
和 `OdomStats` 数据契约；`ASTRIBOT_BRIDGE_NATIVE_ODOM=1`（或已有的
`ASTRIBOT_BRIDGE_NATIVE_KERNELS=1`）才启用 C++ 对象。该核只读 SDK 数据，不拥有
写通路，也不改变 odom/TF 节点的发布时序。

## 必须通过的实验

1. **C++ 单元测试**：检查 body/world 坐标、SE(2) 转弯积分、连续角度误差、正数
   步长、时间首拍/回退/钳位和非法步长。
2. **同输入差分**：固定随机种子 `20260920`，10,000 组位姿、速度、角度和频率；
   每个 kernel 与 Python 参考实现逐元素比较，绝对/相对误差 `3e-12`。
3. **边界回归**：`±π`、多圈 yaw、极小转角、极小正步长、body/world 两种输入、
   非法 frame 和零/负步长。非法输入仍由 Python API 转成 `ChassisConfigError`。
4. **状态回放**：使用同一组递增时间戳和 pose，比较 `PoseFrameIntegrator` 的
   anchor、integral、target、换向和换帧结果，以及 `SdkPoseHistory` 的时间插值。
   当前已覆盖固定 700 tick 桥接回放和 10,000 组随机状态序列。
5. **轮控差分**：固定随机种子回放 10,000 组 PID 状态，覆盖饱和、reset、正/零/负
   `dt`、摩擦死区、有限值检查和四轮逆运动学；非有限输入必须走 Python 回退。
6. **性能基线**：在同一 CPU、同一 Python 进程中分别运行 1,000,000 次 kernel，
   记录 wall time、结果 checksum 和进程 RSS。性能提升不能通过降低检查或改变
   精度获得。

### 2026-09-20 基线结果

测试机为 Intel Core i7-14700K、Python 3.10.12，固定输入，7 次重复取中位数，
每次 1,000,000 次 `integrate_step_dt`，checksum 必须一致：
可复现实验脚本为
`ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_chassis_math.py`。

| 路径 | 中位耗时 | 调用率 | checksum | 最大 RSS |
| --- | ---: | ---: | ---: | ---: |
| Python 参考实现 | 0.197147 s | 5.07 M/s | `3.033003000e8` | 12,992 kB |
| Python→C++ 兼容调用 | 0.385123 s | 2.60 M/s | `3.033003000e8` | 12,992 kB |
| pybind 直接函数调用 | 0.205030 s | 4.88 M/s | `3.033003000e8` | 12,992 kB |
| 直接 C++（同一 kernel，20 M 次） | 0.066 s | 303.7 M/s | 相同数值路径 | 3,808 kB（进程） |

当前结论是：逐函数跨 Python/C++ 边界不能作为 250 Hz 桥接性能优化，兼容调用比
Python 参考慢约 95%。因此生产默认保持 Python，native 只用于差分和后续整体状态机
迁移。直接 C++ kernel 的数量级优势只有在 `PoseFrameIntegrator` 和
`ChassisBridgeCore` 一起进入 C++、减少逐拍边界转换后才可兑现；这组微基准不宣称
端到端 ROS 延迟或真实 SDK 性能提升。

第二阶段的 fake-port 状态机回放（7,000 tick、9 次重复）结果：

| 状态对象 | 中位耗时 | tick/s |
| --- | ---: | ---: |
| Python `PoseFrameIntegrator` + `SdkPoseHistory` | 0.340386 s | 20,565 |
| C++ 状态对象（Python core facade） | 0.257571 s | 27,177 |

该隔离回放约减少 24.3% 耗时，但仍包含 Python `ChassisBridgeCore`、fake port 和
列表转换，不能外推为 ROS/SDK 端到端收益。固定 700 tick 的状态、事件和 SDK 写入
目标逐项相同，差分测试为 10/10 通过；native 仍保持 opt-in。

机械臂 cubic 插值微基准（200,000 次、7 次重复）结果：Python 为 0.508371 s，
native 兼容调用为 0.465742 s，约减少 8.4% 耗时；checksum 完全一致。这个收益
来自一次调用内完成整段向量插值，不能外推为 action 端到端延迟。

导航标量预测微基准（500,000 轮，每轮调用上述四个函数）结果：Python 为
0.564231 s（886,162 轮/s），native 兼容壳为 0.607472 s（823,084 轮/s），
checksum 均为 `2.110866390e6`。逐个标量函数跨 pybind 边界约慢 7.7%，所以这个
阶段的 native 开关不默认打开；直接 C++ 调用同一组函数为 0.427143 s。只有在
后续把预测批处理或连同保护状态机整体迁移后，才把这组核作为性能优化而不是仅作
差分依据。

导航 `CommandRestriction.apply` 微基准（500,000 次、7 次取中位数）结果：Python
facade 为 0.651758 s（767,156 次/s），直接 C++ 状态对象为 0.241871 s
（2,067,214 次/s），checksum 均为 `2.249779899e5`，状态对象约减少 62.9% 耗时；
逐拍 native facade 为 0.802602 s（622,974 次/s），比 Python facade 慢 23.2%。
因此保护状态机在完成整段批处理/节点迁移前仍不默认切换，单次 pybind 调用不能作为
端到端安全周期收益的证据。

矩形 `clearance_many` 批量微基准（5,000 个有限 pose/box、7 次取中位数）结果：
Python NumPy 路径为 0.000441 s（checksum `6.255874210e3`），native 兼容壳为
0.000755 s，checksum 相同。当前瓶颈是 Python 数组到 pybind vector 的转换；native
批处理仍先作为差分路径，默认不替换 NumPy 实现。

底盘安全数学微基准（500,000 轮，每轮 leash/跳变/漂移各一次）结果：Python 为
0.750096 s（666,581 轮/s），native 兼容壳为 1.260199 s（396,763 轮/s），
checksum 均为 `2.801231747e6`。这组标量调用同样受 pybind 边界影响，生产默认不
打开；直接 C++ 状态机迁移后再重新测端到端周期。

时间钳位核微基准（500,000 次有限输入、7 次取中位数）结果：Python 为 0.143225 s
（3,491,006 次/s），native 兼容壳为 0.335616 s（1,489,798 次/s），checksum
均为 `2.000000000e3`。因此它当前只作为同输入差分和后续整体状态机迁移的候选，
不会因为单个 pybind 调用而默认打开。

全向轮单轮 PID/摩擦前馈微基准（2026-09-23 复测，500,000 次连续 update、7 次取
中位数、固定参数和输入）结果：Python `_WheelLoop` 为 0.204233 s
（2,448,185 update/s），直接 C++ `WheelLoop` 为 0.001535 s
（325,661,826 update/s），checksum 均为 `8.377633981e6`。这是一组状态对象基准，
不代表 `/cmd_vel` 到 effort controller 的端到端延迟；节点 A/B 以消息序列和超时门槛
为准。可复现实验脚本为
`ws_robot/src/astribot_s1_chassis_effort_drive_native/test/benchmark_wheel_loop.py`。

夹爪执行核心的隔离微基准（100,000 次全闭请求、7 次重复、fake SDK）结果：
Python facade 为 0.307999 s（324,676 次/s），C++ facade 为 0.238958 s
（418,484 次/s），checksum 均为 `1.000000000e7`，隔离执行核心约减少 22.4%
耗时。该数字包含每次跨 pybind 调用的 SDK fake 回调，因此只表示当前端口形状下的
离线服务吞吐，不表示真实厂商 SDK 阻塞时长。半开持续下发、力限、写准入、未知
夹爪、越界、读回失败、SDK 异常、超时和并发拒绝的 6 项差分实验已通过；native
仍需显式设置 `ASTRIBOT_GRIPPER_NATIVE_CORE=1`，默认不改变 ROS 服务路径。
可复现实验脚本为 `ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_gripper_core.py`。

矩形连续扫掠微基准（1,000 个区间、5 次重复、固定恒速命令）结果：Python 为
0.000632 s（1,582,228 interval/s），native 为 0.000195 s（5,118,493 interval/s），
checksum 均为 `7.540138382828e2`，约减少 69.09% 耗时。结果包含区间细分、恒速
SE(2) 位姿和矩形 clearance；多边形 footprint 与上层风险裁决不在该数字内。可复现
脚本为 `ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_motion_clearance.py`。

风险路径位置批处理微基准（100,000 个距离、5 次重复、24 点路径）结果：Python
为 0.085276 s（1,172,667 positions/s），native 为 0.013991 s（7,147,412
positions/s），checksum 均为 `-4.182948678389e4`，约减少 83.59% 耗时。该数字
只覆盖路径插值和剩余路径前处理，不代表完整 `evaluate_risk()` 或 ROS 防护周期。
可复现实验脚本为 `ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_navigation_forecast.py`。

激光占用批处理微基准（10,000 点、20 次重复，同时检查角度盒）结果：Python 为
0.028745 s，native Python facade 为 0.061143 s，checksum `2.660000000000e3`，
native 比 Python 慢 112.71%。当前瓶颈是 Python 点列表到 C++ `std::array` 的转换，
所以 `scan_occupancy` native 只作为同输入差分路径，默认仍由 Python 使用；若后续把
点云/扫描批处理一起下沉，需重新测量。可复现脚本为
`ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_scan_occupancy.py`。

`ChassisOdomSource` 只读状态回放（200,000 个采样、5 次重复、world 速度输入）
结果：Python 为 0.348340 s（574,152 sample/s），native Python facade 为
0.352011 s（568,165 sample/s），checksum `5.217939314397e4`、跳变计数均为 0，
native 比 Python 慢 1.05%。这个结果说明逐采样 pybind 边界抵消了单对象的 C++ 计算
收益；它当前用于差分和后续把 odom 节点只读回路整体迁移的基线，不宣称 ROS/DDS
端到端收益。可复现脚本为
`ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_odom_source.py`。

轮控差分回放还覆盖了四轮逆运动学：随机有限参数与 Python 参考逐元素一致，半径
`<= 1e-6`、非有限参数和错误数组长度保持原有 Python 校验/回退路径。该实验只验证
kernel 数值和状态；一旦出现非有限输入，该轮对象同步 native 状态后持续使用 Python
直到 reset，避免两套积分状态分叉。不代表 ros2_control command interface 或反馈断流
时序已经验收。

机械臂执行器 golden replay 覆盖成功、取消后持续保持、跟踪误差中止和收敛超时：
固定 50 Hz 回放中，phase、error code、事件/反馈序列、SDK 调用元数据逐项一致，
关节目标最大差 `1.11e-16` rad。测试文件为 `test_arm_core_replay.py`，允许值只
用于浮点舍入，不放宽安全门槛。

机械臂完整状态机隔离基准（50,000 次 streaming tick、7 次取中位数、fake SDK）
结果：Python 为 0.216663 s（230,773 tick/s），C++ facade 为 0.108250 s
（461,892 tick/s），checksum 均为 `4.749989235e2`，约减少 50.0% 耗时。该结果
包含 SDK fake 端口回调和反馈读取；不包含 ROS action 调度或真实 SDK 阻塞时长。
可复现实验脚本为 `ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_arm_core.py`。

底盘完整状态机隔离基准（50,000 次 `inner_tick`、5 次取中位数、fake SDK、固定
`0.004 s` 时钟推进）结果：Python 为 1.598118 s（31,287 tick/s），C++ facade
为 0.209952 s（238,150 tick/s），checksum 均为 `2.082032526569e0`，native
候选减少约 86.86% 耗时。回放包含实际位置读写、位姿积分、预览目标、cmd_vel 超时
事件、统计和速度链路取样，不包含 ROS executor 调度、DDS 或真实 SDK 阻塞；可复现
脚本为 `ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_chassis_core.py`。

### 本轮复核记录（2026-09-21）

当前可复现证据统一见 [包络复核与性能报告](evidence/cpp_migration_20260921/README.md)。
包络专项 517 passed（436 配置差分、11 启动准入、70 消息协议），相邻迁移/运输/策略
回归 189 passed。协议测试中的两项专门记录旧 Python NaN odom 放行与 C++ 拒绝的差异，
不能将全部通过表述为所有输入语义均相同。ROS 时间回退的动态 TF 缓存等价性仍待验收。
继续完成清障扫描节点 20 项消息对照，修复动态参数未生效与启动失败退出码错误。
相同 1080 射线扫描的三对实验中，C++ 进程 CPU 减少 88.89%、采样 RSS 减少
59.34%、P95 输入至输出可见延迟减少 37.42%；每实现共检查 15,000 帧。

此前导航/轨迹 CTest 在优化构建下关闭了 assert；旧的通过计数不足以作为断言证据。
本轮显式开启断言，按 Python oracle 修正旧测试的若干错误期望（不改运行算法、不放宽
容差），在全新无 pybind 构建中导航 2/2、轨迹 1/1、轮控 1/1 通过。两包清洁安装不包含
Python 文件；四个直接 C++ 节点的动态依赖和导出符号没有 Python/pybind。

三对、每实现各 30,000 次请求的隔离包络实验：中位 CPU 时间减少 85.14%，采样 RSS
减少 57.60%，请求至包络可见的 P95 延迟减少 47.98%；原始样本及源码/二进制 hash
见报告。它只量化受控 ROS 服务/消息负载，不代表 SDK、Nav2/Gazebo 或整机收益。

本轮启动了独立 domain 的受控 ROS 节点；没有启动完整 ROS/Gazebo 栈或真机。全向轮
pybind 已移除，轨迹、导航、几何三处绑定仍有 Python 消费者，默认路径仍待后续验收。

## 下一阶段的进入条件

上述阶段 1–3 的纯计算 slice 和 `ChassisBridgeCore` 离线状态机实验已经通过。
当前 C++ 核心仍是 opt-in 候选；进入 ROS 集成 A/B 前，还必须在独立 ROS domain
录制同一消息序列，比较 `/astribot/bridge/status`、目标写入次数、停车时刻、时钟
回退和重启恢复。通过后才有资格评审默认切换，不能仅凭 fake SDK 性能数字切换。

禁止用仿真成功替代这些差分检查；仿真只用于之后的 ROS 时序和端到端验收。也禁止
在迁移中放宽 leash、扫描时效、时钟回退、取消或停车门槛。

## 全部迁移的实验路线

后续迁移按下面顺序进行，每一步都保留旧实现作为 oracle，至少有一个可重复的
离线实验和一个 ROS 集成实验。只有同一阶段的离线实验、故障注入和集成实验都通过，
才允许切换默认实现。

| 阶段 | 迁移对象 | 离线实验 | 集成实验 | 不可退化的判据 |
| --- | --- | --- | --- | --- |
| 1 | 数学 kernel（本阶段） | 10,000 组同输入差分、非有限值、步长和坐标边界 | 仅加载 Python 兼容壳，不改变节点 | 数值、异常类型和输出维度一致 |
| 2 | `PoseFrameIntegrator`、`SdkPoseHistory`（当前已完成） | 递增/重复/乱序时间戳，跨 2π 连续 theta，换帧和 re-anchor 回放 | fake SDK + fake clock 的逐拍回放 | anchor、integral、target、历史插值和换向语义一致 |
| 3 | `measure_tick_dt`、`chassis_feedback` 安全数学、`ChassisBridgeCore`、只读 `ChassisOdomSource`（离线 C++ 候选已完成） | 时间首拍/回退/钳位、leash/跳变/漂移 10,000 组差分；odom 的 body/world、跳变历史/累计行程；enable/disable、指令超时、scan stale、pose stale、SDK 读写失败、leash、reset_leash 状态机回放 | 当前仍是 fake-port；默认切换前比较隔离 ROS 节点的 `/astribot/bridge/status`、odom/TF 样本、写入目标和停止时刻 | 数学误差与原因分支一致；状态序列、事件码、停车时刻、写入次数、odom 采样和目标逐拍一致；不降低任何门槛 |
| 4 | `arm_traj_math`、`gripper_math`（纯计算 slice 已完成）；`gripper_core`、`ArmTrajExecutor`、`WaypointDispatcher` 已有 native 候选；`arm_speed_limiter_cpp` 直接节点候选已完成 | 轨迹插值、越限、取消、settling、保持、夹爪极性和阻塞超时的 golden replay；夹爪再加并发/力限/读回故障注入；waypoint 丢点/限位/SDK 故障；限速节点的水平伸展/迟滞、关节回退、TF 缺失/陈旧和定时重发差分 | fake SDK 250 Hz action 回放；限速节点做独立 ROS domain JointState/TF A/B，再做真实后端只读/静止验证 | 关节顺序、限位、取消终态、夹爪换算、错误码、SpeedLimit 消息、TF 故障时保守限速和恢复时刻一致 |
| 4a | `omni_effort_drive_node._WheelLoop`、四轮逆运动学与 `omni_effort_drive_cpp` 直接节点候选 | 10,000 组 PID/饱和/dt/reset 与逆运动学差分回归；消息级回放覆盖 cmd/joint 超时停车、四轮符号和发布长度 | 当前完成独立 ROS domain 节点回放；默认切换前需在独立 ros2_control/Gazebo 域验证 command interface、watchdog、反馈断流和单轮符号 | effort 限幅、抗积分饱和、反馈时效、目标和错误序列一致；不改变控制器所有权 |
| 4b | legacy `envelope_coordinator` 与 `envelope_coordinator_cpp` 直接节点候选 | 双 footprint/TF/时间确认、非法缩小、运动中服务拒绝、epoch/租约和 fixed_v2 拒绝差分 | 独立 ROS domain 的 clock、odom、footprint 和服务 A/B；默认切换前仍需 Nav2 costmap 闭环回放 | 服务要求新鲜静止 odom；新 epoch 必须重新双确认；首次确认要求已知静止样本；几何/限速不得放宽 |
| 5 | 导航恒速预测、矩形连续 sweep、风险路径前处理、scan occupancy、姿态监控、control-time watchdog 与保护 slice（已完成）；多边形 sweep、`risk` 裁决和 ROS protection 时序保留 Python | `body_pose/stopping_horizon/sampling_margin/footprint_axes`、矩形 `clearance_many`/`motion_clearance`、路径投影/剩余路径、占用 cell/角度盒、姿态超限/退化窗口、仿真暂停/回退、`CommandRestriction`、扫描有效性和 clearing endpoint 的随机差分、停止/限速/恢复/非有限故障注入；后续覆盖 `fusion/risk` 的整段回放 | 当前只做无 ROS 加载验证；后续录制 ROS bag 重放，比较 `/cmd_vel`、限制原因和保护状态 | 小核数值与异常路径一致；完整防护迁移前任何原本拒绝的输入仍拒绝，安全限制只能相同或更保守 |
| 6 | `cmd_vel_body_to_world` 直接 C++ 节点与 `planning_session` 状态核（离线 C++ 候选已完成）；其余 ROS 节点和任务编排仍待迁移 | cmd_vel 核心的四元数/旋转/超时/姿态止损故障注入，加上规划请求预算、挂起复用、响应 lease、时钟回退和按目标重置差分；其余消息序列、取消竞争、重启恢复做模型检查 | `cmd_vel_node_impl:=cpp` 在独立域仿真 A/B，再做硬件只读和静止控制验收；逐步删除对应 Python/pybind 适配 | 请求所有权、topic/action/service 接口、时间戳、取消语义、停车时刻和控制权不变 |

每个集成实验都必须记录源码版本、配置摘要、ROS domain、消息类型 hash、时钟源、
输入包/录制文件、输出事件序列和判定脚本版本。A/B 实验不得同时启动两套底盘桥接，
不得在共享仿真中清理或重启其他任务拥有的进程。实机阶段只在已有安全闸门、限速和
停机机制下进行，不使用“放宽阈值后通过”作为迁移证据。
