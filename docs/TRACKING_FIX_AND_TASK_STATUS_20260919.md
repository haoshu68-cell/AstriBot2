# 任务状态与关节跟踪问题修复

## 当前状态

沿用 RVIZ_P0_P5_INTERACTION_DELIVERY_20260919.md 的交付顺序；旧文档是阶段快照，本页汇总最新实现和验收差距。仅仿真，不启动真机。

| 阶段 | 已有实现/证据 | 剩余交付门槛 |
|---|---|---|
| P0 | 统一网关、租约/幂等、状态错误码、假后端与回归 | 持续契约回归；后续接口版本化 |
| P1 | RViz 导航/循环、探索会话、录制、参数证据与只读回放；既有假后端/Qt测试记录 | 探索完成/取消存图、循环取消、回放组合的真实仿真验收；资源/延迟预算 |
| P2 | 地图与工位版本、事务持久化、Voxel激活适配 | 双地图重定位、人工换层中断/恢复的仿真验收 |
| P3 | 规划预览、执行事务核心、MTC仿真抓放、C++取消插件；活动收臂取消通过 | 限位余量与IK分支修复已有一次完整通过，仍需重复及取消回归；统一C++执行适配仍未完全替代旧Python执行器；夹持力/滑落不属于运动学附着证据 |
| P4 | 上位机派发单房间搬运，有完整成功、带载取消/租约丢失记录 | 稳定重复闭环；队列/WCS幂等投递、持久化检查点与事实恢复 |
| P5 | 设计与异常矩阵 | 动态工位、携物跨楼层、边界载荷与故障验收；真机阶段暂不开放 |

## 本轮定位

旧r11失败包显示关节3持续停在3.1 rad上限，但未录制控制器期望状态，不能单凭该包确定误差来源。本轮r13在相同缩放0.03、仿真轮位保持开启/增益10下复现：`MANIPULATION_TRACKING_ERROR:astribot_arm_left_joint_3`，ATTACHED保留、stop_error空。

正常关闭录包后读取 `/arm_left_controller/state` 和 `/transport/execution_guard/status`：故障ROS时间121.382 s，实际3.10000 rad，期望3.04885 rad，误差0.05114546 rad；同期底盘平移约1.75e-6 m、旋转约2.19e-6 rad。前阶段期望关节位置已到3.1 rad硬上限；随后反向期望连续下降，实际保持上限，排除了“仅底盘漂移”及“只是误报跟踪超限”的解释。

失败证据：`/tmp/astribot-tracking-fix/normal_r13.json`，`session_r13/task_1789821943173762017`，正常关闭incident `incidents_r13/incident_1789821925409287041_2569621`。公开上游 [gz_ros2_control Humble实现](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/humble/gz_ros2_control/src/gz_system.cpp) 将位置误差换成速度指令；该说明不等于对本机物理限位卡滞机制完成证明。现阶段明确的是规划选择硬限位分支及实际反向响应失败，底层限位动力学不冒充已修复。

## 修复范围

C++ MTC规划增加默认0.1 rad关节限位余量：每个有界左臂关节都参与路径约束，初始状态越界拒绝，最终输出路点二次检查；不改URDF/硬件限位，不放宽0.05 rad跟踪或20 mm底盘漂移保护。参数只允许启动时配置，运行中变更拒绝，统一录包增加该参数。下一步以完整搬运重复用例及运动取消检验该策略；单测不能代替实跑。

r14第一版直接将全关节路径约束和位姿目标交给PipelinePlanner，PREGRASP出现GOAL_STATE_INVALID；日志显示关节约束满足而末端位姿不满足，当前OMPL选用了关节采样器。没有执行抓取。随后改为抓取/放置共同使用ComputeIK约束枚举，再Connect连接关节目标，Cartesian后续阶段仍保留约束。r14保存正常关闭录包，不能计入修复成功。

边界测试 `execution_guard_boundaries` 和 `joint_planning_margin` 共2项通过；运行服务尝试修改余量0.1→0.05返回successful=false、PLANNING_PARAMETERS_REQUIRE_RESTART。构建与测试日志在 `/tmp/astribot-tracking-fix`。

### r15：修复后完整搬运通过

`normal_r15.json`：363.013 s墙钟，SUCCEEDED/PLACED/attachment为空；run `session_r15/task_1789822634624227773`。仿真实时率曾实测0.2612，因此该耗时不与旧轮140 s直接比较性能。两个导航目标、放置、解除附着和收臂均完成。

旧r13完整计划文件证实PREGRASP和GRASP_APPROACH的关节3目标均为3.1 rad，LIFT结束为3.082862 rad；r15新计划预抓取约−0.450963 rad，接近约−0.451750 rad，确实选择了不同且远离硬限位的分支。

正常停止incident `incidents_r15/incident_1789822620484713522_2818624` 后读取3655条active执行保护样本：最大关节跟踪误差0.037004 rad，最大底盘平移0.001205 m，仅出现等待证据/EXECUTION_WITHIN_BOUNDS。参数快照读回余量0.1、速度/加速度缩放0.03，仍属于observed而非硬件生效认证。

只读规划回归：复用旧失败请求，将起始关节3设为3.1 rad，服务返回ABORTED、MTC_START_OUTSIDE_PLANNING_MARGIN:astribot_arm_left_joint_3、stages为空，没有下发执行动作。日志 `hard_stop_admission_r15.txt`；首次CLI输入需将ROS byte字段从JSON字符转换为字节数组后才通过消息转换，转换失败不是服务端验收结果。阶段顺序/单次消费的4项PlanGuard回归通过。r15自有48个进程按PID/启动时间清理，remaining为空。

r16初次导航启动时behavior_server/get_state服务响应发送超时，lifecycle_manager明确中止激活；未派发搬运，不计动作通过。仅关闭本轮navigation launch及其22个后代，PID/启动时间核对remaining为空，再以r16b重启导航；世界、会话和记录器保留，未清理其他仿真。这是尚待独立治理的DDS生命周期启动可靠性问题，不能归因于关节余量修复。

r16b在velocity_smoother/change_state响应处再次超时，未进入全active；局部重启不能作为该问题已修复的证据。正常停止r16录包后完整关闭本轮隔离图，再重建进行下一用例。后续施工新增启动门槛：区分物理/时钟就绪与Nav2全生命周期就绪，未active禁止派发；DDS服务启动可靠性单列问题，不通过提高动作容差解决。

已在C++ transport_session落实启动门槛：异步查询Nav2生命周期管理器is_active服务，最多一个待请求、1秒请求超时、2秒确认有效期；不阻塞取消/状态回调。丢失/超时/失败确认禁止新派发，过期响应按代次丢弃。未增加运行中机械臂控制权或硬件命令。r17在尚未启动Nav2时直接调用start，返回SIM.NAVIGATION_NOT_READY，会话目录仅runtime.lock，没有run目录或搬运子任务。此门槛修复“导航未就绪仍可能先开始抓取”的准入缺口；不宣称修复DDS服务超时本身。

### 安装一致性与准入回归

r17未启动导航时，启动请求被SIM.NAVIGATION_NOT_READY拒绝；导航激活后准入通过，但子进程因缺少新加入的C++ `_geometry_native` 模块退出，尚未执行动作。隔离安装目录与并行更新中的源码不一致，已补构建astribot_s1_robot_geometry，导入验证成功，SourceInbox/PlanGuard共8项测试通过。此依赖模块属于已有并行开发，本轮仅同步构建，不计为新增实现。

r18重新启动后8个控制器active、Nav2全部active，启动被SIM.GEOMETRY_NOT_READY: HEIGHT_FILTER_CONFIGURATION_UNAVAILABLE拒绝，无动作下发。检查确认旧安装的感知launch仍使用/map_scan_filtered，而当前几何模块要求新版完整高度投影配置。正常关闭r18录包并清理自有48进程后，同步构建导航/感知依赖；r19启动预检查发现上层astribot_s1_perception仍引用旧安装，未派发任务即关闭隔离图，继续补齐perception/gazebo_bringup的launch依赖；保留新版安全门禁，不通过关闭检查使任务通过。r17/r18均保留失败报告，不能计为完整搬运通过。

r20已确认实际投影源为/map_scan，控制器/传感器/导航全部就绪，几何准入通过。随后恢复门禁发现r17构造函数导入失败前已写入unconfirmed_executor租约，正确阻止新执行；不是本轮运动失败。核对r17 PID不存在、原世界已销毁、异常发生在TransportTask.run之前，保存原租约与核销依据到reconciled_r17_lease.json后，仅核销自有domain213记录，启动新会话r20b；未增加自动清锁或跳过恢复的源码逻辑。

### r20b：关节阶段通过，导航覆盖阻塞，取消收尾

完成视觉定位、抓取、ATTACHED、LIFT、TRANSPORT_POSTURE及第一个导航点（仿真位置误差0.000394 m）。第二段长时间停走，现场constraint明确hold=true、max_linear_speed=0、reason=REQUIRED_COVERAGE_UNAVAILABLE。主动通过本轮simulation_transport/cancel终止，任务331.200 s后进入恢复门禁；normal_r20b.json不是SUCCEEDED，不能计完整闭环或重复可靠性通过，也不是自动验收器主动取消用例（报告cancel_requested=false，实际服务取消另存cancel_r20b.txt）。

账本最终CANCELED/USER_CANCEL、stop_error为空，载荷版本3/ATTACHED/左臂TCP链接保留。正常关闭incidents_r20/incident_1789824402100928512_3503871后只读解析：1554条active执行保护样本，最大关节误差0.038727 rad，最大底盘漂移0.00001445 m，仅出现等待证据或EXECUTION_WITHIN_BOUNDS；末尾odom线/角速度0、cmd_vel三分量0、最大关节速度3.54e-11 rad/s。这些末尾样本支持已停止，不冒充完整独立制动时间/持续窗口验收。

覆盖故障原因尚未细分：取消后的单次sensor_health为VALID且360度覆盖，不能反推停走期间同样有效。现有包记录到12444条REQUIRED_COVERAGE_UNAVAILABLE约束，但此前没有sensor_health完整时间序列；已将/navigation/sensor_health加入统一录制配置，下轮必须用捕获时间/有效期和运动方向关联验证，不删除覆盖门禁。数据见metrics_r20b.json、constraint_r20b.txt、sensor_health_after_cancel_r20b.txt。

本轮边界/接口测试：MTC CTest 2项、仿真核心gtest 8项、SourceInbox/PlanGuard pytest 8项、感知附着过滤CTest 2项均通过，合计20项（不重复计入此前同组测试）。配置YAML解析和git diff --check通过；sensor_health新增白名单仅做配置验证，待下轮实际录制确认。

## 后续施工顺序

1. 优先固定构建依赖闭包和运行版本；定位REQUIRED_COVERAGE_UNAVAILABLE的时效/方向覆盖来源及DDS生命周期启动超时。保留限位与覆盖保护，完成同配置完整搬运重复验证、修复后的运动中取消回归。
2. 补齐P1/P2仿真验收：探索无目标分类与完成存图、取消存图/失败重试、循环多点取消、只读回放隔离、双地图事务。
3. 执行异常：时钟暂停/回退、失联、节点重启、动作成功但账本未提交、载荷未知；确认停稳与恢复门禁，不盲目重发。
4. P4：在稳定执行链上接队列、WCS去重/状态重投递与检查点恢复。
5. P5：动态工位与人工跨层仿真，再制定现场准入。未完成前保持真机边界。

所有用例保留成功与失败，区分命令ACK、动作终态、独立停稳和载荷事实；不以反复重试直到成功替代可靠性验收。

收尾：按PID/启动时间关闭本轮49个自有进程，remaining为空；最终domain213/专用partition非shell进程为0。count.sh仍显示43个其他实验栈进程，未清理或干预；没有执行全局pkill、共享内存清理或daemon stop。详细证据目录/tmp/astribot-tracking-fix为临时实验数据，尚非持久归档。

最新续作：[覆盖时效与安装一致性修复](COVERAGE_TIMING_REPAIR_20260919.md)。r21两段导航通过，预放置joint6跟踪超限，完整任务未通过；剩余短时覆盖失效与joint6执行偏差仍须验收。

后续定位和修复见[夹爪物理耦合与接触诊断](GRIPPER_MIMIC_AND_CONTACT_REPAIR_20260919.md)：Gazebo从动关节显式注册后r23完整通过，最大关节误差0.026523 rad、没有接触对或决策覆盖失败；DDS启动可靠性继续单列，不因搬运成功而关闭。

最新验收：r24第二次完整搬运通过，最大关节误差0.036488 rad；r25真实Voxel取消存图通过，但探索固定包络的上层保持所有者尚未接通。当前优先补此接入，再做运动探索、自然完成存图及双地图实跑，详见[探索与地图验收续作](EXPLORATION_MAPPING_ACCEPTANCE_20260919.md)。单轮启动失败与任务结束后的扫描过期记录均保留。
