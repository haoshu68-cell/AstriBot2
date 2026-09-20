# 取消执行与静止保持排查（2026-09-19）

## 真机零速度的含义

当前源码 chassis_cmd_bridge_node 按 chassis_bridge.yaml 的 freq=250 Hz 调度 inner_tick。写权限有效且状态允许时，chassis_bridge_core 仍调用 SDK set_joints_position；输入速度为零后，位置目标保持不变。零速边沿撤除位置提前量后保持目标，不跟随定位噪声重设目标。因此人工推动会形成位置误差和伺服反作用力，符合“hold、推行阻力大”的描述。

启用 final_protection 链路时，其 steady timer 周期为 0.02 s，tick 每次发布最终 Twist，因此静止时仍按配置 50 Hz 输出零速度。

这不等于 ROS 每个节点都无限发布零 Twist。当前 cmd_vel_body_to_world_node 对上游超时补发一次零后清除接收标志；最终保护、其他上游和 SDK 位置环是不同层。停止 ROS 发布不意味着 SDK 松闸/释放伺服，也不意味着人工推行模式已启用。本轮没有连接、部署或操作真机；250 Hz 为仓库配置，未宣称真机实测频率。人工推行需要明确 SDK 的制动/使能接口与退出重定位流程，不能仅删除零速度消息。

## 机械臂取消

Humble 的上游 ExecuteTrajectory capability 把阻塞的 executePathCallback 放在 MutuallyExclusive action callback group，cancel 回调未调用 preemptExecuteTrajectoryCallback；后者虽定义了 stopExecution，却未被该 cancel lambda 调用。参考 [MoveIt Humble 原始实现](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)。这与旧实验的取消响应等待超时、轨迹最终 SUCCEEDED 一致。

新增 astribot_s1_manipulation/CancellableExecution C++ capability：

- 每次只接收一个 ExecuteTrajectory 目标，执行等待放入可回收工作线程。
- 取消回调只设置当前目标的取消意图；执行线程通过原 MoveIt TrajectoryExecutionManager 停止并等待结束，再返回 canceled/PREEMPTED。
- 不直接取消全部控制器目标，不建立第二套轮端或机械臂控制权。
- transport_skills 仿真入口加载新 capability，并明确禁用 move_group/MoveGroupExecuteTrajectoryAction；通用 move_group launch 的 extra/disable 参数默认均为空，保持其他入口默认行为。

实际日志已确认仅加载新 capability；r2 的跟踪保护触发后，MoveIt 执行管理器返回 PREEMPTED，stop_error 为空。不过该任务在预抓取就因 MANIPULATION_TRACKING_ERROR 提前终止，不算收臂取消用例通过。

## 起步碰撞拒绝

重新按 recorder events 的 UTC 与 ROS 时间对应关系读取旧失败包（不能用 UTC 直接查询使用 ROS 时间的 bag timestamp）：

- 任务开始附近：ROS 21.338 s，odom x≈-0.000001 m、y≈-0.000004 m。
- 收臂前：ROS 38.767 s，x=-0.000978 m、y=-0.000294 m。
- 导航前：ROS 45.612 s，x=-0.239204 m、y=0.382147 m、yaw=-0.246230 rad。
- 该包 /cmd_vel 3588 条，非零条数为 0；收臂期间部分车轮出现实际转动。

底盘在导航前已漂移约 0.451 m、转约 14.1°，因此首段载荷轮廓与工位附近障碍相交的拒绝有实际依据。本轮保留起点碰撞和整段扫掠校验，不缩小 footprint，不跳过 segment=0。产生外力的具体机械接触仍需与跟踪误差、碰撞场景和动力学联合分析，不能只归因于规划器。

仿真 omni_effort_drive 原有零轮速 PI 并不等于真机位置保持。增加可选 idle_position_hold（默认 false）与 idle_position_kp（默认 3.0 Nm/rad）：车轮速度均小于 0.05 rad/s、位置完整有限且反馈新鲜时捕获轮位，随后叠加有界位置恢复力矩。非零目标或反馈失效清除旧参考，下一次停止重新捕获，继续遵守原 15 Nm 限幅。没有锁死 Gazebo 模型或直接写物体位姿。五项回归覆盖力矩方向/限幅、运动释放、陈旧/缺失反馈、默认关闭、新停止参考和不完整速度反馈；均通过。

该功能仍只在隔离验收中通过参数服务显式开启；未修改标准驱动默认值或任何真机桥接逻辑。

## 试验边界

本轮 domain=213、独立 Gazebo partition；保留其他会话。r3 出现共享内存端口错误及地图心跳丢失，几何状态不完整，派发被拒绝，run 为空。此轮不是功能验收通过或动作执行失败。后续隔离会话使用独立 Fast DDS UDP profile，未清理全局共享内存。

证据目录 `/tmp/astribot-cancel-hold`，含逐轮 launch 日志、参数服务结果、录包与验收 JSON。仿真位置保持、低速执行及收臂取消后续实跑结果追加于下文；纯单测不替代完整搬运与故障实跑。


### r4：阶段取消通过

报告 `/tmp/astribot-cancel-hold/cancel_r4.json`，run `session_r4/task_1789817215268199172`。在 TRANSPORT_POSTURE 阶段注入取消，最终 CANCELED/USER_CANCEL、stop_error 空，载荷 ATTACHED/版本3/链接不变。底盘和关节停稳、最终零命令、几何和 Gazebo 附着、恢复门禁全部通过。取消到独立验收完成 2.081 s，包含终态与观测保持，不是纯制动时间。此用例在阶段刚进入时取消，不单独证明已经开始的收臂轨迹被中断，继续追加非零关节速度触发用例。

录包 `/tmp/astribot-cancel-hold/incidents_r4/incident_1789817213301325216_1209167` 已正常停止；参数快照确认新 capability 已启用、旧 capability 已禁用、MTC 速度/加速度缩放均 0.03、idle_position_hold=true、增益3.0、控制周期0.01 s、力矩限幅15 Nm。仍保留 observed/effective_confirmed 的区别。


### r5 与更严格的运动触发

r5 在左臂速度 0.021348 rad/s 时发出取消，验收通过、确认时间 2.067 s。但前一 MTC 阶段允许低于 0.03 rad/s 的停稳残余，且该轮没有新的 PREEMPTED 执行记录，因此仅记为带附着载荷的阶段取消，不能直接记作已中断活动收臂轨迹。README 的复现命令已将 minimum_joint_speed_radps 提高到 0.05，后续同时核对执行插件的 STARTED/CANCEL_ACCEPTED/STOP_REQUESTED 和 MoveIt PREEMPTED 记录。

C++ capability 在执行线程异常时锁存 faulted，拒绝后续目标直到进程级处置；异常清理失败单独输出 EXECUTION_STOP_UNCONFIRMED。继续保留 ROS cancel ACK 与执行终态/独立停稳证据的区别。


### r6：活动收臂轨迹取消通过

`/tmp/astribot-cancel-hold/cancel_r6.json` passed=true，run `session_r6/task_1789817724322532910`。在 TRANSPORT_POSTURE 且左臂实测最大关节速度 0.052621 rad/s（阈值0.05，大于上一动作停稳允许值0.03）时发出取消。

MoveIt 日志：1789817772.579781 执行开始；1789817772.953968 取消受理；1789817772.955547 请求 TEM 停止；1789817772.987248 TEM 返回 PREEMPTED。受理到 TEM 终态约33 ms，是软件执行管理器时延，不是物理制动时间。取消请求后2.778 s独立停稳验收完成；账本CANCELED/USER_CANCEL、stop_error空，载荷ATTACHED/版本3/链接不变。最终底盘线速度和角速度均为0，最大关节速度1.33e-8 rad/s，零命令、几何和Gazebo附着成立，恢复门禁有效。完整任务墙钟51.401 s。该轮明确覆盖执行中的收臂取消。


r7 启动时 gz_ros2_control 卡在机器人描述服务响应，时钟未推进，probe ready=false，未派发任务。r8 重启前旧 Gazebo 包装进程未完全退出，出现仓库物理步进但机器人未生成、全局时钟缺失；该轮受启动残留干扰，不作为算法/控制验证证据。随后仅对保存 PID/启动时间确认属于本轮的残留发信号，核对日志目录下无存活进程后才重建 r9。未清理其他 domain 或全局共享内存。


### r9：漂移保护提前中止，不能计作导航通过

正常任务 `session_r9/task_1789818266933197527` 在 TRANSPORT_POSTURE 被 MTC_BASE_MOVED_DURING_EXECUTION 中止，未进入导航，stop_error空、附着载荷保留。保护阈值仍为0.02 m/0.02 rad；未放宽。此次最大 odom 平面位移0.022497 m，录得轮端峰值力矩0.9478 Nm，远低于15 Nm限制。相比旧失败包0.451 m的观测量有下降，但包含低速执行、保持以及更早终止等多个变化，不能归为保持控制独立改善比例。

该轮 diagnostics_recorder 在尝试停止期间未确认成功，读取尚未关闭的SQLite后出现database is locked并退出。保留原始包和异常日志，不将其标为正常关闭的完整incident；后续只在明确停止成功后做数据库检查。这是诊断采集的已知限制，不作为动作验收通过证据。

### r10：完整搬运通过

新建世界 probe ready=true，8 控制器 active；服务确认 idle_position_hold=true、idle_position_kp=10.0，其余仍为 MTC 速度/加速度缩放0.03、轮端15 Nm限幅、原有20 mm/0.02 rad漂移保护。

报告 `/tmp/astribot-cancel-hold/normal_r10.json`，run `session_r10/task_1789818756187870732`：140.308 s，SUCCEEDED / PLACED / attachment为空，子进程正常退出、motion_blocked=false。抓取、附着、带载收臂、两个导航目标、放置、收臂全部完成。两个目标的 Nav2 位姿平面误差分别0.330 mm和0.724 mm，属于本轮仿真位姿验收，不是物理精度认证。

正常停止录包成功后只读检查 `incidents_r10/incident_1789818739354904286_1885294`：clock 161184条、odom 8059条、轮端力矩16117条、实际constraint 11968条、geometry_state 1568条。快照读回保持开启/增益10、新capability、MTC缩放0.03，继续标为observed、effective_confirmed=false。采集开始至首次进入TRANSPORT的odom最大平面位移约1.070 mm；TRANSPORT_POSTURE到首个ENVELOPE_CONFIRM区间522个odom样本，位移约0.00786 mm，轮端峰值0.0495 Nm。时窗通过events的UTC/ROS近邻时间换算，不代表独立标定精度。

这是一轮配置组合下的成功闭环。规划结果和接触过程可能不同，不能据r9/r10直接宣称增益变化的独立因果或重复可靠性已通过。标准仿真保持默认值仍关闭；真机未改。r10按保存PID/启动时间清理48个自有进程，remaining为空，再开始新世界异常验证。

### r11：导航取消前置动作失败

相同保持/缩放配置，新世界 ready=true。`nav_cancel_r11.json` 在47.844 s报告 SIM.TERMINATED_BEFORE_INJECTION，fault_injected=false；run `session_r11/task_1789819095826786157` 的 TRANSPORT_POSTURE 出现 MANIPULATION_TRACKING_ERROR:astribot_arm_left_joint_3。停止传播成功、stop_error空，载荷ATTACHED、恢复门禁有效；未进入带载导航，不能计入导航取消通过率。保留正常关闭录包 `incidents_r11/incident_1789819063334530470_1991192`。

增加 recorder 的 `/arm_left_controller/state`、`/transport/execution_guard/status` 和关节跟踪保护阈值参数白名单，用于后续区分期望轨迹、实际响应和保护触发，不放宽现有保护阈值。r11清理49个自有进程，remaining为空。

### r12：带载实际行走中主动取消通过

新世界及相同保持/缩放配置，`nav_cancel_r12.json` passed=true、fault_injected=true。在TRANSPORT、载荷ATTACHED/版本3、实测平面速度0.020111 m/s时，通过上位机网关主动取消。最终CANCELED/USER_CANCEL、stop_error空、原载荷和附着链接保留；底盘线/角速度0，最大关节速度1.69e-11 rad/s，零命令、几何/Gazebo附着、恢复门禁全部成立。取消到独立验收完成2.725 s，包含终态等待和观测窗口，不是纯制动时间。任务总墙钟45.988 s。

run `session_r12/task_1789819314555545852`；正常停止录包 `incidents_r12/incident_1789819291731227829_2073564`。只读实查clock 61769条、左臂控制器state 2969条、execution_guard/status 3874条、轮端力矩6178条。1028条active保护样本只出现等待证据或EXECUTION_WITHIN_BOUNDS；记录的最大关节跟踪误差0.028344 rad，低于原0.05 rad阈值。此轮没有修改跟踪或漂移保护。

当前结论：活动收臂取消、完整搬运、带载行走主动取消各有通过证据；r11同配置关节跟踪失败仍然存在，不能声称重复可靠性已通过。保持功能默认关闭，只有隔离验收显式启用。后续需用新增控制器/保护回放覆盖更多规划结果，定位偶发跟踪误差；不据单次成功推进真机。

收尾：r12按保存PID/启动时间关闭48个自有进程，remaining为空；最终自有domain/partition存活进程0，count.sh匹配栈进程也为0，未执行全局清理。5项保持回归与8项仿真核心gtest通过，新增C++插件及验收工具构建成功，修改文件语法/XML/YAML与git diff --check通过。证据均保留于 `/tmp/astribot-cancel-hold`；该目录是实验数据，不等于持久归档。

后续限位分支修复、安装一致性检查和r20b覆盖门禁阻塞见 [任务状态与修复记录](TRACKING_FIX_AND_TASK_STATUS_20260919.md)。原r11问题的规划侧修复已有r15完整成功；不据此宣称所有集成场景均通过。
