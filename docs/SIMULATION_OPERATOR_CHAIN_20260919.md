# 上位机功能链路：隔离仿真开发与验收记录

日期：2026-09-19。当前顺序：先完成仿真功能链路和多场景验证，再考虑真机。本记录不宣称 P3–P5 已完成。

最新结果：修复入口配置和时钟先后差后，`operator_clockfix` 已完成一轮上位机驱动的抓取—带载两段导航—放置—收臂闭环。另有抓取前主动取消通过。下面保留失败实验和修复过程，尚未通过全部多场景矩阵。

## 本轮实现

新增 `astribot_sim_validation` 功能包，使用 C++ 实现就绪探测、搬运会话管理和上位机验收客户端。启动采用包内 launch，未增加 Python 采集脚本。已有 Python 搬运执行器继续负责 MoveIt/MTC、资源锁、控制器和载荷事务；前期 C++ ArmExecutionTransaction 尚未替换其内部执行事务。

调用链：RViz 工作站/验收客户端 → operator_backend 控制租约与命令 → simulation_transport 会话 → 既有 transport_task → 感知、抓取、附着确认、行走姿态、导航、放置。

- RViz 增加仿真搬运开始、取消及运行状态；运行或状态未知期间禁用冲突操作。
- 网关阻止搬运 pending/运行/恢复待处理时的新导航、循环导航、探索与换图；拒绝重复搬运启动。
- 会话绑定网关 boot_id 和非秘密 control_session，控制权变化、失联、仿真时钟停滞时请求取消自己的子进程组。
- 派发前持久化 runtime.json；保存独立场景和 ledger。退出码为零还必须同时满足 SUCCEEDED、PLACED、无 attachment，才能报告成功。
- 节点重启、失败及未知状态不自动重放，不自动松爪。新 output_root 仅适用于已重建世界的新实验，不能作为现场任务恢复手段。
- 修复网关 use_sim_time 未透传，以及 `/transport/status` 的 volatile 发布与 transient-local 订阅不兼容。
- 录包增加仿真会话、导航策略状态/观测及场景路径等关键参数；上位机验收客户端按新 run 关联结果，旧成功状态不能完成新任务。

## 隔离和证据边界

ROS domain 213，localhost only，IGN/GZ partition `astribot_operator_validation_213`。启动后核对实际进程环境；源码和依赖构建到 `/tmp/astribot-route-build`，未覆盖共享仿真的 install。地图目录单独配置为 `/tmp/astribot_validation_213/map_catalog`。

沿用 `ros2-stack-ops` 的分层判据。末轮 15 秒探测收到 clock 1368、joint_states 1088、odom 544、scan 116 个有效且时间推进的样本，8 个控制器均 active，报告 ready=true。这是数据与控制器就绪证据，不等同于完整任务成功。

夹持使用 Gazebo 运动学附着；不能证明夹持力、摩擦、滑落或物理接触安全。真机未参与此次验证。

## 实跑场景

| 场景 | 结果与证据 | 判断 |
|---|---|---|
| 原场景，直接执行器 | 抓取、附着确认、行走姿态完成；导航阻塞超时，ledger FAULT，物体 ATTACHED | 搬运闭环失败，载荷事实保留 |
| 原场景，上位机租约 → 会话 → 执行器 | 约 63.8 秒后同样导航失败；会话 RECOVERY_REQUIRED | 上位机提交及失败回传贯通；放置未验收 |
| 导航目标对齐栅格 Y=0.025 | 同样导航失败；全局路径起始仍有短折线 | 目标偏移未解决问题，不能作为修复 |
| 取放工位侧向增加 6 cm | PREGRASP: GOAL_STATE_INVALID，约 43.3 秒终止；物体 WORLD | 扩大净空导致该姿态抓取规划不可行，未进入导航 |
| 运行期间另一客户端申请控制 | CONTROL.BUSY，未派发第二个任务 | 单一控制者保护通过 |
| 启动残留导致两个网关发布者 | 验收客户端 SIM.BACKEND_UNAVAILABLE，未派发任务 | 多来源拒绝生效；随后仅清理本次实例并重新启动 |
| 故障后再次调用 start | SIM.RECOVERY_OR_TASK_PENDING | 未自动重试 |
| 运行时设置 use_sim_time=false | 参数修改被拒绝 | 仿真时间边界保持 |

原场景阻塞观测中，取货台边缘栅格中心为 `(0.075,0.625)`、`(0.125,0.625)`，当前 clearance 约 0.129 m，预测 conflict_time 为 0.1 s，非 immediate 碰撞。全局路径起始点约 `(-0.005,0.025)`，随后至 `(0.045,-0.025)`；预测使用短段切向朝向，控制器使用前瞻起步朝向。两者是否造成保守误阻塞仍需回放重现和几何验证，尚未证明根因。没有缩小 footprint、降低停止门槛或绕过保护。

## 证据位置

本机原始证据根目录 `/tmp/astribot-sim-completion`（临时目录，不是长期归档）：

- `readiness_clearance.json`：数据推进和控制器状态。
- `operator_nominal.json`、`operator_aligned.json`、`operator_clearance.json`：上位机会话结果。
- `operator_runs/task_1789801967508231266/ledger/state.json`：原场景完整账本。
- `operator_aligned/task_1789802472115009100/ledger/state.json`：目标对齐对照。
- `operator_clearance/task_1789802824882470473/ledger/state.json`：扩大净空对照。
- `observation_failure.json`：阻塞观测快照；不能代替整个任务时间序列。
- `incidents/`、`incidents_aligned/`、`incidents_clearance/`：录包与参数记录。
- `final_guard_build.log`、`final_guard_tests.log`：最后一次编译和回归。

源码使用仓库统一日志设施；本轮多终端启动输出另存 `world_*.log`、`navigation_*.log`、`session_*.log` 作为试验证据，不能据这些分离输出宣称一键统一日志启动已验收。

最终回归：operator_backend 35 项、operator_station 13 项、sim_validation 3 项，共 51 项，失败/错误/禁用均为 0。另通过 launch XML/Python 语法检查和 `git diff --check`。此计数不包含前一轮未重跑的地图/路线/探索用例。末轮录包正常停止，关闭本轮独立仿真并核查已记录 PID 与启动时间，未停止其他会话。

## 后续验收顺序

1. 从原场景录包重现起步阻塞，验证路径切向、起步决策与真实控制轨迹的关系；补几何回归后再改策略，保持碰撞与载荷边界。
2. 原场景完成取货 → 导航 → 放置 → 收臂；验证 ledger、载荷版本、最终姿态、控制权释放，不只看进程退出码。
3. 在新世界分别验证抓取前取消、附着后取消、导航中取消、租约丢失、进程重启、时钟暂停、控制器断连；取消受理和停稳完成分别判定。
4. 补齐探索开阔/窄道/孤立区/动态障碍、结束与取消保存地图，多点循环与取消，录包参数时间线和只读回放的联动实跑。
5. 两图切换及人工跨层事务、单房间搬运队列/WCS 去重和断点恢复，最后扩展动态工位与多故障组合。

每一阶段必须有成功与失败场景的可复查证据后再进入下一阶段；现有假后端和纯核心测试不能替代这些 Gazebo 验收。

## 继续施工：统一仿真几何模式与主动取消

后续检查发现本包原入口使用 legacy + p3，未接入仓库已有的 fixed_v2 几何链和经整段碰撞验证的直线路径处理。本轮仅调整隔离验收入口，未改变常规硬件/仿真启动默认值：

- 世界/感知、导航和搬运会话统一 fixed_v2。导航采用自动通道策略 p5；此处 p5 不是项目交付阶段 P5。
- 搬运会话显式传入 `--navigation-geometry-mode`，同时写入本次 scenario.json，避免执行器命令行默认 legacy 覆盖 JSON。
- C++ 派发前检查几何状态唯一发布者、接收时效、源时间/有效期、完整性和附着状态确认。缺失、未来、过期或不完整状态均拒绝，不先抓取再等失败。
- 验收客户端关联异步命令事件；下游拒绝启动报告 START_REJECTED，不再一直等任务终态。运行期间也持续检查状态发布者唯一性。
- 录包补充 clock、实际运动约束、整机几何、包络消费者确认、保持状态、路径质量及模式/高度参数。取消用例使用了该新版清单；此前 v2c 包没有新增的全部话题，不能回溯宣称已经录到。

| 独立实验 | 结果 |
|---|---|
| v2：P4 未提供人工通道文件 | 策略节点拒绝启动，任务在 ADMISSION 以 NAVIGATION_POLICY_HEARTBEAT_UNAVAILABLE 退出，未抓取 |
| v2b：导航已切 fixed_v2，感知仍旧模式 | 附着确认等待超时；几何原因 GEOMETRY_EXCEEDS_OBSERVED_HEIGHT_FILTER。账本 ATTACH_PENDING，保持恢复状态 |
| v2c：感知也切 fixed_v2 | 高度过滤回读 2.2 m，整机几何高 1.64142 m，GEOMETRY_CURRENT；抓取、附着、离台首目标通过。第二目标处 POLICY_LEASE_EXPIRED，账本 ATTACHED/FAULT |
| 主动取消：启动 5 秒后取消 | 6.178 秒获得终态，CANCELED/USER_CANCEL，WORLD、无 attachment、stop_error 为空；会话保持恢复门禁 |

v2c 首目标的 Nav2 位姿误差为 0.00097685 m、0.00064243 rad（约 0.037°），停稳 0.6 秒。仅为本轮仿真导航位姿证据，不代表真机定位精度。第二目标在约 2.6 mm 附近失败，不能与成功到位指标合并。

v2c 在仿真时间 130–153 秒的 230 个策略状态样本中，最大源间隔 0.137 秒，最大处理墙钟时间 0.15042 秒；状态消息不等同于控制器实际收到的约束。该轮有并发编译，是时序归因的干扰因素。新增 POLICY_LEASE_EVIDENCE 输出源年龄、墙钟年龄、租约、epoch 和 sequence，不放宽 0.3 秒保护门槛。

证据：`/tmp/astribot-sim-completion/operator_v2c.json`、`v2c_policy_timing.json`、`geometry_v2c_before.txt`、`height_v2c.txt`、`operator_cancel.json`。取消账本为 `operator_v2d/task_1789804435482018694/ledger/state.json`。本轮新增/修改的仿真 C++ 核心 4 项 gtest 通过，路径跟踪 3 项 CTest 通过；不将上一轮未重跑的其他包测试计入本轮。

### 租约失败复现与修复

无并发编译的 v2e 实验再次通过首目标，随后在第二目标附近失败。新增诊断直接记录：`source_age_s=-0.001000 wall_age_s=0.001037 lease_s=0.300000`，epoch=196376832556989、sequence=20066。这次是约束时间比控制器本地 ROS 时钟快 1 ms，既非接收超时也非 0.3 秒墙钟超时。此前笼统的 POLICY_LEASE_EXPIRED 被 ArrivalController 锁存，后续时钟追上也无法继续任务。

修复使用 C++ PolicyLease 的原子状态判定：未来约束仍不授予运动权限；在源时间提前量、接收墙钟年龄都不超过原租约时，只允许有界 POLICY_CLOCK_WAIT，并输出零速度、清除到位保持计时。等待总墙钟时间也受原租约限制，新消息不能无限延长；源时间追上后仍须原有 fresh 检查通过。真实过期、缺失、过大未来跳变继续失败退出，没有放宽运动许可的时间范围。

新增原生回归覆盖 1 ms 先后差、未来消息零速度限幅、时钟追上、源超时、连续未来消息不得延长等待、较大未来跳变。实际复测记录继续追加；不能把纯单测通过当成完整抓放成功。

### 修复后完整实跑结果

`operator_clockfix/task_1789805125623981020` 最终 SUCCEEDED；载荷版本 5、PLACED、attachment 为空，执行器正常退出，会话 motion_blocked=false，上位机客户端收到匹配本次 run 的成功结果。总墙钟时间 214.703 秒。流程包含 PLACE_APPROACH、DETACH_CONFIRM、STOW、EMPTY_PAYLOAD_HOLD 和 PLACEMENT_VERIFY。

| 验收点 | 实测 |
|---|---|
| 首导航目标 `(0.45,0,0)` | 导航位姿位置误差 0.986 mm，偏航误差 0.000366° |
| 第二导航目标 `(1.1,0,0)` | 导航位姿位置误差 1.093 mm，偏航误差 0.053064° |
| 放置验证 | 规划场景位置误差 1.185 mm；Gazebo 两次位置观测误差 1.020 mm |
| 载荷终态 | PLACED，附着解除，完成收臂 |
| 回归 | 仿真核心 4 项 gtest、路径跟踪 4 项 CTest 全部通过；编译及 diff 检查通过 |

上述导航数值为 map/Nav2 位姿来源，放置数值为仿真账本/Gazebo 来源；仍无夹持力、滑落与真实接触验证。这是一轮成功，不是重复可靠性或全部异常场景验收。

成功报告 `/tmp/astribot-sim-completion/operator_clockfix.json`；账本目录 `/tmp/astribot-sim-completion/operator_clockfix/task_1789805125623981020/ledger`；完整录包、参数快照和 session.log 位于 `/tmp/astribot-sim-completion/incidents_clockfix/incident_1789805123584291926_2364295`。停止录包服务返回成功，SQLite 中确认 clock、实际 constraint、geometry_state、envelope_v2 均有记录；参数快照中 planner 的模式为 fixed_v2。参数服务读回不等同于底层实际生效认证，保留 recorder 原有 effective_confirmed 标记。

本轮独立仿真已关闭，按保存的 PID/启动时间复核无本轮存活进程，未清理其他会话。下一步优先重复成功用例，再覆盖附着后取消、导航中取消、策略失联/时钟暂停，然后推进探索存图、循环导航和回放联合场景。已补验收客户端可配置墙钟等待上限 max_wait_sec（默认 900 秒），不会据等待超时把任务判为成功。


### 阶段触发异常验收（继续施工）

新增 C++ `transport_fault_acceptance`，通过上位机网关申请/续租和派发任务，按本次 run 的 ledger 精确触发异常，不使用缺少 run ID 的全局 transport/status 作为任务关联依据。支持附着后主动取消、带载移动时停止续租；不新增采集脚本，不直接发布速度。断言覆盖任务终态、载荷版本与附着链接、实际底盘/关节停稳、零速度输出、完整几何、Gazebo 附着误差、唯一数据来源与控制权丢失。冻结/重复数据不能伪装停稳。具体阈值和配置写入验收 JSON；录包增加载荷附着状态和验收节点关键参数白名单。

前两轮 TRANSPORT_POSTURE 取消，账本均为 CANCELED、stop_error 空、载荷 ATTACHED。但验收端仍超时。第二轮增加逐项诊断后，发现每次同步 renew 处理新消息，循环却沿用服务调用前的 steady clock 值，导致 receipt 比判断时刻更新，周期性清零连续停稳判定。已在服务调用后刷新判断时间，保留原有阈值。新增单测覆盖服务处理期间的新观测与旧判断时间，仿真功能包共 8 项 gtest 全部通过。

第三轮 `fault_attach3_result.json` 揭示另一项执行层失败：在 TRANSPORT_POSTURE 取消，账本最终 FAULT/WAIT_TIMEOUT，载荷保持 ATTACHED、stop_error 空、会话恢复门禁有效。对应 MoveIt 执行日志仍记录该轨迹最终 SUCCEEDED，尚不能证明取消及时中断轨迹。现有 action 异常处理调用 cancel_active，内部取消响应等待限时 10 s，抛出的异常可能替换原始 Canceled；需继续验证取消传播与执行器终态，不能只修改错误码或放宽验收来判为通过。此次未修改既有 Python 执行器或底层控制权限。

失败证据保留在 `/tmp/astribot-sim-completion/fault_attach_result.json`、`fault_attach2_result.json`、`fault_attach3_result.json`，以及各自 incidents 和 ledger 目录。前两轮录包尚无新增的载荷状态话题，第三轮有；验收参数白名单从后续导航取消用例开始应用，第三轮关键阈值以报告为准。


带载导航主动取消用例 `operator_fault_navcancel/task_1789815775407709932` 在注入前终止：规划器报 `PATH_QUALITY_UNSAFE: candidate footprint touches obstacle, unknown or map boundary; segment=0`，账本 reason 为 `ACTION_FAILED:6:navigation_policy=INPUT_UNAVAILABLE`，附着载荷保留、恢复门禁生效。fault_injected=false，不能算取消成功或取消失败，而是前置运动未成立。本次包络半长 0.375049 m、半宽 0.367950 m；与先前正常闭环的一轮成功不能合并成稳定性已通过，需复核不同规划结果下的行走姿态、载荷包络与离台路径净空，保留碰撞校验。

该轮录包 `/tmp/astribot-sim-completion/incidents_fault_navcancel/incident_1789815774153523471_641826` 正常停止后检查 SQLite：clock 50744 条、实际 constraint 3586 条、geometry_state 548 条、载荷附着状态 2426 条；验收节点参数快照 3 次，记录 cancel / TRANSPORT / 0.02 m/s 等设置。参数快照是 observed，effective_confirmed=false，不能当作硬件实际生效证明。


### 带载行走中租约丢失通过

`fault_lease_result.json`：在本次 TRANSPORT 且实测速度 0.020289 m/s 时停止续租，未主动调用取消/释放服务。上位机控制会话失效后，执行器取消导航；最终 ledger CANCELED/USER_CANCEL、载荷 ATTACHED/版本 3/链接不变、stop_error 空。独立观测确认最终底盘线速度和角速度为 0，关节最大速度约 2.51e-11 rad/s，最终 cmd_vel 为零，几何及 Gazebo 附着确认通过，会话 RECOVERY_REQUIRED/motion_blocked=true/owned_pid=-1。修复后的连续停稳判断通过。

停止续租到所有验收条件满足为 8.336 s，包含租约剩余时间、取消、终态等待与停稳保持，**不是制动距离或纯制动时间指标**。本次任务总墙钟 54.634 s，仅成功 1 轮失联验证。账本 `/tmp/astribot-sim-completion/operator_fault_lease/task_1789815957884919433/ledger`，报告 `/tmp/astribot-sim-completion/fault_lease_result.json`。

失联用例录包已成功停止：`/tmp/astribot-sim-completion/incidents_fault_lease/incident_1789815956512640874_710335`，含 session.log、bag 和参数事件。SQLite 实查 clock 70892 条、constraint 5165 条、geometry_state 740 条、附着状态 3308 条；验收节点参数快照 5 次。最终构建、8 项 gtest、recorder YAML 解析与 diff 检查通过。本轮各场景按记录的独立 PID/启动时间停止；末轮复核本轮存活进程为 0。全局 count.sh 仍有其他会话进程，未停止或清理它们。

### 收臂取消修复与零速保持排查

新增 C++ `CancellableExecution`，将执行等待移出 action 回调，取消通过原 MoveIt 执行管理器停止。r6 在关节速度 0.052621 rad/s 时取消活动收臂轨迹，PREEMPTED、CANCELED、附着载荷保留和独立停稳均通过。进一步核对旧起步拒绝包，导航前底盘已在零命令下漂移约 0.451 m；因此继续保留碰撞拒绝，测试默认关闭的仿真轮位保持选项。真机 SDK 位置保持逻辑未改。逐轮参数、失败边界和后续结果见 [取消执行与静止保持记录](CANCEL_AND_STATIONARY_HOLD_20260919.md)，不以阶段取消或保护中止替代完整导航验收。

显式保持增益10、MTC缩放0.03的r10完整搬运通过，140.308 s；r12在0.020111 m/s带载行走时主动取消，2.725 s完成独立停稳验收，载荷保留、恢复门禁有效。两轮录包正常关闭。r11相同配置在收臂关节跟踪保护中止、尚未注入取消，保留失败记录；重复稳定性未验收。标准仿真保持默认值及真机行为均未改变。
