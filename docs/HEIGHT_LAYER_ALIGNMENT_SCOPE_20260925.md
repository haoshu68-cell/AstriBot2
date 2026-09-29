# 分层地图仅用于三段跟踪器的起终点对齐

用户于2026-09-25收缩本轮范围：搜索、MPPI和现有到位调整暂不动；分层切片只用于三段跟踪器起始位置和终点对齐。此前“整栈分层搜索”施工已撤回，不作为当前路线。

## 行为边界

- 全局搜索 `ExactGoalPlanner`、MPPI critics、Arrival 精调、BT、普通二维 costmap 保持本轮开始前的实现。
- `ArrivalController` 继承 `ThreePhaseController`，并在常规跟踪时调用其计算入口；配置中的 inner=MPPI 不等于没有三段跟踪器。不更换当前控制器。
- 只在 `ALIGN_START` 和 `ALIGN_GOAL` 发出旋转命令之前，以固定姿态的分层包络检查分层地图中的旋转扫掠区域。
- 不更改 FOLLOW、CORNER_ALIGN、路由搜索、局部轨迹采样及 Arrival 的平移/航向精调。
- 数据不齐、层配置不一致、有效层未知或旋转扫掠碰撞，仅拒绝受影响的对齐操作；不把分层地图变成跟踪全过程的新准入。
- `idle_position_hold=true`、`idle_position_kp=3.0`保持。

## 数据流

`Gazebo当前碰撞形体/位姿/载荷附着归属 → 原子HeightSliceMaps → 起终点对齐检查`

`RobotGeometryState同次分层 → 固定包络reference_state_sequence对应层（clearance一次） → 对齐用height_geometry_hash`

原 `installed_geometry_hash` 继续代表原二维包络，原消费者和ACK语义不变。新增层身份只用于对齐，不将独立图加载当作已启用全栈分层规划。归档点云切片仍只输出障碍与未知；只有本次完整仿真碰撞几何产品才可输出明确域内的空闲格，不能冒充真实感知自由空间。

## 验证口径

2026-09-25，限定范围实现和离线/节点验证已通过：

- `alignment_install` 完成新消息及实际消费者构建；path_tracking 在最终头文件冻结后清理本包构建产物重新编译，避免安装头保留旧修改时间造成增量编译遗漏。
- `tracking_tests_clean.log`：8/8 CTest 通过，覆盖分层高度区分、旋转扫掠、未知区/边界、坐标转换、层配置与哈希，以及原始起点/拐角/策略限制契约。
- `layered_alignment_reader` 使用真实 ROS 回环，验证地图缺失/正常/碰撞/过期，START/GOAL 使用新检查，FOLLOW/CORNER 保持原流程，已收敛终点可以退出。
- `scope_test_final.log`：9 个冻结文件哈希一致；三段跟踪器只有 START/GOAL 检查及其生命周期接线改变。
- `geometry_tests.log` 与 `fixed_envelope_tests.log` 分别 3/3 CTest 通过。旧二维 installed_geometry_hash、ACK 与常规准入语义保留。
- `warehouse_map_runtime/result.json`：实际 Gazebo 仓库地图生产节点通过，8 次源时间推进快照、5 层均有占据/空闲/未知；此处没有机器人运动。

初次测试失败及修复证据保留：旧动态库/旧内联头引起构建身份问题；既有 corner_contract 夹具漏初始化真实 costmap，已仅补测试初始化，Arrival 生产实现未改。

完整搬运闭环仍待验证，不能据上述结果宣称抓取—搬运—放置完成。下一步使用当前仓库仿真和单一拥有的会话，机器人正对两个操作台执行正常主线；不扩展 VLA、真机或其他故障矩阵。

本轮操作记录位于 `runs/layered_stack_20260925/`。先前广义整栈方案及撤回代码只是留档，不在启用路径。

## 正对操作台主线联调

当前候选的取货停靠点为 world `(0.10, 0.15, π/2)`，放置停靠点为 `(1.15, 0.17, π/2)`。两处目标在机器人前方 0.55m、侧向 0m；导航目标由同会话实测 map/world 变换生成。取货前已实测面向误差 `5.91e-5 rad`，位置误差 `2.14e-5 m`。

- 初始 head pitch=0.95 的候选产生头部与躯干碰撞，已改回 0.65，yaw=0。实际全场景初姿碰撞检查通过；没有改碰撞几何或 ACM。
- 第一次地图联调暴露 `UNSUPPORTED_SYSTEM_PLUGIN`：现有物体账本不认识新地图插件，因此拒绝完整空载声明。这是集成遗漏。已在原清单登记精确 `astribot::HeightSliceMap`、`libastribot_height_slice_map.so` 与当前 world 实体，其他未知插件规则保留；两项 inventory CTest 通过。scene44 已实测地图加载后 EMPTY 确认正常，正式父任务已接受。
- 所有失败场次及清理记录保留；尚未完成主线前，录屏只能标注为场景/尝试记录。

联调索引：`runs/mainline_20260925/front_facing_alignment_0740/`。候选及每场 `used_*` 位于 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/front_facing_transfer_candidate_20260925/`。

## 当前抓取接近修正及会话保留

scene44 在实际运动前被 MTC 拒绝：`GRASP_APPROACH` 中左臂 link_3 与 torso link_4 自碰撞；已确认任务资源释放和实测停稳。问题计时从 2026-09-25 08:00 开始。

离线完整 PlanningScene（包含实际 ACM 和 Octomap）复现原位 8 条数值接近路径全部碰撞。仅将两个停靠点 world X 增加 0.15m，保持朝向 π/2、物体和抓取姿态不变，8 条候选中 7 条通过全身及环境 FCL 采样检查；目标位于机器人前方 0.55m、左侧 0.15m。停靠朝向仍正对台面。证据在 `runs/front_grasp_offline_20260925/`，仅代表离线采样；实际 KDL/MTC 分支、抓取、搬运及放置仍须实跑。

按用户最新要求，私有运行脚本增加保留当前会话的选项。每次任务结束后保留 Gazebo、反馈链路和录屏，同会话继续任务仍检查前次资源释放/停稳及当前空载/场景状态，不为重复尝试重启整栈。已通过的限定范围构建及测试不重复运行。

## scene45 同会话进展（2026-09-25）

world58 的 scene45 已实际完成 PREGRASP、GRASP_APPROACH、GRASP_CONFIRM、ATTACH_CONFIRM 和 LIFT，物体真实附着并离开取货台。随后的 TRANSPORT_POSTURE 前场景检查在 15 秒内未取得稳定窗口，返回 `MTC_SCENE_NOT_STABLE`。父任务释放资源且实测停稳通过，但物体仍被持有；这不是完整搬运通过。

停稳后采集的 12 份完整场景有 3 种原始 Octomap 字节哈希，但规范化碰撞占用签名完全一致；差异来自仍处于占据状态的概率值。这只说明事后观测窗口稳定，不能反推失败窗口的根因。未修改稳定性门槛或清空地图；起步受阻、梯度退让场景按用户要求延后。

归还物体到取货台暴露 PLACE 原有支撑选择硬编码为放置台。已用目标位姿、实际附件偏移及已登记的两张台面选择唯一支撑，只替换原临时接触对；8 项离线 gtest 通过。保留参数和运行依赖，仅替换 MTC planner 进程，Gazebo、底盘、执行器及录屏保持。该修复的运行时验收仍随回台动作验证。

回台首次规划因斜向退臂使夹爪连杆与物体碰撞而拒绝，未实际运动，释放和停稳通过。回台出口已改为沿接近方向竖直抬离 0.12m，由同一碰撞检查重新规划；不忽略夹爪碰撞。证据分别为 scene45 的 `return_pick/` 和 `return_pick_retry1/`，代码及构建记录在联调索引的 `support_selection_fix/`。

`return_pick_retry1/result.json` 已通过：PREPLACE、PLACE_APPROACH、RELEASE、DETACH_CONFIRM、RETREAT、STOW 后，独立账本/完整场景空载、仿真物理解除附着与台面位置稳定均成立，再主动取消已确认的持有动作并确认资源释放、36 帧/0.7 秒停稳。这里的 `TASK_CANCELED` 是放置完成后的正常持有资源释放，不能把它写成完整 FixedStationTransfer 成功。完整主线目标生成同步改用唯一竖直 0.12m 退臂，不保留斜向旧候选；旧文件仅在审计目录留档。

同会话重跑的验证脚本另外修正了实际发现的两项运维问题：新订阅者先取得 map/base TF，再采集同时间世界观测；已有 C++ executor 会保留前次 transfer 的导航 client，继续任务的发令前图允许同一 executor 的 0/1 个 client，而冷启动仍为 0，动作接受后仍为精确 1。其他外来 client、活动目标、UUID 绑定、资源释放及实测停稳规则未变。`full_transfer_retry1` 因旧生命周期假设在提交前拒绝，`NO_ACCEPTED_PARENT` 且停稳通过；改正后以新的 `full_transfer_retry2` 记录继续，不覆盖失败证据。
