# M1 非收拢臂姿接线验证 — 2026-09-24

本项从 10:35 +08 开始，沿用该起点排查。唯一仿真操作者为导航任务；没有真机操作。M2 静止质量链已经完成，不重复该矩阵。

## 当前结果

尚未发送机械臂轨迹。正常箱体、两个工位、观察相机、权威 EMPTY 库存和独立完整 PlanningScene 已在 Gazebo 中建立并核对；实际 PREGRASP→停稳→Hold→六消费者包络确认仍未通过。

证据目录：`/home/yjh/WorkSpace/astribot_validation/M1_scene_prepare_20260924_1024`。

| 场次 | 实测结果 | 收尾 |
| --- | --- | --- |
| first_scene01 | 准备工具错误地向 robot_state_publisher 请求未声明的 semantic 参数，空回包触发 IndexError；未创建夹具、未发动作。已按节点修正查询字段。 | 自有进程正常退出。 |
| first_scene02 | 同尺寸 60×60×120 mm、0.2 kg box 已有物理碰撞体与惯量；两工位及观察相机进入背景参考，box 单独登记。真实 detached、EMPTY、完整场景及底盘 0.7 s 零运动通过。随后暴露 native 执行器冷启动循环，未提交 Action。 | probe 收尾通过；overview bridge 需强制回收，因此场次总 cleanup 标为无效。事后逐一核对 8 个已捕获身份全部退出，supervisor 自有残留为空。 |
| first_scene03 | 新执行器和纯规划连接修复已实际加载；六组 FJT client 均唯一属于 native。本次 Action 被接受，真实导航撤销 ACK 后进入 MTC。随后因第 4 关节初态位于软规划余量之外被拒绝，没有 child_submission 或关节运动。 | Action 返回 resources_released=true；所属进程全部退出。MoveGroup 在受控 SIGINT 后卸载时出现 -11，单列为退出故障，不称干净退出或运行中规划失败原因。 |

`first_stage_progress.json` 汇总三场原始结果；`first_scene02/fixtures/result.json` 保存实际 Gazebo 位姿、世界到机身变换、加载记录及库存。`first_scene02/first_stage/result.json` 保存空 envelope 观测、停稳证据和失败原因。录像 `first_scene02/first_stage.mp4` 只有准备后静止画面，不能作为机械臂运动通过证据。

## 已确认的主流程阻塞

真实 fixed coordinator 在首次固定包络提案前没有输出；撤销已有包络时仍保持 FIXED_POSTURE 模式，并将有效期收紧到当前时刻。native PlanToHold 却在取得机械臂资源前要求已有、未过期、mode=HOLD 的包络。先 Hold 才能提案固定包络、先新鲜固定 HOLD 才能执行机械臂，形成循环。第二场实际记录的 `/navigation/envelope_v2` 样本数为零，与源码一致。

修复由 M1 单一写入者负责：在首个 ARM 副作用之前，以本次任务/lease 绑定的真实导航撤销响应和实测底盘停稳建立依据；不能把“未收到 envelope”解释为撤销确认，也不能发布伪造 HOLD 绕过。导航正许可继续拒绝。先覆盖真实 coordinator 的“初始无输出、撤销后立即过期”协议，再使用新独立安装层复测本场。

截至 11:03，M1 候选已完成上述修复及暖启动消息顺序修正，独立安装 7/7 CTest、11/11 协议通过，见 [cold 修复交付](evidence/m1_transport_20260924/COLD_START_REPAIR.md)。第三场尚未启动：总调度审查又确认纯规划路径调用 `GripperCommander.configure` 时无条件创建真实夹爪 Action client，会与 native 的单一控制方检查冲突。正在将这些规划调用改为仅使用几何计算，不创建控制连接。第二场未保存夹爪 client graph，本项冲突目前有源码、安装解析及规划节点启动日志依据，不能表述为该场已实际触发的第二个错误。两项均沿首段 10:35 起点处理。

11:15 第三场已越过上述两项阻塞。实际左/右第 4 关节约为 `0 rad`，URDF 硬限位为 `[-0.06,2.61] rad`；当前规划余量 `0.1 rad` 将允许起点下界收紧至 `0.04 rad`，导致默认合法初态报 `MTC_START_OUTSIDE_PLANNING_MARGIN:astribot_arm_left_joint_4`。原始目标、六控制方图、关节及终态在 `first_scene03/first_stage/result.json`；本 lease 的真实撤销与无 JTC 发送事实在 `first_scene03/own_lease_journal.json`。该问题仍沿 10:35 起点，由 MTC 所属窗口处理合法初态进入安全规划区的逻辑；未修改反馈、初始关节或统一降低规划余量。

第三场还获得实际头相机同 stamp RGB/Depth/Info、独立 Gazebo pose 记录，以及仅由台面几何生成的观察 region。M5 已完成 26.5 s 同时刻组包核对；当前头朝前，左侧工位 region 的有效深度点为零。这些属于静态采样，不是运动覆盖或模型检测通过。

## 零姿态准备与独立 READY 场景的边界

总调度复核后保留第三场拒绝。第 4 关节距硬下限仅 0.06 rad，现有 0.05 rad 执行偏差报警没有给出额外停止余量；JTC 的实际插值也不能用线性关节采样替代。不能仅调小软余量使本次通过。零姿态进入工作姿态仍需原生准备链，属于主流程未完成项。

11:29 只读检查确认：`warehouse_sim.launch.py` 只暴露整机出生位置和航向，当前 `astribot_s1_ros2_control.xacro` 没有初始关节参数，实际第三场 URDF 也没有 `initial_value`。已加载的 gz_ros2_control 源码支持通过 position state interface 的 `initial_value` 初始化真实 Gazebo 关节与 command，但项目尚未接入该配置，不能称为现成 READY 场景入口。本轮没有新增入口或插件，没有发布伪关节状态，没有启动第四场。

已有姿态来源为：SRDF 左臂 `ready=[0,-0.5,-0.6,1,0,0,0]`；技能规划器右臂 `transport_compact=[0,0,0.6,1,0,0,0]`；场景配置 `head_pick_joints=[1.45,0.65]`。旧 `normal_grasp_video_20260923/task01` 有 READY_RIGHT、READY_LEFT、LOOK_PICK 和后续规划事件。该历史流程属于旧执行器证据；即使以后以这些值生成独立 READY 初态场景，也不证明当前原生执行器从零姿态起步的问题已修复。

## 坐标及场景口径

MoveIt 的 fixed virtual joint 将 planning world 固定到 torso_base，不能把该 world 当成 Gazebo 世界。准备工具读取实际 robot 模型与夹具的同次世界位姿，将夹具反变换到 `astribot_torso_base`；完整场景读回保持该 frame。后续目标只能从当前场景对象生成，不复用旧地图坐标。

箱体是静态运动学夹具。本项不证明接触抓取、夹持力或动态载荷惯性效果。首段即使通过，也只证明 PREGRASP 与正式 Hold，不等于完整 PICK/PLACE。

下一步是首段通过后按[实际臂展窄通道矩阵](NARROW_PASSAGE_ARM_SPAN_PLAN_20260923.md)验证真实关节轮廓、非对称通行宽度及全身退出后的转向，不新增许可等待状态。

11:35:10 按单问题一小时规则暂停零初态排查，保留以上未通过结论。独立 READY 初始姿态配置从 11:27 保守计时，已有 16 项离线检查和两个包独立构建通过，尚未实际启动；该工作不能冲销本项失败或重置本项计时。

可选入口为 `warehouse_sim.launch.py sim_initial_joint_profile:=transport_ready`；受控验证脚本通过 `ASTRIBOT_SIM_INITIAL_JOINT_PROFILE=transport_ready` 向冻结的启动器传递同一枚举值。默认空值保持原模型。配置来自 `astribot_s1_description/config/sim_initial_transport_ready.yaml`，Gazebo 创建物理关节时初始化，不发布伪 `/joint_states`，不执行零到 READY 的轨迹。MoveIt 不使用这些硬件初始化标签，实际启动时逐项对比全部 link/joint 几何并分别保留原 XML 哈希。

该独立场景的离线检查、安装身份和运行脚本位于 `/home/yjh/WorkSpace/astribot_validation/READY_profile_20260924_1133`。目录尾缀不是计时证据，真实起点以其中 `timing.json` 为准。实际 22 关节、TF、完整碰撞场景及后续 Hold 验证通过前，不能据此宣布 M1 或 M4 通过。

## READY 独立首场实测

`ready_scene01` 于 11:42:53 启动，11:44:07 完成自有资源清理。该场因协作调度消息交错收到人工 SIGINT，不能记作完整首段通过；同时日志已给出独立于取消的规划后拒绝事实。

- 实际 `/joint_states` 的 22 个主动 position 关节完整；最大初值位置偏差 `0.0019009145 rad`，最大实测速度 `5.41e-14 rad/s`。保存源时间和首次同 stamp 的 steady 接收时间，不以 geometry 缓存或 PlanningScene 代替原始速度。
- RSP 与 MoveIt 全部 link/joint 的几何、碰撞、惯量、限位一致；两者原 XML 哈希及 22/0 个初值标签的已知差异单独保存。左右末端与头部 TF、完整场景状态有效性通过。
- PlanningScene 原始关节时间戳为零、速度为空，这是合法的场景序列化；保留原值，只独立核对位置。该工装修复另有 3 项离线边界检查，其中正例使用历史 7 轴真实 JTC feedback，未伪造历史 22 轴速度。
- 父 Action 在仿真 `34.006 s` 发送，`52.314 s` 规划终态 success=true，随后 native 立即以 `MTC_SCENE_CHANGED` 撤销并归还资源；没有 `child_submission`，没有实际机械臂轨迹。规划后原始 Scene 未落盘，现有记录不能确定具体哪个字段变化；不得用 Action 前读回冒充规划后数据。
- 资源 `RELEASE_CONFIRMED`；18 条捕获身份、12 个唯一 PID 全部退出。MoveGroup 在 SIGINT 卸载时出现 `-11`，单列退出故障，不称干净退出，也不作为规划后拒绝的原因。

原始数据见该场 `summary.json`、`fixtures/result.json`、`first_stage/result.json`、`own_lease_journal.json`、`post_cleanup_identity_audit.json`。静态头部快照、同场台面 region 及 `first_stage.mp4` 已交 M5 分析，不代替动作覆盖。下一场等待场景签名问题定位，避免原样重复必败场景。

## 12:15 起：首段已开始运动，执行响应仍未通过

`M1_scene_binding_20260924/scene_binding_scene01` 实际加载冻结 canonical scene 候选，规划前/后原始 CDR 已保留。场景复核通过，六个真实 JTC child 被接收；不能把这六个响应当作固定包络的六消费者 ACK。轨迹本地起算为 38.270 s，随后真实左臂运动，38.639 s 上层记录 `EXECUTION_GUARD_UNHEALTHY`，取消后六 child 终态齐全，父任务释放资源，自有进程均退出。完整首段、Hold、六消费者 ACK 未通过。

38.610 s 同消息的 joint1 期望/实际角度分别为 `-0.123952424/-0.072622670 rad`，差 `0.051329754 rad`，超过已有 `0.05 rad` 门限。首参考仅变化 `0.000124766 rad`，后续时标连续；实际位置接口配置与反馈支持位置伺服滞后这一候选原因。原始 guard status 未录制，不能认定唯一停止分支；38.660 s 的底盘角速度越限晚于 unhealthy 反馈。另一个停止收尾的 `RESOURCE_CLOCK_RESET` 由总调度独立处理，不能据此称 Gazebo 时钟回退。

原首段规划节点指标为 4.498 s、峰速 0.840 rad/s。总调度已同意规划最终检查/缓存之前统一时间伸长 2.5 倍作为下一候选，默认行为不变，执行器不修改绑定后轨迹；名义时长 11.245 s、峰速 0.336 rad/s。JTC 已采参考加速度峰 4.5395 rad/s² 高于规划节点日志 2.480，故节点指标不是样条全程极值保证。保护门限和控制器均不放宽。

原始诊断与 189 行逐关节量：`/home/yjh/WorkSpace/astribot_validation/M1_scene_binding_20260924/scene_binding_scene01/execution_response_analysis`。下一场工装正在接入已有 M5 observer、原始 guard reason、完整实际轨迹和有限 head 原始采集；截至本记录仍未重起仿真，等冻结候选与总调度明确放行。本问题沿 12:15 计时，13:15 未关闭则按规则暂停，不重置计时。M4 实际臂展通行仍等待正常首段/保持成立，已有 21 项适配及 6 场真实历史几何离线证据不替代它。

## 时间伸长候选的实际结果

`M1_execution_response_20260924/execution_response_scene01` 在 Action 前因验证工装同步 observer 检查阻塞回调而报 `STALE_LEDGER_CAPTURE`，没有机械臂动作。已将整段文件/拓扑检查提前到原 ready/fresh 等待之前，之后重新读取完整 Scene、TF 和目标；不修改时效。旧序/新序的离线顺序对照保留在 `observer_precheck_order_fix`。

`execution_response_scene02` 实际参数为时间系数 2.5、原速度/加速度系数 0.1/0.1、原关节余量 0.1。首段真实下发 426 点、7.619381332 s、节点峰速 0.320496 rad/s；7 个轨迹/Goal CDR 已解码，左臂实际 Goal 与绑定轨迹完全一致。本次 OMPL 路径与前次 717 点不同，不声称是同一路径 A/B。

70.704 s 首次执行反馈，79.029 s 实际 `FIRST_STAGE_HOLD_CONFIRMED`；六个 child 均终态成功。guard 的执行段始终 `EXECUTION_WITHIN_BOUNDS`，采样最大关节误差 0.0320496 rad、底盘平移偏差 0.00005616 m、旋转偏差 0.00018256 rad。限定的机械臂首段及 Hold 已观察成立，完整主线仍未通过。

随后 ACK 验证回调访问当前及冻结 `EnvelopeApplyStatus` 都不存在的 `mode` 字段，异常也中断 probe 停稳收尾。父任务未证实释放，原 domain90 journal 记录 `quarantined/RESOURCE_LEASE_EXPIRED` 与导航撤销；不能把仿真进程退出改写为资源释放。25 份捕获身份对应 17 个唯一 PID 均退出；录像 383 个实际解码帧与 sidecar/summary 相等，但覆盖分析单列。修正只删除 ACK 错误字段读取，保留 Envelope 本身的 mode；实际安装消息 RED→GREEN 和既有 session/epoch/hash/六消费者拒绝对照通过。

domain90 journal 原样保留。下一次若放行，使用新 world/session/domain91 验证六 ACK 和可靠释放；不是恢复旧事务。证据：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924` 下各场原始结果、`motion_summary.json`、`ack_contract_fix` 与 `new_world91`。

## 13:03–13:05 首段闭环通过

全新 `execution_response_scene03_world91` 在相同时间系数 2.5 下完成真实 PREGRASP、保持、六消费者确认及取消释放。轨迹 738 点、11.658845208 s，节点峰速 0.322889 rad/s；guard 最大采样关节误差 0.0322653 rad。六个控制器子动作成功；父 Action 最终 CANCELED 且 resources_released=true，journal 为 RELEASED，独立停稳通过，所有捕获的自有进程身份均已退出。本项关闭 12:15 起的首段跟随问题，不代表完整 PICK/PLACE 或零姿态准备通过。

取消前可独立复算的六 ACK 见证是 `first_stage/result.json` 的 events[3]（ROS 71.505 s）、envelopes[10] 和 acks[4,5,8,9,12,13]；同 session/epoch/hash 全部 applied，最大接收龄 0.040657 s、源龄 0.041 s。总计 424 条 ACK 包含取消后记录，不作为取消前数量。typed ArmHold 参与在线判断，但未完整序列化；held_joints 是 geometry.joints。在线门禁核接收龄、hold_confirmed/hold_id，lease/epoch 由 Action feedback 与 executor 绑定，不能声称已独立核查 typed 消息源时间或字段。

视频实际解码 515 帧与 sidecar/summary 一致。observer 提前结束为 130/INCOMPLETE，不是完整 300 秒采样通过；M5 独立窗口分析见 `docs/evidence/mainline_m5_20260924/execution_response_scene03_world91_motion`。固定帧率录像不能代替实际采样间隔。

下一项完整 PICK 使用新原生 `/transport/manipulate_to_hold`，与冻结首段执行器不同。私有准备在 `M1_execution_response_20260924/full_action_wiring`：先只读实际模型 XML，再启动带物体注册参数的执行器，之后使用原停稳/idle 检查创建夹具及独立盘点/Scene。这样避免“准备等执行器、执行器等准备”的循环。尚未启动完整动作；等待完整候选修复、协议和独立审查通过。
