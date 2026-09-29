# 起步受阻：先退出、再规划目标

2026-09-26 16:07 用户授权正式实施；保留 front53 及此前同问题排查记录，不把失败记录改为通过。

## 当前范围与分工

主线固定工位 PICK → HOLD → 感知选择退出位置 → 低速保持朝向后退 → 实测停稳 → 原目标规划/跟踪 → PLACE。

- root：DeparturePlanner、候选选择、插件注册、统一构建和仿真。
- 完善path_tracking到位精度：DepartureController、共用分层快照读取器、控制器验证。
- 盘点可迁移至 C++ 的 Python 实现：前置 BT 顺序、插件配置、顺序验证。
- 排查图像积压与传输方式：录屏及同次地图/包络/路径证据捕获准备。

## 最新架构边界（用户明确要求）

独立领域包 `astribot_s1_navigation_recovery` 统一拥有起点检查、脱困编排、告警、退路规划、受约束退出执行及复检。它依赖现有 path_tracking 的包络/政策/分层快照读取和停稳判据；path_tracking 不反向依赖 recovery。正常 GridBased / MPPI / Arrival 不承担起点故障处置。

同一前置模块内：NavigationStartAssessment 提供只读检查；EnsureNavigationStart 拥有 CHECK → RECOVER → RECHECK → READY/ALARM；DeparturePlanner 只产生安全退路；DepartureController 只执行并确认停稳。只允许一次退出；复检失败上报告警，不能通过清除报警或失败回退继续目标规划。

起点评估返回 READY / RECOVERY_REQUIRED / BLOCKED / UNAVAILABLE 及实际起点、二维地图内容哈希、分层图版本、几何哈希、包络 epoch。告警使用 /navigation/start_alarm 的 DiagnosticArray，绑定 execution_id、阶段及原始原因；仅真正复检通过才能恢复导航准入。起点3D碰撞/数据缺失不自动后退。

部署复用既有 Nav2 进程：recovery 几何查询和退路插件加载于持有 global costmap 的 planner_server，controller 插件加载于 controller_server，恢复BT加载于 bt_navigator。逻辑归属在独立恢复包，无新增全局速度写入者、无新进程。

## 实现与数据流

当前会话的 NavigateToPose 默认树在 RequireNavigationEnvelope / PolicyExecution 内顺序执行：

1. EnsureNavigationStart 先调用本模块 /navigation/assess_start。READY 直接放行；RECOVERY_REQUIRED 才进入以下退出子树；BLOCKED/UNAVAILABLE 报警并失败。
2. ComputePathToPose(Departure)：读取当前位姿、当前二维 costmap、带版本的分层地图及带载包络。起点已满足退出交接条件则返回单点，否则沿当前机身负 X 轴以 5 cm 间隔寻找最近安全位置，最多 2 m。
3. FollowPath(Departure)：复用 Nav2 控制器服务器和现有运动约束，速度上限 5 cm/s；不启用原生 BackUp，不在任务层直接发布速度。
4. DepartureController 以实测位姿/速度确认停稳后向既有 ArrivalGoalChecker 报告到达。单点也经过停稳确认。
5. EnsureNavigationStart 重新查询实际起点；必须 READY 后，才执行原 GridBased 规划与 FollowPath 跟踪。原 KeepSafePath 重规划不重新执行退出阶段。

同一个外层导航任务/资源权限；退出 FollowPath 和后续 FollowPath 是各自独立的子 Action UUID。当前修改默认树，作用于本会话 NavigateToPose，不声称仅作用于某个 transport goal。

## 几何与感知边界

退出段正式使用当前分层几何模型，包含第 0 点及完整扫掠；保留原包络安全裕度。二维总投影相交不会直接豁免，只有高度分离后的起点和全段均无碰撞才可能放行。退出终点还需满足原二维模型及面向目标方向的转向检查；停稳后目标规划独立复核，不预先宣称一定可达。

分层碰撞读取从原 ALIGN_START/ALIGN_GOAL 扩展到新 Departure 阶段；原搜索、MPPI、Arrival 控制律和碰撞标准不改。idle_position_hold=true、kp=3.0 不改。

当前分层图的 evidence_kind 为 gazebo_collision_geometry，来自实时仿真碰撞几何投影，不是实际相机/激光观测融合，不代表真机后向覆盖已验收。后续真实感知源接入不是本轮范围。已有扫描、传感器健康及政策限速/停止约束继续有效，不覆盖或旁路其 HOLD。

起点三维相交、后方未知/障碍、无候选、地图/包络过期或变版、退出偏离/超时均明确失败。动态变化由每控制周期的新鲜快照和剩余段/制动扫掠复核；没有未来动态预测保证。

## 最小验收

- C++ 候选选择：无需退出、高低障碍分离、真实起点碰撞、后方障碍/未知、距离预算、地图变化。
- Controller：输入约束、低速反向、偏离/新障碍拒绝、单点及移动后真实停稳。
- BT：停稳前不调目标规划、失败不进入下一段、正常重规划不重复后退、新任务重评估。
- 插件构建/加载与原受影响测试。
- 单一导航环境正常工作台闭环与录屏：区分退出、导航、放置、EMPTY、释放和停稳。未运行或失败不得标通过。

## 前次结果补充

front53 原任务终态保留 NAVIGATION_NOT_SUCCEEDED / resources_released=false。之后同一执行器完成资源交接，独立记录 before_shutdown_late_release_stop.json 确认释放与零速，再关闭自有仿真。此收尾不改变完整搬运未通过的结果。

## 16:39 仿真准备结果与修正

front54 已加载新 recovery core/BT 插件，但动作验收脚本仍要求旧观察器 43 项配置，新观察器实际为 51 项，因此在发送父动作前拒绝。记录 `M5_OBSERVER_CONFIGURATION_MISMATCH`、`python_parent_goals_sent=0`、`NO_ACCEPTED_PARENT`、`cleanup_stop.passed=true`；不是起步脱困或完整搬运通过。验证脚本仅更新观察项数量，保持逐项话题、类型、哈希与动作所有权检查。

另发现旧 Arrival StartManeuverChannel 和 Python StartManeuverAdapter 在真实覆盖层仍启用；正在归档后删除两端，防止目标规划后出现第二条自动后退流程。正常 ALIGN 实时碰撞拒绝保留。

本轮连续 bag 因 SQLite database is locked 异常中断。观察任务曾打开运行中数据库进行只读计数，时点与读锁竞争一致，不能把该观察声称无扰动。以后仅在记录进程退出后读取数据库；失败原件保留，不当作完整录制。视频 1102 帧完整，但无任务动作，无需展示为空闲演示。

front54 通过既有 root runner 的启动身份发送退出，完成实测停稳和 owned 收尾；supervisor state=stopped、remaining_owned_pids=[]。下一次仅在删除重复旧链和回归通过后启动。

## 生产策略接线审查边界

Departure 发布 active_path，现有 policy 消费此路径；普通 check_path/GridBased 仅在 READY 后调用，故不因尚未生成普通路径就必然 HOLD。但现有 navigation_constraint 对原扫描点执行二维扫掠，若真实扫描点进入投影包络，即使分层通过也会 INDEPENDENT_SWEEP_RISK。front53 的栅格碰撞不足以证明该条件成立；现场须保留 constraint.reason，不旁路它。

同时识别但本轮正常静态主线不扩展验证的现存契约缺口：动态风险预测从 XY 路径切线推断机身朝向，无法表达保持机头朝向倒车时非对称载荷的姿态；旧 Bool path_blocked 在换路径时可能保留，缺少对应 typed evidence 时可沿用旧阻塞。这两项不能由本轮静态工位成功外推为已解决。本轮新进程初始 path_blocked=false，无动态 tracks 的工位场景不证明动态搬运回归。

## 16:47 旧链归档与构建

旧控制器请求/REVERSE支路、Python StartManeuverAdapter/策略、2项旧消息及生成列表、profile旧参数/解析已删除；保留正常ALIGN实现。19个源码/文档变更的改前文件与补丁保存在 `runs/mainline_20260926/retire_legacy_start_maneuver/`。

4包统一构建成功，新恢复4项CTest、原跟踪受影响7项CTest、2项YAML检查通过；新C++ profile独立探针与Python逐值差分368项通过。私有实际policy复制自前次运行版本，仅删除本链路，新profile除删除旧块其余数值一致。旧生成消息的增量Python __init__残留在启动前import检查被捕获；转为干净构建消息包，未以手改生成文件掩盖来源。

## front55 实场结果与一小时暂缓点

2026-09-26 17:05:18 实场完成 PICK/LIFT/TRANSPORT_POSTURE 后进入新的起点检查。日志顺序：

- `NAVIGATION_START stage=CHECK execution=92037347310255:1`
- `/navigation/assess_start` 返回 `state=1 / START_PLANAR_CLEARANCE_REQUIRES_DEPARTURE`，实际位姿 `(0.251791096,0.149580161,1.566104983)`；该响应同时记录地图/几何/包络版本。
- 下一次 BT tick 的既有外层包络检查报 `NAVIGATION_ENVELOPE_FAILURE: ENVELOPE_EXPIRED`，导航终止，未到 RECOVER/RECHECK/READY，未执行退出或 PLACE。

这证明新服务在目标规划前被真实调用且返回需恢复；不能证明退出执行、复检后规划或完整搬运通过。暂不推断过期根因；未改变包络时效或碰撞门槛。

原任务结果 `RENEW_REJECTED:NAVIGATION_NOT_SUCCEEDED`，父任务返回 `resources_released=false`，保持原始失败记录。随后独立读取执行器/持久账本确认同任务资源已 RELEASED，实测 0.7 s 内位移/转角/速度为零；该后续证明不覆盖原失败。证据为 front55 case 下 `before_shutdown_late_release_stop.json`。

本轮辅助验证修正也留档：观察数量43→51；记录终止/提前退出不能判采集通过；脚本只支持最多600 s，撤回不合法3600 s观察器参数；同PID bash→exec的身份应在环境加载后捕获，首次过早记录已作为无动作失败归档。首个动作前失败不含 executor_binary 时，仅在未发送父动作、无准入不确定性、同冷启动执行器身份、已停稳/释放的严格条件下使用冷启动凭据确认重试归属。没有改写失败结果为通过。

16:07开始的本问题达到1小时，17:07执行有界安全收尾并暂缓进一步定位。保留当前 front55/domain88 仿真，不再发目标、不重启。5个自有录制/观测进程退出后才读取bag元数据及视频；当前一套新recovery/清理后policy仍在运行。恢复入口：同次bag的 `/navigation/envelope_v2`、geometry、ack、rosout与精确CHECK时间，先辨明为何外层包络过期，再考虑下一次动作。

结果索引：`runs/mainline_20260926/start_recovery/front55_verification_summary.json`。
完整案例：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/front_facing_transfer_candidate_20260925/front_transfer_scene55_world88_20260926`。
演示：`runs/mainline_20260926/demo/front55_pick_and_start_check.mp4`。该录像仅展示抓取与前置检查处停止，不是完整搬运演示。

## 用户继续后：外层包络旧心跳积压已复现并修复（实场待重验）

保留16:07起累计排查及17:07暂缓记录，用户再次明确继续后使用已关闭的front55同次bag续查。失败前持续READY_FIXED，最近消息626.15有效至626.325455136；明确上游撤销在失败104ms之后，不是该次失败起因。

Humble `spin_some`对本组唯一订阅每tick只take一条。首tick的depth10最老消息stamp626.012有效至626.159246556，在now626.150仍有效；下一tick读取第二老stamp626.033/valid_until626.130441239，在now626.154已过期，锁存ENVELOPE_EXPIRED；队列后仍有有效新心跳。此顺序与同次bag和实际两个BT tick吻合。

真实ROS/真实RequireNavigationEnvelope插件夹具复现：旧front55库6场景中仅积压场景失败；最终修正版6/6通过。每tick使用1ms有界spin_all，排空或预算结束后再对所选样本判时效；逐回调继续锁存明确撤销、身份及模式/坐标等既有非时效错误，不减少QoS深度、不放宽时间阈值。预算耗尽并不保证已取最新；仍按原门槛失败。最终ELF SHA033ce08ba805b19139e8aab43b73aa021483cb575d95d08b9c929c023ca79fbf。

初版候选库未包含编译过程中补入的结构校验，被HOLD夹心测试抓住，保留失败库/日志；最终冻结源码、删除单个旧对象强制编译后通过。没有把工作源码当已部署产物。证据：`runs/mainline_20260926/envelope_bt_repro/`。原PolicyExecution取消/重启和EnvelopeEvidence边界测试仍通过。下一同场景front56/domain91只替换此BT插件，并在动作前读取实际映射与SHA。


front56/domain91 在执行器启动时因该domain已有历史账本、对应旧恢复marker缺失而报 RESOURCE_MARKER_MISSING，未发送动作。原记录未修改，未清空全机资源。已关闭该自有无动作会话，并检查domain83既无transport持久账本又无现存进程，使用同配置front57/domain83；实际三项BT/recovery映射与SHA均在动作前读回。此为操作前置检查遗漏，不归因于新脱困算法。

## front57 与 ACK 时序修复

front57 首次动作在抓取前 GEOMETRY_INVALID_OR_EXPIRED 终止，原失败保留。同实例一次重试完成 PICK/LIFT/TRANSPORT_POSTURE，但在发送导航目标之前 NAVIGATION_ACK_INVALID；navigation_goal_uuid 为空，未进入起点检查。父任务确认 RELEASE_CONFIRMED、cleanup_complete；随后独立只读检查 0.7 s 实测位移、角度、速度为零。没有在已附着载荷状态下再次发 PICK。

359 条观察器 ACK 的时间结构合法，不能由观察器的时钟替代失败 helper 回调的时钟。代码确认：helper 把 source > 本地接收 ROS 时间与非法时间结构混为同一永久失败，而协调器对尚未来到的正 ACK 只暂不采用。此次未记录失败回调的实际差值，不能宣称 front57 的具体超前量已测得。

最小修复仅改 fixed_station_navigation.cpp 的 ACK 分支：合法未来正 ACK 不采用，原证据与原500 ms截止不变；未来负 ACK 在下发后仍撤销，下发前保存 false 与原 source，旧 positive 不能覆盖撤销时间戳。公共 fresh、包络/Odom、版本与停稳条件均未改。增加实际回调 source/local/ahead 诊断，不增加容忍窗口。

真实 C++ Helper/ROS 通信、模拟端点夹具共11项通过。原6项加未来正 ACK 等待、未来负 ACK 发出前拒绝/发出后取消、零时间戳拒绝、持续未来正 ACK 不续期5项。测试使用系统时钟并注入+1/+2 s的确定时间差，不是Gazebo或真实/clock同步验收。同一新增用例加载旧库稳定失败 NAVIGATION_ACK_INVALID，新库通过。证据 `runs/mainline_20260926/ack_clock_order/`，候选库SHA `2f34a7014e5bd0dff114e0592847c9ece8514016fec65657571ac2fb7bc87361`。

front57 已按自有runner身份正常退出，remaining_owned_pids=[]；连续bag和observer返回0，录像2866帧与侧录/解码计数一致。原runner的result指向首次动作，retry1的动作结论须单独读取，不能把两者混为一次通过。下一次front58/domain40仅更换ACK helper；在动作前增加实际映射SHA读回。完整主线仍未通过，待本次同场景重验。
