# 导航起点准入与脱困

本模块先在导航任务准入之后检查实际起点，READY 后才放行目标规划。工位搬运还接收目标规划报告的明确起步连接失败，通过同一脱困模块抵达安全位置，再从实际位置重新规划。正常搜索、MPPI、Arrival 不负责恢复编排。

工位搬运的执行期感知受阻也进入此闭环：有效感知风险导致 HOLD 时，策略返回可恢复的受阻结果；`RecoverPolicyObstruction` 结束旧跟踪，提交同一导航会话的 `RECOVER` 模式，再由 `ComputePathWithRecovery` 等待 FollowPath 终态及 SLAM 净位移停稳。每段脱困完成后重新查询感知，直到出口转向可行且起点检查 READY，才从实际位置规划原目标；`RESUME` 明确确认后恢复正常跟踪。服务的 `ROUTE_EVALUATION_PENDING` 只是待处理，不是模式提交成功或失败。恢复次数与时间预算不会随同一会话重试重置。

脱困选点在 C++ 内完成。`PlanStartRecovery.use_policy_obstacles=true` 时，从 `/navigation_policy/recovery_obstacles` 取得判阻模块自己的当前世界快照：每个障碍包含当前占用、已有预测扫掠及 2σ 不确定度。固定朝向平移检查连续凸包扫掠，出口检查所需转向；沿用已安装整机包络的安全余量，不重复添加。原始障碍坐标系保留到碰撞计算，不能把旋转后的矩形直接扩大成地图轴对齐盒。缺输入、非度量障碍、当前包络已相交、无安全出口明确失败，不用空路径伪装脱困成功。该快照只证明规划时的观测，短段执行中的动态环境变化仍在停稳后复查。

距离目标 0.5 m 内仍先尝试既有工位精调；若精调轨迹被分层占用阻挡，则同样转入脱困。缺地图或位姿等输入故障不自动转成运动恢复。正常搜索、MPPI、到位控制、底盘保持参数及非导航阶段监控规则不变。

```text
导航意图（不等于运动许可）
  → EnsureNavigationStart [本模块]
      → NavigationStartAssessment：读取当前位姿、地图和带载包络
      ├─ READY + 执行许可 → 放行目标规划
      ├─ RECOVERY_REQUIRED → 告警状态 → DeparturePlanner
      │    → 本段准入及路径检查一次 → FollowPath(DepartureController) → 实测停稳 → 刷新数据再次检查
      │    ├─ READY → 清除本次阻塞告警 → 放行目标规划
      │    ├─ RECOVERY_REQUIRED且有进展 → 更新快照，再选一个短动作
      │    └─ 无进展/无安全方向/预算耗尽 → 告警并返回失败
      ├─ UNAVAILABLE → 有界等待输入恢复，超时报警
      └─ BLOCKED → 实体相交，报警并返回失败
  → GridBased → 原 FollowPath → 到位
```

## 模块职责

| 类 | 输入 | 输出与职责 | 不负责 |
|---|---|---|---|
| NavigationStartAssessment | 当前 TF 位姿、global costmap、height maps、固定包络、任务目标 | `/navigation/assess_start` 只读状态、原因及版本 | 运动、报警清除、选择重试 |
| EnsureNavigationStart | 检查结果、退出子动作结果、执行关联 | 有界循环恢复、逐步停稳后复检、`/navigation/start_alarm` | 速度命令、碰撞算法 |
| ComputePathWithRecovery | 原目标、PlanCandidate INITIAL 的明确首连接失败、SLAM 停稳、脱困结果 | 结束旧跟踪、请求安全退出、复检实际位置、重新规划原目标；保持同一导航事务 | 普通搜索、直接发速度、提前结束搬运父任务 |
| DeparturePlanner | 新鲜当前位姿/地图/包络 | 保持朝向，比较后/左/右/前的安全退出，每次执行至多0.2 m | 最终目标路线、自动恢复编排 |
| DepartureController | 规划器已确认的短段路径、实测位姿/速度 | 经现有 Nav2 控制服务器输出低速命令；实测停稳后成功 | 重新选择目标、故障决策、报警管理 |

本包规划与检查依赖既有 path_tracking 的分层读取；短段控制依赖现有停止判据，正常 path_tracking 不依赖本包。插件加载于现有 Nav2 planner_server/controller_server/bt_navigator；逻辑按包划分，无新增进程和速度写入旁路。

## 几何契约

起点三维相交返回 BLOCKED；仅二维投影不足、分层几何实际无碰才进入候选退出。完整退出扫掠包含第 0 点和各高度层，未知区域阻断，原包络裕度不减少。候选完整出口满足原二维模型；每个短步不要求已经到达出口。起点只检查实际当前朝向，不假定朝目标原地旋转；完整目标规划仍独立负责后续路径可达性。

候选搜索最大2 m、步长0.05 m，优先最短分层安全出口，等长按后/左/右/前排序；一次平移至多0.2 m，速度最高0.05 m/s。根据用户明确要求，执行该短段时不重复消费包络、ACK、分层地图或策略约束做准入。控制器保留反馈、路径跟踪、速度/距离范围、取消和实测停止。动作实测停稳后重新取实际状态检查；仍被困时要求实际位移至少0.03 m，且不回到已访问位置，否则报警。最多10个短动作，整个前置过程120 s。

用户 2026-09-28 要求暂不做脱困执行偏离校验：已移除横向/航向偏差、相对原起点偏移和越程拒绝分支及专用横向偏差参数。短段仍由 Nav2 当前控制位姿反馈跟踪，轻微越过终点时以限速反馈修正；SLAM 只提供初始和结束的相对停稳证据，不与规划起点直接相减。脱困完成后重新查询实际当前位置及环境；READY 后普通 ComputePathToPose 不传入旧 start，由规划服务读取此刻机器人位置。规划起点不是预设脱困终点。

诊断使用新鲜完整包络，不依赖五方ACK已齐。每个短段规划前完成准入；规划器确认路径后，恢复子树直接执行FollowPath。删除执行段的RequireNavigationEnvelope、控制器重复碰撞/准入及policy新增分层重复检查，普通导航子树仍保留其原有RequireNavigationEnvelope和控制/策略检查。搬运调用方在初次绑定后只监督Nav2任务生命周期和停车，不复制内部持续准入。感知只证明规划时观测下的路径可用，短段执行期间环境变化留到停稳后复检，这不是绝对安全或动态避障验证。

当前分层输入仅接受 `gazebo_collision_geometry`；它是实时仿真碰撞几何，不是实机相机/LiDAR感知覆盖证明。本轮仅仿真 fixed_v2 退出；其他模式加载插件不等于具有退出执行能力。

## 数据接口与失效

- 工位规划恢复：`PlanCandidate.INITIAL → START_CONNECTION_BLOCKED → 旧 FollowPath 结束及 SLAM 停稳 → /navigation/plan_start_recovery → FollowPath(Departure) → AssessNavigationStart → INITIAL`。失败响应携带实际候选的 `required_start_heading`，恢复服务据此检查安全出口处的姿态连接；普通起点检查不假定该朝向。其他规划失败仍明确失败，不一律转脱困。恢复预算从首次明确首连接失败开始，最多 10 段 / 120 s，同一执行的重规划或 BT halt 不重置预算。新规划只保留原目标，起点由服务重新读取实际位置；旧路径已清空。
- `/navigation/plan_start_recovery`：PlanStartRecovery，请求包含 execution_id、朝向参考坐标系和实际失败朝向；响应为只读的安全短段路径或明确失败原因。沿用 DeparturePlanner 的分层平移、出口旋转扫掠和裕度，路径选择本身不发送运动。
- `/navigation/assess_start`：AssessNavigationStart，请求绑定 execution_id 和目标；响应状态 READY / RECOVERY_REQUIRED / BLOCKED / UNAVAILABLE、实际起点、采样时间、地图内容哈希、分层版本、几何哈希、包络 epoch。服务不发布运动。
- `/navigation/start_alarm`：DiagnosticArray，可靠队列 10；记录执行关联、阶段、初始/当前原因和版本。成功仅在检查或复检 READY 后发布 OK；错误返回失败，外层任务处理原有取消、保载、资源与停稳。
- `/path_tracking/departure_path` 与 `/path_tracking/departure_evidence`：退出候选诊断；实际执行仍通过 ComputePathToPose / FollowPath Action。
- 输入/查询与许可各有2 s等待窗口（steady）；输入暂失每100 ms重查。退出单步执行预算沿用120 s、无进展15 s，整体前置预算120 s；源时间与steady停稳窗口均至少 0.6 s。
- 取消通过 Nav2 原 action 链传播。BT状态机测试不能证明实际controller停止；现场需读取子动作终态、SLAM 位姿停稳窗口与速度命令。

验证与证据索引见 [实施记录](../../../../docs/START_DEPARTURE_IMPLEMENTATION_20260926.md)。
