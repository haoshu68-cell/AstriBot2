# Native PICK 准备阶段接线建议与 MTC 起点拒绝记录

记录时间：2026-09-24 11:31 +08。状态：**只读源码审查及接线设计，未施工、未执行准备动作、未通过新仿真**。首段问题仍从 10:35 计时，11:35 到点按原规则暂缓；本设计不重置计时。M1 独占 native 写权，本文件不修改其源码、安装或完整链设计。

## 已确认事实及保留的拒绝

第三场证据：`/home/yjh/WorkSpace/astribot_validation/M1_scene_prepare_20260924_1024/first_scene03/first_stage/result.json`。

- `passed=false`；实际初始左 joint_4=`2.5652906924701136e-11` rad；完整 22 关节来自实测 JointState。
- 父动作返回 `MTC_PLAN_FAILED:MTC_START_OUTSIDE_PLANNING_MARGIN:astribot_arm_left_joint_4`、`resources_released=true`；`cleanup_complete=true`、`cleanup_errors=[]`、`resource_disposition=RELEASE_CONFIRMED`。
- 该错误发生在 MTC 起点检查，不能记为 PREGRASP 执行或完整抓放通过。总调度已确认第三场冷启动撤销与控制器图独占准入通过。
- `astribot_s1_transport_mtc/src/mtc_planner.cpp:183–195` 对实际起点执行原 `planningInterval` 检查；joint_4 硬限 [-0.06, 2.61]、0.1 rad 内缩后下界 +0.04。保留此拒绝，不扩第一段约束、不更改初始关节、不降低全局 margin。

近限执行预算仍未闭合：native 起点匹配容差 0.025、geometry 默认误差 0.003、执行 guard 最大跟踪报警 0.05/20 ms 定时检查，并不构成含观测延迟、控制器插值和制动位移的统一误差保证。`execution_guard.cpp:93–98` 的报警门槛不是可保证的最大偏差。因此不实施“首 Connect 放宽到初始 q 再单调进入”的替代方案。本轮准备链按既有仿真规划/执行规则验证；不要求凭空证明硬件安全，也不把仿真结果外推成硬件保证。

旧受权顺序有源码与历史证据：`astribot_s1_transport/astribot_s1_transport/core.py:213–216` 为 READY_RIGHT(`transport_compact`) → READY_LEFT(`ready`) → OPEN → LOOK_PICK；`runs/normal_grasp_20260923/task08/pick_mtc_plan.json` 的首 PREGRASP 起点左 joint_4≈0.999984，头部 [1.45, 0.65]。这说明旧入口先准备，再规划抓取；历史路径不是当前 fixed_v2 完整链验收，也不能把历史关节值写成当前实测状态。

## 最小入口与阶段

建议只接 **`/transport/manipulate_to_hold` / `ManipulationToHold` 的空载 PICK 前置**。保留 `/transport/plan_to_hold` 冻结的“只执行原首段”语义。PLACE 已持物时不得复用空载 READY/OPEN；它继续按实际载荷和 PLACE 前置规划。

顺序为：同一 owner 准入/租约 → 导航撤销 ACK 与读回、底盘停止 → 全库存及完整场景确认 EMPTY → READY_RIGHT → 实测到位稳定 → READY_LEFT → 实测到位稳定 → OPEN（若原 PICK 合同要求）→ HEAD_PICK → 实测到位稳定 → 新采集/TF 与 SNAPSHOT → M3（启用时）→ 原 MTC → 后续完整操作。

三个准备请求复用 `/transport/plan_skill`：

|阶段|PlanSkill 请求|目标来源|
|---|---|---|
|READY_RIGHT|`operation=named, group=arm_right, named_target=transport_compact`|`transport_skill_planner.cpp:103–108` 的现有目标|
|READY_LEFT|`operation=named, group=arm_left, named_target=ready`|SRDF `astribot_s1.srdf:155–163`|
|HEAD_PICK|`operation=joints, group=head, joint_target=head_pick_joints`|已核对的任务配置 `warehouse_transfer.json:69`，当前 [1.45, 0.65]；由 owner 配置注入，不另造常数|

OPEN 是原链单独前置，不用 arm/head 准备动作冒充夹爪确认；若保留执行，也必须走同一 owner 的夹爪子动作终态与实测确认。准备段不接收调用者任意轨迹，不重启旧 Python 执行器。

## 两类规划规则的职责

`transport_skill_planner.cpp:24–32,96–117` 的具名臂动作复用 `DualArmPlanner::planSingleArm`：MoveIt 模型限位与 OMPL 有效性检查，返回 waypoint 的碰撞复查、奇异策略检查，然后原 IPTP 0.3/0.3 时间参数化（优化关闭）及速度/加速度指标检查，见 `dual_arm_planner.cpp:532–599`。不能把该最后指标说成另一次位置限位检查，也不能把 waypoint 复查说成独立连续扫掠证明。

奇异策略保留现状：`singularity_monitor.hpp:25–36` 为最小奇异值 0.02、条件数 80、`allow_singular_start=true`；允许起始奇异前缀，离开后拒绝再次进入。它不是“每个点均非奇异”的合同，也不能仅凭 success 推断终点已经脱离奇异；仿真需记录实际终点与报告。

头部支路 `transport_skill_planner.cpp:54–78` 校验目标尺寸/有限值/模型限位，以不大于 0.02 rad 的关节步长插值，再调用 `validateExternalTrajectory`。后者按模型硬限、最多 0.025 rad 的插值碰撞采样、适用的臂奇异策略及原参数化检查；它不施加 MTC 0.1 rad 内缩。

准备动作负责从实际初态进入工作姿态；MTC 的额外 0.1 rad 内缩负责抓放操作域。只有准备完成后的**新实测起点**通过原 MTC 检查，才能开始抓放。规则职责不同，不等于近限跟踪预算已解决。

## Native 接线必须闭合的边界

1. **同一控制权。** 整个准备和操作使用同一 ResourceAuthority/lease、完整 22 关节和六控制器声明；独占图、geometry/ledger 版本、底盘停止与导航撤销持续有效。不能在段间释放再申请。沿用 `hold_executor.cpp:123–130` 的原停止/新鲜度条件。
2. **规划输出绑定。** `PlanSkill.srv` 只返回 success/reason/trajectory，没有 context、实际 scene 签名或完整起点。native 调用者在请求前后关联本次 lease/阶段代次/请求序号；响应只属于该次准备。使用 owner 独立读取的完整 scene 和新鲜完整 22 关节构建每个 waypoint（非运动组保持当前实测状态，附着取真实 scene），校验轨迹首点/关节组和目标，再在副本上调用现有 `validateExternalTrajectory`。验证副本可参数化，但不能偷换原服务输出的路径/时间；实际下发仍用已验证的服务输出。原服务内部 scene 不可仅靠两次外部相同 hash 推定相同。输出后外部 world/附件/epoch/非运动关节发生实质变化应终止本次执行并明确原因，不改签名冒充已验证。
3. **每段真实结束。** 运动组使用本段轨迹，其他资源保持；沿用六子目标终态屏障和目标误差检查，随后完整 22 关节满足既有 ROS/steady 双时钟 500 ms 稳定条件（`hold_executor.cpp:432–442`）。内部稳定不发布最终 ArmHold。服务“已到目标/空轨迹”只能在 owner 实测目标和稳定也满足时跳过动作，不能以空响应直接推进。
4. **正确跟踪来源。** 当前 `execution_guard.cpp:95–97` 固定订阅左臂控制器；它不能保护 READY_RIGHT 或头部。最小接法：复用该 guard 可执行文件，为右臂、头部各启动独立命名实例，把实际 controller-state topic 与各自 set/status 端点显式 remap，native 按本段选对应 guard；左臂实例保持原接口。所有实例沿用原阈值、TF/时间规则，没有 FJT client。此处 remap 用于匹配真实反馈源，不伪装控制权。需要在 launch 和 manifest 中明确三组对应关系。
5. **取消/故障。** 服务尚未返回时取消只使该规划响应失效；它不是资源释放证明。已有执行子目标须记录真实 UUID/终态，UNKNOWN 或取消未决保持资源。旧阶段迟到响应不推进新阶段。原异常原因与收尾失败均保留，满足真实终态和稳定才按既有规则释放。
6. **头部后新快照。** HEAD_PICK 稳定后等待实际采集 stamp 晚于该稳定屏障的新 RGB-D/CameraInfo 和对应精确 stamp 的 TF；依原 CameraHealth、校准/source/clock epoch 和原期限验证，再构造 M3 Request。缓存的移动前图像不能靠重写 stamp 使用。重新取得此时完整 scene 签名和 actual start；过期就明确失败，不延长 M3 期限。

现有 `ManipulationToHold` 请求已经携带 pre_target/target 等目标；准备动作不能悄悄把移动前 M3 结果改写成“头移动后已重新观测”。首期明确目标 fixture 模式可以保留这些显式目标、在准备后重取实际 scene；真正 M3 模式必须由同一 owner 在准备完成后构造 `pick_planning_client.hpp` 的 capture/context，再启动候选。两者在调用契约/证据中明确，不临时添加含糊布尔值或静默替换用户目标。

## 实施文件和仿真验收

由 M1 修改 `astribot_s1_transport_native/src/hold_executor.cpp` 的完整 PICK 前置状态及 PlanSkill client，按现有组织方式复用场景/轨迹验收；必要依赖写本包 CMake/package。部署 launch 添加右臂/头部 guard 实例及端点参数、任务已存在的 head_pick_joints。复用现有 `PlanSkill.srv`、`ManipulationToHold.action`、`SetExecutionGuard.srv`；首期无需新服务或修改 MTC。独立 full_build/full_install，冻结首段及 M2 原安装。

实施后最小仿真回归（当前均 **NOT_RUN**）：

- 真冷启动 q4≈0、真实 EMPTY：同一 lease 按右→左→头顺序运行，逐段终态与全关节稳定，新快照后原 MTC margin 通过并执行到 Hold；不得改初始关节。此结果仍不代表完整抓放。
- 已在 ready/head 目标：无动作响应仍经过实测目标和稳定，能正常推进。
- 准备扫掠中加入实际障碍：右臂/左臂/头部至少覆盖各条实际规划支路的拒绝；原无效限位、适用的奇异策略、时间参数化失败不得被跳过。
- 代表性段取消及子目标 UNKNOWN/迟到终态；核对未决期间不释放、无下一段目标，最终只按真实终态和稳定收尾。同时核对右臂/头部 guard 确实消费对应控制器反馈。
- 规划期间改变实际 scene/完整起点/附件或 epoch，旧响应不得执行；停止条件失效或反馈过期走原故障路径。
- 头部移动前缓存采集/错 stamp TF 拒绝，移动后新采集可进入 M3；所有期限保持原口径。

恢复入口：M1 审核此文档并在其独占 native 范围施工；M2 唯一操作者在独立候选上做上述仿真。当前只完成设计与失败证据归档，MTC margin 拒绝和近限预算问题没有标为 GREEN。
