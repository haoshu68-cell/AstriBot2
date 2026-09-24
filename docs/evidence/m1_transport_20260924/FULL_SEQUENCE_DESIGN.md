# M1 完整操作的最小接线设计

状态：总调度 2026-09-24 已认可顺序和独占同节点方向；下列尚未接线部分不是已实现能力。首段基线 `7fafe827` / `install` 和修复 `b041e530` / `next_install` 均保持冻结；后续完整链只写独立 full_build/full_install。

## 目标及入口

同一 `task_trajectory_executor` 保留 `/transport/plan_to_hold`（只执行首段）并新增 `/transport/manipulate_to_hold`，类型 `ManipulationToHold.action`。请求复用完整 PlanManipulation 的具名目标字段及 task/request/context 身份；没有外部任意轨迹、stop_index 或改变旧 Action 语义的 bool。两个 Action 共用唯一 ResourceAuthority、canonical lock/journal 及 active task。全阶段完成后仍持有资源并输出 Hold；不是自动授权底盘导航。

内部 `consume_plan` 复用 MTC 原始完整结果。后续 M3 同进程库接入必须传入原始绑定上下文/来源期限，不能将其结果重新规划成另一个 context。首期实施明确目标普通 PICK/PLACE；不提前开放未完成的 M3 适配。

## 必要准备前置（2026-09-24 追加，待设计审查）

真实第三场已通过冷启动撤销及图独占检查，随后在 MTC 起点余量准入失败。总调度追溯原 `core.py`：PREGRASP 之前有 READY_RIGHT `transport_compact`、READY_LEFT named `ready`；历史 task08 的 PREGRASP 起点 q4≈0.99998，而新首段直接从 q4=0 规划，低于 0.1 rad 余量对应的 +0.04 下界。头部 `head_pick` 观察姿态也属于真实感知前置，不能将未朝向台面导致的 0 点判成物体算法失败。

完整链必须在**同一 ResourceAuthority** 下显式执行准备姿态和头部观测姿态，再进入快照及 PICK/PLACE 计划。复用现有 PlanSkill 几何校验；实际控制器样条、近硬限起点及原预算仍需专项审查。不得删除 MTC 余量、用未经证明的“单调入域”绕过 guard，或重启旧 Python 执行器争夺控制权。总调度负责该准备链设计审查，结论确认后本包统一接入；本次 payload client 修改不扩大冻结首段行为。

## 逐阶段事务

| 操作 | 阶段顺序 | 允许推进的真实证据 |
|---|---|---|
| PICK | PREGRASP → GRASP_APPROACH → GRASP_CONFIRM → ATTACH_CONFIRM → LIFT → TRANSPORT_POSTURE | ARM/GRIPPER 各段唯一 UUID 成功终态、实际到位和连续稳定；附着须物理命令已应用、完整新库存、保守场景提交及独立读回、账本和几何同版后确认 |
| PLACE | PREPLACE → PLACE_APPROACH → RELEASE → DETACH_CONFIRM → RETREAT:原方案分支 → STOW | 同上；解绑后单对象放置必须有实际落点及再次采样稳定；全库存 EMPTY 是独立后置，不能由一个对象 PLACED 推导 |

每个阶段开始重新检查同一租约、计划上下文、实际全部资源关节起点、附着 ID/几何、场景和实测底盘停止。ARM 用现有执行 guard；当前 guard 只接左臂控制器状态，不能把 GRIPPER 关节名送进去冒充夹爪跟踪保护。所有阶段只按实际事件推进一次，不因规划成功或取消 ACK 推进。

上段 child context 只有在终态屏障和实测稳定后才可更换；回调同时绑定 lease、阶段序号及 UUID，旧阶段迟到反馈不得推进新段。内部段间稳定不发布最终 ArmHold；最终姿态确认之后才生成新的 hold ID、保持授权，交 M2 新包络 epoch / 六 ACK。任一未决子动作、物理命令或 scene 事务阻止资源释放，必须留存持久化记录和具体原因。

## 场景变化分类

1. 未预期 world、ACM、其他附着物或元数据变化：明确拒绝。
2. 只有 occupancy 变化：复用原 RevalidateManipulation，绑定原 context/start_index，稳定 1 s（最长 15 s）后检查原剩余路径；响应后独立完整场景读回须仍相同，不重新规划、不刷新计划 120 s 寿命。
3. 明确受权的感知提案提交、ATTACH、DETACH：分别属于带版本和原因的事务，不能当作外界变化忽略，也不能直接将旧场景 hash 改成新值冒充验证。

实际物理库存使用膨胀保守碰撞体和其自身 touch_links。正确顺序为 **物理命令应用确认 → 完整物理库存 → 将该观测对象提交 PlanningScene → ledger 独立 GetPlanningScene → 新 geometry**。不能写原始 MTC BOX/pad touch_links 假装与物理库存相同。

外部接口已于 `0b356d39` 完成离线交付，M1 待接入：`/transport/revalidate_payload_transition` 接收原 context、transaction_id、start_index=4 和完整实测 PlanningScene；仅允许对应缓存 ATTACH_CONFIRM/DETACH_CONFIRM 的本物体转换，重新校验实际保守几何下的原剩余轨迹和接触策略，并更新缓存及轨迹 waypoint 附着体。它保留原 context、120 s 期限和原路径，不替代物理确认、账本、几何及独立场景读回。相同 transaction_id 仅对同一载荷幂等；响应后仍须再次独立完整读回。原 occupancy-only 服务不能用于第 3 类转换。当前是服务离线验证通过，不是 M1 接入、实际附着或完整搬运通过。

## 感知期限

M3 原采集 +5 s 是候选准入截止，不是整个搬运任务的寿命。过期候选不能启动新的依赖该候选的接近/闭合段；需要新观测、相同实例/场景证据并重验，不能只改时间戳。物理 ATTACH 经完整新账本/几何确认后，载荷事实由新来源接管，不靠抓前点云续命。实际阶段耗时未测，当前不宣称此期限接线已经闭合。

## 最小验证与剩余工作

- 保持已验收首段协议基线、旧 HoldResources 取消屏障回归；旧入口的 pending_cancel 实测通过（domain177，末子终态到释放 0.644 s）。
- occupancy 同 context 通过、重验拒绝、响应错 context、成功响应后读回再变四项必要协议已在首段修复候选通过。原基线正常 occupancy 的 RED（MTC_SCENE_CHANGED，无 JTC）保留在修复证据中。
- 完整正常 PICK 和 PLACE 的阶段/UUID/事件顺序、同对象保守几何转换、最终 Hold 是下一验收；当前 NOT_RUN。
- M2 在冻结首段 overlay 上独占真实仿真；本任务在其测量窗口不重构建。没有以离线/模拟返回值替代真实非 home、实际物理附着或完整运输验收。

问题起点仍 09:51，10:51 为统一状态检查点；首段关闭与后续正常开发分别记录，同一持续未解排查不换名重置。当前占据重验行为差异从该主问题派生，失败与环境导入错误均保留。
