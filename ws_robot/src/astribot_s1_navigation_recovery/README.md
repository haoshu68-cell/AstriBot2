# 导航起点准入与脱困

本模块位于导航任务准入之后、目标路径规划之前。只有本模块返回 READY，BT 才调用原 GridBased 目标规划。正常规划器、MPPI、Arrival 不处理起点告警或恢复编排。

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
| DeparturePlanner | 新鲜当前位姿/地图/包络 | 保持朝向，比较后/左/右/前的安全退出，每次执行至多0.2 m | 最终目标路线、自动恢复编排 |
| DepartureController | 规划器已确认的短段路径、实测位姿/速度 | 经现有 Nav2 控制服务器输出低速命令；实测停稳后成功 | 重新选择目标、故障决策、报警管理 |

本包规划与检查依赖既有 path_tracking 的分层读取；短段控制依赖现有停止判据，正常 path_tracking 不依赖本包。插件加载于现有 Nav2 planner_server/controller_server/bt_navigator；逻辑按包划分，无新增进程和速度写入旁路。

## 几何契约

起点三维相交返回 BLOCKED；仅二维投影不足、分层几何实际无碰才进入候选退出。完整退出扫掠包含第 0 点和各高度层，未知区域阻断，原包络裕度不减少。候选完整出口满足原二维模型；每个短步不要求已经到达出口。起点只检查实际当前朝向，不假定朝目标原地旋转；完整目标规划仍独立负责后续路径可达性。

候选搜索最大2 m、步长0.05 m，优先最短分层安全出口，等长按后/左/右/前排序；一次平移至多0.2 m，速度最高0.05 m/s。按固定双臂整机预测方案，执行期间每周期重新读取有效固定包络及分层地图，对实测速度和本拍指令检查一秒扫掠。障碍相交、许可撤销或过期即拒绝继续输出；不重新选择目标或改变退出方向。动作实测停稳后仍须复检；最多10个短动作，前置过程预算120 s。

诊断和路径选择可使用尚未授权运动的完整几何；实际运动必须具有有效包络。执行仍经原 Nav2 Action/控制服务器，没有额外速度发布节点。一秒恒速扫掠不是经物理验证的停止包络，整体验收必须另测停止链路。

分层输入接受 `gazebo_collision_geometry` 或 `static_archive_occupied_endpoints_only`，所有未知单元保持阻断。前者不证明真实传感器覆盖，后者不证明未观测空间为空闲；本实现不提供动态感知融合。

## 数据接口与失效

- `/navigation/assess_start`：AssessNavigationStart，请求绑定 execution_id 和目标；响应状态 READY / RECOVERY_REQUIRED / BLOCKED / UNAVAILABLE、实际起点、采样时间、地图内容哈希、分层版本、几何哈希、包络 epoch。服务不发布运动。
- `/navigation/start_alarm`：DiagnosticArray，可靠队列 10；记录执行关联、阶段、初始/当前原因和版本。成功仅在检查或复检 READY 后发布 OK；错误返回失败，外层任务处理原有取消、保载、资源与停稳。
- `/path_tracking/departure_path` 与 `/path_tracking/departure_evidence`：退出候选诊断；实际执行仍通过 ComputePathToPose / FollowPath Action。
- 输入/查询与许可各有2 s等待窗口（steady）；输入暂失每100 ms重查。退出单步执行预算沿用120 s、无进展15 s，整体前置预算120 s；源时间与steady停稳窗口均至少 0.6 s。
- 取消通过 Nav2 原 action 链传播。BT状态机测试不能证明实际controller停止；现场需读取子动作终态、里程计与速度命令。

验证与证据索引见 [实施记录](../../../../docs/START_DEPARTURE_IMPLEMENTATION_20260926.md)。
