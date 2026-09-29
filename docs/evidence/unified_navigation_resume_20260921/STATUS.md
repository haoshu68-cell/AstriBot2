# 当前优先级：先贯通正常流程，非主线问题延后

按用户最新要求，正常主流程通过后继续 I1 服务入口整合，同时审查窄通道实际臂展；不再扩大 I0.2。30%横向/航向及平均位姿速度偏差已按用户口径接受，原始结果保留。

导航主线为：启动并取得定位/地图 → 下发目标 → 正常跟踪/必要转向 → 到位停车 → 完成下一目标。协作搬运主线为：正常抓取 → 实际附着确认 → 运输姿态 → 短距搬运 → 放置/脱离 → 完成。两条流程不以所有扩展矩阵通过为统一前置，只检查各步实际必需条件。

当前正常返程已修复并通过仿真：按用户要求撤掉新增的旧扫描剩余期限门控，恢复原 0.3 s 策略心跳；实际碰撞、输入失效和显式 HOLD 保留。R6 前进 0.8 m 到位 0.341 mm /0.0421°；返回原点并转向 90° 到位 1.164 mm /0.0293°，均确认停稳。最终保护 6274 条正向约束的输入期限审计通过。此次只验证正常两目标流程，不将 I0.2 或社会场景全项记为完成。详细见 [返程修复](RETURN90_REPAIR_20260923.md)。

延期项：暂停/恢复、消费者失联及多类故障注入、静止载荷质量扩展验证、全套社会场景、30%内质量微调、额外视觉证据、已缓解启动问题的深层根因。若某项在正常流程中直接导致失败，再报告具体阻塞与最小修复，不自动扩展。

载荷验证工具离线25项通过，最后复核已停止，尚未运行验证；不作为正常抓取流程的总前置。

协作搬运任务已确认：task08 正常抓取—附着抬升—两段携物导航—放置—退回收臂 SUCCEEDED；随后 normal_grasp_video_20260923/task01 同仓库演示复跑也 SUCCEEDED，协作方报告放置误差 0.116 mm，所属仿真已完成停稳与清理。这是 legacy compatibility + Gazebo 运动学附着的正常链，不等于 fixed_v2 完整抓放入口已接通；后者仍缺 CPP_TASK_HOLD_EXECUTOR。以上为协作方实测回传，未混为本窗口独立抓放验收。

本轮窄通道确认并修正长臂进出转角的计划/执行不一致：新增 C++ 连续转向外形范围计算；确定不可行的路线转角立即返回失败，不消耗等待预算。19 项 C++ 检查和 16 项策略回归通过；配套独立构建及普通 fixed_v2 往返通过（0.943 mm / 0.0562°、1.207 mm / 0.0419°），这不是完整臂展通道专项通过。实际双臂、携物、不对称偏移、入口姿态调整和完整退出逻辑见 [臂展窄通道方案](../../NARROW_PASSAGE_ARM_SPAN_PLAN_20260923.md)。I1 的单服务接线及 7 项离线入口回归已通过，无人 off/H2 与横穿共 3 场、6 次到位通过；最终合并候选 fixed_v2 往返通过且已停稳清理。I1–I6 整体未完成，详见 [本轮记录](I1_ENTRY_AND_ARM_SPAN_20260923.md)。

只在完成一段正常流程、遇到直接阻塞、需要用户决定时汇报；复用现有结果和会话，同一修复通过相关检查后即继续下一步。共同任务规则由搬运任务维护，保持单一仿真操作归属。

以下均为历史进度，旧截止及“当前”措辞只代表当时状态。

# 最新推进决定：扫描时效暂搁置，继续载荷集成验证

用户经协作任务明确“扫描源过期先过。继续其他任务，后续再排查”。保留返程失败和时效证据为待查，不改300ms门槛，不写成实测PASS。R0仅有默认关闭源码/离线证据，未接入旧受控安装，取消其本轮冷接和返程复测。下一步按正式质量节点接线→static EMPTY及ROS冻结撤销恢复→kinematic四夹具准备和质量准入顺序；待交付/FP资源窗口结束后由本任务独占运行。仍沿用15:51:54–17:21:54总预算。

# 当前进展：I0.2 恢复启动通信与消费者失联验证

协作任务获用户续作指令，本轮共用15:51:54–17:21:54窗口，子步骤不重置。新入口只在原N4安装末尾选择既有bounded URDF修复库，DDS、六相机、运动/物理参数与旧质量节点不变。实际Gazebo maps确认bea2defb库；首个RSP参数回复超时再次发生，第二次有界读取成功，仿真已就绪。当前consumer_pause_01运行中；原90°返程问题仍待新场验证。计划16:25前清理后交还FoundationPose的15分钟GPU时段。

16:08实测更新：三次冷启动就绪，其中两次首回复丢失后有界重试成功；实际库maps已核对。consumer_pause_02通过（触发至停稳5.8115cm，恢复同epoch，到位0.5863mm/0.03015°）。return90_01首段通过、返程180秒超时；无具体碰撞拒绝，返程独立scan源P95 58ms且TF全可用，策略scan龄期P95 476ms、510次过期HOLD。三个所属栈及显示已清理，16:08交还FP。后续先C++ R0分段计时诊断，仍保持原截止17:21:54。

当前证据：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154`。以下保留前轮记录。

# 当前进展：共同主线 I0.2，按时间上限收尾暂停

本轮14:05:45–15:35:45，保留原90分钟上限并提前预留收尾。用户已明确横向、航向及平均位姿速度偏差30%以内接受：三对off/H2共6场、12次到位通过；15项质量/一致性检查通过；L/U四场、12个转角窗口按新口径接受，原始失败保留，控制/停车阈值未改。

N4正式EMPTY、当前实测姿态保持、六消费者ACK的直线首段两次通过。独立“撤销保持→实测停车→新epoch恢复”通过，触发后位移1.677cm，恢复到位1.151mm/0.0490°。90°返程仍失败；消费者失联补测10/11均在导航前遇到模型参数回复超时及物理不推进，未执行注入，不列为PASS。

新增C++具体拒绝日志独立构建14/14通过；11未进入导航，尚无其闭环拒绝证据。拒绝分支日志仍在地图锁内，有额外输出耗时，不声称时延等价。详细修复顺序见 [N4时效方案](../../N4_PERCEPTION_TIMING_REPAIR_20260923.md)。非home/有载导航、最新质量绑定的运行接收、I0.2整体验收及I1–I6尚未完成。

本轮21个所属session均结束且remaining为空，/proc所属运行进程为0，专属显示:96退出；已通知FoundationPose任务释放维护/GPU窗口。没有操作真机。共同chassis-effort-drive基线保留，验证工具提交579b69c1、b087aeb2；诊断精确补丁留存在运行证据中，不整文件提交其他任务的改动。

当前证据目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_140545`；机器可读汇总为 `current_regression_evidence.json`，清理为 `current_cleanup.json`。

以下为按时间保留的历史进度，不作为当前状态。


# 2026-09-23 C96 核心通过，进入全仓集成逻辑检查

C2 16/16；C3 三项停止与四阶段横穿等待/实际停稳/恢复共 7 项核心通过；C4 保存 86 对 Gazebo/RViz 画面并检查首、中、末帧，限制见 C96 报告。按用户最新要求进入全仓集成，再开展 I0.2 合并构建。fixed_v2 过期用例因初始保持/几何/消费者确认未建立而未注入，延期验证；C1 扩展重复及 D 完整矩阵仍未测，均不写成 PASS。本轮所有所属仿真与显示进程已回收。

# C95：隔离仿真回归已收尾，C未验收

## C96 当前推进决定（2026-09-23）

用户暂认为 C1 当前候选通过，允许进入 C2。这是阶段推进决定，C95 尚缺 26 个标准矩阵样本及完整不退化证据继续保留，不改写为实测 PASS。C94 控制器 + C95 独立轮驱动候选的 C2 复杂路径/边界回归已 16/16 通过。当前运行 C3 核心触发、停止和恢复场景，并安排成对 Gazebo/RViz 观察。按用户最新决定，确认这些核心逻辑后进入全仓集成检查与后续 I0.2 合并构建；扩展组合仍单列未测。

本次剩余 C 任务统一计时 **05:40:41–07:10:41 +08**，含排查、重试、收尾。真机未开放，主线运行代码/共享安装未替换。当前目录：[C96 证据](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C96_20260923_054041)。以下 C95 段落保留历史状态。


本轮从2026-09-22 23:48:43开始，按90分钟上限在截止前收尾。原C94控制器加原轮驱动的标准矩阵19/36通过，1场实际失败，16场未执行；诊断又复现一次低速终点无进展。定位到低速反向时相反积分抵消驱动力矩的贡献。

已制作独立C++轮驱动修复候选，未改速度/精度/超时/物理参数。候选12/12场仿真成功，最大到位误差1.431 mm / 0.0531°；相当于新版本10个标准矩阵场次和2个控制场次，尚缺26场矩阵。效果不退化和C2–C4未完成，故未合入主线/共享install，不进入D、I0.2、I1–I6或真机。

[本轮报告](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/unified_navigation_resume_20260921/C95_SIMULATION_REGRESSION_20260923.md) · [剩余任务与边界](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C95_20260922_234843/NEXT_STEPS.md) · [收尾审计](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C95_20260922_234843/final_process_audit.json)。本次自有仿真/显示进程已退出，其他会话未清理。

下方为历史记录，不代表当前候选验收。

---

# 统一导航任务链：C91 按90分钟预算暂停

本轮计时 **2026-09-22 13:41:42–15:11:42 +08**。C1–C4 为同一任务；I0.1 **NOT_PASSED**，D、I0.2、I1–I6及真机未开放。

最终候选 **v13** 修复了终点接管跳过未完成路段、位姿跳变差分速度自我放宽阈值，以及零弧长原地转向不能进入精调的问题。独立构建、CTest 7/7及独立静态复核已完成。原运动/制动/精度参数未改，到位提速仍单列待验证。

当前逐项统计：`{'PASS': 14, 'CANCEL_PASS': 1, 'INVALID_PEDESTRIAN_STATIC_CLEARANCE': 4, 'CROSSING_UNACCEPTED': 2, 'EXPECTED_UNSUPPORTED_REJECTION': 1, 'FAULT_STOP_OBSERVED': 4, 'INFRA_FAILURE': 3}`。PASS为基础控制用例，不代表C整项验收；CROSSING_UNACCEPTED来自行人实际轨迹静态净空证据不足。旧四场横穿因行人终点净空不足而作废。基础设施失败不计入控制器成功率。

- [本轮报告](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/REPORT.md)
- [逐项结果](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/v13_summary.json)
- [后续清单](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/NEXT_STEPS.md)
- [Gazebo/RViz双视图审查](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/v13_cross_clear_ALIGN_CORNER/visual/review.json)
- [实际行人净空问题](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/crossing_actual_clearance_issue.json)
- [进程归属审计](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/final_process_audit.json)

下方仅为C90历史，不能计入当前候选验收。

---

# C90 历史记录（90 分钟暂停）

本轮从 2026-09-22 09:53:30 +08 开始，截止 11:23:30；重连未重新计时。已预留收尾时间停止本轮仿真并完成归属审计。I0.1 仍为 **NOT_PASSED**，D、I0.2、I1–I6 和真机未开放。

| 工作 | 状态 |
|---|---|
| A：冻结基线与证据口径 | 已完成，历史报告范围 |
| B：感知所有权与启动 | 已完成，限感知/启动验证范围 |
| C：标准角点与多场景边界 | 本轮预算结束暂停，未验收 |
| 后续阶段和真机 | 未开放 |

当前候选 v10：独立构建和 7/7 CTest 通过；16 个控制器场景通过，4 个取消场景确认停稳。已通过场景的终点误差最大 1.770 mm / 0.0829°，来自 Gazebo 真值，不能外推真机精度。标准角点 ±45/90/135 首轮通过，−45 三次通过；同候选全矩阵三次重复尚未补齐。

明确未通过：近 180° 回头路径提前终点接管而漏走路段；等价重规划重新进入角点对齐；两次位姿跳变由碰撞保护而非预期不连续检测拦截；RViz 第二组截图模型/轮廓分离尚未定位。自交长路线在脚本墙钟时限内未结束，作为未完成记录。包络场景的 fixed_v2/off 参数组合错误和一次参数服务超时分别记录为基础设施问题。

到位精调提速已形成单独候选：上限 0.08 m/s / 0.20 rad/s，当前默认仍为 0.06 m/s / 0.15 rad/s，尚未应用或部署。先修 C 的确认问题再做对照。

- [本轮完整报告](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C90/REPORT.md)
- [逐项数据及判据](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C90/case_inventory.json)
- [后续修复顺序及边界方案](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C90/NEXT_FIXES.md)
- [轨迹对照图](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C90/completed_corner_trajectories.png)：不同候选的诊断对照，不作为同版本完整 A/B 验收。
- [运行日志](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C90/v10_warm_on/stack/session.log)；latest_sim 只作索引。
- [归属进程审计](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C90/final_process_audit.json)：本轮仿真和独立显示残留为零，其他会话未清理。

运行时改动为 C++。共享 install 未覆盖、未操作真机；源码与已测试 v10 快照的相关文件哈希一致。旧 v8/v9 的失败与原一小时记录保留为历史证据，不计入 v10 通过次数。
