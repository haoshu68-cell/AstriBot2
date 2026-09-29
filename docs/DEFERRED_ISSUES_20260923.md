# 暂缓问题与难点清单

本清单不是验收通过记录。采用 [全窗口任务推进规则](TASK_EXECUTION_POLICY_20260923.md)，更新时保留原始失败与各次排查时间。共同清单由搬运仿真窗口维护，其他窗口回传自己的问题条目。

## MTC-PAYLOAD-REMAINING-PATH-01：实际附着后的收臂路径碰台

- 2026-09-25 05:02:36 恢复本问题，原失败与累计排查保留；本轮检查点 06:02:36。仅实施实际附件下单段收臂重规划与显式轨迹交接，原碰撞/时效及资源条件不变。独立构建和定向验证进行中，未完成普通全场，不记通过。

- 状态：`resumed_pending_validation`，不是通过。最早计时2026-09-25 00:26:36，原检查点01:26:36；01:23最后指定候选失败后有界收尾并暂缓，现按上方恢复轮继续，不通过改名或换窗口重计时。
- 现象：scene27的TRANSPORT_POSTURE首坏index41/788，scene31提高PICK抬升到60mm后仍在index119/840碰取货台。两次都已实际抓取并应用物理附着，但都未执行LIFT/NAV/PLACE。
- 确认事实：名义6×6×12cm物体规划与实际7.146969×7.146969×13.146969cm保守附件不同，并有实测TCP偏置。scene27完整URDF FK/角点检查显示下降18.207mm及倾斜增加竖向半高6.315mm耗尽首状态24.264mm净空；名义形体尚有6.590mm时，保守形体已重叠0.257mm。重验正确拒绝，不能缩小模型或关闭碰撞换通过。
- 已尝试：修正同手两pad合法接触（C++几何测试通过且现场接触集确认）；精确记录首坏RobotState；向内120mm端点有净空但Cartesian自碰撞，前向120mm路径不完整；最后原XY/vertical60初始规划通过，实际剩余路径仍碰台。不再扩大候选搜索。scene31独立复算确认提高后首状态净空54.264mm，首坏状态箱中心下降56.100mm且旋转增加5.127mm竖向半高，15轴SAT仍有交叠；只看端点或最低角高度不足以判断整个有限台面的碰撞。
- 剩余入口：先决定规划阶段如何使用可追溯的保守附件，或实际附着后如何显式返回并重新验证新timed轨迹。当前RevalidatePayloadTransition仅返回布尔结果/原因等，没有新轨迹字段，不能仅改缓存并让执行器继续旧路径。此为下一次有界修复的接口前置，不是本轮已实现功能。
- 安全收尾：scene31业务UNRESOLVED、domain60 journal保留；36帧/0.7s底盘零速零漂移，外层会话stopped、无自有残留。旧quarantine不清除，不能把进程清理记为资源释放。后续NAV/PLACE实际验收被依赖阻塞，扩展矩阵不作为替代进度。
- 证据：[scene31收尾](evidence/mainline_20260924/scene27_actual_payload_collision/scene31_final_checkpoint.json)、[原精确几何](evidence/mainline_20260924/scene27_actual_payload_collision/scene27_clearance.json)、[总调度记录](MAINLINE_DISPATCH_20260924.md)。最新失败录像见收尾JSON中的绝对video_path。

## MAINLINE-INTERMITTENT-01：准备及非导航瞬态观测

- 按用户要求`deferred_by_user`，不影响已保存的失败语义。scene25 INITIAL_READY_TF_STALE；scene26两条导航publisher图均为空（仅证明本探针未取得身份，不证明错误writer）；scene30 PREGRASP两帧实测角速度超0.03rad/s，峰值0.051326rad/s，7条观测cmd_vel均为零，未见命令越权证据，但不能记为误报。
- scene30取消后实测停稳且父资源释放，scene31同阈值正常复跑越过该点；不据此宣称底因修复或长期稳定。当前不查DDS、性能或动力学专项，不改期限/速度门槛。
- 证据：[观测审查](evidence/mainline_20260924/scene27_actual_payload_collision/scene30_motion_readonly_audit.json)及同目录scene26/scene30记录，均保留输入哈希与场景身份。

## SCAN-FRESHNESS-01：策略使用的扫描过期

- 当前状态：`normal_return90_reproduction_fixed_and_verified`。已独立核对导航窗口 R6 的 `result.json`、`navigation/result.json` 和 `stack/session.json`：原 `return_90` 两段均通过，`cleanup_complete=true`，会话 stopped 且无自有残留进程。
- R6 实测（仿真 odom 到位口径）：0.8 m 前进误差 0.341 mm / 0.0421°；回到原点并转 90°误差 1.164 mm / 0.0293°。每段末尾 0.7 s、36 个样本观测到零速度和零漂移。
- 处理说明：按导航窗口最新回传，移除近期策略发布端重复使用旧 scan deadline 压缩心跳的门控，恢复固定 0.3 s 策略心跳；独立最终保护仍检查最新传感器、碰撞、包络、失联及显式 HOLD。清除确认修正和 C++ 静态快照优化保留。源码及专属回归由导航窗口维护，主窗口这里只汇总已核对的正常返程运行证据。
- 验收边界：只关闭本正常返程复现的卡点，不宣称全部扫描压力场景、I0.2、多载荷、社会导航或历史碰撞归因已通过；这些任务按各自证据继续排队。本次仿真/GPU资源已经交回，当前没有新的仿真运行指令。
- 当前证据：原始目录 `/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154/scan_r6_01`；主窗口核对摘要为 [R6 证据摘要](evidence/scan_r6_handoff_20260923.json)。下列 R3 及之前的失败、投入和假设保留为历史。
- 历史重开：用户随后明确要求定位修复，本项于16:35:59重开一轮，17:35:59检查点；下列暂缓记录保留作历史。新证据与验收状态见 [修复记录](SCAN_FRESHNESS_RETURN90_REPAIR_20260923.md)。
- R3 检查点历史状态：`deferred_after_checkpoint_partial_repairs_verified`。C++ TF修复后R1两次返程通过；R2/R3发布门禁和两段期限不续期的采集wire审计通过；R3还修复了控制器临时过期永久锁存。但R3最终返程仍因120.55s起步对齐超时失败，不能以R1成功记整体放行。17:33:54仿真已安全清理，本问题不再续开排查，17:35:59同一检查点及历史投入保留。
- 历史状态：`deferred_by_user`；此前用户要求先继续其他任务。历史累计排查时间待从各窗口记录归并，不能视为零。上一轮 A4 恢复始于 15:51:54，该时间不是本问题累计起点。
- 现象：返程 90°任务超时；同场景独立观测的源扫描新鲜，但策略使用扫描出现过期。
- 本轮证据摘要：返程首段到位成功，返程任务 180 s 超时；扫描独立观测 P95 58 ms/max 69 ms，策略扫描年龄 P95 476 ms/max 788 ms，510 次 STALE。安全停车、资源释放及会话清理已完成。指标以原始批次文件为准，不将不同阶段分位数相加。
- 原始证据根目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154`，其中返程批次的 `freshness_analysis.json`、窗口 `window1_results.json` 和 `window1_cleanup.json`。
- 重开前已完成：保留冻结版本基线；原算法与默认关闭/启用的离线选帧对照各 12 场一致。当时R0尚未部署；当前R0实际加载及分段指标见修复记录。
- 重开前已知/未知：源接收与策略消费之间存在时效差；当时未确认内部各阶段贡献。当前已确认主要等待边，但不能把外部TF可查或单个候选视为全部根因闭环。
- R3 时影响：该返程场景及依赖持续有效扫描的动态导航不能验收；不阻塞静止载荷接线或独立视觉接口工作。
- R3 时剩余卡点：原始采集deadline保留后，短正向租约之间存在间隙，保护反复重置清除确认窗口，返程ALIGN_START难以推进。R3中POLICY_UNAVAILABLE 2807、PROTECTION_CLEAR_CONFIRMATION 8317为诊断采样计数，非独立故障次数。
- R3 检查点提出的后续实验：按R3冻结安全链关联逐条剩余租期、下一条间隔、snapshot/risk/start_maneuver耗时和确认重置；据同场证据减少主耗时，再补取消/ACK失联及多场景恢复。不能用延长300ms预算、改stamp或删除确认窗口换成功；R0缺陷已修复。导航窗口17:38后转独立质量验证脚本离线修复，未重启本问题仿真。
- 恢复条件：导航窗口独占仿真并与视觉窗口交接 GPU；300 ms 时效门槛保持原值。

## SCAN-DIAGNOSTICS-01：默认关闭的 R0 诊断候选审查项

- 当前状态：`fixed_and_loaded_in_r0`。两项缺陷均已先复现再修复，12项C++检查和3项实际绑定检查通过；绑定SHA `a7da70ec26936779e9851c9e5c2aaec210666858a69ee8a7d79f456982350eda`已在`scan_r0_01`实际加载。诊断仍默认关闭。
- 现象一：C++ 按 256 字节直接截断中文等 UTF-8 文本，pybind `snapshot()` 可能抛 UnicodeDecodeError；启用诊断前必须修复并验证完整绑定路径。
- 现象二：记录器 reset(1) 后仍可接收 epoch 0，形成顶层纪元与 successful 记录纪元不一致；这是诊断准确性问题，不是运动授权逻辑。
- 入口：`ws_robot/src/astribot_s1_navigation_policy_native/include/astribot_s1_navigation_policy_native/scan_timing.hpp`；已有 9 项 C++ 检查不足以覆盖上述两项。
- 已完成最小实验：实际绑定中文截断保持完整UTF-8码点；reset后旧纪元receive计入拒绝，不污染successful。证据为`runs/joint_acceptance_20260923/scan_timing_fix_20260923_1637/result.json`；未将非UTF-8 Python孤立代理字符支持混入ROS有效字符串验收。
- 计时：审查复现发生于本轮 15:51:54 后；精确本项开始时间未单独记录，标未知，不虚构耗时。

## FP-BACKEND-01：FoundationPose 后端镜像与依赖就绪

- 状态：`deferred`，视觉窗口已在检查点收尾并转S0独立离线任务，GPU交还导航；本清单仅汇总其回传。
- 计时：已存最早失败证据为15:38:27，替代此前约15:40的粗略估计；主动排查累计时间未完整单列，标未知。15:57:25–16:08:28资源交接和镜像传输/解包另列；本轮约16:36停止后端工作并交接资源，保留历史。
- 已确认：Docker/Toolkit安装、容器GPU可见、官方模型/样例/CAD和15项CPU检查完成；固定基镜像下载通过，两个ONNX模型完整语义/输入检查通过。尚未构建最终后端/engine或执行模型GPU推理。
- 待解：TRT 24.08 基镜像 TensorRT 10.3.0.26 与固定 common 要求的 10.3.0.30 不同，必须统一最终构建/运行版本后才生成 engine。
- 证据：`docs/FOUNDATIONPOSE_P0_PROGRESS_20260923.md`、`docs/evidence/foundationpose_p0_20260923/backend_checkpoint_1636.json`；v1检查器因tmpfs noexec失败的记录保留，v2通过不覆盖旧记录。
- 可独立下一项：装配 S0 几何/匹配检查或感知契约工作。容器显卡可见不代替 FoundationPose 推理验收。

## 协调状态

- 2026-09-23 16:28 左右：三个活动窗口均已确认执行新规则；FoundationPose 和导航窗口已回传各自问题清单。
- 载荷质量节点已交付精确二进制路径与 SHA256；下一验证要求读回来源身份、实际加载版本，避免 geometry-only EMPTY 兼容路径冒充真实 Consumer 接线。
- 静止载荷接线本轮导航侧 16:19 起复核；这只是本轮区间，先前 A4 相关投入另行保留，不能作为新问题清零。
- 视觉窗口17时段回传S0截面子项完成，Git `62e00dac`；6形状×3级间隙、27份DXF独立读回、18配对标称间隙及7几何边界测试通过。报告`docs/ASSEMBLY_S0_PROFILE_VALIDATION_20260923.md`。这不是3D/夹爪扫掠/IK/接触验收；圆件→六边孔及切角件→矩形孔存在可容纳错配，后续必须补物体实例→模型→指定槽位→槽位版本门禁。保持无GPU/ROS/下载/重型构建，资源继续归导航R2。

## NAV-COLLISION-08：历史 FOLLOW_POLYGON_SWEEP_BLOCKED 归因

- 状态：`deferred`。历史批次 08 的具体碰撞拒绝原因尚未复现确认；14 项 C++ 诊断测试已通过，但最新返程批次实际加载诊断后产生 0 次 COMMAND_SAFETY_REJECT，不能替代历史问题归因。
- 证据：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_140545` 与同级 `I0_2_navigation_20260923_155154`。
- 后续最小实验：冻结历史场景、实际包络/地图/姿态与二进制，取得该拒绝发生点的轨迹扫掠和几何证据；保持碰撞门槛，不把无拒绝的新场景认作历史修复。
- 累计排查时间未完整单列，标未知；由导航窗口在重开前归并。

## INFRA-RSP-01：RSP 参数服务首回复丢失

- 状态：`underlying_cause_deferred_with_verified_mitigation`。bounded URDF 有界重试已在 3 次冷启动验证，其中 2 次首回复丢失后第二次恢复；启动永久等待已缓解，DDS 首回复丢失底因尚未确认。
- 历史始于 9/22，累计耗时未知；不因更换会话重开无目标排查。
- 证据：上述 `I0_2_navigation_20260923_155154` 及 `docs/evidence/joint_acceptance_20260923/a4_resume_155154` 的启动 helper / preflight 检查日志。
- 后续最小实验：保存精确服务请求与响应日志，在独占场景下核对响应路径；不能凭 Docker 后安装或共享仿真曾存在就推断根因。

## NAV-VISUAL-01：横穿场景完整视觉证据不足

- 状态：`evidence_pending`，尚非已证实算法故障。终点截图行人部分出画，不能确认全身脚部接地和全图动态障碍清除。
- 后续：由导航窗口在不改变原验收口径下补完整视角及对应时序证据；单张截图不足以证明全程状态。
- 目前未记录独立排查起点，不虚构已耗时；可在依赖满足的普通场景验证中补齐，不必先等待扫描问题破解。


### 2026-09-26 17:07 主线起点恢复前置：front55 暂缓

16:07授权本轮起步受阻实现，累计至1h后暂缓进一步排查；未重置此前同问题历史。实现/离线回归已通过，实场完成抓取后在目标规划前调用起点评估，得到RECOVERY_REQUIRED；约10 ms后外层BT报ENVELOPE_EXPIRED，未后退/未PLACE。原失败保持，随后另证实同任务资源RELEASED且停稳。现场保持front55/domain88，录制已收尾；无新目标。

已做：独立C++ recovery模块、前置CHECK/RECOVER/RECHECK、删除并归档旧规划后自动后退链、构建与4+7项CTest、2配置检查、368 profile差分通过。未做：定位本次外层包络过期真实原因、实际退出/复检/目标规划/PLACE闭环。不得放宽过期阈值声称通过。

证据/恢复入口：[实施记录](START_DEPARTURE_IMPLEMENTATION_20260926.md)；`runs/mainline_20260926/start_recovery/front55_verification_summary.json` 与其中case的完整bag、rosout和晚到资源释放/停稳证明。先按CHECK 17:05:18.487及assessment 17:05:18.492对齐geometry/envelope/ack，排查外层17:05:18.497过期；当前仅知道拒绝原因，不知道根因。

## NAV-START-20260926：短段执行期间重复准入已删除，实际运动验收待续

- 同一问题保留18:52–18:55开始窗口，按最早18:52计一小时，不因三个窗口分工或用户细化短段语义重置。19:52进行有界交接。
- front61已完成PICK并进入RECOVER，选定BACKWARD 0.10m并发出FollowPath；helper包络时间分支报NAVIGATION_ENVELOPE_INVALID，未观察到非零底盘命令或实际后退。归因细节见`runs/mainline_20260926/recovery_loop_1852/front61_failure_analysis.json`，具体回调时差未直接记录。
- 根据用户要求，已删除helper绑定后的Envelope/ACK重复准入、恢复BT的RequireNavigationEnvelope、DepartureController的持续地图/包络/策略检查、policy新增的恢复分层分支。保留单次路径准入、实际反馈、取消/停稳和停后复检，普通导航检查不变。
- 已通过恢复5组CTest、helper18项隔离ROS测试、普通policy5项ROS回归与v4六个产物/包选择核验；这些不是Gazebo运动验收。当前仿真仍运行冻结v3，资源RELEASE_CONFIRMED，0.7秒实测停车通过，本次录制已关闭；没有替换活动安装。
- 后续仅做最小运动验收：将`deployment_candidate_v4/runner_environment.bash`和`manifest.json`中的真实产物应用到归属明确的当前验证会话，再核对实际加载，验证短段位移→STOPPED→刷新RECHECK→READY→普通规划。未通过前不标搬运主线完成，不扩展矩阵，不反复重新PICK。当前运行器无独立复用带载姿态的正式入口，需先核对现有任务入口能否复用；不能以手工发速度或伪造账本绕过。
- 证据根目录：`runs/mainline_20260926/recovery_loop_1852`，最新汇总`verification_checkpoint.json`。旧程序已归档删除见`docs/OBSOLETE_RUNTIME_CLEANUP_20260926.md`。


### 2026-09-28 主线恢复验收更新（保留原排查计时）

front64 已实际完成后退约10.04cm→STOPPED→RECHECK→START_READY，原“未观察到后退”状态已被本次运动证据补齐。随后普通规划原路径 segment=30 包络碰撞，未PLACE。邻近地图重放指向放置台近缘采样余量不足；后移停靠目标10cm离线扫掠通过，线上原失败路径未保存，不能称逐点复现。

front65验证后移目标时，在PICK的TRANSPORT_POSTURE阶段出现UNEXPECTED_MOTION_OUTSIDE_NAVIGATION而取消，尚未到导航；原停稳窗口超时，后续独立只读窗口已确认RELEASED和实测停稳。该次不标通过，按用户偶现问题留存要求不扩大问题矩阵。两次录屏已收尾。下一入口为`docs/MAINLINE_LATEST_TIME_ACCEPTANCE_20260928.md`及`runs/mainline_20260926/latest_time_contract/orchestration/front65_retained_stop.json`；不得重置历史起点/时间链路计时，不重复降低已冻结阈值。

### 2026-09-28 scene73：感知脱困实现已验证，主线被左腕跟踪保护阻断

- 感知受阻问题沿用09:19 UTC起点，未因新增恢复入口重新计时。已接通同一导航会话的RECOVER→停稳→同源感知短步复查→实际位置新规划→RESUME，C++几何/ROS服务/BT夹具通过；不能作为Gazebo主线验收。
- scene73已接收完整搬运任务，但在PICK/TRANSPORT_POSTURE结束前中断，未提交导航。observer直接状态确认`MANIPULATION_TRACKING_ERROR:astribot_arm_left_joint_7`，joint_stamp=152.270s，触发误差0.0532918rad超过0.05rad；JTC同期峰值0.0585573rad。不是SLAM漂移或时间戳拒绝。跟踪误差的机械/控制底因尚未排查，按用户偶发问题留档要求不扩大矩阵、不放宽限值。
- 父任务终态、资源RELEASE_CONFIRMED、SLAM停稳与自有整栈退出均确认。867帧录像校验通过，bag封存返回0，observer覆盖终态且无写入丢失。
- 恢复入口：`runs/mainline_20260928/workstation_alignment/policy_obstruction_recovery/STATUS.md`、`scene73_guard_evidence.json`、`scene73_command.json`。先处理/复验左腕运输姿态跟踪，随后同一导航环境重跑主线并实际观察感知受阻恢复链；尚未完成完整搬运/放置。保持`idle_position_hold=true`、`idle_position_kp=3.0`，不恢复非导航SLAM位移取消或世界刚性固定。
