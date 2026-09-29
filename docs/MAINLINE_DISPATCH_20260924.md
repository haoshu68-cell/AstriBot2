# 主线统一调度记录 — 2026-09-24

## 当前恢复（2026-09-25 06:01）

用户明确继续主线、固定已验配置、不扩大矩阵。按任务编排/C++/仿真归属技能执行。本轮仅定位scene38的附着前场景重验变化：M1独占执行器失败快照诊断，root独占构建/唯一仿真，M5只读冻结配置和实际录屏验收。idle_position_hold=true、kp3、已授权仿真angular profile、2.5倍轨迹时间、原场景及相机基线冻结。

该场景一致性问题首次观察05:56，检查点06:56；原payload路径问题00:26及05:02→06:02计时/失败保留，不借此次诊断重置。scene34证明新实际附件后缀已接纳、LIFT成功，但TRANSPORT因原底盘位移限制取消，后续NAV/PLACE未验。scene38三抓取阶段已确认，未实际附着。先取得本次比较双方快照中的真实差异再修正，不以猜测改变碰撞或时效语义。分层地图方案保留，主线验收前不施工其扩展。

成功标准仍为当前导航仓库同一父Action完成PICK→有载NAV→PLACE→权威EMPTY及独立Scene→终态、实测停稳、资源释放、可解码录像。单个诊断/阶段通过不得替代此验收。

- 06:09–06:11：scene39完整PICK已成立：PREGRASP、GRASP_APPROACH、GRASP_CONFIRM、实际ATTACH确认、LIFT及TRANSPORT_POSTURE均完成；新后缀实际采用。随后代码在确认Hold的tick中先发送导航包络请求，末尾才发布正向Hold；实际104.508s为false、104.543s为true、104.562s又为false，SetFixedEnvelope返回ARM_HOLD_UNCONFIRMED。journal issued_at是租约快照而非事件独立时刻，不据其计算交接延迟；未发送Nav目标。父任务cancel/resources_released=true、独立停稳通过、stack stopped/remaining[]。scene38场景变化本轮未复现，按用户“偶现问题暂过”留档不再分析，不记修复。只定位正常PICK→NAV交接发布/服务顺序，起点06:09/检查点07:09；不重置任何旧问题计时。固定配置不变，分层地图等扩展继续冻结。

## 恢复检查点（2026-09-25 05:02:36–06:02:36）

- 按用户继续主线与既定建议顺序恢复 `MTC-PAYLOAD-REMAINING-PATH-01`；原 00:26:36–01:26:36 失败及累计排查保留。本轮恢复检查点 06:02:36，不以子任务或窗口重置问题计时。
- 仅修复实际附件导致原 TRANSPORT_POSTURE 碰撞：保持 LIFT 和原目标，使用实际保守附件重规划此单段，显式回传剩余两阶段；执行器在独立 Scene 读回和原准入通过后原子接纳。10 秒服务、30 秒附件事务、120 秒计划寿命及碰撞/速度门槛不变。
- M1 负责 MTC 服务及定向测试；root 负责执行器、接口、独立构建和唯一仿真；M5 负责返回/接纳/实际 FJT 下发轨迹指纹与原视频验收；协作代理仅补 native 原子接纳和指纹单元测试。其他矩阵冻结。
- 05:18 完成独立构建及 6 组 62 项 C++ 定向检查；M5 轨迹交接 17 项、原几何 7 项离线检查通过。初次 native 测试误加载旧动态库的失败日志保留，正确安装路径下复测通过。05:21 放行唯一 scene32/domain59，普通单箱 PICK → NAV → PLACE → EMPTY/Scene/停稳/释放及可解码录像尚待结果。新消息、MTC、native 均使用独立候选安装，不覆盖旧运行依赖。

- 05:54：按用户最新指示固定 idle_position_hold=true，源码默认值、私有配置覆盖及实际读回一致；kp仍3.0，已写入AGENTS长期规则。用户另授权本轮仿真角位移0.05rad/非导航角速度0.10rad/s，原平移/线速度及碰撞、时效、到位、最终停稳门槛不变；默认配置不启用放宽profile，非仿真拒绝。MTC/新native/底盘独立构建通过，guard1组、native4组、底盘2组CTest通过。scene38/domain53唯一图形仿真已启动，Gazebo/只读RViz均可见，真实底盘binary和参数读回已证；完整PICK→NAV→PLACE待验，不作为原严格条件验收。原06:02:36检查点不重置。

- 05:56：scene38实际读回idle=true/kp3、executor/guard relaxed=true；执行至GRASP_CONFIRM后MTC_SCENE_CHANGED_DURING_REVALIDATION，未到实际载荷后缀接纳/NAV/PLACE。父任务canceled/resources_released=true，36样本/0.7s独立零速零漂移停稳成立；stack stopped、remaining[]。配置固定要求已完成实际验证，主线仍未通过；M1仅只读核scene变化触发点，M5复核阶段与视频。

## 原始调度记录

用户已授权本窗口统一调度现有或新任务，并按主线逐项施工验证。依据 [主线审查](MAINLINE_AND_EXTENSION_REVIEW_20260924.md)，本记录只管理主线交付，不扩展到 VLA、FoundationPose 新后端、装配或真机。

总调度：`01a0c405-627f-7742-b500-e057726ed2e7`（设计机器人搬运仿真流程）。本轮开始 2026-09-24 09:50 +08。每个问题单独记录原始起点，排查 1 小时未解决则有界收尾并转独立项，不能换窗口重置。

## 当前主线检查点（2026-09-25 01:25）

- 用户要求偶现问题先留档，本轮未展开TF/DDS/底盘瞬态动力学专项，也未降低门槛。scene25 TF未就绪、scene26发布者图未确认、scene30两帧底盘角速度越限均有独立证据；scene30观测命令全零，但不能把真实速度越限称作误报。
- 正常主线仍未完成。最终scene31/domain60确认PREGRASP、GRASP_APPROACH、GRASP_CONFIRM及实际物理附着；随后真实保守载荷的TRANSPORT_POSTURE路径仍碰取货台（首坏119/840）。未执行LIFT、有载NAV或PLACE。
- 本问题原起点00:26:36、截止01:26:36不变，最后已选择候选验证结束，现只做有界留档收尾。指垫接触契约修复保留；向内120mm被自碰撞拒绝，前向120mm未取得完整Cartesian路径，最后vertical60虽通过初始完整规划，但没有解决实际附件收臂碰撞。不继续试参数，也不把任何候选记作通过。
- 已确认根因边界：初始PICK规划使用名义世界物体；附着后的权威模型为更大保守尺寸并带实测TCP偏置。当前服务只验证并保留旧timed路径，无法返回重新规划的轨迹。提高终点净空不能保证后续OMPL全路径不重新贴近台面。恢复时应先明确一致几何的路径生成/显式新轨迹回传边界，不能暗换缓存或取消重验。
- scene31父status6/resources_released=false、业务UNRESOLVED/domain60 journal隔离；底盘收尾36帧/0.7s零速零漂移，外层cleanup=true、stack stopped、remaining_owned_pids=[]。这些进程清理与停车证据不等于业务资源释放。录像545帧可解码，计数一致，属于失败现场，不能交付为成功搬运演示。
- 最终验证fixture冻结：prepare_transfer_goal.py SHA52cbceb61f04a1719fc5129caf125738b411156d2ec63a511765ed9d09c516a3；scenario.json SHAad21630122ec833dc7ca17de21117aed7c57a58e4263ba1b96b29cca180dce3a。原代码/库及实际绑定未改。旧候选已归档，当前生效脚本只保留vertical60，无forward/inward分支；活动覆盖层仍有依赖，不能删掉共享底层。
- 证据：[当前收尾](evidence/mainline_20260924/scene27_actual_payload_collision/scene31_final_checkpoint.json)、[原首坏点复算](evidence/mainline_20260924/scene27_actual_payload_collision/scene27_clearance.json)、[瞬态运动只读审查](evidence/mainline_20260924/scene27_actual_payload_collision/scene30_motion_readonly_audit.json)。Git先行归档8222c697/dbbfc3dd，最终收尾再单独提交；最终复算见[scene31全部角点/SAT](evidence/mainline_20260924/scene27_actual_payload_collision/scene31_clearance.json)：first净空54.264mm，首坏状态箱中心下降56.100mm、竖向半高增加5.127mm，确有台体交叠。
- 后续有载导航与放置依赖此路径修复，当前限定范围内没有可独立验收的下游运动；其他协作任务冻结，不因此开启VLA、规划器、感知或多场景扩展矩阵。

## 历史主线检查点（2026-09-25 00:36）

- 唯一目标仍为同一普通单箱父Action：PICK → 有载NAV → PLACE → 权威EMPTY/独立Scene读回 → 终态、停稳、资源释放及录像。VLA、真机和所有扩展矩阵继续冻结。
- 偶发TF本地滞后、future-drop及启动失败专项已按用户要求留档暂缓。反复阻断正常链的明确采样顺序竞态已最小修复，6项测试通过；scene23未复现并成功执行物理附着。旧原始起点23:30保留，修复不代表其他时效问题通过。
- 重复world REMOVE修复6项真实MoveIt测试通过，scene23已进入附着后的剩余计划重验，说明旧apply拒绝点已越过；没有额外独立保存attach后完整Scene，证据层级不混用。
- 当前直接阻断为scene23剩余TRANSPORT_POSTURE候选中L11指垫与箱体误作禁止接触，起点00:26:36。权威body此前只有TCP，未保留规划允许的两个指垫。经独立URDF/Commander/接触阶段审查，只将同手L11/R11加入实际附件touch集合，非pad和外部碰撞继续检查；包络、质量、位姿不变。
- root构建并通过2项几何测试（含左右精确接触集/非pad和几何不变断言）；只覆盖新几何共享库。scene24/domain67已启动，实际Gazebo新库及旧依赖均在发Goal前核对。新库SHA `cec3b525c9354f1f1d797eabbae5149079d847c70b01c3379874b767474bc8ca`。结果待定。
- scene23业务UNRESOLVED、domain68 journal隔离，外层进程清理完成，录屏614/614/614；旧quarantine未清除。主线仍未验收。M1冻结产品源码，M5仅整理实际证据；root唯一构建/仿真操作者。

00:48 补充：scene24正确加载pad新库，实际权威接触集已为TCP+同手两pad；剩余TRANSPORT_POSTURE改报箱体与取货台碰撞，未执行LIFT或收臂。业务UNRESOLVED/domain67隔离，外层清理完成，474帧视频索引一致。实际附件理想lift终点保守箱底为1.059264275 m、相对台面净空+24.264 mm；现有记录缺缓存收臂路径，不能推断碰撞在首点或仅因高度不足。代码审查未找到错误使用阶段初态的证据。

只增加失败状态诊断：原首坏检查索引（含插值）及first/bad/last完整RobotState，使用max_digits10保存。原返回reason/碰撞/插值/时间不改。root构建并通过payload_transition和trajectory_time_scaling两组现有CTest，6份YAML状态可解析；初版6位精度日志另存，不混用为准确复算。scene25/domain66正在同一正常场景运行，新MTC planner SHA `c6bf0df19f275baaaccdaf06abd8870e39aeb4d41f339a2070b682c286cc6fda`，guard/scene_signature不变。原00:26:36问题时钟继续，截止01:26:36；此诊断不是问题修复。

00:56 补充：scene25在父Goal前遇到INITIAL_READY_TF_STALE，scene26在父Goal前未观测到两条导航输出publisher（两个列表均为空），均已清理，未发运动目标。scene26只读审查确认检查器一次取图、未记录错误writer或多writer；不能由空图判定生产节点不存在。两轮按用户最新要求保留证据暂缓，不展开DDS/TF专项或放宽准入。scene27/domain64继续同一正常场景和相同候选；root独占仿真，几何协作代理仅准备首坏状态离线复算，M1产品冻结。原碰撞排查起点及01:26:36截止不变。

01:02 补充：scene27正常越过准备/三个抓取阶段和物理附着，实际剩余TRANSPORT_POSTURE首坏状态已保存：检查序号41/788，箱体与取货台碰撞。独立现场URDF FK及同frame台面复算显示：首状态净空+24.264268mm；首坏保守箱底−0.257164mm，名义箱底仍+6.590478mm。TCP下降20.3208mm、箱中心下降18.2068mm并倾斜约11.639°，尚未横向离台即耗尽净空。非错误stage起点，不能靠末点或单纯lift终点证明全路径安全。scene27父status6/resources_released=false、业务UNRESOLVED；收尾底盘实测停稳，外层cleanup完成，503帧录像计数一致；domain64隔离journal保留。

本轮选择仅修普通场景既有pick_exit候选：原3cm抬升同时向同场机器人方向离台，待全部角点核算后取120mm候选；不改MTC/API、不放宽碰撞、不减少实际保守模型。实际附件后全剩余路径重验仍为放行条件，不能宣称通用预测/重规划功能已实现。M1独占验证fixture的prepare_transfer_goal.py/scenario.json；root独占运行器与仿真；几何代理只做离线核算。原碰撞01:26:36截止不变。

01:04 补充：scene27完整角点复算冻结（SHA70f679ee…）：径向120mm离台候选终点横向净空27.003mm，竖向24.264mm；只证明终点，不保证段内与OMPL后续回扫。M1验证fixture只改变pick_exit的XY（world约[.083028,.581206,1.225]），其他Goal字段逐项不变，6种无效距离拒绝；未改变MTC或物理尺寸。root核对script SHA19dca6a3…及scenario deda660b…后启动scene28/domain63，录屏与原实际附着全路径重验仍开启。结果未定，不能提前计入主线通过。

01:15 补充：scene28径向向内120mm离台在规划阶段被arm_left_link_2/head_link_2自碰撞拒绝；scene29前向120mm离台也未生成完整Cartesian路径（fraction0.788767，VALID_OR_DIFFERENT_IK_BRANCH），均未发JTC、业务释放及外层清理成立，录像分别323/311帧一致。不将端点净空当作可达性。本轮只再取一个已选择的最小候选：原XY、PICK exit总抬升60mm，PREGRASP仍30mm，其他目标不变；依据原vertical30的LIFT已通过而后续收臂下降/倾斜消耗约24.521mm净空。新终点静态净空约54.264mm，但新轨迹仍必须由原完整路径检查放行，不沿用旧路径安全结论。原01:26:36截止保留，最后候选失败则安全留档暂缓，不展开IK/参数矩阵。

## 历史主线检查点（2026-09-25 00:10）

- 按用户最新要求，TF 本地接收停滞、物理状态偶发 STALE 及 scene20 启动失败均留档暂缓；不展开专项矩阵。主线验收目标和原碰撞、时效、停稳及资源门槛不变。
- scene21 实际通过三个抓取阶段并应用物理附着，但 MoveIt 场景服务因重复删除同 ID 世界物体返回失败。父任务 status=6、resources_released=false，domain70 日志隔离保留；外层进程清理成功不代表业务释放。该直接阻断起点为 9月24日23:58。
- M1 最小修复仅删除附着 diff 中冗余 world REMOVE，独立审查无阻断；root 构建并使用实际 MoveIt 2.5.9 库通过全部 6 项 payload_scene 测试。测试包括旧操作部分生效仍失败、新附着成功、保守尺寸、实际放置、其他物体保留与无关变更拒绝。没有减小几何或放宽读回。
- root 正运行唯一 scene22/domain69 正常链并录屏；使用新 payload_scene 库 SHA `7b1f554d91108b89bcc499d9e2241ee99d8ee437fda9cfe79d13ad0b8ad98717`，执行器及实际加载库均按哈希绑定。其他窗口只做有界代码/证据审查，禁止共享构建与仿真并发。
- 主线仍未验收：需同一父 Action 完成 PICK → 有载 NAV → PLACE → 权威 EMPTY/独立 Scene 读回 → 终态、实测停稳、释放及可播放录像。scene22 结果待定；当前6项通过只证明场景变换语义。

00:13 补充：scene22 再次在三段抓取确认后、物理提交前被 STALE 终止；资源明确释放、实测收尾与进程清理成立，录像解码580帧与索引/元数据一致，未越过场景应用修复点。新增诊断明确显示 `poll_steady=21786366339340`、`received_steady=21786367169203`，而 poll ROS/receipt ROS/source stamp 均为111.32 s；即回调在调用方取时后0.829863 ms到达，原检查误当回退。仅对这一反复阻断正常链且已有直接证据的采样顺序问题作有界最小修复；原问题仍从23:30计时，00:30未完成则暂缓，不展开TF/future-drop/性能专项。

00:21 补充：M1 快照取时修复完成，root 统一重建含新版hpp布局的executor+payload_client_test；使用build库实际通过6/6（2.483s），含确定性回调顺序、真实回退、重复帧不续期、原2s期限实际跨界拒绝。新执行器SHA `466699591b98eb3ed2a5a37a62a96fd90a30fa12e7aaff415f95c218fc197fcb`，payload_client库SHA `e4f2fbee6ce87305abd857abc14e68b9306e9d8fed4f5cef5695df6424e4d29c`。新scene23/domain68待起；M5仅修验证器最后尺寸判据：detach保留权威附件保守几何，不能误与原物理箱体尺寸比较。该静态冲突已由scene21权威尺寸和真实MoveIt测试确认，仍没有完整PLACE实际通过证据。

00:24 补充：M5 最终world尺寸判据按本父事务ATTACH/DETACH、同源raw与confirmed ledger绑定保守primitive后精确比较；原物理尺寸scenario未改，缺来源或错版无回退。根独立重跑7项离线正反例全过，仅合成完整事务+scene21真实尺寸，不冒充PLACE实际通过。验证器SHA `2c0c04601d690bfe4c7810f197d508de0682594587da137953ce9bbc43d67eb2`。scene23/domain68已由root唯一启动，所有候选冻结；两组C++修复与验证器证据分别留档Git `64718b6e`、`6fc9e417`、`f3cdf5f0`，实际父Action待验收。

00:27 补充：scene23 实际执行GRASP_CONFIRM后发物理command1，source revision2/sequence1609已应用；MoveIt场景申请和独立读回越过旧失败点，随后剩余MTC TRANSPORT_POSTURE重验拒绝 `astribot_gripper_left_Link_L11<->transport_box_01` 碰撞。此轮未出现PAYLOAD_STATE_STALE，支持快照顺序修复在本普通场景越过旧阻断，不代表所有时效问题或长期稳定性通过。父status6/resources_released=false，domain68 journal隔离；外层cleanup=true、stack stopped/remaining[]，录屏614帧解码/索引/元数据一致。新直接碰撞阻断起点00:26:36；初查actual body touch_links仅TCP，而原planned body允许手部接触，正核对合法指垫与几何冲突边界，不降低碰撞门槛。主线仍无LIFT/NAV/PLACE通过。

## 历史主线检查点（23:28）

- 根任务已接管唯一仿真操作；M1 冻结执行器并交叉审核，M3 仅定位当前过滤确认阻断，M5 整理真实阶段与录像证据。所有扩展矩阵继续冻结。
- scene12 的夹爪目标要求对 60 mm 刚体压入 4 mm，实测停在约 59.932 mm；仅仿真 MTC 路径取消这项预紧，保持夹爪 0.02 rad 到位门槛。scene13 已真实确认 GRASP_CONFIRM 并应用附着，之后因过渡失效几何的空 model 字段被错误比较而失败；资源 UNRESOLVED、journal quarantined，进程退出不等于业务释放。几何计算与前后源码证据已在 Git `0ea3e64d` 留档。
- M1 修复载荷事务中暂时失效几何的字段解释，始终保留 source/raw/底盘停稳与位移、原期限及最终版本提交屏障。根任务构建并通过域 41/42 两项隔离协议：有效恢复后继续；完整 model 真改变仍拒绝并隔离。它们不是 Gazebo 全流程验收。新执行器 SHA `84b8c2d127607a14e3936672e6597f4ea6daf395e7f609717c778695a0999b99`。
- scene14 在 MTC 规划期间遇到 `ATTACHMENT_FILTER_UNCONFIRMED`，未发送 JTC；父任务取消并确认资源释放。原问题始于约 23:17。主 slice 本地 TF 查询从 source 70.4 到 72.4 持续看到 latest 70.3；独立 M5 与生产端高频记录仍连续，ledger 全程同版本 confirmed。当前证据支持本地 TF 接收/缓存停滞，不证明具体 DDS 或线程根因。
- scene15/domain76 于 23:28 启动同一正常场景，仅主 slice 动态 `/tf` 订阅改为 best_effort 作单因子验证；静态 TF、所有时效/碰撞/版本门槛及二进制不变。补实际 QoS/GID 读回及 ACK 被动记录。候选目录 `runs/mainline_20260924/filter_tf_qos_2332` 名称只是标识，不是实际开始时刻。
- 主线仍未通过：还需同一父 Action 完成 PICK → 有载 NAV → PLACE → 权威 EMPTY/独立 Scene 读回 → 终态、实测停稳、释放和录像。旧候选仍为活动依赖，不因已归档而删除。

23:31 补充：scene15 已确认 PREGRASP、GRASP_APPROACH、GRASP_CONFIRM，进入 ATTACH_CONFIRM；0 physical_submission 后因 `PAYLOAD_STATE_STALE` 失败并释放。413 个观测到的 GeometryState 全为 CURRENT；仅支持本轮未复现过滤失效，不证明 DDS 假设已修复。物理状态源在 94.201–95.681 s 连续 75 帧、最大 source gap 20 ms，但独立 echo 没有本机 executor receipt 时间。M1 独占 `payload_client.cpp` 加最小失效诊断，根任务负责构建与实际库映射核验；原 23:30 问题起点保留。M3 归档 TF 单因子证据，M5 等下一实际阶段进展。协议与 scene12/13 小归档已单独提交 `96d7c424`。

用户于 23:38 明确偶发问题暂过、留存后续专项排查。现将 scene14 本地 TF 滞后与 scene15 物理读回 STALE 标为暂缓（未修复/未验收），停止扩大定位。下一场 scene16/domain75 继续正常单箱流程，仍使用已验证 native84b 安装及 scene15 的 TF 订阅候选；不加载未构建/未验证的物理状态诊断改动，不放宽任何运行门槛。

23:44 主线必要性复核：scene16 与 scene18 在首个参数服务发现阶段重复撤销停稳窗口，均无 fixture 修改与父任务；已确认 `read_model_parameters` 直接阻塞 `wait_for_service`，未调用已有 `spin`。仅委派 M1 修正此只读工具等待与原小测试，保持发现/总超时与实际停稳标准。scene17 规划阶段的附件状态暂时无效已按用户指示留档，不追加专项排查。物理读回诊断归档提交 `e71f2d11`，未验证诊断仅存档，活动源码已精确恢复至该候选前状态，避免后续构建误采用。

23:58 主线推进：准备发现修复通过 5/5 离线及 scene19 实际准备，已提交 `4d02b7c7`。scene19 再次在 ATTACH_CONFIRM 前触发物理读回 STALE，因此仅恢复已归档的最小失效诊断，5/5 物理客户端用例通过。scene20 启动时 VoxelSLAM -11，原流程收尾 remaining[]，按用户要求留档暂过。scene21 越过物理读回，command_id=1 的实际附着已应用，随后 `PAYLOAD_SCENE_APPLY_REJECTED`；父任务 status6/resources_released=false、隔离保留，仿真进程收尾不等于业务释放。M1 获得仅 payload_scene.cpp/其测试与必要测试依赖的单写权限，核对真实 MoveIt PlanningScene 附着/世界对象更新语义；根仍独占构建和仿真。此新直接阻断起点23:58，其他偶发专项仍暂缓。

## 历史主线检查点（19:05）

22:50 恢复补充：M2 在 19:15 因调用额度中断，scene10 已取消、资源释放、停稳并退出；中断期间没有继续仿真。用户再次明确继续后由根任务接管实际执行，保留原问题起点与已用排查时间，不以换窗口重置。scene10 确证记录器 GC 长尾 160–259 ms，但实际首因是 PREGRASP 中段真实底盘角速度 -0.07309 rad/s 超过原 0.03 门槛，并非本场里程计过期；根因待查。JTC/geometry 纯归档流已改为 CDR 暂存、完整安全收尾后还原原 JSON，实际消息 6/6 往返字段一致；不改 GC 策略或验收阈值。scene11/domain80 在相同正常场补实际控轮二进制/参数读回及轮力矩记录，其他场景继续冻结。19:05 的发现等待与工位绑定修复结论不变；完整主线仍未通过。

- 唯一验收对象为当前导航仓库中的普通单箱、同一父 Action：PICK → 有载 NAV → PLACE → 权威 EMPTY 与独立 PlanningScene 读回 → 子任务终态、实测停稳、资源释放，并有可播放录像。主线尚未通过；VLA、真机、新规划器、窄通道和多负载扩展均不启动。
- 用户要求的停车、限速、不可执行已前置到 Nav2/BT/路由及其控制器；当前五消费者确认，不再要求末级执行保护 ACK。旧六 ACK 记录仅为历史证据。
- M2 为唯一仿真操作者，负责私有验证脚本与同案逐次运行；M1 的 C++ 执行器冻结待命，根任务统一审查/构建/集成；M5 只核对已接受并实际执行的场次录像。M3 与扩展任务保持冻结。每次失败先定位原始证据，不盲目重跑。
- 事件式底盘命令兼容修复通过 6/6 隔离 ROS 协议；初次工位观测在绑定 world epoch 前被丢弃的问题已修复，相关 C++ 单元验证 4/4 + 4/4。证据已分别留档于 Git `9aad27ab`、`3706dede`，不作为完整搬运通过依据。
- 当前安装绑定见 [station candidate](evidence/mainline_20260924/transfer_station_binding_1841/candidate_binding.json)。执行器 SHA256 `3fd0bfa945b9b597dd3b3e90abb0cce9be6cba7ee22efeba28d0cc7b53b19763`；运行时校验同时包含实际加载的 helper 共享库。
- scene09/domain100 首次接受正式父任务，实际通过工位 Scene 确认与规划，进入机械臂动作并反馈 GRASP_CONFIRM。之后验证器因 ODOM_STALE 主动取消：父任务返回 canceled/resources_released=true，独立 cleanup_stop 通过，未进入 NAV/PLACE。不能把反馈 GRASP_CONFIRM 当成物体已成功附着。
- scene09 对齐同一 steady 时间的独立记录显示，取消前 /odom、/clock、joint 持续更新，而 probe 多种回调同时停顿。已把问题收敛到验证器自身阻塞；正在加入轻量 callback/spin/GC 计时以区分具体路径，未改 300 ms 时效或物理门槛。问题起点为 scene09 本次停顿，初始发现等待问题自约 18:39 计时并已在 scene09 越过，历史失败保留。
- 19:05 必要性复核：仅上述验证器阻塞直接妨碍本次正常流程。旧准备失败录像不再逐场分析；M5 只处理 scene09 实际动作与后续正式场次。旧候选安装尚被 overlay 引用，主线替代版本未验收前不删除活动依赖。

以下时间点、表格与“当前关闭”章节保留原始历史，不代表本检查点的新验收结论。

最新边界调整（14:30 起，15:30 检查点）：用户明确取消独立整机末级执行保护，停车/限速/不可执行均前置至 Nav2/BT/路由规划。完整父任务保存 14:30 未编译检查点，不继续按旧六 ACK 接线。当前分工：M1 五 ACK 协调器/导航 helper，M2 前置仲裁/BT，M3 C++ NavigationConstraint（无速度写入），根任务负责 Nav2 指令统一限速、启动接线与隔离构建，M5 仅更新本次诊断采集。平滑器与底盘输出不新增上肢状态门禁。详见 [边界决定](NAVIGATION_UPPER_BODY_BOUNDARY_20260924.md)。14:42 必要性复核：这些变化均为用户明确改动的直接依赖，未启动窄通道、模型、GPU或多负载新矩阵；旧主线问题计时/证据不重置。

最新恢复（14:05）：用户已批准 READY→固定工位单箱正式 C++ 闭环优先，真实模型、零位准备及扩展矩阵后置。M1 首因修复指定五场隔离协议已全部通过并冻结；目前仅施工同租约组合父任务、导航 helper、工位 Scene 刷新，尚未启动新真实场。14:20 必要性复核确认这三项直接阻断当前主线，M5只等待实际素材做既有采证。新的限定契约和单写分工见 [固定工位主线契约](evidence/mainline_20260924/fixed_station_transfer_contract.md)。下文13:44暂停和旧状态作为历史保留，由本段及后续新证据覆盖。

## 分工及文件所有权

15:10 检查点：原 14:10 起的连续父任务暂停，未完成整栈或组合执行器验证。导航 helper 三例已通过，原两项失败由测试端一次 spin 只接收一条 ACK 引起，改为有界排空后通过；生产 TTL、停止窗口均未放宽。父 runner 的首次 client 数量及反馈父/子 context 匹配仍需恢复时核对，不能以 helper 通过宣称父流程通过。

14:30 起的导航边界任务继续至原 15:30 检查点：复核发现臂展耦合默认仍在末端缩放 Twist，将其改为上游百分比限速源并合入 NavigationConstraint。根负责启动装配，M3 负责约束消费，独立 agent 负责复用臂展算法的限速源；未启动新的 Gazebo 场景。

| 工作包 | 执行任务 | 本轮交付边界 | 修改范围 | 当前状态 |
|---|---|---|---|---|
| M0 取消撤销/集成 | 总调度 | 核实兼容取消超时是否阻断运动许可撤销；修复已证问题；接口协调、复核、最终集成 | 旧 transport Python 兼容路径及其测试；本调度记录 | 限定修复110/110与Git完成；集成继续 |
| M1 C++ 执行器 | 盘点可迁移至 C++ 的 Python 实现 `01a0beb7-23e6-7112-8221-7f2a198b659f` | 先完成受权的 MTC 轨迹执行→终态→实测停稳→保持，再继续正常抓放编排 | `astribot_s1_transport_native`；独立证据/构建目录 | 真实首段执行与Hold已确证；旧完整候选PICK/PLACE合成协议通过，新候选raw失效屏障有限证据成立，最终首因返回及新版兼容回归未关闭；13:44按时限暂停 |
| M2 账本/质量/包络；M4 通道 | 完善path_tracking到位精度 `01a09962-8e15-71e0-b9a6-a92798318698` | 先核对并验证 A4 新节点实际有载准入/失效撤销；M1 就绪后实际非 home 接线；之后 M4 | 导航固定包络/载荷接线和通道；不改 native 执行器 | 首段跟随限定通过；scene02六ACK工具异常与释放未验，world91首段六ACK、正常取消/停稳/资源释放已通过；M4全身几何离线6场通过 |
| M3 感知到规划 | 制定FoundationPose整合方案 `01a0ccca-7607-7bc0-b7d9-12dc8c8e6eda` | 复用现有 GraspNet/CAD 6D，提供实际版本与物体身份绑定的候选调用接线；先离线、后集成 | `astribot_s1_manipulation_perception`，必要感知接口；不改 native 执行器、共享启动或 GPU 基线 | 72项与epoch0补丁7项通过；静止入口20项C++检查通过并提交497c4e90，实际source/模型未验，停止新增实现 |
| M5 覆盖和负载验证准备 | 排查图像积压与传输方式 `01a0c70b-a5f6-77e0-82b7-3e740774ed0d` | 准备当前六相机真实话题的运动/覆盖采集和判据；只补必要验证脚本 | 独立 `tools/vision` 验证脚本和证据；本轮不改运行时 | world91首段515帧及41话题观测已归档；动作窗口最大接收gap0.863s，固定10fps不是实时回放；完整连续流程仍未验 |
| M6 多负载回归 | 总调度安排后续 | 待 M1/M2 正常有载链成立后展开最小矩阵 | 尚不分派运行 | 依赖等待 |

共享接口/构建入口改动先报告具体路径和所需字段，由总调度分配单一写入者。所有任务保留别人的未提交修改，不 checkout/reset/clean 当前共享树，不改共享 install，不自行创建对照开发分支。独立构建与测试目录按本任务命名；主线证据保存源/参数/安装哈希。

## 资源调度

- 首轮仿真/GPU操作权：**导航任务 M2**，只在核实原有会话归属、无冲突后启动同一导航仓库；使用独立会话身份和安装覆盖层。其他任务本轮离线工作，不启动第二套 Gazebo、GPU 推理压力或操作该会话。
- M2 完成实际质量接线验证后，报告会话、查询环境、实际 overlay、状态和进程归属；是否保持会话复用由总调度决定。M1 的动作验证需等明确交接，不能因看到服务可用就发送动作。
- M1 构建建议最多 2 个编译进程；M3 最多 1 个。M5 不运行压力测试。执行测量时进一步停重构建，以免把并行竞争误判为性能问题。
- 不清理归属未知的仿真；遇到冲突立即报告所有者和证据。任何真机操作不在本轮授权范围。

## 接口与验收顺序

1. M1 首次回报先列最小轨迹执行接口、资源 owner/epoch、场景/几何绑定和完成证据；M2 回报所需 Hold/包络请求输入，M3 回报物体与候选上下文。总调度比对后再进行跨包接线。
2. 代码/离线验证可并行；实际运动依次为 M2 静止有载链 → M1 非 home 动作/保持 → M1+M2 普通携物链 → M3 模型驱动完整抓放 → M4/M5/M6 专项矩阵。
3. 复用当前正常成功场景、MTC 退路和独立放置确认；禁止降低碰撞/时效/质量门槛，不以单对象完成代替权威 EMPTY，不把推理候选视为已获执行权限。
4. 各任务记录：逻辑前置、最早问题计时、修改路径、实际测试命令/结果、失败及延期项、供集成使用的接口/overlay。报告“已实现、离线通过、仿真通过”分别列出。
5. 完成一个受影响边界的必要验证后即推进后续，不重复全仓矩阵；最终统一正常流程保留录屏与事件/状态证据。

## 调度事件

- 09:50：用户授权统一派发；已核对现有任务，优先复用四个相关窗口，不新建重复任务。
- 09:53：四个任务已派发且接单；M0 最小取消缺口复现交给本任务内已有审查代理，修改范围为旧兼容路径与对应测试。
- 09:55：M3 接口确定为本感知包导出的 C++ 客户端库，由 M1 后续链接，不新加常驻网关。M3 独占其包 CMake/package；既有推理服务单在途，按同一快照串行 6D→GraspNet，继承原始期限后再调用 MTC，不并发双推理请求、不直接驱动。M1 首先完成受权非 home 轨迹，不等待感知。
- 09:57：上一条推理串行方案被源码约束否定：两服务共用单在途，输入准入硬限 0.5 s，第二次推理会在准入前过期。修正为复用 pose-only / grasp-only 两个既有服务实例，仅各启用所需 worker，未使用端点私有重映射；客户端对同一快照并发发送，保留原采集时间及期限。M3 负责确认启动固定场景/包络版本与 M1 实际上下文的接入限制，不以固定版本冒充实时状态。
- 10:00：M0 限定修复通过当前正确绑定下 transport 110/110 离线测试，保留初次错误绑定的失败记录。M2 确认 fixed_v2 的 SetRobotEnvelope(transport_ready=false) 可运动中立即收紧并锁存；legacy 同名入口仍先要求停稳。因此旧兼容修复只关闭“异常跳过后续收尾”，不宣称补齐 legacy 移动中撤销。
- 10:02：总调度修正文档中的过期 fixed_v2 完整抓放入口，并负责将现有 MTC canonical_octomap 导出为 C++ 可链接库，M1 不复制算法或编译兄弟包源路径。
- 10:02 左右：M3 注册几何独立审查发现候选 depth、宽度及夹爪四连杆位置会影响 TCP 变换；GraspNet 开口宽度不可直接当物体厚度传给 MTC。由审查代理提供精确坐标链与仿真注册依据，M3 首期不以未校验常量放行。
- 10:02 左右：M2 开始 EMPTY/时钟冻结实际功能验证；已通知 M1/M3 暂缓重构建，root 尚未启动库构建。实际未完全隔离的首窗口不计独占性能证据。
- 10:04 左右：M2 EMPTY/新 Hold/六 ACK、暂停后撤销及新 epoch 恢复通过；总链观测撤销 0.406 s，仅作功能证据，不宣称 50 ms 质量定时器性能。构建恢复。M1 尚需离线协议验证，不让 M2 空占会话等待。
- 10:07：M0 主调度独立重跑 transport 110/110，通过；精确补丁和测试留存 Git `7f1b362f`，其他已有 EnvelopeInbox 修改未纳入。
- 10:08：MTC 库独立 Release 构建、原包 3/3 与安装后独立消费者 1/1 通过，覆盖层已交 M1；无共享安装改动。M5 被动元数据观察器及分析脚本离线 25/25 通过，实际运动与 ROI 仍待测。
- 10:10 左右：M2 真实有载首试发现验证器未处理 Humble 字符形式 CollisionObject.operation，实际几何匹配却误拒绝；所属会话已清理，按原计时修工具边界后复测，不改产品门槛。M1 合成协议预留 domain 170–176（顺序运行），M3 合成协议预留 100；均不操作 Gazebo 或真机。
- 10:19：M2 静止质量链完成：新独立世界 domain90 中 8 次六 ACK、1 次低报拒绝，真实附件 `0.5→1.25→0.75→EMPTY` 转换和新 epoch 恢复通过；最终完整 EMPTY、停稳和资源释放确认，自有进程全部退出。域89先前错误验证顺序产生的未决 journal 保留，新世界通过不算旧事务恢复。见 [M2 实测](M2_MASS_RUNTIME_20260924.md)。
- 10:23：M3 客户端 10 项契约、6 项隔离 ROS 协议、5 项实际夹爪 FK/mesh 几何通过。独立审查要求的 UNKNOWN 非终态修复已纳入；GraspNet width 不当作物体厚度、候选 depth 与实际夹爪 q 共同决定 TCP 映射。仅提案库就绪，不算实际模型驱动抓放。
- 10:24：M1 首段独立构建 6/6 CTest、正常合成协议通过，负向协议继续。总调度安排独立代码审查；M2 准备下一独占场次，尚未启动运动。实际验收先限完整 MTC 计划中的首个 PREGRASP→实测停稳→正式 Hold，不称完整 PICK。
- 10:26：总调度独立重跑 M2 验证工装 26 项、M5 ROI 合成测试 18 项，均通过。M5 首个 30 s 静止样本仍有 5 个 >250 ms 间隔；源戳也跳帧而时钟连续，只支持单流观测缺帧，尚不能归因渲染、桥接或 DDS。构建负载未持续监测，不作独占性能结论。
- 10:28：M5 离线 ROI 工具能量化真实投影表面与有效深度的区别，缺实际同刻快照和覆盖阈值，验收保持 UNKNOWN。后续由 M2 操作者采集，M5 不另开仿真。总调度独立重跑 M3 10 项纯 C++ 契约通过。
- 10:30：下一正常箱体场次保留 60×60×120 mm 外观/位置，补明确物理 collision/inertial 并登记为 detached 库存；旧演示 visual-only 物体成绩不能证明新物理接触。此方柱需要 8 元素对称组，M3 获得 object_pose_core CLI/语义测试和本包 registry 的限定单写权限，复用既有对称 API，不改算法或尺寸。原问题计时仍保留。
- 10:31：M1 首段 56 个 C++ 用例、7 场合成协议及安装身份完成独立复核，精确代码/证据提交 `7fafe827`；M2 静止链证据与工具兼容补丁提交 `628a91f6`，M5 ROI 工具及合成证据提交 `a6255d2a`。M1 安装继续冻结，后续源码与构建另用 next_build/next_install。
- 10:35：授权 M2 唯一操作者开始真实首段非 home→Hold→六 ACK 场次，新的实际集成问题起点 10:35；目标保存视频与被动数据，暂不声明独占性能。预计动作窗口 10:40–10:47，其他重编译暂停。已关闭的静止 M2 不重复全矩阵。
- 10:35：确认完整操作新缺口：MTC 旧重验仅替换 octomap，不能检查真实库存膨胀附件。总调度分派专属实现者独占 `astribot_s1_transport_mtc` 与 `astribot_transport_msgs`，增加独立 payload transition 重验，保留旧接口与冻结安装。限定原 context、合法事件后 index4、原路径/期限，替换剩余场景和轨迹各 RobotState 的实际附件后检查；成功才更新缓存，不写实时 Scene/ledger，不授予执行权。该问题独立起点 10:35。
- 10:36：统一计时口径：10:51/10:53 是本轮问题清单检查点；单个持续未解的问题超过 1 小时才有界暂缓。正常关闭一项后可进入不同依赖任务，不能把整个工作包运行时长当作必须停掉全部工作的期限，也不能用新名称延长同一个未解问题。
- 10:42–10:48：真实首段前两场均未发送机械臂轨迹。第一场为准备工具错误参数读取；第二场完成实际箱体/工位/权威 EMPTY 后暴露首次固定包络的冷启动循环。所属进程均已退出，第二场 observer bridge 需强制回收，保留 cleanup 失败，不能将零残留改写为正常退出。见 [实际首段记录](M1_NONHOME_RUNTIME_20260924.md)。
- 10:46：M3 开始真实 RGB-D 到 Request 的 C++ 接线；保留实际源戳、处理/标定/时钟版本和捕获时 TF。当前无完整箱体快照，2048 个真实有效点保持 UNKNOWN。独立工位投影可显式 decimation=1，不能篡改导航基线或复制点补足数量。
- 10:52：MTC 新增实际载荷转换重验完成，11 个新增用例及原 3 组回归通过，根任务复核 4/4 CTest；原路径、时间和 context 不变，实际保守附件替换到后续 stage 与每个 waypoint。提交 `0b356d39`，尚未接入实际 ATTACH/DETACH。
- 10:56：M5 精确同刻快照准备工具离线 12 项通过，提交 `f7e5934c`；没有同刻真实图像/深度/CameraInfo/TF/独立真值时仍不生成覆盖验收。
- 10:57–11:01：M2 在等首段候选时关闭独立 M4 入口适配误拒绝：限制改为到实际偏置目标的横移量，保持原 0.3 m 门槛与扫掠检查。实际 advance 分支左右镜像及拒绝边界 RED→GREEN，21 项通过；未修改运行安装或进行通道运动。
- 11:02：M1 冷/暖启动及同 context 占据图重验候选冻结到 `next_install`，11 场最终合成协议通过；旧安装未覆盖。根任务独立审查全部源/安装哈希及关键屏障，无新增阻塞，根任务 CTest 7/7 通过。PayloadCommand 草稿的已知缺口仍未作为运行能力验收。
- 11:04：总调度发现真实启动的两个规划器通过 GripperCommander 创建真实夹爪 FJT client，与 native 独占控制检查冲突。M2 确认实际安装解析路径和源码一致，但未保存旧 live graph，不冒充已观测。安排最小 planning-only 初始化接口及实际 ROS graph 测试；不放宽 native 独占检查，不让 M2 反复空启动。首段问题仍沿 10:35 计时。
- 11:05：历史仓库图像含多个橙色连通域；M3 全图颜色唯一性不能支持当前工位。允许使用显式工位 3D region（坐标系、版本及捕获时 TF）过滤真实点，不使用目标实体真值位姿或真值 bbox；区域内多个候选继续拒绝。历史点数只是取样可行性证据，当前模型驱动流程仍待实测。
- 构建记录修正：早期 M3 colcon 虽设置 CMAKE_BUILD_PARALLEL_LEVEL=1，实际命令出现 `-j28 -l28`，不能声称编译线程独占或受限。后续明确 MAKEFLAGS 或直接 `cmake --build --parallel 1` 并审查 command.log；功能测试结果不替代性能隔离证据。
- 11:14：仅规划 Commander 新入口完成，4 项真实模型/隔离 ROS graph 验证通过，旧 API/类布局保留。根任务独立核三个安装产物 hash 后放行第三场。后续补齐可选 fixture 的四项显式 SKIP 与给定 fixture 的四项 PASS，未改运行安装；精确提交 `429f28a4`。
- 11:15：第三场实际 cold 准入与 6 个 FJT client 唯一 owner 均通过，own lease 已得到 navigation revoke ACK，进入 MTC。随后因 `MTC_START_OUTSIDE_PLANNING_MARGIN:astribot_arm_left_joint_4` 拒绝：真实 q≈0、硬限 [-0.06,2.61]、0.1 margin 下界 +0.04。没有 JTC child submission，父任务明确释放；自有进程全部退出。MoveGroup 受控卸载 -11 另记 clean_shutdown=false，不冒充干净退出。
- 11:19–11:22：两份源码审查否定“只拓首段区间即可安全执行”的快捷修复：现有 0.05 跟踪报警阈值不等于含检测、取消和停止过程的误差上界；旧位置线性采样也不证明 JTC 实际样条的极值。保留 MTC 拒绝，不删起点检查，不降低全局 margin。原成功流程先经 READY_RIGHT/READY_LEFT 和 head_pick，而新 native 首段直接跳到 MTC；缺失准备阶段作为主线明确前置，沿原 10:35 问题计时。
- 11:23：M5 第三场真实同刻 RGB/Depth/CameraInfo/健康/TF/独立真值组包完成，stamp=26.5 s；工位内真实点 stride1/2 均 0，评价用箱体投影 ROI=0。当前头朝前、工作台在左侧，属于 VIEW_NOT_READY；不是 GraspNet 失败，也不是抽稀不足。不得补点或把真值裁框送进 Request。
- 11:23：M3 修复接收时已消耗的源龄未投影到 steady 的漏洞；原余量传递到 Request/PlannedPick，重复采样不得续期。13 项 builder、10 项 client、6 项订阅协议通过，TF 立即查询及最终 manifest 仍需封板复核。该证据尚未启动真实模型推理或证明当前视角可用。
- 11:27：为先推进其他可验证主线，另设明确的 READY 前置场景。复用 gz_ros2_control 已有真实 JointPositionReset，新增受控仿真初始关节 profile；默认不写 initial_value、不改变普通基线。其成绩只用于已准备姿态的正常链，不能证明 zero→READY。该独立配置项保守起点为 11:27；目录中的 `1133` 是早期误估名称，已纠正 timing 记录。
- 11:29：PayloadCommand/PayloadClient 已完成 9 个核心与 5 个私有 Ignition Transport 客户端用例、8/8 CTest 和安装后复核。独立审查确认 pending/reconciliation、原剩余租期、首次 poll 前重复 physical capture 等缺口已关闭；根任务独立 9/9，通过并精确提交 `15d559ce`。入队 ACK 不代表应用、超时保持未决、EMPTY/放置仍由后续真实账本与落点判断；目前未接入六阶段执行器。
- 11:31：准备阶段设计落盘到 [native_preparation_design.md](evidence/mainline_20260924/native_preparation_design.md)。只建议接完整空载 PICK，冻结旧首段 Action；PLACE 有载状态不得再跑空载准备/OPEN。未实现的右臂/头部反馈接线、近限进入和验收边界明确保留。
- 11:35：原 zero 初态→首段问题达到 1 小时，按用户规则**暂停该问题的实际尝试并留档**。当前冻结结果是 cold/控制权通过、MTC soft-margin 拒绝、0 ARM child、明确资源释放；不启动放宽余量的第四场。M1 仅继续从合法实际姿态开始的完整 PICK/PLACE，M2 继续独立 READY 场景，M3/M5 继续各自已划定的接口/观测任务。
- 11:36：READY 配置 16 项静态检查和两包独立构建完成；根任务发现新准备工装误将完整 PlanningScene 的零 stamp/空 velocity 当实时传感器。已用第三场真实结构证实该序列化合法，要求工装使用独立新鲜 JointState/JTC 证明 22 关节停稳，Scene 仅做位置和碰撞读回；禁止填假 stamp/将缺速度写成零。修正前不启动新场。
- 11:44：M3 必要离线项封板：独立复核 33 包文件、4 包外授权文件、22 安装产物和 13 测试/库二进制身份一致，7 组 72 项当前检查零失败。原样保留 19 新/4 改/10 旧来源区别；实际视角、projector、模型、M1 执行尚未通过。包与证据精确留档 `90c12aea`；object_pose_core 旧包未局部纳入成不完整包，其工作区 hash 依赖仍记录，HEAD 不是独立发布基线。
- 11:42–11:44：READY scene01 的 22 轴真实位置/速度/时间、TF、完整 Scene 位置及碰撞核验通过，最大配置偏差 0.001901 rad。MTC 34.115→52.314 s 成功规划，但随后 native 报 MTC_SCENE_CHANGED、0 child；此拒绝早于人工调度取消。消息时序导致操作者在明确放行前起场、随后有界收尾，已纠正为只按主调度明确放行执行。资源 RELEASE_CONFIRMED，12 个唯一 PID 全部退出；MoveGroup 卸载 -11 仍记异常。没有恢复已取消事务或原样空跑 scene02。
- 11:48：新场的两个客户端前置 full Scene 在规范化后仅有 aft_mapped→base 固定变换 7 个浮点微漂差异；真正 native 规划前后回包没有落盘，不能把该前置对当作最终根因证据。正审查无关外部 TF 与实际碰撞坐标的边界，不加浮点容差吞真实几何变化。该运行时问题起点保留 11:43。
- 11:50：READY 4 文件独立复核及当前实际初始化证据精确提交 `7eee2c96`，未夹带旧相机、网络或 social 差异。原 zero→READY 继续暂停；READY 初始化通过不证明轨迹、Hold 或完整抓放。M2 实际测量结束后释放有限编译窗口，M1 继续完整 Action，M5 离线核 READY 图像覆盖。

- 11:52：开始独立 source→双模型只读 probe 准备，原问题截止 12:52；完整 MTC 18.2 s 与感知结果 5 s 原期限不匹配仍保留，不能延长模型有效期。该 probe 不调用 MTC 或运动接口，实际必须先具备 M2 真实 Hold/六 ACK 及有期场景注册。
- 12:00–12:13：公共 canonical scene 最小修复完成并留档 `41a9e487`：验证所有实际几何坐标，只移除未消费的 fixed TF；世界物体/附件/ACM/地图内容仍精确比较，不加浮点容差。MTC 5/5、native 8/8、4 场合成协议通过，冻结独立 first-stage 候选；后续 XML 补档 `b2d8faaf`。
- 12:13–12:17：明确放行 M2 唯一操作者运行 scene_binding_scene01。READY/MTC/场景复核均通过，6 个实际 controller child 全部接收，机械臂约执行 0.3 s 后 `EXECUTION_GUARD_UNHEALTHY`；6 子动作全部取消、资源明确释放、底盘停稳、12 个唯一 PID 退出。MoveGroup 卸载 -11 仍单列异常。该实景跨过 11:43 起的场景绑定问题，不代表首段/Hold完成。
- 12:19–12:24：根任务独立解码该场原始前/后 PlanningScene CDR，除实际关节样本和未消费 fixed TF 外完整场景相同。原时钟/轨迹产物及失败证据保存至 [first-stage archive](evidence/scene_binding_first_stage_20260924/README.md)。
- 12:20：定位独立 `RESOURCE_CLOCK_RESET`：同 tick 的 stop 使用更新时钟后，cleanup 仍给 Authority 旧时间，导致伪回退；真实 Gazebo rollback 未获证。私有冻结候选采用 cleanup 前重新采样 ROS/steady，不放宽真实回退拒绝。
- 12:25–12:28：M1完整Action交付可恢复checkpoint：构建成功、10/10组件target、安装后4/4 payload target、normal218/pending_cancel219首段合成协议通过；尚无完整六阶段协议及实际抓放验收。按11:28→12:28时限暂停未完部分，保留开发源码patch/hash/安装，禁止仅因组件通过就启用。见 [checkpoint](evidence/m1_transport_20260924/FULL_EXECUTOR_CHECKPOINT.md)。
- 12:28–12:33：M2实采17个运动期JTC样本表明首点连续，实际位置控制近似 v=9.50×error，无速度前馈，左一轴误差达0.05133 rad；原始guard reason缺失，仍不声称唯一原因。批准下一场单因素规划阶段时间伸长2.5倍候选（默认1），原路径和所有门槛不变；MTC单写者先验证时间/导数及缓存一致性。跟随问题原起点12:15、截止13:15。
- 12:33：时钟缺陷独立RED/GREEN通过：域220旧binary在34个递增clock样本下误报；域221新candidate在36个递增样本下正常收尾、6 cancel ACK及终态、资源释放、无CLOCK_RESET，现有8/8CTest通过。下一实际场仍待轨迹取证、时间伸长和observer预检完成后由总调度放行。

- 12:43 左右：用户要求全部暂停；根任务通知四个执行窗口及子任务。四窗口回报无自有运行进程，下一场未启动；所有代码/证据/未提交修改和原计时保留，MTC收尾代理被中断。
- 12:44：用户明确要求全部继续。重新分派原四窗口并核对暂停检查点。M1从已保存fullAction恢复六阶段协议，不重做已过组件，保留原始起点与前次超时暂停；M2继续12:15起跟随问题与下一独占场准备；M3封住离线probe，实际仍等Hold/六ACK；M5完成sidecar封板和runner复核。此次恢复不自动重放任何旧动作。
- 12:45–12:47：时间伸长候选源码/产物/封存XML再次独立核对一致，先前根任务6/6CTest通过；精确8文件修改及证据提交 `f31f4be2`。真实Gazebo跟随尚未验证。M2已离线解析新native→新MTC→新manipulation及6产物hash；尚待录像和runner最终确认，未授权起场。

- 12:49–12:50：M5录像source/test冻结，root独立4项编码解码测试通过；M2新增实际视频解码与sidecar/summary/索引核验。observer四项接线和独立版本审查完成，root逐项核对最终11来源/6产物hash一致，明确放行 `execution_response_scene01`（domain90，session execution_response_20260924）。只验证READY首段→Hold→六ACK，其他窗口停止重构建/ROS/GPU；7话题3秒静止bag先结束flush再发Action，41话题观察器覆盖动作前后20秒。录像/guard证据提交 `4180589a`，实际结果待定。
- 12:50：M3一次性入口和操作单已准备完成；真实source/双模型仍 `BLOCKED_PREREQUISITE` / `NOT_RUN`，缺本场真实Hold与六ACK、新登记、原始同刻输入；不转写成模型失败。原11:52→12:52保留，不自动续录或延长capture期限。

- 12:52 左右：execution_response_scene01在Action前因验证器 `STALE_LEDGER_CAPTURE` 退出，0ARM/0parent动作；同步observer日志/拓扑预检约0.45949 s未spin，超过原0.3 s账本接收时限。实际新candidate/参数[2.5,.1,.1,.1]读回及7话题raw flush成立，不能因此判定轨迹候选失败。307帧真实解码=sidecar=summary；observer130按INCOMPLETE，资源明确释放、自有进程全部退出。批准仅将同步预检移到现有动态ready采样之前，原门槛和输入源戳不改，待必要顺序验证再放行。

- 12:53：同步observer检查只整体移到原ENTRY ready spin之前；原序离线替身复现STALE、新序收新帧通过，full Scene/TF/目标仍在新动态采样之后。root核精确diff与hash后放行scene02。
- 12:56 左右：scene02真实首段执行完成，6 child全部success，FIRST_STAGE_HOLD_CONFIRMED@79.029 s；guard active样本最大误差0.0320496 rad，未调原0.05门槛。真实指令426点/7.619381332 s，与前场717点不是同一路径A/B。随后工装访问EnvelopeApplyStatus不存在的mode字段中断，六ACK及取消收尾未验；domain90 journal最终租约过期隔离，原样保留，17唯一PID退出不等同资源释放。
- 12:57–12:59：ACK工装仅删除不存在字段，使用实际冻结msg/generated类RED→GREEN，六消费者及epoch/hash/session/缺消费者拒绝规则不变。拟在全新domain91世界验证正常ACK/释放，禁止重置domain90旧事务。M1完整协议225首次运行18JTC后未进物理命令，确认单线程Buffer错误使用timeout overload；独立审查确认零timeout也会触发dedicated-thread检查，lookup亦需无timeout TimePoint重载。225安全终态/RELEASED，冻结失败；229预留修复回归，未放松任何TTL。
- 12:56–12:59：M3额外静态核对发现现有六ACK强制navigation_allowed=true，与一次性probe要求false矛盾。已将实际状态修正为BLOCKED_CONTRACT/NOT_RUN，旧离线检查不升级为可运行。批准设计最小只读验证入口契约修正：区分几何允许位和实际底盘静止，用真实typedHold/当前版本与新鲜运动证据；不修改导航core、不伪造ACK，不延长感知期限。待精确方案与必要验证。

- 13:03–13:05：root核对domain91两处隔离修改、冻结11文件与6产物后明确放行scene03_world91；仅首段→Hold→六ACK→取消/停稳/释放，domain90原quarantine保留。M1/M3无构建/协议，M5只读分析。scene02独立CDR/JTC复核确认346条期望样本与实际下发样条相符、最大采样误差0.0320496133 rad；样条采样峰加速度1.419789318 rad/s²，不混用waypoint导数0.654258555。
- 13:05：M3静止入口设计引用现有measured_stop与native固定基准阈值，获准4文件私有probe实施；明确导航状态事件流不能自行证明空闲，M2必须提供实际冷启动/图与入口排他证据。当前只写代码，不启动ROS/构建。

- 13:06–13:07：scene03_world91通过限定首段闭环，实际738点/11.658845208 s、最大采样误差0.0322653 rad；取消前六consumer同版本ACK齐全，parent canceled/resources_released、独立停稳、journal RELEASED成立，所有自有进程退出。515视频帧实际解码一致；observer130仍INCOMPLETE。typed Hold在工装回调中参与门控但原msg未完整落盘，summary旧typed_hold键实为held JointState，独立原始typed证据缺口另记。
- 13:07：释放计算窗口给M1单线程必要构建与四场合成协议。scene01/02原失败、原视频帧及实际轨迹消费分析提交 `ccd2b2d8`；CSV标准CRLF用cr-at-eol检查通过，未改原数值或哈希。
- 13:09左右：M1 TF必要回归4项通过，229正常PICK与226PLACE均30JTC/1物理命令、真实ledger转换、剩余重验/读回、最终Hold及取消释放通过。只属合成协议，不含Gazebo物理；按序继续227延迟账本与228意外revision。

- 13:15–13:22：完整执行器独立审查发现已收到新raw却可回退旧配对/旧ledger推进的P1，暂停真实full场。231旧candidate在发布late r3后确认payload并发6个后续JTC，随后geometry失效隔离；旧ELF缺接收诊断，只称发布时序复现，不冒称直接观察已消费。最小修复在最新raw回调及全部后续屏障锁存revision/UNKNOWN/不完整/内容冲突，保留未决资源；源码独立复核通过，GREEN与兼容实测待验。
- 13:28：M3静止入口19个纯C++用例通过，SG7/实际模型未验；独立审查用真实scene03发现正常envelope可同stamp从WAITING转READY，通用冲突规则误拒绝，继续做必要限定修正，不以既有19PASS宣布入口完成。
- 用户追加范围约束：**先完成主线，不扩大化；已通过内容的对比与旧版本先留档校验，再从活动目录删除。** root统一清理，先列本轮所有者与实际依赖。仍用于主线运行或本次修复验证的版本待替代者通过后清理；历史原始失败、未决journal不得当作废弃实现擦除。停止额外方案研究/重复矩阵，只保留阻断正常流程的必要修复和取证。

- 13:32：按用户新要求完成首批退役：time_scaling/before、scene_binding/timing_build和timing_install共808普通文件逐项SHA/权限/内容归档校验后删除；原始大小36591802 bytes，归档位于 /home/yjh/WorkSpace/astribot_validation/validated_version_archives_20260924。当前M2/M3依赖安装、原journal/log/CDR保留；完整映射见 [清理结果](evidence/mainline_20260924/validated_version_cleanup_result.json)。

## 当前关闭与下一放行

- 正式关闭的限定边界：M0取消异常收尾；MTC payload transition离线重验；planning-only控制权；静止质量/权威附件/EMPTY/六ACK；READY实际初始化；实际规划场景绑定；scene02首段真实执行与到位Hold。各自证据不等于完整搬运抓放。
- 原 zero→READY 原生准备缺口按10:35→11:35暂停；完整六阶段Action在12:28留档后由用户12:44明确恢复，13:44仍未全部通过，按一小时规则再次保存检查点并停止新代码和运行。不得以READY初始条件替代准备，也不得以首段协议替代完整动作。
- world91首段执行、实测Hold、同版本六ACK、取消与资源释放已通过并退出（0fe54e2d）；world91成功不算domain90旧隔离事务恢复。原typed ArmHoldStatus没有保存，仅在线检查与协调器/Action/journal间接证据；不能称独立raw Hold复核齐全。
- M1新候选domain26真实消费late raw revision后，后续JTC为0、payload确认0、最终Hold为0、保留隔离资源；原驱动exit1保留。清理阶段geometry_fresh覆盖首因导致最终Action仅返回GEOMETRY_UNCONFIRMED，是尚未修复的诊断缺口。27–30的UNKNOWN、正常PICK、正常PLACE、合法延迟场均未跑。见 [13:44检查点](evidence/m1_transport_20260924/FULL_RESUME_CHECKPOINT_1344.md)。
- M2完整PICK接线及23项离线检查准备完成，尚未启用真实完整场；M3静止入口20项C++检查通过（497c4e90），SG7真实负向、实际source/双模型未验；M4实际通道、M5多相机覆盖、M6多负载仍依赖正常完整链。采集窗口启动次序、模型结果5秒期限与慢MTC冲突仅记录，不延长期限、不追加方案施工。
- 本轮实际首段录屏已归档，world91共515帧；动作窗口66接收帧最大gap0.863秒，固定10fps回放不等于现实时间。尚未提供完整抓取/附着/运输/放置录屏。主线仍未完成。
- 已完成11处旧构建、对比安装和源副本的归档→逐文件校验→删除：原普通文件145845231 bytes，压缩包32936728 bytes，外置于 `/home/yjh/WorkSpace/astribot_validation/validated_version_archives_20260924`。11份包及11份manifest的SHA复核通过，原目录均不存在。当前运行依赖与未验候选、原journal/log/CDR保留；[清理结果](evidence/mainline_20260924/validated_version_cleanup_result.json)是恢复索引。此处仅关闭本轮已核实可退役目录，不声称全仓旧方案均已清空。
