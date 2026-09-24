# 下一场真实 RGB-D 与双模型：受控一次性验证方案

> 历史设计留档：本文保留初版两文件方案及当时前提。它已被 `497c4e90` 的四文件静止入口修订替代；必填 `hold_owner_id` 等当前接线见 [操作清单](LIVE_OPERATION_CHECKLIST.md)，20 项 C++ 检查见 [修订证据](stationary_gate_revision/README.md)。旧 `probe_build` 已留档后删除，勿按本设计恢复旧入口。实际 source/双模型仍未验，采集窗口与服务启动次序尚未闭合。

设计起点：2026-09-24 11:52；若同一问题持续未解决，12:52按项目规则记录并暂缓。状态：**两文件入口已实现，隔离构建与离线检查通过；实际 source 与双模型未运行**。本文件不修改已有11:44离线验收记录。

后续授权/进度：总调度批准两文件一次性probe及clock0的source.bind/协议单测两处局部变更。clock0已在新隔离build完成先失败后通过的7例定向协议验证，见clock_epoch_zero/；旧72证据与旧安装未覆盖。probe已经完成静态审查修正、单编译器隔离构建、8项精确提取代码合成检查和5项实际二进制配置拒绝检查。12:21前总调度已释放构建窗口；实际 Gazebo/GPU 仍未授权启动。M2刚结束的首段没有建立成功 Hold/六 ACK，不能用于本probe租约。实际状态以新manifest为准。

## 已核对的前提与范围

M5的 `docs/evidence/mainline_m5_20260924/ready_scene01_static_snapshot/request_point_gate.json` 给出28.3 s实际RGB/深度/K/TF：唯一橙色连通域2466点、有效率1.0，无补点，重投影误差最大1.86e-5 px；相应9项输入/源码hash已逐项匹配。region revision为 `worktable_region_v1:41a07571b298e166ade98e2789cd4fe043049f58d249724bb92eb4ca9790e146`。它只证明该单帧点数/有效率可行，下一场重建工位绑定并重新采集。stride2的618点是反事实对照，不是可用输入。

下一场首要目标：**真实source → 同一新鲜请求 → CAD 6D和GraspNet并发 → 双方结果/终态/原始期限核验**。这里的6D后端仍是既有 `astribot_object_pose_core/object_pose_register`，不是FoundationPose。FoundationPose后端另有阶段与未验收项。

不启动机械臂、夹爪或底盘执行客户端，不写PlanningScene/物体账本；不调用MTC作为本场通过条件。场次仍由M2所有，总调度分配实际ROS域/partition/overlay/GPU窗口。启动前按本场进程、环境与图核对唯一发布者，禁止使用已结束READY_scene01的环境作为新授权。

## 为什么本场先止于双模型

冻结 `PickPlanningClient::plan()` 的 `deadline=r.result_deadline_steady`，每次推理和规划等待均检查同一原期限；`planning_goal()` 又将MTC timeout裁到剩余ROS期限。原始结果最迟capture+5s，默认单次规划上限还只有4s。总调度记录READY六段规划34.115→52.314，约18.2s。该已观察耗时与当前合同不兼容；不能靠延长有效期、重打采集戳或将历史帧重新发布过流程。

两模型是否能在原5s内共同完成尚未知。历史旧场、旧输入甚至失败结果的延迟不能证明当前方柱模型与2466点可以通过。本场记录每端Goal发送尝试及接受/终态的客户端轮询观察steady与ROS时刻（不冒充DDS到达时刻）、原capture/准入/结果期限、worker inference_time和有效返回值。每次worker是实际启动/加载成本，不剔除冷启动后宣称达标。

后续规划需要另立合同：由新鲜观测产生不可执行的几何提案；长耗时规划后重新观测同一实例，对称等价6D、场景/底座/关节/包络和完整路径重新验证，绑定新观测再取得执行准入。**该流程尚未实现，不能把重新观测当成给旧计划续期；本轮不修改5s合同。** 若只做长规划耗时研究，其结果须标为过期的规划研究产物，不能作为当前有效PlannedPick。

## 最小入口与精确改动预报

已有 `pick_planning_protocol_test` 启动合成对端；旧 `validate_inference_actions.py` 使用测试版本1且不消费新source。二者不能直接当作真实入口。

建议仅新增：

- `tools/vision/m3_source_probe/CMakeLists.txt`：独立验证程序构建，显式指向冻结安装；不改ROS包CMake，不安装到共享目录。
- `tools/vision/m3_source_probe/probe.cpp`：单次任务后退出的C++ probe，使用冻结 `SingleBoxRequestSource`、`inference_goals()`、`check_context()`。Goal句柄可复用包内 `src/pending_action.hpp`，显式绑定该私有头的源码hash；不复制一套业务库，不新增常驻网关。

入口只支持此次source+双模型验证，不预留执行或通用任务系统。同一个持续spin的节点承载传感器、TF、当前上下文与Action客户端；等待服务和owner上下文就绪后才取新帧。失败/过期仅取消本次两个具体Goal，最多1s steady收集终态；不确定终态单列并通知所属服务所有者，不cancel-all，不伪装成功。

结果检查包含两端terminal状态、success、原capture header/实例/相机/source/model hash/标定/场景/包络/valid_until绑定，以及6D的position_valid/orientation_valid和候选数量/geometry_valid。source处理epoch继续由原Request及ProjectionHealth绑定。输出原始结果消息与必要CDR/JSON证据，不做夹爪映射通过或MTC通过结论。

**clock_epoch兼容问题已修正：** 初始真实geometry clock_epoch=0现在合法；source仍拒绝时钟回退后继续使用同代际。定向7例协议验证通过，修正已由总调度提交2945d3a0。probe仅用新clock0_install，旧安装和原72项验收不变。

## 启动接线与固定参数

全部节点 `use_sim_time=true`，使用同一场次的ROS环境。下面是待场次所有者填入实时值的操作清单，不是已执行命令。

| 组件 | 接入 | 必需配置 |
|---|---|---|
| 原始相机 | `/camera/raw/head_rgbd/image`、`/camera/raw/head_rgbd/depth_image`、`/camera/raw/head_rgbd/camera_info` | 使用本场实际frame、K/D/R/P、stamp；不固定历史内参 |
| 相机健康 | `/perception/camera_health/head_rgbd` | 实际source_epoch、calibration_revision、当前有效capture窗口 |
| 独立工位投影器 | 既有 `rgbd_pointcloud_node`，节点名 `single_box_projector` | 冻结 `config/single_box_projection.yaml`；CPU、decimation=1、0.08<Z<5.0m、pair age0.25s、无fallback |
| 真实XYZ | `/manipulation/single_box/head_rgbd/points` | 现有provider的全像素含NaN、point_step16、xyz float32 offsets0/4/8；运行时核对，不重发离线点云 |
| 投影健康 | `/perception/projection_health/single_box/head_rgbd` | 唯一该投影器发布者、实际processing_epoch、epoch首帧、持续有效 |
| 位姿实例 | 既有server，名 `object_pose_server` | pose_worker配置；grasp_worker为空；只使用 `/perception/estimate_object_pose` |
| 抓取实例 | 既有server，名 `grasp_proposal_server` | grasp_worker/model配置，device=cuda；pose_worker为空；只使用 `/perception/compute_grasps` |

两实例通过冻结 `paired_inference.launch.py` 启动；未用Action由该launch重映射到各实例disabled名，避免同一Action出现两个真实服务器。共同参数文件来自 `normal_box/inference.template.yaml`；两者的CameraHealth、ProjectionHealth、camera_id/frame及版本完全一致。三个启动参数 `calibration_revision`、`planning_scene_revision`、`envelope_epoch` 必须由本场owner提供真实非零值，启动后读回且描述符read_only；任一版本变化则本job失效，由所有者重建实例，不在线set伪装更新。

目前模板路径供预检：pose worker为任务隔离安装下 `astribot_object_pose_core/lib/astribot_object_pose_core/object_pose_register`；grasp worker为 `/home/yjh/.cache/astribot/graspnet/build_cuda/graspnet_worker`，模型为 `runs/grasp_pose_sim_20260921/models/graspnet_cuda_v2.pt`；模型注册表为 `normal_box/registry.json`。运行前核对实际可执行文件、依赖和内容hash，不把路径存在当作服务就绪。pose_model_revision遵循现有CAD hash冒号visibility hash，grasp_model_revision为实际模型文件hash。注册ID `transport_box_60x60x120`，实例ID `transport_box_01`。

## Context与工位租约：不得自行填造的输入

| 字段 | 权威来源/当前限制 |
|---|---|
| camera_id/frame/source_epoch/calibration_revision | 本场有效CameraHealth与实际CameraInfo；两server读回一致 |
| processing_epoch | 专用ProjectionHealth；不得替代成camera source_epoch |
| camera_info_revision | 冻结source对实际CameraInfo规范内容计算；同版本内容变化拒绝 |
| envelope_epoch | 本场 `/navigation/envelope_v2` 的epoch与coordinator_session_id；不拿旧lease或时间当epoch |
| clock_epoch | 本场owner的时钟代际；geometry发布器初始0有效，clock0修正已验证，不能静默改值 |
| scene_revision / scene_signature | M1/task owner对当前完整 `/get_planning_scene` 的版本与现有规范绑定。MoveIt消息本身没有这些应用版本；本轮按下文验证会话显式登记 |
| identity_revision / confirmed_instances | 本场owner单实例工位绑定，和权威库存/当前独立scene一致；颜色不生成身份 |
| station_frame/region/region_revision | 本场table-only区域：桌面XY、向上0.3m、水平扩展0，经本场桌体pose×primitive_pose变换8角再取AABB；不读目标GT |
| fixture issued_at/valid_until | owner在本场持续确认条件下给出；issue早于capture，expiry晚于使用；不能复制冻结JSON或由probe无限续租 |
| Request两个steady期限 | source在原接收ROS/steady双时钟上生成；准入≤capture+0.5s，结果≤min(capture+5s,fixture expiry)；late delivery和暂停均消耗预算 |

`current()`必须持续读取这些当前事实并复用source.bind_context；不能永远返回初始Request副本。M1现有 `bind_scene()` 返回清除关节样本/规范排序/独立canonical octomap后的pair，`trajectory_scene_binding`已由总调度作为冻结公共库导出。probe不能另写一个看似相同但语义不同的scene签名。若本场owner尚不能提供版本与租约，就只关闭真实传感器provider采集事实，不声称source Request准入或双模型集成已经具备前置条件。

总调度已提供：M1公开安装导出 `astribot_s1_transport_native::trajectory_scene_binding`，公共头scene_binding.hpp，`bind_scene(PlanningScene, const std::set<std::string>& known_links = {})`。probe必须链接root冻结的该库，不直接编译兄弟包源码、不接fullAction WIP；known_links从本场robot_state_publisher实际robot_description解析。统一canonical-frame处理只忽略无关TF，实际world/ACM/附件/占据仍参与比较，不在probe中再实现签名规则。

M1确认没有现成通用生产current Context发布接口；总调度已授权**验证会话显式拥有静态snapshot登记**：M2先建立真实静止Hold与六ACK，给出coordinator_session_id、hold_id与所有权证据。本probe独立读回全Scene，按公共canonical库登记scene revision（由该owner预留的真实登记序号）、canonical内容hash与实例绑定，并持续异步独立重读。任何合法Scene变化都结束此次登记；不使用固定参数1表示不断变化的场景。该合同只支持一次source/双模型验证，不代表生产M1集成。

probe的具体初版约束：全程最多60s steady；登记租期最多30s，双模型仍最多capture+5s。Scene查询间隔100ms，最近已确认快照从原查询发送时起最多1s，网络/处理延迟不重新起算。该1s是验证会话的有界观测口径，不能宣称瞬时察觉场景变化；没有执行权限。实际geometry/envelope按原样epoch及ROS/steady新鲜度核验，重复消息不续期。六ACK的原始证据由M2前置提供，probe不代替既有仲裁/确认流程。

最小owner配置清单（运行时填真实值，不提交历史值作live配置）：

```json
{
  "session_id": "本场owner唯一任务ID",
  "owner_evidence": "/本场/静止Hold和六ACK及单实例工位证据.json",
  "input_recording": "/本场/M2预先启动的有界原始流录制目录",
  "registration_sequence": 1,
  "output_directory": "/本场/尚不存在的probe输出目录",
  "coordinator_session_id": "本场真实协调器session",
  "hold_id": "本场仍有效的真实Hold ID",
  "confirmed_instances": 1,
  "object_instance": "transport_box_01",
  "station_id": "transport_box_01_pick_station",
  "model_id": "transport_box_60x60x120",
  "pose_worker": "/本次确认的/object_pose_register",
  "grasp_worker": "/本次确认的/graspnet_worker",
  "model_registry": "/本仓库/docs/evidence/m3_perception_planning_20260924/normal_box/registry.json",
  "grasp_model": "/本仓库/runs/grasp_pose_sim_20260921/models/graspnet_cuda_v2.pt"
}
```

示例中的registration_sequence=1仅说明合法的第一次登记；实际必须由本场owner登记序列赋值，并与session_id、独立Scene原文/hash、issued/expiry一并落盘，不是固定测试版本。probe从本场桌体几何按已批准table-only规则生成region，不读取目标GT作为分割输入。准备好专用projector和真实owner条件后启动probe；其输出registration.json后，所有者据登记值启动双实例并核对参数。probe在等待服务期间继续监控Scene/Health/租期，不能在服务启动后补改旧登记续期。

## 下一实际场次顺序与证据

1. M2建立全新受控READY场次、确认资源归属和静止状态，准备本场table region及owner Context来源；不复用上一场租约。
2. 所有者部署专用projector和两实例，核对实际二进制/overlay与参数读回；确认唯一发布者/服务、clock/TF/两类Health持续有效。不能仅按进程存在判就绪。
3. M2按M5提供的方案预先启动head限定有界录制，实际核对进程、订阅及输出。保留原RGB、完整provider PointCloud2、CameraInfo、CameraHealth、ProjectionHealth、TF和clock，并记录接收时钟。probe等待owner与两个Action就绪，取一个新的source Request，仅保存分割后cloud、该Request的Info/TF和原接收期限；通过source后同一轮发送两个Goal。owner config中的input_recording只关联外部目录，raw_input_evidence始终为EXTERNAL_RECORDING_PENDING_VERIFICATION；必须离线核对实包中的同capture/stamp/epoch，目录存在不算原始输入证据通过。外部订阅接收时钟不冒充source内部接收时钟。
4. 在原5s范围内记录双方结果；任何上下文变化/暂停过期/模型失败都保留原原因，取消peer并确认终态。一次结果结束退出，不用无限重试选出成功样本。
5. 证据分别标记 `SOURCE_LIVE_REQUEST`、`POSE_RESULT`、`GRASP_RESULT`、`BOTH_WITHIN_ORIGINAL_DEADLINE`；MTC/执行保持NOT_RUN。双模型按时返回失败只证明时效，不证明算法成功；一端无有效姿态/候选则该项失败。
6. 总调度据此决定下一项是模型/可见性问题、实时接入问题，还是单独推进长规划与新观测重验合同。单个实际问题计时与原证据保留。

## 待实际场次验证

- 两文件probe与clock0局部修改已获授权；构建和上述离线验证通过。
- 必须在新场建立真实静止Hold、六ACK与外部有界录制；当前没有可复用的成功租约。
- 生产M1 Context发布接口仍未实现；本轮只支持验证会话静态登记。
- 本场两个真实worker同时运行的完成时间、健康持续性、方柱6D结果和有效抓取候选。均需实际场次测得。

## 首轮静态审查修正

逐端取消与观察异常分别记录，不跳过另一端清理；同一1秒清理截止不延长。记录已接受Goal UUID，拒绝、等待接受、等待终态、UNKNOWN和观察异常均单列。both_within_original_deadline在双方均有明确结果终态时独立计算；未能判断时为null，source_and_models_validated另列。模型/worker文件hash与最初输入就绪在初次Scene查询前完成，避免启动工作消耗已登记快照的新鲜度。原始输入交M2/M5外部录制并保持待核对标志。
