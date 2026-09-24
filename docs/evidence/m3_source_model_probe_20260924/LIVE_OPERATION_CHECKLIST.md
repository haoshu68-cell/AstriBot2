# M3真实source与双模型：最小操作单（尚未执行）

**静止入口离线修订已通过，真实场未验**：`497c4e90` 的四文件修订已构建，20 项 C++ 检查通过，见 [stationary_gate_revision/README.md](stationary_gate_revision/README.md)。当前候选为 `runs/m3_source_model_probe_20260924/stationary_gate_build/m3_source_probe`，SHA256 `2626ed5e2b66d00b62d8183c9d30ab566266b76c0b5cc45530a10243bdb424fc`；SG7 真实负向及 source/双模型尚未验证，仍不放行实际场。原 `probe_build` 的导航禁止条件与六 ACK 合同互斥，已留档校验后删除。下段 98 项 hash 保留为修订前的历史核对，不是当前工作树声明。采集窗口及服务启动顺序仍有下文记录的缺口。

2026-09-24恢复后只读核对：4eac3656两文件源码、原证据、二进制、private header、链接/加载依赖共98项hash全部匹配。现有worker及模型文件存在，两个worker的ldd均无缺库。没有启动ROS/GPU/模型、重新编译或重跑原72项测试；冻结源码及manifest不变。原问题11:52起点、12:52检查点保留；无实际前置则真实验证保持NOT_RUN。

## 一、场次与资源

M2独占管理本场所有子进程、domain/partition/overlay、GPU时段和证据目录。必须先取得仍有效的真实静止Hold与六ACK，保留hold_id、coordinator_session_id、geometry clock_epoch、envelope epoch、单实例所有权证据，并持续保持。最近仅ARM→Hold的7话题基线录制不算本次模型输入；本单不能自行启动该场或续用其租约。

|资源|当前实际约束|尚未实测|
|---|---|---|
|任务|一个probe job；两个独立server，各单job busy门控；最多两个同时运行的worker子进程|真实并发成功率|
|位姿|object_pose_register，CPU CAD配准，每请求新进程|峰值RSS、OpenCV实际线程数、当前点云耗时；没有进程级内存硬上限|
|抓取|graspnet_worker --device cuda；一次一个CUDA worker，每请求重新加载模型；代码设置ATen intra-op线程1|实际总线程数、CUDA峰值显存、冷启动时延；不能由4.4MB权重推导显存预算|
|期限|每端Goal timeout=min(4秒,原请求ROS剩余期限)；整个结果仍受原capture+5秒及steady期限约束|两个真实worker是否共同满足期限|
|入口|60秒总steady等待上限；一次登记最多30秒；Scene读回从请求发出起最大1秒陈旧；清理共同1秒|当前负载下实际响应尾延迟|
|采集|专用CPU decimation1投影；9话题bag总进程窗口3秒，flush另最多10秒；元数据20秒|实际吞吐/丢帧；640×360@20Hz纯payload估算约318MB/3秒，非显存或磁盘硬上限|

不要并行启动其它模型任务。Gazebo渲染仍由M2拥有，不能把本场称作无其它GPU负载的单模型性能测试。现有worker按具体PID进程组收尾；probe只取消自己的两个Goal，禁止cancel-all。两服务的临时JobDirectory退出会清理worker临时文件，因此当前永久证据是probe的结果CDR/总inference_time和owner进程记录，不能声称永久保留了worker内部load_ms日志或显存峰值。

## 二、先准备所有者配置与专用输入

使用本目录runtime_overlay.bash；它只设置依赖路径，不设置本场资源归属。输入与输出目录分开：`input_recording`指向**已存在的本场采集父目录**；bag输出为其下尚不存在的`head_raw_bag`。probe的`output_directory`也必须尚不存在。不要把尚未创建的bag目录填为input_recording，也不要先创建bag自身输出目录。

owner配置字段见PLAN，均填本场真实值；修订新增必填 `hold_owner_id`，必须来自实际 HoldResources 持有者并匹配 typed ArmHoldStatus，不能用 probe session_id 代替。registration_sequence由owner预留本场实际登记序号，不能抄旧值。M2还须独立留存冷启动、导航入口已清空及最终命令链排他证据；事件流静默不能作为空闲证明。固定文件路径：

- pose_worker：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/m3_perception_planning_20260924/install/astribot_object_pose_core/lib/astribot_object_pose_core/object_pose_register`
- grasp_worker：`/home/yjh/.cache/astribot/graspnet/build_cuda/graspnet_worker`
- grasp_model：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/grasp_pose_sim_20260921/models/graspnet_cuda_v2.pt`
- model_registry：`/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m3_perception_planning_20260924/normal_box/registry.json`
- model_id：`transport_box_60x60x120`；object_instance：`transport_box_01`；station_id：`transport_box_01_pick_station`。若本场实例不同，当前单实例准备条件不成立，不能只改ID冒充匹配。

专用projector由M2的既有child管理器启动，未授权时不运行：

```bash
/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_20260923_065115/install/astribot_s1_perception_components/lib/astribot_s1_perception_components/rgbd_pointcloud_node \
  --ros-args -r __node:=single_box_projector \
  --params-file /home/yjh/WorkSpace/astribot_sdk_ros2/runs/m3_perception_planning_20260924/install/astribot_s1_manipulation_perception/share/astribot_s1_manipulation_perception/config/single_box_projection.yaml \
  -p 'activation_topic:=""'
```

YAML为CPU、无fallback、decimation1、depth范围0.08–5m、pair age0.25秒、use_sim_time=true。空activation_topic仅表示数据投影不要求Session激活，不授予任何模型/运动权限。保存参数读回、唯一publisher、有效ProjectionHealth与原depth/info同stamp的完整XYZ。节点存在或发现topic不算数据有效。

## 三、登记、采集、双实例：严格顺序

1. M2确认上述静止所有权、相机/专用投影器健康。先启动M5的20秒head元数据observer并核对实际订阅；与41-topic observer同名，不能并行同名运行。尚不启动本场两个推理server，避免probe提前发送。
2. M2的child启动以下probe；它独立取完整Scene、公共canonical绑定与真实URDF links，按owner序号登记。等待本次输出目录的registration.json或明确失败，不能读上场文件。

```bash
: "${M3_OWNER_CONFIG:?本场已核实owner配置绝对路径}"
: "${M3_PROBE_BINARY:?总调度确认静止修订验证通过的新二进制绝对路径}"
"$M3_PROBE_BINARY" \
  "$M3_OWNER_CONFIG" --ros-args -p use_sim_time:=true
```

3. 复制`normal_box/inference.template.yaml`至本场证据目录形成`inference.yaml`，仅将optical_frame/camera_id设置为新registration.json的实际值；核对worker/model/registry路径与owner配置完全一致。不能直接信任模板中的历史frame。将registration.json三个整数calibration_revision/planning_scene_revision/envelope_epoch作为以下launch参数；三者必须大于零。clock_epoch可以为零，但它不是paired launch的这三个参数之一。
4. **先启动M5 9话题3秒raw bag，确认该recorder存活且九个实际兼容订阅已建立，再启动下面的两实例launch。** probe会在两server服务与Action就绪、参数核对后立即取新帧并发Goal；没有额外“等待bag”门控。不要先启动两server，再期望事后启动bag能补录输入。

```bash
: "${M3_RUN_DIR:?本场证据目录}"
: "${M3_CALIBRATION_REVISION:?来自本次registration.json}"
: "${M3_SCENE_REVISION:?来自本次registration.json}"
: "${M3_ENVELOPE_EPOCH:?来自本次registration.json}"
ros2 launch /home/yjh/WorkSpace/astribot_sdk_ros2/runs/m3_source_model_probe_20260924/clock0_install/astribot_s1_manipulation_perception/share/astribot_s1_manipulation_perception/launch/paired_inference.launch.py \
  config:="$M3_RUN_DIR/inference.yaml" \
  calibration_revision:="$M3_CALIBRATION_REVISION" \
  planning_scene_revision:="$M3_SCENE_REVISION" \
  envelope_epoch:="$M3_ENVELOPE_EPOCH"
```

当前3秒窗口包含DDS发现、订阅核对及server启动，**尚不能保证覆盖最终capture**。若窗口已结束而server尚未启动，不再为这次采集新发模型任务，应有界取消自己的probe并记缺证；若启动后模型请求落在窗口外，保留结果并将raw证据判失败。不能自动续录、重新盖stamp、重复试到成功。若实际证明窗口布局不够，需要总调度另行调整采集方案；本次不新增门控功能、不改冻结源码。

5. 两server分别为/object_pose_server与/grasp_proposal_server。前者grasp_worker为空，后者pose_worker为空；未用Action已重映射disabled名字。确认每个正式Action只有一个server。probe会读取并核对readonly参数描述符、实际版本/worker路径、cuda、健康topics、age预算和use_sim_time；任一失败结束该job，不在线set版本绕过检查。版本或Scene变化后本次登记作废，不能修改旧registration.json续期。
6. 保留report.json、registration.json、Request/结果CDR、Action UUID/终态/取消证据。本probe不调用MTC/执行。按M5手册flush并保留真实退出码、metadata、完整raw bag与topic计数；124只表示限时到达，不表示数据验收通过。
7. 用report.capture.stamp_ns精确核对RGB/depth/CameraInfo/专用cloud和Health/epoch/TF/clock上下文。input_recording目录存在永远只表示关联；probe的raw_input_evidence仍PENDING，必须单独形成实包核验结论。若无实际模型capture，不能把候选静止帧称为模型输入。

M5唯一采集操作来源：[CAPTURE_HANDOFF.md](../mainline_m5_20260924/CAPTURE_HANDOFF.md)、[QoS](../mainline_m5_20260924/head_raw_qos.yaml)、[元数据配置](../mainline_m5_20260924/head_metadata_topics.json)。本单复用它们，不维护第二份录制命令。

## 四、结论口径与收尾

SOURCE_LIVE_REQUEST、POSE_RESULT、GRASP_RESULT、BOTH_WITHIN_ORIGINAL_DEADLINE、RAW_INPUT_EVIDENCE分别判断；及时失败不等于算法成功，成功结果缺原始输入也不能称完整验收。此处位姿worker是CAD配准，不是FoundationPose。FoundationPose仍待FP-BACKEND-01恢复条件。

任何失败保留原原因，probe最多1秒取消/等待自己的Goals；终态不明交对应server所有者处理。M2核对PID/startticks后收尾本场进程、记录returncode与残留，不能停止其它会话。没有真实Hold/六ACK/新鲜登记则本单停在准备完成，不启动实际probe。
