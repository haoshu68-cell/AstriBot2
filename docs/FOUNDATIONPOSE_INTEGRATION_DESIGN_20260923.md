# FoundationPose 已知 CAD 刚体整合技术方案

日期：2026-09-23。状态：待实施的技术设计。本轮完成当前源码核对、官方资料核对与方案评审，没有安装模型、启动 ROS/Gazebo、运行推理或验证机器人动作。

## 1. 目标与选型

用户已确认首期对象为**已有 CAD/网格模型的刚体**。沿用此前仿真优先范围，先交付物体自身的 6D 位姿、连续跟踪、丢失拒绝和重定位，再接入抓取规划。首个闭环限定：单个目标、单路 RGB-D、固定底盘、抓取前静止目标。后续增加多实例、腕相机和相机运动；真机标定、动态抓取和运输不是首期验收结论。

推荐采用 **Isaac ROS release-3.2 的 FoundationPose C++/TensorRT 后端 + 本项目 C++ 感知监督与质量门控**。保留现有 PPF/ICP 为对照基线和显式可选后端，保留现有 GraspNet 服务。FoundationPose 负责物体位姿，GraspNet 负责夹爪抓取候选，MTC/MoveIt 与任务执行器继续负责规划和执行。

| 路线 | 用途与代价 | 决策 |
|---|---|---|
| Isaac ROS 3.2 / C++ / TensorRT，独立容器与进程 | 官方已有 Humble 接口；需适配 NITROS/GXF、模型资产和项目生命周期 | 主线，先做兼容性实测 |
| NVlabs 研究实现 / Python | 适合复现官方算法、检查输入和对照输出；第三方 Python/CUDA 依赖较重 | 仅离线基准、模型准备和诊断，不新增常驻 Python 业务服务 |
| 自行完整移植网络、渲染与搜索到 C++ | 可减少外部 ROS 依赖，但复现、优化与维护成本大 | 首期不采用 |

FoundationPose 的已知模型模式需要图像、深度、内参、目标初始化区域和 CAD；它本身不负责开放词汇识别或确认目标实例身份。新 CAD 通常不需重新训练姿态网络，但目标检测/分割仍需具备识别该物体的能力。[官方算法说明](https://github.com/NVlabs/FoundationPose)

## 2. 当前项目基线与准确改动点

本次读取的是 `HEAD=09bc10d7` 上叠加大量未提交/未跟踪文件的工作区，不能仅用该提交复现现状。以下是源码证据，不代表安装产物或当前运行栈已采用。

| 当前入口 | 已有能力 | 本次需补齐 |
|---|---|---|
| `astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp` | C++ ComputeGrasps / EstimateObjectPose；输入检查、worker 管理、取消/版本检查 | 现有 pose 分支写 `scene.xyz` 并解析 PPF 协议，不能只换 worker 路径；需抽象后端和增加 RGB-D 请求路径 |
| `astribot_perception_msgs/action/EstimateObjectPose.action` | 已知 model_id + 分割后 PointCloud2 + 来源/场景上下文 | 没有 RGB、depth、CameraInfo、mask 或不可变快照引用 |
| `astribot_object_pose_core` | C++ PPF/ICP、可见性与歧义门控 | 保留算法和原门槛，不冒称它是 FoundationPose |
| `astribot_graspnet_runtime` | LibTorch C++ GraspNet worker | 复用现有接口，绑定同一目标实例和采集快照 |
| `rgbd_object_pose_node.cpp` | 检测 ROI 的可见表面中心，orientation_valid=false | 不把该输出改名当作完整物体位姿；作为定位/初始化线索 |
| `yolo_detector_node.cpp` / `rgbd_vision_pipeline.yaml` | 当前检测框，配置明确 `segmentation_supported:false` | 实例 mask、CAD 身份匹配、跨帧 object_id/track_id |
| `camera_health`、RGB-D projection、`astribot_sensor_sync` | 相机健康、处理 epoch、有界处理及同步证据框架 | 不能把健康状态或共同时间戳当作深度已对齐、物理硬同步或外参准确 |
| `astribot_s1_transport_mtc/src/mtc_planner.cpp` | 接收场景和目标位姿；固定规划 frame 为 `astribot_torso_base` | 感知目标→抓取目标选择→场景事务→MTC 的接线仍需独立实现与验证 |

关键源码位置：姿态服务 524–577 行、消息定义 1–15 行、MTC 目标 frame 检查 82–102 行。工作区旧的视觉 YAML 仍写 `model_service_implemented:false`，与现有 GraspNet 服务不符；实施时同步修正说明，不以旧配置推断能力不存在。

历史 [GraspNet 与姿态验证记录](GRASP_POSE_SIMULATION_20260921.md)记录旧相机条件下 PPF/ICP 七场景 **2/7 接受、5/7 拒绝**，不是七场景成功；初期目标是改善可观测场景的接受率与跟踪能力。该记录不能作为新相机外参下的当前精度结论。本轮未复跑这些记录。

本次关键文件 SHA256：

```text
EstimateObjectPose.action
7e07d7ad9b38690537da480f09ee409d197a3d5791b00a06353c8f1b18e0da3c
ObjectPoseObservation.msg
954dfd38d6aa668a46752e4fae85d3193c092308d159b07ff36d261de691ae09
manipulation_perception_server.cpp
544ba0fcfb8b0934bf7c57274d106d29a0fb7c8b35c198093fb8a2c8563b264e
```

## 3. 运行环境与依赖冻结

本轮只读确认本机为 Ubuntu 22.04.5、RTX 4090、24564 MiB 显存、驱动 580.178.04。硬件代际和驱动满足所查 Isaac ROS 3.2 基础要求，但未检查当前可用显存、Docker GPU 通路或进行本机兼容性推理，不能由配置直接承诺实时性。[3.2 计算平台要求](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html)

实施冻结 `isaac_ros_common`、`isaac_ros_pose_estimation` 等实际依赖的 commit、容器 digest、Debian 包版本、CUDA/TensorRT/GXF 版本及 ONNX SHA256；不用浮动 `latest`。本轮核对的 pose estimation release-3.2 源码为 `9caca619bcc9d637b3107e17c1a77132c9d7863b`，其余依赖版本由 P0 解析、复现后生成完整锁定清单，不凭空填写。Humble 主线采用 3.2 文档，不能混用当前其他 ROS 发行版的安装命令。

先采用官方 `1.0.0_onnx` 的 refine/score 模型，在目标机器生成 TensorRT engine。3.2 文档指出 TensorRT 10.3+ 下 FP16 存在精度问题，初版采用 FP32；任何精度优化均需独立对照。转换阶段至少需要文档所列的 7.5 GB 空闲显存，这不是运行时总显存上限。[3.2 安装与转换说明](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html)

项目运行时模块全部使用 C++。Python 仅用于 launch、资产转换、采集、回放、评分，以及隔离的官方参考程序。现有 LibTorch/CUDA 环境与新 TensorRT 容器不混合链接，不修改全局 CUDA 软链接或共享工作区安装链。

依赖清单分别记录研究代码、Isaac ROS 包、GXF 组件、网络权重和 CAD 资产的来源与许可；不能由某个 ROS 包的 Apache-2.0 推断所有模型和资产采用同一许可。研究版 LICENSE 有研究/评估用途限制；NGC 权重按所下载版本的随附许可记录，不把其他下载渠道模型卡自动套用到旧权重。[研究版许可](https://github.com/NVlabs/FoundationPose/blob/main/LICENSE)、[Isaac ROS 许可](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation/blob/release-3.2/LICENSE)、[NGC 模型来源](https://catalog.ngc.nvidia.com/orgs/nvidia/isaac/models/foundationpose/1.0.0)

## 4. 数据链与模块划分

下图箭头表示数据流；所有进入执行层的数据仍需现有任务准入。

```text
现有 RGB / depth / CameraInfo + CameraHealth + 相机会话
  → C++ RGB-D snapshot adapter：整流、深度配准、时间/版本检查
  → C++ 目标实例选择 + mask
  → 不可变 RgbdObjectSnapshot
       ├─ FoundationPose register / track → 原始物体姿态
       │    → C++ 几何/身份/时效/对称性门控 → ObjectPoseEstimate
       └─ 同一 mask 的物体点云 → 现有 ComputeGrasps → 抓取候选

ObjectPoseEstimate + 全场景障碍 + 对象账本
  → 任务拥有的场景投影/读回 → 版本化规划快照
  → CAD 抓取模板或 GraspNet 候选 + TCP 转换
  → MTC/MoveIt：IK、碰撞、路径与任务上下文检查
  → 现有执行器、控制器和保护链

Gazebo 真值 → 独立评分器（不进入 mask、姿态或规划输入）
```

| 模块与目录（新增为拟议） | 职责 | 不拥有的权限 |
|---|---|---|
| 扩展 `astribot_perception_msgs` | 新快照、位姿估计、跟踪健康、Action 契约 | 不做算法与执行准入 |
| 扩展 `astribot_s1_perception_components` | `rgbd_snapshot_node.cpp`、`object_mask_node.cpp`、有界图像缓存与跨帧身份 | 不提交 attached object 或操作机械臂 |
| 新建 `astribot_foundationpose_runtime` | Isaac 后端 C++ 适配、进程/请求监督、模型资产校验、注册/跟踪调度；核心与 ROS/backend 分层 | 不授予运动、不作为全局场景写入者 |
| 扩展 `astribot_s1_manipulation_perception` | 新 Action 门面，旧 PPF/GraspNet 服务兼容，规划上下文检查 | 不把网络分数解释成抓取成功率 |
| 扩展 `astribot_s1_manipulation` | C++ 位姿到抓取目标转换、候选选择及任务场景接口 | 不绕过现有任务执行器 |
| `tools/vision` | 资产准备、固定数据集、对照评测、故障注入和证据归档脚本 | 不作为持续在线业务服务 |

工程依赖方向为 `ROS adapter → core/contracts`、`Isaac backend adapter → core/contracts + Isaac`。独立核心定义 PoseBackend、状态机与质量规则，不依赖具体 ROS 或 NVIDIA 节点。

首期一个目标、一个 GPU 在途推理、一个最新待处理槽；模型启动时加载并预热，不在每次 Action 重新载入。正式推理不继续沿用逐帧落盘 XYZ/JSON 协议；文件仅用于回放和诊断。

## 5. 输入几何、mask 与资产契约

### 5.1 RGB-D 必须形成真实一致的快照

首个相机候选使用 `torso_rgbd`，基于项目已有历史推理入口；P0 必须重新确认当下视野、有效深度和几何一致性后才能采用。2026-09-23 跨窗口核对后更正：当前 six-camera preset 关闭后处理，历史实测 raw 入口为：

```text
/camera/raw/torso_rgbd/image
/camera/raw/torso_rgbd/depth_image
/camera/raw/torso_rgbd/camera_info
message frame: astribot_s1/astribot_torso_link_4/torso_rgbd_sensor
```

依据为 `docs/evidence/joint_acceptance_20260923/camera/six_camera_raw_streams.json`，不是本轮在线观测。旧 `/camera/torso_rgbd/color/image_raw` 路径仅可在明确启用且验证相应后处理配置时使用。原始 frame 不能仅重命名为 `torso_rgbd_camera_optical_frame`，需核验实际传感器轴与几何变换。

话题名称不能决定是否已整流。适配器明确检查并输出：RGB8、32FC1 米制 Z-depth、mono8 实例 mask、同尺寸/同光学 frame 的 pinhole CameraInfo。16UC1 毫米仅在入口转换一次；不把沿光线距离直接当 Z 值。

RGB 必须整流；深度必须投影到该 RGB 光学坐标和像素平面，包含深度内参、RGB/深度外参及遮挡 z-buffer。分辨率相同、使用同一个 remap 或换 frame 名字都不能证明已配准。遮挡孔洞保留 invalid，不以补洞制造测量。

裁剪、缩放和去畸变必须同步更新 K/P 与 mask 的像素坐标，保存处理变换及 revision。验收包含非零畸变、非中心裁剪、非整数缩放和深度边界，不只测试图像中心的理想相机。

本轮发现 `camera_calibration_postprocess_node.cpp:113–149` 执行 remap，同时 `publishInfo` 仍复制 profile 的 D；下游点云又按 K 投影。这要求在接入前明确这条仿真链究竟产生什么成像几何，验证 K/D/P、尺寸与图像一致性，避免二次去畸变。采用已整流输入时新分支使用对应 K/P、D=0；必须先验证图像实际含义，不能只改元数据掩盖问题。

原始 RGB、depth 的各自采集时间保存在快照中；mask 继承它实际分割的 RGB 帧，CameraInfo 绑定校准版本。仿真首期要求同次采集的精确配对。未来允许软配对时，需约定运动补偿与最大误差，不能改写原始时间戳来通过 ExactSync。

### 5.2 mask 与物体身份

- 离线算法基准可用人工检查的可见像素 mask，标记 `mask_source=manual_fixture`；它只证明姿态后端，不计自动分割验收。
- 首个运行时场景采用 C++ 检测/任务 ROI + 深度连通域、桌面剔除和机器人自遮挡过滤；限定物体可分离的桌面环境。ROI 包含多个实例或不能分离时拒绝，不能只选面积最大的连通域冒称指定对象。
- 任意杂乱环境需要单独接入实例分割后端与适配该物体的模型，网络推理仍用 C++ TensorRT/ONNX。单纯检测框或颜色夹具不能作为一般实例分割验收。
- `model_id` 标识 CAD 类型，`object_id` 标识业务实例，`track_id` 标识此次跟踪，`detection_id` 标识某一帧。现有 YOLO 的检测 ID 含时间戳，不能直接用作持续对象 ID。
- 同型号多物体以任务选择、分割、空间关联维持身份；接近/交叉导致关联歧义时 `IDENTITY_AMBIGUOUS`。维护 `association_epoch`，遮挡重现、检测器重启或目标被替换后重新确认；几何匹配同 CAD 不证明是原实例。首期的单实例条件也需运行时检查。首期不做无证据的自动跨相机身份融合。

### 5.3 model registry

现有 `.xyz` 点集不足以支持渲染式位姿估计。为每个模型增加受控注册资产：

```yaml
model_id: asymmetric_union_v1
units: m
mesh: meshes/asymmetric_union/centered.obj
texture: meshes/asymmetric_union/albedo.png
collision_mesh: meshes/asymmetric_union/collision.obj
object_frame: asymmetric_union
mesh_from_object: <确定的刚体变换，不是推理结果>
symmetry: <none / finite transforms / continuous axis>
dimensions_m: <离线验证的包围尺寸>
mesh_sha256: <准备资产后生成>
texture_sha256: <准备资产后生成>
grasp_templates: <可选，物体系下的 TCP 抓取变换>
```

这里是字段模板，不是可直接启动的配置；实施产物必须填入真实资产与哈希。模型尺度、原点、朝向、法向、可视网格和碰撞网格分别验证。渲染后端所用中心化 mesh 与业务对象坐标之间的变换永久保存；不能因为 mesh 原点重置而改变物体业务坐标。无纹理 CAD 先用明确材质验证适用性，不把生成纹理当真实观测。

## 6. ROS 接口与时间语义

优先增加类型与端点，不直接修改旧 EstimateObjectPose / ComputeGrasps 的输入语义；旧 PPF 客户端保留。现有消息缺少的字段通过新 wrapper 承载，避免丢失元数据的旧客户端误消费。

| 拟议接口 | 内容 | 语义 |
|---|---|---|
| `RgbdObjectSnapshot.msg`（内部数据） | snapshot_id、RGB/depth/mask/K、各原始 stamp、frame、camera_id、object_id/model_id、source/clock/processing epoch、calibration/mask revision、有效期 | 不可变；对象身份与像素在同一快照 |
| `EstimateObjectPoseRgbd.action` | task_id、seed_snapshot_id、object_id/model_id、expected revisions、墙钟总期限 | 在线一次请求获得新鲜有效位姿，可经历注册和追踪；结果显式含 seed 与最终 accepted_snapshot_id |
| `TrackObjectPose.action` | object_id/model_id、camera_id、会话最大时长、相机会话 token、总获取预算 | 有界持续跟踪；feedback/state + 观测 Topic；取消撤销本会话输出，不代表机器人停稳 |
| `ObjectPoseEstimate.msg` | 既有 ObjectPoseObservation + snapshot/track/session ID、association_epoch、backend_revision、cad_revision、weights_revision、engine_revision、processing/clock epoch、quality_flags、symmetry、covariance_valid | CAD 与网络权重版本分开；原始质量分与几何指标分开；valid_until 不续旧样本 |
| `PoseTrackingHealth.msg` | READY/INITIALIZING/TRACKING/LOST/RECOVERING/FAULT、fault reason、last sample、worker_epoch | 状态心跳不刷新 last sample 或观测有效期 |

注册完成可能返回较旧 t0 的姿态。它只作为后端私有的跟踪种子，不发布为可规划结果；随后使用同实例的新 RGB-D 帧跟踪，只有最终采集龄期与墙钟延迟都合格才完成在线 Action。若跨帧运动或身份关联不足以从种子恢复，重新注册并继续使用原总期限；不能重置超时预算。离线固定快照估计使用专门回放工具，不伪装为在线有效结果。

另外分别冻结最大 seed_age、相邻跟踪帧间隔与允许相对运动量；由 P0/P3 的运动测试确定具体值。首期获取窗口要求目标、相机和相关机器人关节均静止。新鲜时间戳不证明从旧种子正确收敛，仍必须通过该新帧的独立渲染/深度一致性检查；超出经过验证的收敛范围就拒绝或重注册。

快照注册时要求输入新鲜；注册期间保留图像的有界引用和独立任务期限。快照的观测有效期到期不延长，内部推理假设可在任务期限内继续计算，但在新帧验证之前始终无下游有效性。最终结果携带实际使用的新快照，不把 t0 结果重盖成 t1。

连续原始位姿跟踪只依赖传感器、标定、模型和实例状态，不绑定每次 PlanningScene 更新；任务侧提取规划快照时再绑定 scene_revision、envelope_epoch、机器人状态及适用的定位版本。在线一次性 Action 若指定任务上下文，则请求中途换版拒绝。这样不会因任务自己提交场景版本而让所有视觉跟踪反复失效。

输入图像 QoS 与实际发布端匹配，建议 SensorData KeepLast(4)；组帧与后端待处理队列均有界。元数据/状态用 reliable 有界队列；raw backend topic 仅在私有 namespace 中使用。图像保持共享引用，跨容器先以标准 ROS 消息连通并测量复制开销，不能预先宣称跨进程或跨容器零拷贝。

snapshot registry 由 snapshot adapter 唯一拥有，ID 含 boot nonce 与单调序号，永不复用。消费者通过有期限的 pin/release 获取不可变内容；元数据引用与像素内容一并存活，容量不足时拒绝新 pin，不覆盖在途快照。首期建议每相机缓存最多 4 组、最多 2 组被 pin，总像素字节预算 128 MiB，单组像素预算 32 MiB；输入 DDS 分配和后端显存另计。具体内存预算在 P1 按分辨率与实例数冻结，快照过期仍不得通过 pin 续期。

由 C++ 外层监督器创建唯一 job/session/worker epoch。完整关联键为 `{worker_epoch, session_id, request_id, snapshot_id, frame_sequence}`。上游缺少项目 request_id 时，一个私有后端实例仅允许一个在途请求，并验证输出源时间与唯一输入映射；取消后必须完成可证明的队列/回调排空屏障，或回收并重启该后端图，再接受新请求。不能仅靠“收到下一个 Detection3DArray”关联当前请求。原始后端 TF 重映射到私有诊断通道，不进入权威对象 TF；下游不得仅查 latest TF 判断有效性，因为停止广播不会清除已有 TF 缓存。

## 7. 注册、跟踪与质量门控

```text
UNCONFIGURED → READY → WAITING_INPUT → INITIALIZING → VALIDATING
                                                      ↓
                                                   TRACKING
                                                      ↓ 质量/身份/时间失效
                                           LOST → REACQUIRING
                                                      ↓ 超预算/后端故障
                                                    FAULT
取消/租约撤销 → REVOKED；旧 worker 未确认结束 → QUARANTINED
```

Isaac 3.2 默认 selector 使用四路 ExactSync，跟踪模式仍等待 RGB/depth/mask/info，且默认按 20 秒墙钟周期重新注册；这不是质量驱动的 lost/reinit 机制。因此项目使用 C++ 监督器选择注册/跟踪输入，替代默认 selector 的任务控制职责；跟踪所需的 RGB-D/K 与 mask 更新频率分别定义，不每帧调用慢分割来阻塞 tracker。[selector 源码](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation/blob/release-3.2/isaac_ros_foundationpose/src/foundationpose_selector_node.cpp)

每次接受位姿前检查：

1. 数值有限、R 正交且 det(R)≈1、单位/尺度正确、目标在有效深度与工作空间内。
2. 在该帧相机视角渲染 CAD，比较可见深度残差、观测支持率和轮廓重叠；区分被其他物体遮挡的表面与真正冲突的表面。
3. instance 与 mask 关联、跨帧运动界限、对称等价类、相机运动补偿、模型/标定版本一致。
4. 数据采集时间、单调接收龄期、推理时间、worker/source/clock epoch 与任务有效期。
5. 发布前在同一提交临界区再验证上下文，防止取消/重启与成功结果竞争。

本轮所查 Isaac decoder 用 score 选择候选，但没有将该网络分数写入最终 Detection3D；tracking 模式还跳过 score 输入。因此结果必须支持 `raw_score_available=false`，不能读到默认零值就当作模型置信度。若后续显式暴露 score，它仅作排序或有标定依据的门控，不能解释成置信概率或协方差。[固定版本 decoder](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation/blob/9caca619bcc9d637b3107e17c1a77132c9d7863b/isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/foundationpose_decoder.cpp#L118)

没有校准协方差时 `covariance_valid=false`，旧兼容输出沿用保守 unknown 表示，不能把 ICP 残差平方填成六自由度协方差。下游融合或规划根据经验验证的误差界保守处理。缺失网络 score 不阻止独立质量检查，也不由项目虚构补值。

FoundationPose 采用专门定义、固定配置的 RGB-D 质量指标；不把旧 PPF 覆盖率数值换个名字使用。旧基线门槛保持原值。新阈值在独立标定集合上确定，冻结后在留出的测试集验收，不按失败场景逐个放宽。

对称模型输出等价类与可观测自由度：轴对称物体不能声称唯一 yaw。供旧客户端的 `orientation_valid=true` 仅在它要求的朝向确实可辨时设置；不能用单位四元数替代未知旋转。只对称外形但抓取/放置功能不对称的物体，仍需可辨识纹理/特征或受限任务，不允许仅凭 ADD-S 合格选任意朝向。

候选初值：连续 3 个不同采集帧通过门控后进入 TRACKING；任一硬故障立即失效，软质量不足也立即撤销规划可用性，只允许内部短暂保留种子用于恢复。不能等待累计若干失败才继续给下游发布旧有效位姿。

## 8. 坐标、场景与抓取消费

统一记号 `T_A_B` 表示把 B 系点转换到 A 系。对采集时刻 t：

```text
T_base_object(t) = T_base_camera(t) · T_camera_mesh(t) · T_mesh_object
T_base_tcp_goal  = T_base_object · T_object_tcp_grasp
```

其中 base 为当前 MTC 所需 `astribot_torso_base`。必须核实具体后端输出相对原始 mesh 还是中心化 mesh，再绑定注册表变换，避免做两次中心补偿。RGB optical convention、Gazebo 渲染坐标与业务物体系用已知非对称模型、投影点和多轴旋转验证；不能看可视化“差不多”就通过。

TF 查询使用图像采集时刻，禁止使用最新 TF。腕相机必须把同刻关节状态、动态 TF 和静态外参一起纳入来源；缺对应 TF 时拒绝。首期不做相机间平均融合；后续每路独立估计、转换至共同物体系后再按相关性与不确定性融合。

抓取分两条接入路径，首期优先模板路径验证数据链：

- 已知 CAD 的预定义抓取模板：物体 pose 变换模板 TCP，再生成 pregrasp、接近/退出方向和夹爪宽度。
- 已有 GraspNet：使用相同快照的分割点云得到夹爪候选，再通过已验证的 GraspNet frame→夹爪/TCP 变换；物体姿态用于身份、CAD 碰撞模型、语义面及目标约束。完整场景点云仍用于碰撞，不能只保留目标点云。

任务拥有的场景投影模块将 CAD collision geometry 与 pose 提交给 MoveIt，等待 ApplyPlanningScene 响应，并通过独立完整场景读回核实目标 ID、姿态、几何摘要、world/attached 归属和其余相关上下文。应用层管理 scene revision；不能假设 MoveIt 服务自带本项目版本 ACK。读回不一致则不发起 MTC。

Apply 与 readback 本身不是原子事务。现有任务所有者需串行化该对象的视觉提交、attach/detach 和删除，在提交前、读回后及规划消费时检查 `expected_scene_revision + object_revision + attachment_generation`；身份与版本不匹配就撤销。其他相关场景写入也必须进入同一版本观测边界，不能让未登记的外部 writer 绕开校验。并发验收要求同一 object_id 不同时出现在 world 与 attached 集合；检查完成后发生版本改变，同样使旧规划失效。

为避免 10 Hz 观测不断改场景使规划永远失效，任务在明确的采集边界建立规划快照；原始跟踪继续更新。规划/执行前检查物体与机器人漂移是否仍在冻结预算内，超限撤销该计划并重新建立快照。不能简单忽略新观测或给旧 pose 续期。

已有 scene/账本写入逻辑继续拥有提交权；新 C++ 模块作为其受控客户端，不并行成为第二写入者。抓持确认后由原任务执行器提交 ATTACHED；视觉观察仅报告一致性，不覆盖附着变换、不因丢失目标就声明物体已释放或载荷为空。处理故障是否停车/保载由任务与执行保护层决定，感知节点只撤销能力并报告原因。

## 9. 资源、超时与恢复

以下为**建议起始预算，不是本机性能实测或现有验收结果**；P0 输出测量后冻结注册/跟踪/重定位预算，既有安全时效不能为通过而放宽。

| 项目 | 起始策略 |
|---|---|
| 首个相机/目标 | 单相机、单实例、模型预加载；优先复查 torso_rgbd |
| 初始化 | 总请求期限候选 5 s；内部旧帧估计不能直接供规划 |
| 跟踪 | 目标 10 Hz，稳态后端延迟 P95 ≤100 ms；同时报告端到端，不把后端延迟当整体延迟 |
| 可规划新鲜度 | 不超过现有 250 ms 上限，并采用下游消费者更严格的要求；同时检查源时间与单调墙钟 |
| 重定位 | 候选预算 2 s；身份不明或运动超出可恢复范围时允许明确失败 |
| 待处理队列 | 一个在途、一个最新待处理槽；丢旧计数，不积压补算 |
| GPU 压力 | 初期串行调度 FoundationPose 注册和 GraspNet；保留导航/相机资源余量 |
| 长时验证 | 至少 30 min 联合负载，观察热稳定、显存、RTF、消息龄期和故障恢复 |

独立容器/进程不等于 GPU 资源隔离。引擎构建在独占授权窗口进行；运行过程中监测显存高水位、OOM、处理超时和相机新鲜度，过载时减少可选感知工作或拒绝新推理，不修改导航保护预算。

取消撤销 session，立即阻止结果进入可用缓存；推理 kernel 是否完成另行监督。超时回收只针对有身份记录的本任务 worker；确认退出后才生成新 epoch 并启动替代 worker。不能把发送 kill 当作回收成功，也不因 GPU 卡死自动 reset 共享设备。旧进程未退出时隔离，CPU/PPF 回退只能由显式策略选择并重新准入，所有结果标实际 backend。

容器到宿主的 /clock、ROS domain、DDS 配置、namespace、图像 QoS 和大包容量必须在 P1 验证；开发仿真使用独立 instance、domain、Gazebo partition、端口、锁、build/install 与日志，并核对会话拥有者。不能使用历史固定域号推断环境空闲。

## 10. 有序实施与交付门槛

每阶段保留失败记录、实际二进制/配置/模型版本与可复现输入。新功能默认关闭，不覆盖共享工作区 install。

| 阶段 | 具体工作 | 完成条件与交付物 |
|---|---|---|
| P0 基线与后端可行性 | 冻结现有 PPF/GraspNet 数据；准备一个非对称 CAD；隔离构建 3.2 环境、FP32 engine；官方示例与项目录制 RGB-D 运行 | 真实模型输出、依赖 manifest、本机初始化/跟踪分位数/显存；证明坐标与尺度正确，再冻结预算 |
| P1 快照和输入闭环 | C++ RGB-D rectification/alignment、缓存、mask 与实例选择；修正相关 CameraInfo 契约；新增消息和几何测试 | mask 与像素/深度同源；无单位、时间、几何或身份混用；原相机/点云路径回归通过 |
| P2 FoundationPose Action | C++ backend adapter 与 supervisor、EstimateObjectPoseRgbd、新 estimate/health；接入版本、取消、超时、提交检查 | 静止目标单次获取；超时/取消/迟到结果不泄出；与旧 PPF 同输入对照 |
| P3 跟踪与恢复 | TrackObjectPose、有界会话、质量触发 LOST、重注册、新帧恢复；相机运动测试 | 失效立即撤销可用性，恢复使用新快照/epoch；不因时间重标或错误实例恢复 |
| P4 消费到 MTC | 任务规划快照、CAD 碰撞投影/读回、模板抓取、GraspNet 候选选择与 TCP 转换 | 先只规划与 RViz；再由既有执行链做固定底盘仿真抓放；场景和账本提交可追溯 |
| P5 联合回归和放行 | 七场景加新边界矩阵、30 min 多任务负载、故障恢复和回滚演练 | 分别给出算法、感知服务、MTC 规划、仿真抓放结论；不合格项保持关闭 |

关键依赖：`P0 → P1 → P2 → P3 → P4 → P5`；P0 资产和 P1 消息/几何设计可并行，P4 必须等待感知来源和时效规则稳定。复杂多实例分割与腕相机可作为后续独立阶段，不能替代 P2/P3 的已知对象基本验收。

建议工时用于安排资源，非承诺：一名熟悉本项目的工程师完成 P0–P3 约 10–15 个工作日；P4–P5 再约 5–8 日。复杂分割模型、旧版依赖获取失败、共享 GPU 时间不足或资产问题另计。P0 的实测结果用于修订排期，不能仅凭官方帧率估算。

## 11. 验收矩阵与指标

需求对应：R1 准确物体姿态；R2 输入、身份与版本一致；R3 可取消/有界恢复；R4 下游场景与执行责任正确；R5 共存资源不使既有保护退化。全部场景初始标记 `execution=NOT_RUN`；文档形成不等于执行 PASS。

| 场景 | 关联需求/风险 | 独立判据与预期 |
|---|---|---|
| SC01 clear/far/near/tilted/yawed/close/occluded 固定旧数据 A/B | R1，算法与输入影响混淆 | 同一真实 RGB-D、mask、CAD、采集 TF；PPF/FP 分别报告接受、拒绝和误差，不能混用两次相机配置 |
| SC02 新相机配置、多轴姿态、距离、光照与背景 | R1/R2，旧证据外推 | 每个部署目标至少 100 个独立位姿/条件样本，冻结标定/测试拆分；按场景和物体分母统计 |
| SC03 物体完全不可见、单平面、对称轴、错误 CAD/尺度 | R1，伪确定性 | 独立可观测性标注；应拒绝或输出明确等价类，不输出唯一可执行朝向 |
| SC04 双同类实例、交叉/遮挡后交换、错误 mask | R2，身份串换 | 评分器掌握真值身份；歧义拒绝；0 次观测到的错误实例提交 |
| SC05 时钟停滞/回退、RGB-depth错帧、未来帧、TF缺失、标定更新 | R2，时效与变换错误 | 旧快照立即失效，恢复新 epoch 和新采集；覆盖阈值前/等于/后 |
| SC06 注册/跟踪超时、取消与成功同时到达、worker 重启/迟到输出、OOM | R3，旧结果复活 | 无撤销结果进入可用缓存；父节点有界响应；旧 worker 未退出不创建冲突替代实例 |
| SC07 wrist session 撤销、相机运动、超大跨帧位移 | R2/R3 | 不拿最新 TF 补旧帧；追踪不能恢复时总期限内失败；机器人运动阶段单独计数 |
| SC08 计划时目标移动、场景更新、部分读回、ATTACHED 后视觉漂移 | R4，场景和账本分裂 | 旧计划失效；无第二写入者；attached 对象不被感知改写 |
| SC09 FoundationPose + GraspNet + RGB-D + Gazebo + 导航联合负载 | R5，共享 GPU 长尾 | 30 min，ROS/wall/RTF、P50/P95/P99/max、丢帧/拒绝率和显存；超过现有保护预算则不放行 |
| SC10 显式切回 PPF/禁用新后端 | R2/R3/R5，回滚残留 | 关闭并隔离旧会话；backend/epoch 正确；旧 PPF 精度/拒绝/取消语义无退化 |

精度对照沿用旧基线 **平移 ≤20 mm、旋转 ≤10°**，仅是本阶段比较阈值。建议可观测测试集位姿接受且合格比例 ≥95%，并在测试集中无已接受但超阈值结果；这些是待冻结目标，未实际验证。实际抓取还必须根据夹爪开口、接触间隙、TCP 与场景误差预算确定更严格门槛，不能由 20 mm/10° 直接推出可抓取。

非对称物体报告平移误差和 SO(3) 旋转误差；对称物体报告最小等价旋转误差与 ADD-S 或同等对称指标，同时保留功能朝向约束。估计成功率、服务可用率、跟踪有效覆盖率、错误接受率、误拒绝率、重定位时间与抓放成功率分别统计。零样本不算通过，连续相邻视频帧不能冒称独立 100 次试验。

Gazebo 真值和实例标签由独立记录器采集，以准确采集时刻及独立 TF 对齐；真值不进入估计或在线分割。如果用真值 mask 做专门算法上界测试，应明确标为 oracle-only 并从端到端验收剔除。仿真附着若仍为运动学实现，只支持该仿真层结论，不证明摩擦或夹持力。

P5 输出 `R → SC → evidence_path → PASS/FAIL/INVALID/NOT_RUN` 覆盖表，附原始输入、ground truth、实际 frame transform、版本 manifest、后端原始 score（缺失时明确标注）、质量门控理由和耗时。单元、离线、隔离 ROS、联合仿真、真机分别记录。

## 12. 回退与设计边界

新 launch 参数采用 `enable_foundationpose=false` 和明确 `pose_backend=ppf_icp|foundationpose`；新 Action/topic 使用独立名称。质量失败默认返回结构化拒绝，不自动混用另一后端的结果。切换后端必须清理本会话、更新 backend epoch 并重新获取输入；旧 PPF 门槛不降低。

P0 若发现 3.2 无法在当前隔离环境复现、坐标不能明确、持续处理始终不满足已有时效，阶段结论为“后端或实时接入尚未成立”，保留现有能力，改用离线 FoundationPose 评估或重审后端选择。不能用更长的 pose 有效期或静默升级整个 ROS 栈来绕过失败。

首期最小交付是：**真实官方模型 + 一个已知 CAD + 一路已验证 RGB-D + 可取消新鲜位姿获取 + 连续跟踪/失效恢复 + 同输入评测**。只有该门槛通过后，才按 P4 将结果纳入 MTC 仿真抓放链。
