# FoundationPose P0 分步实施计划

> **For agentic workers:** Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Preserve the phase gates.

**Goal:** 为已批准 FoundationPose 方案建立可复现的官方源码/模型、已知 CAD 和历史 RGB-D 基线，待隔离运行环境与 GPU 时段就绪后执行真实推理。

**Architecture:** 项目运行时继续 C++；本阶段只新增准备/验证脚本、资产清单与报告，不修改现有控制链。真实推理前冻结来源、尺度、frame 和资产 hash；准备成功与推理通过分别判定。

**Tech Stack:** Python 3 标准库、NumPy/Pillow（离线准备）、ROS2 Humble / Isaac ROS 3.2 / C++ / TensorRT FP32（待运行阶段）。

**Spec:** ../../FOUNDATIONPOSE_INTEGRATION_DESIGN_20260923.md

## Global Constraints

- 用户已批准按设计逐步实施；继续当前任务，不重复请求相同授权。
- 新业务运行时采用 C++；准备、启动和验证脚本允许 Python。
- 保留 PPF/ICP、GraspNet 和旧消息接口；不降低现有时效/质量/执行门槛。
- 当前并发工作区有大量其他任务改动：仅新增 `tools/vision/foundationpose/`、独立S0准备工具`tools/assembly/`、本任务 evidence 与文档；按最新全窗口规则对本任务已验证阶段及明确标记的候选/失败证据限定路径留Git记录，不提交、重置或覆盖其他人的文件。
- 所有下载、构建、模型和后续 ROS install 使用独立任务目录；不操作共享 ROS install。用户随后授权系统 Docker/NVIDIA Container Toolkit 安装；15:31 导航明确释放、搬运确认没有活动会话后，15:33 实际安装完成。
- 导航/搬运任务已暂停运行并移交维护/GPU资源；后续恢复仍须互相协调。容器基础就绪不开放机器人动作阶段。
- 宿主初查无容器工具链；Docker/NVIDIA Container Toolkit 已安装并通过设备可见性检查，TensorRT/GXF/模型实际推理仍需分项验证。认证内容不写入仓库或证据。
- 离线快照保持原始采集时间、旧标定与 fixture mask 标签；不提供在线执行许可，不将真值输入算法。

## Review Focus

1. 下载失败、截断、来源版本漂移：原子输出、校验哈希、可核对锁定清单；不能把 HTML 错误页当模型。
2. 同名数据重跑：校验原内容，避免静默覆盖既有证据；不同版本输出到不同目录。
3. CAD 米/毫米、mesh 原点和面朝向：验证精确 box union 外表面、拓扑和体积，不把点云改后缀当 mesh。
4. 数据集 RGB/depth/K/mask/TF 混源：逐样本核对 shape/stamp/frame/版本并保存来源，历史 D=0 不自动代表当前流合格。
5. 缺环境或 GPU 排队：输出 BLOCKED/NOT_RUN 与原因，任何 stub/模型解析都不算真实推理 PASS。

## Task 1: CPU 环境与资产准备

**Files:** 新增 `tools/vision/foundationpose/prepare_p0.py`；必要的纯离线验证放入同目录；输出 `docs/evidence/foundationpose_p0_20260923/` 与 `runs/foundationpose_p0_20260923/`。

**Consumes:** 设计中的固定 3.2 官方来源、`astribot_object_pose_core/models/asymmetric_union.visibility.json` 和历史七场景 RGB-D。

**Produces:** `manifest.json`、官方源码 commit、下载文件 SHA256/字节数/许可来源、独立缓存目录、`asymmetric_union.obj` 与模型几何检查、数据集检查报告。

- [x] 创建只执行准备工作的入口，参数显式接受 `--output`、`--cache`、`--dataset`，无 ROS/GPU 初始化副作用。
- [x] 固定 `isaac_ros_common=fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01`、`isaac_ros_pose_estimation=9caca619bcc9d637b3107e17c1a77132c9d7863b`，记录 release-3.2 来源。
- [x] 下载官方 `1.0.0_onnx` refine/score，记录实际哈希，不将自行计算的 hash 冒称发布方签名。
- [x] 从已知三 box union 生成精确边界三角网格、材质及对象坐标变换；保持原几何和米制尺度。
- [x] 校验历史七场景数据，保留原相机版本和 HSV fixture 标识；提供不可变输入清单。
- [x] 实际运行准备入口，检查正常结果与损坏/错 shape/单位等关键拒绝行为。

准备运行契约：

```bash
python3 tools/vision/foundationpose/prepare_p0.py --help
# 输出目录与实际数据路径在现场审查后写入 evidence manifest；不使用浮动默认数据集。
```

## Task 2: 运行条件与后续可复现命令

**Files:** 新增 `tools/vision/foundationpose/README.md`、`docs/FOUNDATIONPOSE_P0_PROGRESS_20260923.md`。

**Consumes:** Task 1 manifest 与环境只读结果、协作资源边界。

**Produces:** 完整事实清单和执行门槛，明确准备阶段已运行、engine/节点推理未运行。

- [x] 在报告中分开记录环境、下载、模型结构检查、CAD 几何、历史数据、engine 和推理状态。
- [x] 固定 FP32 转换参数：refine 输入 1/1/42 ×160×160×6，score 输入 1/1/252 ×160×160×6；只在隔离 GPU 环境满足后执行。
- [x] 下一阶段先官方示例，再项目单帧，最后连续序列；现有七场景单帧不能证明 tracking。
- [x] 若无系统容器基础依赖或资源释放，保留明确 BLOCKED/NOT_RUN，继续独立场景设计与接口准备，不绕过运行门槛。

## Task 3: 本轮设计与协作交付

**Files:** `docs/PEG_IN_HOLE_ASSEMBLY_DESIGN_20260923.md`、`docs/FOUNDATIONPOSE_ASSEMBLY_COORDINATION_20260923.md`、原 FoundationPose 方案的相机入口更正。

**Consumes:** 官方开源任务/CAD 来源、实际夹爪/TCP/物理执行边界、另两任务回报。

**Produces:** 六形状可换槽板设计、抓持段优先版本、渐进间隙与验收矩阵、到位/停稳/新鲜观测/装配执行的接口衔接。

- [x] 参考 ManiSkill AssemblingKits、IndustRealKit、robosuite 与 NIST，逐项区分代码、资产与许可。
- [x] 明确单边间隙、非圆形法向 offset、孔深/倒角/抓持段/夹爪退出空间及对称性。
- [x] 不将导航 3 cm/1.5° 或 FoundationPose 20 mm/10° 的阶段精度直接当作插入公差。
- [x] 记录现有 kinematic 附着无法证明接触插入；毫米级阶段需物理接触/反馈及更严格计量门槛。
- [x] 将分工与 raw 话题/frame 修正回传另两任务；不跨越 I0.2/A4 去执行新运动阶段。

## 执行记录

- 2026-09-23：用户批准设计后开始 P0；新增多形状装配设计与跨窗口协作要求。
- 决策：本轮只使用新增文件命名空间和独立缓存/输出目录，保持各窗口既有源码/分支所有权；若判断错误，代价是后续需要迁移这些新增文件，不会重写他方运行控制代码。
- 初始外部条件为容器工具链缺失、导航独占；这些条件后续已解除，下面继续追加实际证据。P0 不因准备工具通过而提前标记整体完成。

### 本轮实测记录

- 准备工具与9项边界检查完成，主任务独立复跑9/9；CAD为108顶点/212面，独立OBJ解析检查通过；七场景历史输入核对并隔离真值。
- 两份官方ONNX与发布方哈希匹配；common固定归档下载成功。pose完整源码归档两轮120/240秒超时，v1/v2实际CPU准备总状态均为BLOCKED，未发布截断文件；上述实现清单打勾不代表远端资产全部就绪。
- 容器准备脚本普通用户实测成功，签名APT源下固定9个新增包，升级/移除均0；未实际安装、未启动容器服务，等待导航所有者明确释放。
- 槽板设计、PNG/SVG、DESIGN_ONLY参数YAML、协作契约和相机入口修正均完成并回传协作任务。
- P0整体保持BLOCKED；engine/官方示例/项目推理/tracking/机器人执行均NOT_RUN。详见 ../../FOUNDATIONPOSE_P0_PROGRESS_20260923.md。
- 后续固定commit源码checkout成功（181文件，LFS指针保留）；官方FoundationPose单帧所需4个PNG单独按内容hash取得，另复制相机/mesh/材质，总7素材核对通过。v2历史归档下载失败清单保持不变。

## Task 4: 隔离后端与真实离线推理（资源释放后续行）

**Files:** 仅在 `tools/vision/foundationpose/` 增加容器配置与验证脚本；输出独立 runs 子目录并更新 P0 证据页。

- [x] 安装容器基础依赖，核对既有包/驱动未变；官方 CUDA 临时容器设备可见性检查通过。
- [ ] 固定可获取的官方镜像 digest，解析 ROS Humble / Isaac ROS 3.2 依赖，保存实际包清单；不可获取的历史标签作为独立失败记录保留。
- [ ] 核对真实 ONNX 输入/输出，FP32 构建 refine/score，记录环境、日志、哈希、时间与峰值显存。
- [ ] 使用官方单帧做真实离线推理，检查非空、有限值、四元数、输入输出时间和 frame 对应；仅收到消息不能算通过。
- [ ] 只挂载算法输入/CAD/模型，运行项目七个单帧；真值在估计器外独立评估，不改变历史采集时间。
- [ ] 记录失败与限制，退出本任务容器并回传协作状态；连续 tracking 与机器人闭环仍独立验收。

续行环境决策：固定 common 提交计算出的 Humble 镜像标签不存在。另一个可获取官方开发镜像压缩约 22.1 GB，当前先核对固定 NVIDIA TensorRT 24.08 amd64 digest 的精简容器路线；它不是原 common 开发镜像的等价验证，必须记录并实测实际依赖差异。ROS 软件源 HTTP + signed-by 用法来自固定 common Dockerfile；已独立验证 InRelease 的 Open Robotics 签名，不关闭 TLS 或 APT 签名检查。

16:36续行检查点：固定基镜像已取得；实际TensorRT10.3.0.26与目标30不同。两份官方ONNX1.16.0完整CPU语义/输入检查通过。后端镜像/engine/推理未运行，FP-BACKEND-01按新一小时规则deferred并交还资源；不重置历史计时、不继续以环境等待占住整条链。详见P0进展页与backend_checkpoint_1636.json。

## Task 5: 独立槽板S0截面与二维CAD筛查

**前置复核：** 使用已批准的六形状/顶部抓持段候选尺寸；本项只有解析几何与离线CAD轮廓，不需要FoundationPose、ROS、GPU、仿真或实机控制权。输入是DESIGN_ONLY参数，不把尚未观察的板位姿当运行事实。

**边界：** 检查直壁截面、标称法向间隙、板/模块容纳和中心对齐的离散yaw错槽可容纳矩阵。正例给出具体yaw和净空证明；未找到不能证明所有平移/旋转都无法放入。不包含倒角、三维抓持柱、夹爪扫掠、可达或接触。

- [x] 明确D孔的候选解析几何：半径R+c圆盘与x<=a+c半平面的交集，边界交点为尖角；加工/入口倒角仍未设计，不使用示意采样多边形替代圆弧。
- [x] 导出原始参数化二维DXF轮廓与机器可读报告，单位显式，输出版本独立。
- [x] 验证六种形状名义朝向在三级间隙下均满足尺寸；提供可复核错槽正例，明确业务实例到槽位强绑定仍需运行时实现。
- [x] 用解析检查和独立DXF读回核对输出；报告只限S0截面，不提前验收精密三维CAD或插入场景。

S0子项实测：7项边界检查通过、18组匹配间隙通过；108组合筛查产生10组错槽正例（6种不同形状对）。DXF首版缺少实体子类标记被独立reader拒绝，失败证据保留；v2共27文件由ezdxf1.4.3读回并独立计算，错误/修复0。三维CAD内核尚未配置，后续依赖准备与重型扫掠须等共享窗口允许，不能把本子项写成完整S0验收。
