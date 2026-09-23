# FoundationPose P0 实施进展与接续条件

2026-09-23，任务 `01a0ccca-7607-7bc0-b7d9-12dc8c8e6eda`。本页记录本轮新增实现和实际证据，不代替 [整合设计](FOUNDATIONPOSE_INTEGRATION_DESIGN_20260923.md) 的阶段验收。

## 结论

已实现 CPU 离线准备入口与拒绝边界检查；本轮新增多形状槽板设计和跨任务协作契约。Docker/NVIDIA Container Toolkit 安装、容器显卡访问、固定 TensorRT 基镜像获取、两份官方 ONNX 完整语义和输入合同检查已通过。**P0 整体尚未通过：Isaac ROS 后端、FP32 engine、官方示例、项目真实推理与 tracking 尚未验收。** 后端依赖问题 `FP-BACKEND-01` 按最新一小时规则在本轮检查点暂缓，转独立槽板 S0；基础容器、模型格式和网格生成通过不能代替模型推理通过。

当前新增文件分工：

- [离线准备入口](../tools/vision/foundationpose/prepare_p0.py)：固定官方源码/模型来源、下载校验、CAD 边界网格、历史数据输入分离。
- [边界检查](../tools/vision/foundationpose/test_prepare_p0.py)：主任务独立复跑 9 项，通过；包括错误单位、非流形接触、下载错误/超时、缓存篡改、不可变输出、错误 mask 形状与真值隔离。
- [槽板设计](PEG_IN_HOLE_ASSEMBLY_DESIGN_20260923.md)、[候选参数](assets/assembly_20260923/scene_spec_v1.yaml) 与 [示意图](assets/assembly_20260923/assembly_candidate_top_grasp.png)：设计材料，尚非可运行物理场景。
- [协作契约](FOUNDATIONPOSE_ASSEMBLY_COORDINATION_20260923.md)：与“完善path_tracking到位精度”“设计机器人搬运仿真流程”确认任务所有权、停稳/观察/资源交接与当前相机入口。

## 已取得的离线证据

| 项目 | 本轮实际结果 | 证据与限制 |
|---|---|---|
| 边界检查 | 9/9 通过，主任务独立复跑 | [检查日志](evidence/foundationpose_p0_20260923/boundary_tests.log)；只验证准备工具 |
| 官方 refine ONNX | 68,170,161 字节，与发布方 SHA256 一致 | `06ad19f2c3598cb76733feec084d3f6802e7ff143882ec42ba368df7e38ae094` |
| 官方 score ONNX | 63,910,294 字节，与发布方 SHA256 一致 | `49a4f5f094358913670733ec31e856b96271c869f9949aa3a0361cf7cf8f0be8` |
| CAD | 108 顶点、212 三角面；闭合、朝向与体积检查通过 | [独立 OBJ 检查](evidence/foundationpose_p0_20260923/cad_output_independent_check.json)；体积 0.001749 m³、面积 0.10315 m²，后端坐标尚未验证 |
| 历史 RGB-D | 七个静态单帧通过内部输入检查 | HSV fixture，不是通用分割或当前传感器验收 |
| 完整 pose 源码归档 | `prepared_v1` 120 秒、`prepared_v2` 240 秒均超时，未发布截断归档 | [首轮清单](evidence/foundationpose_p0_20260923/preparation_v1_manifest.json)与[第二轮实际清单](../runs/foundationpose_p0_20260923/prepared_v2/manifest.json)；两轮 CPU 准备总状态均为 BLOCKED，不伪记整体成功 |
| 固定提交源码 checkout | 后续改用 Git 获取成功：181 个跟踪文件，共 2,783,038 字节 | [源码清单](evidence/foundationpose_p0_20260923/source_checkout.json)；完整 Git 源文件可用，12 个 LFS 文件仍为指针，不等于已取得其测试载荷或完整展开归档 |
| 官方单帧样例素材 | 单独取得四个 PNG 和相机/OBJ/MTL，共七个素材文件；哈希和格式检查通过 | [样例清单](evidence/foundationpose_p0_20260923/official_sample.json)；未运行上游测试或模型，未改变源码 checkout |
| 容器基础安装 | 15:33实际完成；新增 9 包、升级 0、移除 0 | [安装证据](evidence/foundationpose_p0_20260923/container_installation.json)；驱动和既有包版本未变 |
| 容器显卡访问 | PASS；无网络、只读临时容器内读取同一 RTX 4090，退出删除 | 验证设备/驱动可见性，不是 CUDA 算法、性能或 FoundationPose 推理验收 |
| ONNX 语义和输入合同 | 两份均 PASS（16:34） | ONNX 1.16.0 full_check；不是模型推理 |
| engine、推理、tracking | NOT_RUN | 后端版本对齐尚未完成 |

模型存于独立本机缓存 `/home/yjh/.cache/astribot/foundationpose/p0_20260923/models/`，未加入仓库；源码与模型许可分别记录。`isaac_ros_common` 固定 commit 归档已完整保存。pose estimation 后续禁用 hooks/LFS smudge 获取固定提交源码，HEAD 为 `9caca619bcc9d637b3107e17c1a77132c9d7863b`，tree 为 `46c29b8943e753ad0f12bb5235ced0902a0ee763`，未执行仓库代码。

[LFS 指针审计](evidence/foundationpose_p0_20260923/pose_lfs_pointer_audit.json)显示五份不同测试 ONNX 对应载荷合计 613,587,602 字节，另有重复路径。它解释了归档可能因 LFS 展开而膨胀；两次失败传输没有给出完整归档大小。新的源码获取成功作为独立证据追加，不修改旧 `prepared_v2` 的失败清单。官方 FoundationPose 单帧所需四个图像/纹理载荷随后按 LFS 内容哈希单独下载到缓存 `official_sample/`；未下载其他任务或 dummy 测试权重。

该官方样例为 640×480、原始 RGB PNG 为 RGBA；上游测试默认 OpenCV 读取后转为 RGB，后续适配必须显式处理 alpha/通道顺序。深度为 uint16 毫米，上游测试转 float32 并除以 1000 后发布 `32FC1` 米；本项目历史 fixture 已是 float32 米，不能重复换算。官方 mask 的 3252 个像素中有 605 个零深度，原样保留，未填洞或修改 mask；不能直接套用本项目历史 fixture 的“mask 内全部有有效深度”输入假设。

## 运行环境与 Docker 的作用

初查宿主 Ubuntu 22.04.5 / ROS2 Humble、RTX 4090，驱动 580.178.04。初查 Docker、NVIDIA Container Toolkit、TensorRT/GXF 均未就绪；15:33已补齐前两项，未修改显卡驱动、系统 ROS 或共享 install。

Docker 不是 FoundationPose 算法本身的硬性条件。Isaac ROS 3.2 官方也允许在宿主配置 Humble 与依赖，但推荐其开发容器，示例按该容器组织。本项目选择容器，是为了固定 Isaac ROS 3.2 / CUDA / TensorRT / GXF 依赖，并与现有 GraspNet、ROS2 工作区隔离；容器只承担感知后端，主控制栈继续由现有流程运行。[官方 3.2 环境说明](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html)

NVIDIA Container Toolkit 负责把宿主显卡设备及驱动相关能力提供给容器；安装它不等于安装/验证 FoundationPose，也不需要本轮升级宿主显卡驱动。[NVIDIA 安装说明](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

用户已明确授权安装 Docker Engine 与 NVIDIA Container Toolkit。按导航任务要求，I0.2 性能回归期间仅做预演；15:31收到明确资源释放通知、搬运任务确认没有其他会话后，才执行实际安装。Docker 包启动了服务并创建默认网络，原配置不存在的事实和安装前后接口/路由均有记录。[Docker Ubuntu 安装说明](https://docs.docker.com/engine/install/ubuntu/)

[安装脚本](../tools/vision/foundationpose/install_container_prerequisites.sh)和[说明](../tools/vision/foundationpose/INSTALL_CONTAINER.md)已写入；主任务已审查并独立核对安装前后日志。实际安装 Docker Engine/CLI 29.8.1、containerd 2.3.5、Buildx 0.37.1、Compose 5.5.1、NVIDIA Toolkit 四包 1.20.1-1。脚本隔离宿主 APT hooks，安装前再次核对包清单，拒绝升级/移除或改动驱动/CUDA/ROS 包，不添加 docker 组权限。安装脚本本身不运行容器；后续独立验证使用 NVIDIA 官方 CUDA 12.6.1 base 镜像，固定 digest 为 `sha256:4989b8d89253f415f4273e4d4a1748ad0e82b172a8675ed7bcab3a9f7157cf66`，容器无网络、只读、drop ALL capabilities。`nvidia-smi` 返回与宿主一致的 GPU UUID 和驱动。

首次 Docker Hub 镜像拉取遇到域名解析目标与 TLS 证书不匹配，未关闭 TLS 校验；切换 NVIDIA 官方 `nvcr.io` 后成功。该网络故障与容器显卡访问结果分别记录。

### 后端镜像与续行状态

固定 common 提交的构建规则计算出 `x86_64-ros2_humble_ed892dec4a2599932448db8534fc4cf2`，实际 manifest 查询失败；[NVIDIA 官方论坛](https://forums.developer.nvidia.com/t/is-image-tag-of-isaac-ros-dev-base-for-x86-64-in-v3-2-14-correct/353379)也有对应标签缺失记录。可取得的另一个官方开发镜像有92层、压缩约22.1 GB，本轮未拉取。

当前准备最小候选镜像 [Dockerfile.p0](../tools/vision/foundationpose/Dockerfile.p0)，固定 NVIDIA TensorRT 24.08 amd64 digest 为 `sha256:5ee1d4376a8b73c6a7f756e684fff4f2a06883ae3680de5d8fafc8c96a8e42ef`（36层、压缩约5.49 GB）。这是不同的实际依赖组合，尚未验证，不冒称原开发镜像已运行。候选配置锁定 TensorRT 10.3.0.30 及 FoundationPose/GXF FoundationPose 3.2.14；官方 release-3 仓库的其他依赖合法混用3.2.5/3.2.10等补丁版本，实际解析清单仍待构建记录。

ROS HTTPS 入口也出现证书域名不匹配；固定 common Dockerfile 使用 `http://packages.ros.org` 配合签名密钥。本轮按该路径取得 InRelease 并用单独通过 HTTPS 获取的 ROS key 验证，Open Robotics 签名通过。APT 签名与 HTTPS 校验均未关闭，没有修改宿主 DNS 或使用第三方不明镜像。

15:57导航任务再次取得独占回归窗口，要求暂停大下载；本任务立即中止 TensorRT 拉取（exit130），缓存层保留。**镜像仍未完整取得，候选容器未构建，engine 和后端推理仍 NOT_RUN。** 等明确清理交还后续传，不因预计16:25到点自动开始。

等待期间新增 [引擎构建脚本](../tools/vision/foundationpose/build_engines.sh)、只启动 C++ 后端的 [离线 launch](../tools/vision/foundationpose/p0_fixture.launch.py) 与 [回放检查器](../tools/vision/foundationpose/validate_fixture.py)。检查器六项输出边界测试通过，八个真实素材输入的 CPU 解析检查通过，见 [输入证据](evidence/foundationpose_p0_20260923/replay_input_checks.json)。这些只证明脚本边界和输入转换；尚未启动 ROS 后端或实际估计物体位姿。

16:03全部CPU检查合并复跑15/15通过；16:04保存 [续行准备收据](evidence/foundationpose_p0_20260923/runtime_preparation_1604.json) 和最小镜像构建上下文，仍未执行构建。管理员交互会话的认证缓存已主动失效并关闭终端；没有留后台下载、引擎生成或ROS容器。

### 16:36 检查点：基镜像和 ONNX 通过，后端依赖暂缓

16:08导航明确清理交还后恢复下载。第二次拉取在大层完成、解包期间达到20分钟上限（exit124）；第三次只做有界缓存续行，exit0，固定 digest 已完整取得。三份日志分别保留，不能把中断/超时日志改成成功。实际镜像约14.64 GB，Ubuntu22.04.4、CUDA12.6、TensorRT **10.3.0.26**；它与 fixed common 的10.3.0.30不同，[NVIDIA发布说明](https://docs.nvidia.com/deeplearning/frameworks/container-release-notes/index.html)也列明该基版本。不能先在26构建engine再未经验证拿到30运行。

官方 FoundationPose ELF 只读检查确认直接依赖 `libcvcuda.so.0` 和 `libnvcv_types.so.0`。已按 fixed common 取得 CV-CUDA v0.5.0-beta 官方包并保存来源/本地SHA256；GitHub未提供该旧资产的发布方digest，不能冒称签名认证。镜像配置v3增加独立签名CUDA源，并将完整TensorRT库/工具族对齐30；APT解析、Python绑定残留与实际动态库路径、GXF闭合依赖仍待验证。**候选Dockerfile还未构建。**

独立CPU模型检查使用临时容器、无GPU设备，安装固定ONNX1.16.0/protobuf4.25.3到临时目录。v1因Docker tmpfs默认noexec不能加载检查器扩展而失败；实际挂载标志查明后，v2仅为该临时工具目录启用exec，其余只读/非特权限制保留，退出0。两份模型full_check和输入合同均PASS：输入分别为float32 `input1/input2=[batch_size,160,160,6]`；refine为IR8/opset17、输出各`[batch_size,3]`，score为IR7/opset14、输出`[1,batch_size]`。见 `runs/foundationpose_p0_20260923/onnx_check_v2/base_and_models.json`，无任何真实姿态输出。

[检查点收据](evidence/foundationpose_p0_20260923/backend_checkpoint_1636.json)记录稳定问题ID、最早现存失败时间15:38:27、下载/资源等待、尝试结果、剩余假设和恢复入口。实际最初启动及累计主动排查时间没有完整单列，保持未知而非零；接近一小时检查点时不再开启无法在余下窗口内完成的环境构建。状态为deferred，engine/推理被其阻塞，15项CPU工具检查与两份ONNX通过不变。

16:36按唯一名称前缀读取本任务容器为空，GPU没有计算进程；管理员认证失效并关闭终端，向导航明确交还所有下载/仿真/GPU资源。下一独立项为S0解析截面、目标间隙、错槽可容纳筛查与二维CAD轮廓；不依赖该后端，不宣称精密三维CAD、完整夹爪扫掠、可达或接触插入已经验收。后续难点轮次先审查v3签名APT解析和真实加载版本，再申请GPU生成引擎。

## 数据与几何边界

历史七场景为 `clear / close / far / near / occluded / tilted / yawed`，来源为 `docs/evidence/grasp_pose_sim_20260921/simulation/snapshots/`。每场景只有单帧 640×360 RGB-D 与 HSV 洋红 fixture mask，不能据此声称通用分割或连续跟踪已验证。

历史相机 frame 为 `astribot_s1/astribot_torso_base/torso_rgbd_sensor`，标定版本 1。当前 raw 入口记录的 frame 为 `astribot_s1/astribot_torso_link_4/torso_rgbd_sensor`；两者不能互换。本轮检查历史数据内部 stamp/K/D/P/TF 一致性，不证明当前相机光学轴或 RGB-depth 配准合格。

准备输出分为 `algorithm_inputs/` 与 `evaluation_only/`。未来 estimator 只挂载算法输入和 CAD，不能挂载评估目录或包含评估信息的 manifest；目录分开本身不构成操作系统权限隔离。后采样真值只用于已有静态目标评估，不可充当动态图像时刻真值。

CAD 来自现有三 box union，保持米制单位与原对象业务坐标；同时输出居中 mesh 和显式坐标变换。纹理为合成 fixture 洋红色，未做真实物体外观建模。网格几何合格不证明后端输出 frame 已正确。

## 下一步执行顺序

1. 已完成：所有者明确释放后安装官方签名包，保存安装前后版本与配置。
2. 已完成：容器服务、NVIDIA runtime注册和显卡访问检查；`nvidia-smi` 成功不能当作 FoundationPose 通过。
3. 解析固定 3.2 依赖与容器 digest，隔离构建 FP32 refine/score engine，保存构建命令、模型 hash、GPU/驱动/CUDA/TensorRT 版本和峰值显存。
4. 先运行官方示例，核对真实输出；再运行已知 CAD 的项目单帧，核对尺度、坐标、质量拒绝和延迟；最后采集独立连续序列验证跟踪。
5. P0 结果决定时效预算和后端可行性。只有感知阶段门槛成立且导航/搬运前置满足后，才进入动作接线；不因本轮完成离线准备而开放装配运动。

首期 FP32 构建候选沿用 3.2 文档：refine 的两个输入各为 `min=1×160×160×6 / opt=1×160×160×6 / max=42×160×160×6`；score 的两个输入各为 `min=1×160×160×6 / opt=1×160×160×6 / max=252×160×160×6`。必须在实际 ONNX 输入名、TensorRT 版本及官方参数复核后执行，不能把这段参数说明当作已经生成的 engine。

槽板后续依次验收 CAD/可达与夹爪退出空间、视觉相对定位、轨迹执行、理想刚性夹持接触、真实摩擦夹持接触；齐平嵌入另设计夹爪避让槽或可移除抓持件。
