# RGB-D 异步处理与 CUDA 可选后端（2026-09-22）

本次落实用户提出的“短回调 → 有界配对 → 最新有效帧 → CPU/CUDA → 发布前时效检查”。运行时使用 C++，Python 仅用于启动、采集、分析和验证。本文件区分实现、单元验证与仿真验收；W1 尚未放行。

## 数据流与执行边界

```text
Depth Image + CameraInfo
  → 接收回调：时间戳、尺寸上限、引用保存（不做像素计算）
  → 精确时间戳配对：各最多 4 帧；匹配 frame、尺寸与内参
  → 1 个待处理槽：新帧替换旧待处理帧；另有最多 1 个正在执行的任务
  → 独立 C++ 工作线程：CPU 或 CUDA 深度反投影
  → 核验 ROS 时间龄期、steady 接收龄期、时钟代际、标定上下文、相机会话
  → 发布原始采集时间戳的 PointCloud2
```

当前改造范围是四路 RGB-D 的 **深度到点云节点**，配对输入是 Depth + CameraInfo。RGB + Depth + 检测框的语义配对仍由现有识别/姿态链路负责，不能把此次改造说成整个多模态处理栈已完成迁移。

- 深度与 CameraInfo 精确时间戳相同；不存在“随便取最近一张”混帧。持续缺帧时不构造虚假的配对。
- 单源缓存最多 4 个引用，待处理槽 1 个；DDS 输入 KeepLast(4)，输出 KeepLast(1)。允许的单幅输入上限为 4,194,304 像素、33,554,432 字节。上限约束节点保留的数据，不能阻止 DDS 在回调前反序列化超大报文。
- 计算开始及结束均校验时效，默认 250 ms，同时使用 ROS 时间和 steady 时间；冻结仿真时钟不能延长帧有效期。
- 计算中到达的新帧覆盖待处理槽，不直接作废仍在时效内的在途帧，避免高输入频率下永远不发布。
- 时间回退、内参/尺寸/frame 变化、相机会话 token/owner/execution 变化会使旧任务失效。腕部会话撤销或到期后，旧结果不能提交。
- 发布与上下文失效共享短锁，像素循环和 GPU 等待不持锁。DDS 发布自身仍可能有调度延迟，因此这不是硬实时或严格无锁承诺。
- 点云保留采集 stamp 与 optical frame；不重新盖“当前时间”伪装新鲜，不放宽导航/感知安全阈值。
- 本模块不产生导航放行、机械臂动作或抓取执行指令；相机会话、世界快照、任务控制权仍由各自现有模块负责。

## 后端、构建与部署

`projection_backend:=cpu|cuda`，默认 CPU；构建开关 `ASTRIBOT_ENABLE_CUDA_PROJECTION` 默认 OFF。请求未构建的 CUDA 后端时明确启动失败，不默默降级。CUDA 通过 Driver API + NVRTC C API 实现，独立于 LibTorch 的 C++ ABI，启动时编译一次内核，工作线程复用 stream/显存缓冲。

本机可用的 CUDA 12.4 组件位于 `/home/yjh/.cache/astribot/graspnet/torch-2.5.1-cu124/nvidia`。示例只安装到独立前缀，禁止把独立 CMake 默认前缀误装到系统：

```bash
source ws_robot/install/setup.bash
cmake -S ws_robot/src/astribot_s1_perception_components -B runs/rgbd_cuda_build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX="$PWD/runs/rgbd_cuda_install/astribot_s1_perception_components" \
  -DASTRIBOT_ENABLE_CUDA_PROJECTION=ON \
  -DASTRIBOT_CUDA_ROOT=/home/yjh/.cache/astribot/graspnet/torch-2.5.1-cu124/nvidia
cmake --build runs/rgbd_cuda_build --target rgbd_pointcloud_node depth_projection_test rgbd_pointcloud_test -j2
cmake --install runs/rgbd_cuda_build --component rgbd_projection
```

该组件安装仅包含节点；独立前缀运行仍需完整包索引、launch 和其余依赖 overlay。本轮在已冻结的完整包副本中替换节点和两份 launch，未覆盖共享 `ws_robot/install`。安装节点带 NVRTC 库所在目录的 RUNPATH；换机部署需重新构建/配置依赖路径。

接口支持 16UC1/mono16 毫米与 32FC1 米、大小端、行 padding、抽稀、深度范围过滤。无效像素输出 NaN。CPU/CUDA 保持相同几何与输出格式，不能通过减少安全覆盖来换性能。

CUDA 分配失败与执行报错会丢弃该结果并记录错误；扩容失败后清空容量记录，使后续重新分配可恢复。**驱动硬挂起并没有进程内有界取消保证**：GPU 同步调用及析构等待可能阻塞，进程级 watchdog/隔离与 GPU 故障恢复矩阵仍是未完成项，不能宣布已验收。

## GPU 使用策略与实测

RTX 4090，100 次测量，预热 10 次，数值为中位数；CUDA 包含主机→设备、内核、设备→主机，未包含 DDS、配对等待和首次 NVRTC 编译。

| 图像/抽稀 | CPU / ms | CUDA / ms | 本轮判断 |
|---|---:|---:|---|
| 320×180 / 1 | 0.235 | 0.273 | CPU 更合适 |
| 640×360 / 1 | 0.948 | 0.774 | 可选择 CUDA，仍需端到端比较 |
| 1280×720 / 1 | 3.771 | 2.589 | CUDA 有收益，约 1.46 倍 |
| 640×360 / 4 | 0.050 | 0.187 | CPU 更合适 |
| 1280×720 / 4 | 0.331 | 0.644 | CPU 更合适 |

证据：`runs/task_chain_20260922_afternoon/projection_benchmark.jsonl`；可重复测量程序 `tools/vision/benchmark_depth_projection.cpp`。这不是整条导航或仿真的加速比。

| 模块 | 当前状态 | 后续准入原则 |
|---|---|---|
| 深度反投影 | 本轮新增 CPU/CUDA 可选 C++ 后端 | 密集图像考虑 CUDA，稀疏小图 CPU；不自动切换 |
| GraspNet | 既有 C++ LibTorch CUDA worker；本轮并发压力运行 | 当前输入为冻结历史点云，仅验证负载和执行，不代表实时抓取通过 |
| Gazebo 相机渲染 | 已使用 NVIDIA/OpenGL；本轮增加可选 Ogre 工作线程数补丁 | 原有默认值不改；4/1 线程候选仍未通过持续新鲜度验收 |
| YOLO | 当前系统 OpenCV DNN CPU | 后续独立引入 CUDA-enabled OpenCV 或 TensorRT/ONNX GPU 后端，需模型/解码一致性与端到端时效对照 |
| 点云过滤、体素化、物体姿态求解 | 后续性能热点候选 | 先量测，再整段驻留 GPU，避免每算子往返拷贝 |
| 包络准入、租约/心跳、安全保护、Nav2 控制 | 保留现有 C++ CPU 路径 | 不能仅凭“可并行”移到与感知共享 GPU 的关键安全路径 |

减少主机/设备数据搬运的原则参考 NVIDIA [CUDA Best Practices](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html#data-transfer-between-host-and-device)；运行时编译接口参考 [NVRTC](https://docs.nvidia.com/cuda/nvrtc/index.html)。

## 验证与未放行项

最终测试计数和仿真观测见 `docs/evidence/task_chain_20260922_afternoon/async_validation.json`。仿真运行日志：`runs/task_chain_20260922_afternoon/navigation_09_async/session.log`；进程、配置和二进制证据：`async_runtime_manifest.json`。

600 秒相机会话 + CPU 头/腹点云 + CUDA 双腕点云 + CUDA GraspNet 压力 + MPPI 发起导航。导航在首个到位过程中因 `WAITING_FOR:controller,local_costmap` 撤销并取消，不能算 600 秒连续运动或完整通道成功。持续新鲜度已有超限观测，不放行 W1。单帧投影只有毫秒级，不能据此认定它是全部 250 ms 断流的根因。

未完成：消费者 ACK 丢失原因与恢复、250 ms 接收间隔长尾、30 分钟联合压力、各相机任务 ROI/有效深度/遮挡与三维覆盖、GPU 硬挂起/进程恢复、后续语义处理线程迁移。真机触发同步、物理标定、VLA 和完整抓取搬运放置仍不在本轮验收范围。
