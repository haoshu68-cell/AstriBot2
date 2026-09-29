# 仿真 GPU 渲染与工作线程候选

本轮目标是恢复 RGB-D 在真实墙钟下的持续新鲜度。250 ms 门槛、物理步长、机器人碰撞几何和摄像头视场保持不变。本候选尚未构成 W1 整体验收。

## 当前实现与定位证据

- 本机 RTX 4090 已用于 Gazebo 的 Ogre2/OpenGL 渲染。`nvidia-smi` 中的 Gazebo 图形进程和进程加载的 NVIDIA 库可作为运行证据；“compute-apps 为空”不能推断图形渲染未使用 GPU。
- 原版 Fortress 的 Ogre2 场景按逻辑 CPU 数创建工作线程，本机检测到 28 个。30 秒诊断采样中，Ogre 工作线程调用栈包含 45% 的样本，futex_wait 独占样本占 25.2%。这些是采样比例，不是端到端延迟百分比。
- 4 线程诊断的 Ogre 工作线程包含样本比例为 13.9%，futex_wait 为 4%。640 宽图像的首轮 120 秒诊断没有 STALE，但正式候选后续仍出现约 257 ms 的间隔，不能据此宣布问题已修复。同机其他仿真负载有变化，暂不能给出受控加速倍数。
- 运行中的 GraspNet C++ 入口支持 CUDA；压力回归使用冻结点云，输出无执行权限。YOLO 的现有 C++ OpenCV DNN 路径明确选择 CPU，本机系统 OpenCV 也未提供 CUDA 模块。后续迁移应单独选择并验证 GPU 推理后端，不能只把设备参数改成 CUDA。

## C++ 候选与配置边界

补丁 `tools/sim/patches/ignition-rendering6_6.6.4-worker-count.patch` 只增加渲染工作线程配置：

- `ASTRIBOT_OGRE2_WORKER_THREADS` 未设置：保留上游检测行为。
- 显式设置：只接受整数 1～64，空值、负数、非数字和越界值拒绝启动。
- 不修改图像尺寸、曝光/采样时间戳、渲染算法、激光扫描参数、物理更新周期或安全阈值。
- 源码固定为 `ignition-rendering6_6.6.4`，下载包 SHA-256 固定并在解包前核验。
- 候选安装到调用者提供的新目录，不覆盖系统、共享 ROS 安装或其他仿真实例。

准备候选：

```bash
python3 tools/sim/build_rendering_candidate.py \
  --output /absolute/new/isolated/render_candidate --jobs 2
```

可用 `--archive /absolute/cached/upstream.tar.gz` 使用缓存；同样必须通过固定校验值。编译前执行实际补丁中的 C++ 参数检查测试，编译后检查动态库依赖。构建和回归不要同时进行，以免编译负载混入性能对照。

仅在明确归属的仿真启动环境中启用：

```bash
source /absolute/new/isolated/render_candidate/env.sh
export ASTRIBOT_OGRE2_WORKER_THREADS=4
# 随后按该实例的隔离启动流程运行；线程值只是候选，并非已放行默认值。
```

Fortress 的系统插件查找会先加入编译时路径，再追加环境路径。因此候选同时安装同版本 rendering core 和 Ogre2，并通过局部 `LD_LIBRARY_PATH` 选择它们。只设置 `IGN_RENDERING_PLUGIN_PATH` 在本机实际仍加载了系统插件。必须核对 Gazebo `/proc/<pid>/maps` 中两个库均来自本候选目录；环境变量存在、进程存活、导航 ready 都不能替代此项检查。正式候选不使用诊断注入库或性能采样库。

回退：停止本任务拥有的实例，使用未加载候选 `env.sh` 的新环境重启。不能在仍运行的进程上替换库，也不能通过全局清理其他 ROS/Gazebo 会话回退。

## 数据流与验收口径

```text
Gazebo 物理步进 → Ogre2 CPU 场景更新/剔除 → NVIDIA GPU 渲染
    → 原始 RGB / Depth / CameraInfo → bridge
    → C++ 配对、相机租约、健康检查 → 点云 / 导航消费者
```

GPU 渲染并不消除 CPU 场景更新、线程同步、数据回传或 DDS 排队。任何 GPU 优化都需同时观察真实接收间隔、ROS 时间龄期、实际仿真推进率及控制指标。

每次记录相机内外参、分辨率、渲染库哈希、线程数、同机负载和时间窗口。各阶段分别确认：

1. 静止：四路原始流、精确时间戳配对、健康状态、真实接收间隔、租约获取/续租/释放/超时。
2. 实际导航：保留固定姿态、完整碰撞包络和通道规则，记录 FOLLOW 段与到位段，不将静止 MPPI 节点当作运动负载。
3. CUDA 压力：C++ GraspNet 对冻结点云真实推理，与导航/相机的重叠区间单独记录；不称为实时抓取准确率验证。
4. 故障与重启：缺帧、TF、时钟、版本、GPU 故障需有明确失效与恢复证据，不能由正常运行结果推定。
5. 30 分钟回归：逐阶段记录负载和失败原因。成功完成采集不等于持续健康通过；任何 STALE 均保留，不扩大 250 ms 门槛。

验证采集器会先保存时序和健康证据，再尝试图像/TF 导出。本轮通过实际相机数据加观察端 TF 导出异常，验证失败时 `stream_timing.json` 仍保留；该用例不等同于运行时 TF 故障恢复验收。

低分辨率 320 宽只用于瓶颈诊断；本轮恢复的是原导航仿真的 640 宽档，不是相机固件标定的 1280 宽档。场景 ROI、遮挡、深度覆盖和下游识别精度仍需分别验收。

上游依据：[Fortress Ogre2Scene 源码](https://github.com/gazebosim/gz-rendering/blob/ignition-rendering6_6.6.4/ogre2/src/Ogre2Scene.cc)、[CPU profiler](https://gperftools.github.io/gperftools/cpuprofile.html)。运行证据见 `docs/evidence/task_chain_20260922_afternoon/`，与历史记录分开保存。
