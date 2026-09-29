# 四路深度图 484–582 ms 接收间隔分析

分析对象为 `runs/task_chain_20260922_afternoon/camera_09_async_cuda` 与对应的 `navigation_09_async` 历史仿真。此次只读分析既有数据、运行参数与代码，没有重新启动仿真、修改运行时或执行真机测试。

## 结论与证据边界

可定位到**原始深度图到达观察回调之前的共享路径出现现实时间长尾**；现有证据更支持仿真推进/渲染产出与共享桥接调度问题，而非深度到点云的像素计算积压。进一步核查发现，**本轮与其他任务的 Gazebo 仿真实际并发，部分时段同机三套仿真；不能把本轮当作独占主机性能基线**。排除跨任务资源竞争应优先于继续修改线程或 CUDA。尚不能把长尾唯一归因于多实例、Ogre、GPU、DDS 或某一把锁，需要独占对照及边界单调时钟记录。

这轮已使用 C++ 异步点云工作线程，头/腹 CPU、双腕 CUDA、全部 decimation=4。`use_camera_postprocess:=false`，因此回调内 `cv::remap` 未参与本轮。此前“点云全部同步 CPU”的描述不适用于这轮运行。

## 实测数据

测量点为 `/camera/raw/<camera>/depth_image` 的观察工具订阅回调，单位为现实单调时钟间隔。它不是单帧端到端传输延迟。

| 相机 | 帧数 | P50 / ms | P99 / ms | 最大 / ms | >250 ms 间隔数 |
|---|---:|---:|---:|---:|---:|
| 头部 | 6506 | 87.490 | 171.437 | 582.427 | 5 |
| 腹部 | 6506 | 87.464 | 171.995 | 483.653 | 2 |
| 左腕 | 6506 | 87.325 | 170.226 | 517.797 | 3 |
| 右腕 | 6506 | 87.370 | 170.285 | 570.880 | 3 |

每路有 6505 个源时间戳间隔，全部为 0.050 s，无非正间隔；本窗口原始深度观察序列没有显示跳帧或时间回退。该事实不证明每个下游节点均未丢帧。

记录覆盖 600.000306 s 现实时间，ROS 时钟推进 325.278 s，窗口平均 RTF=0.54213；实际平均接收约 10.84 Hz。20 Hz 仿真时间采样在此推进率下，平均现实周期约 92.2 ms。平均减速解释正常周期拉长，不能单独解释约半秒的极值，极值还需要短时停顿或交付抖动。

原始深度观察 ROS 年龄最大：头部 4 ms，其余 1 ms。但图像与 `/clock` 共用桥接路径，ROS 年龄不能当现实时间传输延迟，不能据此排除桥接阻塞。

来源：

- [原始流聚合统计](../runs/task_chain_20260922_afternoon/camera_09_async_cuda/capture/stream_timing.json)
- [采集摘要及相机规格](../runs/task_chain_20260922_afternoon/camera_09_async_cuda/capture/summary.json)
- [启动参数](../runs/task_chain_20260922_afternoon/navigation_09_async/session.json)
- [实际点云节点二进制及参数](../runs/task_chain_20260922_afternoon/async_runtime_manifest.json)

## 原因分析

### 0. 已确认同机多实例并发，且与共同 STALE 窗口重叠

四相机采集窗口为北京时间 14:43:17.982–14:53:17.982，所属 Domain 89 仿真在 14:42:15–14:53:38 运行。另一个工作目录 `/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/` 中保存了 Domain 71/72 的独立仿真记录。逐会话 `session.json` 和 `session.log` 中的 Gazebo 启动 PID 相互印证，不是把同一个 Gazebo 的父子启动器重复计数。

| 本轮共同 STALE 时间（北京时间，近似） | 同时存在的其他仿真 | 包含本轮的实例数 |
|---|---|---:|
| 14:43:34–35 | Domain 71 `v12_on_-90_2`，会话 14:42:52–14:44:28，Gazebo PID 3658219 | 至少 2 |
| 14:50:10–11 | Domain 71 `v13_in_place_-90`，14:49:49–14:51:01，PID 3826362；Domain 72 `v13_pose_jump_CORNER_RECOVERY`，14:50:05–14:51:05，PID 3835291 | 至少 3 |
| 14:51:27–28 | Domain 71 `v13_straight`，14:51:01–14:52:20，PID 3866969；Domain 72 `v13_cross_CORNER_APPROACH`，14:51:23–14:54:18，PID 3876181 | 至少 3 |

这些实例启用了相机、点云、雷达与导航相关工作负载。Domain 和 Gazebo partition 隔离通信，不隔离 CPU、GPU、内存带宽和驱动资源。多实例是已确认的性能实验干扰因素，并且很可能贡献仿真减速与长尾；尚无独占/并发 A/B 能量化其贡献或证明它是唯一原因。原始图像峰值的事件级时间未保存，上表使用逐条健康状态的共同过期窗口，不声称已精确对齐每个图像最大值。

历史证据示例：

- [第一段并发会话](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/v12_on_-90_2/stack/session.json)
- [Domain 71 并发运行日志](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/v13_in_place_-90/stack/session.log)
- [Domain 72 并发运行日志](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C91/v13_pose_jump_CORNER_RECOVERY/stack/session.log)

15:06 的一次只读进程快照也看到 Domain 71/72 两个不同的 Gazebo server PID；这是当前状态的补充，不作为历史并发的替代证据。本分析没有停止它们。

### 1. 共享上游慢/停有较强证据

健康记录中，四路都出现相同采集时间戳 37.600 s 长时间不更新，约在采集开始后的现实时间 16.55–17.19 s 才转换到 37.650 s。以健康消息首次看到新采集时间戳计算，四路转换间隔约为 500–599 ms。这是健康派生间隔，不冒充原始图像最大间隔。

整个窗口各路健康消息的最大观察回调间隔约 54–71 ms，合并四路后最大空档为 31.756 ms。说明观察程序并没有完全停止执行半秒；实际健康节点也在旧图像上持续报告 STALE。逐图像回调调度偏差仍可能存在。

同一时段四路健康消息中的 ROS 时钟也出现重复/缓慢推进，支持仿真推进或共享 `/clock` 交付路径停顿。当前数据不能独立区分 Gazebo 原生时钟停止、图像渲染阻塞和桥接交付停止。

来源：[逐条健康记录](../runs/task_chain_20260922_afternoon/camera_09_async_cuda/capture/health_events.json)。

### 2. 渲染线程同步是已发现的热点，但不是该极值的唯一已证根因

较早的独立 30 秒采样中，Ogre worker 调用栈包含 45% 样本，futex_wait 独占 25.2%；4 线程诊断对应约为 13.9% 与 4%。这些是 profiler 样本比例，不是图像等待时间占比，也不是受控加速倍数。同机负载与测试条件存在变化。

异步轮启动环境已经配置 `ASTRIBOT_OGRE2_WORKER_THREADS=4`，不能把“从 28 改成 4”当作尚未做过的修复。已有 `renderer_runtime.json` 对应更早 PID，不足以单独证明 navigation_09 的实际库和线程数；下一轮应随每个实例冻结 `/proc/<pid>/maps`、配置和线程记录。

来源：[渲染诊断说明](SIM_RENDERING_GPU_CANDIDATE_20260922.md)、[原始 profiler](../runs/task_chain_20260922_afternoon/profile_01.txt)、[4 线程 profiler](../runs/task_chain_20260922_afternoon/profile_02_threads4.txt)、[异步启动环境](../runs/task_chain_20260922_afternoon/query_env_async.sh)。

### 3. 实际运行仍使用共同传感器桥

`navigation_09_async/session.log` 中 `parameter_bridge-6` 同时创建 `/clock`、里程计、TF、双 LiDAR、IMU、四路 RGB-D 和双目桥接。另一 `parameter_bridge-2` 是 social state。不能把它们解释成相机已按路拆分。

共用进程形成共享分配、转换、调度和发布资源，是合理的长尾候选，但“同进程”不等于所有传输回调一定同线程串行；必须测量桥接入口/出口时间。已有大图像 SHM 配置也不能替代该测量。

来源：[本轮运行日志](../runs/task_chain_20260922_afternoon/navigation_09_async/session.log)、[当前启动源码](../ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py)。

### 4. 点云投影耗时不符合半秒瓶颈量级

下表是节点末尾累计日志覆盖的节点生命周期最大值，不是精确限定到 600 秒窗口的逐帧数据，各列最大值不一定来自同一帧。

| 节点 | 后端 | 最大计算 / ms | 最大回调收件至 worker 开始 / ms |
|---|---|---:|---:|
| 头部 | CPU | 2.163 | 90.854 |
| 腹部 | CPU | 1.170 | 125.371 |
| 左腕 | CUDA | 4.074 | 113.361 |
| 右腕 | CUDA | 3.648 | 101.103 |

四路日志均为 replaced=0、expired=0、errors=0，pending_peak=1；已配对工作项未显示持续堆积。接收至 worker 开始还包含 Depth/CameraInfo 配对等待，不能全解释成 CPU 调度延迟。

当前部分输入时间门控直接 return，且未打印 received/evicted；因此这些零值不能证明所有输入都没有被拒收。原始深度探针与点云工作线程是不同消费者，后者的局部排队不直接等于前者的接收空档。

来源：[头腹日志](../runs/task_chain_20260922_afternoon/navigation_09_async/session.log)、[左腕日志](../runs/task_chain_20260922_afternoon/camera_09_async_cuda/left_wrist_rgbd.log)、[右腕日志](../runs/task_chain_20260922_afternoon/camera_09_async_cuda/right_wrist_rgbd.log)、[指标实现](../ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp)。

### 5. CUDA 压力与停顿重叠，但因果未建立

GraspNet 压力窗口约从采集第 12 秒持续到第 561 秒，262 次完成的 worker 调用几乎连续；各次均重新加载模型，load_ms 约 168–320 ms。共同停顿窗口与这些调用重叠，但持续负载覆盖绝大部分实验，时间重叠本身不足以证明 GraspNet 触发停顿。

点云 decimation=4 的独立 640×360 基准中 CPU 中位数约 0.050 ms，CUDA（包含 H2D/kernel/D2H）约 0.187 ms。此配置继续扩大 CUDA 覆盖不是优先修复；结果不能直接推广为其他分辨率或 Orin 真机性能。

来源：[推理压力原始结果](../runs/task_chain_20260922_afternoon/inference_09_cuda/report.json)、[投影基准](../runs/task_chain_20260922_afternoon/projection_benchmark.jsonl)。

## 解决方案与执行顺序

### P0：先建立真正独占的仿真性能基线

协调其他任务的所有者，在独占时间窗口重跑完全相同的四路相机、异步投影、GraspNet 和导航负载，保持全部几何、参数与安全阈值不变。不要根据进程名全局清理其他人的实例。记录全机 Gazebo server 的 PID、实例、启动/退出时间与 CPU/GPU 负载，并检查测试中途是否有新实例进入。

依次对照单仿真、单仿真加本任务推理、已知第二实例并发，分别比较 RTF、P99、最大接收间隔、>250 ms 次数与 STALE。若独占后长尾消失，应首先以全机性能测试租约/串行调度解决实验干扰；若独占后仍超限，再执行以下分段诊断与实现候选。多实例并发功能测试仍可保留，但须明确不属于独占时延验收。

### P0：补齐能区分阶段的观测

增加低开销 C++ 诊断，按 camera_id、source stamp、epoch 关联以下事件：

1. Gazebo 生成/发布图像和原生时钟的 steady 时间；
2. bridge 收到消息、转换完成、ROS publish 前后的 steady 时间；
3. ROS 消费者收到图像、配对成功、worker 开始/结束、发布前后的 steady 时间。

保存逐帧元数据或有界异常前后窗口，异步落盘；不在热回调保存整幅图片。指标分别记录源帧间隔、传输/转换耗时、回调调度、配对等待、计算、发布和队列峰值；拒绝原因区分 future、old、out-of-order、unpaired、evicted、overwritten。

当前 Python 采集器同一执行循环同时接收六路图像、健康、点云、TF 和 `/clock`；600 秒记录了 325262 条时钟样本，且只保存图像间隔分位数，没有保存原始逐帧接收时间。因此无法严谨地给四个图像最大值做事件级因果对齐。应增加独立元数据探针并对照观察者开/关对时序的影响。

判定原则：Gazebo 原生发布已慢则优化仿真/渲染；原生正常而桥出口慢则优化桥；桥出口正常而订阅慢则检查 DDS/接收调度；原始输入及时而结果慢才针对投影/推理优化。

### P1：控制渲染/计算调度竞争

保持相机几何、分辨率、采样率、碰撞几何、物理步长与 250 ms 门槛，固定其他负载，对 Ogre worker=1/2/4/8 做短诊断；以 P99、最大间隔、超限次数而非平均 FPS 选候选。测试期间不叠加构建或其他未记录负载。

确认 CPU 调度等待后再为仿真、桥接与接收线程配置有限线程预算或分配互不抢占的 CPU 集合，并同时检查控制线程延迟。不能直接提高所有线程优先级，也不能用无限增加线程掩盖 barrier 等待。

### P1：实际拆分桥接，并明确队列策略

将 clock/状态、LiDAR 与图像桥拆开；同一相机 RGB/Depth/Info 保持成组、原始 stamp 和同步约束。保留大图像 DDS SHM 配置，核实每个实际发布/订阅进程加载的配置。

相机采用兼容的 sensor-data QoS 和有界小队列（从 2–4 的候选开始），积压时丢旧、统计缺对；状态/控制接口保留原有可靠性。具体参数按已安装 Humble 桥版本验证。拆分仅作为受控 A/B 候选，不预先宣布能解决 Gazebo 产出停顿。

### P2：减少无收益的 GPU 竞争

用四路 CPU 异步稀疏投影建立对照，再分别加入双腕 CUDA、GraspNet 和实际运动，测量增量影响。保持既有最新有效帧与有界缓存，不重复实现已经完成的 worker 改造。

若确认 GraspNet 冷启动/模型加载或并发竞争引入峰值，可改成长驻 C++ worker、预热并复用模型与缓冲，限制同时在途请求；保留超时、取消、epoch 和退出恢复。若同卡推理仍干扰渲染，再评估分离 GPU 或对推理做负载准入。

### 验收

每个候选先做单因素对照，再覆盖四路相机、SLAM、实际 MPPI 运动和推理联合负载；明确导航是否真正运动以及停止/取消时段。最终执行完整 30 分钟压力窗口，并分别记录各阶段。

沿用 250 ms 时效合同：完整要求窗口内无未解释的接收超限、STALE 或失效后旧帧发布，原始采集 stamp 不被重盖；同时验证配对、深度几何、会话撤销、时钟异常、控制反馈与消费者 ACK。若未达到则继续标记未验收，不靠扩大阈值或减少安全覆盖获得通过。有限窗口通过不构成硬实时保证。

本轮输出为原因分析与方案，尚未实施或验证上述新候选。
