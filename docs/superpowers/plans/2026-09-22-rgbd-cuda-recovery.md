# W1 后续：RGB-D CUDA 处理恢复

状态：步骤 1–6 已有候选实现，已完成进程/ROS 夹具和同仓库双腕 CUDA 负载回归；下游导航保护与同类时序复核继续验证。最终范围和原始证据见 `docs/RGBD_CUDA_PROCESS_RECOVERY_20260922.md`。本计划延续已批准的 W1 故障恢复任务；C++ 运行时、独立安装、同一导航仓库仿真、不降低 250 ms 时效门槛。真机、VLA 和完整动态抓放继续暂缓。

## 已确认问题与边界

- `depth_projection_cuda.cpp` 的计算结束及析构都调用无期限的 `cuStreamSynchronize`；`rgbd_pointcloud_node.cpp` 析构直接等待工作线程。因此“回调不阻塞”不等于“GPU 不返回时可恢复或退出”。
- 原始 `CameraHealth` 只检查图像与内参。CUDA 计算无输出时它仍可能 OK，不能作为处理链健康。此次相机 source epoch 修复不覆盖 processing epoch。
- 已有 GraspNet `worker_process.cpp` 具有 C++ spawn、私有进程组和计算期限，但终止后仍阻塞 `waitpid`。不能直接称为有界 GPU 故障恢复器。
- 输入非法、单次扩容 OOM、GPU 忙但最终返回，与进程状态损坏、驱动调用永久阻塞、设备级故障必须区分。已有 OOM 测试只证明特定分配失败后重试正确。

## 数据流与控制权

```text
C++ 短接收回调 → 有界精确配对 → 最新待处理槽
  → C++ 子进程管理器（不调用 CUDA）
      → 持久 CUDA worker：初始化、缓冲复用、投影
      ← 有界结果：parent_boot / worker_epoch / request / generation
  → 父节点校验采集时效、会话、标定上下文与版本
  → 唯一 PointCloud2 发布点

原始相机健康 + 处理健康 → 真正依赖点云的能力准入
```

CPU 默认路径保留现有线程实现；CUDA 显式选择持久子进程，不为每帧创建进程。原始采集 header 始终保留。处理健康与相机健康分开，不能为了复用消息而把投影故障伪装成相机断流。

## 依次实施

1. 冻结 worker IPC 契约和最大输入/输出尺寸。请求同时绑定父进程启动身份、worker epoch、单调请求编号、输入 generation 和采集时间；跨进程返回不能刷新原始期限。单项进行中、单项最新待处理，禁止无界排队。IPC 必须有界、非阻塞、校验长度；不得传裸指针。
2. C++ 监督状态机：STARTING → READY → RUNNING；超时/退出/不可继续错误 → STOPPING；确认退出后可有限次数 BACKOFF/重启；未确认退出 → QUARANTINED。等待初始化、处理、终止和回收均采用 steady deadline。使用 pidfd 或已证明归属的私有进程组，非阻塞回收，不把发送 KILL 等同于已退出。
3. 父节点在故障时撤销发布、递增 processing epoch、清配对和待处理缓存；旧 worker 的迟到结果必须在唯一发布点丢弃。新 worker 自检通过后，只有恢复边界之后的新完整帧处理成功才能恢复处理健康。
4. 暴露处理状态、实际 backend、worker epoch、最后成功采集/发布时间、队列深度、超时/丢弃/重启原因。选用项目现有强类型状态惯例，再接确实依赖点云的消费者；标准 PointCloud2 自身不携带内部 generation，不能指望下游从它还原进程版本。
5. CPU 回退默认关闭；仅预先显式配置允许时启用，必须记录 backend 切换、原因与新 processing epoch。CPU 不能满足原预算则保持无效，不能静默降低频率、精度或安全门槛。
6. 独立候选构建和故障夹具通过后，再进行同导航仓库的双腕负载与故障回归。记录新安装版本，不能继承旧线程实现的 30 分钟压力放行。

## 验收矩阵

| 场景 | 注入 | 放行条件 |
|---|---|---|
| 慢计算，最终返回 | C++ delayed projector，预算内/外 | 接收继续、队列有界、过期不发布、新帧恢复 |
| 单次/连续 OOM | 现有分配拦截夹具 | 不沿用错误容量；恢复后数值等价；连续失败有界拒绝 |
| worker 返回不可继续错误 | 独立 C++ fixture | 不复用坏上下文，换 worker epoch，旧请求无成功结果 |
| worker 不返回 | 阻塞 fixture / 暂停自有 worker | 父节点状态仍响应、及时禁发，退出回收过程可观测 |
| 未确认进程退出 | 假进程适配器模拟 reap 未完成 | 进入 QUARANTINED，不启动替代 worker，不虚报释放 |
| 恢复后迟到旧结果 | fixture 保存旧请求后延迟递交 | 旧 epoch/request/generation 被拒，新完整帧才放行 |
| 计算中上下文变化 | session、标定、clock rollback | 原请求失效，不通过重试延长旧帧期限 |
| GPU 忙，原始图像正常 | 受控已有 CUDA 负载 | raw health 与 processing health 分开，点云能力缺失不能放行 |
| 显式 CPU 回退开/关 | 两种配置 | 关闭时明确失败；开启时状态可追踪，精度/时效不降低 |
| 退出与故障同时发生 | 阻塞 fixture + shutdown | 实测完整调用墙钟；不能只检查 kill 前填入的 elapsed |
| 双腕并发，一侧故障 | 单侧 fixture | 不误杀、不重置另一侧；隔离资源、日志和 epoch |

不主动制造真实非法 kernel、GPU reset、驱动重载或设备级挂死。独立进程能隔离进程内状态，不能隔离显存、计算带宽或整个设备故障。设备无法恢复时正确结果是撤销能力并保留明确故障，不能声称自动恢复。默认设备 0、primary context 与未来同进程组件的关系需在设备选择契约中记录。

## 当前证据关系

`docs/RGBD_EXCLUSIVE_REGRESSION_20260922.md` 保留原 CPU/CUDA 配对、压力及 source epoch 恢复记录。新实现的测试及同仓库验证单独位于 `docs/evidence/task_chain_20260922_gpu_recovery/` 和 `runs/task_chain_20260922_gpu_recovery/`。旧实现测试与仿真结果不能替代新候选验收；进程内故障夹具及 SIGSTOP/SIGKILL 恢复不外推为真实设备级挂死恢复。
