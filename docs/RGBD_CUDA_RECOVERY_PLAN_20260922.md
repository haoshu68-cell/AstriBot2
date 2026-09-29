# RGB-D CUDA 故障隔离与恢复实施计划

> **For agentic workers:** 使用 `superpowers:executing-plans` 或 `superpowers:subagent-driven-development` 按下列独立任务实施和审查。当前仅交付计划；复选框均表示尚未实施。

**状态：DESIGN / NOT_IMPLEMENTED / NOT_VALIDATED。** 本文来自 2026-09-22 的只读源码审查，没有运行 ROS、CUDA 或故障注入。它是 W1 的下一子项，不是 W1 放行记录，也不改变其他任务的验收范围。

**Goal：** CUDA 投影阻塞或失败时，使 ROS 接收、失效判定、取消和节点退出不依赖 CUDA 调用返回，并阻止旧结果提交和重复启动失控 worker。

**Architecture：** 保留 C++ ROS 节点的短回调、有界配对、最新待处理槽及最终提交检查；将 CUDA context、NVRTC 和投影计算放入受父进程监督的常驻 C++ 子进程。父进程使用有界 IPC、steady deadline 和非阻塞回收；无法确认子进程退出时保持隔离，禁止再次启动 CUDA worker。

**Tech Stack：** 当前 C++17、ROS2 Humble、CUDA Driver API / NVRTC；Linux `posix_spawn`、`socketpair`、`poll`、`memfd_create`、`waitpid(WNOHANG)`，优先使用可用的 pidfd 管理直接子进程身份。不新增 Python 运行时模块。

**Spec：** [任务链中的 W1 章节](superpowers/plans/2026-09-21-remaining-work.md)、[当前异步 RGB-D 设计与未放行项](RGBD_ASYNC_CUDA_20260922.md)、[本轮独占环境回归记录](RGBD_EXCLUSIVE_REGRESSION_20260922.md)。实施前读任务链正文 W1。

## 全局约束

- C++ 优先；Python 仅用于启动、外部采集、验证和分析。
- `projection_backend` 默认 CPU。选择 CUDA 后遇错不得静默切 CPU；任何自动 fallback 必须另有显式 opt-in，默认关闭。本子项首版不新增自动 fallback。
- 保持当前默认 `max_pair_age_sec=0.25`，允许配置范围不扩大；不通过延长安全有效期换取通过。监督恢复预算与帧有效期分别记录。
- 保留原始采集 stamp 和 optical frame；重试不刷新帧的 ROS/steady 截止时间。
- 配对缓存各最多 4 个引用、待处理槽 1 个、在途请求 1 个；本计划不增加第二个在途投影。
- 当前单图上限为 4,194,304 像素和 33,554,432 字节；输出每点 16 字节，最大 67,108,864 字节。所有乘加、偏移和转换先验证再分配。
- 不 detach 持有节点或帧缓冲的线程；不使用线程强制取消规避生命周期问题。
- 不做 GPU reset、驱动卸载、设备重置、宿主机重启或其他硬件恢复操作。硬件/驱动恢复不在本方案内。
- kernel D-state 等原因导致 SIGKILL 后仍未退出时，结论是 **QUARANTINED / worker 未确认回收**，不是“已杀死”或“资源已释放”。禁止同一逻辑处理实例重新启动 worker；即使父节点重启也必须检查持久化隔离记录。
- 不修改导航碰撞、心跳、几何 ACK 或相机会话权限门槛。原始 CameraHealth 与派生点云处理状态分别表达。
- 本计划不直接修改 `astribot_s1_manipulation_perception` 的推理服务。复用其拥有权和监督模式，避免为 RGB-D 每帧创建推理进程。
- 仿真、GPU 压力和运动验证只能在已核对归属的独占会话中执行；本文没有执行这些操作。

## 审查重点

1. GPU 不返回时，CPU 侧 deadline 是否仍可推进；析构是否偷偷重新进入 CUDA 或阻塞回收。
2. EOF、半个响应、错误长度、重复/旧代际响应是否都不能生成有效点云。
3. 杀进程后不退出时是否仍误释放共享缓冲、启动第二个 worker，或丢失进程身份。
4. 取消、完成、时钟回退、标定/会话变化相邻发生时，旧结果是否仍可能提交。
5. 性能报告是否记录完整 API 返回耗时和恢复时间，而不是仅记录发信号前的 elapsed。

## 当前实现与证据边界

以下行号为审查时源码位置，实施前应重新核对；它们描述现状，不代表新方案已经实现。

| 现状 | 源码证据 | 边界 |
|---|---|---|
| 计算不持接收锁；新帧替换一个待处理槽 | [rgbd_pointcloud_node.cpp:134](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp:134) | 计算永不返回时仍只有一个在途任务，但没有恢复 |
| 计算开始与提交前核验原始时间、generation、会话 | [rgbd_pointcloud_node.cpp:125](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp:125)、[提交路径:156](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp:156) | 证明晚返回可被丢弃，不证明 CUDA 调用可取消 |
| 工作线程异常被捕获，之后继续复用 projector | [rgbd_pointcloud_node.cpp:172](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp:172) | 没有损坏 context 的隔离/重建状态机 |
| 析构等待工作线程 join | [rgbd_pointcloud_node.cpp:102](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp:102) | 未设置完整退出期限 |
| CUDA 计算及清理调用同步等待 | [depth_projection_cuda.cpp:69](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/depth_projection_cuda.cpp:69)、[清理:30](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/depth_projection_cuda.cpp:30) | 正常、错误及析构阶段均可能阻塞；单换 `cuStreamQuery` 不能覆盖全部路径 |
| CUDA 未构建时明确失败 | [depth_projection.cpp:38](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/depth_projection.cpp:38) | 保持显式 backend，不能静默 fallback |
| 显存扩容失败后可重新分配 | [depth_projection_cuda.cpp:62](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/depth_projection_cuda.cpp:62)、[现有注入测试:67](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/test/depth_projection_test.cpp:67) | 不等于非法访问、context 失效、设备丢失都可恢复 |
| CameraHealth 订阅 RGB/Depth/CameraInfo | [camera_health_node.cpp:93](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/src/camera_health_node.cpp:93) | 原始相机 OK 不等于 GPU 点云 OK |
| 推理 worker 用私有进程组、steady deadline 和 SIGKILL | [worker_process.cpp:26](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/worker_process.cpp:26) | 可复用设计模式；不直接建立感知组件到抓取服务的反向依赖 |
| 推理 worker 杀后仍阻塞 waitpid | [worker_process.cpp:70](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/worker_process.cpp:70) | `elapsed_sec` 在回收前更新；[既有测试:19](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/test/worker_process_test.cpp:19) 未独立测完整函数耗时，不能照抄其“有界”结论 |

## 最小数据流与控制边界

```text
Depth + CameraInfo
  -> ROS 父节点：时间/尺寸校验、各 4 帧缓存、精确配对
  -> 最新待处理槽（1）
  -> 父节点处理线程：最后一次准入检查 -> 写入独占输入区
  -> 固定长度请求头（worker instance + generation + request ID）
  -> 常驻 C++ 子进程：CUDA 初始化一次 -> 投影 -> 写独占输出区
  -> 固定长度响应头
  -> 父节点：协议/身份/长度/时效/会话复核 -> 原 stamp 点云发布

steady deadline / 取消 / shutdown / 子进程退出 / IPC 故障
  -> 使在途请求失效 -> 拒绝提交 -> 停止拥有的子进程
  -> 已回收：有限重试与新配对
  -> 未确认回收：QUARANTINED，禁止重启
```

CPU 模式继续使用当前本地 projector。CUDA 模式父节点只操作 IPC 和普通内存，不初始化、清理 CUDA，不等待 CUDA event。处理状态与原始相机健康分开：建议新增 `RgbdProcessingState` 消息，字段在任务 4 中固定；本子项不把 CameraHealth 的 `source_epoch` 重新定义成 CUDA worker 的代际。

## IPC v1 契约

采用一个 `AF_UNIX / SOCK_STREAM` socketpair 传递控制消息，父端非阻塞；两个继承到子进程的 memfd 分别存输入和输出。最大控制帧 256 字节，输入/输出容量分别受上述 32 MiB / 64 MiB 限制。memfd 限制扩缩容；不在每帧创建文件，不通过临时图片或 JSON 传输像素。

- 每次 worker 启动创建新的 instance ID、socket 和两个 memfd；旧 worker 未回收前，不把它的内存用于新 worker。
- 仅一个在途请求。父进程提交前写完整输入，子进程收到合法请求后读取；子进程写完整输出后才发送完成响应。父进程收到完整且匹配的响应后才能读取输出。
- 头部显式逐字段编码，不直接发送 C++ struct，避免 padding/ABI 依赖。固定 little-endian；包含 magic、协议版本、消息类型、头长度、状态码、instance ID、generation、request ID、输入长度、输出长度、width/height/step、encoding/endian、decimation、投影参数。参数采用显式 IEEE754 binary64 编码并检查有限值；编码表与字节偏移由协议单元测试锁定。
- `READY` 握手发生在初始化完成后。`REQUEST` 与 `RESULT` 必须分别使用固定长度；`ERROR` 只传枚举错误码，不接收任意长度错误字符串。其他消息类型直接拒绝。
- 先验证 magic/version/type/固定头长、身份和长度，再读共享区。`input_bytes <= 33554432`，`width*height <= 4194304`，`step*height <= input_bytes`；运算采用经过溢出检查的 uint64。输出长度必须严格等于 `ceil(width/decimation)*ceil(height/decimation)*16` 且不超过 67108864。
- 非阻塞读取使用固定数组和 `bytes_received`；短读/EAGAIN 不算成功，不重新计时。固定头完整之前 EOF 为 `IPC_TRUNCATED_RESPONSE`，零字节即 EOF 为 `WORKER_DISCONNECTED`。
- 完整响应后的 EOF：先做 `waitpid(WNOHANG)` 状态检查；若已确认 worker 退出，响应不提交。即使暂未确认退出也必须经过正常时效/代际/会话检查，随后断连关闭该 worker，不复用连接。
- EOF、协议非法、超时、取消、进程退出任一种终态只结算一次；同一请求的重复响应、旧 instance/generation/request ID 全部拒绝。
- 对每个请求保留不可延长的 `min(接收 steady 截止时间, 该请求监督截止时间)`；首次发送、半包、重试、READY 或重复响应均不得刷新截止时间。
- 分配共享容量、传输/解析、初始化、处理、停止/回收分别计时。新增 IPC 开销必须纳入同输入 CPU/CUDA 对照，不预先承诺 CUDA 一定更快。

## 状态、故障与恢复

`STOPPED -> STARTING -> READY -> BUSY -> READY` 是正常状态。故障进入 `STOPPING`；确认回收才进入 `BACKOFF`，未确认回收进入 `QUARANTINED`。重试预算耗尽进入 `FAILED`。

首版建议配置为：启动预算 10 s、监督检查周期 10 ms、回收预算 500 ms、重试最多 2 次、退避依次 250/500 ms。**这些是候选实现/测试参数，不是已获实测支持的验收成绩。** 本计划不要求扩大帧有效期；请求时限始终不晚于该帧剩余有效期。生产配置合入前须固定参数清单并验证负载下的误杀率。

取消与 deadline 优先于同时到达的成功。故障处理先递增处理 generation、使在途请求不可提交，再向拥有的子进程发停止信号；每次状态切换记录唯一 fault ID。新 worker 的 READY 不等于已恢复，必须等到新的完整配对成功发布，且发布时满足原有时效、标定、时钟和会话检查。

停止阶段使用非阻塞 waitpid/pidfd 观测；不得在超时路径或析构中调用阻塞 waitpid。若回收期限到达仍未确认退出，记录 pid、启动身份、instance、进程组、故障原因和隔离状态，继续保留拥有权。普通故障进程可被杀死并回收；D-state 等不可立即回收场景只能承诺父进程失效判断和状态返回，不承诺 GPU/内核已停止。这里的有界是用户态监督和测试预算，不是对内核、文件系统和调度器作硬实时保证。

父进程退出前不应再等 CUDA 或无限 waitpid；已存在的 worker 身份/隔离记录不得因退出被删除。节点重启时先在该逻辑处理实例的私有运行目录加锁并检查记录：身份仍活跃或无法确认时禁止启动；确认旧身份已不存在后才能清理记录并重新准入。不得凭裸 PID 相同就向无关进程发信号，不把掉锁等同于 GPU 已释放。重试计数属于逻辑处理会话并随拥有权记录保存，不能通过重启父节点清零；预算耗尽后需要显式发起新的处理会话，且仍要先确认旧 worker 已退出。

## 文件与任务拆分

以下文件均为**拟新增/修改**，当前文档没有创建这些模块。运行时新增文件统一放在 `ws_robot/src/astribot_s1_perception_components`，不创建新的算法服务包。实施时将源快照、构建、安装和日志分别冻结到 `runs/rgbd_cuda_recovery_20260922/src`、`build`、`install`、`logs`；若该目录已存在，新增带唯一后缀的目录并同步证据路径，不覆盖旧实验。以下测试命令以该首次目录为准，执行时不得把测试目录换成共享 install。

### 任务 1：固定有界 IPC 协议和离线故障夹具

**文件：** 新增 `include/astribot_s1_perception_components/projection_worker_protocol.hpp`、`src/projection_worker_protocol.cpp`、`test/projection_worker_protocol_test.cpp`、`test/projection_fixture_worker.cpp`；修改本包 `CMakeLists.txt`。

**接口：** 提供 `encode_request`、`decode_request`、`encode_response`、`decode_response` 和增量读取器；编码结果为固定 `std::array<uint8_t, N>`，非法输入返回枚举 `ProtocolError`。只有完整校验后返回结构化结果，不暴露“部分有效”响应。

```cpp
enum class DecodeState { NeedMore, Complete, Rejected };
enum class ProtocolError { None, BadHeader, BadIdentity, BadSize,
                           Truncated, Disconnected, Duplicate };
struct DecodeOutcome { DecodeState state; ProtocolError error; };
// kResponseHeaderBytes 由 v1 编码表确定，且静态断言 <= 256。
DecodeOutcome append_response_bytes(const uint8_t* data, size_t size);
DecodeOutcome finish_response_on_eof();
```

- [ ] 先写失败测试：按每个可能字节位置切分合法响应；完整前均 `NeedMore`，EOF 均 `Truncated`，全部字节到齐才 `Complete`。
- [ ] 写输入/输出容量的 `limit-1 / limit / limit+1`、整数溢出、零尺寸、非法 decimation、NaN/Inf 内参、错误实例/代际/request ID 测试。
- [ ] 实现固定编码和检查；通过所有协议测试。fixture 支持 `ok`、`partial`、`eof`、`bad_size`、`old_generation`、`duplicate`、`hang_init`、`hang_request`、`hang_shutdown`，这些均不需要 GPU。
- [ ] 运行 `ctest --test-dir runs/rgbd_cuda_recovery_20260922/build -R projection_worker_protocol_test --output-on-failure`，保存 RED/GREEN 与协议编码表。

### 任务 2：独立子进程拥有权、有界回收和隔离

**文件：** 新增 `include/astribot_s1_perception_components/projection_worker_process.hpp`、`src/projection_worker_process.cpp`、`test/projection_worker_process_test.cpp`；修改本包 CMake。

**接口：** 定义 `WorkerIdentity`、`StopOutcome { Reaped, Quarantined }` 与 `stop_owned_worker(deadline)`；系统调用封装可注入，生产实现只操作本实例创建的进程。子进程使用固定绝对路径及 argv，不经过 shell。

```cpp
enum class StopOutcome { Reaped, Quarantined };
struct StopResult {
  StopOutcome outcome;
  std::chrono::nanoseconds total_elapsed;
  bool exit_observed;
};
// deadline 使用 steady_clock；不含任何 CUDA 操作或阻塞 waitpid。
StopResult stop_owned_worker(std::chrono::steady_clock::time_point deadline);
```

- [ ] 用假 ProcessOps 注入“kill 成功但每次非阻塞 wait 都未退出”，先让测试失败；预期回收期限后返回 `Quarantined`，下一次 start 明确拒绝。
- [ ] 实现非阻塞监督、身份记录、启动互斥及重启时的隔离检查；测试 PID 被复用时不能向新身份发送信号。
- [ ] 使用无需 GPU 的 fixture 验证真实可杀进程、提前退出、取消与退出同时发生、父节点退出后隔离记录保留。禁止故意制造真实内核 D-state；该分支以系统调用夹具验证。
- [ ] 用调用者外部 steady clock 测完整返回耗时，包括发信号、回收、状态整理；不要只断言结果中的杀前 elapsed。测试记录实际返回时间及采用的调度余量。
- [ ] 运行 `ctest --test-dir runs/rgbd_cuda_recovery_20260922/build -R projection_worker_process_test --output-on-failure`；证明无孤儿/重复 worker 的范围仅限实际 fixture，D-state 实测保持 NOT_RUN。

### 任务 3：常驻 CUDA worker 和进程代理 projector

**文件：** 新增 `src/depth_projection_worker_main.cpp`、`src/depth_projection_process.cpp`、`test/depth_projection_process_test.cpp`；修改 `include/astribot_s1_perception_components/depth_projection.hpp`、`src/depth_projection.cpp`、`CMakeLists.txt`。CUDA 算法本身保留在现有 `depth_projection_cuda.cpp`。

**接口：** `ProcessDepthProjector` 仍实现现有 `DepthProjector::project`；代理只操作共享内存/IPC，实际 `make_cuda_depth_projector()` 仅在子进程调用。CPU 工厂路径保持原实现。父节点工作线程可在非阻塞 IPC 等待期间检查 shutdown/context 取消标志。

- [ ] 先用任务 1 的 fixture 写测试：正常输出、初始化不返回、计算不返回、半包后挂起、错误响应、EOF、重复结果、旧 worker 响应；每种失败都不得返回成功 output。
- [ ] 实现常驻子进程初始化一次、一次一个请求；每个 worker 新建共享区域。检查命令绝对路径、FD 继承白名单、输入/输出长度和身份。
- [ ] 为初始化、请求、停止分别传递不可延长的 steady deadline；重试重新建 worker，但不得重放过期帧或提高并发数。
- [ ] `hang_shutdown` 夹具验证父析构不调用子进程 CUDA 清理，也不无限等待；未确认退出进入隔离。
- [ ] 先运行 CPU-only fixture 测试；其通过后才在独占 GPU 下运行同输入 CUDA 数值和错误注入。未构建 CUDA 的配置必须仍明确拒绝 backend=cuda。

### 任务 4：ROS 提交、处理状态与恢复条件接入

**文件：** 修改 `src/rgbd_pointcloud_node.cpp`、`test/rgbd_pointcloud_test.cpp`；在 `ws_robot/src/astribot_perception_msgs/msg` 新增 `RgbdProcessingState.msg`，修改消息包 CMake。若现有状态消息已满足下列字段，实施时优先复用且记录精确字段映射，不维护同义的双份状态。

**拟定消息字段：** header、camera_id、backend、worker_instance、processing_generation、state、reason_code、inflight_request_id、last_input_stamp、last_output_stamp、inflight_wall_age_sec、fault_count、restart_count、worker_exit_confirmed。`state` 区分 STARTING/READY/BUSY/STOPPING/BACKOFF/QUARANTINED/FAILED；“READY”不能代替 fresh output 判定。

- [ ] 先扩展 ROS 测试：GPU fixture 超时后原始 CameraHealth 仍 OK，但处理状态失效且无新点云；新 worker READY 而无新配对时仍不能报告输出恢复。
- [ ] 保留已有 Job generation、ROS age、steady 接收 age 和会话核验；新增 processing generation 只约束派生计算，不能改写原始 source epoch 或 stamp。
- [ ] 响应准备提交时再次检查 shutdown、会话释放、token/owner/execution、标定/frame、时钟回退和原期限。使用同步屏障夹具覆盖“完成与取消同一轮”并让取消优先。
- [ ] 故障后清除待处理旧 Job，仅允许 fault 后接收的新完整配对提交；重复图像和旧 IPC 响应不能触发恢复。旧 in-flight 缓冲直到确认 worker 不再使用才回收。
- [ ] 确认状态发布不依赖 CUDA 锁，不把 5 秒累计日志当恢复心跳；保持当前默认 CPU 及明确 CUDA 选择，未配置 opt-in 不存在 fallback。
- [ ] 运行现有 rgbd_pointcloud、depth_projection、latency 和 camera health 合同测试，以及新增进程/IPC 测试；记录数值、版本和取消语义未退化。

### 任务 5：独占环境的故障和性能回归

**文件：** 新增外部验证脚本 `tools/vision/validate_projection_recovery.py` 和证据目录 `docs/evidence/rgbd_cuda_recovery_20260922/`；实施完成后更新本文状态与已有 RGB-D 文档。脚本仅协调已拥有的验证进程，不能作为运行时 watchdog。

- [ ] 固定配置、二进制哈希、模型/输入、实例、domain/partition、GPU 负载、时钟和处理状态话题，记录独占会话清理结果。
- [ ] 先静止测试表中 CPU-only 故障，再做真实 CUDA 正常/错误/竞争负载。导航运动组合只在下游派生点云新鲜度门控已被确认后开展，不以 CameraHealth OK 直接放行。
- [ ] 对同一分辨率、抽稀、输入和负载交错进行旧实现/进程实现 A/B；记录接收→提交、IPC、计算、恢复、父节点 shutdown 完整耗时、RSS/显存、丢帧及原因，不能仅比较 kernel 时间。
- [ ] 每个故障后再跑正常帧，确认新代际、新完整配对、原始 stamp、新鲜结果；零恢复样本不能算通过。
- [ ] 正常组合按 W1 既有要求运行 10 分钟、联合压力 30 分钟；这些持续运行不能替代故障矩阵，也不能复用旧二进制成绩给新实现放行。
- [ ] 生成结果矩阵；按 functional/safety/performance/evidence 四维分别判定，保留失败和无效实验。没有真实设备/驱动硬故障证据时明确标 NOT_RUN。

## 验证矩阵与放行条件

所有行初始 `execution=NOT_RUN`。CPU fixture/系统调用模拟是离线证据；隔离 ROS 与完整导航仿真分别记录，不能合并为真机验收。

| 场景 | 刺激 | 必须满足的独立判据 | 归属任务 |
|---|---|---|---|
| P01 正常投影 | CPU/CUDA 同输入、尺寸和抽稀 | 数值、stamp、frame 一致；完整延迟入预算 | 3、5 |
| P02 尺寸/协议边界 | 最大值附近、溢出、非法头 | 分配/读取前拒绝；无成功结果 | 1 |
| P03 EOF/partial | 在每个头部位置截断、半包停顿 | 不提交；不可刷新 deadline；明确原因 | 1、3 |
| P04 GPU 忙/慢返回 | 耗时在帧期限前后交错 | 超期零提交，队列有界，记录丢帧 | 3、5 |
| P05 CUDA 返回错误 | OOM、launch/sync 错误注入 | 失败结果不发布；按错误策略恢复或失败 | 3、5 |
| P06 永不返回 | 初始化/请求/析构 fixture hang | 父响应与完整 shutdown 在测试预算内；无 CUDA 等待 | 2、3 |
| P07 不可回收 | 假 waitpid 永远未退出 | QUARANTINED；不声称释放；重启被拒绝 | 2 |
| P08 迟到/重复 | 旧 instance/generation/request ID | 零提交、零旧结果恢复 | 1、3、4 |
| P09 取消竞争 | 完成同时撤销会话/停止 | 取消优先；资源回收结果可追踪 | 2、4 |
| P10 上下文变化 | GPU 在途时回退时钟、改标定/会话 | 旧 Job 作废；新配对才恢复 | 4 |
| P11 连续故障 | 超过 2 次重试、退出再启动 | 退避/预算不重置，状态 FAILED/隔离可辨 | 2、3 |
| P12 健康分离 | 原始流正常、投影停止 | raw CameraHealth 与处理状态分离；下游不沿用旧点云 | 4、5 |
| P13 显式 backend | 未构建 CUDA、初始化失败 | 明确失败；没有未配置的 CPU fallback | 3、4 |
| P14 并发负载 | 相机+SLAM+MPPI+推理 | 同一新鲜度/安全标准；完整时长和运动范围真实记录 | 5 |

阶段放行必须同时具备：协议与拥有权测试、ROS 旧结果拒绝、实际可回收子进程故障恢复、当前配置性能证据。真实 D-state 的“无法回收时隔离”由可控系统调用模型验证；它不能被包装成真实 GPU 设备恢复成功。整个 W1 仍需相机 ROI、遮挡、三维覆盖及其余未通过场景共同放行。

## 文档交付自检

- [x] 只新增本计划文档，没有改运行模块或启动验证。
- [x] 现状、计划、候选预算、未执行测试明确分开。
- [x] 保留 kernel D-state 隔离/禁止重启、完整返回耗时、显式 backend 和硬件重置排除边界。
- [x] IPC 的实例/代际/请求、容量、EOF、partial、重复响应和共享内存生命周期均有对应任务及场景。
- [ ] 实施、构建、离线测试、ROS 测试、CUDA 实测和导航仿真结果由后续执行者逐项填入；当前均未完成。
