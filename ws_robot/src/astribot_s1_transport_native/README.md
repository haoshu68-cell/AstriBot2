# C++ 搬运保持授权核心

底盘监测统一读取 `/slam/pose`（`PoseWithCovarianceStamped`，`map` 中的底盘位姿）。PICK/PLACE 执行过程中不再以底盘位姿变化取消任务；MTC guard 只检查关节跟踪误差。初始操作准入与导航交接 helper 共用严格递增采样戳的 SLAM 停稳尾窗，实际接收跨度至少 0.6 秒。相邻 Δx、Δy 和 wrapped Δyaw 按正负累加，每个样本的净位移不得超过 0.005 m、净航向变化不得超过 0.01 rad；不累计绝对增量，曾越界后返回也不能确认该窗口静止。导航交接仍需匹配动作终态及零命令。监测不读取 `/odom`、twist 或底盘 TF，也不计算瞬时差分速度；没有 SLAM 位姿不能确认停稳。

## 实际附件与剩余轨迹交接

完整 `trajectory_executor` 在物理 ATTACH/DETACH 后调用 `RevalidatePayloadTransition`，接收从索引 4 开始的完整两阶段后缀。仅 PICK 的 TRANSPORT_POSTURE 允许按实际附件重新规划；LIFT 和其他未重规划路径必须逐字保持原轨迹及时间。候选在独立完整 Scene 再读回、最新附件账本、停稳、几何与资源确认后原子接纳，之后才确认物理阶段并推进。10 秒重验、30 秒附件事务及原 120 秒计划寿命不延长；取消或接纳失败不得继续执行旧后缀。

资源 journal 的 `payload_suffix_adopted` 保存服务返回和执行器接纳轨迹的 SHA256；`child_trajectory_submission` 从最终实际 FJT Goal 取指纹，`child_response` 将同一不可变元数据绑定到子目标 UUID。仅活动控制器带这些轨迹字段，其余保持控制器沿用原记录。指纹用于核对交接证据，不代表控制器已执行；仍须同 UUID 成功终态与实测阶段完成。完整导航仿真验收状态以当次证据为准。

`arm_hold_core` 是保持授权和资源事务核心。`hold_executor` 将它接到仿真的实际控制器，只接受显式“保持当前实测姿态”任务，不接收任意目标关节位置。运行时使用 C++；Python 只保留旧入口的恢复兼容检查和验证脚本。

## 实际执行数据流

`HoldResources(task_id, request_id)` → 同一主机/domain 排他锁与持久化资源记录 → 6 个真实 `FollowJointTrajectory` 子目标 → 每个目标 UUID 对应的成功终态 → 新鲜完整几何和控制器 claims → 双时间 500 ms 稳定窗口 → `/navigation/arm_hold`。

`/transport/hold_executor/renew` 接收 lease、resource epoch、严格递增序列。资源租约为 2 s，保持消息取几何/控制器/资源剩余期的最小值且不超过 300 ms。资源所有者仍须续约；仅保持消息持续发布不能替代任务所有权。

取消、失效、外部控制客户端出现或子目标失败先撤销保持，再由调度定时器取消已知子目标。未返回接受应答的目标保持 pending，迟到接受后补发取消。所有子目标达到可证终态之后重新采集至少 500 ms 稳定窗口，落盘结果和资源交接后才返回 `resources_released=true`。UNKNOWN、丢失结果和取消 ACK 都不是终态证据；10 s 内无法确认时返回未释放并隔离资源。Humble 在结果回调中持有目标锁，因此回调内禁止重入目标状态查询/取消。

## 使用和边界

必须在已确认归属的隔离导航仿真中同时设置 `use_sim_time=true` 和 `simulation_commissioning=true`。当前固定覆盖双臂、双夹爪、头、躯干的 22 个关节、6 个 JTC；其他拓扑会拒绝准入。使用同一 domain 时，旧运输任务和本节点共用 `/tmp/astribot_transport_domain_<domain>.lock`。持久化文件位于 `~/.local/state/astribot/transport/domain_<domain>.jsonl`，不能通过指定另一输出目录绕过未决事务。域名必须为规范十进制，拒绝八进制/前导零歧义。

规划用 MoveIt 须设置 `allow_trajectory_execution=false`；项目 launch 会同时移除控制器客户端配置。Humble 的 `No controller_names specified` 日志在这种模式下是预期的零控制器配置。禁止同时运行旧 Python `fixed_v2` 保持发布路径，该入口已显式拒绝。

这是已登记写入者之间的合作式资源所有权。节点检查隐藏 send_goal 服务客户端、同名节点、直接轨迹发布者、保持发布者和控制器 claims，并在不确定时拒绝。ROS graph 检查存在发现延迟，不能视为 DDS 身份鉴权或厂家执行端 epoch 栅栏；硬件和不合作写入者隔离仍需独立验收。本节点不发送底盘目标，也不会自动调用 `SetFixedEnvelope` 或替六消费者生成 ACK。

共享锁文件不替换 inode。新旧实现先同步完整保护标记，再缩短文件；native 日志和 legacy 未决标记相互核验。重启未决事务只进入 `RESOURCE_RECOVERY_REQUIRED`，不重放动作、不自动清除文件。当前未提供自动人工核销接口。

## 调用顺序

1. 正式资源所有者提供 `ResourceGrant`：任务、资源租约、资源 epoch、所拥有的关节集合、签发/失效时间及本地接收双时间。
2. 执行器完成一次归属于该任务的保持/姿态动作，提供匹配的 `HoldCompletion`。必须成功且终态，完成证据不能超过 500 ms。
3. 调用 `begin()`，使用本轮唯一 hold ID；持续传入完整几何 `geometry()`、实际控制器查询响应 `controllers()` 和连续租约 `resource()`。
4. 用 `status()` 取得有界 `ArmHoldStatus`，用 `request()` 生成与同一 hold 和当前几何序列绑定的 `SetFixedEnvelope::Request`。保持发布已由本包执行器接线；六消费者协调服务的调用仍属于后续 A3。
5. 协调器还需核对停止证据、几何版本、载荷质量、速度/制动限制，以及六个消费者的同一版本 ACK；库自身不放行导航。
6. 取消、资源丢失、已确认状态下的控制器失效、版本变化或位置漂移会撤销保持；迟到反馈不能恢复旧 hold。重新执行必须使用新的 ID 和新的正式完成证据。

## 核心边界

稳定观察要求至少三次不同采集，在源时间和单调时间上均覆盖 500 ms；相邻样本间隔最多 300 ms，位置波动不得超过几何声明误差的四分之一。逐关节误差上限 0.025 rad，采集偏差最多 100 ms，必须拥有全部关节资源。

几何序列不能为零，同一序列不能表示不同采集或内容。重复采集不能增加稳定样本或续约，只允许收紧原期限。资源续约不能跨越旧租约过期的间隙，包括仍在等待稳定的阶段。

控制器状态来自 `ListControllers` 完整 active `JointTrajectoryController` position claims；重复占用或缺失关节不得确认。首次控制器响应晚于稳定几何时保持等待，收到新鲜有效响应后可完成初始化；已确认之后的失效则锁存撤销。

确认状态有效期取几何、控制器、任务资源的 ROS/单调时间剩余量，且最多 300 ms。稳定样本缓存最多 128，进程内保留最多 64 个已用 hold ID，耗尽时拒绝新建，避免无界增长或 ID 复用。

本库的输入结构不能证明调用方确实持有资源。后续正式任务适配必须将资源服务器和执行器的实际结果原样接入；不应通过启动参数、固定 owner 字符串或离线测试数据冒充在线所有权。

离线用例与现有 `FixedEnvelopeCore` 流程测试见 `docs/PAYLOAD_STATE_OFFLINE_20260923.md`。本轮真实保持及隔离 Action 协议测试记录在 `docs/evidence/joint_acceptance_20260923/a2/`；接口夹具、导航仿真和整机动态验收是不同证据层。
