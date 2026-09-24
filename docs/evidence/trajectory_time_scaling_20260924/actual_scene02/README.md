# scene02 时间伸长候选实采核验

本场确认了时间伸长候选被实际控制器消费、六个子动作成功终态及 Hold 观察；**整体协议未通过**。ACK 读取工具异常后租约过期进入隔离，六消费者 ACK、正常取消及资源释放未验证。本文仅离线解析已有记录，没有启动 ROS/Gazebo、重编或使用 GPU，未改原始 journal。

原始证据：[execution_response_scene02](/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02)。任务 `m1_task_c5a3bda3e01f4c5fbd29309a9cd032a7`，context `m1_context_a115642e1aac4045a6e183d0d6b283b6`，lease `f9191e67-043b-4a02-a2ad-cd5e56e18a4c_1`。

`manipulation_runtime_manifest.json` 中 MTC ELF SHA256 为 `9a9aeed9769f4fdaa33d5db374b6338c8030d1ccc9070e40e0e8cae78ebfce1f`，native 为 `c8bc8e9472c14e0ff355d83438e067b39e007496aee7c253a8cd345a2601f7c7`；当前文件重算均匹配。两个独立参数记录均为 `trajectory_time_scaling=2.5`、速度/加速度比例 `0.1/0.1`、关节 margin `0.1`。完整进程身份、库哈希、原始文件哈希在 [analysis.json](analysis.json)。

解码实际发出前保存的七份 CDR：PREGRASP 与左臂 Goal 的 JointTrajectory 逐字段完全相同，426 点、7.619381332 s；另五个控制器是单点保持 Goal，持续时间相同。`transport_skills.log:553` 的同场 IPTP 日志为伸长前 426 点、3.048 s、峰速 0.801 rad/s、峰加速度 4.089 rad/s²，四舍五入精度内与缩放后的 CDR 一致；没有保存该路径未缩放版本的完整 CDR，不能把舍入日志当作精确比例测量。

| 指标 | 结果 |
|---|---:|
| CDR 节点峰速 | 0.320496134634 rad/s |
| CDR 节点峰加速度 | 0.654258555251 rad/s² |
| JTC 实采期望峰速 | 0.320496184657 rad/s |
| JTC 实采期望峰加速度 | 1.419789318156 rad/s² |
| JTC 实采实际峰速 | 0.320546728269 rad/s |
| JTC 同消息 desired−actual 最大采样绝对误差 | 0.032049613342 rad |
| guard 原始状态最大采样关节误差 | 0.032049613342 rad |
| 末端样本 actual 对 CDR 终点最大误差 | 0.000001407776 rad |

JTC 指标来自 `/arm_left_controller/controller_state` 的执行期 346 条消息，源 ROS 时间 70.690–78.290 s、轨迹时间 0.010–7.610 s；源最大样本间隔 50 ms，接收 steady 最大间隔 90.940 ms。按源 stamp 减 desired.time_from_start 得到一致的动作起算 70.680 s。使用 CDR 的位置、速度和加速度作分段五次 Hermite 重算，和所有样本的 desired 差值上界分别为位置 `2.55e-15`、速度 `7.79e-13`、加速度 `2.62e-10`。逐关节样本保存在 [jtc_active_samples.csv](jtc_active_samples.csv)。重算核对的是这些采样点，不是连续轨迹碰撞证明。

峰误差发生于左臂 joint_5，JTC 源 ROS 72.980 s、轨迹时间 2.300 s；guard 在 72.981 s 引用 joint_stamp 72.980 s，记录相同误差。459 条同 context 的 active guard 样本包含最初 2 条 `WAITING_FOR_EXECUTION_EVIDENCE`，随后 457 条均为 `healthy=true / EXECUTION_WITHIN_BOUNDS`。这些样本的基座平移和旋转最大值分别为 `5.61612e-5 m`、`1.82558e-4 rad`。

`own_lease_journal.json[21:27]` 的六个 child_terminal 均 success=true、result_code=4，UUID 各自匹配 child_response；索引 27 为 hold_confirmed。`feedback_records[761]` 在 ROS 79.029 s 记录同一父 goal 的 Hold=true；executor 原始状态索引 1242–1279 共 38 条记录同租约的 Hold=true。末端 JTC 源 ROS 79.020 s 的期望位置等于 CDR 终点，实际最大速度 `1.56420e-5 rad/s`。其他五个控制器有完整 Goal 和终态证据，本份 first_stage 记录仅包含左臂 JTC 状态，未据此推断其他关节连续误差。

`first_stage/result.json` 明确 passed=false：`EnvelopeApplyStatus` 没有 `mode` 属性，acks=[]、cleanup_complete=false、hold_handle_unresolved=true。journal 索引 28–29 为 `quarantined / RESOURCE_LEASE_EXPIRED` 及 navigation_revoked。journal 的 issued_at/valid_until 是租约字段，不是每个事件的发生时间，不能据其标记精确隔离时刻。外层进程清理记录为 true，25 条所属进程身份检查均已停止；这不替代业务取消或资源释放。

本场 426 点路径不同于旧场 717 点路径，不能视为同轨迹的精确 A/B，也不能唯一归因旧场失败。采样最大误差不能当作全连续时间上界。原跟随问题的 12:15 起点、13:15 检查点保持不变，没有因本次离线分析重置。

复现仅需在 `/opt/ros/humble/setup.bash` 提供消息类型后运行本目录 [analyze.py](analyze.py)；脚本不调用 rclpy.init，不创建节点，仅读取旧文件并写本目录的派生 JSON/CSV。本次生成物哈希见 `SHA256SUMS`。
