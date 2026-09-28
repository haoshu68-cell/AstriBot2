# 自有 Gazebo 停止量测（P0 部分证据）

运行代码：`beab7bc1`；验证入口：`tools/validation/measure_arm_execution_stop.py`。
会话：`/tmp/astribot_arm_dynamic_recovery/simulation/session02`，ROS domain94，分区 `astribot_arm_recovery_p0_20260928_b`。旧 domain93/session01 为只读发现相机原点问题的会话。没有接管外部仿真。

唯一运行的被测控制器类型为 `astribot_s1_manipulation/OwnedTrajectoryController`，六个控制器均实际读回；MoveGroup 运动执行关闭。六摄像头、真实 VoxelSLAM 和导航 p2 支持进程开启，但没有发送导航目标。底盘 `idle_position_hold=true`、`idle_position_kp=3.0` 实际读回。负载为空。

本实验验证：左腕 joint7、±0.2 rad/4 s 三次轨迹，在实际移动 0.025 rad 后触发取消/停发心跳/显式撤销/执行所有者 SIGKILL，观察旧 Goal 终态、保持指令和连续关节+SLAM 停稳窗口。正反方向交替，四类各 30 轮，120/120 符合下述功能判据。每次运动前用 MoveGroup 对完整规划模型进行 41 个状态的碰撞预检。

验收限定：旧目标按路径进入 CANCELED 或 ABORTED；不继续到原终点；收到保持指令；新鲜关节和 map 下 SLAM 在至少 0.6 s 窗口满足候选停稳阈值。过程量测为候选边界的采集，不等于已经批准全臂安全速度/制动距离。采样净空不是连续空间证明；没有测夹持力、冲击、动态障碍或自动重规划。

SIGKILL 由真正持有 FJT Action 和心跳的子进程执行，其最终清理代码不会运行；另一个只读观察者记录退出码 -9、同一 Goal UUID 的 ABORTED 状态和实际停止。停发心跳则仍保留执行进程，两者分开报告。

Gazebo 后端版本为本机固定的 gz_ros2_control：位置误差 × 更新频率 × 0.1 写成速度命令，不能当作真机受限加速度/力矩制动模型。实际库路径、进程身份、哈希见 `loaded_libraries02.json`。MoveGroup 和仿真机器人模型的 123 个 link/joint 元素一致，见模型对照；不意味着这些模型的物理参数已经标定。

测量起止使用单调墙钟，同时保存 ROS 源时间戳；统计包括观测传输/采样误差。停止确认耗时包含 0.6 s 稳定窗口，不能称作纯物理停车时间。`world_stats_during_stops.jsonl` 是本次停止试验期间的一段世界统计窗口，不能代表每个试验的全程 RTF。

原始结果归档将包含单次探索与交错重复两组。各轮 JSON 保存目标 UUID、事件、关节/SLAM 样本。复现必须先建立自己的隔离会话并重新核验资源和环境；路径存在不是会话仍在运行的证明，不得直接向陈旧 domain94 发命令。

归档：`raw_trials_and_sessions.tar.gz`，逐文件大小/哈希见 `archive_contents.json`。压缩包已重新打开并逐文件校验。`summary.json` 含 P50/P95/P99/max，采用 nearest-rank 分位数；每组只有 30 个样本，P99 等于样本最大值，不是未来时延上界。

世界统计第一次查询了不匹配的 world topic，没有收到数据，手动结束该只读查询；空文件留在归档。查询实际 topic 列表并核对自有进程的 transport 环境后，从 `/world/default/stats` 收到 20 帧，正常完成。该空查询不作为暂停/故障证据。
