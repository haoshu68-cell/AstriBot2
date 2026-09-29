# 机械臂状态到导航上游限速

当前运行节点保留名称 `arm_chassis_speed_coupling_node`，职责已变更：只读取完整双臂关节状态与臂展 TF，向 `/navigation_policy/arm_speed_limit` 发布 `nav2_msgs/msg/SpeedLimit`。不订阅或发布 Twist，不拥有 `/cmd_vel`。

该专用接口 `percentage=true`，`speed_limit` 范围 0–100；**0 表示 HOLD**，由导航约束节点转为 MotionConstraint，不能接入原生 `/speed_limit`（该原生接口把 0 解释为取消限速）。header.frame_id 为 chassis_base_frame，header.stamp 为关节采集时刻。

保留臂展/关节速度计算活动度及平滑比例。完整 14 关节的 position 和 velocity 必须有限，采样戳只用于同源选新，不因领先本地时钟或年龄置 0；旧帧与重复帧忽略。缺失关节或非法数值的新样本明确置 0，旧正常帧不能恢复它。50 ms 定时复发保留原采集时间。TF 使用最新已有变换，查询节流使用 steady 时间。TF 缺失沿用 full-reach 活动度，并立即输出 min_speed_scale；这只是已存在的启发式限速，不构成载荷稳定性的物理验证。

旧版末端 Twist 桥接包已逐文件哈希归档到 `docs/evidence/mainline_20260924/arm_navigation_speed_limit/before.tar.gz`。历史 `test_ros.py` 与 `benchmark_ros.py` 已核验归档 SHA 后删除，删除清单见同目录 `retired_entrypoints.json`；当前纯算法差分所需 `test/reference` 保留。

当前验证入口：

- `dynamics_core_differential`：保留算法差分。
- `arm_navigation_limit_core`：完整关节和数值边界。
- `arm_navigation_limit_boundary`：源码及启动接口边界。
- `arm_navigation_limit_ros`：单进程有界协议验证。由会话所有者显式传入相同的 `DYNAMICS_DOMAIN`、`ROS_DOMAIN_ID`，并设置 `ROS_LOCALHOST_ONLY=1`；可用 `DYNAMICS_EVIDENCE` 指定证据目录。

2026-09-24 实际证据：统一构建及安装通过；算法差分、源码边界检查通过；新 C++ 边界测试原先失败于浮点精确相等，修为 1 ULP 数值边界后单项复跑通过，原失败保留。最终节点在隔离 ROS domain 36 的上游协议测试 1/1 通过：无 Twist 发布/订阅，正常百分比输出，同采集时间不续期，缺失关节/速度字段输出 0，新有效样本恢复；测试进程和节点均退出 0，归属进程无残留。

原始日志、XML、进程与消息记录已复制并校验至 `docs/evidence/mainline_20260924/arm_navigation_speed_limit/verified/`。这证明本包构建、核心与隔离 ROS 接口；尚不代表本轮整场 Gazebo 搬运或真机验收。
