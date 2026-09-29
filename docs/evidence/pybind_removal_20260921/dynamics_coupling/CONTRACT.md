# 冻结接口与行为：2026-09-21

盘查基于当前共享工作树；不代表任何 Git commit 全树快照。原 Python 三文件按
`baseline_manifest.json` SHA256 原样复制到包内 `test/reference`。对照环境强制
`ASTRIBOT_BRIDGE_NATIVE_KERNELS=0`。生产改用 ament_cmake 同名 ELF executable
`astribot_s1_dynamics_coupling/arm_chassis_speed_coupling_node`；launch 入口、参数
YAML、respawn 配置保持原样，无生产 Python 包、console 或兼容转发入口。

## 消息、拓扑、时钟

- 输入 `geometry_msgs/Twist` 默认 `/cmd_vel_pre_arm_coupling`、
  `sensor_msgs/JointState` 默认 `/joint_states`；输出 Twist 默认 `/cmd_vel`。
  名称参数和 ROS remap 保留，KeepLast(10)、reliable、volatile。
- 每条 Twist 产生一条输出，只有 linear.x/y 和 angular.z 乘以 scale；其余分量置零。
  不新增 timer、watchdog 输出、控制器所有权或坐标变换。
- 默认水平伸展模式：latest TF，从 chassis_base_frame 到全部 monitored_links。
  任一路缺失/陈旧/未来或 xyz 非有限，或监控列表空，展开维度 activity=1。
  TF timestamp=0 是静态链 timeless；非零时 `0 <= ROS_now-stamp <= timeout`。
- reach 缓存从最后查询时刻计时，`0 <= delta < period` 时复用；等于边界或时钟回退重查。
  查询失败也缓存失败直到下一次允许查询。Humble Python Buffer 无 ROS-clock jump
  清空回调；C++ Buffer 使用独立 system clock，维持相同节点 ROS 时钟回退行为。
- 关节新鲜度使用接收回调时的 ROS clock，忽略消息 header 时间。无任何关节输入或
  `age > joint_state_timeout_sec` 使用 degraded_scale；等于边界和负 age 都仍 fresh。
- 陈旧关节 warning 的2秒限流使用 system clock；活动量 info 的 log_throttle_sec
  则使用 ROS clock（暂停/回退时日志节流行为不同），与原节点各自保持一致。
- 无迟滞：EMA `alpha*raw + (1-alpha)*previous`，初值1，在关节回调中更新。

## 参数读取

构造时绑定：input_topic、output_topic、joint_states_topic、extension_metric。
在线设置这些参数可成功但既有 subscription/publisher/metric 不改变，要求重启生效。

每次使用时读取：chassis_base_frame、monitored_links、reach_tf_timeout_sec、
reach_folded_m、reach_full_m、reach_update_period_sec、folded_reference_rad、
extension_full_rad、velocity_full_rad_s、min_speed_scale、scale_smoothing_alpha、
joint_state_timeout_sec、degraded_scale、log_throttle_sec。参考/frames/links 更新的
可见时刻仍受已有 reach cache 约束。use_sim_time 使用 ROS 默认动态时钟支持。

浮点和浮点数组参数保持 ROS 静态类型。Humble Python 和 C++ 均在声明/原子更新阶段
拒绝 integer、bool、string 替代 double，或 integer array 替代 double array；不会因
源码里 float(...) 而接受这些 override。错误类型原子更新整批拒绝。

启动 reach_tf_timeout_sec 非有限或<=0：非零退出。非法 metric 字符串，或 startup
reach_folded_m<0 / full<=folded：报告错误并退回 joint_deviation，保持原策略。

## 核心与异常契约

- 水平 reach=sqrt(x*x+y*y)，不使用 z；但 TF xyz 都必须有限。
- activity=max(extension, velocity)，raw=max(minimum,min(1,1-activity*(1-minimum)))。
- legacy joint metric 使用原始绝对角差，无角度 wrap；与参考列表 zip 截断。
  missing/unknown joint 跳过，空列表 activity=0；重复 name 最后值生效。
- velocity 数量不等于 name 数量时整体丢弃速度维度；否则也按字典最后值处理。
  extension/velocity 满量程<=1e-6 时对应 activity=0。
- 动态 reach interval 非法引发该次 joint 计算异常，保留旧 scale 与 freshness。
- effective-scale 计算异常本帧 scale=1；publish 缩放消息异常尝试原消息转发。
  异常注入到 rclcpp middleware 发布器未测试；不可宣称进程崩溃无中断。

## 原实现问题：本次未悄改策略

这些是保留的原行为，不是新安全保证：空 legacy joint 消息可能使 scale=1；负 joint
age 会视为 fresh；缺少参数数值范围/有限性检查，NaN或越界 alpha/minimum/degraded
能造成不合理输出；NaN/Inf Twist 可透传；一般 scale 异常可放行1。非有限 joint
position/velocity 在 Python min/max 参数顺序下饱和 activity=1，C++明确保持该行为。

无效控制参数应作为独立安全契约变更设计与验证，不能把本次等价迁移证据解释为
对任意输入的 fail-closed 证明。更早的 README 中“任何异常均不限速”并不精确：joint
计算异常实际保留旧状态，TF缺失实际按满活动量保护。本文件以源码和实测为准。
