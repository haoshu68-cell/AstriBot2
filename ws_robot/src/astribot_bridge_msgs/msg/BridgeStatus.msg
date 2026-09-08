# 控制桥接层的运行状态。
#
# 为什么状态要用一个专门的话题、而不是只打日志
# ==========================================
# 日志是给人看的、会被 SDK 的 fd 级重定向吞掉、也没法被上层节点消费。
# 本项目已实测过 SDK 一 import 就 os.dup2 掉 fd 1/2（见 state_bridge_node
# 头部说明），那之后连"响亮失败"的 ERROR 都发不出来。所以桥接的每一种故障
# 都必须走这个话题，让诊断节点和录包能拿到结构化的证据。
#
# 约定：state 只允许取下面的枚举值，禁止裸数字。任何新增故障都要在这里加枚举，
# 不允许用 detail 字符串携带"新故障类型"——那样上层没法可靠地分支。

std_msgs/Header header

# 发布者节点名（同一个话题上有多个桥接节点在发，靠这个区分）
string node_name

# 当前状态，取值见下方常量
uint8 state

# 人类可读的补充说明。只用来携带"同一状态下的具体数值/上下文"，
# 不承载状态类型本身。
string detail

# 两个通用指标位。语义随 state 变化，各状态的含义见常量注释。
# 用固定两个字段而不是 map，是为了让 rosbag 里的曲线可以直接画。
float64 metric_1
float64 metric_2

# ---------------------------------------------------------------------------
# 状态枚举
# ---------------------------------------------------------------------------

# 正常工作
uint8 OK = 0

# 未使能。底盘桥接的默认启动态（start_disabled: true），不是故障。
uint8 NOT_ENABLED = 1

# SDK 会话不可用：wait_for_interface_alive() 失败，或后端消失。
uint8 SDK_NOT_ALIVE = 2

# 任何 SDK 调用抛了异常。detail 携带异常文本。
# 禁止吞异常——捕获后必须转成这个状态 + Action 错误码。
uint8 SDK_CALL_FAILED = 3

# 机器人模式不是 safe。metric_1 未用，detail 携带实际 mode 字符串。
uint8 ROBOT_MODE_UNEXPECTED = 4

# 参数非法已回退保守默认值。detail 说明哪个参数、回退成了什么。
uint8 CONFIG_FALLBACK = 5

# 控制循环周期超限。metric_1=实测周期 P99(s)，metric_2=目标周期(s)。
uint8 LOOP_OVERRUN = 6

# 内环步长被钳位（实测 dt 超过 max_tick_dt_sec，或时钟未前进）。
# metric_1=钳位前的实测步长(s)，metric_2=上限(s)。
#
# 为什么要单独一个枚举而不复用 LOOP_OVERRUN
# ----------------------------------------
# LOOP_OVERRUN 只说"这一拍慢了"，是**观测**；TICK_DT_CLAMPED 说的是
# "我改了积分步长"，是**动作**。步长直接乘在速度上，被钳位就意味着底盘这一拍
# 的位移被人为压小了 —— 静默钳位等于静默改变底盘速度，必须单独可见。
uint8 TICK_DT_CLAMPED = 7

# --- 底盘专用 ---

# /cmd_vel 看门狗触发，速度已置零。metric_1=距上一条指令的秒数。
uint8 CMD_VEL_TIMEOUT = 20

# leash 牵引绳触发：指令位置与 SDK 实际位置偏差超阈，积分与校正均已冻结。
# metric_1=xy 偏差(m)，metric_2=theta 偏差(rad)。
uint8 LEASH_TRIPPED = 21

# 位姿源不可用（TF 查不到），校正已冻结，退化为纯开环 + leash。
uint8 SLAM_UNAVAILABLE_OPEN_LOOP = 22

# 位姿源数据过龄。metric_1=龄期(s)，metric_2=阈值(s)。
uint8 SLAM_STALE = 23

# 检测到位姿跳变（重定位事件），已重新对齐期望位姿而非施加巨大校正。
# metric_1=跳变距离(m)，metric_2=阈值(m)。
uint8 SLAM_RELOCALIZED = 24

# 里程计漂移偏高：同一时间窗内"轮子以为走了多远"与"世界里实际走了多远"
# 的差超阈，是打滑的累积证据。metric_1=差值(m)，metric_2=窗口内实际位移(m)。
uint8 ODOM_DRIFT_HIGH = 25

# 闭环处于退化模式：pose_source=ground_truth 时 map->odom 是恒等静态 TF，
# 外环误差恒≈0，闭环不产生实际校正。
# !!! 这个状态位存在的唯一目的：防止把 ground_truth 下的漂亮数字
#     误当成"闭环有效性已验收" !!!
uint8 CORRECTION_DEGENERATE = 26

# 位姿源丢失且 require_slam_to_enable=true，已主动停车（速度置零）。
# metric_1=丢失时长(s)，metric_2=宽限阈值(s)。
uint8 SLAM_LOST_STOPPED = 27

# 位姿源实现**抛了异常**——这是端口契约违约，不是常规的"暂时查不到"。
# PosePort.lookup() 承诺查不到时返回 (None, None) 而不抛；真抛了说明位姿源
# 侧有缺陷（签名不匹配、TF 缓冲未初始化、依赖未就绪等）。detail 携带异常文本。
#
# !!! 为什么单独给一个枚举，而不是复用 SDK_CALL_FAILED !!!
# 位姿查询**不是 SDK 调用**。之前这里报的是 SDK_CALL_FAILED，运维看到会去查
# 机器人/SDK，而真实故障在 TF/位姿源 —— 本项目已经反复为这种"报错指向错误
# 子系统"付过代价。也不能直接折进 SLAM_UNAVAILABLE_OPEN_LOOP：那会把一个
# **真 bug** 伪装成正常的降级工况。
uint8 POSE_PORT_FAILED = 28

# /scan 龄期超过 scan_max_age_sec，本拍速度已置零（可自动恢复）。
# metric_1=龄期(s)，metric_2=阈值(s)。
#
# 与 SLAM_STALE(23) 并列而不合并：位姿陈旧丢的是"我在哪"，感知陈旧丢的是
# "周围有什么"。前者退化为开环+leash 还能继续走，后者继续走就是拿旧障碍图
# 开车，处置完全不同。
uint8 SCAN_STALE = 29

# /scan 持续陈旧超过 scan_loss_grace_sec，已闩锁停车（要人介入）。
# metric_1=持续陈旧时长(s)，metric_2=宽限阈值(s)。
# 与 SLAM_LOST_STOPPED(27) 同构：短暂异常自动恢复，持续异常必须有人看到。
uint8 SCAN_LOST_STOPPED = 30

# 从未收到过 /scan，速度置零。metric_1=已等待时长(s)，metric_2=阈值(s)。
#
# !!! 为什么与 SCAN_STALE 分开 !!!
# "收到过但变旧了"是上游故障（查雷达/驱动/转换节点）；"一帧都没收到过"几乎
# 总是话题名或 QoS 配错（查配置）。本项目已因 QoS 单向不兼容
# （BEST_EFFORT 发布 + RELIABLE 订阅，一帧都收不到、只有一条 WARNING）
# 浪费过一轮排查 —— 这个区分就是为那个场景留的。
uint8 SCAN_NEVER_RECEIVED = 31

# --- 机械臂专用 ---

# 目标越关节限位。detail 携带越限的关节序号与数值。
uint8 LIMIT_VIOLATION = 40

# SDK 运行时限位与 URDF 派生限位不一致，说明真值源没贯通。
uint8 LIMIT_SOURCE_MISMATCH = 41

# 跟踪误差超阈。metric_1=最大误差(rad)，metric_2=阈值(rad)。
uint8 TRACKING_ERROR_EXCEEDED = 42

# 发完最后一点后未在超时内收敛到目标容差。
# metric_1=残余误差(rad)，metric_2=容差(rad)。
uint8 SETTLE_TIMEOUT = 43

# --- 写通路准入（WriteGate）---

# 图上同时存在多个后端。D-1 统一 domain 下这会让指令同时打到两个后端。
# detail 列出发现的后端。
uint8 MULTIPLE_BACKENDS = 60

# launch 声明的 target 与实际后端不一致。
uint8 TARGET_MISMATCH = 61

# 目标是真机但未显式授权（allow_write_to_real）。
uint8 REAL_WRITE_NOT_AUTHORIZED = 62

# 位姿源声明与目标组合非法（例如 ground_truth + real）。
uint8 POSE_SOURCE_INVALID = 63
