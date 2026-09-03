// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/footprint.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
// tf2::getYaw(geometry_msgs Quaternion) 的 fromMsg 特化在 tf2_geometry_msgs 里，
// 只 include tf2/utils.h 能编过但链接期报 undefined reference。
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace astribot_s1_autonomy
{

namespace
{
constexpr int kLogThrottleMs = 3000;

/// 角度归一化到 (-pi, pi]，用于朝向偏差判定。
double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}

/// 状态转换白名单。需求禁止「乱序跳转」，所以这里是白名单而非黑名单：
/// 没在表里出现的组合一律非法，非法即回退安全态，不存在「顺手放过」的可能。
bool isTransitionAllowed(ExplorationState from, ExplorationState to)
{
  if (from == to) {
    return true;                              // 自转换视为空操作
  }
  if (to == ExplorationState::kIdle || to == ExplorationState::kPaused) {
    return true;                              // 安全态/异常态是任何状态的合法出口
  }
  switch (from) {
    case ExplorationState::kIdle:
      // 冷启动自举只能从 IDLE / GEN_NEXT_POINT 进（这两处都保证没有在途目标）。
      return to == ExplorationState::kGenNextPoint || to == ExplorationState::kCompleted ||
             to == ExplorationState::kBootstrap;
    case ExplorationState::kBootstrap:
      // 自举结束（正常转完、被安全门拦下、或超时）**一律回 IDLE**，
      // 由 IDLE 重新走「地图/定位/里程计是否就绪」这套前置条件，不许直接跳去选点。
      return to == ExplorationState::kIdle;
    case ExplorationState::kEscape:
      // 脱困结束一律回 GEN_NEXT_POINT 重新走完整校验流程 ——
      // 不许直接跳去 VALIDATING：出带之后地图/候选都该重新算一遍。
      // 失败路径（超时/无解/红线）走 kPaused，由上面那条统一出口覆盖。
      return to == ExplorationState::kGenNextPoint;
    case ExplorationState::kGenNextPoint:
      return to == ExplorationState::kValidating || to == ExplorationState::kCompleted ||
             to == ExplorationState::kBootstrap;
    case ExplorationState::kValidating:
      // 候选全被拒 → 回 kGenNextPoint 重采样；校验通过 → kNavigating。
      // 起点落在膨胀带（≠ 找不到合法点）→ kEscape，这两者的正确响应完全相反。
      return to == ExplorationState::kNavigating || to == ExplorationState::kGenNextPoint ||
             to == ExplorationState::kEscape;
    case ExplorationState::kNavigating:
      // 只有 Nav2 报成功才进 kArrived；失败走 registerNavFailure → kGenNextPoint/kPaused。
      return to == ExplorationState::kArrived || to == ExplorationState::kGenNextPoint;
    case ExplorationState::kArrived:
      return to == ExplorationState::kGenNextPoint || to == ExplorationState::kCompleted;
    case ExplorationState::kPaused:
      return to == ExplorationState::kGenNextPoint || to == ExplorationState::kCompleted;
    case ExplorationState::kCompleted:
      return to == ExplorationState::kGenNextPoint;   // 人工 resume 后可重新探索
  }
  return false;
}
}  // namespace

ExplorationCoordinatorNode::ExplorationCoordinatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("exploration_coordinator_node", options),
  // 每一个 rclcpp::Time 成员都必须显式指定 RCL_ROS_TIME：默认构造是 RCL_SYSTEM_TIME，
  // 与 now() 相减会直接抛 "can't subtract times with different time sources"。
  // 顺序必须与头文件里的声明顺序一致，否则 -Wreorder 告警。
  state_entered_time_(0, 0, RCL_ROS_TIME),
  active_path_time_(0, 0, RCL_ROS_TIME),
  last_replan_check_time_(0, 0, RCL_ROS_TIME),
  last_replan_time_(0, 0, RCL_ROS_TIME),
  nav_started_time_(0, 0, RCL_ROS_TIME),
  plan_requested_time_(0, 0, RCL_ROS_TIME),
  dwell_started_time_(0, 0, RCL_ROS_TIME),
  latest_map_time_(0, 0, RCL_ROS_TIME),
  latest_costmap_time_(0, 0, RCL_ROS_TIME),
  latest_odom_time_(0, 0, RCL_ROS_TIME),
  bootstrap_started_time_(0, 0, RCL_ROS_TIME),
  bootstrap_stall_since_(0, 0, RCL_ROS_TIME),
  escape_started_time_(0, 0, RCL_ROS_TIME),
  last_breadcrumb_time_(0, 0, RCL_ROS_TIME),
  latest_scan_time_(0, 0, RCL_ROS_TIME)
{
  declareParameters();

  std::string error;
  if (!loadParameters(error)) {
    // 参数非法时不崩溃、也不用默认值蒙着跑：停在 kIdle 空转，等参数被改对。
    RCLCPP_ERROR(get_logger(), "参数校验失败，协调器将停在 IDLE 空转: %s", error.c_str());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // 带 timeout 的 lookupTransform 从非执行器线程调用时必须让 listener 自带 spin 线程，
  // 否则动态 TF 恒超时（静态 TF 却正常，因此现象上很像「一切正常」）。
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_buffer_->setUsingDedicatedThread(true);

  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(1));
  complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    complete_topic_, rclcpp::QoS(1).transient_local());
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    current_goal_topic_, rclcpp::QoS(1));

  // slam_toolbox 的 /map 是 transient_local + reliable，订阅端必须匹配，
  // 否则会出现「话题存在但一直收不到地图」。
  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = io_cb_group_;
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {mapCallback(msg);},
    sub_opts);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {odomCallback(msg);},
    sub_opts);

  // 全局代价地图。只在启用「用 costmap 校验」时才订阅——关掉时不订阅，
  // 免得白占一份带宽和一次 O(w*h) 的转换开销。
  //
  // QoS 用默认的 reliable/volatile：costmap_raw 由 nav2_costmap_2d 以
  // update_frequency(本项目 1.0Hz) 周期发布，是持续更新的活数据，
  // 不像 /map 那样需要 transient_local 补发历史。
  if (use_costmap_for_validation_) {
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::QoS(1),
      [this](const nav2_msgs::msg::Costmap::ConstSharedPtr msg) {costmapCallback(msg);},
      sub_opts);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "validation.use_costmap=false：下发前校验将使用 /map 而不是全局代价地图。"
      "这是已知会让探索一个目标都发不出去的配置(两层判据查不同的图)，"
      "仅供对比排查，正常运行请置 true");
  }

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, nav_action_name_, io_cb_group_);
  plan_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this, plan_action_name_, io_cb_group_);
  follow_client_ = rclcpp_action::create_client<FollowPath>(
    this, follow_action_name_, io_cb_group_);
  last_replan_time_ = now();
  last_replan_check_time_ = now();
  active_path_time_ = now();
  bootstrap_stall_since_ = now();
  bootstrap_started_time_ = now();

  // ---- 冷启动自举的发布/订阅/定时器 ----
  // 关闭自举时一个句柄都不建：既省资源，也让「谁能发 cmd_vel」这件事在
  // 配置层面就是确定的（disabled 时本节点物理上没有 cmd_vel 发布者）。
  if (bootstrap_mode_ != BootstrapMode::kDisabled) {
    bootstrap_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      bootstrap_cmd_vel_topic_, rclcpp::QoS(1));
    // !!! 必须 BEST_EFFORT !!! 点云切片出来的 scan 是 SensorDataQoS 发布的，
    // 用 RELIABLE 订阅一帧都收不到；而「收不到」在安全门里等价于「拒绝运动」，
    // 表现就是自举永远被拦下、却看不出为什么。
    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = io_cb_group_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      bootstrap_scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {scanCallback(msg);},
      scan_opts);
    const auto cmd_period = std::chrono::duration<double>(1.0 / bootstrap_cmd_rate_hz_);
    bootstrap_cmd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(cmd_period),
      [this]() {bootstrapCmdTick();}, timer_cb_group_);
  }

  pause_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/pause",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onPauseService(req, res);},
    rmw_qos_profile_services_default, io_cb_group_);
  resume_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/resume",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onResumeService(req, res);},
    rmw_qos_profile_services_default, io_cb_group_);

  state_entered_time_ = now();
  const auto period = std::chrono::duration<double>(
    control_period_sec_ > 0.0 ? control_period_sec_ : 0.5);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {controlTick();}, timer_cb_group_);

  setupFootprintWatchdog();

  publishComplete(false);

  RCLCPP_INFO(
    get_logger(),
    "探索协调器已启动: 地图=%s 里程计=%s 导航动作=%s 规划动作=%s 节拍=%.2fs",
    map_topic_.c_str(), odom_topic_.c_str(), nav_action_name_.c_str(),
    plan_action_name_.c_str(), control_period_sec_);
  RCLCPP_INFO(
    get_logger(),
    "抵达判定: xy<=%.2fm yaw<=%.2frad(启用=%s) 驻留>=%.2fs 速度<=%.3fm/s",
    arrival_xy_tolerance_, arrival_yaw_tolerance_, check_yaw_ ? "是" : "否",
    dwell_time_sec_, settle_speed_);
  // 注意不要写成 (std::to_string(x) + "s").c_str() 三目 —— 那是个悬垂临时量。
  const std::string age_desc =
    path_max_age_sec_ > 0.0 ? (std::to_string(path_max_age_sec_) + "s") : std::string("不限");
  RCLCPP_INFO(
    get_logger(),
    "跟踪期重规划: 策略=%s 检查节拍=%.2fs 最小间隔=%.2fs 偏离阈值=%.2fm 路径寿命=%s",
    replan_policy_ == ReplanPolicy::kOnInvalid ? "on_invalid(未失效不换路径)" : "periodic(回退)",
    replan_check_period_sec_, replan_min_interval_sec_, path_deviation_limit_m_,
    age_desc.c_str());
  if (bootstrap_mode_ == BootstrapMode::kDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "冷启动自举已关闭(bootstrap_mode=disabled)："
      "若 SLAM 冷启动时地图为空，需人工发 cmd_vel 推一把才能开始探索");
  } else {
    RCLCPP_INFO(
      get_logger(),
      "冷启动自举: 原地旋转 %.2frad/s × %.1fs，触发等待 %.1fs，上限 %d 次，"
      "速度发到 %s(经全部下游限速与安全层)，安全门=激光 %s 新鲜且最近障碍 >=%.2fm",
      bootstrap_angular_vel_, bootstrap_duration_sec_, bootstrap_trigger_wait_sec_,
      bootstrap_max_attempts_, bootstrap_cmd_vel_topic_.c_str(),
      bootstrap_scan_topic_.c_str(), bootstrap_min_clearance_m_);
  if (escape_enabled_) {
    RCLCPP_INFO(
      get_logger(),
      "膨胀带脱困: 触发=连续%d次起点致命 搜索半径=%.2fm 方向约束=±%.2frad "
      "限速=%.2fm/s %.2frad/s 超时=%.1fs 上限=%d次 出带连击=%d拍 "
      "来路窗口=%.0fs@%.1fHz。红线=/map 判占据或未知即停机、禁止脱困",
      escape_trigger_failures_, escape_search_radius_m_, escape_heading_tol_rad_,
      escape_linear_vel_, escape_angular_vel_, escape_timeout_sec_,
      escape_max_attempts_, escape_clear_ticks_,
      breadcrumb_window_sec_, breadcrumb_sample_hz_);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "膨胀带脱困已禁用(escape_enabled=false): 起点落在膨胀带时只能等人工 ~/resume");
  }
  }
}

ExplorationCoordinatorNode::~ExplorationCoordinatorNode()
{
  // 析构时把在途目标撤掉，避免节点没了、Nav2 还在往一个没人监管的目标开。
  if (control_timer_) {
    control_timer_->cancel();
  }
  // 自举定时器先停，再发零速：反过来的话刚发的零速可能被定时器的下一帧覆盖。
  if (bootstrap_cmd_timer_) {
    bootstrap_cmd_timer_->cancel();
  }
  if (state_ == ExplorationState::kBootstrap) {
    publishBootstrapCmd(true);
    RCLCPP_WARN(get_logger(), "协调器在自举过程中退出，已发零速停车");
  }
  if (nav_client_ && nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
  }
  // follow_path 模式下在途的是 FollowPath，同样要撤 —— 漏掉它会让节点退出后
  // controller_server 继续沿着一条没人监管的路径开。
  if (follow_client_ && follow_goal_handle_) {
    follow_client_->async_cancel_goal(follow_goal_handle_);
  }
  RCLCPP_INFO(
    get_logger(),
    "协调器退出: 下发=%" PRIu64 " 成功=%" PRIu64 " 候选被拒=%" PRIu64,
    goals_dispatched_, goals_succeeded_, candidates_rejected_);
}

// ============================ 参数 ============================
// 需求硬性要求：禁止硬编码距离阈值/判定参数，全部外置可配置。
// 因此这里每一个数字都只作为「declare 的缺省值」存在，运行值一律来自 YAML。

void ExplorationCoordinatorNode::declareParameters()
{
  auto describe = [](const std::string & text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      return d;
    };

  declare_parameter<std::string>("map_topic", "/map", describe("输入占据栅格话题(仅用于前沿搜索)"));
  declare_parameter<std::string>(
    "costmap_topic", "/global_costmap/costmap_raw",
    describe("全局代价地图话题(nav2_msgs/Costmap，仅用于下发前校验)"));
  declare_parameter<std::string>("odom_topic", "/odom", describe("里程计话题，用于速度收敛判定"));
  declare_parameter<std::string>(
    "state_topic", "/exploration/state", describe("状态机状态话题(String)"));
  // ---- 足迹锁存看门狗 ----
  // 控制器可以在窄通道里临时缩小代价地图的足迹；设它的进程一旦死掉/卡住，
  // 代价地图会一直按小足迹算，机器人从此被系统性低估而日志毫无异常。
  // 本看门狗按"续租"判活：写话题上持续有非默认足迹请求 = 设它的人还活着。
  declare_parameter<bool>(
    "footprint_watchdog.enabled", false,
    describe("是否启用足迹锁存看门狗。控制器开了 narrow_square_enabled 时必须一起开"));
  declare_parameter<std::string>(
    "footprint_watchdog.write_topic", "/global_costmap/footprint",
    describe("代价地图的足迹**写入**话题(Polygon)。既订阅它判续租，也用它复原"));
  declare_parameter<std::string>(
    "footprint_watchdog.readback_topic", "/global_costmap/published_footprint",
    describe("代价地图的足迹**回读**话题(PolygonStamped)，判当前到底是什么足迹"));
  declare_parameter<std::string>(
    "footprint_watchdog.default_footprint",
    "[[0.42, 0.0], [0.297, 0.297], [0.0, 0.42], [-0.297, 0.297], "
    "[-0.42, 0.0], [-0.297, -0.297], [0.0, -0.42], [0.297, -0.297]]",
    describe("默认(大)足迹。必须与控制器的 narrow_footprint_default 一致"));
  declare_parameter<double>(
    "footprint_watchdog.lease_timeout_sec", 4.0,
    describe("续租超时(s)。回读到非默认足迹且这么久没有新请求 ⇒ 强制复原。"
      "必须大于控制器的 narrow_square_lease_period，也必须 >= "
      "readback_period_sec*(consecutive_reads+1)，否则会在读数还凑不齐时就动手"));
  declare_parameter<double>(
    "footprint_watchdog.readback_period_sec", 1.0,
    describe("回读话题的周期(s)，必须与 global costmap 的 publish_frequency 一致。"
      "只用于启动守卫的算术，不参与运行期判定"));
  declare_parameter<double>(
    "footprint_watchdog.readback_stale_sec", 3.0,
    describe("回读龄期上限(s)。最新回读比这还老 ⇒ 读数不是当前值，看门狗**不动手**"
      "并计数上报（不能拿冻结的读数当当前状态）"));
  declare_parameter<int>(
    "footprint_watchdog.consecutive_reads", 3,
    describe("需要连续多少次**回读**都是非默认足迹才允许动手。"
      "在回读回调里数，不在 tick 里数 —— 后者会把同一个 1Hz 采样重复计两次"));
  declare_parameter<std::string>(
    "complete_topic", "/exploration/complete", describe("探索完成标志话题(Bool)"));
  declare_parameter<std::string>(
    "current_goal_topic", "/exploration/current_goal",
    describe("当前已下发目标话题，仅供可视化"));
  declare_parameter<std::string>(
    "nav_action_name", "navigate_to_pose", describe("Nav2 导航动作名"));
  declare_parameter<std::string>(
    "nav_behavior_tree", "",
    describe(
      "下发目标时指定的行为树 xml 绝对路径。留空=用 bt_navigator 的默认树。"
      "探索场景填 astribot_s1_navigation 的 navigate_to_pose_explore_three_phase.xml，"
      "它把 FollowPath 的 controller_id 指向三段式控制器(终点不转朝向)"));
  declare_parameter<std::string>(
    "plan_action_name", "compute_path_to_pose", describe("Nav2 全局规划动作名(仅用于校验)"));
  // ---- 下发方式（需求1：复用已校验路径）----
  declare_parameter<std::string>(
    "nav_dispatch_mode", "follow_path",
    describe(
      "下发方式: follow_path=把已校验路径直接交给控制器跟踪(跟踪的就是被校验过的那条，"
      "省一次重规划，但 BT 的重规划与恢复行为拿不到，须靠 replan_period_sec 自己补)；"
      "navigate_to_pose=只发目标点，由 BT 内部重新规划(保留 BT 全部能力)，一键回退用"));
  // ---- 膨胀带脱困（ESCAPE）----
  //
  // 实测依据（2026-08-31 一次 3h41m 运行）：planner_server 报
  // "Starting point in lethal space!" 25652 次，14 次到位全部集中在开头 2.5 分钟。
  // 成因：SmacPlanner2D 判起点用单个中心格 cost>=INSCRIBED(253)，而 253 是
  // 膨胀层写的；机器人中心一进这条带，去任何目标都被拒。而 follow_path 模式
  // 绕过 BT，clear_costmap/backup/spin 全拿不到，自动恢复只是状态复位。
  declare_parameter<bool>(
    "escape_enabled", true,
    describe("是否启用膨胀带脱困。false=回退到改动前行为(卡住只能等人工)"));
  declare_parameter<int>(
    "escape_trigger_failures", 5,
    describe("连续多少次「起点致命」才触发脱困。避免偶发抖动就让机器人动起来"));
  declare_parameter<double>(
    "escape_search_radius_m", 1.5,
    describe("脱困目标搜索半径(m)。脱困是短距离动作，不是重规划"));
  declare_parameter<double>(
    "escape_heading_tol_rad", 1.5708,
    describe("方向约束：候选目标方位与参考朝向的最大夹角(rad)。默认 ±90°，"
             "拒绝背向任务方向逃逸；趟1无解时会放开并告警"));
  declare_parameter<double>(
    "escape_linear_vel", 0.08,
    describe("脱困线速度上限(m/s)。实测本底盘 0.02m/s 就能动(无静摩擦地板)，"
             "0.08 时 4s 滑行 <0.03m，远小于 253 带宽 0.388m"));
  declare_parameter<double>(
    "escape_angular_vel", 0.20,
    describe("脱困角速度上限(rad/s)。实测 wz=0.05 就能动，跟踪比 0.73~0.92"));
  declare_parameter<double>(
    "escape_arrive_tol_m", 0.05,
    describe("到目标多近算本段走完(m)。取一个栅格"));
  declare_parameter<double>(
    "escape_align_tol_rad", 0.35,
    describe("航向对齐容差(rad)。误差在此内 wz 归零，避免在带里原地抖动"));
  declare_parameter<double>(
    "escape_timeout_sec", 20.0,
    describe("单次脱困超时(s)。253 带宽=内切半径 0.388m，0.08m/s 走完约 5s，留 4 倍余量"));
  declare_parameter<int>(
    "escape_max_attempts", 3,
    describe("脱困次数上限。达上限后进 PAUSED 等人工，禁止死循环脱困"));
  declare_parameter<int>(
    "escape_clear_ticks", 5,
    describe("连续多少拍读到非致命才算真出带。防栅格边界抖动导致状态来回跳"));
  declare_parameter<double>(
    "breadcrumb_window_sec", 60.0,
    describe("来路轨迹保留时长(s)。脱困优先沿来路退，因为来路可通行是可证明的"));
  declare_parameter<double>(
    "breadcrumb_sample_hz", 2.0,
    describe("来路轨迹采样频率(Hz)"));

  declare_parameter<std::string>(
    "follow_action_name", "follow_path", describe("controller_server 的 FollowPath 动作名"));
  declare_parameter<std::string>(
    "follow_controller_id", "FollowPathExplore",
    describe(
      "FollowPath 用哪个控制器实例。探索场景用 FollowPathExplore(终点不转朝向)。"
      "必须真的在 controller_plugins 里，否则每次 FollowPath 直接 abort"));
  declare_parameter<std::string>(
    "follow_goal_checker_id", "",
    describe(
      "FollowPath 用哪个 goal checker。留空=用 controller_server 的唯一那个。"
      "!!! 只有在 goal_checker_plugins 列了多项时才需要填 !!! "
      "该端口无默认值，列表有多项而这里留空会让每次 FollowPath 直接 abort"));
  declare_parameter<int>(
    "follow_max_retries", 1,
    describe(
      "follow_path 模式下 FollowPath 中止后对同一目标的重试次数。"
      "0=不重试(每次进度停滞都直接判失败，实测会让探索几乎无法完成)。"
      "默认 1，与默认行为树 RecoveryNode number_of_retries=\"1\" 对齐"));
  declare_parameter<double>(
    "replan_period_sec", 1.0,
    describe(
      "follow_path 模式下的重规划周期(s)。**只有 replan_policy=periodic 时才生效**。"
      "0=不重规划(路径失效只能等超时)。取 1.0 与 nav2 默认行为树的 RateController hz=1.0 一致"));
  declare_parameter<std::string>(
    "replan_policy", "on_invalid",
    describe(
      "跟踪期重规划策略。on_invalid(默认)=只在当前路径失效时才换路径，"
      "满足「未跟踪到位不得规划下一条路径」；periodic=旧的无条件周期重规划，"
      "只保留作一键回退(实测该策略下 36 个目标换了 393 条路径，无一条被跟踪到位)"));
  declare_parameter<double>(
    "replan_check_period_sec", 0.5,
    describe(
      "on_invalid 策略下检查「当前路径是否还能用」的节拍(s)。"
      "纯本地几何+栅格校验，不发任何 action"));
  declare_parameter<double>(
    "replan_min_interval_sec", 1.0,
    describe("两次重规划请求的最小间隔(s)，防止判据在阈值附近抖动导致连续换路径"));
  declare_parameter<double>(
    "path_max_age_sec", 0.0,
    describe(
      "当前路径的最大寿命(s)，>0 才启用，纯兜底。"
      "0=不因为「路径旧」而换路径 —— 正常情况下路径失效由剩余段校验判定，"
      "而不该由时间判定，否则又退化成周期重规划"));
  declare_parameter<int>(
    "max_invalid_replan_attempts", 3,
    describe(
      "连续多少次「当前路径已判不可通行 + 重规划也拿不到合法替代」就放弃该目标。"
      "不能是 0(那会让一帧代价地图抖动就丢目标)，也不能很大 —— "
      "实测沿用一条已判死的路径会让机器人原地顶 17s 才被 progress checker 救回来"));
  declare_parameter<double>(
    "path_deviation_limit_m", 0.6,
    describe(
      "机器人偏离当前路径多远就认为这条路径不再描述它的处境(m)、需要重规划。"
      "必须 > 抵达容差与控制器横向误差量级，否则正常跟踪抖动就会触发换路径"));
  declare_parameter<std::string>("map_frame", "map", describe("地图坐标系"));
  declare_parameter<std::string>(
    "robot_base_frame", "astribot_torso_base",
    describe("机器人本体坐标系。**本机器人没有 base_link**，根 frame 是 "
             "astribot_torso_base（Nav2 六处 robot_base_frame 也是它）。"
             "默认值原为 base_link，靠 yaml 覆盖才对 —— 而本仓库踩过 "
             "\"params_file 泄漏 / 节点名 remap 导致整份 yaml 静默失效\" 的坑，"
             "那时会回落到这个默认值，TF 查询全部失败且难以归因。"));
  declare_parameter<std::string>(
    "planner_id", "", describe("指定全局规划器名，空串表示用 Nav2 默认"));

  declare_parameter<double>("control_period_sec", 0.5, describe("状态机节拍(s)"));
  declare_parameter<double>("tf_timeout_sec", 0.2, describe("TF 查询超时(s)"));
  declare_parameter<double>("map_timeout_sec", 10.0, describe("地图多久未更新算超时(s)"));
  declare_parameter<double>(
    "costmap_timeout_sec", 5.0,
    describe("全局代价地图多久未更新算超时(s)。超时则拒绝下发，绝不退回用 /map 校验"));
  declare_parameter<double>("odom_timeout_sec", 1.0, describe("里程计多久未更新算丢失(s)"));
  declare_parameter<double>("plan_timeout_sec", 5.0, describe("路径校验请求超时(s)"));
  declare_parameter<double>("nav_timeout_sec", 120.0, describe("单个导航目标最长允许时间(s)"));

  declare_parameter<double>(
    "arrival_xy_tolerance", 0.30,
    describe("抵达位置容差(m)。Nav2 报成功后仍要实测位置，超差按导航失败处理"));
  declare_parameter<double>("arrival_yaw_tolerance", 0.40, describe("抵达朝向容差(rad)"));
  declare_parameter<bool>(
    "check_yaw", false,
    describe("是否把朝向纳入抵达判定。探索目标的朝向只是「看向未知区」的建议值，默认不卡"));
  declare_parameter<double>(
    "dwell_time_sec", 1.5,
    describe("稳定驻留时长(s)：位置/速度连续满足该时长才算目标收敛"));
  declare_parameter<double>(
    "settle_speed", 0.05, describe("驻留判定的速度上限(m/s)，超过则重新计时"));

  declare_parameter<int>(
    "max_candidates_per_cycle", 5,
    describe("单轮最多校验多少个候选点，防止一轮里无界地试"));
  declare_parameter<int>(
    "max_sample_failures", 4, describe("连续多少轮采不到合法点后转 PAUSED"));
  declare_parameter<int>(
    "max_validation_failures", 4,
    describe("连续多少轮「采到了候选点但校验全废」后转 PAUSED（与 max_sample_failures 分开计）"));
  declare_parameter<int>(
    "max_consecutive_nav_failures", 3, describe("连续多少次导航失败后转 PAUSED"));
  declare_parameter<double>(
    "pause_cooldown_sec", 10.0, describe("进入 PAUSED 后多久尝试一次自动恢复(s)"));
  declare_parameter<int>(
    "max_auto_resume_attempts", 3,
    describe("自动恢复次数上限。达上限后只等人工 resume，禁止死循环重试"));
  declare_parameter<int>("visit_history_limit", 50, describe("历史访问记录保留条数上限"));

  // ---- 未知区域禁行校验参数 ----
  declare_parameter<int>("validator.occupied_threshold", 65, describe("占据判定阈值(0~100)"));
  declare_parameter<int>("validator.free_threshold", 25, describe("空闲判定阈值(0~100)"));
  declare_parameter<double>(
    "validator.goal_clearance_radius", 0.42,
    describe("目标点【占据】净空半径(m)：碰撞约束，应>=机器人外接半径"));
  declare_parameter<double>(
    "validator.goal_unknown_clearance_radius", 0.0,
    describe("目标点【未知】净空半径(m)：必须 < 地图分辨率(实际就是 0)，"
      "否则前沿点(天生紧贴未知区一格)会全部不合法，探索永远发不出目标"));
  declare_parameter<double>(
    "validator.path_sample_step", 0.0,
    describe("路径段内采样步长(m)，<=0 表示自动取地图分辨率的一半"));
  declare_parameter<double>(
    "validator.path_endpoint_tolerance", 0.5,
    describe("规划终点与请求目标的最大偏差(m)，防止半截路径被当成成功"));
  declare_parameter<int>(
    "validator.max_samples", 200000, describe("单条路径采样点数上限，防御性无界保护"));

  // ---- 下发前校验所用栅格图的选择 + 代价地图转换参数 ----
  declare_parameter<bool>(
    "validation.use_costmap", true,
    describe("下发前校验是否用全局代价地图(与规划器同一张图)。"
      "置 false 退回用 /map 校验，那是已知会锁死探索的旧行为，仅供对比排查"));
  declare_parameter<int>(
    "validation.costmap_unknown_cost", 255,
    describe("代价地图中视为未知的值。nav2 固定为 255(NO_INFORMATION)"));
  declare_parameter<int>(
    "validation.costmap_lethal_cost_threshold", 253,
    describe("代价地图中视为致命障碍的下界(含)。默认 253=INSCRIBED_INFLATED_OBSTACLE，"
      "语义是「机器人中心在此则足迹必然碰撞」。不要调低：调低会把可通行的膨胀带"
      "判成障碍，贴墙路径全被否决，探索重新锁死"));

  // ---- 前沿搜索算法参数（与 frontier_explorer 同一套语义）----
  declare_parameter<int>("search.occupied_threshold", 65, describe("占据判定阈值(0~100)"));
  declare_parameter<int>("search.free_threshold", 25, describe("空闲判定阈值(0~100)"));
  declare_parameter<double>("search.obstacle_inflation_radius", 0.45, describe("障碍膨胀半径(m)"));
  declare_parameter<int>(
    "search.min_obstacle_cluster_cells", 3, describe("小于该格数的占据斑块视为噪声"));
  declare_parameter<bool>("search.use_eight_connectivity", true, describe("前沿判定是否用 8 邻域"));
  declare_parameter<int>("search.min_frontier_cells", 12, describe("前沿块最小格数"));
  declare_parameter<double>("search.gain_window_radius", 1.5, describe("未知增益统计窗口半径(m)"));
  declare_parameter<double>(
    "search.adaptive_sample_gain", 0.15, describe("采样数 = ceil(前沿格数 * 该系数)"));
  declare_parameter<int>("search.min_samples_per_cluster", 1, describe("每个前沿块最少采样数"));
  declare_parameter<int>("search.max_samples_per_cluster", 8, describe("每个前沿块最多采样数"));
  declare_parameter<double>("search.required_clearance_radius", 0.45, describe("候选点净空半径(m)"));
  declare_parameter<double>("search.min_goal_distance", 0.8, describe("目标点最小距离(m)"));
  declare_parameter<double>("search.max_goal_distance", 0.0, describe("目标点最大距离(m)，<=0 不限"));
  declare_parameter<double>("search.weight_distance", 1.0, describe("距离代价权重"));
  declare_parameter<double>("search.weight_gain", 6.0, describe("未知增益权重"));
  declare_parameter<double>("search.weight_visit_penalty", 4.0, describe("历史访问惩罚权重"));
  declare_parameter<double>("search.visit_penalty_radius", 1.2, describe("历史访问惩罚作用半径(m)"));
  declare_parameter<int>("search.random_seed", 20260819, describe("采样随机种子"));

  // ---- 冷启动自举（破 SLAM 冷启动死锁）----
  //
  // 死锁链条（实测过）：slam_toolbox 只在机器人移动超过 minimum_travel_heading(0.2rad)
  // 或 minimum_travel_distance(0.2m) 之后才插入新扫描 -> 刚上电时地图一个已知格都没有
  // -> 本节点在地图内容不可用时不下发目标 -> 机器人不动 -> 地图不长 -> 永远不动。
  // 上一轮是靠外部脚本发 cmd_vel 走 0.79m 把它推开的（地图 0 -> 26 m²）。
  declare_parameter<std::string>(
    "bootstrap_mode", "rotate",
    describe(
      "冷启动自举方式。rotate(默认)=只原地旋转；disabled=关闭自举(需人工推一把)。"
      "刻意不提供平移：底盘足迹是外接半径 0.42 / 内切 0.388 的正八边形，"
      "原地旋转最多扫过 3.2cm 环带，几何上几乎不进入新区域；平移是开环积分推进，"
      "风险面完全不同"));
  declare_parameter<std::string>(
    "bootstrap_cmd_vel_topic", "/cmd_vel_nav_body_raw",
    describe(
      "自举速度指令发到哪个话题。默认取 controller_server 的输出点，"
      "这样 velocity_smoother -> cmd_vel_body_to_world(倾倒监控) -> 臂-底盘耦合限速 "
      "-> 力矩闭环 leash 这一整条安全链全部照常生效。"
      "直发 /cmd_vel 会绕过全部四层，不要那样配"));
  declare_parameter<std::string>(
    "bootstrap_yaw_frame", "odom",
    describe(
      "自举量转角用哪个 frame。**默认 odom，不要改成 map**："
      "map -> odom 由 SLAM 发布，而真正的冷启动死锁下 SLAM 还没出图，"
      "用 map 会让自举被自己的前置条件挡死(实测连续 71 次取不到朝向)——"
      "那正是它要破的死锁。而且 map 系 yaw 含 SLAM 回环修正，会污染测量"));
  declare_parameter<std::string>(
    "bootstrap_scan_topic", "/scan_from_cloud",
    describe("自举安全门所用激光话题。应与 nav2 代价地图订阅的是同一条"));
  declare_parameter<double>(
    "bootstrap_angular_vel", 0.40,
    describe("自举原地旋转角速度(rad/s)。会被下游耦合限速再缩放，实际值可能小得多"));
  declare_parameter<double>(
    "bootstrap_duration_sec", 4.0,
    describe(
      "单次自举旋转时长(s)。要按**最坏情况**的下游限速取：耦合节点 min_speed_scale=0.15，"
      "0.40*0.15=0.06rad/s，4s 才转出 0.24rad，刚过 minimum_travel_heading(0.2)"));
  declare_parameter<double>(
    "bootstrap_cmd_rate_hz", 20.0,
    describe(
      "自举期间速度指令重发频率(Hz)。必须显著高于底盘 cmd_vel_timeout_sec=0.5 的倒数，"
      "否则速度会被反复超时归零。不能靠 0.5s 的状态机节拍来发"));
  declare_parameter<double>(
    "bootstrap_trigger_wait_sec", 6.0,
    describe(
      "地图内容持续不可用/采不到候选多久后才自举(s)。"
      "不设 0：启动瞬态里地图本来就要几秒才长出来，立刻自举等于抢在 SLAM 前面动"));
  declare_parameter<double>(
    "bootstrap_scan_timeout_sec", 1.0,
    describe("激光多久未更新就拒绝自举(s)。无数据必须拒绝运动，不得当成「周围没有障碍」"));
  declare_parameter<double>(
    "bootstrap_min_clearance_m", 0.42,
    describe(
      "自举前要求的最小周边净空(m)，取机器人外接半径。"
      "语义：如果最近障碍已经进到自己的足迹半径以内，就不要再转了 —— "
      "此时八边形的顶点可能已经接触障碍"));
  declare_parameter<double>(
    "bootstrap_min_yaw_delta", 0.20,
    describe(
      "一次自举至少要实际转出多少角度才算「真的动了」(rad)。"
      "必须 >= slam_toolbox 的 minimum_travel_heading(0.2)，否则转了也不插入扫描。"
      "达不到时会显式告警并指出最可能的原因(下游限速)，不静默算成功"));
  declare_parameter<int>(
    "bootstrap_max_attempts", 6,
    describe(
      "自举次数上限(每次成功下发目标后清零)。6 次 × 0.24rad(最坏)~1.6rad(不限速) "
      "覆盖从小半圈到多圈；有上限是为了禁止死循环重试"));
  declare_parameter<int>(
    "min_known_cells_for_decision", 100,
    describe(
      "地图里至少要有多少个已知格才认为「内容足以做探索决策」。"
      "这条是 COMPLETED 判定的前置条件：冷启动时地图收到了但全是未知，"
      "前沿格数也是 0，只看前沿格数会把空地图判成「探索完成」"));
}

bool ExplorationCoordinatorNode::loadParameters(std::string & error)
{
  map_topic_ = get_parameter("map_topic").as_string();
  costmap_topic_ = get_parameter("costmap_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  state_topic_ = get_parameter("state_topic").as_string();
  fp_watchdog_enabled_ = get_parameter("footprint_watchdog.enabled").as_bool();
  fp_watchdog_write_topic_ = get_parameter("footprint_watchdog.write_topic").as_string();
  fp_watchdog_readback_topic_ = get_parameter("footprint_watchdog.readback_topic").as_string();
  fp_watchdog_default_footprint_ =
    get_parameter("footprint_watchdog.default_footprint").as_string();
  fp_watchdog_lease_timeout_sec_ =
    get_parameter("footprint_watchdog.lease_timeout_sec").as_double();
  fp_watchdog_readback_period_sec_ =
    get_parameter("footprint_watchdog.readback_period_sec").as_double();
  fp_watchdog_readback_stale_sec_ =
    get_parameter("footprint_watchdog.readback_stale_sec").as_double();
  fp_watchdog_consecutive_reads_ =
    static_cast<int>(get_parameter("footprint_watchdog.consecutive_reads").as_int());
  complete_topic_ = get_parameter("complete_topic").as_string();
  current_goal_topic_ = get_parameter("current_goal_topic").as_string();
  nav_action_name_ = get_parameter("nav_action_name").as_string();
  nav_behavior_tree_ = get_parameter("nav_behavior_tree").as_string();
  const std::string mode_str = get_parameter("nav_dispatch_mode").as_string();
  if (mode_str == "follow_path") {
    dispatch_mode_ = DispatchMode::kFollowPath;
  } else if (mode_str == "navigate_to_pose") {
    dispatch_mode_ = DispatchMode::kNavigateToPose;
  } else {
    // 非法值拒绝启动，不静默回落 —— 回落会让"我明明配了复用"变成静默失效。
    throw std::runtime_error(
      "nav_dispatch_mode 非法: '" + mode_str + "'，只接受 follow_path / navigate_to_pose");
  }
  // ---- 脱困参数读取 + 自检 ----
  escape_enabled_ = get_parameter("escape_enabled").as_bool();
  escape_trigger_failures_ = static_cast<int>(get_parameter("escape_trigger_failures").as_int());
  escape_search_radius_m_ = get_parameter("escape_search_radius_m").as_double();
  escape_heading_tol_rad_ = get_parameter("escape_heading_tol_rad").as_double();
  escape_linear_vel_ = get_parameter("escape_linear_vel").as_double();
  escape_angular_vel_ = get_parameter("escape_angular_vel").as_double();
  escape_arrive_tol_m_ = get_parameter("escape_arrive_tol_m").as_double();
  escape_align_tol_rad_ = get_parameter("escape_align_tol_rad").as_double();
  escape_timeout_sec_ = get_parameter("escape_timeout_sec").as_double();
  escape_max_attempts_ = static_cast<int>(get_parameter("escape_max_attempts").as_int());
  escape_clear_ticks_ = static_cast<int>(get_parameter("escape_clear_ticks").as_int());
  breadcrumb_window_sec_ = get_parameter("breadcrumb_window_sec").as_double();
  breadcrumb_sample_hz_ = get_parameter("breadcrumb_sample_hz").as_double();
  if (escape_enabled_) {
    // 非法参数直接拒绝启动，不带着危险配置蒙着跑 —— 本包既有纪律。
    if (escape_trigger_failures_ < 1) {
      throw std::runtime_error("escape_trigger_failures 必须 >= 1");
    }
    if (!(escape_search_radius_m_ > 0.0)) {
      throw std::runtime_error("escape_search_radius_m 必须 > 0");
    }
    if (!(escape_linear_vel_ > 0.0) || !(escape_angular_vel_ > 0.0)) {
      throw std::runtime_error("escape_linear_vel / escape_angular_vel 必须 > 0");
    }
    if (!(escape_timeout_sec_ > 0.0)) {
      throw std::runtime_error("escape_timeout_sec 必须 > 0");
    }
    if (escape_clear_ticks_ < 1) {
      // 0 会让 escapeCleared 永不成立(见其实现)，那等于脱困永远不结束。
      throw std::runtime_error("escape_clear_ticks 必须 >= 1");
    }
    if (escape_max_attempts_ < 1) {
      throw std::runtime_error("escape_max_attempts 必须 >= 1");
    }
    if (!(breadcrumb_sample_hz_ > 0.0) || !(breadcrumb_window_sec_ > 0.0)) {
      throw std::runtime_error("breadcrumb_sample_hz / breadcrumb_window_sec 必须 > 0");
    }
    if (escape_linear_vel_ > 0.30) {
      // 脱困是故障恢复态，不是正常导航。高速脱困等于让一个已经异常的
      // 状态机在贴着障碍的地方快跑。
      throw std::runtime_error(
        "escape_linear_vel 不得超过 0.30 m/s：脱困必须低速(见 escape_logic.hpp 文件头)");
    }
  }
  follow_action_name_ = get_parameter("follow_action_name").as_string();
  follow_controller_id_ = get_parameter("follow_controller_id").as_string();
  follow_goal_checker_id_ = get_parameter("follow_goal_checker_id").as_string();
  replan_period_sec_ = get_parameter("replan_period_sec").as_double();
  const std::string policy_str = get_parameter("replan_policy").as_string();
  if (policy_str == "on_invalid") {
    replan_policy_ = ReplanPolicy::kOnInvalid;
  } else if (policy_str == "periodic") {
    replan_policy_ = ReplanPolicy::kPeriodic;
  } else {
    throw std::runtime_error(
      "replan_policy 非法: '" + policy_str + "'，只接受 on_invalid / periodic");
  }
  replan_check_period_sec_ = get_parameter("replan_check_period_sec").as_double();
  replan_min_interval_sec_ = get_parameter("replan_min_interval_sec").as_double();
  path_max_age_sec_ = get_parameter("path_max_age_sec").as_double();
  path_deviation_limit_m_ = get_parameter("path_deviation_limit_m").as_double();
  if (replan_check_period_sec_ <= 0.0) {
    throw std::runtime_error("replan_check_period_sec 必须 > 0");
  }
  if (replan_min_interval_sec_ < 0.0 || path_max_age_sec_ < 0.0) {
    throw std::runtime_error("replan_min_interval_sec / path_max_age_sec 不能为负");
  }
  if (!(path_deviation_limit_m_ > 0.0)) {
    throw std::runtime_error("path_deviation_limit_m 必须 > 0");
  }
  max_invalid_replan_attempts_ =
    static_cast<int>(get_parameter("max_invalid_replan_attempts").as_int());
  if (max_invalid_replan_attempts_ < 1) {
    throw std::runtime_error(
      "max_invalid_replan_attempts 必须 >=1：0 等于允许无限期跟踪一条已判死的路径");
  }
  follow_max_retries_ = static_cast<int>(get_parameter("follow_max_retries").as_int());
  if (follow_max_retries_ < 0) {
    throw std::runtime_error("follow_max_retries 不能为负");
  }
  if (replan_period_sec_ < 0.0) {
    throw std::runtime_error("replan_period_sec 不能为负");
  }
  // 注意：path_deviation_limit_m 与 arrival_xy_tolerance 的耦合校验放在下面
  // arrival_* 参数读完之后 —— 在这里查会拿到还没赋值的 0.0，等于没查。
  if (dispatch_mode_ == DispatchMode::kFollowPath) {
    if (follow_action_name_.empty() || follow_controller_id_.empty()) {
      throw std::runtime_error(
        "follow_path 模式下 follow_action_name / follow_controller_id 不能为空");
    }
    if (replan_policy_ == ReplanPolicy::kPeriodic && replan_period_sec_ == 0.0) {
      // 允许，但必须说清代价：BT 的重规划拿不到了，路径失效只能等 nav_timeout。
      RCLCPP_WARN(
        get_logger(),
        "follow_path 模式且 replan_policy=periodic + replan_period_sec=0：不做重规划。"
        "路径中途失效时只能等 nav_timeout_sec(%.1fs) 超时，"
        "而 BT 的 1Hz 重规划与恢复行为在本模式下都拿不到",
        nav_timeout_sec_);
    }
    if (replan_policy_ == ReplanPolicy::kPeriodic) {
      RCLCPP_WARN(
        get_logger(),
        "replan_policy=periodic 是回退配置：会无条件每 %.1fs 换一条路径，"
        "没有任何一条会被跟踪到位(实测 36 个目标换了 393 条)。"
        "正常运行请用 on_invalid",
        replan_period_sec_);
    }
  }
  plan_action_name_ = get_parameter("plan_action_name").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  planner_id_ = get_parameter("planner_id").as_string();
  if (map_frame_.empty() || robot_base_frame_.empty() ||
    nav_action_name_.empty() || plan_action_name_.empty())
  {
    error = "map_frame / robot_base_frame / nav_action_name / plan_action_name 不能为空";
    return false;
  }

  control_period_sec_ = get_parameter("control_period_sec").as_double();
  tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
  map_timeout_sec_ = get_parameter("map_timeout_sec").as_double();
  costmap_timeout_sec_ = get_parameter("costmap_timeout_sec").as_double();
  odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
  plan_timeout_sec_ = get_parameter("plan_timeout_sec").as_double();
  nav_timeout_sec_ = get_parameter("nav_timeout_sec").as_double();
  if (control_period_sec_ <= 0.0 || tf_timeout_sec_ < 0.0 || map_timeout_sec_ <= 0.0 ||
    odom_timeout_sec_ <= 0.0 || plan_timeout_sec_ <= 0.0 || nav_timeout_sec_ <= 0.0 ||
    costmap_timeout_sec_ <= 0.0)
  {
    error = "节拍/超时参数非法：control_period_sec>0, tf_timeout_sec>=0, 其余超时>0";
    return false;
  }

  arrival_xy_tolerance_ = get_parameter("arrival_xy_tolerance").as_double();
  arrival_yaw_tolerance_ = get_parameter("arrival_yaw_tolerance").as_double();
  check_yaw_ = get_parameter("check_yaw").as_bool();
  dwell_time_sec_ = get_parameter("dwell_time_sec").as_double();
  settle_speed_ = get_parameter("settle_speed").as_double();
  if (arrival_xy_tolerance_ <= 0.0 || arrival_yaw_tolerance_ <= 0.0 ||
    dwell_time_sec_ < 0.0 || settle_speed_ < 0.0)
  {
    error = "抵达判定参数非法：容差>0, dwell_time_sec>=0, settle_speed>=0";
    return false;
  }
  // 判据颠倒必须拒绝启动：偏离阈值若不大于抵达容差，收尾阶段的正常贴合误差
  // 就会被判成「已偏离路径」，于是每次快到目标时都换一条新路径 ——
  // 那正是本次要修掉的现象，不能靠新参数再造一遍。
  if (path_deviation_limit_m_ <= arrival_xy_tolerance_) {
    error = "path_deviation_limit_m(" + std::to_string(path_deviation_limit_m_) +
      ") 必须 > arrival_xy_tolerance(" + std::to_string(arrival_xy_tolerance_) +
      ")，否则收尾阶段的正常贴合误差会被判成偏离路径、反复换路径";
    return false;
  }

  max_candidates_per_cycle_ = static_cast<int>(get_parameter("max_candidates_per_cycle").as_int());
  failure_budget_.max_sample_failures =
    static_cast<int>(get_parameter("max_sample_failures").as_int());
  failure_budget_.max_validation_failures =
    static_cast<int>(get_parameter("max_validation_failures").as_int());
  max_consecutive_nav_failures_ =
    static_cast<int>(get_parameter("max_consecutive_nav_failures").as_int());
  pause_cooldown_sec_ = get_parameter("pause_cooldown_sec").as_double();
  max_auto_resume_attempts_ = static_cast<int>(get_parameter("max_auto_resume_attempts").as_int());
  if (max_candidates_per_cycle_ < 1 || failure_budget_.max_sample_failures < 1 ||
    failure_budget_.max_validation_failures < 1 ||
    max_consecutive_nav_failures_ < 1 || pause_cooldown_sec_ < 0.0 ||
    max_auto_resume_attempts_ < 0)
  {
    error = "重试限次参数非法：max_* >=1, pause_cooldown_sec>=0, max_auto_resume_attempts>=0";
    return false;
  }

  const auto history_limit = get_parameter("visit_history_limit").as_int();
  if (history_limit < 1) {
    error = "visit_history_limit 必须 >=1";
    return false;
  }
  visit_history_limit_ = static_cast<std::size_t>(history_limit);

  PathValidatorParams vp;
  vp.occupied_threshold = static_cast<int>(get_parameter("validator.occupied_threshold").as_int());
  vp.free_threshold = static_cast<int>(get_parameter("validator.free_threshold").as_int());
  vp.goal_clearance_radius = get_parameter("validator.goal_clearance_radius").as_double();
  vp.goal_unknown_clearance_radius =
    get_parameter("validator.goal_unknown_clearance_radius").as_double();
  unknown_clearance_radius_ = vp.goal_unknown_clearance_radius;
  vp.path_sample_step = get_parameter("validator.path_sample_step").as_double();
  vp.path_endpoint_tolerance = get_parameter("validator.path_endpoint_tolerance").as_double();
  const auto max_samples = get_parameter("validator.max_samples").as_int();
  if (max_samples < 1) {
    error = "validator.max_samples 必须 >=1";
    return false;
  }
  vp.max_samples = static_cast<std::size_t>(max_samples);
  std::string validator_error;
  if (!validator_.configure(vp, validator_error)) {
    error = "未知区校验参数非法: " + validator_error;
    return false;
  }

  use_costmap_for_validation_ = get_parameter("validation.use_costmap").as_bool();
  costmap_params_.unknown_cost =
    static_cast<int>(get_parameter("validation.costmap_unknown_cost").as_int());
  costmap_params_.lethal_cost_threshold =
    static_cast<int>(get_parameter("validation.costmap_lethal_cost_threshold").as_int());
  std::string costmap_param_error;
  if (!validateCostmapAdapterParams(costmap_params_, costmap_param_error)) {
    error = "代价地图转换参数非法: " + costmap_param_error;
    return false;
  }
  if (use_costmap_for_validation_ && costmap_topic_.empty()) {
    error = "validation.use_costmap=true 时 costmap_topic 不能为空";
    return false;
  }
  // 「阈值口径」与「校验数据源」必须配套。这条实测栽过两次，两次方向相反：
  //
  // 1) 太大：方案A 刚落地时 goal_clearance_radius 还留着按 /map 标定的 0.42，
  //    一轮跑下来 18 次成功下发、却有 691 次目标复检否决(99.1% 都是这一条)。
  //    原因是重复计算足迹半径：costmap 里代价 >=253 的语义已经是
  //    「机器人中心在此则足迹必然碰撞」，膨胀余量算过了；再套 0.42m 邻域，
  //    等于要求目标离真实障碍 内切半径0.388 + 0.42 ≈ 0.81m。
  //
  // 2) 太小：于是改成 0.0（纯碰撞判据，理论上不多不少），结果 3 个目标全部
  //    校验通过、Nav2 全部接受、然后**全部导航超时**，恢复行为触发 9 次，
  //    控制器 26 次 "Failed to make progress"。目标压在致命区边界上，
  //    而控制器有自己的收敛容差球，球内任何一点都可能让足迹碰撞。
  //
  // 结论：正确取值是**控制器的 xy_goal_tolerance**，语义是
  // 「以目标为心、控制器容差为半径的球内，足迹处处不碰撞」——既合法又收敛得进去。
  // 这里只能查出方向1(重复计算)；方向2 依赖 Nav2 侧参数，本节点读不到，
  // 靠 yaml 注释和这段记录约束。
  if (use_costmap_for_validation_ && vp.goal_clearance_radius > 0.42) {
    RCLCPP_WARN(
      get_logger(),
      "validator.goal_clearance_radius=%.3fm 与 validation.use_costmap=true 不配套："
      "代价地图里 >=%d 的格本身就表示「机器人中心在此则足迹必然碰撞」，"
      "再要求这么大的邻域净空等于把足迹半径重复计一次，会让大量贴墙前沿被误否决"
      "(实测该配置下 99%% 的目标复检失败都源于此)。"
      "建议取 nav2 controller 的 xy_goal_tolerance(本项目 0.25)："
      "既不重复计足迹，又能保证控制器收敛球内不碰撞",
      vp.goal_clearance_radius, costmap_params_.lethal_cost_threshold);
  }

  FrontierSearchParams sp;
  sp.occupied_threshold = static_cast<int>(get_parameter("search.occupied_threshold").as_int());
  sp.free_threshold = static_cast<int>(get_parameter("search.free_threshold").as_int());
  sp.obstacle_inflation_radius = get_parameter("search.obstacle_inflation_radius").as_double();
  sp.min_obstacle_cluster_cells =
    static_cast<int>(get_parameter("search.min_obstacle_cluster_cells").as_int());
  sp.use_eight_connectivity = get_parameter("search.use_eight_connectivity").as_bool();
  sp.min_frontier_cells = static_cast<int>(get_parameter("search.min_frontier_cells").as_int());
  sp.gain_window_radius = get_parameter("search.gain_window_radius").as_double();
  sp.adaptive_sample_gain = get_parameter("search.adaptive_sample_gain").as_double();
  sp.min_samples_per_cluster =
    static_cast<int>(get_parameter("search.min_samples_per_cluster").as_int());
  sp.max_samples_per_cluster =
    static_cast<int>(get_parameter("search.max_samples_per_cluster").as_int());
  sp.required_clearance_radius = get_parameter("search.required_clearance_radius").as_double();
  sp.min_goal_distance = get_parameter("search.min_goal_distance").as_double();
  sp.max_goal_distance = get_parameter("search.max_goal_distance").as_double();
  sp.weight_distance = get_parameter("search.weight_distance").as_double();
  sp.weight_gain = get_parameter("search.weight_gain").as_double();
  sp.weight_visit_penalty = get_parameter("search.weight_visit_penalty").as_double();
  sp.visit_penalty_radius = get_parameter("search.visit_penalty_radius").as_double();
  sp.random_seed = static_cast<unsigned int>(get_parameter("search.random_seed").as_int());
  std::string search_error;
  if (!search_.configure(sp, search_error)) {
    error = "前沿搜索参数非法: " + search_error;
    return false;
  }
  search_params_ = sp;

  // ---- 冷启动自举 ----
  const std::string boot_str = get_parameter("bootstrap_mode").as_string();
  if (boot_str == "rotate") {
    bootstrap_mode_ = BootstrapMode::kRotate;
  } else if (boot_str == "disabled") {
    bootstrap_mode_ = BootstrapMode::kDisabled;
  } else {
    error = "bootstrap_mode 非法: '" + boot_str + "'，只接受 rotate / disabled";
    return false;
  }
  bootstrap_cmd_vel_topic_ = get_parameter("bootstrap_cmd_vel_topic").as_string();
  bootstrap_scan_topic_ = get_parameter("bootstrap_scan_topic").as_string();
  bootstrap_yaw_frame_ = get_parameter("bootstrap_yaw_frame").as_string();
  bootstrap_angular_vel_ = get_parameter("bootstrap_angular_vel").as_double();
  bootstrap_duration_sec_ = get_parameter("bootstrap_duration_sec").as_double();
  bootstrap_cmd_rate_hz_ = get_parameter("bootstrap_cmd_rate_hz").as_double();
  bootstrap_trigger_wait_sec_ = get_parameter("bootstrap_trigger_wait_sec").as_double();
  bootstrap_scan_timeout_sec_ = get_parameter("bootstrap_scan_timeout_sec").as_double();
  bootstrap_min_clearance_m_ = get_parameter("bootstrap_min_clearance_m").as_double();
  bootstrap_min_yaw_delta_ = get_parameter("bootstrap_min_yaw_delta").as_double();
  bootstrap_max_attempts_ = static_cast<int>(get_parameter("bootstrap_max_attempts").as_int());
  const auto known_cells = get_parameter("min_known_cells_for_decision").as_int();
  if (known_cells < 1) {
    error = "min_known_cells_for_decision 必须 >=1";
    return false;
  }
  min_known_cells_for_decision_ = static_cast<std::size_t>(known_cells);
  if (bootstrap_mode_ != BootstrapMode::kDisabled) {
    if (bootstrap_cmd_vel_topic_.empty() || bootstrap_scan_topic_.empty() ||
      bootstrap_yaw_frame_.empty())
    {
      error = "自举已启用但 bootstrap_cmd_vel_topic / bootstrap_scan_topic / "
        "bootstrap_yaw_frame 为空";
      return false;
    }
    if (!(bootstrap_angular_vel_ > 0.0) || !(bootstrap_duration_sec_ > 0.0) ||
      !(bootstrap_cmd_rate_hz_ > 0.0) || bootstrap_trigger_wait_sec_ < 0.0 ||
      !(bootstrap_scan_timeout_sec_ > 0.0) || !(bootstrap_min_clearance_m_ > 0.0) ||
      !(bootstrap_min_yaw_delta_ > 0.0) || bootstrap_max_attempts_ < 1)
    {
      error = "自举参数非法：角速度/时长/频率/超时/净空/最小转角 均须 >0，次数上限 >=1";
      return false;
    }
    // 频率必须显著高于底盘 cmd_vel 超时的倒数，否则速度会被反复归零。
    // 底盘 cmd_vel_timeout_sec 是另一个包的参数，本节点读不到，
    // 这里按已知值 0.5s 做下限校验并在注释里记录出处。
    constexpr double kChassisCmdVelTimeoutSec = 0.5;
    if (bootstrap_cmd_rate_hz_ < 2.0 / kChassisCmdVelTimeoutSec) {
      error = "bootstrap_cmd_rate_hz 太低(" + std::to_string(bootstrap_cmd_rate_hz_) +
        ")：底盘 cmd_vel_timeout_sec=0.5，至少要 4Hz 才不会被反复超时归零";
      return false;
    }
    // 转不出 minimum_travel_heading 就等于没转，SLAM 不会插入新扫描。
    // 这条只能查「配置上是否自相矛盾」，实际转出多少由 tickBootstrap 实测并告警。
    if (bootstrap_angular_vel_ * bootstrap_duration_sec_ < bootstrap_min_yaw_delta_) {
      error = "自举配置自相矛盾：即使完全不限速，"
        "bootstrap_angular_vel * bootstrap_duration_sec = " +
        std::to_string(bootstrap_angular_vel_ * bootstrap_duration_sec_) +
        "rad 也达不到 bootstrap_min_yaw_delta=" + std::to_string(bootstrap_min_yaw_delta_) + "rad";
      return false;
    }
  }

  error.clear();
  return true;
}

// ====================== 数据回调与前置条件 ======================
// 需求：所有回调必须做空指针/有效性判断；地图为空或延迟必须拦截逻辑而不是崩溃。

void ExplorationCoordinatorNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  if (!msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "收到空的地图消息指针，已忽略");
    return;
  }

  const std::size_t expected =
    static_cast<std::size_t>(msg->info.width) * static_cast<std::size_t>(msg->info.height);
  if (msg->info.width == 0U || msg->info.height == 0U || expected == 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "地图尺寸为 0 (%ux%u)，忽略本帧", msg->info.width, msg->info.height);
    return;
  }
  if (msg->data.size() != expected) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "地图 data 长度 %zu 与 width*height=%zu 不一致，忽略本帧(防越界读)",
      msg->data.size(), expected);
    return;
  }
  if (!(msg->info.resolution > 0.0)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "地图分辨率非法(%.4f)，忽略本帧", msg->info.resolution);
    return;
  }
  // 未知净空半径与地图分辨率的相容性检查。
  // 只有拿到真实地图才知道分辨率，所以这条只能在这里做，不能在参数校验里做。
  //
  // 机制：前沿格的定义是「空闲格且 8 邻域含未知格」，未知邻居的格距离
  // d_sq = 1(正交) 或 2(对角)；校验按 d_sq <= (r/resolution)^2 遍历邻域。
  // 于是 r >= resolution 时 (r/res)^2 >= 1 >= d_sq，**每一个**前沿候选都会被否，
  // 探索表现为 dispatched 恒为 0、机器人一步不动，而日志只说「目标校验淘汰 N」。
  // 这个坑真实踩过两次(0.42 -> 0.10 仍然中招)，所以在这里硬性告警。
  if (unknown_clearance_radius_ >= msg->info.resolution) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "validator.goal_unknown_clearance_radius=%.3fm >= 地图分辨率 %.3fm，"
      "这会让【每一个】前沿候选都被目标校验否决(前沿格的未知邻居只有一格远)，"
      "探索将永远发不出目标。请把该参数改成 0.0",
      unknown_clearance_radius_, msg->info.resolution);
  }
  // 时间戳校验：未来戳说明有节点时钟不同步，这种地图用来做未知区判定风险很大。
  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() > 0) {
    const double ahead = (stamp - now()).seconds();
    if (ahead > 1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "地图时间戳比本地时钟超前 %.2fs，可能存在时钟不同步", ahead);
    }
  }

  auto snapshot = std::make_shared<GridMap>();
  snapshot->width = msg->info.width;
  snapshot->height = msg->info.height;
  snapshot->resolution = msg->info.resolution;
  snapshot->origin_x = msg->info.origin.position.x;
  snapshot->origin_y = msg->info.origin.position.y;
  snapshot->data = msg->data;

  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_ = std::move(snapshot);
  latest_map_time_ = now();
}

void ExplorationCoordinatorNode::costmapCallback(
  const nav2_msgs::msg::Costmap::ConstSharedPtr & msg)
{
  if (!msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "收到空的代价地图消息指针，已忽略");
    return;
  }

  // 尺寸/长度/分辨率的全部校验都在 costmapToGridMap 里，
  // 这里不重复一遍——重复校验迟早会和被调方漂移，出问题时两处说法不一致更难查。
  auto snapshot = std::make_shared<GridMap>();
  std::string convert_error;
  if (!costmapToGridMap(
      msg->metadata.size_x, msg->metadata.size_y,
      static_cast<double>(msg->metadata.resolution),
      msg->metadata.origin.position.x, msg->metadata.origin.position.y,
      msg->data, costmap_params_, *snapshot, convert_error))
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "代价地图转换失败，忽略本帧: %s", convert_error.c_str());
    return;
  }

  // 代价地图的 origin 会随机器人滚动(rolling window)，与 /map 的 origin 不同，
  // 这是正常的——两张图各自用自己的 origin 做 worldToMap，不能混用。
  // 这里只在首帧打一条日志，便于确认订阅到的确实是全局图。
  const CostmapGridStats stats = summarizeGrid(*snapshot);
  const unsigned int width = snapshot->width;
  const unsigned int height = snapshot->height;
  const double resolution = snapshot->resolution;
  const double ox = snapshot->origin_x;
  const double oy = snapshot->origin_y;

  bool first_frame = false;
  {
    // latest_costmap_time_ 由本锁保护，首帧判定必须在锁内做，不能在锁外读。
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    first_frame = (latest_costmap_time_.nanoseconds() == 0);
    latest_costmap_ = std::move(snapshot);
    latest_costmap_time_ = now();
  }

  RCLCPP_INFO_EXPRESSION(
    get_logger(), first_frame,
    "首帧全局代价地图: %ux%u @%.3fm 原点(%.2f, %.2f) | 未知 %.1f%% 致命 %.1f%% 空闲 %.1f%%"
    " (阈值: 未知值=%d 致命下界=%d)",
    width, height, resolution, ox, oy,
    stats.unknownRatio() * 100.0, stats.lethalRatio() * 100.0,
    (1.0 - stats.unknownRatio() - stats.lethalRatio()) * 100.0,
    costmap_params_.unknown_cost, costmap_params_.lethal_cost_threshold);
}

std::shared_ptr<GridMap> ExplorationCoordinatorNode::validationGrid(std::string & why)
{
  why.clear();
  if (!use_costmap_for_validation_) {
    // 回退路径：用 /map 校验。已知会锁死探索，仅供对比排查，构造时已 WARN 过。
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!latest_map_ || !latest_map_->consistent()) {
      why = "校验用 /map 不可用";
      return nullptr;
    }
    return latest_map_;
  }

  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!latest_costmap_ || !latest_costmap_->consistent()) {
    // 绝不在这里退回 /map：那正是把两层判据放到两张图上的错误做法。
    // 拿不到代价地图就不下发，宁可停着也不能拿错误的图放行目标。
    why = "尚未收到可用的全局代价地图(" + costmap_topic_ + ")";
    return nullptr;
  }
  const double age = (now() - latest_costmap_time_).seconds();
  if (age > costmap_timeout_sec_) {
    why = "全局代价地图已 " + std::to_string(age) + "s 未更新(超时上限 " +
      std::to_string(costmap_timeout_sec_) + "s)";
    return nullptr;
  }
  return latest_costmap_;
}

void ExplorationCoordinatorNode::odomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  if (!msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "收到空的里程计消息指针，已忽略");
    return;
  }
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  if (!std::isfinite(vx) || !std::isfinite(vy)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "里程计速度含 NaN/Inf，忽略本帧");
    return;
  }
  std::lock_guard<std::mutex> lock(odom_mutex_);
  // 全向底盘可以纯横移，速度必须取合速度，只看 linear.x 会把横移当成「已静止」。
  odom_speed_ = std::hypot(vx, vy);
  latest_odom_time_ = now();
}

void ExplorationCoordinatorNode::scanCallback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  if (!msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "收到空的激光消息指针，已忽略");
    return;
  }
  // 取全周最小有效距离。自举是原地旋转，机器人会依次朝向每个方向，
  // 所以关心的是「一圈里最近的障碍」，不是只看正前方一个扇区。
  double min_range = std::numeric_limits<double>::infinity();
  std::size_t valid = 0U;
  for (const float r : msg->ranges) {
    // NaN/Inf 是「这个方向没有回波」，不是「这个方向很近」，必须跳过而不是当成 0。
    if (!std::isfinite(r)) {
      continue;
    }
    if (r < msg->range_min || r > msg->range_max) {
      continue;
    }
    ++valid;
    min_range = std::min(min_range, static_cast<double>(r));
  }
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_time_ = now();
  if (valid == 0U) {
    // 一个有效点都没有：这**不是**「周围空旷」。置成 -1 让安全门判为不可用。
    latest_scan_min_range_ = -1.0;
    has_scan_ = true;
    return;
  }
  latest_scan_min_range_ = min_range;
  has_scan_ = true;
}

bool ExplorationCoordinatorNode::mapUsable(const GridMap & map, std::size_t & known_cells) const
{
  known_cells = 0U;
  for (const int8_t v : map.data) {
    if (v >= 0) {                      // >=0 即已知（0 空闲 ~ 100 占据），-1 是未知
      ++known_cells;
    }
  }
  return known_cells >= min_known_cells_for_decision_;
}

bool ExplorationCoordinatorNode::stackReady(std::string & why)
{
  if (!mapReady(why)) {
    return false;
  }
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (!robotPose(x, y, yaw, why)) {
    return false;
  }
  if (!odomReady(why)) {
    return false;
  }
  // 校验图也必须在 —— 它不在时一个候选都产不出来，那不是"探索失败"，
  // 是栈还没起齐。之前正是这一项缺失导致 0 派发却把预算烧光。
  std::string grid_why;
  if (!validationGrid(grid_why)) {
    why = "校验图未就绪: " + grid_why;
    return false;
  }
  return true;
}

bool ExplorationCoordinatorNode::mapReady(std::string & why)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_map_) {
    why = "尚未收到占据栅格地图";
    return false;
  }
  if (!latest_map_->consistent()) {
    why = "地图快照自身不自洽";
    return false;
  }
  const double age = (now() - latest_map_time_).seconds();
  if (age > map_timeout_sec_) {
    why = "地图已 " + std::to_string(age) + "s 未更新(超时)";
    return false;
  }
  why.clear();
  return true;
}

bool ExplorationCoordinatorNode::odomReady(std::string & why)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (latest_odom_time_.nanoseconds() == 0) {
    why = "尚未收到里程计";
    return false;
  }
  const double age = (now() - latest_odom_time_).seconds();
  if (age > odom_timeout_sec_) {
    why = "里程计已 " + std::to_string(age) + "s 未更新";
    return false;
  }
  why.clear();
  return true;
}

bool ExplorationCoordinatorNode::poseInFrame(
  const std::string & frame, double & x, double & y, double & yaw, std::string & why)
{
  if (!tf_buffer_) {
    why = "TF buffer 未初始化";
    return false;
  }
  try {
    // 取最新可用变换：探索调度是低频决策，不需要和某一帧数据严格对齐。
    const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      frame, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_sec_));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = tf2::getYaw(tf.transform.rotation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      why = "TF 返回值含 NaN/Inf";
      return false;
    }
    why.clear();
    return true;
  } catch (const tf2::TransformException & e) {
    why = std::string("TF 查询失败(") + frame + " -> " + robot_base_frame_ + "): " + e.what();
    return false;
  }
}

bool ExplorationCoordinatorNode::robotPose(double & x, double & y, double & yaw, std::string & why)
{
  // map -> base 查不到即视为定位丢失，上层必须冻结目标发布。
  return poseInFrame(map_frame_, x, y, yaw, why);
}

void ExplorationCoordinatorNode::recordVisit(double x, double y)
{
  for (auto & v : visit_history_) {
    if (std::hypot(v.x - x, v.y - y) <= search_params_.visit_penalty_radius) {
      if (v.count < 10U) {   // 惩罚上限，避免计数无界增长把代价函数压死
        ++v.count;
      }
      return;
    }
  }
  visit_history_.push_back(VisitRecord{x, y, 1U});
  if (visit_history_.size() > visit_history_limit_) {
    visit_history_.erase(visit_history_.begin());
  }
}

void ExplorationCoordinatorNode::resetCycleState()
{
  candidates_.clear();
  candidate_index_ = 0U;
  // 一轮结束（成功/失败都算）后，正在跟踪的路径不再有效。
  // 忘了清会让下一个目标的第一次 needsReplan 拿上一个目标的路径去算偏离度，
  // 结果必然远超阈值 -> 刚下发就立刻换一次路径。
  active_path_.clear();
  replan_forced_ = false;
  active_path_impassable_ = false;
  invalid_replan_count_ = 0;
}

// ========================= 状态机 =========================

void ExplorationCoordinatorNode::transitionTo(ExplorationState next, const std::string & why)
{
  if (state_ == next) {
    return;                                   // 空转换不刷新计时，避免驻留/冷却计时被反复重置
  }
  // 离开自举态**一定**要显式发一帧零速，不能只是「停止发布」。
  // 放在这里而不是各个出口：pause 服务、安全门中断、正常转完、非法转换回退
  // 全都要经过这里，一处覆盖所有出口，漏一条就是「已暂停但机器人还在转」。
  if (drivesChassisDirectly(state_) && !drivesChassisDirectly(next)) {
    publishBootstrapCmd(true);
  }
  if (!isTransitionAllowed(state_, next)) {
    // 非法转换不是「纠正一下继续跑」，而是状态机本身出了问题：
    // 强制回退到 kIdle 这个无在途目标的安全态，并把在途目标撤掉。
    RCLCPP_ERROR(
      get_logger(), "非法状态转换 %s -> %s (%s)，强制回退 IDLE",
      toString(state_), toString(next), why.c_str());
    cancelActiveNavGoal("状态机非法转换，回退安全态");
    resetCycleState();
    state_ = ExplorationState::kIdle;
    state_entered_time_ = now();
    return;
  }
  RCLCPP_INFO(get_logger(), "状态 %s -> %s : %s", toString(state_), toString(next), why.c_str());
  state_ = next;
  state_entered_time_ = now();
}

void ExplorationCoordinatorNode::setupFootprintWatchdog()
{
  if (!fp_watchdog_enabled_) {
    RCLCPP_INFO(
      get_logger(),
      "足迹锁存看门狗未启用(footprint_watchdog.enabled=false)。"
      "只有在控制器开了 narrow_square_enabled 时才需要它。");
    return;
  }

  std::vector<geometry_msgs::msg::Point> default_fp;
  if (!nav2_costmap_2d::makeFootprintFromString(fp_watchdog_default_footprint_, default_fp) ||
    default_fp.size() < 3U)
  {
    // 🔴 看门狗自己配错了绝不能静默降级 —— 那等于"以为有兜底其实没有"，
    //    比明确没有兜底更危险。
    throw std::runtime_error(
      "footprint_watchdog.default_footprint 解析失败或少于 3 点: '" +
      fp_watchdog_default_footprint_ + "' —— 看门狗无法复原，拒绝启动");
  }
  fp_watchdog_default_vertices_ = default_fp.size();
  if (!(fp_watchdog_lease_timeout_sec_ > 0.0)) {
    throw std::runtime_error("footprint_watchdog.lease_timeout_sec 必须 > 0");
  }
  if (fp_watchdog_consecutive_reads_ < 1) {
    throw std::runtime_error("footprint_watchdog.consecutive_reads 必须 >= 1");
  }
  if (!(fp_watchdog_readback_period_sec_ > 0.0)) {
    throw std::runtime_error("footprint_watchdog.readback_period_sec 必须 > 0");
  }
  // 🔴 算术守卫：凑齐 N 次连续回读**至少**需要 N 个回读周期。租约超时若比这还短，
  //    看门狗就可能在读数根本还凑不齐时动手 —— 那正是第一轮实测里
  //    2.0s 超时 vs 1.0s 回读周期造成误判的病因。留 1 个周期的抖动余量。
  const double min_lease =
    fp_watchdog_readback_period_sec_ * (fp_watchdog_consecutive_reads_ + 1);
  if (fp_watchdog_lease_timeout_sec_ < min_lease) {
    throw std::runtime_error(
      "footprint_watchdog.lease_timeout_sec(" +
      std::to_string(fp_watchdog_lease_timeout_sec_) + ") < readback_period_sec*(" +
      std::to_string(fp_watchdog_consecutive_reads_) + "+1)=" + std::to_string(min_lease) +
      " —— 连续读数还凑不齐就可能动手，拒绝启动");
  }
  // 🔴 陈旧阈值必须容得下至少 2 个回读周期，否则正常抖动就会让看门狗
  //    永远处于"读数太老、不敢动手"的瞎眼状态，等于没有兜底。
  if (fp_watchdog_readback_stale_sec_ < 2.0 * fp_watchdog_readback_period_sec_) {
    throw std::runtime_error(
      "footprint_watchdog.readback_stale_sec(" +
      std::to_string(fp_watchdog_readback_stale_sec_) + ") < 2*readback_period_sec(" +
      std::to_string(2.0 * fp_watchdog_readback_period_sec_) +
      ") —— 正常抖动就会让看门狗永久瞎眼，拒绝启动");
  }

  // ⚠️ 独立回调组：与 controlTick 共用互斥组会把这两个订阅饿死
  //    （本项目实测过订阅回调一次都执行不到、且全程无任何告警）。
  fp_watchdog_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = fp_watchdog_cb_group_;

  // 写话题：谁在请求换足迹。请求非默认足迹即视为一次"续租"。
  fp_write_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
    fp_watchdog_write_topic_, rclcpp::QoS(5),
    [this](geometry_msgs::msg::Polygon::SharedPtr msg) {
      if (msg->points.size() != fp_watchdog_default_vertices_) {
        fp_last_request_ns_.store(now().nanoseconds());
      }
    }, opts);

  // 回读话题：代价地图**当前**是什么足迹。只看顶点数 —— published_footprint
  // 是变换到机器人当前位姿的，坐标随位姿变而顶点数不变。
  // 连续计数在**这里**累加，不在 tick 里 —— tick 是 2Hz、回读是 1Hz，
  // 在 tick 里数会把同一个采样重复计两次（采样别名）。
  fp_readback_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    fp_watchdog_readback_topic_, rclcpp::QoS(1),
    [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
      const int n = static_cast<int>(msg->polygon.points.size());
      fp_current_vertices_.store(n);
      fp_last_readback_ns_.store(now().nanoseconds());
      if (n == static_cast<int>(fp_watchdog_default_vertices_)) {
        fp_nondefault_streak_.store(0);
      } else {
        fp_nondefault_streak_.fetch_add(1);
      }
    }, opts);

  fp_revert_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
    fp_watchdog_write_topic_, rclcpp::QoS(5));

  fp_watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), [this]() {footprintWatchdogTick();}, fp_watchdog_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "足迹锁存看门狗已启用: 写话题='%s' 回读='%s' 默认足迹 %zu 点。"
    "动手需三条同时成立: ①回读龄期<=%.1fs(活的) ②连续 %d 次回读均为非默认 "
    "③租约龄期>%.1fs。回读周期按 %.1fs 计，连续读数下限 %.1fs < 租约超时 ✓",
    fp_watchdog_write_topic_.c_str(), fp_watchdog_readback_topic_.c_str(),
    fp_watchdog_default_vertices_, fp_watchdog_readback_stale_sec_,
    fp_watchdog_consecutive_reads_, fp_watchdog_lease_timeout_sec_,
    fp_watchdog_readback_period_sec_,
    fp_watchdog_readback_period_sec_ * fp_watchdog_consecutive_reads_);
}

void ExplorationCoordinatorNode::footprintWatchdogTick()
{
  if (!fp_watchdog_enabled_ || !fp_revert_pub_) {
    return;
  }
  const int cur = fp_current_vertices_.load();
  if (cur == 0) {
    return;                 // 还没回读到，什么都不判（缺数据不等于异常）
  }
  if (cur == static_cast<int>(fp_watchdog_default_vertices_)) {
    return;                 // 已经是默认足迹，正常
  }

  // ---- 条件①：读数必须是**活的** ----
  // 🔴 冻结的读数不能当当前值用。回读停更时 cur 会永远停在最后那个非默认值，
  //    据此动手等于凭一张过期快照判定现状（本项目已犯过三次同类错）。
  const int64_t last_read = fp_last_readback_ns_.load();
  const double read_age = (last_read == 0) ?
    std::numeric_limits<double>::infinity() :
    static_cast<double>(now().nanoseconds() - last_read) / 1e9;
  if (read_age > fp_watchdog_readback_stale_sec_) {
    ++fp_watchdog_stale_skips_;
    // 必须显式上报：否则"看门狗没动手"会被读成"一切正常"，
    // 而真相是它已经瞎了、根本没有兜底。
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "🔴 足迹看门狗**瞎眼**: 最新回读已 %.1fs 未更新(>%.1fs)，"
      "最后读到的是 %d 顶点的非默认足迹。无法判定代价地图现状 ⇒ 本拍不动手。"
      "请查 '%s' 是否还在发布(global costmap 是否存活) | 累计放弃判定=%d 次",
      read_age, fp_watchdog_readback_stale_sec_, cur,
      fp_watchdog_readback_topic_.c_str(), fp_watchdog_stale_skips_);
    return;
  }

  // ---- 条件②：连续 N 次回读都是非默认 ----
  // 单次读数不足以动手：控制器侧切换本身有 ~1Hz 量级的节拍，
  // 单次命中可能只是"正在正常切换"的中间态。
  const int streak = fp_nondefault_streak_.load();
  if (streak < fp_watchdog_consecutive_reads_) {
    return;
  }

  // ---- 条件③：租约过期 ----
  const int64_t last = fp_last_request_ns_.load();
  const double age = (last == 0) ?
    std::numeric_limits<double>::infinity() :
    static_cast<double>(now().nanoseconds() - last) / 1e9;
  if (age <= fp_watchdog_lease_timeout_sec_) {
    return;                 // 有人还在续租，说明设它的进程活着
  }

  // 🔴 三条全成立：设小足迹的那个进程要么死了、要么卡住了。强制复原。
  std::vector<geometry_msgs::msg::Point> default_fp;
  (void)nav2_costmap_2d::makeFootprintFromString(fp_watchdog_default_footprint_, default_fp);
  geometry_msgs::msg::Polygon msg;
  msg.points.reserve(default_fp.size());
  for (const auto & pt : default_fp) {
    geometry_msgs::msg::Point32 p32;
    p32.x = static_cast<float>(pt.x);
    p32.y = static_cast<float>(pt.y);
    p32.z = 0.0F;
    msg.points.push_back(p32);
  }
  fp_revert_pub_->publish(msg);
  ++fp_watchdog_reverts_;
  // 复原后重置连续计数：下一次动手必须重新凑齐 N 次读数，
  // 否则同一批陈旧 streak 会让它连发好几拍。
  fp_nondefault_streak_.store(0);
  RCLCPP_ERROR(
    get_logger(),
    "🔴 足迹锁存看门狗介入(第 %d 次): 代价地图挂着 %d 顶点的非默认足迹"
    "(连续 %d 次回读确认，最新读数龄期 %.1fs)，而续租已停了 %.1fs(>%.1fs) "
    "⇒ 已强制发回 %zu 顶点的默认足迹。"
    "这说明设小足迹的进程死了或卡住了 —— 请查 controller_server 是否存活。"
    "在此之前规划器一直在用被低估的机器人尺寸做可通行判定。",
    fp_watchdog_reverts_, cur, streak, read_age, age, fp_watchdog_lease_timeout_sec_,
    fp_watchdog_default_vertices_);
}

void ExplorationCoordinatorNode::controlTick()

{
  // 整个 tick 在一把锁里完成：状态判断和状态修改之间不留缝隙，
  // 这是「禁止多线程重复下发」的第一道保险（第二道是 nav_goal_in_flight_）。
  std::lock_guard<std::mutex> lock(state_mutex_);

  // 来路轨迹全程记录（函数内部按 breadcrumb_sample_hz_ 节流）。
  // 放在 switch 之前而不是塞进某几个状态里：脱困可能在任何时刻被触发，
  // 而那时需要的是**之前**走过的路 —— 只在 NAVIGATING 记会漏掉自举/校验期的位移。
  // ESCAPE 期间不记：那时记的是脱困自己走的路，会污染「来路」的语义。
  if (state_ != ExplorationState::kEscape) {
    recordBreadcrumb();
  }

  switch (state_) {
    case ExplorationState::kIdle:
      tickIdle();
      break;
    case ExplorationState::kGenNextPoint:
      tickGenNextPoint();
      break;
    case ExplorationState::kBootstrap:
      tickBootstrap();
      break;
    case ExplorationState::kEscape:
      tickEscape();
      break;
    case ExplorationState::kValidating:
      tickValidating();
      break;
    case ExplorationState::kNavigating:
      tickNavigating();
      break;
    case ExplorationState::kArrived:
      tickArrived();
      break;
    case ExplorationState::kPaused:
      tickPaused();
      break;
    case ExplorationState::kCompleted:
      tickCompleted();
      break;
  }

  publishState();
}

void ExplorationCoordinatorNode::tickIdle()
{
  if (manually_paused_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "已被人工暂停，等待 ~/resume 服务");
    return;
  }
  // 自检：IDLE 态不应该有在途目标。若有（例如刚从异常态回退），先撤干净。
  if (nav_goal_in_flight_.load()) {
    cancelActiveNavGoal("IDLE 态发现残留在途目标");
    return;
  }

  std::string why;
  if (!mapReady(why)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "等待地图就绪: %s", why.c_str());
    // 「一直收不到地图」也可能是 SLAM 在等机器人先动一动（见 shouldBootstrap）。
    // 注意仍然要先等 bootstrap_trigger_wait_sec，不抢在 SLAM 前面动。
    std::string boot_why;
    if (shouldBootstrap("地图未就绪", boot_why)) {
      beginBootstrap(boot_why);
    }
    return;
  }
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (!robotPose(x, y, yaw, why)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "等待定位就绪: %s", why.c_str());
    return;
  }
  if (!odomReady(why)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "等待里程计就绪: %s", why.c_str());
    return;
  }
  // 地图收到了、但内容还不足以做决策（冷启动时全是未知格）：
  // 这时进 GEN_NEXT_POINT 只会因为「前沿格 0」被判成探索完成，必须先自举。
  {
    std::shared_ptr<GridMap> map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = latest_map_;
    }
    std::size_t known = 0U;
    if (map && !mapUsable(*map, known)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "地图已收到但内容不足以决策：已知格 %zu < %zu(min_known_cells_for_decision)",
        known, min_known_cells_for_decision_);
      std::string boot_why;
      if (shouldBootstrap("地图已知格不足以做探索决策", boot_why)) {
        beginBootstrap(boot_why);
      }
      return;
    }
  }
  transitionTo(ExplorationState::kGenNextPoint, "地图/定位/里程计均就绪");
}

// ======================== 冷启动自举 ========================
//
// 破的是这个死锁：slam_toolbox 只在机器人移动超过 minimum_travel_heading(0.2rad)
// 之后才插入新扫描 → 冷启动时地图没有已知格 → 本节点不下发目标 → 机器人不动
// → 地图不长 → 永远不动。上一轮验证是靠外部脚本发 cmd_vel 走 0.79m 破的
// （地图 0 → 26 m²），本节点把这一步收进来，做成有限次 + 有安全门 + 只旋转。

bool ExplorationCoordinatorNode::shouldBootstrap(
  const std::string & context, std::string & why)
{
  if (bootstrap_mode_ == BootstrapMode::kDisabled) {
    why = "自举已关闭";
    return false;
  }
  if (manually_paused_) {
    why = "人工暂停中";
    return false;              // 人工暂停期间绝不自己动
  }
  if (nav_goal_in_flight_.load()) {
    why = "有在途导航目标";
    return false;              // 与 Nav2 抢底盘是绝对不允许的
  }
  if (bootstrap_count_ >= bootstrap_max_attempts_) {
    // 达上限只记日志、不动，等人工介入。禁止死循环重试。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "自举已达上限 %d 次仍未能让地图可用，停止自举(最近一次结论: %s)",
      bootstrap_max_attempts_, bootstrap_last_result_.c_str());
    why = "自举次数已用尽";
    return false;
  }
  // 停滞计时：不是一发现地图不可用就动，先等 bootstrap_trigger_wait_sec。
  // 启动瞬态里地图本来就要几秒才长出来，立刻自举等于抢在 SLAM 前面动。
  const rclcpp::Time now_time = now();
  if (!bootstrap_stall_active_) {
    bootstrap_stall_active_ = true;
    bootstrap_stall_since_ = now_time;
    why = "停滞计时刚开始";
    return false;
  }
  const double stalled = (now_time - bootstrap_stall_since_).seconds();
  if (stalled < bootstrap_trigger_wait_sec_) {
    why = "停滞时长未达触发阈值";
    return false;
  }
  std::string safe_why;
  if (!bootstrapSafe(safe_why)) {
    // 被安全门拦下必须显式可见（禁止静默失败），但不消耗次数预算 ——
    // 拦下的原因（激光没来）可能几秒后就消失了。
    bootstrap_last_result_ = "被安全门拦下: " + safe_why;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "需要自举但安全门不允许运动: %s(不消耗次数预算)", safe_why.c_str());
    why = safe_why;
    return false;
  }
  why = context + "已持续 " + std::to_string(stalled) + "s";
  return true;
}

void ExplorationCoordinatorNode::beginBootstrap(const std::string & why)
{
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  std::string pose_why;
  // 起始朝向取不到就不进自举：没有它就无法在结束时实测「到底转了多少」，
  // 而那正是判断自举是否真的起作用的唯一依据（指令发出去 ≠ 机器人动了）。
  //
  // !!! 必须用 bootstrap_yaw_frame_（默认 odom）而不是 map !!!
  // 这一条是实测踩出来的：robotPose() 查的是 map -> base，而 map -> odom 由 SLAM 发布。
  // 真正的冷启动死锁下 SLAM 还没出图、也就没有那条 TF，于是自举被自己的前置条件挡住
  // （实测连续 71 次「需要自举但取不到当前朝向」），而自举本来就是为破这个死锁存在的。
  // odom -> base 来自底盘里程计，与 SLAM 无关；而且量一次原地小转本来就该在 odom 系里量
  //（map 系的 yaw 还含 SLAM 回环修正，会污染测量）。
  if (!poseInFrame(bootstrap_yaw_frame_, rx, ry, ryaw, pose_why)) {
    bootstrap_last_result_ = "无法取得起始朝向: " + pose_why;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "需要自举但取不到当前朝向(%s)，本次跳过", pose_why.c_str());
    return;
  }
  ++bootstrap_count_;
  bootstrap_start_yaw_ = ryaw;
  bootstrap_start_yaw_valid_ = true;
  bootstrap_started_time_ = now();
  bootstrap_stall_active_ = false;      // 转完回 IDLE 后重新开始计时
  bootstrap_last_result_ = "进行中";
  RCLCPP_WARN(
    get_logger(),
    "自举 %d/%d：原地旋转 %.2frad/s × %.1fs（触发原因: %s）。"
    "速度发往 %s，下游限速与 leash 保护照常生效",
    bootstrap_count_, bootstrap_max_attempts_, bootstrap_angular_vel_,
    bootstrap_duration_sec_, why.c_str(), bootstrap_cmd_vel_topic_.c_str());
  transitionTo(ExplorationState::kBootstrap, "自举: " + why);
}

bool ExplorationCoordinatorNode::bootstrapSafe(std::string & why)
{
  double min_range = -1.0;
  double age = 0.0;
  bool have = false;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    have = has_scan_;
    min_range = latest_scan_min_range_;
    if (have) {
      age = (now() - latest_scan_time_).seconds();
    }
  }
  if (!have) {
    // 关键规则：没有数据 **不等于** 安全。本项目已经在探针脚本上踩过一次
    // （RELIABLE 订阅收不到 BEST_EFFORT 的 scan，代码把「无数据」当「前方无障碍」，盲走 3m）。
    why = "尚未收到任何激光数据(无数据不得视为安全)";
    return false;
  }
  if (age > bootstrap_scan_timeout_sec_) {
    why = "激光已 " + std::to_string(age) + "s 未更新(>" +
      std::to_string(bootstrap_scan_timeout_sec_) + "s)";
    return false;
  }
  if (!(min_range > 0.0)) {
    why = "激光本帧没有任何有效回波";
    return false;
  }
  if (min_range < bootstrap_min_clearance_m_) {
    // 原地旋转本身只扫过内切半径(0.388)到外接半径(0.42)之间约 3.2cm 的环带，
    // 但如果最近障碍已经进到外接半径以内，八边形的顶点可能已经接触障碍，此时不该再转。
    why = "最近障碍 " + std::to_string(min_range) + "m < 要求净空 " +
      std::to_string(bootstrap_min_clearance_m_) + "m";
    return false;
  }
  why.clear();
  return true;
}

void ExplorationCoordinatorNode::publishBootstrapCmd(bool zero)
{
  if (!bootstrap_cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  // 只转不平移。这是 BootstrapMode::kRotate 的全部含义，不留平移分支。
  cmd.angular.z = zero ? 0.0 : bootstrap_angular_vel_;
  bootstrap_cmd_pub_->publish(cmd);
}

void ExplorationCoordinatorNode::bootstrapCmdTick()
{
  // 高频重发速度指令。底盘 cmd_vel_timeout_sec=0.5，靠 0.5s 的状态机节拍发
  // 正好卡在超时边界上，速度会被反复归零 —— 所以必须有这个独立定时器。
  std::lock_guard<std::mutex> lock(state_mutex_);
  // 只有 drivesChassisDirectly() 为真的状态才允许发速度 —— 唯一出处在
  // exploration_state.hpp，新增直驱状态时改那一处即可，这里不再各自列举。
  if (!drivesChassisDirectly(state_)) {
    return;                            // 非直驱态一帧都不发，避免与 Nav2 抢底盘
  }
  if (state_ == ExplorationState::kBootstrap) {
    publishBootstrapCmd(false);
  } else {
    publishEscapeCmd(escape_cmd_);
  }
}

// =====================================================================
// 膨胀带脱困（ESCAPE）
// =====================================================================

CellReading ExplorationCoordinatorNode::readRobotCell()
{
  CellReading r;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string why;
  if (!robotPose(x, y, yaw, why)) {
    return r;                          // 两个 valid 都是 false ⇒ 判 kDataInsufficient
  }

  // costmap：规划器视角，含膨胀。三态阈值 253 与 SmacPlanner2D 的
  // INSCRIBED 完全一致（见 costmap_adapter.hpp 的 lethal_cost_threshold）。
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (latest_costmap_ && latest_costmap_->consistent()) {
      unsigned int mx = 0U;
      unsigned int my = 0U;
      if (latest_costmap_->worldToMap(x, y, mx, my)) {
        r.costmap_tri = static_cast<int>(latest_costmap_->data[latest_costmap_->index(mx, my)]);
        r.costmap_valid = true;
      }
    }
  }
  // /map：物理真值，不含 nav2 膨胀。红线判据只认它。
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (latest_map_ && latest_map_->consistent()) {
      unsigned int mx = 0U;
      unsigned int my = 0U;
      if (latest_map_->worldToMap(x, y, mx, my)) {
        r.map_tri = static_cast<int>(latest_map_->data[latest_map_->index(mx, my)]);
        r.map_valid = true;
      }
    }
  }
  return r;
}

EscapeVerdict ExplorationCoordinatorNode::checkEscapeTrigger(std::string & why)
{
  if (!escape_enabled_) {
    why = "escape_enabled=false，不做脱困判定";
    return EscapeVerdict::kNone;
  }
  const CellReading reading = readRobotCell();

  // 起点致命计数单独维护：consecutive_invalid_ 混了目标侧失败，
  // 用它做触发阈值会把「目标不可达」也算进脱困的账上。
  if (reading.costmap_valid &&
    isPlannerLethal(reading.costmap_tri, EscapeGridThresholds{}))
  {
    ++start_lethal_failures_;
  } else {
    start_lethal_failures_ = 0;
  }

  EscapeTriggerConfig cfg;
  cfg.trigger_failures = escape_trigger_failures_;
  return evaluateEscapeTrigger(reading, start_lethal_failures_, cfg, why);
}

bool ExplorationCoordinatorNode::pickEscapeTarget(PlanarPoint & target, std::string & why)
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string pose_why;
  if (!robotPose(x, y, yaw, pose_why)) {
    why = "取不到机器人位姿: " + pose_why;
    return false;
  }
  const PlanarPoint robot{x, y};

  GridMap costmap_snapshot;
  GridMap map_snapshot;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!latest_costmap_ || !latest_costmap_->consistent()) {
      why = "costmap 不可用";
      return false;
    }
    costmap_snapshot = *latest_costmap_;
  }
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!latest_map_ || !latest_map_->consistent()) {
      why = "/map 不可用";
      return false;
    }
    map_snapshot = *latest_map_;
  }

  EscapeSearchConfig cfg;
  cfg.search_radius_m = escape_search_radius_m_;
  cfg.heading_tol_rad = escape_heading_tol_rad_;

  // 一级：反向沿来路。机器人是自己开进来的 ⇒ 来路可通行是**可证明的**。
  std::string bc_why;
  if (pickBreadcrumbTarget(
      breadcrumbs_, robot, costmap_snapshot, map_snapshot, cfg, target, bc_why))
  {
    why = bc_why;
    return true;
  }
  RCLCPP_INFO(get_logger(), "来路不可用：%s", bc_why.c_str());

  // 二级：最近可规划格。不用代价梯度 —— 实测足迹代价在窄于 1.62m 的通道里
  // 恒为 253、零梯度；而「三态非致命」是规划器会接受起点的**充分**条件。
  double ref_heading = yaw;
  if (!candidates_.empty() && candidate_index_ < candidates_.size()) {
    const auto & c = candidates_[candidate_index_];
    ref_heading = std::atan2(c.y - y, c.x - x);   // 朝当前候选前沿点的方位
  }
  bool relaxed = false;
  std::string nn_why;
  if (!findNearestPlannableCell(
      robot, ref_heading, costmap_snapshot, map_snapshot, cfg, target, relaxed, nn_why))
  {
    why = nn_why;
    return false;
  }
  if (relaxed) {
    // 放开方向约束必须显式可见：这意味着机器人可能朝任务反方向退。
    RCLCPP_WARN(get_logger(), "脱困已放开方向约束：%s", nn_why.c_str());
  }
  why = nn_why;
  return true;
}

void ExplorationCoordinatorNode::publishEscapeCmd(const EscapeCommand & cmd)
{
  if (!bootstrap_cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = cmd.vx;
  t.linear.y = cmd.vy;
  t.linear.z = 0.0;
  t.angular.x = 0.0;
  t.angular.y = 0.0;
  t.angular.z = cmd.wz;
  bootstrap_cmd_pub_->publish(t);
}

void ExplorationCoordinatorNode::recordBreadcrumb()
{
  if (!escape_enabled_) {
    return;
  }
  const rclcpp::Time t = now();
  if (last_breadcrumb_time_.nanoseconds() != 0) {
    const double dt = (t - last_breadcrumb_time_).seconds();
    if (dt < (1.0 / breadcrumb_sample_hz_)) {
      return;
    }
  }
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string why;
  if (!robotPose(x, y, yaw, why)) {
    return;                            // 取不到位姿就不记，不记假点
  }
  last_breadcrumb_time_ = t;
  breadcrumbs_.push_back(Breadcrumb{PlanarPoint{x, y}, t.seconds()});

  // 裁掉过期的。用时间窗而不是固定条数：采样频率可配，条数窗会随频率漂移。
  const double cutoff = t.seconds() - breadcrumb_window_sec_;
  std::size_t drop = 0U;
  while (drop < breadcrumbs_.size() && breadcrumbs_[drop].stamp_sec < cutoff) {
    ++drop;
  }
  if (drop > 0U) {
    breadcrumbs_.erase(breadcrumbs_.begin(), breadcrumbs_.begin() + static_cast<long>(drop));
  }
}

void ExplorationCoordinatorNode::tickEscape()
{
  const double elapsed = (now() - escape_started_time_).seconds();

  // ---- 每拍复查红线。不是只在入口查一次 ----
  // 脱困过程中 SLAM 可能新插入一帧扫描，把机器人所在格标成占据。
  // 那一刻必须立刻停，不能沿用入口时的判断。
  const CellReading reading = readRobotCell();
  if (reading.map_valid && isPhysicallyOccupied(reading.map_tri, EscapeGridThresholds{})) {
    escape_cmd_ = EscapeCommand{};
    publishEscapeCmd(escape_cmd_);     // 立刻显式发零速，不是"停止发布"
    escape_last_result_ = "脱困中途 /map 判占据(物理真堵)";
    RCLCPP_ERROR(
      get_logger(), "🔴 脱困中途检测到物理真堵(%.1fs)，已发零速停车并暂停", elapsed);
    transitionTo(ExplorationState::kPaused, "脱困中途物理真堵");
    return;
  }

  // ---- 安全门持续生效（复用自举那一套激光判据）----
  std::string safe_why;
  if (!bootstrapSafe(safe_why)) {
    escape_cmd_ = EscapeCommand{};
    publishEscapeCmd(escape_cmd_);
    escape_last_result_ = "被安全门中断: " + safe_why;
    RCLCPP_WARN(
      get_logger(), "脱困被安全门中断(%.1fs): %s，已发零速停车", elapsed, safe_why.c_str());
    transitionTo(ExplorationState::kPaused, "脱困被安全门中断");
    return;
  }

  // ---- 出带判定 ----
  if (reading.costmap_valid &&
    !isPlannerLethal(reading.costmap_tri, EscapeGridThresholds{}))
  {
    ++escape_clear_streak_;
  } else {
    escape_clear_streak_ = 0;
  }
  if (escapeCleared(escape_clear_streak_, escape_clear_ticks_)) {
    escape_cmd_ = EscapeCommand{};
    publishEscapeCmd(escape_cmd_);
    escape_target_valid_ = false;
    start_lethal_failures_ = 0;
    escape_last_result_ = "成功出带";
    RCLCPP_INFO(
      get_logger(),
      "脱困成功：连续 %d 拍读到非致命，耗时 %.1fs。回 GEN_NEXT_POINT 重新校验",
      escape_clear_ticks_, elapsed);
    transitionTo(ExplorationState::kGenNextPoint, "脱困成功，起点已可规划");
    return;
  }

  // ---- 超时 ----
  if (elapsed > escape_timeout_sec_) {
    escape_cmd_ = EscapeCommand{};
    publishEscapeCmd(escape_cmd_);
    escape_last_result_ = "超时未出带";
    RCLCPP_ERROR(
      get_logger(), "脱困超时(%.1fs > %.1fs)仍未出带，已停车并暂停",
      elapsed, escape_timeout_sec_);
    transitionTo(ExplorationState::kPaused, "脱困超时");
    return;
  }

  // ---- 产生本拍速度 ----
  if (!escape_target_valid_) {
    escape_cmd_ = EscapeCommand{};
    publishEscapeCmd(escape_cmd_);
    escape_last_result_ = "无有效目标";
    transitionTo(ExplorationState::kPaused, "脱困目标失效");
    return;
  }
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string pose_why;
  if (!robotPose(x, y, yaw, pose_why)) {
    escape_cmd_ = EscapeCommand{};
    publishEscapeCmd(escape_cmd_);     // 位姿丢了立刻停，不靠旧位姿盲走
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "脱困期间取不到位姿(%s)，本拍发零速", pose_why.c_str());
    return;
  }

  EscapeLimits lim;
  lim.max_linear = escape_linear_vel_;
  lim.max_angular = escape_angular_vel_;
  lim.arrive_tol_m = escape_arrive_tol_m_;
  lim.align_tol_rad = escape_align_tol_rad_;
  escape_cmd_ = escapeVelocity(PlanarPoint{x, y}, yaw, escape_target_, lim);
  publishEscapeCmd(escape_cmd_);

  if (escape_cmd_.arrived) {
    // 走到本段目标却仍未出带：重新选一个目标继续，而不是判失败 ——
    // 一次挪动 0.05m 量级，可能需要几段才出带。超时保护仍然兜着。
    std::string tgt_why;
    if (pickEscapeTarget(escape_target_, tgt_why)) {
      RCLCPP_INFO(
        get_logger(), "脱困本段到达但仍在带内，换下一段目标(%.2f, %.2f)：%s",
        escape_target_.x, escape_target_.y, tgt_why.c_str());
    } else {
      escape_target_valid_ = false;
      RCLCPP_WARN(get_logger(), "脱困本段到达但选不出下一段目标：%s", tgt_why.c_str());
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kLogThrottleMs,
    "脱困中 %.1fs/%.1fs：目标(%.2f, %.2f) 速度(%.3f, %.3f, %.3f) 出带连击 %d/%d",
    elapsed, escape_timeout_sec_, escape_target_.x, escape_target_.y,
    escape_cmd_.vx, escape_cmd_.vy, escape_cmd_.wz,
    escape_clear_streak_, escape_clear_ticks_);
}

void ExplorationCoordinatorNode::tickBootstrap()
{
  const double elapsed = (now() - bootstrap_started_time_).seconds();

  // 安全门在整个自举过程中持续生效，不是只在入口查一次。
  std::string safe_why;
  if (!bootstrapSafe(safe_why)) {
    publishBootstrapCmd(true);         // 立刻显式发零速，不是"停止发布"
    bootstrap_last_result_ = "中途被安全门中断: " + safe_why;
    RCLCPP_WARN(
      get_logger(), "自举中途被安全门中断(%.1fs): %s，已发零速停车", elapsed, safe_why.c_str());
    transitionTo(ExplorationState::kIdle, "自举被安全门中断");
    return;
  }

  if (elapsed < bootstrap_duration_sec_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "自举旋转中 %.1fs/%.1fs", elapsed, bootstrap_duration_sec_);
    return;
  }

  // 时间到：先停车，再实测到底转了多少。
  publishBootstrapCmd(true);

  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  std::string pose_why;
  if (!bootstrap_start_yaw_valid_ ||
    !poseInFrame(bootstrap_yaw_frame_, rx, ry, ryaw, pose_why))
  {
    bootstrap_last_result_ = "无法实测转角: " + pose_why;
    RCLCPP_WARN(get_logger(), "自举结束但取不到当前朝向(%s)，无法确认是否真的转动", pose_why.c_str());
    transitionTo(ExplorationState::kIdle, "自举结束(转角未知)");
    return;
  }
  const double turned = std::fabs(normalizeAngle(ryaw - bootstrap_start_yaw_));
  bootstrap_start_yaw_valid_ = false;

  if (turned < bootstrap_min_yaw_delta_) {
    // 「指令发出去了」不等于「机器人动了」——这一条必须实测，且达不到时要显式告警，
    // 否则自举会静默地什么也没做，而现象仍然是「机器人不动」，排查会绕回原点。
    bootstrap_last_result_ = "实测仅转 " + std::to_string(turned) + "rad，未达阈值";
    RCLCPP_ERROR(
      get_logger(),
      "自举 %d/%d 实际只转了 %.3frad < %.3frad(slam_toolbox minimum_travel_heading)，"
      "SLAM 不会插入新扫描、地图不会增长。最可能的原因是下游限速："
      "臂-底盘耦合节点 min_speed_scale=0.15 会把 %.2frad/s 缩到 %.3frad/s。"
      "排查顺序：先 echo %s 看下发值，再 echo /cmd_vel 看缩放后的值",
      bootstrap_count_, bootstrap_max_attempts_, turned, bootstrap_min_yaw_delta_,
      bootstrap_angular_vel_, bootstrap_angular_vel_ * 0.15,
      bootstrap_cmd_vel_topic_.c_str());
  } else {
    bootstrap_last_result_ = "成功转出 " + std::to_string(turned) + "rad";
    RCLCPP_INFO(
      get_logger(), "自举 %d/%d 完成：实测转出 %.3frad(>=%.3f)，等待 SLAM 插入新扫描",
      bootstrap_count_, bootstrap_max_attempts_, turned, bootstrap_min_yaw_delta_);
  }
  transitionTo(ExplorationState::kIdle, "自举动作结束");
}

void ExplorationCoordinatorNode::tickGenNextPoint()
{
  // 冗余但必要的断言式检查：生成目标点这件事只允许在 kGenNextPoint 发生。
  if (!allowsGoalGeneration(state_)) {
    RCLCPP_ERROR(get_logger(), "在 %s 态被要求生成目标点，已拦截", toString(state_));
    transitionTo(ExplorationState::kIdle, "非法的生成时机");
    return;
  }
  // 严格单点推进：上一目标还在途就绝不生成下一个。
  if (nav_goal_in_flight_.load()) {
    RCLCPP_ERROR(
      get_logger(), "检测到在途导航目标却进入了生成态，拦截并等待其结束");
    transitionTo(ExplorationState::kNavigating, "存在在途目标，纠正状态");
    return;
  }

  std::string why;
  if (!mapReady(why)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "地图不可用，暂缓生成: %s", why.c_str());
    transitionTo(ExplorationState::kIdle, "地图不可用");
    return;
  }
  if (!odomReady(why)) {
    RCLCPP_WARN(get_logger(), "里程计丢失，冻结目标生成: %s", why.c_str());
    transitionTo(ExplorationState::kPaused, "里程计丢失");
    return;
  }
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  if (!robotPose(rx, ry, ryaw, why)) {
    RCLCPP_WARN(get_logger(), "定位丢失，冻结目标生成: %s", why.c_str());
    transitionTo(ExplorationState::kPaused, "定位丢失");
    return;
  }

  std::shared_ptr<GridMap> map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = latest_map_;
  }
  if (!map) {
    transitionTo(ExplorationState::kIdle, "地图快照为空");
    return;
  }

  std::size_t raw_frontier_cells = 0U;
  auto candidates = generateCandidates(*map, rx, ry, raw_frontier_cells);

  // 关键区分（写反了会让上层提前停止探索）：
  //   一个前沿格都没有 ⇒ 真的探索完了
  //   有前沿格但候选点全不合法 ⇒ 只是暂时找不到合法点，属异常暂停
  if (raw_frontier_cells == 0U) {
    // …但还有第三种，只看前沿格数会漏：**地图本身还几乎全是未知**。
    // 冷启动时地图收到了、自洽、不超时（mapReady 全部通过），可是一个已知格都没有，
    // 于是前沿格数也是 0 —— 若直接判 COMPLETED，探索会在真正开始前就"完成"。
    std::size_t known = 0U;
    if (!mapUsable(*map, known)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "前沿格 0 且地图已知格仅 %zu(<%zu) —— 判为「还没开始」而不是「探索完成」",
        known, min_known_cells_for_decision_);
      std::string boot_why;
      if (shouldBootstrap("前沿格为 0 且地图几乎全是未知", boot_why)) {
        beginBootstrap(boot_why);
      } else {
        transitionTo(ExplorationState::kIdle, "地图内容不足，等待自举或人工介入");
      }
      return;
    }
    exploration_complete_ = true;
    transitionTo(ExplorationState::kCompleted, "地图内已无任何前沿格，探索完成");
    return;
  }
  if (candidates.empty()) {
    std::string ready_why;
    if (!stackReady(ready_why)) {
      // 栈没起齐时采不到候选是必然的，不该消耗预算（实测这正是一次死锁的成因）。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "未采到合法候选点，但栈尚未就绪(%s) —— 本次不计入连续失败", ready_why.c_str());
      return;
    }
    const bool budget_used_up = failure_budget_.onNoCandidateSampled();
    RCLCPP_WARN(
      get_logger(),
      "本轮未采到合法候选点(前沿格 %zu 个仍存在)，连续采样失败 %d/%d",
      raw_frontier_cells, failure_budget_.sample_failures,
      failure_budget_.max_sample_failures);
    if (budget_used_up) {
      // 转 PAUSED 之前先给自举一个机会：原地转一圈往往能把「贴着未知区、
      // 净空判定卡在边界上」的候选点变成合法。次数用完了才真的 PAUSED。
      std::string boot_why;
      if (shouldBootstrap("连续多轮采不到合法候选点", boot_why)) {
        beginBootstrap(boot_why);
        return;
      }
      transitionTo(ExplorationState::kPaused, "连续多轮无合法候选点");
    }
    return;
  }

  // 能采到候选就说明地图内容已经够用了：清掉自举的停滞计时与次数预算。
  bootstrap_stall_active_ = false;
  bootstrap_count_ = 0;
  // 只清**采样**预算。校验预算刻意不在这里清 —— 「采样->校验全废->重采样」
  // 循环里每轮都会走到这一行，清了就等于把校验预算的上限废掉（实测空转 400s）。
  failure_budget_.onCandidatesSampled();
  candidates_ = std::move(candidates);
  candidate_index_ = 0U;
  RCLCPP_INFO(
    get_logger(), "生成 %zu 个候选点(前沿格 %zu)，开始逐个做未知区路径校验",
    candidates_.size(), raw_frontier_cells);
  transitionTo(ExplorationState::kValidating, "候选点已生成");
  requestPlanForCurrentCandidate();
}

void ExplorationCoordinatorNode::tickValidating()
{
  if (!plan_request_in_flight_.load()) {
    // 校验请求不在途：可能是 action server 当时没就绪，重试一次。
    requestPlanForCurrentCandidate();
    return;
  }
  const double waited = (now() - plan_requested_time_).seconds();
  if (waited > plan_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "路径校验请求超时 %.1fs，丢弃当前候选", waited);
    if (plan_client_ && plan_goal_handle_) {
      plan_client_->async_cancel_goal(plan_goal_handle_);
    }
    plan_goal_handle_.reset();
    plan_request_in_flight_.store(false);
    rejectCurrentCandidate("路径校验超时");
  }
}

void ExplorationCoordinatorNode::tickNavigating()
{
  // 一致性自检：导航态必须有在途目标，否则说明结果回调丢了。
  if (!nav_goal_in_flight_.load()) {
    RCLCPP_ERROR(get_logger(), "导航态却无在途目标，回退安全态重新调度");
    has_active_goal_ = false;
    transitionTo(ExplorationState::kIdle, "导航态与在途标志不一致");
    return;
  }

  std::string why;
  if (!odomReady(why)) {
    RCLCPP_WARN(get_logger(), "导航中里程计丢失: %s", why.c_str());
    cancelActiveNavGoal("里程计丢失");
    transitionTo(ExplorationState::kPaused, "导航中里程计丢失");
    return;
  }
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  if (!robotPose(rx, ry, ryaw, why)) {
    RCLCPP_WARN(get_logger(), "导航中定位丢失: %s", why.c_str());
    cancelActiveNavGoal("定位丢失");
    transitionTo(ExplorationState::kPaused, "导航中定位丢失");
    return;
  }

  const double elapsed = (now() - nav_started_time_).seconds();
  if (elapsed > nav_timeout_sec_) {
    RCLCPP_WARN(
      get_logger(), "导航超时 %.1fs > %.1fs，撤销当前目标", elapsed, nav_timeout_sec_);
    cancelActiveNavGoal("导航超时");
    registerNavFailure("导航超时");
    return;
  }

  // follow_path 模式下 BT 的 1Hz 重规划拿不到了，由这里周期性补上。
  maybeRequestReplan(now());

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kLogThrottleMs,
    "导航中 %.1fs/%.1fs 目标(%.2f, %.2f) 当前(%.2f, %.2f) 剩余 %.2fm",
    elapsed, nav_timeout_sec_, active_goal_.x, active_goal_.y, rx, ry,
    std::hypot(active_goal_.x - rx, active_goal_.y - ry));
}

void ExplorationCoordinatorNode::tickArrived()
{
  std::string why;
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  if (!robotPose(rx, ry, ryaw, why)) {
    RCLCPP_WARN(get_logger(), "抵达校验期间定位丢失: %s", why.c_str());
    transitionTo(ExplorationState::kPaused, "抵达校验期间定位丢失");
    return;
  }

  const double dxy = std::hypot(active_goal_.x - rx, active_goal_.y - ry);
  if (dxy > arrival_xy_tolerance_) {
    // Nav2 报了成功，实测位置却不达标 —— 不能当成抵达，否则「抵达校验」形同虚设。
    RCLCPP_WARN(
      get_logger(), "Nav2 报成功但位置偏差 %.2fm > 容差 %.2fm，按导航失败处理",
      dxy, arrival_xy_tolerance_);
    dwell_active_ = false;
    registerNavFailure("抵达位置偏差超容差");
    return;
  }
  if (check_yaw_) {
    const double dyaw = std::fabs(normalizeAngle(active_goal_.yaw - ryaw));
    if (dyaw > arrival_yaw_tolerance_) {
      RCLCPP_WARN(
        get_logger(), "朝向偏差 %.2frad > 容差 %.2frad，按导航失败处理",
        dyaw, arrival_yaw_tolerance_);
      dwell_active_ = false;
      registerNavFailure("抵达朝向偏差超容差");
      return;
    }
  }

  double speed = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    speed = odom_speed_;
  }
  if (speed > settle_speed_) {
    // 位置到了但还在动，说明还没稳住：重新计时，不允许「路过即算抵达」。
    dwell_active_ = false;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "位置已达标但速度 %.3f > %.3f m/s，驻留计时重置", speed, settle_speed_);
    return;
  }

  if (!dwell_active_) {
    dwell_active_ = true;
    dwell_started_time_ = now();
    RCLCPP_INFO(get_logger(), "开始驻留计时(偏差 %.2fm 速度 %.3fm/s)", dxy, speed);
    return;
  }
  const double dwelled = (now() - dwell_started_time_).seconds();
  if (dwelled < dwell_time_sec_) {
    return;                                   // 还没稳够，继续等
  }

  // 到这里才算「完全抵达 + 稳定驻留 + 目标收敛」，才允许生成下一个探索点。
  ++goals_succeeded_;
  nav_failure_count_ = 0;
  auto_resume_count_ = 0;
  recordVisit(active_goal_.x, active_goal_.y);
  has_active_goal_ = false;
  dwell_active_ = false;
  resetCycleState();
  RCLCPP_INFO(
    get_logger(),
    "目标收敛完成(偏差 %.2fm 驻留 %.2fs)，累计成功 %" PRIu64 " 个，允许生成下一目标",
    dxy, dwelled, goals_succeeded_);
  transitionTo(ExplorationState::kGenNextPoint, "当前目标已完全收敛");
}

void ExplorationCoordinatorNode::tickPaused()
{
  if (manually_paused_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "人工暂停中，目标发布已冻结，等待 ~/resume");
    return;
  }
  const double held = (now() - state_entered_time_).seconds();
  if (held < pause_cooldown_sec_) {
    return;
  }
  // 禁止死循环重试：自动恢复有次数上限，超限后只等人工介入。
  if (auto_resume_count_ >= max_auto_resume_attempts_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "自动恢复已达上限 %d 次，停止重试，等待人工调用 ~/resume 服务",
      max_auto_resume_attempts_);
    return;
  }
  // 栈还没起齐时的恢复尝试不消耗预算：否则启动阶段几次瞬态就把预算用完，
  // 之后即使一切正常也只能等人工 ~/resume。
  std::string ready_why;
  const bool ready = stackReady(ready_why);
  if (ready) {
    ++auto_resume_count_;
  }
  failure_budget_.resetAll();
  nav_failure_count_ = 0;
  resetCycleState();
  if (ready) {
    RCLCPP_WARN(
      get_logger(), "暂停 %.1fs 后尝试第 %d/%d 次自动恢复",
      held, auto_resume_count_, max_auto_resume_attempts_);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "暂停 %.1fs 后恢复；栈尚未就绪(%s)，本次不消耗恢复预算(仍为 %d/%d)",
      held, ready_why.c_str(), auto_resume_count_, max_auto_resume_attempts_);
  }
  transitionTo(ExplorationState::kIdle, "自动恢复");
}

void ExplorationCoordinatorNode::tickCompleted()
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kLogThrottleMs,
    "探索完成，已停止生成新目标(成功 %" PRIu64 " 个目标)", goals_succeeded_);
}

// ================== 候选生成与未知区域禁行校验 ==================

std::vector<ExplorationCoordinatorNode::GoalCandidatePose>
ExplorationCoordinatorNode::generateCandidates(
  const GridMap & map, double rx, double ry, std::size_t & raw_frontier_cells)
{
  std::vector<GoalCandidatePose> out;
  raw_frontier_cells = 0U;
  if (!map.consistent()) {
    RCLCPP_ERROR(get_logger(), "地图快照不自洽，拒绝生成候选点");
    return out;
  }

  FrontierSearch::Result result;
  search_.search(map, rx, ry, visit_history_, result);
  raw_frontier_cells = result.raw_frontier_cell_count;

  // 前沿块被否的原因做个直方图。「有前沿格但 0 个前沿块通过过滤」时，
  // 光看总数完全判断不出是哪条约束太紧（面积？可达性？距离？），
  // 而这恰好是现场最常见的卡点，所以这段统计常开而不是藏在 debug 里。
  if (result.accepted_cluster_count == 0U && !result.clusters.empty()) {
    std::vector<std::pair<std::string, std::size_t>> reason_hist;
    for (const auto & cl : result.clusters) {
      if (cl.accepted) {
        continue;
      }
      auto it = std::find_if(
        reason_hist.begin(), reason_hist.end(),
        [&cl](const std::pair<std::string, std::size_t> & e) {return e.first == cl.reject_reason;});
      if (it == reason_hist.end()) {
        reason_hist.emplace_back(cl.reject_reason, 1U);
      } else {
        ++it->second;
      }
    }
    std::string hist;
    for (const auto & e : reason_hist) {
      if (!hist.empty()) {
        hist += ", ";
      }
      hist += e.first + " x" + std::to_string(e.second);
    }
    RCLCPP_WARN(
      get_logger(), "全部 %zu 个前沿块被否，原因分布: %s",
      result.clusters.size(), hist.c_str());
  }

  // 校验1（目标点）就在这里做：前沿搜索只保证「不在膨胀障碍里」，
  // 它不保证目标点净空半径内没有未知格。未知区禁行是本节点自己的责任。
  //
  // 这里用的是**校验图**（默认 global_costmap），不是传进来的 /map。
  // 两张图对「占据」的定义并不一致，只用 /map 筛会让候选队列装满
  // 「在 /map 里空闲、但在 costmap 里是致命区」的点，每一个都要白跑一次
  // ComputePathToPose 才在下发前复检时被否掉。
  // 实测（净空半径校准到 0.25 之后）：536 次目标复检否决里 514 次（95.9%）
  // 是「目标格在 costmap 里代价=100(致命)」，正是这一类。
  //
  // 前沿**搜索**仍然必须用 /map —— 前沿的定义依赖「未知」这个状态。
  // 分工不变：/map 找前沿，costmap 判可站。
  std::string screen_why;
  const std::shared_ptr<GridMap> screen_grid = validationGrid(screen_why);
  if (!screen_grid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "候选筛选所需的校验图不可用(%s)，本轮不产出候选(绝不用错误的图放行目标)",
      screen_why.c_str());
    return out;
  }

  std::size_t dropped_unknown = 0U;
  std::size_t dropped_by_search = 0U;
  std::string search_reject_sample;
  std::string goal_reject_sample;
  for (const auto & c : result.candidates) {
    if (!c.valid) {
      // 前沿搜索自己否掉的候选。必须能看到原因，否则「候选 0 个」这种现象
      // 只能靠猜是哪一条约束太紧。
      ++dropped_by_search;
      if (search_reject_sample.empty()) {
        search_reject_sample = c.reject_reason;
      }
      RCLCPP_DEBUG(
        get_logger(), "候选点(%.2f, %.2f) 被前沿搜索否决: %s",
        c.x, c.y, c.reject_reason.c_str());
      continue;
    }
    const ValidationResult vr = validator_.validateGoal(*screen_grid, c.x, c.y);
    if (!vr.valid) {
      ++dropped_unknown;
      if (goal_reject_sample.empty()) {
        goal_reject_sample = vr.reason;
      }
      RCLCPP_DEBUG(
        get_logger(), "候选点(%.2f, %.2f) 目标校验未通过: %s", c.x, c.y, vr.reason.c_str());
      continue;
    }
    GoalCandidatePose g;
    g.x = c.x;
    g.y = c.y;
    g.yaw = c.yaw;
    g.cost = c.cost;
    out.push_back(g);
  }

  std::sort(
    out.begin(), out.end(),
    [](const GoalCandidatePose & a, const GoalCandidatePose & b) {return a.cost < b.cost;});
  if (out.size() > static_cast<std::size_t>(max_candidates_per_cycle_)) {
    out.resize(static_cast<std::size_t>(max_candidates_per_cycle_));
  }

  RCLCPP_INFO(
    get_logger(),
    "前沿搜索: 原始前沿格 %zu, 有效前沿块 %zu, 候选 %zu 个"
    "(搜索否决 %zu, 目标校验淘汰 %zu) | %s",
    result.raw_frontier_cell_count, result.accepted_cluster_count, out.size(),
    dropped_by_search, dropped_unknown, result.summary.c_str());
  if (out.empty() && dropped_by_search > 0U) {
    // 「有前沿格但一个候选都留不下」几乎总是某条距离/净空约束设得太紧，
    // 把首个否决原因抬到 WARN，省得每次都要开 debug 重跑一遍。
    RCLCPP_WARN(
      get_logger(), "全部 %zu 个候选被前沿搜索否决，首个原因: %s",
      dropped_by_search, search_reject_sample.c_str());
  }
  if (out.empty() && dropped_unknown > 0U) {
    // 同理：被目标校验（在 costmap 上）淘汰光时也要给出首个原因。
    // 最常见的两条是「目标格在 costmap 里是致命区」（前沿贴墙，被膨胀吃掉）
    // 和「净空半径内有占据格」（净空半径与控制器容差不匹配）。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "全部 %zu 个候选被目标校验淘汰，首个原因: %s",
      dropped_unknown, goal_reject_sample.c_str());
  }
  return out;
}

void ExplorationCoordinatorNode::requestPlanForCurrentCandidate()
{
  if (state_ != ExplorationState::kValidating) {
    RCLCPP_ERROR(get_logger(), "在 %s 态被要求发起路径校验，已拦截", toString(state_));
    return;
  }
  if (candidate_index_ >= candidates_.size()) {
    onCandidatesExhausted("候选队列已全部尝试完毕");
    return;
  }
  if (!plan_client_ || !plan_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "全局规划动作 %s 尚未就绪，等待 Nav2 起来", plan_action_name_.c_str());
    return;
  }
  // 同一候选只允许有一个校验请求在途。
  bool expected = false;
  if (!plan_request_in_flight_.compare_exchange_strong(expected, true)) {
    return;
  }

  const GoalCandidatePose & c = candidates_[candidate_index_];
  ComputePathToPose::Goal goal;
  goal.goal.header.frame_id = map_frame_;
  goal.goal.header.stamp = now();
  goal.goal.pose.position.x = c.x;
  goal.goal.pose.position.y = c.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, c.yaw);
  goal.goal.pose.orientation.x = q.x();
  goal.goal.pose.orientation.y = q.y();
  goal.goal.pose.orientation.z = q.z();
  goal.goal.pose.orientation.w = q.w();
  goal.use_start = false;                      // 让 Nav2 用机器人当前实际位姿作起点
  goal.planner_id = planner_id_;

  rclcpp_action::Client<ComputePathToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](const PlanGoalHandle::SharedPtr & handle) {onPlanGoalResponse(handle);};
  opts.result_callback =
    [this](const PlanGoalHandle::WrappedResult & result) {onPlanResult(result);};
  plan_client_->async_send_goal(goal, opts);
  plan_requested_time_ = now();
  RCLCPP_INFO(
    get_logger(), "校验候选 %zu/%zu: 请求到 (%.2f, %.2f) 的全局路径",
    candidate_index_ + 1U, candidates_.size(), c.x, c.y);
}

void ExplorationCoordinatorNode::onPlanGoalResponse(const PlanGoalHandle::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!handle) {
    plan_request_in_flight_.store(false);
    RCLCPP_WARN(get_logger(), "全局规划请求被 Nav2 拒绝，丢弃当前候选");
    rejectCurrentCandidate("规划请求被拒绝");
    return;
  }
  plan_goal_handle_ = handle;
}

void ExplorationCoordinatorNode::onPlanResult(const PlanGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  plan_request_in_flight_.store(false);
  plan_goal_handle_.reset();

  // ---- 分支一：这次规划是「跟踪中的周期性重规划」，不是候选校验 ----
  // 两者共用 plan_client_，靠 plan_request_is_replan_ 区分。
  if (plan_request_is_replan_) {
    plan_request_is_replan_ = false;
    if (state_ != ExplorationState::kNavigating) {
      return;                       // 已经不在跟踪了，丢弃
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->path.poses.empty())
    {
      // 重规划请求本身没成。当前路径若还能走就沿用；若已判死则由收口函数计次。
      onReplanProducedNoUsablePath("全局规划失败或返回空路径");
      return;
    }
    // 新路径同样要过双层校验 —— 否则重规划会变成绕过校验的后门。
    std::string why;
    const std::shared_ptr<GridMap> grid = validationGrid(why);
    if (!grid) {
      onReplanProducedNoUsablePath("校验图不可用: " + why);
      return;
    }
    std::vector<PlanarPoint> pts;
    pts.reserve(result.result->path.poses.size());
    for (const auto & p : result.result->path.poses) {
      pts.push_back(PlanarPoint{p.pose.position.x, p.pose.position.y});
    }
    const PlanarPoint requested{active_goal_.x, active_goal_.y};
    const ValidationResult chk = validator_.validatePath(*grid, pts, requested);
    if (!chk.valid) {
      onReplanProducedNoUsablePath("新路径未通过校验: " + chk.reason);
      return;
    }
    // 换路径：直接发新的 FollowPath 目标，controller_server 会抢占旧的。
    // 注意这里**不**走 dispatchFollowPath —— 那个函数带「只允许在 VALIDATING
    // 下发」的时序锁，而这里本来就在 NAVIGATING，且不该增加 goals_dispatched_。
    if (follow_client_ && follow_client_->action_server_is_ready()) {
      FollowPath::Goal fp;
      fp.path = result.result->path;
      fp.controller_id = follow_controller_id_;
      fp.goal_checker_id = follow_goal_checker_id_;
      rclcpp_action::Client<FollowPath>::SendGoalOptions o;
      o.goal_response_callback =
        [this](const FollowGoalHandle::SharedPtr & h) {onFollowGoalResponse(h);};
      o.result_callback =
        [this](const FollowGoalHandle::WrappedResult & r) {onFollowResult(r);};
      // 新目标的 id 会在 onFollowGoalResponse 里覆盖 current_follow_goal_id_，
      // 在那之前旧 id 仍然有效；旧目标被抢占后的结果靠 id 比对被忽略。
      follow_client_->async_send_goal(fp, o);
      // 换上新路径后必须同步更新记录 —— 否则"剩余段是否还能走"永远在校验那条
      // 已经被替换掉的旧路径，判据会一直误判、每轮都触发重规划。
      active_path_ = pts;
      active_path_time_ = now();
      // 换成了一条通过校验的新路径：判死状态与连续计数一起清零。
      active_path_impassable_ = false;
      invalid_replan_count_ = 0;
      RCLCPP_DEBUG(
        get_logger(), "跟踪中重规划已换上新路径(%zu 顶点)", fp.path.poses.size());
    }
    return;
  }

  // ---- 分支二：候选校验（原有逻辑）----
  // 迟到的结果：状态早就走开了(比如已被暂停)，直接丢弃，绝不据此下发目标。
  if (state_ != ExplorationState::kValidating) {
    RCLCPP_DEBUG(get_logger(), "收到迟到的规划结果(当前 %s)，已丢弃", toString(state_));
    return;
  }
  if (candidate_index_ >= candidates_.size()) {
    onCandidatesExhausted("规划结果到达时候选队列已空");
    return;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
    // 局部/全局无路可走：不是错误，是这个候选点不可达，换下一个。
    // 把「全局规划失败」拆成两种，因为正确响应完全相反：
    //   · 起点致命(机器人站在膨胀带里) → 挪机器人，换候选点没用
    //   · 目标不可达               → 换候选点，挪机器人没用
    // 改动前两者都归成「全局规划失败(无可行路径)」，实测 25648 次里
    // 混着 28 次真正的「路径穿越未知/占据区」，根因被这个归类掩盖了。
    std::string esc_why;
    const EscapeVerdict verdict = checkEscapeTrigger(esc_why);
    if (verdict == EscapeVerdict::kBlockedPhysically) {
      // 🔴 红线：物理真堵。停机告警，绝不尝试脱困。
      RCLCPP_ERROR(get_logger(), "%s", esc_why.c_str());
      escape_last_result_ = esc_why;
      transitionTo(ExplorationState::kPaused, "物理真堵(禁止脱困)");
      return;
    }
    if (verdict == EscapeVerdict::kEscape) {
      if (escape_count_ >= escape_max_attempts_) {
        RCLCPP_ERROR(
          get_logger(),
          "起点仍在膨胀带但脱困已用满 %d 次，停止重试，等待人工 ~/resume。%s",
          escape_max_attempts_, esc_why.c_str());
        escape_last_result_ = "次数用尽";
        transitionTo(ExplorationState::kPaused, "脱困次数用尽");
        return;
      }
      std::string tgt_why;
      if (!pickEscapeTarget(escape_target_, tgt_why)) {
        // 选不出脱困目标，**先给自举一个机会**，用尽了才 PAUSED。
        //
        // 这里原来是直接 PAUSED，冷启动时必死（2026-09-01 实测）：
        //   首帧全局代价地图 未知 96.3% 致命 2.9% 空闲 0.8%
        //   -> 脱困环搜索要求「非致命 且 非未知 且 /map 可站 且 直线可达」，
        //      96% 未知的图上几乎没有格子合格 -> 脱困无解 -> PAUSED
        //   -> 自动恢复 -> 地图没变 -> 立刻又无解 -> ... 直到恢复预算用尽
        //   实测那一轮 131s 就死、到位=0，整轮验收作废。
        //
        // 为什么自举是正确的兜底，而不是"再试一次同样的事"：
        //   1. 原地旋转**不平移**，物理上不可能穿过占据栅格 —— 天然满足红线，
        //      比脱困本身更安全（脱困要走直线，自举只转）；
        //   2. 它把未知变已知，正好解决"无解是因为周围全未知"这个根因；
        //   3. shouldBootstrap 自带全部闸门：次数上限、停滞计时、安全门
        //      (激光新鲜 + 最近障碍 >=0.42m)、人工暂停中不动、有在途目标不动。
        //      所以这里不需要再加一层判断，也不会变成死循环重试。
        //
        // 注意红线仍然在上面先判：verdict == kBlockedPhysically 时早已 return，
        // 走到这里意味着**不是**物理真堵。别把这个顺序调过来。
        //
        // 与「连续多轮采不到合法候选点」那条路保持同一形状（先自举、后 PAUSED）：
        // 两条路都是"地图内容不够用"，不该一条有兜底另一条没有。
        std::string boot_why;
        if (shouldBootstrap("脱困选不出目标(疑似周围全未知)", boot_why)) {
          escape_last_result_ = "选不出目标转自举: " + tgt_why;
          RCLCPP_WARN(
            get_logger(),
            "需要脱困但选不出目标(%s) —— 改用原地自举把未知变已知(%s)",
            tgt_why.c_str(), boot_why.c_str());
          beginBootstrap(boot_why);
          return;
        }
        RCLCPP_ERROR(
          get_logger(), "需要脱困但选不出目标: %s（自举也不可用: %s）",
          tgt_why.c_str(), boot_why.c_str());
        escape_last_result_ = "选不出目标: " + tgt_why;
        transitionTo(ExplorationState::kPaused, "脱困无解");
        return;
      }
      escape_target_valid_ = true;
      escape_clear_streak_ = 0;
      escape_cmd_ = EscapeCommand{};
      escape_started_time_ = now();
      ++escape_count_;
      RCLCPP_WARN(
        get_logger(),
        "脱困 %d/%d：%s。目标(%.2f, %.2f)，%s。"
        "限速 %.2fm/s %.2frad/s，速度发往 %s(下游限速与 leash 照常生效)",
        escape_count_, escape_max_attempts_, esc_why.c_str(),
        escape_target_.x, escape_target_.y, tgt_why.c_str(),
        escape_linear_vel_, escape_angular_vel_, bootstrap_cmd_vel_topic_.c_str());
      transitionTo(ExplorationState::kEscape, "起点落在膨胀带，开始脱困");
      return;
    }
    rejectCurrentCandidate("全局规划失败(无可行路径)");
    return;
  }

  const GoalCandidatePose candidate = candidates_[candidate_index_];
  const auto & path = result.result->path;
  if (path.poses.empty()) {
    rejectCurrentCandidate("规划返回空路径");
    return;
  }

  // 下发前校验用的栅格图 —— 必须是 planner_server 规划时所用的那一张
  // （默认全局代价地图），否则两层判据查不同的图，会互相锁死：
  // 实测用 /map 校验时 631/631 次候选全被否，探索一个目标都发不出去。
  // 详见 costmap_adapter.hpp 文件头。
  std::string grid_why;
  const std::shared_ptr<GridMap> grid = validationGrid(grid_why);
  if (!grid) {
    RCLCPP_WARN(
      get_logger(), "校验用栅格图不可用(%s)，本次候选作废(绝不在无图状态下下发)",
      grid_why.c_str());
    rejectCurrentCandidate("校验用栅格图不可用: " + grid_why);
    return;
  }

  // 目标点重新校验一遍：从生成到现在地图可能已经更新，
  // 原先合法的目标可能已经被新观测判成占据/未知。
  const ValidationResult goal_check = validator_.validateGoal(*grid, candidate.x, candidate.y);
  if (!goal_check.valid) {
    rejectCurrentCandidate("目标点复检未通过: " + goal_check.reason);
    return;
  }

  // 校验2（路径）：逐段插值采样，任何一个采样点落在未知格上就整条否掉。
  // 终点截断检查(path_endpoint_tolerance)也在 validatePath 内，一并保留——
  // 它拦的是「规划器把不可达目标尽力靠近后返回半截路径、action 仍报成功」这一类，
  // 与查哪张图无关，是独立有价值的一条。
  std::vector<PlanarPoint> pts;
  pts.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    pts.push_back(PlanarPoint{p.pose.position.x, p.pose.position.y});
  }
  const PlanarPoint requested{candidate.x, candidate.y};
  const ValidationResult path_check = validator_.validatePath(*grid, pts, requested);
  if (!path_check.valid) {
    RCLCPP_WARN(
      get_logger(),
      "路径未知区校验未通过(第 %d 段, 首个非法点 %.2f, %.2f, 已查 %zu 点): %s",
      path_check.bad_segment_index, path_check.first_bad_point.x,
      path_check.first_bad_point.y, path_check.samples_checked, path_check.reason.c_str());
    rejectCurrentCandidate("路径穿越未知/占据区");
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "候选 (%.2f, %.2f) 双层校验通过: 路径 %zu 个顶点 / %zu 个采样点全部位于已知可通行区",
    candidate.x, candidate.y, path.poses.size(), path_check.samples_checked);
  // 需求1：follow_path 模式下把**刚刚通过校验的这条路径**直接交给控制器，
  // 而不是只发目标点让 BT 再规划一条（那条不曾被校验过）。
  if (dispatch_mode_ == DispatchMode::kFollowPath) {
    dispatchFollowPath(candidate, path);
  } else {
    dispatchNavGoal(candidate);
  }
}

void ExplorationCoordinatorNode::rejectCurrentCandidate(const std::string & reason)
{
  ++candidates_rejected_;
  if (candidate_index_ < candidates_.size()) {
    const auto & c = candidates_[candidate_index_];
    // 把被拒的点也记入访问历史：否则下一轮采样极可能又把它挑出来，白跑一遍校验。
    recordVisit(c.x, c.y);
    RCLCPP_WARN(
      get_logger(), "丢弃候选 %zu/%zu (%.2f, %.2f): %s",
      candidate_index_ + 1U, candidates_.size(), c.x, c.y, reason.c_str());
    ++candidate_index_;
  }
  if (candidate_index_ >= candidates_.size()) {
    onCandidatesExhausted(reason);
    return;
  }
  // 刻意**不**在这里直接调 requestPlanForCurrentCandidate()（原来是这么写的，实测踩坑）。
  //
  // 本函数的调用方之一是 onPlanGoalResponse()，它在入口就持有 state_mutex_
  // （非递归 std::mutex）。在持锁状态下再发一个 action goal，如果这个 goal 又被
  // 同步拒绝，就会在同一条 io_cb_group_（MutuallyExclusive）上重新进入
  // onPlanGoalResponse 并二次取同一把锁 —— 整个节点会彻底静默。
  //
  // 实测现象（planner_server 因为 Nav2 bringup 卡住而停在 inactive、于是
  // compute_path_to_pose 同步拒绝每一个 goal）：日志停在
  //   丢弃候选 1/4 (-0.58, 0.69): 规划请求被拒绝
  // 之后 283s 零输出 —— 既没有"校验候选 2/4"，也没有本该立刻打印的
  // "全局规划动作尚未就绪"节流告警，连 0.5s 的 tick 日志都没有
  // （tick 也要取 state_mutex_，所以锁一挂住就全静默）。
  //
  // 不需要在这里重发：tickValidating() 每个节拍都会检查
  // "校验请求不在途 -> requestPlanForCurrentCandidate()"，
  // 由定时器入口统一持锁、单层调用，天然不会重入。代价只是每个被拒候选
  // 多等一个 0.5s 节拍（4 个候选最多 2s），换掉一个能让节点假死的重入路径，
  // 这个交换很值。
  //
  // 不变式（新增的，别再破坏它）：**持有 state_mutex_ 时不得发 action goal**。
  // 同一风险还存在于 dispatchGoal() 里的 nav_client_->async_send_goal()，
  // 那条路径上 onNavGoalResponse 只走 registerNavFailure、不再回头发 goal，
  // 所以目前只差一层、没有闭环重入，但同样不该指望这一点长期成立。
}

void ExplorationCoordinatorNode::onCandidatesExhausted(const std::string & reason)
{
  resetCycleState();
  // 用**校验**预算而不是采样预算。这一路失败的循环是「采样 -> 校验全废 ->
  // 重采样」，而采样预算在 generateCandidates 采样成功时被清零 —— 也就是这个
  // 循环里每轮都被清一次，它的上限结构上永不可达（实测 837/837 全是 1/4，
  // 某轮据此空转 400s）。清零条件见 ExplorationFailureBudget 的注释。
  const bool budget_used_up = failure_budget_.onAllCandidatesInvalid();
  RCLCPP_WARN(
    get_logger(), "本轮候选全部不合法(%s)，连续校验失败 %d/%d",
    reason.c_str(), failure_budget_.validation_failures,
    failure_budget_.max_validation_failures);
  if (budget_used_up) {
    // 注意：这里是 PAUSED 不是 COMPLETED —— 前沿还在，只是暂时找不到合法通路。
    transitionTo(ExplorationState::kPaused, "连续多轮候选全部不合法");
    return;
  }
  transitionTo(ExplorationState::kGenNextPoint, "重新采样候选点");
}

// ==================== 导航下发（唯一出口）====================

void ExplorationCoordinatorNode::dispatchNavGoal(const GoalCandidatePose & goal)
{
  // ---- 时序锁 1：状态锁。只有校验通过的那一刻(kValidating)才允许下发。----
  if (state_ != ExplorationState::kValidating) {
    RCLCPP_ERROR(
      get_logger(), "试图在 %s 态下发目标，已拦截(下发只允许发生在 VALIDATING)",
      toString(state_));
    transitionTo(ExplorationState::kIdle, "非法下发时机");
    return;
  }
  // ---- 时序锁 2：在途标志。原子 CAS，任何并发下发企图都会失败在这里。----
  bool expected = false;
  if (!nav_goal_in_flight_.compare_exchange_strong(expected, true)) {
    RCLCPP_ERROR(
      get_logger(), "检测到并发下发企图：已有目标在途，本次下发被拒绝");
    return;
  }
  if (!nav_client_ || !nav_client_->action_server_is_ready()) {
    nav_goal_in_flight_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "导航动作 %s 尚未就绪，本次不下发", nav_action_name_.c_str());
    return;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose.header.frame_id = map_frame_;
  nav_goal.pose.header.stamp = now();
  nav_goal.pose.pose.position.x = goal.x;
  nav_goal.pose.pose.position.y = goal.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal.yaw);
  nav_goal.pose.pose.orientation.x = q.x();
  nav_goal.pose.pose.orientation.y = q.y();
  nav_goal.pose.pose.orientation.z = q.z();
  nav_goal.pose.pose.orientation.w = q.w();
  // 指定行为树：探索场景用把 controller_id 指向 FollowPathExplore 的那一份，
  // 从而启用三段式跟踪（起步对齐 -> 跟踪 -> **不**对齐终点姿态）。
  // 留空则走 bt_navigator 默认树，行为与接入前完全一致（一键回退）。
  nav_goal.behavior_tree = nav_behavior_tree_;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](const NavGoalHandle::SharedPtr & handle) {onNavGoalResponse(handle);};
  opts.result_callback =
    [this](const NavGoalHandle::WrappedResult & result) {onNavResult(result);};
  nav_client_->async_send_goal(nav_goal, opts);

  active_goal_ = goal;
  has_active_goal_ = true;
  nav_started_time_ = now();
  dwell_active_ = false;
  ++goals_dispatched_;
  // 校验失败预算只在**目标真的发出去了**才清零 —— 这是走出「采样->校验全废->
  // 重采样」循环的唯一标志。放到采样成功处清过一版，结果计数器永远是 1
  // （见 ExplorationFailureBudget 的注释）。
  failure_budget_.onGoalDispatched();

  if (goal_pub_) {
    goal_pub_->publish(nav_goal.pose);
  }
  RCLCPP_INFO(
    get_logger(),
    "下发导航目标 #%" PRIu64 " (%.2f, %.2f, yaw=%.2f) 代价=%.3f",
    goals_dispatched_, goal.x, goal.y, goal.yaw, goal.cost);
  transitionTo(ExplorationState::kNavigating, "目标已下发给 Nav2");
}

void ExplorationCoordinatorNode::onNavGoalResponse(const NavGoalHandle::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!handle) {
    nav_goal_in_flight_.store(false);
    has_active_goal_ = false;
    RCLCPP_ERROR(get_logger(), "导航目标被 Nav2 拒绝");
    registerNavFailure("导航目标被拒绝");
    return;
  }
  nav_goal_handle_ = handle;
  RCLCPP_INFO(get_logger(), "导航目标已被 Nav2 接受，开始执行");
}

void ExplorationCoordinatorNode::dispatchFollowPath(
  const GoalCandidatePose & goal, const nav_msgs::msg::Path & path)
{
  // 时序锁与 dispatchNavGoal 完全同构 —— 两条通路都必须保证「同时只有一个目标在飞」。
  if (state_ != ExplorationState::kValidating) {
    RCLCPP_ERROR(
      get_logger(), "试图在 %s 态下发 FollowPath，已拦截(下发只允许发生在 VALIDATING)",
      toString(state_));
    transitionTo(ExplorationState::kIdle, "非法下发时机");
    return;
  }
  bool expected = false;
  if (!nav_goal_in_flight_.compare_exchange_strong(expected, true)) {
    RCLCPP_ERROR(get_logger(), "检测到并发下发企图：已有目标在途，本次 FollowPath 被拒绝");
    return;
  }
  if (!follow_client_ || !follow_client_->action_server_is_ready()) {
    nav_goal_in_flight_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "FollowPath 动作 %s 尚未就绪，本次不下发", follow_action_name_.c_str());
    return;
  }
  if (path.poses.empty()) {
    nav_goal_in_flight_.store(false);
    RCLCPP_ERROR(get_logger(), "已校验路径为空，拒绝下发(不应发生)");
    registerNavFailure("已校验路径为空");
    return;
  }

  FollowPath::Goal fp_goal;
  fp_goal.path = path;
  fp_goal.controller_id = follow_controller_id_;
  fp_goal.goal_checker_id = follow_goal_checker_id_;

  rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](const FollowGoalHandle::SharedPtr & handle) {onFollowGoalResponse(handle);};
  opts.result_callback =
    [this](const FollowGoalHandle::WrappedResult & result) {onFollowResult(result);};
  follow_client_->async_send_goal(fp_goal, opts);

  active_goal_ = goal;
  has_active_goal_ = true;
  nav_started_time_ = now();
  last_replan_time_ = now();
  dwell_active_ = false;
  follow_retry_count_ = 0;          // 新目标，重试预算重置
  ++goals_dispatched_;
  // 与 dispatchNavGoal 同一条不变式：校验失败预算只在目标真的发出去了才清零。
  failure_budget_.onGoalDispatched();
  // 记下正在跟踪的这条路径。on_invalid 策略靠它判断「还需不需要换路径」，
  // 没有它就只能退回按时间无条件换路径。
  active_path_.clear();
  active_path_.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    active_path_.push_back(PlanarPoint{p.pose.position.x, p.pose.position.y});
  }
  active_path_time_ = now();
  last_replan_check_time_ = now();
  replan_forced_ = false;
  active_path_impassable_ = false;
  invalid_replan_count_ = 0;
  // 成功下发目标说明地图已经够用：清掉自举预算，让后续真的停滞时还能再自举。
  bootstrap_count_ = 0;
  bootstrap_stall_active_ = false;

  if (goal_pub_) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = map_frame_;
    p.header.stamp = now();
    p.pose.position.x = goal.x;
    p.pose.position.y = goal.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal.yaw);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    goal_pub_->publish(p);
  }
  RCLCPP_INFO(
    get_logger(),
    "下发 FollowPath #%" PRIu64 " (%.2f, %.2f) 复用已校验路径 %zu 顶点 "
    "controller=%s checker=%s",
    goals_dispatched_, goal.x, goal.y, path.poses.size(),
    follow_controller_id_.c_str(),
    follow_goal_checker_id_.empty() ? "(默认)" : follow_goal_checker_id_.c_str());
  transitionTo(ExplorationState::kNavigating, "已校验路径已交给控制器");
}

void ExplorationCoordinatorNode::onFollowGoalResponse(const FollowGoalHandle::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!handle) {
    nav_goal_in_flight_.store(false);
    RCLCPP_WARN(get_logger(), "FollowPath 目标被 controller_server 拒绝");
    registerNavFailure("FollowPath 目标被拒绝");
    return;
  }
  follow_goal_handle_ = handle;
  current_follow_goal_id_ = handle->get_goal_id();
  has_current_follow_goal_id_ = true;
  RCLCPP_INFO(get_logger(), "FollowPath 已被接受，开始跟踪已校验路径");
}

void ExplorationCoordinatorNode::onFollowResult(const FollowGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // 只认当前这个目标的结果。被重规划取代的旧目标会以 ABORTED 回来，
  // 那不是控制器失败，而是我们自己换了路径 —— 当成失败会形成自激循环
  // （实测：66 个终止结果 vs controller_server 真正 abort 仅 2 次）。
  if (has_current_follow_goal_id_ && result.goal_id != current_follow_goal_id_) {
    RCLCPP_DEBUG(
      get_logger(), "忽略被重规划取代的旧 FollowPath 结果(code=%d)",
      static_cast<int>(result.code));
    return;
  }

  nav_goal_in_flight_.store(false);
  follow_goal_handle_.reset();
  has_current_follow_goal_id_ = false;

  if (state_ != ExplorationState::kNavigating) {
    RCLCPP_DEBUG(get_logger(), "收到迟到的 FollowPath 结果(当前 %s)，已丢弃", toString(state_));
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // 与 NavigateToPose 分支一致：控制器报成功只是「它认为到了」，
      // 仍要走 kArrived 的实测校验。
      transitionTo(ExplorationState::kArrived, "控制器报告成功，待实测校验");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // BT 模式下这一步会被 RecoveryNode 吸收一次（清代价地图后重试），
      // follow_path 模式没有 BT，必须自己重试，否则每次进度停滞都直接判死。
      // 实测未加重试时：12 次下发全部因 Failed to make progress 判失败、0 次收敛。
      if (follow_retry_count_ < follow_max_retries_) {
        ++follow_retry_count_;
        RCLCPP_WARN(
          get_logger(),
          "FollowPath 中止，对同一目标重试 %d/%d（重新规划后再下发）",
          follow_retry_count_, follow_max_retries_);
        // 用显式的强制标志触发重规划。
        // 早先的写法是把 last_replan_time_ 推到过去，那只在 periodic 策略下有效；
        // on_invalid 策略根本不看时间，那样写会静默地什么都不发生。
        replan_forced_ = true;
        // 保持 NAVIGATING 与在途标志：这仍然是同一个目标，不是新目标。
        nav_goal_in_flight_.store(true);
        return;
      }
      registerNavFailure("FollowPath 中止且重试已用尽");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      registerNavFailure("FollowPath 被取消");
      break;
    default:
      registerNavFailure("FollowPath 返回未知结果码");
      break;
  }
}

void ExplorationCoordinatorNode::maybeRequestReplan(const rclcpp::Time & now_time)
{
  // 只有 follow_path 模式需要自己重规划：navigate_to_pose 模式由 BT 的
  // RateController(hz=1.0) 负责，本节点不该重复插手。
  if (dispatch_mode_ != DispatchMode::kFollowPath) {
    return;
  }

  if (replan_policy_ == ReplanPolicy::kPeriodic) {
    // ---- 回退策略：无条件周期重规划（旧行为）----
    // 实测代价：36 个目标下发 393 次 FollowPath，每条路径平均 1.5s 后就被下一条抢占，
    // 没有一条被跟踪到位。保留它只为一键对比回归。
    if (replan_period_sec_ <= 0.0) {
      return;
    }
    if (!replan_forced_ && (now_time - last_replan_time_).seconds() < replan_period_sec_) {
      return;
    }
    requestReplan(now_time, replan_forced_ ? "控制器中止后强制重规划" : "周期重规划(periodic 策略)");
    return;
  }

  // ---- 默认策略 on_invalid：未失效就不换路径 ----
  // 检查节拍只控制"多久看一眼"，看一眼是纯本地计算，不发 action。
  if ((now_time - last_replan_check_time_).seconds() < replan_check_period_sec_) {
    return;
  }
  last_replan_check_time_ = now_time;

  std::string why;
  if (!needsReplan(now_time, why)) {
    RCLCPP_DEBUG(get_logger(), "当前路径仍然有效，继续跟踪(不重规划)");
    // 路径又变回可通行了（代价地图刷新是常事），把判死状态和计数一起清掉。
    if (active_path_impassable_) {
      RCLCPP_INFO(get_logger(), "当前路径恢复可通行，撤销判死状态");
      active_path_impassable_ = false;
      invalid_replan_count_ = 0;
    }
    return;
  }
  // 防抖：判据在阈值附近来回跳时，不允许连续换路径。
  const double since_last = (now_time - last_replan_time_).seconds();
  if (!replan_forced_ && since_last < replan_min_interval_sec_) {
    RCLCPP_DEBUG(
      get_logger(), "需要重规划(%s)但距上次仅 %.2fs < %.2fs，本轮跳过",
      why.c_str(), since_last, replan_min_interval_sec_);
    return;
  }
  requestReplan(now_time, why);
}

bool ExplorationCoordinatorNode::needsReplan(const rclcpp::Time & now_time, std::string & why)
{
  // 1) 控制器已经中止，当前路径事实上作废。
  if (replan_forced_) {
    why = "控制器中止后强制重规划";
    return true;
  }
  // 2) 没有在途路径（异常兜底：正常情况下 NAVIGATING 一定有路径）。
  if (active_path_.empty()) {
    why = "无在途路径记录";
    return true;
  }
  // 3) 路径超龄（>0 才启用，纯兜底）。
  if (path_max_age_sec_ > 0.0) {
    const double age = (now_time - active_path_time_).seconds();
    if (age > path_max_age_sec_) {
      why = "路径已 " + std::to_string(age) + "s，超过 path_max_age_sec";
      return true;
    }
  }
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  std::string pose_why;
  if (!robotPose(rx, ry, ryaw, pose_why)) {
    // 取不到位姿时**不**重规划：定位丢失由 tickNavigating 单独处理（转 PAUSED），
    // 在这里换路径只会拿着一个不可信的起点去规划。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "重规划判据取不到位姿(%s)，本轮保持当前路径", pose_why.c_str());
    return false;
  }
  const PlanarPoint robot{rx, ry};

  // 4) 机器人已偏离这条路径 —— 它不再描述机器人的处境。
  const double dev = pathDeviation(active_path_, robot);
  if (dev > path_deviation_limit_m_) {
    why = "已偏离当前路径 " + std::to_string(dev) + "m > " +
      std::to_string(path_deviation_limit_m_) + "m";
    return true;
  }

  // 5) 剩余段是否还可通行。只查剩余段：身后新观测到的障碍与"还能不能继续跟"无关。
  const std::vector<PlanarPoint> rest = remainingPath(active_path_, robot);
  if (rest.size() < 2U) {
    // 只剩 1 个点 = 已经到路径末端附近。这不是"路径失效"，恰恰是快到了，
    // 此时换路径会打断收尾。交给抵达判定/goal checker 收口。
    return false;
  }
  std::string grid_why;
  const std::shared_ptr<GridMap> grid = validationGrid(grid_why);
  if (!grid) {
    // 校验图暂时不可用时不换路径：换了也没法校验新路径，且当前路径此前是通过校验的。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "重规划判据拿不到校验图(%s)，本轮保持当前路径", grid_why.c_str());
    return false;
  }
  const PlanarPoint requested{active_goal_.x, active_goal_.y};
  const ValidationResult chk = validator_.validatePath(*grid, rest, requested);
  if (!chk.valid) {
    why = "剩余段已不可通行: " + chk.reason;
    // 记下「这条路径已经判死」。后面若重规划也拿不到合法替代，
    // 就不能再沿用它 —— 那等于明知走不通还继续往里顶。
    active_path_impassable_ = true;
    return true;
  }
  return false;
}

void ExplorationCoordinatorNode::onReplanProducedNoUsablePath(const std::string & detail)
{
  if (!active_path_impassable_) {
    // 当前路径还没被判死：这次只是规划请求本身没成，沿用当前路径是对的。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "跟踪期重规划未取得可用路径(%s)，当前路径仍可通行，继续沿用", detail.c_str());
    return;
  }
  ++invalid_replan_count_;
  RCLCPP_WARN(
    get_logger(),
    "当前路径已判不可通行、重规划也无合法替代(%s)，连续 %d/%d 次",
    detail.c_str(), invalid_replan_count_, max_invalid_replan_attempts_);
  if (invalid_replan_count_ < max_invalid_replan_attempts_) {
    // 留一点瞬态余量：代价地图偶尔会因为一帧观测把通路刷成占据，下一帧就好了。
    return;
  }
  // 到这里就是「这个目标现在真的走不通」。撤掉在途目标、交给状态机另选一个，
  // 而不是继续顶着走 —— 实测顶了 17s 才被 progress checker 救回来。
  RCLCPP_ERROR(
    get_logger(),
    "放弃当前目标(%.2f, %.2f)：路径不可通行且 %d 次重规划都拿不到合法替代",
    active_goal_.x, active_goal_.y, invalid_replan_count_);
  invalid_replan_count_ = 0;
  active_path_impassable_ = false;
  cancelActiveNavGoal("路径不可通行且无合法替代");
  registerNavFailure("路径不可通行且重规划无合法替代");
}

void ExplorationCoordinatorNode::requestReplan(
  const rclcpp::Time & now_time, const std::string & why)
{
  bool expected = false;
  if (!plan_request_in_flight_.compare_exchange_strong(expected, true)) {
    return;                       // 上一次重规划还没回来，跳过本轮
  }
  if (!plan_client_ || !plan_client_->action_server_is_ready()) {
    plan_request_in_flight_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "需要重规划(%s)但全局规划动作未就绪", why.c_str());
    return;
  }
  last_replan_time_ = now_time;
  plan_request_is_replan_ = true;
  replan_forced_ = false;         // 已经据此发出请求，清掉强制标志
  RCLCPP_INFO(get_logger(), "跟踪期重规划: %s", why.c_str());

  ComputePathToPose::Goal goal;
  goal.goal.header.frame_id = map_frame_;
  goal.goal.header.stamp = now_time;
  goal.goal.pose.position.x = active_goal_.x;
  goal.goal.pose.position.y = active_goal_.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, active_goal_.yaw);
  goal.goal.pose.orientation.x = q.x();
  goal.goal.pose.orientation.y = q.y();
  goal.goal.pose.orientation.z = q.z();
  goal.goal.pose.orientation.w = q.w();
  goal.use_start = false;
  goal.planner_id = planner_id_;

  rclcpp_action::Client<ComputePathToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](const PlanGoalHandle::SharedPtr & handle) {onPlanGoalResponse(handle);};
  opts.result_callback =
    [this](const PlanGoalHandle::WrappedResult & result) {onPlanResult(result);};
  plan_client_->async_send_goal(goal, opts);
}

void ExplorationCoordinatorNode::onNavResult(const NavGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  nav_goal_in_flight_.store(false);
  nav_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (state_ == ExplorationState::kNavigating) {
        // Nav2 说到了，但还不算抵达：要进 kArrived 做实测位置+驻留校验。
        RCLCPP_INFO(get_logger(), "Nav2 报告导航成功，进入抵达校验");
        transitionTo(ExplorationState::kArrived, "Nav2 报告成功，待实测校验");
      } else {
        RCLCPP_DEBUG(
          get_logger(), "在 %s 态收到导航成功结果，已忽略", toString(state_));
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      registerNavFailure("Nav2 中止(无法规划或控制失败)");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // 主动取消(超时/定位丢失/人工暂停)已经在取消处处理过状态，这里不再叠加失败计数。
      RCLCPP_INFO(get_logger(), "导航目标已取消(当前 %s)", toString(state_));
      has_active_goal_ = false;
      break;
    default:
      registerNavFailure("导航返回未知结果码");
      break;
  }
}

void ExplorationCoordinatorNode::cancelActiveNavGoal(const std::string & reason)
{
  if (nav_client_ && nav_goal_handle_) {
    RCLCPP_WARN(get_logger(), "取消在途导航目标: %s", reason.c_str());
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_.reset();
  }
  // 两条下发通路都要撤。只撤一条会出现「已暂停但机器人还在走」——
  // 而暂停的语义正是"立刻停止下发并停下来"。
  if (follow_client_ && follow_goal_handle_) {
    RCLCPP_WARN(get_logger(), "取消在途 FollowPath: %s", reason.c_str());
    follow_client_->async_cancel_goal(follow_goal_handle_);
    follow_goal_handle_.reset();
  }
  nav_goal_in_flight_.store(false);
  has_active_goal_ = false;
  dwell_active_ = false;
  // 目标撤了，路径记录也必须清 —— 留着会让下一次 needsReplan 拿旧路径做判断。
  active_path_.clear();
  replan_forced_ = false;
  active_path_impassable_ = false;
  invalid_replan_count_ = 0;
}

void ExplorationCoordinatorNode::registerNavFailure(const std::string & reason)
{
  // 未就绪期间的失败不计数（见 stackReady 文件头）。仍然要清理状态并回到选点，
  // 只是不消耗"连续失败"预算 —— 否则启动瞬态就能把机器人永久停住。
  std::string ready_why;
  const bool ready = stackReady(ready_why);
  if (!ready) {
    if (has_active_goal_) {
      recordVisit(active_goal_.x, active_goal_.y);
    }
    has_active_goal_ = false;
    dwell_active_ = false;
    resetCycleState();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "导航失败(%s)，但栈尚未就绪(%s) —— 本次不计入连续失败",
      reason.c_str(), ready_why.c_str());
    transitionTo(ExplorationState::kGenNextPoint, "栈未就绪期间的失败，不计数");
    return;
  }
  ++nav_failure_count_;
  if (has_active_goal_) {
    // 失败的目标也记入访问历史，避免下一轮又选中同一个走不通的点。
    recordVisit(active_goal_.x, active_goal_.y);
  }
  has_active_goal_ = false;
  dwell_active_ = false;
  resetCycleState();
  RCLCPP_WARN(
    get_logger(), "导航失败(%s)，连续失败 %d/%d",
    reason.c_str(), nav_failure_count_, max_consecutive_nav_failures_);
  if (nav_failure_count_ >= max_consecutive_nav_failures_) {
    transitionTo(ExplorationState::kPaused, "连续导航失败，疑似被困");
    return;
  }
  transitionTo(ExplorationState::kGenNextPoint, "导航失败，重新选点");
}

// ===================== 状态输出与人工干预 =====================

void ExplorationCoordinatorNode::publishState()
{
  if (!state_pub_) {
    return;
  }
  // 单行、字段化，便于 `ros2 topic echo` 直接读，也便于脚本抓取做验证。
  std::string detail = "state=";
  detail += toString(state_);
  detail += " goal_in_flight=";
  detail += nav_goal_in_flight_.load() ? "1" : "0";
  if (has_active_goal_) {
    char buf[96];
    snprintf(buf, sizeof(buf), " goal=(%.2f,%.2f)", active_goal_.x, active_goal_.y);
    detail += buf;
  }
  // 缓冲大小要**跟着格式串一起长**：snprintf 超长是静默截断，
  // 上报里少掉尾巴几个计数器不会报错，只会让排查时看不见。
  char stats[288];
  snprintf(
    stats, sizeof(stats),
    " candidate=%zu/%zu dispatched=%" PRIu64 " succeeded=%" PRIu64
    " rejected=%" PRIu64 " nav_fail=%d/%d sample_fail=%d/%d validate_fail=%d/%d"
    " auto_resume=%d/%d",
    candidates_.empty() ? 0U : candidate_index_ + 1U, candidates_.size(),
    goals_dispatched_, goals_succeeded_, candidates_rejected_,
    nav_failure_count_, max_consecutive_nav_failures_,
    failure_budget_.sample_failures, failure_budget_.max_sample_failures,
    failure_budget_.validation_failures, failure_budget_.max_validation_failures,
    auto_resume_count_, max_auto_resume_attempts_);
  detail += stats;
  if (manually_paused_) {
    detail += " manual_pause=1";
  }
  // 自举与路径复用状态也要上报：需求「禁止静默失败」——自举被安全门拦下、
  // 或者转了却没转够，都必须在状态话题上能直接看到，不能只躺在日志里。
  {
    char boot[288];
    snprintf(
      boot, sizeof(boot),
      " bootstrap=%d/%d bootstrap_result=%s path_pts=%zu replan_policy=%s",
      bootstrap_count_, bootstrap_max_attempts_, bootstrap_last_result_.c_str(),
      active_path_.size(),
      replan_policy_ == ReplanPolicy::kOnInvalid ? "on_invalid" : "periodic");
    detail += boot;
  }

  std_msgs::msg::String msg;
  msg.data = detail;
  state_pub_->publish(msg);

  // 完成标志只在跳变时发，避免 transient_local 队列被同值刷满。
  const bool done = (state_ == ExplorationState::kCompleted);
  if (done != exploration_complete_) {
    exploration_complete_ = done;
    publishComplete(done);
  }
}

void ExplorationCoordinatorNode::publishComplete(bool done)
{
  if (!complete_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = done;
  complete_pub_->publish(msg);
}

void ExplorationCoordinatorNode::onPauseService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!response) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  manually_paused_ = true;
  cancelActiveNavGoal("人工暂停");
  resetCycleState();
  transitionTo(ExplorationState::kPaused, "收到人工暂停请求");
  response->success = true;
  response->message = "探索已暂停，目标发布已冻结";
  RCLCPP_WARN(get_logger(), "收到人工暂停请求，已冻结目标发布");
}

void ExplorationCoordinatorNode::onResumeService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!response) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  manually_paused_ = false;
  // 人工恢复视为一次「重置」：所有失败计数清零，包括自动恢复次数。
  failure_budget_.resetAll();
  nav_failure_count_ = 0;
  auto_resume_count_ = 0;
  resetCycleState();
  transitionTo(ExplorationState::kIdle, "收到人工恢复请求");
  response->success = true;
  response->message = "探索已恢复，计数器已重置";
  RCLCPP_INFO(get_logger(), "收到人工恢复请求，失败计数已重置");
}

}  // namespace astribot_s1_autonomy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astribot_s1_autonomy::ExplorationCoordinatorNode)
