// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

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
      return to == ExplorationState::kGenNextPoint || to == ExplorationState::kCompleted;
    case ExplorationState::kGenNextPoint:
      return to == ExplorationState::kValidating || to == ExplorationState::kCompleted;
    case ExplorationState::kValidating:
      // 候选全被拒 → 回 kGenNextPoint 重采样；校验通过 → kNavigating。
      return to == ExplorationState::kNavigating || to == ExplorationState::kGenNextPoint;
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
  state_entered_time_(0, 0, RCL_ROS_TIME),
  nav_started_time_(0, 0, RCL_ROS_TIME),
  plan_requested_time_(0, 0, RCL_ROS_TIME),
  dwell_started_time_(0, 0, RCL_ROS_TIME),
  latest_map_time_(0, 0, RCL_ROS_TIME),
  latest_odom_time_(0, 0, RCL_ROS_TIME)
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

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, nav_action_name_, io_cb_group_);
  plan_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this, plan_action_name_, io_cb_group_);

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
}

ExplorationCoordinatorNode::~ExplorationCoordinatorNode()
{
  // 析构时把在途目标撤掉，避免节点没了、Nav2 还在往一个没人监管的目标开。
  if (control_timer_) {
    control_timer_->cancel();
  }
  if (nav_client_ && nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
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

  declare_parameter<std::string>("map_topic", "/map", describe("输入占据栅格话题"));
  declare_parameter<std::string>("odom_topic", "/odom", describe("里程计话题，用于速度收敛判定"));
  declare_parameter<std::string>(
    "state_topic", "/exploration/state", describe("状态机状态话题(String)"));
  declare_parameter<std::string>(
    "complete_topic", "/exploration/complete", describe("探索完成标志话题(Bool)"));
  declare_parameter<std::string>(
    "current_goal_topic", "/exploration/current_goal",
    describe("当前已下发目标话题，仅供可视化"));
  declare_parameter<std::string>(
    "nav_action_name", "navigate_to_pose", describe("Nav2 导航动作名"));
  declare_parameter<std::string>(
    "plan_action_name", "compute_path_to_pose", describe("Nav2 全局规划动作名(仅用于校验)"));
  declare_parameter<std::string>("map_frame", "map", describe("地图坐标系"));
  declare_parameter<std::string>("robot_base_frame", "base_link", describe("机器人本体坐标系"));
  declare_parameter<std::string>(
    "planner_id", "", describe("指定全局规划器名，空串表示用 Nav2 默认"));

  declare_parameter<double>("control_period_sec", 0.5, describe("状态机节拍(s)"));
  declare_parameter<double>("tf_timeout_sec", 0.2, describe("TF 查询超时(s)"));
  declare_parameter<double>("map_timeout_sec", 10.0, describe("地图多久未更新算超时(s)"));
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
}

bool ExplorationCoordinatorNode::loadParameters(std::string & error)
{
  map_topic_ = get_parameter("map_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  state_topic_ = get_parameter("state_topic").as_string();
  complete_topic_ = get_parameter("complete_topic").as_string();
  current_goal_topic_ = get_parameter("current_goal_topic").as_string();
  nav_action_name_ = get_parameter("nav_action_name").as_string();
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
  odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
  plan_timeout_sec_ = get_parameter("plan_timeout_sec").as_double();
  nav_timeout_sec_ = get_parameter("nav_timeout_sec").as_double();
  if (control_period_sec_ <= 0.0 || tf_timeout_sec_ < 0.0 || map_timeout_sec_ <= 0.0 ||
    odom_timeout_sec_ <= 0.0 || plan_timeout_sec_ <= 0.0 || nav_timeout_sec_ <= 0.0)
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

  max_candidates_per_cycle_ = static_cast<int>(get_parameter("max_candidates_per_cycle").as_int());
  max_sample_failures_ = static_cast<int>(get_parameter("max_sample_failures").as_int());
  max_consecutive_nav_failures_ =
    static_cast<int>(get_parameter("max_consecutive_nav_failures").as_int());
  pause_cooldown_sec_ = get_parameter("pause_cooldown_sec").as_double();
  max_auto_resume_attempts_ = static_cast<int>(get_parameter("max_auto_resume_attempts").as_int());
  if (max_candidates_per_cycle_ < 1 || max_sample_failures_ < 1 ||
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

bool ExplorationCoordinatorNode::robotPose(double & x, double & y, double & yaw, std::string & why)
{
  if (!tf_buffer_) {
    why = "TF buffer 未初始化";
    return false;
  }
  try {
    // 取最新可用变换：探索调度是低频决策，不需要和某一帧数据严格对齐。
    const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      map_frame_, robot_base_frame_, tf2::TimePointZero,
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
    // map -> base 查不到即视为定位丢失，上层必须冻结目标发布。
    why = std::string("TF 查询失败(") + map_frame_ + " -> " + robot_base_frame_ + "): " + e.what();
    return false;
  }
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
}

// ========================= 状态机 =========================

void ExplorationCoordinatorNode::transitionTo(ExplorationState next, const std::string & why)
{
  if (state_ == next) {
    return;                                   // 空转换不刷新计时，避免驻留/冷却计时被反复重置
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

void ExplorationCoordinatorNode::controlTick()
{
  // 整个 tick 在一把锁里完成：状态判断和状态修改之间不留缝隙，
  // 这是「禁止多线程重复下发」的第一道保险（第二道是 nav_goal_in_flight_）。
  std::lock_guard<std::mutex> lock(state_mutex_);

  switch (state_) {
    case ExplorationState::kIdle:
      tickIdle();
      break;
    case ExplorationState::kGenNextPoint:
      tickGenNextPoint();
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
  transitionTo(ExplorationState::kGenNextPoint, "地图/定位/里程计均就绪");
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
    exploration_complete_ = true;
    transitionTo(ExplorationState::kCompleted, "地图内已无任何前沿格，探索完成");
    return;
  }
  if (candidates.empty()) {
    ++sample_failure_count_;
    RCLCPP_WARN(
      get_logger(),
      "本轮未采到合法候选点(前沿格 %zu 个仍存在)，连续失败 %d/%d",
      raw_frontier_cells, sample_failure_count_, max_sample_failures_);
    if (sample_failure_count_ >= max_sample_failures_) {
      transitionTo(ExplorationState::kPaused, "连续多轮无合法候选点");
    }
    return;
  }

  sample_failure_count_ = 0;
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
  ++auto_resume_count_;
  sample_failure_count_ = 0;
  nav_failure_count_ = 0;
  resetCycleState();
  RCLCPP_WARN(
    get_logger(), "暂停 %.1fs 后尝试第 %d/%d 次自动恢复",
    held, auto_resume_count_, max_auto_resume_attempts_);
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
  std::size_t dropped_unknown = 0U;
  std::size_t dropped_by_search = 0U;
  std::string search_reject_sample;
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
    const ValidationResult vr = validator_.validateGoal(map, c.x, c.y);
    if (!vr.valid) {
      ++dropped_unknown;
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
    rejectCurrentCandidate("全局规划失败(无可行路径)");
    return;
  }

  const GoalCandidatePose candidate = candidates_[candidate_index_];
  const auto & path = result.result->path;
  if (path.poses.empty()) {
    rejectCurrentCandidate("规划返回空路径");
    return;
  }

  std::shared_ptr<GridMap> map;
  {
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    map = latest_map_;
  }
  if (!map || !map->consistent()) {
    RCLCPP_WARN(get_logger(), "校验时地图不可用，本次候选作废(绝不在无图状态下下发)");
    rejectCurrentCandidate("校验时地图不可用");
    return;
  }

  // 目标点重新校验一遍：从生成到现在地图可能已经更新，
  // 原先合法的目标可能已经被新观测判成占据/未知。
  const ValidationResult goal_check = validator_.validateGoal(*map, candidate.x, candidate.y);
  if (!goal_check.valid) {
    rejectCurrentCandidate("目标点复检未通过: " + goal_check.reason);
    return;
  }

  // 校验2（路径）：逐段插值采样，任何一个采样点落在未知格上就整条否掉。
  std::vector<PlanarPoint> pts;
  pts.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    pts.push_back(PlanarPoint{p.pose.position.x, p.pose.position.y});
  }
  const PlanarPoint requested{candidate.x, candidate.y};
  const ValidationResult path_check = validator_.validatePath(*map, pts, requested);
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
  dispatchNavGoal(candidate);
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
  ++sample_failure_count_;
  RCLCPP_WARN(
    get_logger(), "本轮候选全部不合法(%s)，连续失败 %d/%d",
    reason.c_str(), sample_failure_count_, max_sample_failures_);
  if (sample_failure_count_ >= max_sample_failures_) {
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
  nav_goal_in_flight_.store(false);
  has_active_goal_ = false;
  dwell_active_ = false;
}

void ExplorationCoordinatorNode::registerNavFailure(const std::string & reason)
{
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
  char stats[224];
  snprintf(
    stats, sizeof(stats),
    " candidate=%zu/%zu dispatched=%" PRIu64 " succeeded=%" PRIu64
    " rejected=%" PRIu64 " nav_fail=%d/%d sample_fail=%d/%d auto_resume=%d/%d",
    candidates_.empty() ? 0U : candidate_index_ + 1U, candidates_.size(),
    goals_dispatched_, goals_succeeded_, candidates_rejected_,
    nav_failure_count_, max_consecutive_nav_failures_,
    sample_failure_count_, max_sample_failures_,
    auto_resume_count_, max_auto_resume_attempts_);
  detail += stats;
  if (manually_paused_) {
    detail += " manual_pause=1";
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
  sample_failure_count_ = 0;
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
