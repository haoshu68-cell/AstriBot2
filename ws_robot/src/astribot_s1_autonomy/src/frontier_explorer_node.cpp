// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/frontier_explorer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"

namespace astribot_s1_autonomy
{

namespace
{
constexpr int kLogThrottleMs = 3000;
/// 探索状态机的字符串常量，避免各处手写字面量导致不一致。
constexpr char kStateExploring[] = "EXPLORING";
constexpr char kStateComplete[] = "COMPLETE";
constexpr char kStateWaitingMap[] = "WAITING_MAP";
constexpr char kStateWaitingTf[] = "WAITING_TF";
constexpr char kStateNoValidGoal[] = "NO_VALID_GOAL";
}  // namespace

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("frontier_explorer_node", options),
  latest_map_stamp_(0, 0, RCL_ROS_TIME)
{
  declareParameters();

  std::string error;
  if (!loadParameters(error)) {
    RCLCPP_ERROR(
      get_logger(),
      "参数校验失败，节点将保持空转直到参数被修正: %s", error.c_str());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // 与感知节点同理：规划跑在独立工作线程里、且带 timeout 查 TF，
  // 必须让 listener 自带 spin 线程并显式告知 buffer，否则带超时的查询恒失败。
  // 详见 pointcloud_slice_scan_node.cpp 里同一处的详细踩坑说明。
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_buffer_->setUsingDedicatedThread(true);

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(1));
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(1));
  // 完成标志用 transient_local：晚启动的订阅者也能立刻拿到「已探索完成」。
  complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    complete_topic_, rclcpp::QoS(1).transient_local());
  if (publish_markers_) {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1));
  }

  // slam_toolbox 的 /map 是 transient_local + reliable，订阅端必须匹配，
  // 否则会出现「话题存在但一直收不到地图」这种最难查的情况。
  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {mapCallback(msg);});

  running_.store(true);
  planner_thread_ = std::thread([this]() {plannerLoop();});

  RCLCPP_INFO(
    get_logger(),
    "探索节点已启动: 地图=%s 目标输出=%s 规划周期=%.2fs 地图系=%s 本体系=%s",
    map_topic_.c_str(), goal_topic_.c_str(), planning_period_sec_,
    map_frame_.c_str(), robot_base_frame_.c_str());
}

FrontierExplorerNode::~FrontierExplorerNode()
{
  running_.store(false);
  planner_cv_.notify_all();
  if (planner_thread_.joinable()) {
    planner_thread_.join();
  }
}

void FrontierExplorerNode::declareParameters()
{
  auto describe = [](const std::string & text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      return d;
    };

  declare_parameter<std::string>("map_topic", "/map", describe("输入占据栅格话题"));
  declare_parameter<std::string>("goal_topic", "/explore/goal_pose", describe("输出目标位姿话题"));
  declare_parameter<std::string>("status_topic", "/explore/status", describe("探索状态话题"));
  declare_parameter<std::string>("complete_topic", "/explore/complete", describe("探索完成标志话题"));
  declare_parameter<std::string>("marker_topic", "~/debug_markers", describe("调试 Marker 话题"));
  declare_parameter<std::string>("map_frame", "map", describe("地图坐标系"));
  declare_parameter<std::string>(
    "robot_base_frame", "astribot_torso_base",
    describe("机器人本体坐标系。**本机器人没有 base_link**，根 frame 是 "
             "astribot_torso_base（Nav2 六处 robot_base_frame 也是它）。"
             "默认值原为 base_link，靠 yaml 覆盖才对 —— 而本仓库踩过 "
             "\"params_file 泄漏 / 节点名 remap 导致整份 yaml 静默失效\" 的坑，"
             "那时会回落到这个默认值，TF 查询全部失败且难以归因。"));

  declare_parameter<double>("tf_timeout_sec", 0.2, describe("TF 查询超时(s)"));
  declare_parameter<double>("planning_period_sec", 2.0, describe("探索规划周期(s)"));
  declare_parameter<double>("map_timeout_sec", 10.0, describe("地图多久未更新算超时(s)"));

  declare_parameter<double>("goal_same_tolerance", 0.35, describe("两次目标点距离小于该值视为同一目标(m)"));
  declare_parameter<int>(
    "max_same_goal_count", 3,
    describe("目标不变且机器人无推进连续多少轮后触发重新全局采样"));
  declare_parameter<double>(
    "robot_progress_tolerance", 0.15,
    describe("两轮之间机器人位移超过该值即算有推进(m)，此时重复同一目标属正常"));
  declare_parameter<int>(
    "max_relaxation_level", 3, describe("约束放宽的最大级数"));
  declare_parameter<double>("relaxation_scale", 0.6, describe("每级放宽时约束乘的系数(<1 表示放宽)"));
  declare_parameter<int>("max_consecutive_failures", 5, describe("连续多少次采样失败后输出告警"));
  declare_parameter<int>("visit_history_limit", 50, describe("历史访问记录保留条数上限"));
  declare_parameter<bool>("publish_markers", true, describe("是否发布调试 Marker"));

  // ---- 前沿搜索算法参数 ----
  declare_parameter<int>("search.occupied_threshold", 65, describe("占据判定阈值(0~100)"));
  declare_parameter<int>("search.free_threshold", 25, describe("空闲判定阈值(0~100)"));
  declare_parameter<double>("search.obstacle_inflation_radius", 0.35, describe("障碍膨胀半径(m)"));
  declare_parameter<int>("search.min_obstacle_cluster_cells", 3, describe("小于该格数的占据斑块视为噪声"));
  declare_parameter<bool>("search.use_eight_connectivity", true, describe("前沿判定是否用 8 邻域"));
  declare_parameter<int>("search.min_frontier_cells", 12, describe("前沿块最小格数"));
  declare_parameter<double>("search.gain_window_radius", 1.5, describe("未知增益统计窗口半径(m)"));
  declare_parameter<double>(
    "search.adaptive_sample_gain", 0.15,
    describe("采样数 = ceil(前沿格数 * 该系数)"));
  declare_parameter<int>("search.min_samples_per_cluster", 1, describe("每个前沿块最少采样数"));
  declare_parameter<int>("search.max_samples_per_cluster", 8, describe("每个前沿块最多采样数"));
  declare_parameter<double>("search.required_clearance_radius", 0.35, describe("候选点净空半径(m)"));
  declare_parameter<double>("search.min_goal_distance", 0.8, describe("目标点与机器人的最小距离(m)"));
  declare_parameter<double>("search.max_goal_distance", 0.0, describe("目标点最大距离(m)，<=0 不限制"));
  declare_parameter<double>("search.weight_distance", 1.0, describe("距离代价权重"));
  declare_parameter<double>("search.weight_gain", 6.0, describe("未知增益权重"));
  declare_parameter<double>("search.weight_visit_penalty", 4.0, describe("历史访问惩罚权重"));
  declare_parameter<double>("search.visit_penalty_radius", 1.2, describe("历史访问惩罚作用半径(m)"));
  declare_parameter<int>("search.random_seed", 20260819, describe("采样随机种子，固定值便于复现"));
}

bool FrontierExplorerNode::loadParameters(std::string & error)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  map_topic_ = get_parameter("map_topic").as_string();
  goal_topic_ = get_parameter("goal_topic").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  complete_topic_ = get_parameter("complete_topic").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  if (map_frame_.empty() || robot_base_frame_.empty()) {
    error = "map_frame / robot_base_frame 不能为空";
    return false;
  }

  tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
  planning_period_sec_ = get_parameter("planning_period_sec").as_double();
  map_timeout_sec_ = get_parameter("map_timeout_sec").as_double();
  if (tf_timeout_sec_ < 0.0 || planning_period_sec_ <= 0.0 || map_timeout_sec_ <= 0.0) {
    error = "tf_timeout_sec>=0, planning_period_sec>0, map_timeout_sec>0";
    return false;
  }

  goal_same_tolerance_ = get_parameter("goal_same_tolerance").as_double();
  max_same_goal_count_ = static_cast<int>(get_parameter("max_same_goal_count").as_int());
  robot_progress_tolerance_ = get_parameter("robot_progress_tolerance").as_double();
  max_relaxation_level_ = static_cast<int>(get_parameter("max_relaxation_level").as_int());
  relaxation_scale_ = get_parameter("relaxation_scale").as_double();
  max_consecutive_failures_ = static_cast<int>(get_parameter("max_consecutive_failures").as_int());
  if (goal_same_tolerance_ < 0.0 || robot_progress_tolerance_ < 0.0 ||
    max_same_goal_count_ < 1 || max_relaxation_level_ < 0 ||
    !(relaxation_scale_ > 0.0 && relaxation_scale_ <= 1.0) || max_consecutive_failures_ < 1)
  {
    error = "震荡/放宽相关参数非法：要求 tolerance>=0, max_same_goal_count>=1, "
      "max_relaxation_level>=0, 0<relaxation_scale<=1, max_consecutive_failures>=1";
    return false;
  }

  const auto history_limit = get_parameter("visit_history_limit").as_int();
  if (history_limit < 1) {
    error = "visit_history_limit 必须 >=1";
    return false;
  }
  visit_history_limit_ = static_cast<std::size_t>(history_limit);
  publish_markers_ = get_parameter("publish_markers").as_bool();

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

void FrontierExplorerNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  if (!msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "收到空的地图消息指针，已忽略");
    return;
  }

  // ---- 地图有效性校验：尺寸、分辨率、data 长度必须自洽 ----
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
      "地图 data 长度 %zu 与 width*height=%zu 不一致，忽略本帧(防止越界读)",
      msg->data.size(), expected);
    return;
  }
  if (!(msg->info.resolution > 0.0)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "地图分辨率非法(%.4f)，忽略本帧", msg->info.resolution);
    return;
  }

  // 回调里只做拷贝，重活交给规划线程。
  auto snapshot = std::make_shared<GridMap>();
  snapshot->width = msg->info.width;
  snapshot->height = msg->info.height;
  snapshot->resolution = msg->info.resolution;
  snapshot->origin_x = msg->info.origin.position.x;
  snapshot->origin_y = msg->info.origin.position.y;
  snapshot->data = msg->data;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = std::move(snapshot);
    latest_map_stamp_ = now();
  }
  map_ready_warned_ = false;
}

bool FrontierExplorerNode::lookupRobotPose(double & x, double & y)
{
  if (!tf_buffer_) {
    return false;
  }
  try {
    // 用 TimePointZero 取最新可用变换：探索规划是低频决策，
    // 不需要和某一帧点云严格对齐，取最新反而更稳。
    const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      map_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_sec_));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "TF 查询失败(%s -> %s)，本轮跳过规划: %s",
      map_frame_.c_str(), robot_base_frame_.c_str(), e.what());
    return false;
  }
}

int FrontierExplorerNode::searchWithRelaxation(
  const GridMap & map, double robot_x, double robot_y, FrontierSearch::Result & out)
{
  // 第 0 级：用 YAML 原始约束。
  search_.search(map, robot_x, robot_y, visit_history_, out);
  if (out.best_candidate_index >= 0) {
    return 0;
  }

  // 逐级放宽：每级把「最小前沿块面积」「最小目标距离」「净空半径」按系数缩小。
  // 这是为了应对走廊尽头、狭窄空间这类原始约束过严导致采不到点的场景，
  // 而不是无限重试同一组约束（那才是真正的死循环）。
  for (int level = 1; level <= max_relaxation_level_; ++level) {
    FrontierSearchParams relaxed = search_params_;
    const double scale = std::pow(relaxation_scale_, static_cast<double>(level));
    relaxed.min_frontier_cells = std::max(
      1,
      static_cast<int>(
        std::floor(static_cast<double>(search_params_.min_frontier_cells) * scale)));
    relaxed.min_goal_distance = search_params_.min_goal_distance * scale;
    relaxed.required_clearance_radius = search_params_.required_clearance_radius * scale;

    std::string err;
    FrontierSearch relaxed_search;
    if (!relaxed_search.configure(relaxed, err)) {
      RCLCPP_WARN(
        get_logger(), "放宽等级 %d 的参数非法，停止放宽: %s", level, err.c_str());
      break;
    }
    relaxed_search.search(map, robot_x, robot_y, visit_history_, out);
    if (out.best_candidate_index >= 0) {
      RCLCPP_WARN(
        get_logger(),
        "原始约束采不到有效目标，已放宽到等级 %d 后成功"
        "(min_frontier_cells=%d min_goal_distance=%.2fm clearance=%.2fm)",
        level, relaxed.min_frontier_cells, relaxed.min_goal_distance,
        relaxed.required_clearance_radius);
      return level;
    }
  }
  return -1;
}

bool FrontierExplorerNode::isSameAsLastGoal(double x, double y) const
{
  if (!has_last_goal_) {
    return false;
  }
  return std::hypot(x - last_goal_x_, y - last_goal_y_) <= goal_same_tolerance_;
}

void FrontierExplorerNode::plannerLoop()
{
  while (running_.load()) {
    {
      // 用 condition_variable 定时等待，而不是 sleep：
      // 这样析构时能立刻被 notify 唤醒退出，不用等满一个周期。
      std::unique_lock<std::mutex> lock(planner_mutex_);
      planner_cv_.wait_for(
        lock, std::chrono::duration<double>(planning_period_sec_),
        [this]() {return !running_.load();});
    }
    if (!running_.load()) {
      return;
    }
    try {
      planOnce();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "探索规划发生未预期异常，本轮跳过: %s", e.what());
    }
  }
}

void FrontierExplorerNode::planOnce()
{
  // ---- 1) 地图就绪性检查 ----
  std::shared_ptr<GridMap> map;
  rclcpp::Time map_stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = latest_map_;
    map_stamp = latest_map_stamp_;
  }
  if (!map) {
    if (!map_ready_warned_) {
      RCLCPP_WARN(get_logger(), "尚未收到占据栅格地图(%s)，等待地图就绪", map_topic_.c_str());
      map_ready_warned_ = true;
    }
    publishStatus(kStateWaitingMap, "尚未收到地图");
    return;
  }
  if (!map->consistent()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "地图快照不自洽，本轮跳过");
    publishStatus(kStateWaitingMap, "地图快照不自洽");
    return;
  }
  const double map_age = (now() - map_stamp).seconds();
  if (map_age > map_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "地图已 %.1fs 未更新(超时阈值 %.1fs)，仍用旧地图规划但请检查 SLAM 是否存活",
      map_age, map_timeout_sec_);
  }

  // ---- 2) 机器人位姿 ----
  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!lookupRobotPose(robot_x, robot_y)) {
    publishStatus(kStateWaitingTf, "TF 不可用");
    return;
  }

  // ---- 3) 搜索（带放宽）----
  FrontierSearch::Result result;
  int relaxation_level = 0;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    relaxation_level = searchWithRelaxation(*map, robot_x, robot_y, result);
  }

  if (publish_markers_) {
    publishMarkers(*map, result, now());
  }

  // ---- 4) 探索完成判定 ----
  // 「完全没有前沿格」才算真正探索完成；
  // 「有前沿但都被过滤/采不到点」是另一回事，不能误报完成，
  // 否则上层会提前停止探索。
  if (result.raw_frontier_cell_count == 0U) {
    if (!exploration_complete_) {
      RCLCPP_INFO(
        get_logger(), "地图中已无任何前沿格，判定环境探索完成。%s", result.summary.c_str());
      exploration_complete_ = true;
      std_msgs::msg::Bool done;
      done.data = true;
      complete_pub_->publish(done);
    }
    publishStatus(kStateComplete, result.summary);
    return;
  }

  // 出现新前沿（例如门被打开、SLAM 补上了新区域）时撤销「完成」状态。
  if (exploration_complete_) {
    RCLCPP_INFO(get_logger(), "检测到新的前沿区域，退出探索完成状态，继续探索");
    exploration_complete_ = false;
    std_msgs::msg::Bool done;
    done.data = false;
    complete_pub_->publish(done);
  }

  if (result.best_candidate_index < 0) {
    ++consecutive_failure_count_;
    if (consecutive_failure_count_ >= max_consecutive_failures_) {
      RCLCPP_WARN(
        get_logger(),
        "连续 %d 轮采不到有效目标(已放宽到最大等级 %d)。%s",
        consecutive_failure_count_, max_relaxation_level_, result.summary.c_str());
      // 清空历史访问惩罚：很可能是惩罚项把所有候选都压住了。
      // 这是「降低采样约束条件，避免死循环」的最后一招。
      if (!visit_history_.empty()) {
        RCLCPP_WARN(
          get_logger(), "清空 %zu 条历史访问记录后重试，解除惩罚项对候选的压制",
          visit_history_.size());
        visit_history_.clear();
      }
      consecutive_failure_count_ = 0;
    }
    publishStatus(kStateNoValidGoal, result.summary);
    return;
  }
  consecutive_failure_count_ = 0;

  GoalCandidate goal = result.candidates[static_cast<std::size_t>(result.best_candidate_index)];

  // ---- 5) 震荡检测 ----
  //
  // !!! 这里的判据有一处很关键的修正（实测发现的设计缺陷）!!!
  // 最初的写法是「连续 N 次输出同一个目标就判为震荡」，但那是错的：
  // 本模块以固定周期重复发布目标，而机器人正在赶往该目标的途中，
  // **反复发布同一个稳定目标恰恰是正常且期望的行为**。
  // 实测症状：机器人还没动（外部没人执行目标）时，每 2s 就误判一次震荡，
  // 惩罚无界累加（20→24→25…），代价从 75 涨到 95，日志被刷满，
  // 而目标其实根本没得换（当时只有 1 个前沿块通过过滤）。
  //
  // 真正的病态情形是「目标不变 **且** 机器人没有推进」——那才说明卡住了
  // （目标不可达 / 执行端没接 / 反复规划失败）。因此判据加上机器人位移条件。
  bool robot_made_progress = true;
  if (has_last_goal_) {
    robot_made_progress =
      std::hypot(robot_x - last_plan_robot_x_, robot_y - last_plan_robot_y_) >
      robot_progress_tolerance_;
  }

  if (isSameAsLastGoal(goal.x, goal.y) && !robot_made_progress) {
    ++same_goal_count_;
    if (same_goal_count_ >= max_same_goal_count_) {
      // 限流：机器人长时间不动（典型情况是根本没人把目标送进 Nav2）时，
      // 这个条件会一直成立，不限流会把日志刷满。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs * 5,
        "连续 %d 轮目标停在(%.2f, %.2f)且机器人位移 < %.2fm，判定卡住："
        "加重该点历史惩罚并重新全局采样。"
        "若一直如此，请确认有外部节点在订阅 %s 并调用 Nav2 执行",
        same_goal_count_, goal.x, goal.y, robot_progress_tolerance_,
        goal_topic_.c_str());
      VisitRecord rec;
      rec.x = goal.x;
      rec.y = goal.y;
      rec.count = static_cast<unsigned int>(max_same_goal_count_);
      visit_history_.push_back(rec);
      same_goal_count_ = 0;

      // 立刻重新搜索一次，这次带上加重后的惩罚，通常会选到别的前沿块。
      FrontierSearch::Result retry;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        (void)searchWithRelaxation(*map, robot_x, robot_y, retry);
      }
      if (retry.best_candidate_index >= 0) {
        const GoalCandidate & alt =
          retry.candidates[static_cast<std::size_t>(retry.best_candidate_index)];
        if (isSameAsLastGoal(alt.x, alt.y)) {
          // 重采样后还是同一个点，说明当前确实只有这一处可去
          // （典型情况：只剩一个前沿块通过过滤）。此时再堆惩罚毫无意义，
          // 只会把代价越推越高并刷日志，因此保持目标不变、降级为提示。
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), kLogThrottleMs * 4,
            "重采样后仍是同一目标，当前只有 %zu 个前沿块可用，保持该目标不变",
            retry.accepted_cluster_count);
          if (!visit_history_.empty()) {
            visit_history_.pop_back();   // 撤销刚加的惩罚，避免无界累加
          }
        } else {
          goal = alt;
          result = std::move(retry);
        }
      } else {
        publishStatus(kStateNoValidGoal, "震荡回退后仍无有效目标");
        return;
      }
    }
  } else {
    same_goal_count_ = 0;
  }

  last_plan_robot_x_ = robot_x;
  last_plan_robot_y_ = robot_y;

  // ---- 6) 输出目标 + 记录历史 ----
  publishGoal(goal, now());

  const bool goal_changed = !isSameAsLastGoal(goal.x, goal.y);

  last_goal_x_ = goal.x;
  last_goal_y_ = goal.y;
  has_last_goal_ = true;

  // 历史记录的语义是「这个地方我**被派去过几次**」，不是「这个目标我发了几帧」。
  //
  // !!! 实测踩坑：最初这里每轮都对命中的记录 ++count，而本节点是周期性重发目标，
  // 于是同一个目标每 2s 就把 count 加一，惩罚项无界增长
  // （实测惩罚 20→42、代价 75→163 一路涨不停），
  // 最终会把这块前沿彻底压死、即使它其实是唯一可去的地方。
  // 正确做法：只在目标**真正发生变化**时才记一次；重发同一目标不动历史。
  // 另外对 count 设上限，防止长时间运行下的数值膨胀。
  constexpr unsigned int kMaxVisitCount = 10U;
  if (goal_changed) {
    bool merged = false;
    for (VisitRecord & rec : visit_history_) {
      if (std::hypot(rec.x - goal.x, rec.y - goal.y) <= goal_same_tolerance_) {
        if (rec.count < kMaxVisitCount) {
          ++rec.count;
        }
        merged = true;
        break;
      }
    }
    if (!merged) {
      VisitRecord rec;
      rec.x = goal.x;
      rec.y = goal.y;
      rec.count = 1U;
      visit_history_.push_back(rec);
    }
  }
  while (visit_history_.size() > visit_history_limit_) {
    visit_history_.erase(visit_history_.begin());
  }

  std::string detail = result.summary;
  if (relaxation_level > 0) {
    detail += " (放宽等级 " + std::to_string(relaxation_level) + ")";
  }
  publishStatus(kStateExploring, detail);

  RCLCPP_INFO(
    get_logger(),
    "输出探索目标 (%.2f, %.2f, yaw=%.2f) 代价=%.3f 距离=%.2fm 增益=%.2f 惩罚=%.2f | %s",
    goal.x, goal.y, goal.yaw, goal.cost, goal.distance,
    goal.gain_normalized, goal.visit_penalty, result.summary.c_str());
}

void FrontierExplorerNode::publishGoal(const GoalCandidate & goal, const rclcpp::Time & stamp)
{
  if (!goal_pub_) {
    return;
  }
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;
  msg.pose.position.x = goal.x;
  msg.pose.position.y = goal.y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal.yaw);
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  goal_pub_->publish(msg);
}

void FrontierExplorerNode::publishStatus(const std::string & state, const std::string & detail)
{
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = state + " | " + detail;
  status_pub_->publish(msg);
}

void FrontierExplorerNode::publishMarkers(
  const GridMap & map, const FrontierSearch::Result & result, const rclcpp::Time & stamp)
{
  if (!marker_pub_) {
    return;
  }
  auto array = std::make_shared<visualization_msgs::msg::MarkerArray>();

  // 先发一个 DELETEALL，避免上一轮残留的前沿块在 RViz 里越积越多。
  {
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = map_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array->markers.push_back(std::move(clear));
  }

  constexpr double kFrontierPointScale = 0.06;
  constexpr double kCandidateScale = 0.14;
  constexpr double kGoalScale = 0.32;
  constexpr double kMarkerZ = 0.05;

  int marker_id = 0;

  // ---- 前沿连通域：通过过滤的用彩色、被过滤的用暗灰 ----
  for (std::size_t ci = 0; ci < result.clusters.size(); ++ci) {
    const FrontierCluster & cluster = result.clusters[ci];
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.ns = cluster.accepted ? "frontier_cluster" : "frontier_rejected";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = kFrontierPointScale;
    m.scale.y = kFrontierPointScale;
    m.pose.orientation.w = 1.0;
    if (cluster.accepted) {
      // 每块前沿换一个色调，方便肉眼确认「聚类是否把不同区域分开了」。
      const float t = static_cast<float>(ci % 6U) / 6.0F;
      m.color.r = 0.2F + (0.8F * t);
      m.color.g = 1.0F - (0.7F * t);
      m.color.b = 0.9F;
      m.color.a = 0.9F;
    } else {
      m.color.r = 0.35F;
      m.color.g = 0.35F;
      m.color.b = 0.35F;
      m.color.a = 0.5F;
    }
    for (const std::size_t cell : cluster.cells) {
      const auto mx = static_cast<unsigned int>(cell % map.width);
      const auto my = static_cast<unsigned int>(cell / map.width);
      geometry_msgs::msg::Point p;
      p.x = map.worldX(mx);
      p.y = map.worldY(my);
      p.z = kMarkerZ;
      m.points.push_back(p);
    }
    array->markers.push_back(std::move(m));

    // 前沿块信息文字：面积/增益/被拒原因，调参时非常有用。
    visualization_msgs::msg::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = map_frame_;
    text.ns = "frontier_label";
    text.id = marker_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = cluster.centroid_x;
    text.pose.position.y = cluster.centroid_y;
    text.pose.position.z = 0.4;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color.r = 1.0F;
    text.color.g = 1.0F;
    text.color.b = 1.0F;
    text.color.a = 0.9F;
    text.text = "#" + std::to_string(ci) + " n=" + std::to_string(cluster.size) +
      " gain=" + std::to_string(cluster.unknown_gain);
    if (!cluster.accepted) {
      text.text += " [" + cluster.reject_reason + "]";
    }
    array->markers.push_back(std::move(text));
  }

  // ---- 边界遍历路径：按前沿块顺序连线，直观看到「遍历了哪些边界块」----
  {
    visualization_msgs::msg::Marker path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    path.ns = "frontier_traversal_path";
    path.id = marker_id++;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.scale.x = 0.04;
    path.color.r = 1.0F;
    path.color.g = 1.0F;
    path.color.b = 0.0F;
    path.color.a = 0.7F;
    path.pose.orientation.w = 1.0;
    for (const FrontierCluster & cluster : result.clusters) {
      if (!cluster.accepted) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = cluster.centroid_x;
      p.y = cluster.centroid_y;
      p.z = kMarkerZ;
      path.points.push_back(p);
    }
    if (path.points.size() >= 2U) {
      array->markers.push_back(std::move(path));
    }
  }

  // ---- 候选采样点：有效=绿，被拒=红 ----
  {
    visualization_msgs::msg::Marker valid_m;
    valid_m.header.stamp = stamp;
    valid_m.header.frame_id = map_frame_;
    valid_m.ns = "candidate_valid";
    valid_m.id = marker_id++;
    valid_m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    valid_m.action = visualization_msgs::msg::Marker::ADD;
    valid_m.scale.x = kCandidateScale;
    valid_m.scale.y = kCandidateScale;
    valid_m.scale.z = kCandidateScale;
    valid_m.color.r = 0.1F;
    valid_m.color.g = 1.0F;
    valid_m.color.b = 0.1F;
    valid_m.color.a = 0.9F;
    valid_m.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker rejected_m = valid_m;
    rejected_m.ns = "candidate_rejected";
    rejected_m.id = marker_id++;
    rejected_m.color.r = 1.0F;
    rejected_m.color.g = 0.15F;
    rejected_m.color.b = 0.1F;
    rejected_m.color.a = 0.7F;

    for (const GoalCandidate & cand : result.candidates) {
      geometry_msgs::msg::Point p;
      p.x = cand.x;
      p.y = cand.y;
      p.z = kMarkerZ;
      if (cand.valid) {
        valid_m.points.push_back(p);
      } else {
        rejected_m.points.push_back(p);
      }
    }
    array->markers.push_back(std::move(valid_m));
    array->markers.push_back(std::move(rejected_m));
  }

  // ---- 选中的目标点：大箭头，方向即目标朝向 ----
  if (result.best_candidate_index >= 0) {
    const GoalCandidate & best =
      result.candidates[static_cast<std::size_t>(result.best_candidate_index)];
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.ns = "selected_goal";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = best.x;
    m.pose.position.y = best.y;
    m.pose.position.z = kMarkerZ;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, best.yaw);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = kGoalScale * 2.0;
    m.scale.y = kGoalScale * 0.5;
    m.scale.z = kGoalScale * 0.5;
    m.color.r = 1.0F;
    m.color.g = 0.55F;
    m.color.b = 0.0F;
    m.color.a = 1.0F;
    array->markers.push_back(std::move(m));
  }

  // ---- 历史访问记录：紫色小球，看清「惩罚区」分布 ----
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.ns = "visit_history";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = kCandidateScale * 0.8;
    m.scale.y = kCandidateScale * 0.8;
    m.scale.z = kCandidateScale * 0.8;
    m.color.r = 0.7F;
    m.color.g = 0.2F;
    m.color.b = 1.0F;
    m.color.a = 0.6F;
    m.pose.orientation.w = 1.0;
    for (const VisitRecord & rec : visit_history_) {
      geometry_msgs::msg::Point p;
      p.x = rec.x;
      p.y = rec.y;
      p.z = kMarkerZ;
      m.points.push_back(p);
    }
    array->markers.push_back(std::move(m));
  }

  marker_pub_->publish(*array);
}

}  // namespace astribot_s1_autonomy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astribot_s1_autonomy::FrontierExplorerNode)
