// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"
#include "astribot_s1_autonomy/frontier_goal_parameters.hpp"

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
#include <random>
#include <sstream>


#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
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
      return to == ExplorationState::kNavigating || to == ExplorationState::kGenNextPoint;
    case ExplorationState::kNavigating:
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
  latest_costmap_time_(0, 0, RCL_ROS_TIME),
  latest_odom_time_(0, 0, RCL_ROS_TIME)
{
  declareParameters();
  manually_paused_ = declare_parameter("start_paused", false);

  std::string error;
  if (!loadParameters(error)) {
    throw std::invalid_argument("探索参数校验失败: " + error);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_buffer_->setUsingDedicatedThread(true);

  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(1));
  std::random_device entropy;
  std::ostringstream identity; identity << std::hex << entropy() << entropy() << entropy();
  operator_boot_ = identity.str();
  operator_status_pub_ = create_publisher<std_msgs::msg::String>("~/operator_status", rclcpp::QoS(1).transient_local());
  operator_events_pub_ = create_publisher<std_msgs::msg::String>("~/command_events", rclcpp::QoS(100));
  operator_command_ = create_service<OperatorCommand>("~/command",
    [this](OperatorCommand::Request::SharedPtr req, OperatorCommand::Response::SharedPtr res) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      struct Audit {std::function<void()> record; ~Audit() {try {record();} catch (...) {}}};
      Audit audit{[this,req,res] {
        std_msgs::msg::String event;
        event.data = nlohmann::json({{"schema_version",1},{"kind","exploration_command_response"},
          {"command_id",req->command_id.substr(0,128)},{"operation",req->operation.substr(0,64)},
          {"expected_boot_id",req->expected_boot_id.substr(0,128)},{"expected_revision",req->expected_revision},
          {"boot_id",operator_boot_},{"revision",res->revision},{"accepted",res->accepted},
          {"reason_code",res->reason_code},{"message",res->message}}).dump();
        operator_events_pub_->publish(event);
        RCLCPP_INFO(get_logger(), "operator_command %s", event.data.c_str());
      }};
      const auto snapshot = operatorStatus();
      res->boot_id = operator_boot_; res->revision = operator_revision_;
      auto reject = [&](const std::string & code) {res->accepted = false; res->reason_code = code; res->message = code;};
      if (req->expected_boot_id != operator_boot_) {reject("REQUEST.BOOT_MISMATCH"); return;}
      if (req->command_id.empty() || req->command_id.size() > 128) {reject("REQUEST.INVALID_ID"); return;}
      const auto fingerprint = nlohmann::json({req->operation,req->expected_revision}).dump();
      auto previous = operator_requests_.find(req->command_id);
      if (previous != operator_requests_.end()) {
        if (previous->second.first != fingerprint) {reject("REQUEST.CONFLICT"); return;}
        *res = previous->second.second; return;
      }
      if (operator_requests_.size() >= 4096) {reject("REQUEST.CAPACITY"); return;}
      if (req->expected_revision != operator_revision_) {reject("STATE.REVISION_MISMATCH"); return;}
      const std::string capability = req->operation == "pause" ? "can_pause" :
        req->operation == "resume" ? "can_resume" : req->operation == "cancel_save" ? "can_cancel_save" : "";
      if (capability.empty()) {reject("REQUEST.UNSUPPORTED"); return;}
      if (!snapshot.at(capability).get<bool>()) {reject("EXPLORATION.OPERATION_BLOCKED"); return;}
      std_srvs::srv::Trigger::Response legacy;
      applyOperatorOperation(req->operation, legacy);
      res->accepted = legacy.success; res->message = legacy.message;
      res->reason_code = legacy.success ? "COMMAND.ACCEPTED" : "EXPLORATION.OPERATION_BLOCKED";
      operatorStatus(); res->revision = operator_revision_;
      operator_requests_.emplace(req->command_id, std::make_pair(fingerprint,*res));
      publishState();
    }, rmw_qos_profile_services_default, command_cb_group_);

  complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    complete_topic_, rclcpp::QoS(1).transient_local());
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    current_goal_topic_, rclcpp::QoS(1));

  rclcpp::QoS map_qos(1);
  map_qos.reliable();
  if (map_transient_local_) {
    map_qos.transient_local();
  }
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


  pause_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/pause",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onPauseService(req, res);},
    rmw_qos_profile_services_default, command_cb_group_);
  resume_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/resume",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onResumeService(req, res);},
    rmw_qos_profile_services_default, command_cb_group_);

  finalize_on_completion_ = declare_parameter("finalize_on_completion", true);
  rcl_interfaces::msg::ParameterDescriptor envelope_descriptor;
  envelope_descriptor.read_only = true;
  require_fixed_envelope_ = declare_parameter("require_fixed_envelope", false, envelope_descriptor);
  if (require_fixed_envelope_) {
    fixed_envelope_sub_ = create_subscription<astribot_navigation_msgs::msg::NavigationEnvelopeV2>(
      "/navigation/envelope_v2", 10,
      [this](astribot_navigation_msgs::msg::NavigationEnvelopeV2::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        fixed_envelope_ = std::move(message);
      }, sub_opts);
  }
  cancel_srv_ = create_service<std_srvs::srv::Trigger>("~/cancel",
    [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      applyOperatorOperation("cancel_save", *res);
    }, rmw_qos_profile_services_default, command_cb_group_);

  execution_status_sub_ = create_subscription<astribot_navigation_msgs::msg::NavigationExecutionStatus>(
    "/navigation/execution_status", rclcpp::QoS(10).transient_local(),
    [this](astribot_navigation_msgs::msg::NavigationExecutionStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (msg->source != "exploration") {return;}
      task_events_.emplace_back(msg->task_id, msg->state);
      if (task_events_.size() > 64) {task_events_.pop_front();}
      if (msg->state == "PREEMPTED") {applyTaskPreemption(msg->task_id);}
    }, sub_opts);
  policy_status_sub_ = create_subscription<astribot_navigation_msgs::msg::NavigationPolicyStatus>(
    "/navigation/policy_status", rclcpp::QoS(10),
    [this](astribot_navigation_msgs::msg::NavigationPolicyStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double age = (now() - rclcpp::Time(msg->stamp)).seconds();
      if (msg->version.task_id == current_task_id_ && age >= 0.0 && age <= msg->lease_s) {
        policy_failure_reason_ = msg->reason;
      }
    }, sub_opts);
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
  RCLCPP_INFO(get_logger(), "探索仅提交 NavigateToPose 任务；局部绕行、路径提交和保护由导航栈负责");
}

ExplorationCoordinatorNode::~ExplorationCoordinatorNode()
{
  if (search_cancel_) {search_cancel_->store(true);}
  if (search_future_.valid()) {search_future_.wait();}
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


void ExplorationCoordinatorNode::declareParameters()
{
  auto describe = [](const std::string & text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      return d;
    };

  declare_parameter<double>("selection_budget_sec", 8.0, describe("候选规划比较总墙钟预算(s)"));
  declare_parameter<double>("path_turn_weight", 0.2, describe("实际规划路径转向累计量的评分权重"));
  declare_parameter<double>("failure_cooldown_sec", 20.0, describe("临时失败候选冷却时间(s)"));
  declare_parameter<double>("failure_radius_m", 0.35, describe("失败候选冷却邻域(m)"));
  declare_parameter<double>("search_result_max_age_sec", 2.0, describe("搜索结果最大墙钟年龄(s)，过期丢弃"));
  declare_parameter<double>("search_result_max_displacement_m", 0.25, describe("搜索期间允许的最大位姿平移(m)"));
  declare_parameter<int>("completion_confirmations", 3, describe("探索完成所需连续确认次数"));
  declare_parameter<double>("completion_stable_sec", 2.0, describe("同一地图内容的完成确认窗口(s，墙钟)"));
  declare_parameter<std::string>("map_topic", "/map", describe("输入占据栅格话题(仅用于前沿搜索)"));
  declare_parameter<bool>(
    "map_transient_local", true,
    describe("map_topic 订阅是否用 TRANSIENT_LOCAL。统一 Voxel 栅格 /map "
             "在仿真和真机均为 transient_local，应保持 true"));
  declare_parameter<std::string>(
    "costmap_topic", "/global_costmap/costmap_raw",
    describe("全局代价地图话题(nav2_msgs/Costmap，仅用于下发前校验)"));
  declare_parameter<std::string>("odom_topic", "/odom", describe("里程计话题，用于速度收敛判定"));
  declare_parameter<std::string>(
    "state_topic", "/exploration/state", describe("状态机状态话题(String)"));
  declare_parameter<std::string>(
    "complete_topic", "/exploration/complete", describe("探索完成标志话题(Bool)"));
  declare_parameter<std::string>(
    "current_goal_topic", "/exploration/current_goal",
    describe("当前已下发目标话题，仅供可视化"));
  declare_parameter<std::string>(
    "nav_action_name", "/exploration/navigate_to_pose", describe("Nav2 导航动作名"));
  declare_parameter<std::string>(
    "nav_behavior_tree", "",
    describe(
      "下发目标时指定的行为树 xml 绝对路径。留空=用 bt_navigator 的默认树。"
      "探索场景填 astribot_s1_navigation 的 navigate_to_pose_explore_three_phase.xml，"
      "它把 FollowPath 的 controller_id 指向三段式控制器(终点不转朝向)"));
  declare_parameter<std::string>(
    "plan_action_name", "compute_path_to_pose", describe("Nav2 全局规划动作名(仅用于校验)"));
  declare_parameter<std::string>(
    "nav_dispatch_mode", "navigate_to_pose",
    describe(
      "仅允许 navigate_to_pose：探索提交任务，由导航策略统一规划及执行。"
      "旧 follow_path 模式已退役，配置该值会拒绝启动"));
  // Migration guards only: exploration never executes these retired modes.
  declare_parameter<bool>(
    "escape_enabled", false, describe("旧脱困模式已退役；仅允许 false，恢复由导航策略负责"));
  declare_parameter<std::string>(
    "bootstrap_mode", "disabled", describe("旧自举模式已退役；仅允许 disabled，地图不可用时等待"));
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

  declare_parameter<int>("validator.occupied_threshold", 65, describe("占据判定阈值(0~100)"));
  declare_parameter<int>("validator.free_threshold", 25, describe("空闲判定阈值(0~100)"));
  declare_parameter<double>(
    "validator.goal_clearance_radius", 0.42,
    describe("目标点【占据】净空半径(m)：碰撞约束。注意不要按外接半径取 —— 2026-08 标定证明那样会把 99% 的贴墙前沿误否决，正确取值是控制器的 xy_goal_tolerance(0.25)"));
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

  declareFrontierGoalParameters(*this);
  declare_parameter<int>("search.occupied_threshold", 65, describe("占据判定阈值(0~100)"));
  declare_parameter<int>("search.free_threshold", 25, describe("空闲判定阈值(0~100)"));
  declare_parameter<double>("search.obstacle_inflation_radius", 0.45, describe("障碍膨胀半径(m)"));
  declare_parameter<int>(
    "search.min_obstacle_cluster_cells", 3, describe("小于该格数的占据斑块视为噪声"));
  declare_parameter<bool>("search.use_eight_connectivity", true, describe("前沿判定是否用 8 邻域"));
  declare_parameter<int>("search.min_frontier_cells", 12, describe("前沿块最小格数"));
  declare_parameter<int>("search.visibility_rays", 72);
  declare_parameter<double>("search.sensor_fov_rad", 6.283185307179586);
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

  declare_parameter<int>(
    "min_known_cells_for_decision", 100,
    describe(
      "地图里至少要有多少个已知格才认为「内容足以做探索决策」。"
      "这条是 COMPLETED 判定的前置条件：冷启动时地图收到了但全是未知，"
      "前沿格数也是 0，只看前沿格数会把空地图判成「探索完成」"));
}

bool ExplorationCoordinatorNode::loadParameters(std::string & error)
{
  if (get_parameter("nav_dispatch_mode").as_string() != "navigate_to_pose" ||
    get_parameter("escape_enabled").as_bool() ||
    get_parameter("bootstrap_mode").as_string() != "disabled")
  {
    error = "旧探索执行模式已退役：仅允许 nav_dispatch_mode=navigate_to_pose, "
      "escape_enabled=false, bootstrap_mode=disabled；路径跟踪、绕行与恢复请配置统一导航策略";
    return false;
  }
  selection_budget_sec_ = get_parameter("selection_budget_sec").as_double();
  path_turn_weight_ = get_parameter("path_turn_weight").as_double();
  failure_cooldown_sec_ = get_parameter("failure_cooldown_sec").as_double();
  failure_radius_m_ = get_parameter("failure_radius_m").as_double();
  if (!std::isfinite(selection_budget_sec_) || selection_budget_sec_ <= 0.0 ||
    !std::isfinite(path_turn_weight_) || path_turn_weight_ < 0.0 ||
    !std::isfinite(failure_cooldown_sec_) || failure_cooldown_sec_ <= 0.0 ||
    !std::isfinite(failure_radius_m_) || failure_radius_m_ < 0.0)
  {error = "候选比较预算或失败冷却参数非法"; return false;}
  search_max_age_sec_ = get_parameter("search_result_max_age_sec").as_double();
  search_max_displacement_m_ = get_parameter("search_result_max_displacement_m").as_double();
  const auto confirmations = get_parameter("completion_confirmations").as_int();
  completion_stable_sec_ = get_parameter("completion_stable_sec").as_double();
  if (!std::isfinite(search_max_age_sec_) || search_max_age_sec_ <= 0.0 ||
    !std::isfinite(search_max_displacement_m_) || search_max_displacement_m_ < 0.0 ||
    confirmations < 2 || confirmations > 1000 || !std::isfinite(completion_stable_sec_) ||
    completion_stable_sec_ <= 0.0)
  {error = "搜索结果时效/位移或完成确认参数非法"; return false;}
  completion_observations_ = static_cast<unsigned int>(confirmations);

  map_topic_ = get_parameter("map_topic").as_string();
  map_transient_local_ = get_parameter("map_transient_local").as_bool();
  costmap_topic_ = get_parameter("costmap_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  state_topic_ = get_parameter("state_topic").as_string();
  complete_topic_ = get_parameter("complete_topic").as_string();
  current_goal_topic_ = get_parameter("current_goal_topic").as_string();
  nav_action_name_ = get_parameter("nav_action_name").as_string();
  nav_behavior_tree_ = get_parameter("nav_behavior_tree").as_string();
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
  if (use_costmap_for_validation_ && vp.goal_clearance_radius > 0.44) {
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
  if (!loadFrontierGoalParameters(*this,sp,error)) {return false;}
  sp.occupied_threshold = static_cast<int>(get_parameter("search.occupied_threshold").as_int());
  sp.free_threshold = static_cast<int>(get_parameter("search.free_threshold").as_int());
  sp.obstacle_inflation_radius = get_parameter("search.obstacle_inflation_radius").as_double();
  sp.min_obstacle_cluster_cells =
    static_cast<int>(get_parameter("search.min_obstacle_cluster_cells").as_int());
  sp.use_eight_connectivity = get_parameter("search.use_eight_connectivity").as_bool();
  sp.min_frontier_cells = static_cast<int>(get_parameter("search.min_frontier_cells").as_int());
  sp.gain_window_radius = get_parameter("search.gain_window_radius").as_double();
  sp.visibility_rays = static_cast<int>(get_parameter("search.visibility_rays").as_int());
  sp.sensor_fov_rad = get_parameter("search.sensor_fov_rad").as_double();
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

  const auto known_cells = get_parameter("min_known_cells_for_decision").as_int();
  if (known_cells < 1) {
    error = "min_known_cells_for_decision 必须 >=1";
    return false;
  }
  min_known_cells_for_decision_ = static_cast<std::size_t>(known_cells);
  error.clear();
  return true;
}


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
  if (unknown_clearance_radius_ >= msg->info.resolution) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "validator.goal_unknown_clearance_radius=%.3fm >= 地图分辨率 %.3fm，"
      "这会让【每一个】前沿候选都被目标校验否决(前沿格的未知邻居只有一格远)，"
      "探索将永远发不出目标。请把该参数改成 0.0",
      unknown_clearance_radius_, msg->info.resolution);
  }
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
  if (!latest_map_ || !sameMapContent(*latest_map_, *snapshot)) {
    latest_map_ = std::move(snapshot);
    ++map_revision_;
  }
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

  const CostmapGridStats stats = summarizeGrid(*snapshot);
  const unsigned int width = snapshot->width;
  const unsigned int height = snapshot->height;
  const double resolution = snapshot->resolution;
  const double ox = snapshot->origin_x;
  const double oy = snapshot->origin_y;

  bool first_frame = false;
  {
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
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!latest_map_ || !latest_map_->consistent()) {
      why = "校验用 /map 不可用";
      return nullptr;
    }
    return latest_map_;
  }

  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!latest_costmap_ || !latest_costmap_->consistent()) {
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
  odom_speed_ = std::hypot(vx, vy);
  latest_odom_time_ = now();
}

bool ExplorationCoordinatorNode::mapUsable(const GridMap & map, std::size_t & known_cells) const
{
  known_cells = 0U;
  for (const int8_t v : map.data) {
    if (v >= 0) {                      // >=0 即已知（0 空闲 ~ 100 占据），-1 是未知
      ++known_cells;
      if (known_cells >= min_known_cells_for_decision_) {return true;}
    }
  }
  return known_cells >= min_known_cells_for_decision_;
}

bool ExplorationCoordinatorNode::stackReady(std::string & why)
{
  if (require_fixed_envelope_) {
    why = fixedEnvelopeReadiness(fixed_envelope_.get(), now().nanoseconds(),
      fixed_envelope_sub_->get_publisher_count());
    if (!why.empty()) return false;
  }
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
    const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      frame, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(0.0));
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
  pending_nav_failure_ = false;
  completion_tracker_.reset();
  ++search_epoch_;
  if (search_cancel_) {search_cancel_->store(true);}
  ++plan_epoch_;
  if (plan_client_ && plan_goal_handle_) {plan_client_->async_cancel_goal(plan_goal_handle_);}
  plan_goal_handle_.reset();
  plan_request_in_flight_.store(false);
  candidates_.clear();
  evaluated_candidates_.clear();
  candidate_index_ = 0U;
}


void ExplorationCoordinatorNode::transitionTo(ExplorationState next, const std::string & why)
{
  transition_reason_ = why;
  if (state_ == next) {
    return;                                   // 空转换不刷新计时，避免驻留/冷却计时被反复重置
  }
  if (!isTransitionAllowed(state_, next)) {
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
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (session_ending_) {
    if (!finalize_client_) {
      finalize_client_ = create_client<std_srvs::srv::Trigger>(
        session_end_reason_ == "COMPLETED" ? "/mapping_session/finalize_completed" : "/mapping_session/finalize_canceled",
        rmw_qos_profile_services_default, io_cb_group_);
    }
    progress_detail_ = session_end_reason_ + std::string(nav_goal_in_flight_.load() ?
      "_WAIT_NAV_TERMINAL" : "_WAIT_MAP_SAVE");
    if (!nav_goal_in_flight_.load() && !finalize_requested_ && finalize_client_->service_is_ready()) {
      finalize_requested_ = true;
      finalize_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto response = future.get();
          RCLCPP_INFO(get_logger(), "建图收尾请求 accepted=%d: %s",
            response->success, response->message.c_str());
        });
    }
    publishState();
    return;
  }
  if (pending_nav_failure_) {
    if (std::chrono::steady_clock::now() < failure_classification_at_) {return;}
    pending_nav_failure_ = false;
    registerNavFailure(policy_failure_reason_.empty() ? "导航执行中止" : policy_failure_reason_);
  }
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
    default:
      transitionTo(ExplorationState::kPaused, "非法或已退役的探索状态");
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
      return;
    }
  }
  transitionTo(ExplorationState::kGenNextPoint, "地图/定位/里程计均就绪");
}


void ExplorationCoordinatorNode::tickGenNextPoint()
{
  if (!allowsGoalGeneration(state_)) {
    RCLCPP_ERROR(get_logger(), "在 %s 态被要求生成目标点，已拦截", toString(state_));
    transitionTo(ExplorationState::kIdle, "非法的生成时机");
    return;
  }
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

  if (!search_future_.valid()) {
    search_cancel_ = std::make_shared<std::atomic<bool>>(false);
    search_map_ = map;
    search_started_ = std::chrono::steady_clock::now();
    search_start_x_ = rx; search_start_y_ = ry;
    search_request_epoch_ = search_epoch_;
    const auto params = search_params_;
    const auto history = visit_history_;
    const auto canceled = search_cancel_;
    search_future_ = std::async(std::launch::async, [map, rx, ry, params, history, canceled]() {
      FrontierSearch search;
      FrontierSearch::Result result;
      std::string error;
      if (search.configure(params, error)) {
        search.search(*map, rx, ry, history, result, [canceled]() {return canceled->load();});
      } else {result.summary = error;}
      return result;
    });
    return;
  }
  if (search_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {return;}
  FrontierSearch::Result result;
  try {result = search_future_.get();}
  catch (const std::exception & error) {
    transitionTo(ExplorationState::kPaused, std::string("搜索工作线程失败: ") + error.what());
    return;
  }
  const double search_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - search_started_).count();
  if (search_request_epoch_ != search_epoch_ || search_cancel_->load() ||
    !usableSearchSnapshot(*search_map_, *map, search_age,
      std::hypot(rx - search_start_x_, ry - search_start_y_),
      search_max_age_sec_, search_max_displacement_m_))
  {
    ++search_discarded_;
    completion_tracker_.reset();
    progress_detail_ = "SEARCH_RESULT_STALE";
    return;
  }
  if (result.status != FrontierSearch::Status::kOk) {
    completion_tracker_.reset();
    progress_detail_ = "SEARCH_FAILED";
    transitionTo(ExplorationState::kPaused, "前沿搜索失败，不能判为探索完成");
    return;
  }
  raw_frontiers_ = result.raw_frontier_cell_count;
  reachable_frontiers_ = result.reachable_frontier_cell_count;
  unresolved_unknown_ = result.unknown_cell_count;
  const auto raw_frontier_cells = result.raw_frontier_cell_count;
  const bool content_changed = map != search_map_;  // equal content reuses its immutable snapshot
  if (content_changed) {
    completion_tracker_.reset();
    ++search_revalidated_;
    // An old empty result may not certify completion or no reachable frontiers in a newer map.
    if (raw_frontier_cells == 0 || reachable_frontiers_ == 0) {return;}
    result.candidates.erase(std::remove_if(result.candidates.begin(), result.candidates.end(),
      [&](const GoalCandidate & candidate) {
        return !isCurrentFrontier(*map, candidate.frontier_x, candidate.frontier_y,
          search_params_.free_threshold, search_params_.use_eight_connectivity) ||
          !search_.goalFootprintIsKnownFree(*map,candidate.x,candidate.y,candidate.yaw);
      }), result.candidates.end());
  }
  if (raw_frontier_cells == 0U) {
    std::size_t known = 0U;
    if (!mapUsable(*map, known)) {
      completion_tracker_.reset();
      progress_detail_ = "INSUFFICIENT_KNOWN_MAP";
      transitionTo(ExplorationState::kIdle, "地图内容不足，等待有效观测");
      return;
    }
    if (unresolved_unknown_ > 0) {
      completion_tracker_.reset();
      progress_detail_ = "UNOBSERVABLE_UNKNOWN";
      transitionTo(ExplorationState::kPaused, "仍有未知格但没有观测前沿，不判为完成");
      return;
    }
    uint64_t revision;
    {std::lock_guard<std::mutex> lock(map_mutex_);
      if (latest_map_ != map) {completion_tracker_.reset(); return;}
      revision = map_revision_;
    }
    const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    progress_detail_ = "CONFIRMING_COMPLETE";
    if (completion_tracker_.observe(revision, seconds, true,
      completion_observations_, completion_stable_sec_))
    {
      completed_map_revision_ = revision;
      progress_detail_ = "CURRENT_MAP_COMPLETE";
      transitionTo(ExplorationState::kCompleted, "当前地图已知区域连续确认无前沿及未知格");
      if (finalize_on_completion_) {
        session_ending_ = true;
        session_end_reason_ = "COMPLETED";
        manually_paused_ = true;
      }
    }
    return;
  }
  completion_tracker_.reset();
  if (reachable_frontiers_ == 0 && !content_changed) {
    progress_detail_ = "UNREACHABLE_FRONTIERS";
    transitionTo(ExplorationState::kPaused, "仍有前沿，但当前包络和可达域下无可达前沿");
    return;
  }
  progress_detail_ = "EXPLORING";
  auto candidates = generateCandidates(result);

  if (candidates.empty()) {
    if (cooldown_filtered_ > 0) {progress_detail_ = "CANDIDATE_COOLDOWN"; return;}
    std::string ready_why;
    if (!stackReady(ready_why)) {
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
      transitionTo(ExplorationState::kPaused, "连续多轮无合法候选点");
    }
    return;
  }

  failure_budget_.onCandidatesSampled();
  evaluated_candidates_.clear();
  selection_started_ = std::chrono::steady_clock::now();
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
  if (std::chrono::duration<double>(std::chrono::steady_clock::now()-selection_started_).count() >= selection_budget_sec_) {
    ++plan_epoch_;
    if (plan_goal_handle_) {plan_client_->async_cancel_goal(plan_goal_handle_);}
    plan_goal_handle_.reset(); plan_request_in_flight_.store(false);
    onCandidatesExhausted("候选规划比较预算耗尽");
    return;
  }
  if (!plan_request_in_flight_.load()) {
    requestPlanForCurrentCandidate();
    return;
  }
  const double waited = (now() - plan_requested_time_).seconds();
  if (waited > plan_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "路径校验请求超时 %.1fs，丢弃当前候选", waited);
    if (plan_client_ && plan_goal_handle_) {
      plan_client_->async_cancel_goal(plan_goal_handle_);
    }
    ++plan_epoch_;
    plan_goal_handle_.reset();
    plan_request_in_flight_.store(false);
    rejectCurrentCandidate("路径校验超时");
  }
}

void ExplorationCoordinatorNode::tickNavigating()
{
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

  // Path replanning and commit belong to PolicyExecution / RouteCoordinator.

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
    RCLCPP_WARN(
      get_logger(), "Nav2 报成功但位置偏差 %.2fm > 容差 %.2fm，按导航失败处理",
      dxy, arrival_xy_tolerance_);
    dwell_active_ = false;
    registerNavFailure("抵达位置偏差超容差");
    return;
  }
  const double dyaw = std::fabs(normalizeAngle(active_goal_.yaw - ryaw));
  if (check_yaw_) {
    if (dyaw > arrival_yaw_tolerance_) {
      RCLCPP_WARN(
        get_logger(), "朝向偏差 %.3frad (%.1f°) > 容差 %.3frad (%.1f°)，按导航失败处理",
        dyaw, dyaw * 180.0 / M_PI, arrival_yaw_tolerance_,
        arrival_yaw_tolerance_ * 180.0 / M_PI);
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
    dwell_active_ = false;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "位置已达标但速度 %.3f > %.3f m/s，驻留计时重置", speed, settle_speed_);
    return;
  }

  if (!dwell_active_) {
    dwell_active_ = true;
    dwell_started_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "开始驻留计时(位置偏差 %.3fm/容差 %.3f 朝向偏差 %.3frad=%.1f°/容差 %.3f%s 速度 %.3fm/s)",
      dxy, arrival_xy_tolerance_, dyaw, dyaw * 180.0 / M_PI, arrival_yaw_tolerance_,
      check_yaw_ ? "" : "(未启用校验)", speed);
    return;
  }
  const double dwelled = (now() - dwell_started_time_).seconds();
  if (dwelled < dwell_time_sec_) {
    return;                                   // 还没稳够，继续等
  }

  ++goals_succeeded_;
  nav_failure_count_ = 0;
  auto_resume_count_ = 0;
  recordVisit(active_goal_.x, active_goal_.y);
  has_active_goal_ = false;
  dwell_active_ = false;
  resetCycleState();
  RCLCPP_INFO(
    get_logger(),
    "目标收敛完成(位置偏差 %.3fm 朝向偏差 %.3frad=%.1f° 驻留 %.2fs)，"
    "累计成功 %" PRIu64 " 个，允许生成下一目标",
    dxy, dyaw, dyaw * 180.0 / M_PI, dwelled, goals_succeeded_);
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
  if (auto_resume_count_ >= max_auto_resume_attempts_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "自动恢复已达上限 %d 次，停止重试，等待人工调用 ~/resume 服务",
      max_auto_resume_attempts_);
    return;
  }
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
  uint64_t revision;
  {std::lock_guard<std::mutex> lock(map_mutex_); revision = map_revision_;}
  if (revision != completed_map_revision_) {
    completion_tracker_.reset();
    progress_detail_ = "MAP_CHANGED_RECHECK";
    transitionTo(ExplorationState::kIdle, "完成后地图内容变化，重新评估探索需求");
    return;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kLogThrottleMs,
    "探索完成，已停止生成新目标(成功 %" PRIu64 " 个目标)", goals_succeeded_);
}


std::vector<ExplorationCoordinatorNode::GoalCandidatePose>
ExplorationCoordinatorNode::generateCandidates(const FrontierSearch::Result & result)
{
  std::vector<GoalCandidatePose> out;
  cooldown_filtered_ = 0;
  uint64_t revision;
  {std::lock_guard<std::mutex> lock(map_mutex_); revision = map_revision_;}
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

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
      ++dropped_by_search;
      if (search_reject_sample.empty()) {
        search_reject_sample = c.reject_reason;
      }
      RCLCPP_DEBUG(
        get_logger(), "候选点(%.2f, %.2f) 被前沿搜索否决: %s",
        c.x, c.y, c.reject_reason.c_str());
      continue;
    }
    if (candidate_failures_.blocked(c.x, c.y, seconds, revision, failure_radius_m_)) {
      ++cooldown_filtered_; continue;
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
    g.frontier_x = c.frontier_x;
    g.frontier_y = c.frontier_y;
    g.x = c.x;
    g.y = c.y;
    g.yaw = c.yaw;
    g.cost = c.cost;
    g.euclidean_distance = c.distance;
    g.cluster_index = c.cluster_index;
    out.push_back(g);
    RCLCPP_INFO(get_logger(),
      "FRONTIER_RETREAT frontier=(%.3f,%.3f) goal=(%.3f,%.3f) yaw=%.3f retreat_m=%.3f",
      c.frontier_x,c.frontier_y,c.x,c.y,c.yaw,c.retreat_distance);
  }

  std::sort(
    out.begin(), out.end(),
    [](const GoalCandidatePose & a, const GoalCandidatePose & b) {return a.cost < b.cost;});
  if (out.size() > static_cast<std::size_t>(max_candidates_per_cycle_)) {
    // First cover different frontiers, then fill the remaining budget with alternate viewpoints.
    std::vector<GoalCandidatePose> diverse;
    std::vector<bool> selected(out.size(), false);
    for (std::size_t i=0;i<out.size() && diverse.size()<static_cast<std::size_t>(max_candidates_per_cycle_);++i) {
      if (std::none_of(diverse.begin(), diverse.end(), [&](const GoalCandidatePose & g) {
        return g.cluster_index == out[i].cluster_index;
      })) {diverse.push_back(out[i]); selected[i]=true;}
    }
    for (std::size_t i=0;i<out.size() && diverse.size()<static_cast<std::size_t>(max_candidates_per_cycle_);++i) {
      if (!selected[i]) {diverse.push_back(out[i]);}
    }
    out = std::move(diverse);
  }

  RCLCPP_INFO(
    get_logger(),
    "前沿搜索: 原始前沿格 %zu, 有效前沿块 %zu, 候选 %zu 个"
    "(搜索否决 %zu, 目标校验淘汰 %zu) | %s",
    result.raw_frontier_cell_count, result.accepted_cluster_count, out.size(),
    dropped_by_search, dropped_unknown, result.summary.c_str());
  if (out.empty() && dropped_by_search > 0U) {
    RCLCPP_WARN(
      get_logger(), "全部 %zu 个候选被前沿搜索否决，首个原因: %s",
      dropped_by_search, search_reject_sample.c_str());
  }
  if (out.empty() && dropped_unknown > 0U) {
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

  const auto epoch = ++plan_epoch_;
  rclcpp_action::Client<ComputePathToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this, epoch](const PlanGoalHandle::SharedPtr & handle) {onPlanGoalResponse(epoch, handle);};
  opts.result_callback =
    [this, epoch](const PlanGoalHandle::WrappedResult & result) {onPlanResult(epoch, result);};
  plan_client_->async_send_goal(goal, opts);
  plan_requested_time_ = now();
  RCLCPP_INFO(
    get_logger(), "校验候选 %zu/%zu: 请求到 (%.2f, %.2f) 的全局路径",
    candidate_index_ + 1U, candidates_.size(), c.x, c.y);
}

void ExplorationCoordinatorNode::onPlanGoalResponse(uint64_t epoch, const PlanGoalHandle::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (epoch != plan_epoch_) {
    if (handle) {plan_client_->async_cancel_goal(handle);}
    return;
  }
  if (!handle) {
    plan_request_in_flight_.store(false);
    RCLCPP_WARN(get_logger(), "全局规划请求被 Nav2 拒绝，丢弃当前候选");
    rejectCurrentCandidate("规划请求被拒绝");
    return;
  }
  plan_goal_handle_ = handle;
}

void ExplorationCoordinatorNode::onPlanResult(uint64_t epoch, const PlanGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (epoch != plan_epoch_) {
    return;
  }
  plan_request_in_flight_.store(false);
  plan_goal_handle_.reset();

  if (state_ != ExplorationState::kValidating) {
    RCLCPP_DEBUG(get_logger(), "收到迟到的规划结果(当前 %s)，已丢弃", toString(state_));
    return;
  }
  if (candidate_index_ >= candidates_.size()) {
    onCandidatesExhausted("规划结果到达时候选队列已空");
    return;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
    rejectCurrentCandidate("全局规划失败；恢复由导航策略负责");
    return;
  }

  const GoalCandidatePose candidate = candidates_[candidate_index_];
  const auto & path = result.result->path;
  if (path.poses.empty()) {
    rejectCurrentCandidate("规划返回空路径");
    return;
  }

  std::string grid_why;
  const std::shared_ptr<GridMap> grid = validationGrid(grid_why);
  if (!grid) {
    RCLCPP_WARN(
      get_logger(), "校验用栅格图不可用(%s)，本次候选作废(绝不在无图状态下下发)",
      grid_why.c_str());
    rejectCurrentCandidate("校验用栅格图不可用: " + grid_why);
    return;
  }

  const ValidationResult goal_check = validator_.validateGoal(*grid, candidate.x, candidate.y);
  if (!goal_check.valid) {
    rejectCurrentCandidate("目标点复检未通过: " + goal_check.reason, true);
    return;
  }

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
    rejectCurrentCandidate("路径穿越未知/占据区", true);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "候选 (%.2f, %.2f) 双层校验通过: 路径 %zu 个顶点 / %zu 个采样点全部位于已知可通行区",
    candidate.x, candidate.y, path.poses.size(), path_check.samples_checked);
  auto evaluated = candidate;
  const auto effort = pathEffort(pts);
  evaluated.cost += search_params_.weight_distance * (effort.length - candidate.euclidean_distance) +
    path_turn_weight_ * effort.turning;
  if (!std::isfinite(evaluated.cost)) {rejectCurrentCandidate("规划路径评分非有限值"); return;}
  evaluated_candidates_.push_back({evaluated, std::move(pts)});
  RCLCPP_INFO(get_logger(), "候选路径比较: 长度=%.2f 转向=%.2f 评分=%.3f", effort.length, effort.turning, evaluated.cost);
  ++candidate_index_;
  if (candidate_index_ >= candidates_.size()) {onCandidatesExhausted("候选路径比较完成");}
}

void ExplorationCoordinatorNode::rejectCurrentCandidate(const std::string & reason, bool map_dependent)
{
  ++candidates_rejected_;
  if (candidate_index_ < candidates_.size()) {
    const auto & c = candidates_[candidate_index_];
    recordCandidateFailure(c.x, c.y, reason, map_dependent);
    RCLCPP_WARN(
      get_logger(), "丢弃候选 %zu/%zu (%.2f, %.2f): %s",
      candidate_index_ + 1U, candidates_.size(), c.x, c.y, reason.c_str());
    ++candidate_index_;
  }
  if (candidate_index_ >= candidates_.size()) {
    onCandidatesExhausted(reason);
    return;
  }
}

void ExplorationCoordinatorNode::recordCandidateFailure(double x, double y, const std::string & reason, bool map_dependent)
{
  uint64_t revision;
  {std::lock_guard<std::mutex> lock(map_mutex_); revision = map_revision_;}
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  candidate_failures_.record(x, y, seconds, failure_cooldown_sec_, revision, map_dependent, reason);
}

bool ExplorationCoordinatorNode::dispatchBestValidatedGoal()
{
  std::sort(evaluated_candidates_.begin(), evaluated_candidates_.end(),
    [](const EvaluatedCandidate & a, const EvaluatedCandidate & b) {return a.goal.cost < b.goal.cost;});
  std::string why;
  const auto grid = validationGrid(why);
  std::shared_ptr<GridMap> map;
  {std::lock_guard<std::mutex> lock(map_mutex_); map = latest_map_;}
  if (!grid || !map || !stackReady(why)) {return false;}
  for (const auto & candidate : evaluated_candidates_) {
    const auto & g = candidate.goal;
    if (!isCurrentFrontier(*map, g.frontier_x, g.frontier_y,
      search_params_.free_threshold, search_params_.use_eight_connectivity) ||
      !search_.goalFootprintIsKnownFree(*map, g.x, g.y, g.yaw) ||
      !validator_.validateGoal(*grid, g.x, g.y).valid ||
      !validator_.validatePath(*grid, candidate.path, PlanarPoint{g.x, g.y}).valid) {continue;}
    dispatchNavGoal(g);  // Only submit the task; the policy chain still validates the executed path.
    return nav_goal_in_flight_.load();
  }
  return false;
}

void ExplorationCoordinatorNode::onCandidatesExhausted(const std::string & reason)
{
  if (dispatchBestValidatedGoal()) {return;}
  resetCycleState();
  const bool budget_used_up = failure_budget_.onAllCandidatesInvalid();
  RCLCPP_WARN(
    get_logger(), "本轮候选全部不合法(%s)，连续校验失败 %d/%d",
    reason.c_str(), failure_budget_.validation_failures,
    failure_budget_.max_validation_failures);
  if (budget_used_up) {
    transitionTo(ExplorationState::kPaused, "连续多轮候选全部不合法");
    return;
  }
  transitionTo(ExplorationState::kGenNextPoint, "重新采样候选点");
}


void ExplorationCoordinatorNode::dispatchNavGoal(const GoalCandidatePose & goal)
{
  if (state_ != ExplorationState::kValidating) {
    RCLCPP_ERROR(
      get_logger(), "试图在 %s 态下发目标，已拦截(下发只允许发生在 VALIDATING)",
      toString(state_));
    transitionTo(ExplorationState::kIdle, "非法下发时机");
    return;
  }
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
  nav_goal.behavior_tree = nav_behavior_tree_;

  const auto epoch = ++nav_epoch_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this, epoch](const NavGoalHandle::SharedPtr & handle) {onNavGoalResponse(epoch, handle);};
  opts.result_callback =
    [this, epoch](const NavGoalHandle::WrappedResult & result) {onNavResult(epoch, result);};
  nav_client_->async_send_goal(nav_goal, opts);

  active_goal_ = goal;
  has_active_goal_ = true;
  nav_started_time_ = now();
  dwell_active_ = false;
  ++goals_dispatched_;
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

void ExplorationCoordinatorNode::onNavGoalResponse(uint64_t epoch, const NavGoalHandle::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (epoch != nav_epoch_) {
    if (handle) {nav_client_->async_cancel_goal(handle);}
    else {nav_goal_in_flight_.store(false);}
    return;
  }
  if (!handle) {
    nav_goal_in_flight_.store(false);
    has_active_goal_ = false;
    RCLCPP_ERROR(get_logger(), "导航目标被 Nav2 拒绝");
    registerNavFailure("导航目标被拒绝");
    return;
  }
  nav_goal_handle_ = handle;
  current_task_id_.clear();
  static const char hex[] = "0123456789abcdef";
  for (const auto byte : handle->get_goal_id()) {
    current_task_id_ += hex[byte >> 4];current_task_id_ += hex[byte & 15];
  }
  policy_failure_reason_.clear();
  for (auto it = task_events_.rbegin(); it != task_events_.rend(); ++it) {
    if (it->first == current_task_id_) {
      if (it->second == "PREEMPTED") {applyTaskPreemption(it->first);}
      break;
    }
  }
  RCLCPP_INFO(get_logger(), "导航目标已被 Nav2 接受，开始执行");
}

void ExplorationCoordinatorNode::applyTaskPreemption(const std::string & task_id)
{
  if (task_id != current_task_id_) {return;}
  pending_nav_failure_ = false;
  manually_paused_ = true;
  has_active_goal_ = false;
  resetCycleState();
  transitionTo(ExplorationState::kPaused, "任务被抢占，等待显式恢复探索");
}

void ExplorationCoordinatorNode::onNavResult(uint64_t epoch, const NavGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (epoch != nav_epoch_) {
    nav_goal_in_flight_.store(false);  // terminal result releases cancellation barrier
    return;
  }
  nav_goal_in_flight_.store(false);
  nav_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (state_ == ExplorationState::kNavigating) {
        RCLCPP_INFO(get_logger(), "Nav2 报告导航成功，进入抵达校验");
        transitionTo(ExplorationState::kArrived, "Nav2 报告成功，待实测校验");
      } else {
        RCLCPP_DEBUG(
          get_logger(), "在 %s 态收到导航成功结果，已忽略", toString(state_));
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      if (!manually_paused_) {
        pending_nav_failure_ = true;
        failure_classification_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
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
  ++nav_epoch_;
  if (nav_client_ && nav_goal_handle_) {
    RCLCPP_WARN(get_logger(), "取消在途导航目标: %s", reason.c_str());
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_.reset();
  }

  // Keep in-flight set until rejection or terminal result, including late acceptance.
  has_active_goal_ = false;
  dwell_active_ = false;
}

void ExplorationCoordinatorNode::registerNavFailure(const std::string & reason)
{
  std::string ready_why;
  const bool ready = stackReady(ready_why);
  if (!ready) {
    // Infrastructure failures must not penalize an otherwise useful exploration target.
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
    recordCandidateFailure(active_goal_.x, active_goal_.y, reason, false);
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


nlohmann::json ExplorationCoordinatorNode::operatorStatus()
{
  std::string readiness;
  bool ready = stackReady(readiness);
  if (ready) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    size_t known = 0;
    if (!latest_map_ || !mapUsable(*latest_map_, known)) {
      ready = false; readiness = "地图已知区域不足，不能决策";
    }
  }
  const bool in_flight = nav_goal_in_flight_.load();
  const bool paused = state_ == ExplorationState::kPaused || manually_paused_;
  const bool cancel_pending = in_flight && !has_active_goal_;
  const bool can_pause = !session_ending_ && !manually_paused_;
  const bool can_resume = !session_ending_ && paused && ready && !in_flight;
  std::string code = session_ending_ ? "EXPLORATION.ENDING" :
    cancel_pending ? "NAV.CANCEL_UNCONFIRMED" : !ready ? "EXPLORATION.NOT_READY" :
    paused ? (manually_paused_ ? "EXPLORATION.PAUSED" : "EXPLORATION.RETRY_WAIT") : "EXPLORATION.RUNNING";
  nlohmann::json status = {{"schema_version",1},{"boot_id",operator_boot_},
    {"state",toString(state_)},{"reason_code",code},{"ready",ready},
    {"readiness_detail",readiness},{"transition_reason",transition_reason_},
    {"progress",progress_detail_},{"manual_pause",manually_paused_},
    {"session_ending",session_ending_},{"end_reason",session_end_reason_},
    {"goal_in_flight",in_flight},{"cancel_pending",cancel_pending},
    {"can_pause",can_pause},{"can_resume",can_resume},{"can_cancel_save",!session_ending_},
    {"can_start_new_session",false},{"completion_scope","current_map"}};
  // Revision covers command preconditions, not continuously changing diagnostics.
  const auto signature = nlohmann::json({toString(state_),code,ready,manually_paused_,
    session_ending_,in_flight,can_pause,can_resume}).dump();
  if (signature != operator_signature_) {operator_signature_ = signature; ++operator_revision_;}
  status["revision"] = operator_revision_;
  status["raw_frontiers"] = raw_frontiers_;
  status["reachable_frontiers"] = reachable_frontiers_;
  status["unknown_cells"] = unresolved_unknown_;
  status["auto_resume_count"] = auto_resume_count_;
  status["max_auto_resume_attempts"] = max_auto_resume_attempts_;
  return status;
}

// Caller owns state_mutex_; legacy and versioned services share these mutations.
void ExplorationCoordinatorNode::applyOperatorOperation(
  const std::string & operation, std_srvs::srv::Trigger::Response & response)
{
  if (operation == "cancel_save") {
    if (!session_ending_) {
      session_ending_ = true; session_end_reason_ = "CANCELED"; manually_paused_ = true;
      cancelActiveNavGoal("取消探索并保存建图"); resetCycleState();
      transitionTo(ExplorationState::kPaused, "探索取消，等待导航终态后保存地图");
    }
    response.success = true;
    response.message = "取消已受理；等待导航终态、停稳与保存结果，不等于已保存";
    return;
  }
  if (session_ending_) {
    response.success = false; response.message = "会话已收尾；不能恢复，需创建新的 SLAM 和探索会话"; return;
  }
  if (operation == "pause") {
    manually_paused_ = true; cancelActiveNavGoal("人工暂停"); resetCycleState();
    transitionTo(ExplorationState::kPaused, "收到人工暂停请求");
    response.success = true; response.message = "暂停已受理，目标发布已冻结；仍需等待导航终态";
  } else if (operation == "resume") {
    manually_paused_ = false; failure_budget_.resetAll(); nav_failure_count_ = 0;
    auto_resume_count_ = 0; resetCycleState();
    transitionTo(ExplorationState::kIdle, "收到人工恢复请求");
    response.success = true; response.message = "探索已恢复；实际派发仍由就绪检查决定";
  } else {response.success = false; response.message = "不支持的操作";}
}

void ExplorationCoordinatorNode::publishState()
{
  if (!state_pub_) {
    return;
  }
  std::string detail = "state=";
  detail += toString(state_);
  detail += " goal_in_flight=";
  detail += nav_goal_in_flight_.load() ? "1" : "0";
  if (nav_goal_in_flight_.load() && !has_active_goal_) {detail += " cancel_pending=1";}
  if (has_active_goal_) {
    char buf[96];
    snprintf(buf, sizeof(buf), " goal=(%.2f,%.2f)", active_goal_.x, active_goal_.y);
    detail += buf;
  }
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
  detail += " progress=" + progress_detail_ + " completion_scope=current_map";
  detail += " raw_frontiers=" + std::to_string(raw_frontiers_);
  detail += " reachable_frontiers=" + std::to_string(reachable_frontiers_);
  detail += " unknown_cells=" + std::to_string(unresolved_unknown_);
  detail += " cooldown_filtered=" + std::to_string(cooldown_filtered_);
  detail += " search_discarded=" + std::to_string(search_discarded_);
  detail += " search_revalidated=" + std::to_string(search_revalidated_);
  if (manually_paused_) {
    detail += " manual_pause=1";
  }

  std_msgs::msg::String msg;
  msg.data = detail;
  state_pub_->publish(msg);
  std_msgs::msg::String operator_msg; operator_msg.data = operatorStatus().dump();
  operator_status_pub_->publish(operator_msg);

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
  applyOperatorOperation("pause", *response);
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
  applyOperatorOperation("resume", *response);
}

}  // namespace astribot_s1_autonomy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astribot_s1_autonomy::ExplorationCoordinatorNode)
