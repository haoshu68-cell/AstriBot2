// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/three_phase_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace astribot_s1_path_tracking
{

namespace
{

/// 从四元数取 yaw。用 tf2::getYaw 而不是自己展开，避免符号约定写错。
double yawOf(const geometry_msgs::msg::Pose & p)
{
  return tf2::getYaw(p.orientation);
}

}  // namespace

void ThreePhaseController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent_.lock();
  if (!node) {
    throw nav2_core::PlannerException("ThreePhaseController: 父节点已失效，无法配置");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  phase_started_ = clock_->now();

  declareAndLoadParams();
  loadInnerController(parent_, tf_, costmap_ros_);

  RCLCPP_INFO(
    logger_,
    "[%s] 三段式跟踪已配置: 内层=%s 起步对齐=%s 终点对齐=%s 跟踪段横移=%s "
    "对齐容差=%.3frad 起步触发=%.3frad 前视=%.2fm 对齐超时=%.1fs",
    name_.c_str(), inner_plugin_.c_str(),
    align_start_enabled_ ? "开" : "关",
    align_goal_enabled_ ? "开" : "关(探索场景)",
    zero_vy_in_follow_ ? "禁止(前向为主)" : "允许(全向)",
    align_tol_rad_, start_min_angle_rad_, lookahead_m_, align_timeout_sec_);
}

void ThreePhaseController::declareAndLoadParams()
{
  auto node = parent_.lock();
  if (!node) {
    throw nav2_core::PlannerException("ThreePhaseController: 读参数时父节点已失效");
  }
  using nav2_util::declare_parameter_if_not_declared;
  const std::string p = name_ + ".";

  declare_parameter_if_not_declared(
    node, p + "inner_controller_plugin",
    rclcpp::ParameterValue(
      std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")));
  declare_parameter_if_not_declared(node, p + "align_start_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "align_goal_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "zero_vy_in_follow", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "align_kp", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, p + "align_max_vel", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, p + "align_floor_vel", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, p + "align_tolerance", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, p + "start_min_angle", rclcpp::ParameterValue(0.20));
  declare_parameter_if_not_declared(node, p + "start_heading_lookahead", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, p + "align_timeout", rclcpp::ParameterValue(15.0));
  declare_parameter_if_not_declared(node, p + "follow_timeout", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p + "fallback_xy_tolerance", rclcpp::ParameterValue(0.18));
  declare_parameter_if_not_declared(
    node, p + "fallback_yaw_tolerance", rclcpp::ParameterValue(0.20));

  node->get_parameter(p + "inner_controller_plugin", inner_plugin_);
  node->get_parameter(p + "align_start_enabled", align_start_enabled_);
  node->get_parameter(p + "align_goal_enabled", align_goal_enabled_);
  node->get_parameter(p + "zero_vy_in_follow", zero_vy_in_follow_);
  node->get_parameter(p + "align_kp", align_kp_);
  node->get_parameter(p + "align_max_vel", align_max_vel_);
  node->get_parameter(p + "align_floor_vel", align_floor_vel_);
  node->get_parameter(p + "align_tolerance", align_tol_rad_);
  node->get_parameter(p + "start_min_angle", start_min_angle_rad_);
  node->get_parameter(p + "start_heading_lookahead", lookahead_m_);
  node->get_parameter(p + "align_timeout", align_timeout_sec_);
  node->get_parameter(p + "follow_timeout", follow_timeout_sec_);
  node->get_parameter(p + "fallback_xy_tolerance", fallback_xy_tol_);
  node->get_parameter(p + "fallback_yaw_tolerance", fallback_yaw_tol_);

  // ---- 参数合法性：非法即拒绝启动，不静默回落 ----
  if (inner_plugin_.empty()) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: inner_controller_plugin 为空 —— 没有内层控制器就没有跟踪能力");
  }
  if (!(align_kp_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: align_kp 必须 > 0");
  }
  if (!(align_max_vel_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: align_max_vel 必须 > 0");
  }
  if (align_floor_vel_ < 0.0) {
    throw nav2_core::PlannerException("ThreePhaseController: align_floor_vel 不能为负");
  }
  if (align_floor_vel_ > align_max_vel_) {
    // 这个组合本身矛盾。alignAngularVelocity 里 max 会赢，但让它静默生效
    // 等于让配置说谎，所以直接拒绝。
    throw nav2_core::PlannerException(
      "ThreePhaseController: align_floor_vel 不能大于 align_max_vel（配置自相矛盾）");
  }
  if (!(align_tol_rad_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: align_tolerance 必须 > 0");
  }
  if (start_min_angle_rad_ < align_tol_rad_) {
    // 触发下限比达标阈值还小 -> 进了 ALIGN_START 立刻就算达标，等于该段永不生效。
    throw nav2_core::PlannerException(
      "ThreePhaseController: start_min_angle 必须 >= align_tolerance，"
      "否则起步对齐段进去就立刻满足、形同虚设");
  }
  if (!(lookahead_m_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: start_heading_lookahead 必须 > 0");
  }
  if (!(align_timeout_sec_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: align_timeout 必须 > 0");
  }
  if (follow_timeout_sec_ < 0.0) {
    throw nav2_core::PlannerException("ThreePhaseController: follow_timeout 不能为负");
  }
  if (!(fallback_xy_tol_ > 0.0) || !(fallback_yaw_tol_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: fallback 容差必须 > 0");
  }
}

void ThreePhaseController::loadInnerController(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  // loader 必须与插件同生命周期：它先析构会把已加载的 .so 卸掉，
  // 之后任何一次 computeVelocityCommands 都是段错误。所以存成成员。
  inner_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core", "nav2_core::Controller");
  try {
    inner_ = inner_loader_->createSharedInstance(inner_plugin_);
  } catch (const pluginlib::PluginlibException & e) {
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: 内层控制器加载失败 '") + inner_plugin_ + "': " + e.what());
  }
  // 内层用独立的参数命名空间，避免它的参数名与本插件的撞车。
  inner_->configure(parent, name_ + ".inner", tf, costmap_ros);
}

void ThreePhaseController::cleanup()
{
  if (inner_) {
    inner_->cleanup();
    inner_.reset();
  }
  inner_loader_.reset();
}

void ThreePhaseController::activate()
{
  if (inner_) {
    inner_->activate();
  }
}

void ThreePhaseController::deactivate()
{
  if (inner_) {
    inner_->deactivate();
  }
}

void ThreePhaseController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // 限速要透传给内层，否则 Nav2 的 /speed_limit 机制（本项目用它做机械臂
  // 展开限速）对跟踪段完全失效。
  if (inner_) {
    inner_->setSpeedLimit(speed_limit, percentage);
  }
}

void ThreePhaseController::enterPhase(Phase p, const rclcpp::Time & now, const char * why)
{
  if (p == phase_) {
    return;
  }
  RCLCPP_INFO(
    logger_, "[%s] 相位 %s -> %s : %s", name_.c_str(), toString(phase_), toString(p), why);
  phase_ = p;
  phase_started_ = now;
}

void ThreePhaseController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
  if (inner_) {
    inner_->setPlan(path);
  }

  const rclcpp::Time now = clock_->now();
  start_heading_valid_ = false;

  if (path.poses.size() < 2U) {
    // 单点路径（通常是"已在目标处"）：没有起始方向可言，直接进跟踪段，
    // 由 GoalChecker 判定是否已到。不在这里报错 —— 这是合法情形。
    enterPhase(Phase::kFollow, now, "路径顶点少于 2 个，无起始方向");
    return;
  }

  std::vector<PlanarPoint> pts;
  pts.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    pts.push_back(PlanarPoint{ps.pose.position.x, ps.pose.position.y});
  }
  double h = 0.0;
  if (pathStartHeading(pts, lookahead_m_, h)) {
    start_heading_ = h;
    start_heading_valid_ = true;
  } else {
    // 路径退化（所有顶点重合）。不猜一个方向，直接跳过起步对齐。
    RCLCPP_WARN(
      logger_, "[%s] 路径退化，无法估计起始方向，跳过起步对齐段", name_.c_str());
  }

  if (align_start_enabled_ && start_heading_valid_) {
    enterPhase(Phase::kAlignStart, now, "收到新路径");
  } else {
    enterPhase(
      Phase::kFollow, now,
      align_start_enabled_ ? "起始方向不可用" : "起步对齐已关闭");
  }
}

void ThreePhaseController::resolveTolerances(
  nav2_core::GoalChecker * goal_checker, double & xy_tol, double & yaw_tol)
{
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (goal_checker != nullptr && goal_checker->getTolerances(pose_tol, vel_tol)) {
    // x/y 容差在 nav2 里是同一个 xy_goal_tolerance 填进 position.x/y 的。
    xy_tol = std::max(pose_tol.position.x, pose_tol.position.y);
    yaw_tol = std::fabs(yawOf(pose_tol));
    if (xy_tol > 0.0 && yaw_tol > 0.0) {
      return;
    }
  }
  // 取不到就用兜底值，但**只告警一次**（每拍 20Hz 刷屏会把真问题淹掉）。
  if (!warned_tolerance_fallback_) {
    warned_tolerance_fallback_ = true;
    RCLCPP_WARN(
      logger_,
      "[%s] 无法从 GoalChecker 取容差，回落到配置值 xy=%.3f yaw=%.3f。"
      "注意：这意味着本控制器与 GoalChecker 的判据可能不一致",
      name_.c_str(), fallback_xy_tol_, fallback_yaw_tol_);
  }
  xy_tol = fallback_xy_tol_;
  yaw_tol = fallback_yaw_tol_;
}

void ThreePhaseController::checkPhaseTimeout(const rclcpp::Time & now)
{
  const double elapsed = (now - phase_started_).seconds();
  if (phase_ == Phase::kAlignStart || phase_ == Phase::kAlignGoal) {
    if (elapsed > align_timeout_sec_) {
      // 显式失败，绝不静默停在这一段。原地转不动的可能原因：
      // 力矩不足、被 /speed_limit 限到过小、或 wz 被下游丢弃。
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: ") + toString(phase_) + " 段超时 " +
        std::to_string(elapsed) + "s > " + std::to_string(align_timeout_sec_) + "s，原地对齐未完成");
    }
  } else if (phase_ == Phase::kFollow && follow_timeout_sec_ > 0.0) {
    if (elapsed > follow_timeout_sec_) {
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: FOLLOW 段超时 ") + std::to_string(elapsed) + "s");
    }
  }
}

geometry_msgs::msg::TwistStamped ThreePhaseController::rotateOnly(
  double error_rad, const std_msgs::msg::Header & header) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.angular.z =
    alignAngularVelocity(error_rad, align_kp_, align_max_vel_, align_floor_vel_, align_tol_rad_);
  return cmd;
}

geometry_msgs::msg::TwistStamped ThreePhaseController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (!inner_) {
    throw nav2_core::PlannerException("ThreePhaseController: 内层控制器不可用");
  }
  const rclcpp::Time now = clock_->now();

  double xy_tol = fallback_xy_tol_;
  double yaw_tol = fallback_yaw_tol_;
  resolveTolerances(goal_checker, xy_tol, yaw_tol);

  const double robot_yaw = yawOf(pose.pose);

  // 起步朝向误差：路径退化时按 0 处理（等价于"不需要转"），
  // 而不是让 NaN 传下去。
  const double start_err =
    start_heading_valid_ ? shortestAngularDiff(robot_yaw, start_heading_) : 0.0;

  // 终点姿态与到终点距离。路径为空时按"已到达"处理，交给 GoalChecker。
  double goal_err = 0.0;
  double dist_to_goal = 0.0;
  if (!plan_.poses.empty()) {
    const auto & last = plan_.poses.back().pose;
    goal_err = shortestAngularDiff(robot_yaw, yawOf(last));
    dist_to_goal = std::hypot(
      last.position.x - pose.pose.position.x, last.position.y - pose.pose.position.y);
  }

  // 先推进相位，再按相位产生指令 —— 顺序反了会多出一拍的错误指令。
  const Phase next = advancePhase(
    phase_, start_err, goal_err, dist_to_goal, xy_tol, align_tol_rad_,
    start_min_angle_rad_, align_goal_enabled_);
  if (next != phase_) {
    const char * why =
      (next == Phase::kFollow) ? "起始方向已对齐" :
      (next == Phase::kAlignGoal) ? "位置已到，开始对齐目标姿态" : "本段完成";
    enterPhase(next, now, why);
  }

  checkPhaseTimeout(now);

  switch (phase_) {
    case Phase::kAlignStart:
      return rotateOnly(start_err, pose.header);

    case Phase::kAlignGoal:
      return rotateOnly(goal_err, pose.header);

    case Phase::kFollow: {
      geometry_msgs::msg::TwistStamped cmd =
        inner_->computeVelocityCommands(pose, velocity, goal_checker);
      if (zero_vy_in_follow_) {
        // 前向为主：把横移分量掐掉。注意这会让内层控制器的输出与它自己的
        // 预测不一致（它以为能横移），所以内层应当配成非全向模型；
        // 这里掐掉只是最后一道保险。
        cmd.twist.linear.y = 0.0;
      }
      return cmd;
    }

    case Phase::kDone:
    default: {
      // 本控制器认为没有更多主动动作。出零速而不是继续调用内层 ——
      // 是否真的到达由 GoalChecker 判，这里不替它下结论。
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
  }
}

}  // namespace astribot_s1_path_tracking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  astribot_s1_path_tracking::ThreePhaseController, nav2_core::Controller)
