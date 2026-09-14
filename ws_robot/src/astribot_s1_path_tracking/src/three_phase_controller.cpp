// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/three_phase_controller.hpp"


#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

  RCLCPP_INFO(
    logger_,
    "[%s] 窄通道贴边通行层已删除(2026-09-07)：足迹代价全覆盖的通道完全交给内层"
    "控制器；跟踪段横移=%s，本层不产生 vy（有 vy 即来自内层）",
    name_.c_str(), zero_vy_in_follow_ ? "禁止" : "允许");

  if (approach_enabled_) {
    RCLCPP_INFO(
      logger_,
      "[%s] 接近段限速已启用: 收敛区=%.2fm 速度下限=%.2fm/s (‖v‖ 按 d/D 线性收敛，"
      "不限 wz)。治的是指令->实际速度的未建模死时间 τ（实测 p50=0.55s、增益 0.977），"
      "终段过冲 ≈ v_接近·τ，而 MPPI 模型里没有 τ 这一项",
      name_.c_str(), approach_dist_m_, approach_v_min_);
  } else {
    RCLCPP_WARN(
      logger_,
      "[%s] 接近段限速已**禁用**(approach_enabled=false): 终段速度完全由内层决定，"
      "过冲会回到 0.146m 量级（run11 实测 p50）—— 这是显式回退档，不是默认值",
      name_.c_str());
  }
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
  declare_parameter_if_not_declared(node, p + "new_goal_epsilon", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, p + "new_attempt_gap", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, p + "approach_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "approach_dist", rclcpp::ParameterValue(1.50));
  declare_parameter_if_not_declared(node, p + "approach_v_min", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, p + "align_inertia_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "align_coast_lag", rclcpp::ParameterValue(0.03));
  declare_parameter_if_not_declared(node, p + "align_coast_decel", rclcpp::ParameterValue(7.0));
  declare_parameter_if_not_declared(node, p + "align_settled_wz", rclcpp::ParameterValue(0.02));

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
  node->get_parameter(p + "new_goal_epsilon", new_goal_epsilon_m_);
  node->get_parameter(p + "new_attempt_gap", new_attempt_gap_sec_);
  node->get_parameter(p + "approach_enabled", approach_enabled_);
  node->get_parameter(p + "approach_dist", approach_dist_m_);
  node->get_parameter(p + "approach_v_min", approach_v_min_);
  node->get_parameter(p + "align_inertia_enabled", align_inertia_enabled_);
  node->get_parameter(p + "align_coast_lag", align_coast_lag_);
  node->get_parameter(p + "align_coast_decel", align_coast_decel_);
  node->get_parameter(p + "align_settled_wz", align_settled_wz_);


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
    throw nav2_core::PlannerException(
      "ThreePhaseController: align_floor_vel 不能大于 align_max_vel（配置自相矛盾）");
  }
  if (!(align_tol_rad_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: align_tolerance 必须 > 0");
  }
  if (start_min_angle_rad_ < align_tol_rad_) {
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
  if (!(new_goal_epsilon_m_ > 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: new_goal_epsilon 必须 > 0");
  }
  if (!(new_attempt_gap_sec_ > 0.0) || !(new_attempt_gap_sec_ < align_timeout_sec_)) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: new_attempt_gap 必须 in (0, align_timeout)");
  }

  if (approach_enabled_) {
    if (!(approach_dist_m_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: approach_dist 必须 > 0");
    }
    if (!(approach_v_min_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: approach_v_min 必须 > 0");
    }
    if (!(approach_v_min_ < approach_dist_m_)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: approach_v_min 必须 < approach_dist（疑似两者写颠倒）");
    }
    if (!(approach_dist_m_ > fallback_xy_tol_)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: approach_dist 必须 > fallback_xy_tolerance，"
        "否则限速段整个落在到位容差内、形同虚设");
    }
  }

  if (!align_inertia_enabled_) {
    RCLCPP_WARN(
      logger_,
      "%s: align_inertia_enabled=false —— 惯性补偿已关闭，行为逐字回到改动前。"
      "此时朝向精度由「align_tolerance + 未补偿余转（实测 0.006~0.033rad）」决定，"
      "**必须**确认 precise_goal_checker.yaw_goal_tolerance >= %.3f，"
      "否则对齐段转到极限也不达标 -> align_timeout %.1fs -> abort -> 3 连败 -> PAUSED。",
      name_.c_str(), align_tol_rad_ + 0.033, align_timeout_sec_);
  } else {
    if (align_coast_lag_ < 0.0) {
      throw nav2_core::PlannerException("ThreePhaseController: align_coast_lag 不能为负");
    }
    if (!(align_coast_decel_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: align_coast_decel 必须 > 0");
    }
    if (!(align_settled_wz_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: align_settled_wz 必须 > 0");
    }
    if (!(align_settled_wz_ < align_floor_vel_)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: align_settled_wz 必须 < align_floor_vel，"
        "否则发着地板速度就算「已静止」，滑行闩锁提前释放 -> 边界自激");
    }
    const double coast_at_max =
      coastAngle(align_max_vel_, align_coast_lag_, align_coast_decel_);
    if (!(coast_at_max < 0.5)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: align_coast_lag/align_coast_decel 组合下，"
        "align_max_vel 处预测余转 >= 0.5rad（28 度）—— 补偿项会盖过误差本身，"
        "对齐段将一拍都不转直到超时。疑似 align_coast_decel 写得过小");
    }
    const double kFloorTrackHi = 0.92;
    const double coast_at_floor =
      coastAngle(align_floor_vel_ * kFloorTrackHi, align_coast_lag_, align_coast_decel_);
    if (!(align_tol_rad_ > coast_at_floor)) {
      RCLCPP_WARN(
        logger_,
        "%s: align_tolerance=%.4f 已经不大于 floor 速度下的预测余转 %.4f —— "
        "这是本层的**精度硬地板**（发得出的最小指令走完死时间就有这么多角度）。"
        "容差压到地板以下 ⇒ 每次都靠「预测停点」提前松手才可能达标，模型误差直接"
        "变成失败率。要更高精度只能减小 align_floor_vel（代价是可能驱动不了底盘）。",
        name_.c_str(), align_tol_rad_, coast_at_floor);
    }
    if (!(align_settled_wz_ < kFloorTrackHi * align_floor_vel_)) {
      RCLCPP_WARN(
        logger_,
        "%s: align_settled_wz=%.4f 接近 floor 速度的实测角速度 %.4f "
        "(=align_floor_vel %.3f × 跟踪比 %.2f) —— 闩锁可能在底盘仍以地板速度旋转时"
        "就释放，边界上会来回抖。建议 <= 其一半。",
        name_.c_str(), align_settled_wz_, kFloorTrackHi * align_floor_vel_,
        align_floor_vel_, kFloorTrackHi);
    }
    RCLCPP_INFO(
      logger_,
      "%s 惯性补偿已启用: lag=%.3fs decel=%.2frad/s^2 settled_wz=%.3frad/s | "
      "预测余转: wz=0.046(地板×0.92)->%.4f  0.148->%.4f  0.30->%.4f  %.2f(max)->%.4f rad | "
      "align_tolerance=%.3f ⇒ 相对地板余转 %.1f 倍余量 | "
      "**lag/decel 是按实测区间 0.006~0.033rad 反推的推断值，不是实测值**："
      "当初没记对应的 wz。每次对齐段滑停都会打一条「预测 vs 实测余转」，"
      "那才是标定这两个数的数据来源。",
      name_.c_str(), align_coast_lag_, align_coast_decel_, align_settled_wz_,
      coast_at_floor,
      coastAngle(0.148, align_coast_lag_, align_coast_decel_),
      coastAngle(0.30, align_coast_lag_, align_coast_decel_),
      align_max_vel_, coast_at_max,
      align_tol_rad_, (coast_at_floor > 0.0) ? align_tol_rad_ / coast_at_floor : 0.0);
  }

}

void ThreePhaseController::loadInnerController(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  inner_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core", "nav2_core::Controller");
  try {
    inner_ = inner_loader_->createSharedInstance(inner_plugin_);
  } catch (const pluginlib::PluginlibException & e) {
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: 内层控制器加载失败 '") + inner_plugin_ + "': " + e.what());
  }
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
  if (inner_) {
    inner_->setSpeedLimit(speed_limit, percentage);
  }
}

void ThreePhaseController::enterPhase(
  Phase p, const rclcpp::Time & now, const char * why, bool restart_timer)
{
  if (p == phase_) {
    if (shouldRestartPhaseTimer(phase_, p, restart_timer)) {
      RCLCPP_INFO(
        logger_, "[%s] 相位仍为 %s，但按新目标重置计时器 : %s",
        name_.c_str(), toString(phase_), why);
      phase_started_ = now;
      if (p == Phase::kAlignStart) {start_alignment_engaged_ = false;}
    }
    return;
  }
  RCLCPP_INFO(
    logger_, "[%s] 相位 %s -> %s : %s", name_.c_str(), toString(phase_), toString(p), why);
  phase_ = p;
  phase_started_ = now;
  align_coasting_ = false;
  if (p == Phase::kAlignStart) {start_alignment_engaged_ = false;}
}

void ThreePhaseController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
  if (inner_) {
    inner_->setPlan(path);
  }

  const rclcpp::Time now = clock_->now();

  if (path.poses.size() < 2U) {
    start_heading_valid_ = false;
    has_last_goal_ = false;
    enterPhase(Phase::kFollow, now, "路径顶点少于 2 个，无起始方向", true);
    return;
  }

  const auto & end = path.poses.back().pose.position;
  const PlanarPoint cur_end{end.x, end.y};

  const bool same_goal =
    has_last_goal_ && isSameGoal(last_goal_end_, cur_end, new_goal_epsilon_m_);
  last_goal_end_ = cur_end;
  has_last_goal_ = true;

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
    start_heading_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[%s] 路径退化，无法估计起始方向，跳过起步对齐段", name_.c_str());
  }

  if (same_goal) {
    const double idle_gap = has_tick_ ? (now - last_tick_time_).seconds() : 0.0;
    if (isFreshFollowAttempt(has_tick_, idle_gap, new_attempt_gap_sec_)) {
      RCLCPP_INFO(
        logger_,
        "[%s] 同一目标的新一次下发（距上次 tick %.2fs > %.2fs，上个 action 已结束），"
        "重置相位 %s 的计时器",
        name_.c_str(), idle_gap, new_attempt_gap_sec_, toString(phase_));
      phase_started_ = now;
      return;
    }
    RCLCPP_DEBUG(
      logger_, "[%s] 同一目标的重规划，保持相位 %s", name_.c_str(), toString(phase_));
    return;
  }

  arrival_logged_ = false;

  if (align_start_enabled_ && start_heading_valid_) {
    enterPhase(Phase::kAlignStart, now, "收到新目标", true);
  } else {
    enterPhase(
      Phase::kFollow, now,
      align_start_enabled_ ? "起始方向不可用" : "起步对齐已关闭", true);
  }
}

void ThreePhaseController::resolveTolerances(
  nav2_core::GoalChecker * goal_checker, double & xy_tol, double & yaw_tol)
{
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (goal_checker != nullptr && goal_checker->getTolerances(pose_tol, vel_tol)) {
    xy_tol = std::max(pose_tol.position.x, pose_tol.position.y);
    yaw_tol = std::fabs(yawOf(pose_tol));
    if (xy_tol > 0.0 && yaw_tol > 0.0) {
      if (!hasTerminalRefinement()) {
        warnIfGoalAlignInert(xy_tol, yaw_tol);
        warnIfYawToleranceUnreachable(yaw_tol);
        noteDeliveredYawAccuracy(yaw_tol);
      }
      return;
    }
  }
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

void ThreePhaseController::warnIfGoalAlignInert(double xy_tol, double yaw_tol)
{
  if (!align_goal_enabled_ || warned_goal_align_inert_) {
    return;
  }
  if (yaw_tol < M_PI) {
    return;                       // 真的在卡朝向，ALIGN_GOAL 有机会执行
  }
  warned_goal_align_inert_ = true;
  RCLCPP_WARN(
    logger_,
    "[%s] align_goal_enabled=true 但 **ALIGN_GOAL 段实际拿不到执行机会**："
    "GoalChecker 的 yaw_goal_tolerance=%.3frad >= pi 等于不约束朝向，"
    "于是它判到位的条件与本层进 ALIGN_GOAL 的条件是同一个数(dist<=%.3fm)，"
    "位置一进容差 nav2 当拍就结束 action。这是既有行为，不是缺陷回归；"
    "要真正启用终点对齐需要另加一个卡朝向的 goal checker 并让 BT/协调器"
    "显式传 goal_checker_id（四处联动，漏一处则每个目标都 abort）",
    name_.c_str(), yaw_tol, xy_tol);
}

void ThreePhaseController::warnIfYawToleranceUnreachable(double yaw_tol)
{
  if (!align_goal_enabled_ || warned_yaw_unreachable_) {
    return;
  }
  const double bound = bestAchievableYawLanding();
  if (yaw_tol >= bound) {
    return;
  }
  warned_yaw_unreachable_ = true;
  RCLCPP_WARN(
    logger_,
    "[%s] **GoalChecker 的 yaw_goal_tolerance=%.4f 紧到本层到不了**："
    "本层最好落点 = align_tolerance %.4f + 松手余转 %.4f = %.4f。"
    "后果不是精度差一点，而是 ALIGN_GOAL 转到极限仍不达标 -> align_timeout %.1fs "
    "-> throw -> FollowPath abort -> 上游 3 连败 -> PAUSED。"
    "要么放宽这个容差到 >= %.4f，要么减小 align_floor_vel/align_tolerance。",
    name_.c_str(), yaw_tol, align_tol_rad_, bound - align_tol_rad_, bound,
    align_timeout_sec_, bound);
}

double ThreePhaseController::bestAchievableYawLanding() const
{
  if (!align_inertia_enabled_) {
    return align_tol_rad_ + 0.033;
  }
  const double kFloorTrackHi = 0.92;
  const double modeled =
    coastAngle(align_floor_vel_ * kFloorTrackHi, align_coast_lag_, align_coast_decel_);
  const double kModelUncertainty = 0.01;
  return align_tol_rad_ + std::max(modeled, kModelUncertainty);
}

void ThreePhaseController::noteDeliveredYawAccuracy(double yaw_tol)
{
  if (!align_goal_enabled_ || noted_delivered_accuracy_) {
    return;
  }
  noted_delivered_accuracy_ = true;

  const double bound = bestAchievableYawLanding();
  RCLCPP_INFO(
    logger_,
    "[%s] 终点航向交付精度 = GoalChecker 的 yaw_goal_tolerance **%.4frad(%.2f°)**，"
    "不是 align_tolerance %.4f。机制：controller_server 先算速度再判到位，"
    "checker 一接受就结束 action、本插件不再拿到 tick，所以本层自己的退出条件"
    "(|误差| <= align_tolerance)**只有在 checker 更紧时才可能生效**。"
    "本层最好落点 %.4f = align_tolerance %.4f + 松手余转 %.4f。",
    name_.c_str(), yaw_tol, yaw_tol * 180.0 / M_PI, align_tol_rad_,
    bound, align_tol_rad_, bound - align_tol_rad_);

  if (yaw_tol >= 2.0 * bound) {
    RCLCPP_WARN(
      logger_,
      "[%s] yaw_goal_tolerance %.4f >= 2x 本层最好落点 %.4f ⇒ **align_tolerance %.4f "
      "名存实亡**：checker 会在本层还差 %.4frad 时就结束 action，收紧 align_tolerance "
      "是空操作（滑停松手那几拍根本不会执行，"
      "\"滑停完成\" 一行都不会打）。要提高终点航向精度就得降 yaw_goal_tolerance，"
      "并让 align_tolerance 跟着满足 align_tolerance + 余转 <= yaw_goal_tolerance。",
      name_.c_str(), yaw_tol, bound, align_tol_rad_, yaw_tol - bound);
  }
}

void ThreePhaseController::checkPhaseTimeout(const rclcpp::Time & now)
{
  const double elapsed = (now - phase_started_).seconds();
  if (phase_ == Phase::kAlignStart || phase_ == Phase::kAlignGoal) {
    if (elapsed > align_timeout_sec_) {
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
  double error_rad, double wz_now, const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;

  if (!align_inertia_enabled_) {
    cmd.twist.angular.z =
      alignAngularVelocity(error_rad, align_kp_, align_max_vel_, align_floor_vel_, align_tol_rad_);
    return cmd;
  }

  if (align_coasting_) {
    if (std::fabs(wz_now) <= align_settled_wz_) {
      align_coasting_ = false;
      const double actual = coast_err_at_stop_ - error_rad;
      RCLCPP_INFO(
        logger_,
        "[%s] %s 滑停完成｜松手时 wz=%+.4frad/s err=%+.5f ⇒ 预测余转 %.5f，"
        "实测 %+.5f（残差 %+.5f）｜落点 |err|=%.5f vs align_tolerance %.3f %s"
        "｜标定用：lag=%.3f decel=%.2f",
        name_.c_str(), toString(phase_), coast_wz_at_stop_, coast_err_at_stop_,
        coast_predicted_, actual, actual - coast_predicted_,
        std::fabs(error_rad), align_tol_rad_,
        (std::fabs(error_rad) <= align_tol_rad_) ? "(达标)" : "(**未达标，将再来一轮**)",
        align_coast_lag_, align_coast_decel_);
    } else {
      cmd.twist.angular.z = 0.0;
      return cmd;
    }
  }

  const double out = alignAngularVelocityWithInertia(
    error_rad, wz_now, align_kp_, align_max_vel_, align_floor_vel_, align_tol_rad_,
    align_coast_lag_, align_coast_decel_);

  if (out == 0.0 && std::fabs(wz_now) > align_settled_wz_) {
    align_coasting_ = true;
    coast_err_at_stop_ = error_rad;
    coast_wz_at_stop_ = wz_now;
    coast_predicted_ = coastAngle(wz_now, align_coast_lag_, align_coast_decel_) *
      ((wz_now > 0.0) ? 1.0 : -1.0);
  }

  cmd.twist.angular.z = out;
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

  last_tick_time_ = now;
  has_tick_ = true;

  double xy_tol = fallback_xy_tol_;
  double yaw_tol = fallback_yaw_tol_;
  resolveTolerances(goal_checker, xy_tol, yaw_tol);

  auto phase_pose = pose;
  if (!plan_.poses.empty() && pose.header.frame_id != plan_.header.frame_id) {
    try {
      phase_pose = tf_->transform(pose, plan_.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::PlannerException(
              std::string("ThreePhaseController: cannot transform pose to plan frame: ") + ex.what());
    }
  }
  const double robot_yaw = yawOf(phase_pose.pose);

  const double start_err =
    start_heading_valid_ ? shortestAngularDiff(robot_yaw, start_heading_) : 0.0;

  double goal_err = 0.0;
  double dist_to_goal = 0.0;
  if (!plan_.poses.empty()) {
    const auto & last = plan_.poses.back().pose;
    goal_err = shortestAngularDiff(robot_yaw, yawOf(last));
    dist_to_goal = std::hypot(
      last.position.x - phase_pose.pose.position.x, last.position.y - phase_pose.pose.position.y);
  }

  if (!arrival_logged_ && !plan_.poses.empty() &&
    dist_to_goal <= xy_tol && std::fabs(goal_err) <= yaw_tol)
  {
    arrival_logged_ = true;
    const auto & goal = plan_.poses.back().pose;
    const double goal_yaw = yawOf(goal);
    const double yaw_err_abs = std::fabs(goal_err);

    ++arrival_count_;
    arrival_xy_sum_ += dist_to_goal;
    arrival_yaw_sum_ += yaw_err_abs;
    arrival_xy_max_ = std::max(arrival_xy_max_, dist_to_goal);
    arrival_yaw_max_ = std::max(arrival_yaw_max_, yaw_err_abs);

    RCLCPP_INFO(
      logger_,
      "[%s] ===== 跟踪完成 #%d（frame=%s）=====\n"
      "  目标位姿  x=%.3f y=%.3f yaw=%.3frad(%.1f°)\n"
      "  当前位姿  x=%.3f y=%.3f yaw=%.3frad(%.1f°)\n"
      "  位置误差  %.3fm  (xy_tol %.3f)\n"
      "  朝向误差  %.3frad = %.1f°  (yaw_tol %.3frad / 本层 align_tol %.3frad)\n"
      "  累计 n=%d  位置 均值 %.3fm 最大 %.3fm  朝向 均值 %.1f° 最大 %.1f°",
      name_.c_str(), arrival_count_, phase_pose.header.frame_id.c_str(),
      goal.position.x, goal.position.y, goal_yaw, goal_yaw * 180.0 / M_PI,
      phase_pose.pose.position.x, phase_pose.pose.position.y, robot_yaw,
      robot_yaw * 180.0 / M_PI,
      dist_to_goal, xy_tol,
      yaw_err_abs, yaw_err_abs * 180.0 / M_PI, yaw_tol, align_tol_rad_,
      arrival_count_,
      arrival_xy_sum_ / arrival_count_, arrival_xy_max_,
      (arrival_yaw_sum_ / arrival_count_) * 180.0 / M_PI,
      arrival_yaw_max_ * 180.0 / M_PI);
  }

  const double wz_now = velocity.angular.z;
  if (phase_ == Phase::kAlignStart && needsStartAlign(start_err, start_min_angle_rad_)) {
    start_alignment_engaged_ = true;
  }
  Phase next = advancePhase(
    phase_, start_err, goal_err, dist_to_goal, xy_tol, align_tol_rad_,
    start_min_angle_rad_, align_goal_enabled_,
    wz_now,
    align_inertia_enabled_ ? align_settled_wz_ : std::numeric_limits<double>::infinity());
  // The trigger threshold only decides whether to start rotating, not when to stop.
  if (phase_ == Phase::kAlignStart && start_alignment_engaged_) {
    next = headingSettled(
      start_err, wz_now, align_tol_rad_,
      align_inertia_enabled_ ? align_settled_wz_ : std::numeric_limits<double>::infinity()) ?
      Phase::kFollow : Phase::kAlignStart;
  }
  if (next != phase_) {
    const double held = (now - phase_started_).seconds();
    char detail[256];
    const char * why =
      (phase_ == Phase::kDone && next == Phase::kFollow) ?
      "已判到位又滑出位置容差，撤回继续跟踪" :
      (phase_ == Phase::kDone && next == Phase::kAlignGoal) ?
      "已判到位但 yaw 在容差外（换了朝向 / 被推歪），撤回重新对齐" :
      (next == Phase::kFollow) ? "起始方向已对齐" :
      (next == Phase::kAlignGoal) ? "位置已到，开始对齐目标姿态" : "本段完成";
    std::snprintf(
      detail, sizeof(detail),
      "%s | 位置差 %.3fm(xy_tol %.3f) 起步朝向差 %.3frad 终点朝向差 %.3frad=%.1f°"
      "(yaw_tol %.3f / align_tol %.3f) 上一段历时 %.2fs(align_timeout %.1f)",
      why, dist_to_goal, xy_tol, std::fabs(start_err), std::fabs(goal_err),
      std::fabs(goal_err) * 180.0 / M_PI, yaw_tol, align_tol_rad_, held,
      align_timeout_sec_);
    enterPhase(next, now, detail);
  }

  checkPhaseTimeout(now);

  switch (phase_) {
    case Phase::kAlignStart:
      return rotateOnly(start_err, wz_now, pose.header);

    case Phase::kAlignGoal:
      return rotateOnly(goal_err, wz_now, pose.header);

    case Phase::kFollow: {
      geometry_msgs::msg::TwistStamped cmd =
        inner_->computeVelocityCommands(pose, velocity, goal_checker);
      if (zero_vy_in_follow_) {
        cmd.twist.linear.y = 0.0;
      }
      applyApproachCap(cmd, dist_to_goal);
      return cmd;
    }

    case Phase::kDone:
    default: {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 5000,
        "[%s] DONE 保持零速：位置差 %.3fm 朝向差 %.3frad(%.1f°) 已收敛到 "
        "align_tol %.3f，等 GoalChecker 收尾",
        name_.c_str(), dist_to_goal, std::fabs(goal_err),
        std::fabs(goal_err) * 180.0 / M_PI, align_tol_rad_);
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
  }
}

void ThreePhaseController::applyApproachCap(
  geometry_msgs::msg::TwistStamped & cmd, double dist_to_goal_m)
{
  if (!approach_enabled_) {
    return;                       // 一键回退：接近段不限速（限速上线前的行为）
  }

  const double vx = cmd.twist.linear.x;
  const double vy = cmd.twist.linear.y;
  const double speed = std::hypot(vx, vy);
  if (!(speed > 1e-6)) {
    return;                       // 内层已经在发零速，没有可限的东西
  }

  const double cap = approachSpeedCap(dist_to_goal_m, approach_dist_m_, approach_v_min_, speed);
  if (!(cap < speed)) {
    return;                       // 收敛区之外，或内层本来就比上限慢
  }

  const double scale = cap / speed;
  cmd.twist.linear.x = vx * scale;
  cmd.twist.linear.y = vy * scale;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "[%s] 接近段限速：d=%.3fm (D=%.2fm) ‖v‖ %.3f -> %.3f m/s",
    name_.c_str(), dist_to_goal_m, approach_dist_m_, speed, cap);
}

}  // namespace astribot_s1_path_tracking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  astribot_s1_path_tracking::ThreePhaseController, nav2_core::Controller)
