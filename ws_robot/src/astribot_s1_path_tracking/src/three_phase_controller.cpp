// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/three_phase_controller.hpp"
#include "astribot_s1_path_tracking/corner_sweep.hpp"


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
  phase_started_ = budget_clock_.now();

  monitor_sub_=node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/slam/pose",rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg){receiveSlamPose(*msg);});
  declareAndLoadParams();
  alignment_collision_.configure(node,costmap_ros);
  std::string progress_name="progress_checker", progress_plugin;
  node->get_parameter_or("progress_checker_plugin",progress_name,progress_name);
  node->get_parameter_or(progress_name+".plugin",progress_plugin,std::string{});
  if(progress_plugin=="astribot_s1_path_tracking::PolicyProgressChecker") {
    execution_=ControllerExecution::forNode(node.get());
  } else if(corner_turn_enabled_) {
    throw std::runtime_error("corner tracking requires PolicyProgressChecker execution lifecycle");
  }
  loadInnerController(parent_, tf_, costmap_ros_);
  if (corner_turn_enabled_) {
    double vx=1., vx_min=-1., vy=1., wz=2.;
    node->get_parameter_or(name_+".inner.vx_max",vx,vx);
    node->get_parameter_or(name_+".inner.vx_min",vx_min,vx_min);
    node->get_parameter_or(name_+".inner.vy_max",vy,vy);
    node->get_parameter_or(name_+".inner.wz_max",wz,wz);
    corner_feedback_linear_bound_mps_=std::max(corner_approach_speed_mps_,
      std::hypot(std::max(std::abs(vx),std::abs(vx_min)),vy));
    corner_feedback_angular_bound_radps_=std::max(align_max_vel_,std::abs(wz));
  }

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

  if (corner_turn_enabled_) {
    RCLCPP_INFO(
      logger_,
      "[%s] 标准角点跟踪已启用: turn=[%.1f°, %.1f°] segment>=%.2fm "
      "prepare=%.2fm capture=%.3fm terminal_guard=%.2fm v=[%.3f, %.3f]m/s",
      name_.c_str(), corner_min_angle_rad_ * 180.0 / M_PI,
      corner_max_angle_rad_ * 180.0 / M_PI, corner_min_segment_m_,
      corner_approach_distance_m_, corner_capture_radius_m_, corner_terminal_guard_m_,
      corner_approach_min_speed_mps_, corner_approach_speed_mps_);
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
  declare_parameter_if_not_declared(node, p + "corner_turn_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, p + "corner_min_angle", rclcpp::ParameterValue(0.61));
  declare_parameter_if_not_declared(node, p + "corner_max_angle", rclcpp::ParameterValue(2.75));
  declare_parameter_if_not_declared(node, p + "corner_min_segment", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, p + "corner_approach_distance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, p + "corner_capture_radius", rclcpp::ParameterValue(0.04));
  declare_parameter_if_not_declared(
    node, p + "corner_lateral_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, p + "corner_approach_speed", rclcpp::ParameterValue(0.08));
  declare_parameter_if_not_declared(
    node, p + "corner_approach_min_speed", rclcpp::ParameterValue(0.02));
  declare_parameter_if_not_declared(node, p + "corner_approach_kp", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, p + "corner_terminal_guard", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(node, p + "corner_timeout", rclcpp::ParameterValue(15.0));
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
  node->get_parameter(p + "corner_turn_enabled", corner_turn_enabled_);
  node->get_parameter(p + "corner_min_angle", corner_min_angle_rad_);
  node->get_parameter(p + "corner_max_angle", corner_max_angle_rad_);
  node->get_parameter(p + "corner_min_segment", corner_min_segment_m_);
  node->get_parameter(p + "corner_approach_distance", corner_approach_distance_m_);
  node->get_parameter(p + "corner_capture_radius", corner_capture_radius_m_);
  node->get_parameter(p + "corner_lateral_tolerance", corner_lateral_tolerance_m_);
  node->get_parameter(p + "corner_approach_speed", corner_approach_speed_mps_);
  node->get_parameter(p + "corner_approach_min_speed", corner_approach_min_speed_mps_);
  node->get_parameter(p + "corner_approach_kp", corner_approach_kp_);
  node->get_parameter(p + "corner_terminal_guard", corner_terminal_guard_m_);
  node->get_parameter(p + "corner_timeout", corner_timeout_sec_);
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

  if (corner_turn_enabled_) {
    if (!(corner_min_angle_rad_ > 0.0) || !(corner_min_angle_rad_ < corner_max_angle_rad_) ||
      !(corner_max_angle_rad_ < M_PI) || !(corner_min_segment_m_ > 0.0) ||
      !(corner_approach_distance_m_ > corner_capture_radius_m_) ||
      !(corner_capture_radius_m_ > 0.0) || !(corner_lateral_tolerance_m_ > 0.0) ||
      !(corner_approach_speed_mps_ > 0.0) || !(corner_approach_min_speed_mps_ > 0.0) ||
      !(corner_approach_min_speed_mps_ <= corner_approach_speed_mps_) ||
      !(corner_approach_kp_ > 0.0) || !(corner_terminal_guard_m_ >= 0.0) ||
      !(corner_timeout_sec_ > 0.0))
    {
      throw nav2_core::PlannerException(
        "ThreePhaseController: standard corner parameters are inconsistent");
    }
    if (corner_approach_distance_m_ > approach_dist_m_ && approach_enabled_) {
      RCLCPP_WARN(
        logger_,
        "[%s] corner_approach_distance=%.3f exceeds approach_dist=%.3f; "
        "corner phase remains bounded and uses its own speed cap",
        name_.c_str(), corner_approach_distance_m_, approach_dist_m_);
    }
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
  monitor_sub_.reset();
  {std::lock_guard<std::mutex> lock(slam_mutex_);slam_pose_={};slam_received_=-1.;slam_valid_=false;}
  alignment_collision_.cleanup();
  pending_plan_.reset();beginExecution();execution_.reset();execution_generation_=0;
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
  pending_plan_.reset();beginExecution();
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
      phase_started_ = budget_clock_.now();
      if (p == Phase::kAlignStart) {start_alignment_engaged_ = false;}
    }
    return;
  }
  RCLCPP_INFO(
    logger_, "[%s] 相位 %s -> %s : %s", name_.c_str(), toString(phase_), toString(p), why);
  phase_ = p;
  phase_started_ = budget_clock_.now();
  align_coasting_ = false;
  if (p == Phase::kAlignStart) {start_alignment_engaged_ = false;}
}

void ThreePhaseController::resetCornerState(bool clear_detection)
{
  corner_stop_.reset();corner_braking_=false;corner_stage_=CornerStage::Idle;
  corner_active_ = false;
  corner_cursor_ = 0U;
  corner_completed_count_ = 0U;
  corner_started_s_ = 0.0;
  corner_heading_ = 0.0;
  corner_position_ = {};
  corner_reanchor_pending_ = false;
  corner_position_recovering_ = false;
  if (clear_detection) {
    corners_.clear();
  }
  emitCornerState("reset");
}

namespace {
std::vector<PlanarPoint> directedVertices(const nav_msgs::msg::Path & path)
{
  std::vector<PlanarPoint> result;
  for (const auto & pose : path.poses) {
    const PlanarPoint next{pose.pose.position.x, pose.pose.position.y};
    if (!result.empty() && std::hypot(next.x-result.back().x, next.y-result.back().y)<1e-8) {
      continue;
    }
    while (result.size()>=2U) {
      const auto & a=result[result.size()-2U];const auto & b=result.back();
      const double ax=b.x-a.x, ay=b.y-a.y, bx=next.x-b.x, by=next.y-b.y;
      if (ax*bx+ay*by<=0. || std::abs(ax*by-ay*bx)>1e-8*std::hypot(ax,ay)*std::hypot(bx,by)) {
        break;
      }
      result.pop_back();
    }
    result.push_back(next);
  }
  return result;
}
bool sameDirectedRoute(const nav_msgs::msg::Path & old_path, const nav_msgs::msg::Path & new_path)
{
  if (old_path.header.frame_id!=new_path.header.frame_id) {return false;}
  const auto a=directedVertices(old_path), b=directedVertices(new_path);
  if (a.size()!=b.size()) {return false;}
  for (std::size_t i=0;i<a.size();++i) {
    if (std::hypot(a[i].x-b[i].x,a[i].y-b[i].y)>1e-4) {return false;}
  }
  return true;
}
}

void ThreePhaseController::setPlan(const nav_msgs::msg::Path & path)
{
  // Nav2 calls progress_checker.reset() after this callback at execution start.
  // Commit on the first control tick, when that explicit generation is known.
  if (execution_) {pending_plan_=path;return;}
  applyPlan(path);
}

void ThreePhaseController::beginExecution()
{
  has_last_goal_=false;has_corner_pose_=false;has_tick_=false;accepted_plan_stamp_ns_=0;
  resetCornerState(true);start_alignment_engaged_=false;arrival_logged_=false;
}

void ThreePhaseController::applyPendingPlan()
{
  if (!pending_plan_) {return;}
  if (execution_) {
    const auto generation=execution_->generation();
    if (generation==0) {
      throw nav2_core::PlannerException("CONTROLLER_EXECUTION_UNAVAILABLE: PolicyProgressChecker required");
    }
    if (generation!=execution_generation_) {beginExecution();execution_generation_=generation;}
  }
  applyPlan(*pending_plan_);
  pending_plan_.reset();
}

void ThreePhaseController::applyPlan(const nav_msgs::msg::Path & path)
{
  const rclcpp::Time now=clock_->now();
  if (path.poses.empty()) {
    plan_update_=PlanUpdate::NewExecution;
    plan_=path;if(inner_){inner_->setPlan(path);}
    start_heading_valid_=false;has_last_goal_=false;resetCornerState(true);
    enterPhase(Phase::kFollow,now,"空路径，无起始方向",true);
    return;
  }
  const auto & end=path.poses.back().pose.position;
  const PlanarPoint cur_end{end.x,end.y};
  bool same_goal=has_last_goal_ && isSameGoal(last_goal_end_,cur_end,new_goal_epsilon_m_);
  if (corner_turn_enabled_ && !plan_.poses.empty()) {
    same_goal=same_goal && std::hypot(end.x-plan_.poses.back().pose.position.x,
      end.y-plan_.poses.back().pose.position.y)<1e-4 && path.header.frame_id==plan_.header.frame_id &&
      std::abs(shortestAngularDiff(yawOf(plan_.poses.back().pose),yawOf(path.poses.back().pose)))<1e-4;
  }
  const double idle_gap=has_tick_?(now-last_tick_time_).seconds():0.;
  const bool fresh=(execution_ || corner_turn_enabled_) ? !has_last_goal_ :
    isFreshFollowAttempt(has_tick_,idle_gap,new_attempt_gap_sec_);
  const bool same_route=same_goal && sameDirectedRoute(plan_,path);
  const bool equivalent=corner_turn_enabled_ && same_route;
  const bool continuing=corner_turn_enabled_ && same_goal && !fresh;
  const auto old_phase_started=phase_started_;
  const bool was_corner_active=corner_active_;
  const int64_t new_stamp=rclcpp::Time(path.header.stamp).nanoseconds();
  const int64_t old_stamp=fresh ? 0 : accepted_plan_stamp_ns_;
  if (corner_turn_enabled_ && old_stamp>0 && new_stamp>0 && new_stamp<old_stamp) {
    return;  // Keep the latest plan when an older observation arrives.
  }
  if (continuing) {
    if (!equivalent) {
      if (new_stamp<=0 || (old_stamp>0 && new_stamp<=old_stamp)) {
        throw nav2_core::PlannerException("CORNER_UNVERSIONED_REPLAN: changed geometry without newer stamp");
      }
      const auto & first=path.poses.front().pose.position;
      if (!has_corner_pose_ || last_corner_pose_.header.frame_id!=path.header.frame_id ||
        std::hypot(first.x-last_corner_pose_.pose.position.x,
          first.y-last_corner_pose_.pose.position.y)>corner_approach_distance_m_)
      {
        throw nav2_core::PlannerException("CORNER_REPLAN_START_UNCONFIRMED: new route must start at current pose");
      }
    }
  }
  std::vector<PlanarPoint> points;
  for (const auto & pose:path.poses) {points.push_back({pose.pose.position.x,pose.pose.position.y});}
  if (corner_turn_enabled_) {
    const auto issue=inspectCornerGeometry(points,corner_min_segment_m_,corner_min_angle_rad_,corner_max_angle_rad_);
    const char * reason=nullptr;
    switch(issue.kind) {
      case CornerGeometry::ShortApproach:reason="CORNER_SHORT_APPROACH";break;
      case CornerGeometry::DenseDogleg:reason="CORNER_DENSE_GEOMETRY";break;
      case CornerGeometry::Turnaround:reason="CORNER_UNSUPPORTED_REVERSAL";break;
      case CornerGeometry::TerminalHandoff:
        RCLCPP_INFO(logger_,"CORNER_TERMINAL_HANDOFF index=%zu short terminal leg; not a completed corner",issue.index);
        break;
      default:break;
    }
    if(reason) {throw nav2_core::PlannerException(std::string(reason)+": index="+std::to_string(issue.index));}
  }
  auto detected=corner_turn_enabled_?detectStandardCorners(points,corner_min_segment_m_,
    corner_min_angle_rad_,corner_max_angle_rad_):std::vector<PathCorner>{};
  if (continuing && equivalent && detected.size()!=corners_.size()) {
    throw nav2_core::PlannerException("CORNER_REPLAN_IDENTITY_CHANGED: corner sequence differs");
  }
  if (corner_turn_enabled_) {
    const auto vertices=directedVertices(path);
    for (std::size_t i=1;i+1<vertices.size();++i) {
      const auto & a=vertices[i-1];const auto & b=vertices[i];const auto & c=vertices[i+1];
      if (std::hypot(b.x-a.x,b.y-a.y)<corner_min_segment_m_ ||
        std::hypot(c.x-b.x,c.y-b.y)<corner_min_segment_m_) {continue;}
      const double turn=std::abs(shortestAngularDiff(
        std::atan2(b.y-a.y,b.x-a.x),std::atan2(c.y-b.y,c.x-b.x)));
      if (turn>corner_max_angle_rad_) {
        throw nav2_core::PlannerException("CORNER_UNSUPPORTED_REVERSAL: route needs a separate turnaround maneuver");
      }
    }
  }
  // Validate before committing either the full path or the inner segment.
  plan_=path;last_goal_end_=cur_end;has_last_goal_=true;corners_=std::move(detected);
  accepted_plan_stamp_ns_=std::max(old_stamp,new_stamp);
  plan_update_=(same_goal && !fresh) ?
    (same_route?PlanUpdate::EquivalentRefresh:PlanUpdate::RouteReplacement):PlanUpdate::NewExecution;
  if (corner_turn_enabled_) {
    ++corner_plan_revision_;
    if (!(continuing && equivalent)) {
      resetCornerState(false);
      corner_reanchor_pending_=continuing;
      if (!continuing) {has_corner_pose_=false;}
    } else if (corner_active_ && corner_cursor_<corners_.size()) {
      corner_position_=corners_[corner_cursor_].position;
      corner_heading_=corners_[corner_cursor_].outgoing_heading;
    }
  } else {resetCornerState(false);}
  if(inner_){inner_->setPlan(path);}updateCornerSegment();
  auto heading_points=points;
  if (corner_turn_enabled_ && !corners_.empty()) {
    const std::size_t begin=corner_completed_count_>0?corners_.at(corner_completed_count_-1).index:0;
    const std::size_t end=corner_cursor_<corners_.size()?corners_[corner_cursor_].index:points.size()-1;
    heading_points.assign(points.begin()+begin,points.begin()+end+1);
  }
  double heading=0.;start_heading_valid_=pathStartHeading(heading_points,lookahead_m_,heading);
  if(start_heading_valid_){start_heading_=heading;}
  if (same_goal && (!corner_turn_enabled_ || (continuing && equivalent))) {
    if(fresh){phase_started_=budget_clock_.now();}
    if (continuing && equivalent) {emitCornerState("plan_refresh");}
    return;
  }
  arrival_logged_=false;
  enterPhase(align_start_enabled_ && start_heading_valid_?Phase::kAlignStart:Phase::kFollow,
    now,continuing?"validated route change; settle before reanchor":"new execution context",true);
  if (continuing && was_corner_active) {phase_started_=old_phase_started;}
  RCLCPP_DEBUG(logger_,"CORNER_PLAN revision=%zu corners=%zu cursor=%zu preserved=%d",
    corner_plan_revision_,corners_.size(),corner_cursor_,continuing && equivalent);
}

bool ThreePhaseController::terminalRefinementAllowed(
  const geometry_msgs::msg::PoseStamped & pose, double capture_radius) const
{
  if (!corner_turn_enabled_) {return true;}
  if (corner_active_ || corner_reanchor_pending_ ||
    pose.header.frame_id!=plan_.header.frame_id) {return false;}
  if (plan_.poses.size()<2U) {return true;}
  const std::size_t begin=corner_completed_count_>0U ?
    corners_.at(corner_completed_count_-1U).index : 0U;
  const std::size_t end=corner_cursor_<corners_.size() ?
    corners_[corner_cursor_].index : plan_.poses.size()-1U;
  std::vector<PlanarPoint> leg;
  double remaining=0.;
  for (std::size_t i=begin;i<plan_.poses.size();++i) {
    const auto & p=plan_.poses[i].pose.position;
    if (i<=end) {leg.push_back({p.x,p.y});}
    if (i>begin) {
      const auto & previous=plan_.poses[i-1U].pose.position;
      remaining+=std::hypot(p.x-previous.x,p.y-previous.y);
    }
  }
  // Heading-only paths have no segment to project onto; Arrival still checks goal distance.
  if (remaining==0.) {return capture_radius>=0.;}
  double progress=0.,lateral=0.;
  if (!projectPathProgress(leg,{pose.pose.position.x,pose.pose.position.y},progress,lateral)) {
    return false;
  }
  // Restrict projection to the current ordered leg, never a nearby future leg.
  return std::isfinite(remaining) && remaining-progress<=capture_radius;
}

void ThreePhaseController::updateCornerSegment()
{
  if (!inner_ || !corner_turn_enabled_) {return;}
  if (plan_.poses.size()<2U) {inner_->setPlan(plan_);return;}
  const std::size_t begin=corner_completed_count_>0U ?
    corners_.at(corner_completed_count_-1U).index : 0U;
  std::size_t end=plan_.poses.size()-1U;
  bool stop_at_corner=false;
  if (corner_cursor_<corners_.size()) {
    const auto & corner=corners_[corner_cursor_];
    double remaining=0.;
    for (std::size_t i=corner.index+1U;i<plan_.poses.size();++i) {
      const auto & a=plan_.poses[i-1U].pose.position;
      const auto & b=plan_.poses[i].pose.position;
      remaining+=std::hypot(b.x-a.x,b.y-a.y);
    }
    if (remaining>corner_approach_distance_m_+corner_terminal_guard_m_) {
      end=corner.index;stop_at_corner=true;
    }
  }
  // Only completed corners advance the start. A skipped terminal corner has
  // not been traversed, so it cannot discard the incoming route.
  auto segment=plan_;
  segment.poses.assign(plan_.poses.begin()+begin,plan_.poses.begin()+end+1U);
  if (stop_at_corner) {
    auto & orientation=segment.poses.back().pose.orientation;
    orientation.x=orientation.y=0.;
    orientation.z=std::sin(corners_[corner_cursor_].incoming_heading/2.);
    orientation.w=std::cos(corners_[corner_cursor_].incoming_heading/2.);
  }
  inner_->setPlan(segment);
}

bool ThreePhaseController::maybeStartCorner(
  const geometry_msgs::msg::PoseStamped & pose, double /*robot_yaw*/, double goal_xy_tolerance,
  double measured_speed)
{
  if (!corner_turn_enabled_ || phase_ != Phase::kFollow || corners_.empty() ||
    corner_cursor_ >= corners_.size() || plan_.poses.size() < 2U)
  {
    return false;
  }
  std::vector<PlanarPoint> points;
  points.reserve(plan_.poses.size());
  for (const auto & item : plan_.poses) {
    points.push_back({item.pose.position.x, item.pose.position.y});
  }
  const auto & corner=corners_[corner_cursor_];
  const std::size_t begin=corner_cursor_>0U?corners_[corner_cursor_-1U].index:0U;
  const double begin_arc=corner_cursor_>0U?corners_[corner_cursor_-1U].arc_length:0.;
  const std::vector<PlanarPoint> leg(points.begin()+begin,points.begin()+corner.index+1U);
  double progress=0.,lateral=0.;
  const PlanarPoint current{pose.pose.position.x,pose.pose.position.y};
  if (!projectPathProgress(leg,current,progress,lateral)) {return false;}
  progress+=begin_arc;
  const double overshoot=(current.x-corner.position.x)*std::cos(corner.incoming_heading)+
    (current.y-corner.position.y)*std::sin(corner.incoming_heading);
  if (overshoot>corner_capture_radius_m_) {
    throw nav2_core::PlannerException("CORNER_MISSED: crossed an unconfirmed corner");
  }
  if (corner_active_) {return true;}
  const double remaining = corner.arc_length - progress;
  double total_arc = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    total_arc += std::hypot(
      points[i].x - points[i - 1U].x, points[i].y - points[i - 1U].y);
  }
  const double post_corner_guard = std::max(corner_terminal_guard_m_, goal_xy_tolerance);
  if (total_arc - corner.arc_length <= corner_approach_distance_m_ + post_corner_guard) {
    RCLCPP_WARN(
      logger_,
      "[%s] 标准角点 #%zu 距终点 %.3fm 小于接近段+终点保护 %.3fm，跳过原地转向，"
      "交给终点 GoalChecker/ALIGN_GOAL，避免 action 先判到位",
      name_.c_str(), corner.index, total_arc - corner.arc_length,
      corner_approach_distance_m_ + post_corner_guard);
    ++corner_cursor_;
    updateCornerSegment();
    return false;
  }
  const double distance = std::hypot(
    corner.position.x - current.x, corner.position.y - current.y);
  const double entry=std::max(corner_approach_distance_m_,
    cornerStoppingDistance(measured_speed)+corner_capture_radius_m_);
  if (remaining < -corner_capture_radius_m_ || remaining > entry ||
    distance > entry + corner_capture_radius_m_ ||
    std::fabs(lateral) > corner_lateral_tolerance_m_)
  {
    return false;
  }
  corner_active_ = true;
  corner_stop_.reset();corner_braking_=false;setCornerStage(CornerStage::Approach);
  corner_position_ = corner.position;
  corner_heading_ = corner.outgoing_heading;
  corner_started_s_ = clock_->now().seconds();
  enterPhase(Phase::kCornerApproach, clock_->now(), "接近标准角点，禁止提前转弯", true);
  RCLCPP_INFO(
    logger_, "[%s] CORNER_APPROACH index=%zu turn=%.1fdeg remaining=%.3fm lateral=%.3fm "
    "heading=%.1fdeg",
    name_.c_str(), corner.index, corner.turn_angle * 180.0 / M_PI, remaining, lateral,
    corner.outgoing_heading * 180.0 / M_PI);
  return true;
}

geometry_msgs::msg::TwistStamped ThreePhaseController::cornerApproachCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::PoseStamped & phase_pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker,
  double distance_to_corner)
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header = pose.header;
  const double measured = std::hypot(velocity.linear.x, velocity.linear.y);
  const double now=clock_->now().seconds();
  const bool stopped=cornerStopped();
  if (distance_to_corner <= corner_capture_radius_m_) {
    recordCornerCommand(stop.twist,now);setCornerStage(CornerStage::SettlingBeforeTurn);
    if (!stopped) {return stop;}
    corner_braking_=false;setCornerStage(CornerStage::Turning);
    enterPhase(Phase::kAlignCorner, clock_->now(), "角点源帧停稳确认，开始转向", true);
    return stop;
  }

  if (measured>.01 && cornerStoppingDistance(measured)>=distance_to_corner-corner_capture_radius_m_*.5) {
    corner_braking_=true;
  }
  if (corner_braking_) {
    recordCornerCommand(stop.twist,now);setCornerStage(CornerStage::SettlingBeforeTurn);
    if (stopped) {corner_braking_=false;}
    return stop;
  }
  setCornerStage(CornerStage::Approach);

  auto command = inner_->computeVelocityCommands(pose, velocity, goal_checker);
  const double speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
  double cap = std::min(corner_approach_speed_mps_, corner_approach_kp_ * distance_to_corner);
  cap = std::max(corner_approach_min_speed_mps_, cap);
  if (speed > 1e-6 && (speed >= corner_approach_min_speed_mps_ || measured > 0.01)) {
    const double scale = std::min(1.0, cap / speed);
    command.twist.linear.x *= scale;
    command.twist.linear.y *= scale;
  } else {
    // corner_position_ is stored in the plan frame.  The Nav2 pose can arrive
    // in a local frame, so use the transformed pose for this fallback vector.
    const double yaw = yawOf(phase_pose.pose);
    const double dx = corner_position_.x - phase_pose.pose.position.x;
    const double dy = corner_position_.y - phase_pose.pose.position.y;
    const double forward = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double norm = std::max(distance_to_corner, 1e-9);
    command.twist.linear.x = cap * forward / norm;
    command.twist.linear.y = cap * lateral / norm;
  }
  // The defining invariant of this phase is no early rotation.
  command.twist.angular.z = 0.0;
  recordCornerCommand(command.twist,now);
  return command;
}

void ThreePhaseController::receiveSlamPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & message)
{
  std::lock_guard<std::mutex> lock(slam_mutex_);
  if(message.header.stamp.sec<0 || message.header.stamp.nanosec>=1000000000u) {
    slam_valid_=false;return;
  }
  if(slam_received_>=0. && rclcpp::Time(message.header.stamp)<=rclcpp::Time(slam_pose_.header.stamp))return;
  slam_pose_.header=message.header;slam_pose_.pose=message.pose.pose;
  slam_received_=steadyNow();
  const auto &p=slam_pose_.pose.position;const auto &q=slam_pose_.pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  slam_valid_=message.header.frame_id=="map" && std::isfinite(p.x) && std::isfinite(p.y) &&
    std::isfinite(p.z) && std::isfinite(norm) && std::abs(norm-1.)<.01;
}

std::pair<geometry_msgs::msg::PoseStamped,double> ThreePhaseController::slamObservation() const
{
  std::lock_guard<std::mutex> lock(slam_mutex_);
  if(slam_received_<0.)throw nav2_core::PlannerException("SLAM_POSE_UNAVAILABLE");
  if(!slam_valid_)throw nav2_core::PlannerException("SLAM_POSE_INVALID: expected map-frame chassis pose");
  return {slam_pose_,slam_received_};
}

bool ThreePhaseController::cornerStopped()
{
  const auto observation=slamObservation();const auto &pose=observation.first;
  return corner_stop_.observe(rclcpp::Time(pose.header.stamp).seconds(),observation.second,
    pose.pose.position.x,pose.pose.position.y,yawOf(pose.pose),cornerSettleDuration(),
    align_settled_wz_,corner_capture_radius_m_*.25,align_tol_rad_*.5);
}

void ThreePhaseController::recordCornerCommand(const geometry_msgs::msg::Twist & command,double now)
{
  (void)now;
  corner_stop_.command(command.linear.x,command.linear.y,command.angular.z,budget_clock_.now().seconds());
}

bool ThreePhaseController::cornerReanchorReady(const rclcpp::Time & now)
{
  if (!corner_turn_enabled_ || !corner_reanchor_pending_) {return true;}
  checkPhaseTimeout(now);
  const bool stopped=cornerStopped();
  geometry_msgs::msg::Twist zero;recordCornerCommand(zero,now.seconds());
  setCornerStage(CornerStage::ReanchorSettling);
  if (!stopped) {return false;}
  corner_reanchor_pending_=false;setCornerStage(CornerStage::Idle);
  return true;
}

void ThreePhaseController::recordCornerPause(double now)
{
  if (!corner_turn_enabled_) {return;}
  cornerStopped();
  geometry_msgs::msg::Twist zero;recordCornerCommand(zero,now);
}

void ThreePhaseController::setCornerStage(CornerStage stage)
{
  if (stage==corner_stage_) {return;}
  corner_stage_=stage;
  emitCornerState("transition");
}

void ThreePhaseController::emitCornerState(const char * reason)
{
  if (!corner_turn_enabled_ || !clock_) {return;}
  const char * names[]={"IDLE","APPROACH","SETTLING_BEFORE_TURN","TURNING","SETTLING_AFTER_TURN","RECOVERING","REANCHOR_SETTLING"};
  const double pose_s=has_corner_pose_?rclcpp::Time(last_corner_pose_.header.stamp).seconds():0.;
  RCLCPP_INFO(logger_,
    "CORNER_STATE state=%s cursor=%zu revision=%zu pose_s=%.9f "
    "schema=1 execution=%llu event=%llu ros_s=%.9f pose_valid=%d active=%d reason=%s",
    names[static_cast<int>(corner_stage_)],corner_cursor_,corner_plan_revision_,pose_s,
    static_cast<unsigned long long>(execution_generation_),
    static_cast<unsigned long long>(++corner_event_sequence_),clock_->now().seconds(),
    has_corner_pose_ && pose_s>0.,corner_active_,reason);
}

void ThreePhaseController::observeCornerPose()
{
  if (!corner_turn_enabled_) {return;}
  const auto pose=slamObservation().first;
  const double source=rclcpp::Time(pose.header.stamp).seconds();
  if(has_corner_pose_ && source<=last_corner_pose_time_s_)return;
  if (has_corner_pose_ && !corners_.empty() && phase_!=Phase::kDone) {
    const double dt=source-last_corner_pose_time_s_;
    const double translation=std::hypot(pose.pose.position.x-last_corner_pose_.pose.position.x,
      pose.pose.position.y-last_corner_pose_.pose.position.y);
    const double rotation=std::abs(shortestAngularDiff(yawOf(last_corner_pose_.pose),yawOf(pose.pose)));
    const double translation_limit=std::max(corner_approach_distance_m_,
      std::min(translation/dt,corner_feedback_linear_bound_mps_)*dt+2.*corner_capture_radius_m_);
    const double rotation_limit=std::max(corner_min_angle_rad_,
      std::min(rotation/dt,corner_feedback_angular_bound_radps_)*dt+2.*align_tol_rad_);
    if (translation>translation_limit || rotation>rotation_limit) {
      RCLCPP_WARN(logger_,
        "CORNER_POSE_DISCONTINUITY source=slam_pose stamp=%.6f previous=%.6f dt=%.6f "
        "distance=%.6f/%.6f rotation=%.6f/%.6f",
        source,last_corner_pose_time_s_,dt,translation,translation_limit,rotation,rotation_limit);
      throw nav2_core::PlannerException("CORNER_LOCALIZATION_DISCONTINUITY: reacquire route before motion");
    }
  }
  last_corner_pose_=pose;has_corner_pose_=true;last_corner_pose_time_s_=source;
}

geometry_msgs::msg::TwistStamped ThreePhaseController::cornerRotationCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity, double error)
{
  const double dx=corner_position_.x-pose.pose.position.x;
  const double dy=corner_position_.y-pose.pose.position.y;
  const double distance=std::hypot(dx,dy);
  const auto observed=slamObservation().first;
  const double observed_distance=std::hypot(corner_position_.x-observed.pose.position.x,
    corner_position_.y-observed.pose.position.y);
  if (!std::isfinite(distance) || observed_distance>corner_lateral_tolerance_m_) {
    throw nav2_core::PlannerException("CORNER_POSITION_LOST: displacement outside recovery region");
  }
  if (distance>corner_capture_radius_m_) {corner_position_recovering_=true;}
  if (distance<corner_capture_radius_m_*.5) {corner_position_recovering_=false;}
  auto command=rotateOnly(error,velocity.angular.z,pose.header);
  if (corner_position_recovering_) {command.twist.angular.z=0.;}
  // Closed-loop position holding counters rotation-induced translation without
  // feeding localization noise into the wheels. Existing execution sweeps apply.
  const double deadband=corner_capture_radius_m_*.25;
  if (distance>deadband) {
    const double speed=std::min(corner_approach_speed_mps_*.5,
      corner_approach_kp_*(distance-deadband));
    const double yaw=yawOf(pose.pose);
    command.twist.linear.x=speed*(std::cos(yaw)*dx+std::sin(yaw)*dy)/distance;
    command.twist.linear.y=speed*(-std::sin(yaw)*dx+std::cos(yaw)*dy)/distance;
  }
  return command;
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
  const double elapsed = (budget_clock_.now() - phase_started_).seconds();
  if (phase_ == Phase::kAlignStart || phase_ == Phase::kAlignGoal) {
    if (elapsed > align_timeout_sec_) {
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: ") + toString(phase_) + " 段超时 " +
        std::to_string(elapsed) + "s > " + std::to_string(align_timeout_sec_) + "s，原地对齐未完成");
    }
  } else if (phase_ == Phase::kCornerApproach || phase_ == Phase::kAlignCorner) {
    if (elapsed > corner_timeout_sec_) {
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: ") + toString(phase_) + " 段超时 " +
        std::to_string(elapsed) + "s > " + std::to_string(corner_timeout_sec_) +
        "s，标准角点未完成");
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
  applyPendingPlan();
  const rclcpp::Time now = clock_->now();

  last_tick_time_ = now;
  has_tick_ = true;

  double xy_tol = fallback_xy_tol_;
  double yaw_tol = fallback_yaw_tol_;
  resolveTolerances(goal_checker, xy_tol, yaw_tol);

  auto phase_pose = pose;
  if (!plan_.poses.empty() && pose.header.frame_id != plan_.header.frame_id) {
    try {
      auto latest=pose;latest.header.stamp={};
      phase_pose = tf_->transform(latest, plan_.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::PlannerException(
              std::string("ThreePhaseController: cannot transform pose to plan frame: ") + ex.what());
    }
  }
  if (corner_turn_enabled_) {
    observeCornerPose();
    if (!cornerReanchorReady(now)) {
      geometry_msgs::msg::TwistStamped stop;stop.header=pose.header;return stop;
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
  if (phase_ == Phase::kFollow) {
    maybeStartCorner(phase_pose, robot_yaw, xy_tol,std::hypot(velocity.linear.x,velocity.linear.y));
  }
  if (phase_ == Phase::kCornerApproach || phase_ == Phase::kAlignCorner) {
    checkPhaseTimeout(now);
    if (phase_ == Phase::kAlignCorner) {
      if (!costmap_ros_ || !costmap_ros_->isCurrent()) {
        throw nav2_core::PlannerException("CORNER_ROTATION_BLOCKED: costmap unavailable");
      }
      auto local = phase_pose;
      try {
        if (local.header.frame_id != costmap_ros_->getGlobalFrameID()) {
          local = tf_->transform(phase_pose, costmap_ros_->getGlobalFrameID());
        }
      } catch (const tf2::TransformException & error) {
        throw nav2_core::PlannerException(std::string("CORNER_ROTATION_TF_UNAVAILABLE: ") + error.what());
      }
      auto * map = costmap_ros_->getCostmap();
      const double local_yaw = yawOf(local.pose);
      const double target_yaw = local_yaw + shortestAngularDiff(robot_yaw, corner_heading_);
      {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
        if (!cornerRotationClear(*map, costmap_ros_->getRobotFootprint(),
            local.pose.position.x, local.pose.position.y, local_yaw, target_yaw)) {
          throw nav2_core::PlannerException("CORNER_ROTATION_BLOCKED: full footprint sweep");
        }
      }
      const double corner_error = shortestAngularDiff(robot_yaw, corner_heading_);
      const bool corner_stopped=cornerStopped();
      if (std::abs(corner_error)<=align_tol_rad_ &&
        std::hypot(corner_position_.x-phase_pose.pose.position.x,
          corner_position_.y-phase_pose.pose.position.y)<=corner_capture_radius_m_*.5)
      {
        geometry_msgs::msg::TwistStamped stop;stop.header=pose.header;
        recordCornerCommand(stop.twist,now.seconds());setCornerStage(CornerStage::SettlingAfterTurn);
        if (!corner_stopped) {return stop;}
        RCLCPP_INFO(
          logger_, "[%s] ALIGN_CORNER 完成 index=%zu error=%.3fdeg，恢复路径跟踪",
          name_.c_str(), corner_cursor_, std::fabs(corner_error) * 180.0 / M_PI);
        ++corner_cursor_;
        corner_completed_count_=corner_cursor_;
        corner_active_ = false;
        setCornerStage(CornerStage::Idle);corner_stop_.reset();
        updateCornerSegment();
        enterPhase(Phase::kFollow, now, "标准角点原地转向完成", true);
        return stop;
      }
      auto command=cornerRotationCommand(phase_pose, velocity, corner_error);
      setCornerStage(corner_position_recovering_?CornerStage::Recovering:CornerStage::Turning);
      {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
        for (const auto & twist : {command.twist,velocity}) {
          if (!cornerCommandClear(*map,costmap_ros_->getRobotFootprint(),
              local.pose.position.x,local.pose.position.y,local_yaw,
              twist.linear.x,twist.linear.y,twist.angular.z)) {
            throw nav2_core::PlannerException("CORNER_COMMAND_BLOCKED: correction or feedback sweep");
          }
        }
      }
      command.header=pose.header;
      recordCornerCommand(command.twist,now.seconds());
      return command;
    }
    const double distance_to_corner = std::hypot(
      corner_position_.x - phase_pose.pose.position.x,
      corner_position_.y - phase_pose.pose.position.y);
    auto command=cornerApproachCommand(
      pose, phase_pose, velocity, goal_checker, distance_to_corner);
    auto local=phase_pose;
    try {
      if (local.header.frame_id!=costmap_ros_->getGlobalFrameID()) {
        local=tf_->transform(local,costmap_ros_->getGlobalFrameID());
      }
    } catch (const tf2::TransformException & error) {
      throw nav2_core::PlannerException(std::string("CORNER_APPROACH_TF_UNAVAILABLE: ")+error.what());
    }
    auto * map=costmap_ros_->getCostmap();
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      for (const auto & twist : {command.twist,velocity}) {
        if (!cornerCommandClear(*map,costmap_ros_->getRobotFootprint(),
            local.pose.position.x,local.pose.position.y,yawOf(local.pose),
            twist.linear.x,twist.linear.y,twist.angular.z)) {
          throw nav2_core::PlannerException("CORNER_APPROACH_BLOCKED: command or feedback sweep");
        }
      }
    }
    return command;
  }

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
    const double held = (budget_clock_.now() - phase_started_).seconds();
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
      if (std::abs(start_err)>align_tol_rad_ || std::abs(wz_now)>align_settled_wz_) {
        alignment_collision_.requireRotationClear(pose,yawOf(pose.pose)+start_err);
      }
      return rotateOnly(start_err, wz_now, pose.header);

    case Phase::kCornerApproach:
    case Phase::kAlignCorner:
      // Corner phases return above after their explicit stop/rotate handling.
      // Keep a conservative zero command if a future change reaches here.
      {
        geometry_msgs::msg::TwistStamped stop;
        stop.header = pose.header;
        return stop;
      }

    case Phase::kAlignGoal:
      if (std::abs(goal_err)>align_tol_rad_ || std::abs(wz_now)>align_settled_wz_) {
        alignment_collision_.requireRotationClear(pose,yawOf(pose.pose)+goal_err);
      }
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

double ThreePhaseController::translationApproachLimit(double distance,double maximum) const
{
  return approach_enabled_ ? approachSpeedCap(distance,approach_dist_m_,approach_v_min_,maximum) : maximum;
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

  const double cap = translationApproachLimit(dist_to_goal_m, speed);
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
