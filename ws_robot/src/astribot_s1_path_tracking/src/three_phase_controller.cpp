// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/three_phase_controller.hpp"

#include <algorithm>
#include <cmath>
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

  // 窄通道配置单独打一行。不打的话，在线根本分不清「没触发」和「压根没开」——
  // 这两者的排查方向完全相反。
  if (narrow_enabled_) {
    RCLCPP_INFO(
      logger_,
      "[%s] 窄通道贴边通行已启用: 触发阈值=足迹%.0f/中心%.0f 消抖=%d拍 退出=%d拍 "
      "扫描=±%.2fm@%.3fm 沿通道=%.2fm/s 横向<=%.2fm/s wz<=%.2frad/s "
      "朝向闸门=%.3frad 有利周期=%.4frad | 红线: 卡住=%.1fs内推进<%.3fm "
      "绝对上限=%.0fs 最多接管=%d次 最大偏离=%.2fm",
      name_.c_str(),
      narrow_trigger_.footprint_lethal_threshold, narrow_trigger_.center_lethal_threshold,
      narrow_trigger_.trigger_ticks, narrow_clear_ticks_,
      narrow_scan_half_width_m_, narrow_scan_step_m_,
      narrow_limits_.v_along, narrow_limits_.v_lateral_max, narrow_limits_.wz_max,
      narrow_limits_.yaw_gate_rad, narrow_favorable_period_rad_,
      narrow_stall_timeout_sec_, narrow_stall_min_gain_m_, narrow_hard_timeout_sec_,
      narrow_max_engagements_, narrow_max_path_deviation_m_);
  } else {
    RCLCPP_WARN(
      logger_,
      "[%s] 窄通道贴边通行已**禁用**(narrow_enabled=false): "
      "足迹代价全覆盖(>=253)的通道将完全交给内层控制器，"
      "该档在本图占可行域 34.68%%，其中 3.42%% 需要贴边才过得去",
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

  // ---- 窄通道贴边通行 ----
  declare_parameter_if_not_declared(node, p + "narrow_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "narrow_trigger_ticks", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(node, p + "narrow_clear_ticks", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(
    node, p + "narrow_footprint_lethal_threshold", rclcpp::ParameterValue(253.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_center_lethal_threshold", rclcpp::ParameterValue(253.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_scan_half_width", rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(node, p + "narrow_scan_step", rclcpp::ParameterValue(0.01));
  declare_parameter_if_not_declared(node, p + "narrow_v_along", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(node, p + "narrow_v_lateral_max", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, p + "narrow_wz_max", rclcpp::ParameterValue(0.20));
  declare_parameter_if_not_declared(node, p + "narrow_kp_lateral", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, p + "narrow_kp_yaw", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, p + "narrow_yaw_gate", rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(
    node, p + "narrow_favorable_period", rclcpp::ParameterValue(0.7853981634));
  declare_parameter_if_not_declared(
    node, p + "narrow_heading_lookahead", rclcpp::ParameterValue(0.40));
  // 卡住判据按**进展**，不按时长（旧的 narrow_timeout 等于给窄通道设了
  // 2.5m 长度上限，实测连砍 3 次健康通行）。旧键若仍在配置里必须显式报错，
  // 不能静默忽略 —— 静默忽略会让人以为超时还在按旧语义生效。
  declare_parameter_if_not_declared(node, p + "narrow_stall_timeout", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_stall_min_gain", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, p + "narrow_hard_timeout", rclcpp::ParameterValue(120.0));
  declare_parameter_if_not_declared(node, p + "narrow_max_engagements", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(
    node, p + "narrow_max_path_deviation", rclcpp::ParameterValue(0.50));

  node->get_parameter(p + "narrow_enabled", narrow_enabled_);
  narrow_trigger_.trigger_ticks =
    static_cast<int>(node->get_parameter(p + "narrow_trigger_ticks").as_int());
  narrow_clear_ticks_ = static_cast<int>(node->get_parameter(p + "narrow_clear_ticks").as_int());
  node->get_parameter(
    p + "narrow_footprint_lethal_threshold", narrow_trigger_.footprint_lethal_threshold);
  node->get_parameter(
    p + "narrow_center_lethal_threshold", narrow_trigger_.center_lethal_threshold);
  node->get_parameter(p + "narrow_scan_half_width", narrow_scan_half_width_m_);
  node->get_parameter(p + "narrow_scan_step", narrow_scan_step_m_);
  node->get_parameter(p + "narrow_v_along", narrow_limits_.v_along);
  node->get_parameter(p + "narrow_v_lateral_max", narrow_limits_.v_lateral_max);
  node->get_parameter(p + "narrow_wz_max", narrow_limits_.wz_max);
  node->get_parameter(p + "narrow_kp_lateral", narrow_limits_.kp_lateral);
  node->get_parameter(p + "narrow_kp_yaw", narrow_limits_.kp_yaw);
  node->get_parameter(p + "narrow_yaw_gate", narrow_limits_.yaw_gate_rad);
  node->get_parameter(p + "narrow_favorable_period", narrow_favorable_period_rad_);
  node->get_parameter(p + "narrow_heading_lookahead", narrow_heading_lookahead_m_);
  node->get_parameter(p + "narrow_stall_timeout", narrow_stall_timeout_sec_);
  node->get_parameter(p + "narrow_stall_min_gain", narrow_stall_min_gain_m_);
  node->get_parameter(p + "narrow_hard_timeout", narrow_hard_timeout_sec_);
  if (node->has_parameter(p + "narrow_timeout")) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_timeout 已废弃且语义已变 —— 它把「走得久」当成"
      "「卡住」，等于给窄通道设了 v_along x timeout 的长度上限（实测 2.5m，连砍 3 次"
      "健康通行）。请改用 narrow_stall_timeout + narrow_stall_min_gain（按进展判）"
      "与 narrow_hard_timeout（绝对上限兜底）");
  }
  narrow_max_engagements_ =
    static_cast<int>(node->get_parameter(p + "narrow_max_engagements").as_int());
  node->get_parameter(p + "narrow_max_path_deviation", narrow_max_path_deviation_m_);

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
  if (!(new_goal_epsilon_m_ > 0.0)) {
    // 置 0 会让每次重规划都被判成新目标，退化回"每秒原地转一次"那个缺陷。
    throw nav2_core::PlannerException("ThreePhaseController: new_goal_epsilon 必须 > 0");
  }

  // ---- 窄通道参数校验 ----
  // 关掉时不校验：关掉意味着这些值一个都不会被用到，此时因为一个
  // 用不上的值拒绝启动只会让人误以为窄通道逻辑有问题。
  if (narrow_enabled_) {
    if (narrow_trigger_.trigger_ticks < 1) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_trigger_ticks 必须 >= 1");
    }
    if (narrow_clear_ticks_ < 1) {
      // 0 会让 narrowCleared 永不成立（见其实现），等于接管永不退出。
      throw nav2_core::PlannerException("ThreePhaseController: narrow_clear_ticks 必须 >= 1");
    }
    if (!(narrow_trigger_.footprint_lethal_threshold > 0.0) ||
      narrow_trigger_.footprint_lethal_threshold >= NarrowCostValues::kLethal)
    {
      // >= 254 等于把「真障碍」当成触发条件，那是红线区不是本层目标域。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_footprint_lethal_threshold 必须在 (0, 254) 内 —— "
        "254 是真障碍(红线)，不是本层的触发条件");
    }
    if (!(narrow_trigger_.center_lethal_threshold > 0.0)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_center_lethal_threshold 必须 > 0");
    }
    if (!(narrow_scan_half_width_m_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_scan_half_width 必须 > 0");
    }
    if (!(narrow_scan_step_m_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_scan_step 必须 > 0");
    }
    if (narrow_scan_step_m_ >= narrow_scan_half_width_m_) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_scan_step 必须小于 narrow_scan_half_width，"
        "否则横向扫描只有一个样本、寻优形同虚设");
    }
    if (narrow_scan_step_m_ > 0.025) {
      // 本档目标区间只有 0.032m 宽（外接 0.42 − 内切 0.388）。步长比它的一半
      // 还大就分辨不出「贴哪边能过」，扫描退化成噪声采样。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_scan_step 不得超过 0.025m —— "
        "本档可通行区间仅 0.032m 宽，步长过大等于分辨不出贴边位置");
    }
    if (!(narrow_limits_.v_along > 0.0) || !(narrow_limits_.v_lateral_max > 0.0) ||
      !(narrow_limits_.wz_max > 0.0))
    {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_v_along / v_lateral_max / wz_max 必须 > 0");
    }
    if (narrow_limits_.v_along > 0.30) {
      // 贴边通行是受限通行，不是正常跟踪。此处离墙不足 0.02m，
      // 高速下任何一拍的横向误差都直接变成撞墙。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_v_along 不得超过 0.30 m/s —— 贴边通行必须低速");
    }
    if (!(narrow_limits_.kp_lateral > 0.0) || !(narrow_limits_.kp_yaw > 0.0)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_kp_lateral / narrow_kp_yaw 必须 > 0");
    }
    if (!(narrow_limits_.yaw_gate_rad > 0.0)) {
      // <=0 会让「朝向没对好就不许前进」这道闸门永久打开 ——
      // 而这一档里朝向不对就是过不去，带着错的朝向往前走等于往卡死里走。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_yaw_gate 必须 > 0，否则朝向闸门形同虚设");
    }
    if (!(narrow_favorable_period_rad_ > 0.0)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_favorable_period 必须 > 0（正八边形足迹取 pi/4）");
    }
    if (narrow_limits_.yaw_gate_rad >= narrow_favorable_period_rad_ * 0.5) {
      // 闸门比「有利朝向误差」的取值上限还大，闸门永远不会关。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_yaw_gate 必须小于 narrow_favorable_period/2，"
        "否则有利朝向误差恒在闸门内、闸门永不生效");
    }
    if (!(narrow_heading_lookahead_m_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_heading_lookahead 必须 > 0");
    }
    if (!(narrow_stall_timeout_sec_ > 0.0)) {
      // 用户红线：所有脱困逻辑必须带超时，不能无限循环脱困。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_stall_timeout 必须 > 0（红线：脱困必须带超时）");
    }
    if (!(narrow_stall_min_gain_m_ > 0.0)) {
      // 置 0 会让「任何一点点弧长变化都算进展」，卡住判据形同虚设 ——
      // 栅格噪声就足以每拍刷新进展。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_stall_min_gain 必须 > 0，否则噪声就能冒充进展");
    }
    if (!(narrow_hard_timeout_sec_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_hard_timeout 必须 > 0");
    }
    if (narrow_hard_timeout_sec_ <= narrow_stall_timeout_sec_) {
      // 绝对上限比卡住判据还短 ⇒ 永远是绝对上限先触发，等于退回「按时长判」，
      // 也就是退回那个 2.5m 长度上限的缺陷。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_hard_timeout 必须大于 narrow_stall_timeout，"
        "否则绝对上限会先触发、等于退回「按时长判」那个缺陷");
    }
    if (narrow_stall_min_gain_m_ >= narrow_limits_.v_along * narrow_stall_timeout_sec_) {
      // 要求的推进量超过该时间窗内理论最大位移 ⇒ 正常通行也必然被判卡住。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_stall_min_gain 不得达到 v_along x narrow_stall_timeout"
        "（该窗口内的理论最大推进），否则正常通行也会被判成原地蹭");
    }
    if (narrow_max_engagements_ < 1) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_max_engagements 必须 >= 1");
    }
    if (!(narrow_max_path_deviation_m_ > 0.0)) {
      // 用户红线：脱困不能脱离全局参考路径太远。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_max_path_deviation 必须 > 0（红线：不得脱离参考路径）");
    }
    if (narrow_max_path_deviation_m_ < narrow_scan_half_width_m_) {
      // 扫描范围比允许偏离还大 ⇒ 扫描会主动选到一个「一旦走到就立刻判越界」
      // 的目标，接管必然自杀式退出。
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_max_path_deviation 必须 >= narrow_scan_half_width，"
        "否则横向扫描会选出立刻触发越界的目标");
    }
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

void ThreePhaseController::enterPhase(
  Phase p, const rclcpp::Time & now, const char * why, bool restart_timer)
{
  if (p == phase_) {
    // !!! 相位没变也可能必须重置计时器 !!!
    //
    // 这里曾经是无条件 return，导致一个可以永久锁死整套导航的 bug：
    // 新目标要求进 ALIGN_START，而上一个目标恰好是在 ALIGN_START 段被中止的
    // —— 相位值相同，于是直接 return，phase_started_ 保持上一次的值。
    // 于是新目标的第一拍就判"对齐超时"，抛异常 -> Controller patience exceeded
    // -> FollowPath abort -> 下一个目标又在 ALIGN_START 起步 -> 再次立刻超时。
    // **一旦第一次对齐超时，之后每个目标都必然瞬间失败，且永不恢复。**
    // 实测证据：日志里 "ALIGN_START 段超时 1108.297s > 15.0s" ——
    // 1108s 约等于从启动到当时的总时长，也就是这个计时器从来没被重置过；
    // 恢复探索后 12 个目标 12 个失败、0 次进度停滞（机器人根本没开始动）。
    if (shouldRestartPhaseTimer(phase_, p, restart_timer)) {
      RCLCPP_INFO(
        logger_, "[%s] 相位仍为 %s，但按新目标重置计时器 : %s",
        name_.c_str(), toString(phase_), why);
      phase_started_ = now;
    }
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

  if (path.poses.size() < 2U) {
    // 单点路径（通常是"已在目标处"）：没有起始方向可言，直接进跟踪段，
    // 由 GoalChecker 判定是否已到。不在这里报错 —— 这是合法情形。
    start_heading_valid_ = false;
    has_last_goal_ = false;
    enterPhase(Phase::kFollow, now, "路径顶点少于 2 个，无起始方向", true);
    return;
  }

  const auto & end = path.poses.back().pose.position;
  const PlanarPoint cur_end{end.x, end.y};

  // !!! 关键：默认行为树每秒重规划一次，每次都会调到这里 !!!
  // 若无条件重置到 ALIGN_START，机器人会每秒掉回起步对齐段。
  // 实测（未修时）：86 次重规划里 12 次真的停下来原地转，最长 6.76s，
  // 把连续行驶切成一段段。所以只有终点真的换了才重置相位。
  const bool same_goal =
    has_last_goal_ && isSameGoal(last_goal_end_, cur_end, new_goal_epsilon_m_);
  last_goal_end_ = cur_end;
  has_last_goal_ = true;

  // 接管次数按**目标**计，不按重规划计。默认行为树 1Hz 重规划，
  // 若在这里无条件清零，narrow_max_engagements_ 这道上限永远不会生效 ——
  // 「同一路径反复贴边」正是它要拦的东西。
  if (!same_goal) {
    narrow_engage_count_ = 0;
    narrow_gave_up_ = false;
    narrow_threw_here_ = false;
    narrow_hits_ = 0;
    narrow_clear_hits_ = 0;
    narrow_engaged_ = false;
  }

  // 起始朝向每次都重算（路径形状会变），但它只在需要重置相位时才被用到。
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
    // 同一目标的周期性重规划：保持当前相位，只换路径。
    // 这样起步对齐只在一条路径**开始时**做一次，符合需求 3(a) 的语义。
    RCLCPP_DEBUG(
      logger_, "[%s] 同一目标的重规划，保持相位 %s", name_.c_str(), toString(phase_));
    return;
  }

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

// =====================================================================
// 窄通道贴边通行（状态 3）
//
// 设计依据全部在头文件里。这里只强调一条实现纪律：
// **红线违反一律抛异常，绝不静默继续。**
// 静默继续的后果是机器人贴着真障碍硬挤，而日志上一切正常。
// =====================================================================

bool ThreePhaseController::planInCostmapFrame(std::vector<PlanarPoint> & out) const
{
  out.clear();
  if (plan_.poses.empty() || !costmap_ros_ || !tf_) {
    return false;
  }
  const std::string target = costmap_ros_->getGlobalFrameID();
  const std::string source = plan_.header.frame_id;

  if (source.empty()) {
    return false;      // 没有 frame_id 就无法确认口径，按拿不到处理
  }
  if (source == target) {
    out.reserve(plan_.poses.size());
    for (const auto & ps : plan_.poses) {
      out.push_back(PlanarPoint{ps.pose.position.x, ps.pose.position.y});
    }
    return true;
  }

  // 只查一次变换再自己乘，而不是对每个点调一次 tf ——
  // 路径动辄上百点，每点一次 tf 查询在 20Hz 控制环里是不可接受的开销。
  //
  // 用 TimePointZero 且**不带 timeout**：带 timeout 的 lookupTransform 从
  // 工作线程调用时，对动态 tf 会失败（本项目已记录过这个坑，且它只对
  // 动态 tf 发作、静态 tf 正常，因此看起来像是好的）。map->odom 是动态的。
  tf2::Transform xform;
  try {
    const auto tfs = tf_->lookupTransform(target, source, tf2::TimePointZero);
    tf2::fromMsg(tfs.transform, xform);
  } catch (const tf2::TransformException &) {
    return false;
  }

  out.reserve(plan_.poses.size());
  for (const auto & ps : plan_.poses) {
    const tf2::Vector3 v =
      xform * tf2::Vector3(ps.pose.position.x, ps.pose.position.y, 0.0);
    out.push_back(PlanarPoint{v.x(), v.y()});
  }
  return true;
}

bool ThreePhaseController::readCosts(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  double & center_cost, double & footprint_cost)
{
  if (!costmap_ros_) {
    return false;
  }
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }
  footprint_cache_ = costmap_ros_->getRobotFootprint();
  if (footprint_cache_.size() < 3U) {
    return false;      // 退化足迹算不出包络，宁可不接管
  }
  if (!collision_checker_) {
    collision_checker_ = std::make_unique<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap);
  } else {
    // 代价地图对象可能被重建（lifecycle 重启），每拍重设一次比缓存安全。
    collision_checker_->setCostmap(costmap);
  }

  // 代价地图会被它自己的更新线程改写，读之前必须上锁。
  // 注意：锁只护住读代价这一段，**绝不能**握着它去调内层控制器 ——
  // 内层（MPPI）自己也要拿这把锁，那是必死的死锁。
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  unsigned int mx = 0U;
  unsigned int my = 0U;
  if (!costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;      // 机器人出图
  }
  center_cost = static_cast<double>(costmap->getCost(mx, my));
  footprint_cost = collision_checker_->footprintCostAtPose(
    pose.pose.position.x, pose.pose.position.y, yaw, footprint_cache_);
  return true;
}

void ThreePhaseController::disengageNarrow(const char * why)
{
  if (narrow_engaged_) {
    RCLCPP_INFO(logger_, "[%s] 窄通道接管退出：%s", name_.c_str(), why);
  }
  narrow_engaged_ = false;
  narrow_hits_ = 0;
  narrow_clear_hits_ = 0;
}

ThreePhaseController::NarrowDecision ThreePhaseController::evaluateNarrow(
  const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now)
{
  NarrowDecision none;
  if (!narrow_enabled_) {
    return none;
  }

  const double robot_yaw = yawOf(pose.pose);

  double center_cost = 0.0;
  double footprint_cost = 0.0;
  if (!readCosts(pose, robot_yaw, center_cost, footprint_cost)) {
    // 缺数据不接管。但也不假装安全 —— 内层控制器有它自己的碰撞检查，
    // 这一拍交回给它，而不是由本层出一个基于空数据的指令。
    // 注意这条路径**不计失败次数**：拿不到数据是「没能开始」，不是「没穿过去」。
    disengageNarrow("代价地图不可用");
    return none;
  }

  // ---- 便宜的早退：开阔处直接放行，不去付路径变换/通道方向的开销 ----
  // 中心格致命与朝向无关，也在这里判掉。
  if (center_cost < narrow_trigger_.center_lethal_threshold &&
    footprint_cost < narrow_trigger_.footprint_lethal_threshold)
  {
    ++narrow_clear_hits_;
    narrow_hits_ = 0;
    if (narrow_engaged_ && narrowCleared(narrow_clear_hits_, narrow_clear_ticks_)) {
      // 成功穿过 ⇒ 连续失败计数清零。规则在 narrow_math 里，
      // 不在这里复制一份判断 —— 复制的那份必然与被测的那份漂开。
      updateNarrowFailureCount(/*cleared_through=*/ true, narrow_engage_count_);
      disengageNarrow("足迹已连续脱离致命带");
    }
    // 脱离窄通道即允许再次上报异常（若之后又遇到新的窄处）。
    narrow_threw_here_ = false;
    return none;
  }

  // =====================================================================
  // 前置条件：**能不能开始**。全部放在计数与 narrow_engaged_ 之前。
  //
  // 起初这几项写在接管之后，任一不可用就「进入接管 -> 立刻以失败退出」，
  // 于是实测出现：0.45 秒内「估不出通道方向」连撞三次上限，
  // 整条路径被判死（而同一轮里 43 次真实穿越一次都没误判）。
  // 「没能开始」与「没穿过去」必须分开 —— 这是同一个错的两面。
  // =====================================================================
  std::vector<PlanarPoint> path;
  if (!planInCostmapFrame(path)) {
    if (!warned_narrow_frame_) {
      warned_narrow_frame_ = true;
      RCLCPP_WARN(
        logger_,
        "[%s] 窄通道接管拿不到 '%s' -> '%s' 的路径变换，本层不接管（缺数据不等于安全）",
        name_.c_str(), plan_.header.frame_id.c_str(),
        costmap_ros_ ? costmap_ros_->getGlobalFrameID().c_str() : "?");
    }
    disengageNarrow("路径变换不可用");
    return none;
  }

  const PlanarPoint robot{pose.pose.position.x, pose.pose.position.y};

  double corridor_heading = 0.0;
  if (!corridorHeadingFromPath(path, robot, narrow_heading_lookahead_m_, corridor_heading)) {
    // 没有通道方向就定义不出「横向」这个轴，整套贴边逻辑无从下手。
    // 这是「没能开始」，**不计失败次数**（见上面那段的实测教训）。
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] 窄通道接管估不出通道方向（路径退化？），本层不接管。"
      "这一项不计入放弃上限", name_.c_str());
    disengageNarrow("估不出通道方向");
    return none;
  }

  // ---- 红线判据要的是「最有利朝向下的足迹代价」 ----
  // 正八边形顶点比边中点多伸出 0.034m，顶点朝墙压 254、边朝墙过得去。
  // 问「转正之后还撞不撞」，而不是「现在撞不撞」。详见 narrow_math.hpp。
  const double yaw_error =
    favorableYawError(robot_yaw, corridor_heading, narrow_favorable_period_rad_);
  const double favorable_yaw = robot_yaw - yaw_error;
  double favorable_cost = footprint_cost;
  double dummy_center = 0.0;
  if (!readCosts(pose, favorable_yaw, dummy_center, favorable_cost)) {
    disengageNarrow("代价地图不可用(有利朝向查询)");
    return none;
  }

  const bool have_path = !path.empty();
  const NarrowVerdict verdict = evaluateNarrowTrigger(
    center_cost, footprint_cost, favorable_cost, have_path, narrow_trigger_);

  switch (verdict) {
    case NarrowVerdict::kPhysicallyBlocked:
      // 🔴 红线：即使转到最有利朝向，足迹仍压真障碍(254) ⇒ 真的过不去。
      // 停车告警，禁止贴边硬挤。抛异常让 FollowPath 明确失败、由上层换路径。
      disengageNarrow("最有利朝向下仍压真障碍");
      if (narrow_threw_here_) {
        return none;      // 同一处只抛一次，见 narrow_threw_here_ 的注释
      }
      narrow_threw_here_ = true;
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: 最有利朝向(") + std::to_string(favorable_yaw) +
        "rad)下足迹代价仍为 " + std::to_string(favorable_cost) +
        " >= 254（真障碍），物理堵死，禁止贴边通行");

    case NarrowVerdict::kCenterLethal:
      // 机器人中心已在致命带内 ⇒ 物理放不进去，**不是本层职责**。
      // 这一档（< 0.388m）占该图 31.26%，正确响应是协调器 ESCAPE 把车挪出来。
      disengageNarrow("中心格致命，交给 ESCAPE");
      if (narrow_threw_here_) {
        return none;
      }
      narrow_threw_here_ = true;
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: 中心格代价 ") + std::to_string(center_cost) +
        " 已致命，通道宽度物理上放不进机器人，应由协调器 ESCAPE 挪车，本层不接管");

    case NarrowVerdict::kNoPath:
    case NarrowVerdict::kNone:
      // 走到这里说明 footprint_cost 过阈但 favorable/center 都正常，
      // 而 kNone 只可能来自 footprint_cost 未过阈 —— 上面的早退已经处理过。
      // 保留分支是为了 switch 穷尽（编译器会盯着），不做额外动作。
      ++narrow_clear_hits_;
      narrow_hits_ = 0;
      return none;

    case NarrowVerdict::kNarrow:
      ++narrow_hits_;
      narrow_clear_hits_ = 0;
      break;
  }

  // ---- 到这里 verdict 必为 kNarrow ----
  if (!narrow_engaged_) {
    if (narrow_hits_ < narrow_trigger_.trigger_ticks) {
      return none;      // 消抖未满，先让内层继续试
    }
    if (narrow_gave_up_) {
      // 已经就这条路径放弃过一次了，不再重复抛。
      //
      // 为什么需要这个闸：抛异常到 FollowPath 真的 abort 之间有延迟，
      // 期间 controller_server 仍在以 20Hz 调本函数。实测每次放弃都刷了
      // **15 条**同样的 WARN（0.75s x 20Hz）。多抛的 14 次没有任何作用，
      // 只是把日志淹掉 —— 而这条 WARN 恰恰是需要被看见的那种。
      return none;
    }
    if (shouldGiveUpNarrow(narrow_engage_count_, narrow_max_engagements_, narrow_gave_up_)) {
      // 连续多次贴边都没穿过去 ⇒ 这条路径本身不该走。显式失败让上层换路径，
      // 而不是在这里无限贴边（用户红线：不能无限循环脱困）。
      narrow_gave_up_ = true;
      throw nav2_core::PlannerException(
        std::string("ThreePhaseController: 同一路径连续贴边 ") +
        std::to_string(narrow_engage_count_) + " 次均未穿过，判定该路径不可行");
    }
    narrow_engaged_ = true;
    narrow_started_ = now;
    narrow_progress_ = NarrowProgressState{};   // 每次接管重新起算进展
    // 进入接管即先按「失败」计一次；穿过去时在早退分支清零。
    // 这样计数才是「连续未穿过的次数」。
    updateNarrowFailureCount(/*cleared_through=*/ false, narrow_engage_count_);
    RCLCPP_WARN(
      logger_,
      "[%s] 窄通道接管启动(第 %d 次): 足迹代价=%.0f(有利朝向 %.0f) 中心代价=%.0f "
      "朝向误差=%.3frad 沿通道速度=%.2fm/s 横向上限=%.2fm/s "
      "卡住判据=%.1fs内推进<%.3fm 绝对上限=%.0fs",
      name_.c_str(), narrow_engage_count_, footprint_cost, favorable_cost, center_cost,
      yaw_error, narrow_limits_.v_along, narrow_limits_.v_lateral_max,
      narrow_stall_timeout_sec_, narrow_stall_min_gain_m_, narrow_hard_timeout_sec_);
  }

  // 🔴 红线：必须带超时 —— 但按**进展**判，不按时长判，且进展用**剩余弧长**。
  //
  // 三版演化，每一版都是「拿衡量正常的量去衡量失败」：
  //  v1 时长：narrow_timeout 25s x 0.10m/s = 2.5m，等于悄悄给窄通道设了
  //     2.5 米长度上限。实测一条 2.5m 通道被连砍 3 次（每次停在 25.0x 秒），
  //     期间朝向误差 ±0.008rad、离路径最远 0.112m —— 完全健康，只是走得久。
  //  v2 从起点累计的弧长：replan_policy: on_invalid 会换路径，而新路径的
  //     起点就在机器人附近，于是这个量的原点会跳。实测两种表现：
  //       · 涨到 1.5037m 后换路径 → 掉回 ≈0 → 永远超不过高水位 → 6s 后误判
  //       · 新路径起点就在机器人身上 → 恒 0.000000m，一次都没涨过
  //     两次事件里机器人都在被指令以 0.10m/s 前进(指令 (0.076,0.065))。
  //  v3（现在）剩余弧长 + 换路径重开窗口 + 闸门关不计入窗口。
  //
  // 「真的卡住」= 时间在走，剩余弧长不再减少，且不是在按指令转向。
  if (pathWasReplaced(path, narrow_last_path_end_, narrow_last_path_size_)) {
    // 换了目标/换了路径 ⇒ 剩余弧长会整体跳变，旧高水位不再可比，必须重开窗口。
    // 不重开的话，换到更远的目标那一刻剩余弧长变大，永远超不过旧高水位。
    narrow_progress_ = NarrowProgressState{};
  }
  double remaining_m = 0.0;
  const bool have_progress = pathRemainingArc(path, robot, remaining_m);
  const double now_sec = now.seconds();
  // 闸门是否会关：只依赖 yaw_error 与门限，不需要等到 narrowVelocity 生成
  // 整个指令（那是后面的事）。**共用 yawGateHolding()**，不在这里重写判据 ——
  // 两处各写一遍会漂，而漂的后果正是"闸门关着却在累计无进展"=误杀。
  const bool will_hold_for_yaw = yawGateHolding(yaw_error, narrow_limits_.yaw_gate_rad);
  if (will_hold_for_yaw) {
    // 朝向闸门关着 = 机器人正在**按指令**原地转向到有利朝向，这是本策略
    // 要求的动作，不是卡住。此时弧长必然不动，把它计入无进展窗口就是
    // 「因为正确执行策略而被判失败」。
    //
    // 只推迟窗口、不给假进展：这样闸门一开、真的不动时，6s 照常触发。
    // 无限转圈的风险由 narrow_hard_timeout 兜住（下面那段）。
    narrow_progress_.last_gain_sec = now_sec;
  } else if (have_progress &&
    narrowStalled(
      remaining_m, now_sec, narrow_stall_min_gain_m_, narrow_stall_timeout_sec_,
      narrow_progress_))
  {
    disengageNarrow("原地蹭(剩余弧长不减)");
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: 贴边通行 ") +
      std::to_string(narrow_stall_timeout_sec_) + "s 内剩余弧长未减少 " +
      std::to_string(narrow_stall_min_gain_m_) + "m（当前剩余 " +
      std::to_string(remaining_m) + "m，最好 " +
      std::to_string(narrow_progress_.best_remaining_m) + "m），判定原地蹭");
  }

  // 绝对上限兜底：防止极端情况下（弧长判据本身失效）真的无限贴边。
  // 刻意设在「长通道正常通行」够用的量级之外，不参与日常判断。
  const double engaged_sec = (now - narrow_started_).seconds();
  if (engaged_sec > narrow_hard_timeout_sec_) {
    disengageNarrow("绝对上限兜底");
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: 窄通道贴边通行触及绝对上限 ") +
      std::to_string(engaged_sec) + "s > " + std::to_string(narrow_hard_timeout_sec_) + "s");
  }

  // 🔴 红线：不得脱离全局参考路径太远。
  double nearest_sq = std::numeric_limits<double>::max();
  for (const auto & pt : path) {
    const double d = ((pt.x - robot.x) * (pt.x - robot.x)) + ((pt.y - robot.y) * (pt.y - robot.y));
    nearest_sq = std::min(nearest_sq, d);
  }
  const double dev = std::sqrt(nearest_sq);
  if (dev > narrow_max_path_deviation_m_) {
    disengageNarrow("偏离参考路径过远");
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: 贴边通行已偏离参考路径 ") + std::to_string(dev) +
      "m > " + std::to_string(narrow_max_path_deviation_m_) + "m，停止以防乱跑");
  }

  // 横向扫描要查很多次代价，必须在锁内一次做完。
  LateralScanResult scan;
  {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    collision_checker_->setCostmap(costmap);
    const auto cost_fn = [this](double x, double y, double yaw) -> double {
        return collision_checker_->footprintCostAtPose(x, y, yaw, footprint_cache_);
      };
    scan = scanLateral(
      robot, robot_yaw, corridor_heading, narrow_scan_half_width_m_, narrow_scan_step_m_, cost_fn);
  }

  if (!scan.valid || scan.usable_count == 0U) {
    // 两侧扫描范围内没有任何一个不压真障碍的位置 ⇒ 物理堵死。
    disengageNarrow("横向扫描无可用位置");
    throw nav2_core::PlannerException(
      "ThreePhaseController: 窄通道横向扫描没有任何不压真障碍的位置，物理堵死");
  }

  const NarrowCommand nc = narrowVelocity(
    robot_yaw, corridor_heading, scan.best_offset_m, yaw_error, narrow_limits_);

  NarrowDecision out;
  out.take_over = true;
  out.cmd.header = pose.header;
  out.cmd.twist.linear.x = nc.vx;
  // 注意：这里**故意不套 zero_vy_in_follow_**。贴边通行的全部机制就是横移，
  // 把 vy 掐掉等于把这一层唯一起作用的自由度删掉，接管会变成「只会原地转」。
  out.cmd.twist.linear.y = nc.vy;
  out.cmd.twist.angular.z = nc.wz;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "[%s] 贴边通行: 通道方向=%.2frad 朝向误差=%.3frad%s 横向目标=%+.3fm "
    "代价=%.0f%s 有利朝向代价=%.0f 可用样本=%zu 偏离路径=%.3fm 指令=(%.3f, %.3f, %.3f)",
    name_.c_str(), corridor_heading, yaw_error,
    nc.holding_for_yaw ? "(闸门关:先转不前进)" : "",
    scan.best_offset_m, scan.best_cost, scan.saturated ? "(饱和:放弃横向寻优)" : "",
    favorable_cost, scan.usable_count, dev, nc.vx, nc.vy, nc.wz);

  return out;
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
      // 窄通道判定放在调用内层**之前**：一旦接管，内层这一拍就不该出指令。
      // 顺序反了会先算一遍内层再丢掉，白付一次 MPPI 的开销；更要紧的是
      // 内层被调用过就会更新它自己的内部状态，接管期间那些状态是脏的。
      const NarrowDecision narrow = evaluateNarrow(pose, now);
      if (narrow.take_over) {
        return narrow.cmd;
      }

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
