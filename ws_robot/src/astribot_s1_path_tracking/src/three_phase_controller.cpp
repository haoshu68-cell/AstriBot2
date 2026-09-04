// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/three_phase_controller.hpp"

#include "nav2_costmap_2d/footprint.hpp"

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

  // 缩足迹功能的话题与足迹解析。只有开着才付这些开销，也只有开着才跑启动守卫。
  if (narrow_square_enabled_) {
    std::string fp_topic;
    std::string fp_pub_topic;
    node->get_parameter(name_ + ".global_footprint_topic", fp_topic);
    node->get_parameter(name_ + ".global_published_footprint_topic", fp_pub_topic);
    loadFootprints(node, name_ + ".");
    global_footprint_pub_ =
      node->create_publisher<geometry_msgs::msg::Polygon>(fp_topic, rclcpp::QoS(1));
    global_footprint_pub_->on_activate();
    // 回读：global costmap 把当前足迹按 publish_frequency 发出来。
    // 判据只用**顶点数**，因为 published_footprint 是变换到机器人当前位姿的，
    // 坐标随位姿变，顶点数不变。8 点 = 默认足迹，4 点 = 窄通道足迹。
    global_footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      fp_pub_topic, rclcpp::QoS(1),
      [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        global_footprint_vertices_.store(static_cast<int>(msg->polygon.points.size()));
      });
    RCLCPP_INFO(
      logger_, "[%s] 缩足迹话题: 写 '%s' 回读 '%s'",
      name_.c_str(), fp_topic.c_str(), fp_pub_topic.c_str());
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
    // 第二入口单独打一行，理由同下面的预对齐：A/B 验证要靠日志反证这一臂
    // 到底开没开。上面那条横幅只写了 254/253 两个硬判据，而 254 永不出现的
    // 那一类通道**只能**从这一行看出接管是否可能发生。
    if (narrow_trigger_.saturation_stall_sec > 0.0) {
      RCLCPP_INFO(
        logger_,
        "[%s] 第二入口(代价场饱和+内层无进展)已启用: 饱和阈值=足迹>=%.0f "
        "窗口=%.1fs 位移下限=%.3fm 最短驻留=%.1fs | 254 永不出现的通道靠这条进，"
        "实测依据见 nav2_params 里 narrow_saturation_threshold 上方",
        name_.c_str(), narrow_trigger_.saturation_threshold,
        narrow_trigger_.saturation_stall_sec, narrow_trigger_.saturation_min_move_m,
        narrow_saturation_min_dwell_sec_);
    } else {
      RCLCPP_WARN(
        logger_,
        "[%s] 第二入口(代价场饱和+内层无进展)**已关闭**(narrow_saturation_stall_sec=%.1f)。"
        "足迹恒 253、254 永不出现的通道将无法接管 —— 这是显式回退档，不是默认值",
        name_.c_str(), narrow_trigger_.saturation_stall_sec);
    }
    // 预对齐单独打一行：A/B 验证要靠日志反证这一臂到底是开还是关，
    // 混在上面那条里 grep 不出来（本项目已经因为"反证不到位"废掉过一整轮数据）。
    if (narrow_prealign_enabled_) {
      RCLCPP_INFO(
        logger_,
        "[%s] 进入前朝向预对齐已启用: 前视=%.2fm@%.2fm 切向前视=%.2fm "
        "扫掠步长=%.2frad 超时=%.1fs 每目标上限=%d次 | 退出判据复用朝向闸门 %.3frad",
        name_.c_str(), narrow_prealign_.preview_m, narrow_prealign_.sample_step_m,
        narrow_prealign_.tangent_lookahead_m, narrow_prealign_sweep_step_rad_,
        narrow_prealign_timeout_sec_, narrow_prealign_max_per_goal_,
        narrow_limits_.yaw_gate_rad);
    } else {
      RCLCPP_INFO(
        logger_,
        "[%s] 进入前朝向预对齐已**禁用**(narrow_prealign_enabled=false): "
        "机器人会以当前朝向直接进入窄处，再在里面原地转（预对齐之前的旧行为）",
        name_.c_str());
    }
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
  declare_parameter_if_not_declared(node, p + "new_attempt_gap", rclcpp::ParameterValue(0.5));

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

  // ---- 窄通道贴边通行 ----
  declare_parameter_if_not_declared(node, p + "narrow_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p + "narrow_trigger_ticks", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(node, p + "narrow_clear_ticks", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(
    node, p + "narrow_footprint_lethal_threshold",
    rclcpp::ParameterValue(NarrowCostValues::kLethal));
  declare_parameter_if_not_declared(
    node, p + "narrow_center_lethal_threshold", rclcpp::ParameterValue(253.0));
  // 「代价场饱和 + 内层无进展」第二入口。默认开（阈值 253/3.0s/0.05m），
  // 因为 254 永不出现的那一类通道没有别的入口。要一键回退把
  // narrow_saturation_stall_sec 设成 0 即可（见下方启动守卫）。
  declare_parameter_if_not_declared(
    node, p + "narrow_saturation_threshold", rclcpp::ParameterValue(253.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_saturation_stall_sec", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_saturation_min_move", rclcpp::ParameterValue(0.15));
  // 第二入口的最短驻留。默认 4.0s 是算术下界 (favorable_period/2)/wz_max=3.93s
  // 上取整，不是调出来的值；理由见 narrow_saturation_min_dwell_sec_ 的声明。
  declare_parameter_if_not_declared(
    node, p + "narrow_saturation_min_dwell", rclcpp::ParameterValue(4.0));
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
    node, p + "narrow_yaw_resume", rclcpp::ParameterValue(0.06));
  declare_parameter_if_not_declared(
    node, p + "narrow_favorable_period", rclcpp::ParameterValue(M_PI / 2.0));
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
  // ---- 进入窄通道前的朝向预对齐 ----
  declare_parameter_if_not_declared(
    node, p + "narrow_prealign_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, p + "narrow_prealign_preview", rclcpp::ParameterValue(1.00));
  declare_parameter_if_not_declared(
    node, p + "narrow_prealign_sample_step", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, p + "narrow_prealign_sweep_step", rclcpp::ParameterValue(0.20));
  declare_parameter_if_not_declared(
    node, p + "narrow_prealign_timeout", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_prealign_max_per_goal", rclcpp::ParameterValue(5));
  // ---- 窄通道内临时缩小足迹 ----
  declare_parameter_if_not_declared(
    node, p + "narrow_square_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, p + "narrow_footprint_default", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, p + "narrow_footprint_narrow", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, p + "chassis_min_envelope_radius", rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_hard_timeout", rclcpp::ParameterValue(30.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_lease_period", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_verify_timeout", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_exit_preview", rclcpp::ParameterValue(1.20));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_exit_ticks", rclcpp::ParameterValue(6));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_min_dwell", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, p + "narrow_square_cooldown", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, p + "global_footprint_topic",
    rclcpp::ParameterValue(std::string("/global_costmap/footprint")));
  declare_parameter_if_not_declared(
    node, p + "global_published_footprint_topic",
    rclcpp::ParameterValue(std::string("/global_costmap/published_footprint")));

  node->get_parameter(p + "narrow_enabled", narrow_enabled_);
  narrow_trigger_.trigger_ticks =
    static_cast<int>(node->get_parameter(p + "narrow_trigger_ticks").as_int());
  narrow_clear_ticks_ = static_cast<int>(node->get_parameter(p + "narrow_clear_ticks").as_int());
  node->get_parameter(
    p + "narrow_footprint_lethal_threshold", narrow_trigger_.footprint_lethal_threshold);
  node->get_parameter(
    p + "narrow_center_lethal_threshold", narrow_trigger_.center_lethal_threshold);
  node->get_parameter(
    p + "narrow_saturation_threshold", narrow_trigger_.saturation_threshold);
  node->get_parameter(
    p + "narrow_saturation_stall_sec", narrow_trigger_.saturation_stall_sec);
  node->get_parameter(
    p + "narrow_saturation_min_move", narrow_trigger_.saturation_min_move_m);
  node->get_parameter(p + "narrow_saturation_min_dwell", narrow_saturation_min_dwell_sec_);
  node->get_parameter(p + "narrow_scan_half_width", narrow_scan_half_width_m_);
  node->get_parameter(p + "narrow_scan_step", narrow_scan_step_m_);
  node->get_parameter(p + "narrow_v_along", narrow_limits_.v_along);
  node->get_parameter(p + "narrow_v_lateral_max", narrow_limits_.v_lateral_max);
  node->get_parameter(p + "narrow_wz_max", narrow_limits_.wz_max);
  node->get_parameter(p + "narrow_kp_lateral", narrow_limits_.kp_lateral);
  node->get_parameter(p + "narrow_kp_yaw", narrow_limits_.kp_yaw);
  node->get_parameter(p + "narrow_yaw_gate", narrow_limits_.yaw_gate_rad);
  node->get_parameter(p + "narrow_yaw_resume", narrow_limits_.yaw_resume_rad);
  node->get_parameter(p + "narrow_favorable_period", narrow_favorable_period_rad_);
  node->get_parameter(p + "narrow_heading_lookahead", narrow_heading_lookahead_m_);
  node->get_parameter(p + "narrow_prealign_enabled", narrow_prealign_enabled_);
  node->get_parameter(p + "narrow_prealign_preview", narrow_prealign_.preview_m);
  node->get_parameter(p + "narrow_prealign_sample_step", narrow_prealign_.sample_step_m);
  node->get_parameter(p + "narrow_prealign_sweep_step", narrow_prealign_sweep_step_rad_);
  node->get_parameter(p + "narrow_prealign_timeout", narrow_prealign_timeout_sec_);
  narrow_prealign_max_per_goal_ =
    static_cast<int>(node->get_parameter(p + "narrow_prealign_max_per_goal").as_int());
  node->get_parameter(p + "narrow_square_enabled", narrow_square_enabled_);
  node->get_parameter(p + "narrow_square_hard_timeout", narrow_square_hard_timeout_sec_);
  node->get_parameter(p + "narrow_square_lease_period", narrow_square_lease_period_sec_);
  node->get_parameter(p + "narrow_square_verify_timeout", narrow_square_verify_timeout_sec_);
  node->get_parameter(p + "narrow_square_exit_preview", narrow_square_exit_preview_m_);
  narrow_square_exit_ticks_ =
    static_cast<int>(node->get_parameter(p + "narrow_square_exit_ticks").as_int());
  node->get_parameter(p + "narrow_square_min_dwell", narrow_square_min_dwell_sec_);
  node->get_parameter(p + "narrow_square_cooldown", narrow_square_cooldown_sec_);
  // 预对齐与既有判据**共用同一个值**，不另设阈值 —— 两处各存一份必然漂开。
  narrow_prealign_.tangent_lookahead_m = narrow_heading_lookahead_m_;
  narrow_prealign_.favorable_period_rad = narrow_favorable_period_rad_;
  narrow_prealign_.footprint_lethal_threshold = narrow_trigger_.footprint_lethal_threshold;
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
  if (!(new_attempt_gap_sec_ > 0.0) || !(new_attempt_gap_sec_ < align_timeout_sec_)) {
    // 下界：置 0/负数会让每次重规划都被判成"新一次尝试"，align_timeout 永远
    //       等不到触发 —— 等于把"原地转不动"这道保护关掉。
    // 上界：>= align_timeout 则空档判据永远比超时晚，救不了那个锁死。
    throw nav2_core::PlannerException(
      "ThreePhaseController: new_attempt_gap 必须 in (0, align_timeout)");
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
    // 🔴 足迹**多边形**的阈值必须 > 253。
    //    这条守卫的上一版写的是「必须 < 254」，恰好把唯一正确的值挡在门外，
    //    理由写的是「254 是真障碍(红线)，不是本层触发条件」—— 那个理由是错的：
    //    多边形查询里 253 表示"外轮廓离障碍不超过内切半径"，而内切半径就是
    //    底盘半宽，等于把底盘算了两遍。实测后果：任何窄于 ~1.58m 的通道里
    //    足迹代价恒为 253、零梯度，909 次「连小足迹也过不去」全是这么来的。
    //    两种查询两个阈值的完整推导见 narrow_math.hpp 的 NarrowCostValues。
    if (narrow_trigger_.footprint_lethal_threshold <= NarrowCostValues::kInscribedInflated) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_footprint_lethal_threshold(" +
        std::to_string(narrow_trigger_.footprint_lethal_threshold) +
        ") 必须 > 253。足迹多边形已经表达了底盘尺寸，拿 253(膨胀内切带)当碰撞阈值"
        "等于重复计底盘半宽 —— 后果是窄于约 4 倍底盘半宽的通道里恒为 253、零梯度。"
        "正确值是 254(真障碍本体)。中心格查询才用 253。");
    }
    if (narrow_trigger_.footprint_lethal_threshold > NarrowCostValues::kNoInformation) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_footprint_lethal_threshold 不得 > 255");
    }
    // 🔴 中心格阈值反过来必须 <= 253：那里膨胀带正好代表底盘尺寸。
    //    设成 254 等于「只有中心压在障碍本体上才算致命」，会把机器人
    //    明明放不进去的位置判成可站。
    if (!(narrow_trigger_.center_lethal_threshold > 0.0) ||
      narrow_trigger_.center_lethal_threshold > NarrowCostValues::kInscribedInflated)
    {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_center_lethal_threshold(" +
        std::to_string(narrow_trigger_.center_lethal_threshold) +
        ") 必须在 (0, 253] 内。中心格判据里膨胀带就代表底盘尺寸，"
        "设成 254 会把放不进去的位置判成可站");
    }
    if (!(narrow_scan_half_width_m_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: narrow_scan_half_width 必须 > 0");
    }
    // ---- 「饱和无进展」第二入口的参数校验 ----
    // stall_sec <= 0 是**合法的一键关闭**（窗口永不满足，回退到只有 254 入口），
    // 所以这里不拒绝 0，只拒绝自相矛盾的取值。
    if (narrow_trigger_.saturation_stall_sec > 0.0) {
      if (!(narrow_trigger_.saturation_threshold > 0.0) ||
        narrow_trigger_.saturation_threshold >= narrow_trigger_.footprint_lethal_threshold)
      {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_saturation_threshold(" +
          std::to_string(narrow_trigger_.saturation_threshold) +
          ") 必须在 (0, narrow_footprint_lethal_threshold) 内。"
          "与 254 相等或更大时这条入口与 ④ 完全重合，等于没加，"
          "而 254 永不出现的那一类通道仍然进不去");
      }
      if (!(narrow_trigger_.saturation_min_move_m > 0.0)) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_saturation_min_move 必须 > 0，"
          "否则任何位移都算推进、这条入口永不触发");
      }
      // 🔴 门槛的上界：本层自己在同样窗口内能走多远。
      // 交接只有在「本层比内层快」时才有意义；门槛 >= v_along*窗口 意味着
      // 连本层自己都达不到这个推进量，等于随时都判内层无进展 —— 那是滥用。
      const double layer_reach_m =
        narrow_limits_.v_along * narrow_trigger_.saturation_stall_sec;
      if (narrow_trigger_.saturation_min_move_m >= layer_reach_m) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_saturation_min_move(" +
          std::to_string(narrow_trigger_.saturation_min_move_m) +
          "m) 必须 < narrow_v_along * narrow_saturation_stall_sec = " +
          std::to_string(layer_reach_m) +
          "m，否则连贴边层自己都达不到这个推进量、等于恒判内层无进展");
      }
      // 🔴 最短驻留的算术下界：朝向误差到最近有利朝向不会超过 period/2，
      // 以 wz_max 转正就要 (period/2)/wz_max 秒。驻留短于这个数 ⇒ 接管必然
      // 在「还没转正」时被代价纹理赶出去（实测 13/18 次就是这样，见声明处）。
      const double align_bound_sec =
        saturationAlignBoundSec(narrow_favorable_period_rad_, narrow_limits_.wz_max);
      if (narrow_saturation_min_dwell_sec_ < align_bound_sec) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_saturation_min_dwell(" +
          std::to_string(narrow_saturation_min_dwell_sec_) +
          "s) 必须 >= (narrow_favorable_period/2)/narrow_wz_max = " +
          std::to_string(align_bound_sec) +
          "s，否则第二入口永远在转正之前就被代价纹理赶出去、只会原地振荡");
      }
      // 驻留必须短于卡住判据，否则驻留期本身就会撞上「原地蹭」而被判死。
      if (narrow_saturation_min_dwell_sec_ >= narrow_stall_timeout_sec_) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_saturation_min_dwell(" +
          std::to_string(narrow_saturation_min_dwell_sec_) +
          "s) 必须 < narrow_stall_timeout(" +
          std::to_string(narrow_stall_timeout_sec_) + "s)");
      }
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
  // 🔴 朝向层迟滞守卫：交回阈值必须**严格小于**闸门。
  // 两者相等即没有迟滞，保证在闸门上自激 —— 实测 749 次重新对齐、
  // 30s 硬超时窗口内一步未前进，现象是"能进不能出"，而每条日志都合理。
  // 迟滞带还必须大于抖动幅度，否则等于没加：实测误差分布紧贴闸门
  // (min=0.120 p50=0.131 p95=0.215)，超出量 p95 达 0.095rad。
  if (!(narrow_limits_.yaw_resume_rad > 0.0) ||
      narrow_limits_.yaw_resume_rad >= narrow_limits_.yaw_gate_rad)
  {
    throw std::runtime_error(
      "ThreePhaseController: narrow_yaw_resume(" +
      std::to_string(narrow_limits_.yaw_resume_rad) +
      ") 必须 > 0 且 **严格小于** narrow_yaw_gate(" +
      std::to_string(narrow_limits_.yaw_gate_rad) +
      ")。两者相等就是没有迟滞 -> 在闸门上自激 -> 只进不出。");
    }
    if (!(narrow_favorable_period_rad_ > 0.0)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: narrow_favorable_period 必须 > 0");
    }
    // 逐个足迹校验周期是否保侧向包络。
    // ⚠️ 缩足迹开启时**不在这里**校验：declareAndLoadParams() 跑在
    //    loadFootprints() **之前**，此刻 footprint_default_/narrow_ 还是空的，
    //    check() 会因 size<3 直接 return —— 那是一条静默失效的守卫，
    //    比没有守卫更糟。所以那种情况移到 loadFootprints() 末尾去做。
    if (!narrow_square_enabled_ && costmap_ros_) {
      checkPeriodPreservesLateral(costmap_ros_->getRobotFootprint(), "代价地图当前足迹");
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
    if (narrow_prealign_enabled_) {
      if (!(narrow_prealign_.preview_m > 0.0) || !(narrow_prealign_.sample_step_m > 0.0)) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_prealign_preview / sample_step 必须 > 0");
      }
      if (narrow_prealign_.sample_step_m > narrow_prealign_.preview_m) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_prealign_sample_step 不得大于 narrow_prealign_preview，"
          "否则前视一个点都取不到、预对齐永不触发且不报错");
      }
      if (!(narrow_prealign_sweep_step_rad_ > 0.0)) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_prealign_sweep_step 必须 > 0 —— "
          "扫掠校验是旋转前的安全前置，步长非法时本层会拒绝一切旋转");
      }
      if (!(narrow_prealign_timeout_sec_ > 0.0)) {
        // 用户红线：所有脱困/受限动作必须带超时。
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_prealign_timeout 必须 > 0");
      }
      if (narrow_prealign_max_per_goal_ <= 0) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: narrow_prealign_max_per_goal 必须 >= 1（要关就用 "
          "narrow_prealign_enabled:false，而不是把上限设成 0 让它静默失效）");
      }
      // 🔴 阈值顺序陷阱：退出判据用 narrow_yaw_gate，而 rotateOnly 的角速度在
      // align_tolerance 以内就被压成 0。若 align_tolerance >= yaw_gate，
      // 车会在还没到闸门就停止转动 ⇒ 预对齐每次都只能靠超时收场，且日志上
      // 看起来像「转不过去」而不是「配置颠倒」。
      if (align_tol_rad_ >= narrow_limits_.yaw_gate_rad) {
        throw nav2_core::PlannerException(
          "ThreePhaseController: align_tolerance 必须 < narrow_yaw_gate，否则原地转会在"
          "到达朝向闸门之前就停住，预对齐只能靠超时结束");
      }
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
  // 🔴 防锁存：本层若带着小足迹退出，代价地图会**一直**按小足迹算，
  //    机器人从此被系统性低估，而日志上一切正常。所以 cleanup/deactivate
  //    都必须无条件复原，且要在销毁发布者之前发出去。
  revertFootprint("控制器 cleanup");
  global_footprint_sub_.reset();
  global_footprint_pub_.reset();
  if (inner_) {
    inner_->cleanup();
    inner_.reset();
  }
  inner_loader_.reset();
}

void ThreePhaseController::activate()
{
  // 每次激活都从"默认足迹"这个已知状态起步：上一次会话若异常退出，
  // 代价地图上可能还挂着小足迹。
  square_active_ = false;
  square_pending_ = false;
  if (narrow_square_enabled_) {
    requestFootprint(footprint_default_);
    RCLCPP_INFO(
      logger_, "[%s] 激活：已把足迹重置为默认(%zu 点)，防上一次会话残留小足迹",
      name_.c_str(), footprint_default_.size());
  }
  if (inner_) {
    inner_->activate();
  }
}

void ThreePhaseController::deactivate()
{
  revertFootprint("控制器 deactivate");
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
    // 预对齐次数同理按目标计。若在这里无条件清零，1Hz 重规划会让
    // narrow_prealign_max_per_goal_ 这道防振荡上限永远不生效。
    prealign_used_ = 0;
    warned_prealign_cap_ = false;
    // 但**正在进行的旋转必须中断**：新目标的通道方向大概率不同，
    // 继续转向旧目标算出的朝向是在往错的方向转，且没人会纠正它。
    prealign_active_ = false;
    // 🔴 新目标无条件复原足迹：把锁存窗口压到一个目标之内。
    //    新目标的窄处在哪、要不要缩足迹，都要重新判一次。
    revertFootprint("换了新目标");
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
    //
    // ⚠2026-09-04 实机根因就在这个 return 上。「同一目标」其实混了两件事：
    //   (a) action **还活着**，nav2/协调器周期重规划 -> 确实不该重置相位，
    //       否则退化回"每秒原地转一次"（实测 86 次重规划 12 次真停下来转）。
    //   (b) 上一次 FollowPath 已经 abort，协调器把**同一个目标**重新下发 ——
    //       这是一次全新的尝试，必须给它一份完整的对齐预算。
    // 老代码把 (b) 也走了这条 return，于是 enterPhase 压根没被调用，
    // phase_started_ 继续沿用旧值。实机时间轴：734.0s 最后一次真·新目标之后，
    // 同一目标 (1.68, 8.43) 每 1.5s 重下发，计时器单调爬到 110.708s，
    // 每条新路径第一拍就抛 "ALIGN_START 段超时 110.708s > 15.0s" ->
    // Controller patience exceeded ×43 -> Aborting handle ×48 ->
    // 上游 3 连败 -> PAUSED -> 自动恢复 3 次全在 3s 内再死 -> 永久 parked。
    //
    // 两者路径内容几乎一样，分不开；能分开的只有**时间空档**：action 存续
    // 期间 nav2 以 controller_frequency 持续 tick，action 一结束 tick 就停。
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

ThreePhaseController::PrealignDecision ThreePhaseController::evaluatePrealign(
  const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now)
{
  PrealignDecision none;
  if (!narrow_enabled_) {
    // 窄通道层整体关掉时，绝不能留着小足迹 —— 那是最危险的锁存。
    revertFootprint("窄通道层已禁用");
    return none;
  }
  if (!narrow_prealign_enabled_ && !square_active_ && !square_pending_) {
    return none;
  }
  const double robot_yaw = yawOf(pose.pose);

  // ---- 小足迹已生效/待确认：每拍都必须先走它 ----
  // 复原判据、超时、重新对齐都在里面。
  // ⚠️ 拿不到路径时**不能**就此不动作，否则拿不到路径就等于把小足迹锁存了。
  //    所以这里保留一条退化通路：只看当前位姿的大足迹代价（会被 square_exit_degraded_
  //    计数并写进日志，避免"退化了但看不出来"）。
  if (narrow_square_enabled_ && (square_active_ || square_pending_)) {
    SquareExitProbe exit;
    std::vector<PlanarPoint> path;
    const bool have_path = planInCostmapFrame(path);
    const PlanarPoint robot{pose.pose.position.x, pose.pose.position.y};

    const bool got = withCostQueries(
      [&](const FootprintCostFn & cost_big, const FootprintCostFn &) {
        if (have_path) {
          // 切出判据与切入判据问**同一个几何问题**，只是窗口更长（迟滞）。
          // 第二个 cost_fn 故意传空：这里只问"大足迹过不过得去"，
          // 传空后 previewNarrowStrategy 的第③支(小足迹)直接跳过。
          PrealignConfig exit_cfg = narrow_prealign_;
          exit_cfg.preview_m = narrow_square_exit_preview_m_;
          const StrategyPreview ep =
            previewNarrowStrategy(path, robot, robot_yaw, exit_cfg, cost_big, FootprintCostFn{});
          exit.from_path = true;
          // 只有 kNone(窗口内每一点按当前姿态都过得去) 与 kPathTooShort(前方没路了，
          // 通常已接近目标) 算"装得下"。
          // ⚠️ kAlignOnly 故意**不算**：它意味着大足迹还需要换个朝向才过得去，
          //    而此刻朝向锁在 square_locked_yaw_ 上，复原可能直接落进 253 带。
          exit.clear = (ep.strategy == NarrowStrategy::kNone ||
            ep.strategy == NarrowStrategy::kPathTooShort);
        } else {
          const double big =
            cost_big(pose.pose.position.x, pose.pose.position.y, robot_yaw);
          exit.clear = big < narrow_trigger_.footprint_lethal_threshold;
        }
      });
    if (!got) {
      // 代价地图不可用时**不复原**：复原需要知道大足迹装不装得下，而这正是
      // 现在读不到的量。按"还在窄处"处理，让硬超时兜底。
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3000,
        "[%s] 小足迹生效期间代价地图不可用，无法判复原条件，等硬超时兜底",
        name_.c_str());
      exit.clear = false;
      exit.from_path = false;
    }
    return evaluateSquare(pose, now, StrategyPreview{}, robot_yaw, exit);
  }

  // ---- 已在预对齐中：判完成 / 判超时 / 继续转 ----
  if (prealign_active_) {
    const double err = shortestAngularDiff(robot_yaw, prealign_target_yaw_);
    if (std::fabs(err) <= narrow_limits_.yaw_gate_rad) {
      // 退出判据复用朝向闸门，不新增第二个朝向阈值。
      ++prealign_succeeded_;
      prealign_active_ = false;
      RCLCPP_INFO(
        logger_,
        "[%s] 进入前预对齐完成(第 %d 次): 朝向误差 %.3frad <= 闸门 %.3frad，交回内层跟踪 "
        "| 累计[起N=%d 成N=%d 超N=%d 扫拒N=%d 限N=%d 无余量N=%d]",
        name_.c_str(), prealign_used_, std::fabs(err), narrow_limits_.yaw_gate_rad,
        prealign_triggered_, prealign_succeeded_, prealign_timeout_,
        prealign_sweep_blocked_, prealign_capped_, prealign_blocked_even_favorable_);
      return none;
    }
    if ((now - prealign_started_).seconds() > narrow_prealign_timeout_sec_) {
      // 🔴 显式失败，但**不抛异常**：预对齐失败是「没能开始」，不是「没穿过去」。
      // 计入 narrow_max_engagements 会误杀整条本来可行的路径（见本文件 :725 的教训）。
      ++prealign_timeout_;
      prealign_active_ = false;
      RCLCPP_WARN(
        logger_,
        "[%s] 进入前预对齐超时 %.1fs 放弃：还差 %.3frad 没转到位（目标 %.3frad）。"
        "交回内层跟踪，本次**不计入**放弃上限 | 累计超时=%d",
        name_.c_str(), narrow_prealign_timeout_sec_, std::fabs(err),
        prealign_target_yaw_, prealign_timeout_);
      return none;
    }
    none.take_over = true;
    none.cmd = rotateOnly(err, pose.header);
    return none;
  }

  // ---- 尚未预对齐：看前方要不要 ----
  if (prealign_used_ >= narrow_prealign_max_per_goal_) {
    if (!warned_prealign_cap_) {
      warned_prealign_cap_ = true;
      ++prealign_capped_;
      RCLCPP_WARN(
        logger_,
        "[%s] 进入前预对齐已达本目标上限 %d 次，后续不再预对齐（防振荡）",
        name_.c_str(), narrow_prealign_max_per_goal_);
    }
    return none;
  }

  std::vector<PlanarPoint> path;
  if (!planInCostmapFrame(path)) {
    return none;      // 缺数据不动作。这条路径 evaluateNarrow 已经告警过，不重复刷。
  }
  const PlanarPoint robot{pose.pose.position.x, pose.pose.position.y};

  StrategyPreview preview;
  bool sweep_clear = false;
  double sweep_worst = 0.0;
  double big_cost = 0.0;
  if (!runPrealignQueries(path, robot, robot_yaw, preview, sweep_clear, sweep_worst, big_cost)) {
    return none;      // 代价地图不可用
  }

  switch (preview.strategy) {
    case NarrowStrategy::kAlignOnly:
      break;          // 落到下面的预对齐流程

    case NarrowStrategy::kAlignThenShrink:
      // 大足迹转正也过不去、小足迹转正过得去 ⇒ 交给缩足迹状态机
      //（它内部会先要求对齐到位才切）。扫掠校验不过就不许转，同预对齐。
      //
      // 每拍都计数：这是**本功能有用武之地的拍数**，与 kBlocked 的拍数
      // 直接可比。缺了它就只能拿"节流 INFO 的行数"去比"每拍累加的计数器"，
      // 那个比值是错的（第一轮就是 1 行 vs 790 拍）。
      ++square_applicable_ticks_;
      if (!sweep_clear) {
        ++prealign_sweep_blocked_;
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 3000,
          "[%s] 前方 %.2fm 处需缩足迹，但旋转扫掠途中足迹代价达 %.0f(阈值 %.0f) ⇒ "
          "放弃预对齐，不硬转 | 累计扫掠被拒=%d",
          name_.c_str(), preview.at_distance_m, sweep_worst,
          narrow_trigger_.footprint_lethal_threshold, prealign_sweep_blocked_);
        return none;
      }
      return evaluateSquare(pose, now, preview, robot_yaw, SquareExitProbe{});

    case NarrowStrategy::kBlocked:
      // 前方连**小足迹**转正都过不去 ⇒ 不是本层能解决的，交给红线/ESCAPE。
      ++prealign_blocked_even_favorable_;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "[%s] 前视发现窄处**转正也过不去**（扫 %zu 点，缩足迹%s），预对齐不适用，"
        "由红线/ESCAPE 处理 | 累计=%d，同期「缩足迹能过」=%d 拍"
        "（两者都是逐拍计数，可直接相比；这个比值决定缩足迹在本图上有没有用武之地）",
        name_.c_str(), preview.samples,
        narrow_square_enabled_ ? "也不行" : "未启用", prealign_blocked_even_favorable_,
        square_applicable_ticks_);
      return none;

    case NarrowStrategy::kInvalidConfig:
      // 参数已在启动时校验过，走到这里说明代码/参数不一致，必须吼出来。
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 5000,
        "[%s] 预对齐参数非法（preview=%.2f step=%.2f 切向前视=%.2f 周期=%.4f）——"
        "启动校验与运行期判据不一致，预对齐已停止工作",
        name_.c_str(), narrow_prealign_.preview_m, narrow_prealign_.sample_step_m,
        narrow_prealign_.tangent_lookahead_m, narrow_prealign_.favorable_period_rad);
      return none;

    case NarrowStrategy::kPathTooShort:
    case NarrowStrategy::kNone:
    default:
      return none;      // 开阔或数据不足，正常放行给内层
  }

  // 到这里 strategy 必为 kAlignOnly
  if (!sweep_clear) {
    // 🔴 安全：转过去的途中会扫到膨胀带/真障碍 ⇒ 放弃预对齐，绝不硬转。
    ++prealign_sweep_blocked_;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "[%s] 前方 %.2fm 处需预对齐到 %.3frad，但旋转扫掠途中足迹代价达 %.0f（阈值 %.0f）"
      "⇒ 放弃预对齐，不硬转 | 累计扫掠被拒=%d",
      name_.c_str(), preview.at_distance_m, preview.target_yaw, sweep_worst,
      narrow_trigger_.footprint_lethal_threshold, prealign_sweep_blocked_);
    return none;
  }

  prealign_active_ = true;
  prealign_target_yaw_ = preview.target_yaw;
  prealign_started_ = now;
  ++prealign_used_;
  ++prealign_triggered_;
  const double err = shortestAngularDiff(robot_yaw, prealign_target_yaw_);
  RCLCPP_WARN(
    logger_,
    "[%s] 进入前预对齐启动(第 %d/%d 次): 前方 %.2fm 处当前朝向过不去、转到 %.3frad 过得去；"
    "原地转 %.3frad（扫掠最大代价 %.0f < %.0f）| 前视扫 %zu 点(跳过 %zu) 超时 %.1fs",
    name_.c_str(), prealign_used_, narrow_prealign_max_per_goal_, preview.at_distance_m,
    prealign_target_yaw_, err, sweep_worst, narrow_trigger_.footprint_lethal_threshold,
    preview.samples, preview.skipped_no_tangent, narrow_prealign_timeout_sec_);

  none.take_over = true;
  none.cmd = rotateOnly(err, pose.header);
  return none;
}

void ThreePhaseController::checkPeriodPreservesLateral(
  const std::vector<geometry_msgs::msg::Point> & fp, const char * what) const
{
  // 🔴 校验「按 narrow_favorable_period 取模」是否**保侧向包络**。
  //
  //    2026-09-03 实测事故：共享 pi/4（八边形的对称周期）时，正方形会被算到
  //    "有利朝向 = 通道+45°"，那是它的**对角朝墙**：
  //        面朝墙 0.3100  /  对角朝墙 0.4384（比八边形的 0.4200 还宽）
  //    于是"缩足迹"反而放大足迹，机器人转到最坏姿态再去挤窄处。
  //
  //    合法性不能靠"我觉得它对称"—— 这里实测采样 360 个朝向逐点比。
  //    容差按物理取 1mm：比栅格 0.05m 细 50 倍，又容得下 yaml 三位小数的舍入
  //   （八边形对角 0.297*sqrt2 比轴向 0.42 多 0.0214mm，是舍入不是形状差异；
  //     取 1e-9 会把它当差异而误报 —— 第一版测试就是这么失败的）。
  constexpr double kPeriodTolM = 1e-3;
  if (fp.size() < 3U) {
    // 🔴 不能静默通过：足迹拿不到就等于没校验，而调用方以为校验过了。
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: 校验朝向周期时 ") + what +
      " 少于 3 点，无法判定该周期是否保侧向包络 —— 拒绝启动，不接受未经校验的周期");
  }
  std::vector<PlanarPoint> pts;
  pts.reserve(fp.size());
  for (const auto & q : fp) {
    pts.push_back(PlanarPoint{q.x, q.y});
  }
  if (!periodPreservesLateralExtent(pts, narrow_favorable_period_rad_, kPeriodTolM)) {
    throw nav2_core::PlannerException(
      std::string("ThreePhaseController: narrow_favorable_period=") +
      std::to_string(narrow_favorable_period_rad_) + " 对" + what +
      "**不保侧向包络** —— 按它取模会把机器人转到侧向更宽的姿态"
      "（实测：正方形在 pi/4 同余类里对角朝墙 0.4384 > 面朝墙 0.3100，"
      "比八边形的 0.4200 还宽）。两种足迹统一取 pi/2。");
  }
}

void ThreePhaseController::loadFootprints(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & p)
{
  std::string s_default;
  std::string s_narrow;
  node->get_parameter(p + "narrow_footprint_default", s_default);
  node->get_parameter(p + "narrow_footprint_narrow", s_narrow);
  node->get_parameter(p + "chassis_min_envelope_radius", chassis_min_envelope_radius_);

  // 解析用 nav2 自己的 makeFootprintFromString，不自己写 parser
  //（自己写的那份必然与 costmap 读同一个字符串的方式漂开）。
  if (!nav2_costmap_2d::makeFootprintFromString(s_default, footprint_default_) ||
    footprint_default_.size() < 3U)
  {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_footprint_default 解析失败或少于 3 点: '" + s_default + "'");
  }
  if (!nav2_costmap_2d::makeFootprintFromString(s_narrow, footprint_narrow_) ||
    footprint_narrow_.size() < 3U)
  {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_footprint_narrow 解析失败或少于 3 点: '" + s_narrow + "'");
  }

  // 半径也用 nav2 自己的函数算 —— 膨胀层用的就是它，两处各算一份必然漂开。
  // 注意：这里算的是**未加 padding** 的值；costmap 会再加 footprint_padding，
  // 实测 padding=0.01 时八边形内切 0.388->0.399、正方形 0.310->0.320。
  nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_default_, footprint_default_inscribed_, footprint_default_circumscribed_);
  nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_narrow_, footprint_narrow_inscribed_, footprint_narrow_circumscribed_);

  // 🔴 守卫 1：小足迹必须真的比默认足迹小，否则切了没有任何意义（还白付风险）。
  if (!(footprint_narrow_inscribed_ < footprint_default_inscribed_)) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_footprint_narrow 的内切半径(" +
      std::to_string(footprint_narrow_inscribed_) + ") 必须小于默认足迹的(" +
      std::to_string(footprint_default_inscribed_) + ")，否则切换毫无收益");
  }
  // 🔴 守卫 2：小足迹不得小于底盘物理包络。打错一个字就可能把机器人
  //    建模成比躯干还小，而这种错**在仿真里表现为"通过率提高"**，非常危险。
  if (footprint_narrow_inscribed_ < chassis_min_envelope_radius_) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_footprint_narrow 内切半径(" +
      std::to_string(footprint_narrow_inscribed_) + ") < chassis_min_envelope_radius(" +
      std::to_string(chassis_min_envelope_radius_) +
      ")，这会把机器人建模成比真实底盘还小 —— 拒绝启动");
  }
  // 🔴 守卫 3：切出窗口必须 >= 切入窗口。反了就**保证**振荡：
  //    切入看 1.0m 内有麻烦、切出看 0.6m 内没麻烦，机器人在 0.6~1.0m 之间
  //    两个判据同时成立 ⇒ 切入、复原、切入…（第一轮实测 123 次切换的同类病因）。
  if (narrow_square_exit_preview_m_ < narrow_prealign_.preview_m) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_square_exit_preview(" +
      std::to_string(narrow_square_exit_preview_m_) + ") < narrow_prealign_preview(" +
      std::to_string(narrow_prealign_.preview_m) +
      ")，切出窗口比切入窗口短会保证切换振荡 —— 拒绝启动");
  }
  // 🔴 守卫 4：切出窗口必须容得下至少一个采样步长，否则 previewNarrowStrategy
  //    直接返回 kInvalidConfig，切出判据永远不成立、只能等硬超时。
  if (narrow_square_exit_preview_m_ < narrow_prealign_.sample_step_m) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_square_exit_preview(" +
      std::to_string(narrow_square_exit_preview_m_) + ") < 采样步长(" +
      std::to_string(narrow_prealign_.sample_step_m) + ")，切出判据永远无法成立");
  }
  // 🔴 守卫 5：最短驻留必须显著小于硬超时，否则"最短驻留"会吃掉整个
  //    正常复原通路，让每次都走硬超时那条异常路径。
  if (!(narrow_square_min_dwell_sec_ >= 0.0) ||
    narrow_square_min_dwell_sec_ >= narrow_square_hard_timeout_sec_)
  {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_square_min_dwell(" +
      std::to_string(narrow_square_min_dwell_sec_) + ") 必须 >=0 且 < narrow_square_hard_timeout(" +
      std::to_string(narrow_square_hard_timeout_sec_) + ")");
  }
  if (!(narrow_square_cooldown_sec_ >= 0.0)) {
    throw nav2_core::PlannerException("ThreePhaseController: narrow_square_cooldown 必须 >= 0");
  }
  if (narrow_square_exit_ticks_ < 1) {
    throw nav2_core::PlannerException("ThreePhaseController: narrow_square_exit_ticks 必须 >= 1");
  }
  // 🔴 守卫 6.5：缩足迹**依赖**预对齐。唯一的切入通路是预对齐的前视判定
  //    （顺序不可颠倒：先对齐、后换足迹），所以预对齐关着时缩足迹永远进不去。
  //    而它进不去的表现是**切换 0 次、零告警** —— A/B 会得出"这个机制没用"，
  //    真相是它一次都没跑。这正是"禁止静默失败"要拦的东西，必须启动即拒绝。
  if (!narrow_prealign_enabled_) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_square_enabled=true 但 narrow_prealign_enabled=false。"
      "缩足迹的唯一切入通路是预对齐的前视判定（先对齐后换足迹），"
      "预对齐关着时它永远不会生效且不会报错 —— 拒绝启动，不接受静默无效的配置");
  }
  // 🔴 守卫 6：续租周期必须显著小于回读验证超时，否则"等确认"期间可能一次都没续租。
  if (!(narrow_square_lease_period_sec_ < narrow_square_verify_timeout_sec_)) {
    throw nav2_core::PlannerException(
      "ThreePhaseController: narrow_square_lease_period(" +
      std::to_string(narrow_square_lease_period_sec_) + ") 必须 < narrow_square_verify_timeout(" +
      std::to_string(narrow_square_verify_timeout_sec_) + ")");
  }

  // 🔴 守卫 7：朝向等价周期必须对**两种**足迹都保侧向包络。
  //    放在这里而不是 declareAndLoadParams()：那里跑在本函数之前，
  //    footprint_* 还是空的，校验会因 size<3 静默跳过。
  checkPeriodPreservesLateral(footprint_default_, "默认(八边形)足迹");
  checkPeriodPreservesLateral(footprint_narrow_, "窄通道(正方形)足迹");

  // 🔴 守卫 8：在**对齐后的目标朝向**上，小足迹的侧向半宽必须真的更小。
  //
  //    这是从根上挡住"切了反而更宽"。2026-09-03 实测过：共享 pi/4 周期时
  //    正方形会被算到对角朝墙 0.4384，比八边形的 0.4200 还宽 —— 那次
  //    没有任何一条判据能拦住它，因为所有判据都在问"过不过得去"，
  //    没有一条在问"换了之后是不是真的更窄"。
  //
  //    对齐后 delta = 0（朝向 = 通道方向），所以直接比 delta=0 处的半宽。
  {
    std::vector<PlanarPoint> big;
    std::vector<PlanarPoint> small;
    big.reserve(footprint_default_.size());
    small.reserve(footprint_narrow_.size());
    for (const auto & q : footprint_default_) {big.push_back(PlanarPoint{q.x, q.y});}
    for (const auto & q : footprint_narrow_) {small.push_back(PlanarPoint{q.x, q.y});}
    footprint_default_lateral_ = lateralHalfExtent(big, 0.0);
    footprint_narrow_lateral_ = lateralHalfExtent(small, 0.0);
    if (!(footprint_narrow_lateral_ < footprint_default_lateral_)) {
      throw nav2_core::PlannerException(
        "ThreePhaseController: 对齐到通道方向后，小足迹侧向半宽(" +
        std::to_string(footprint_narrow_lateral_) + ") 不小于大足迹(" +
        std::to_string(footprint_default_lateral_) +
        ") —— 切换毫无收益甚至更宽，拒绝启动。通道宽度只取决于侧向半宽，"
        "不是内切/外接半径。");
    }
  }

  RCLCPP_INFO(
    logger_,
    "[%s] 窄通道缩足迹已启用: 默认足迹 %zu 点(内切 %.4f 外接 %.4f) -> "
    "窄通道足迹 %zu 点(内切 %.4f 外接 %.4f) | 朝向盲区 %.4f -> %.4f "
    "| 最窄通道 %.3fm -> %.3fm | 底盘包络下限 %.3f 硬超时 %.1fs "
    "(以上均**未含** costmap 的 footprint_padding)",
    name_.c_str(),
    footprint_default_.size(), footprint_default_inscribed_, footprint_default_circumscribed_,
    footprint_narrow_.size(), footprint_narrow_inscribed_, footprint_narrow_circumscribed_,
    footprint_default_circumscribed_ - footprint_default_inscribed_,
    footprint_narrow_circumscribed_ - footprint_narrow_inscribed_,
    2.0 * footprint_default_inscribed_, 2.0 * footprint_narrow_inscribed_,
    chassis_min_envelope_radius_, narrow_square_hard_timeout_sec_);
  RCLCPP_INFO(
    logger_,
    "[%s] 缩足迹切换迟滞: 切入窗口 %.2fm / 切出窗口 %.2fm(迟滞带 %.2fm) "
    "| 切出需连续 %d 拍 | 最短驻留 %.1fs | 复原冷却 %.1fs "
    "⇒ 单次切换周期下限 %.1fs（用它反推的切换次数上限即为看门狗与 A/B 的判据）",
    name_.c_str(), narrow_prealign_.preview_m, narrow_square_exit_preview_m_,
    narrow_square_exit_preview_m_ - narrow_prealign_.preview_m,
    narrow_square_exit_ticks_, narrow_square_min_dwell_sec_, narrow_square_cooldown_sec_,
    narrow_square_min_dwell_sec_ + narrow_square_cooldown_sec_);
}

void ThreePhaseController::requestFootprint(const std::vector<geometry_msgs::msg::Point> & fp)
{
  // local costmap 同进程，直接调 —— 有返回保障，不依赖话题。
  if (costmap_ros_) {
    costmap_ros_->setRobotFootprint(fp);
  }
  // global costmap 在 planner_server 进程里，只能发话题（fire-and-forget）。
  // 所以调用方**必须**再走 footprintVerified() 回读确认。
  if (global_footprint_pub_) {
    geometry_msgs::msg::Polygon msg;
    msg.points.reserve(fp.size());
    for (const auto & pt : fp) {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(pt.x);
      p32.y = static_cast<float>(pt.y);
      p32.z = 0.0F;
      msg.points.push_back(p32);
    }
    global_footprint_pub_->publish(msg);
  }
}

bool ThreePhaseController::footprintVerified(std::size_t want_vertices) const
{
  return global_footprint_vertices_.load() == static_cast<int>(want_vertices);
}

void ThreePhaseController::revertFootprint(const char * why)
{
  if (!square_active_ && !square_pending_) {
    return;                         // 幂等：没生效就什么都不做
  }
  const double active_s = square_active_ ?
    (clock_->now() - square_activated_).seconds() : 0.0;
  square_active_ = false;
  square_pending_ = false;
  square_exit_clear_hits_ = 0;
  // 朝向层迟滞的锁存位必须一起清：留着它，下次切入时会带着上一次的
  // "正在重新对齐"状态进来，于是**跳过** yaw_gate 判定直奔 yaw_resume，
  // 那等于把切入阈值悄悄换成了切出阈值。
  square_realigning_ = false;
  // 开启冷却期：防"复原-立刻再切"自激。安全通路（deactivate/cleanup/新目标）
  // 走到这里也会开冷却，那是对的 —— 那些场景下也不该马上再缩。
  if (clock_) {
    square_cooldown_started_ = clock_->now();
    square_cooldown_valid_ = true;
  }
  requestFootprint(footprint_default_);
  RCLCPP_WARN(
    logger_,
    "[%s] 窄通道足迹已复原为默认(%zu 点，内切 %.4f)：%s | 本次小足迹生效 %.1fs "
    "冷却 %.1fs | 累计[切换=%d 回读失败=%d 复原(装得下)=%d 复原(超时)=%d 重新对齐=%d "
    "驻留压住=%d 冷却挡掉=%d 切出退化=%d 可用拍=%d]",
    name_.c_str(), footprint_default_.size(), footprint_default_inscribed_, why, active_s,
    narrow_square_cooldown_sec_,
    square_switch_count_, square_verify_fail_count_, square_revert_cleared_,
    square_revert_timeout_, square_realign_count_,
    square_suppressed_dwell_, square_suppressed_cooldown_, square_exit_degraded_,
    square_applicable_ticks_);
}

void ThreePhaseController::renewFootprintLease(const rclcpp::Time & now)
{
  if (!narrow_square_enabled_ || (!square_active_ && !square_pending_)) {
    return;
  }
  if ((now - square_last_lease_).seconds() < narrow_square_lease_period_sec_) {
    return;
  }
  square_last_lease_ = now;
  requestFootprint(footprint_narrow_);
}

bool ThreePhaseController::withCostQueries(
  const std::function<void(const FootprintCostFn &, const FootprintCostFn &)> & fn)
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
    return false;      // 退化足迹算不出包络，宁可什么都不做
  }
  if (!collision_checker_) {
    collision_checker_ = std::make_unique<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap);
  } else {
    collision_checker_->setCostmap(costmap);
  }

  // 🔴 一次持锁做完全部查询。每次单独加锁会在 20Hz 下产生大量锁竞争；
  //    更要紧的是**绝不能**握着这把锁去调内层控制器（MPPI 也要拿它 ⇒ 必死的死锁）。
  //    所以本函数只把查询能力交给 fn，出了这个作用域调用方才决定动作。
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // 默认足迹：缩足迹功能开着时用 yaml 里那份显式的大足迹（因为此刻 costmap 上
  // 挂的可能已经是小足迹了）；关着时就用 costmap 当前足迹。
  const std::vector<geometry_msgs::msg::Point> & big =
    (narrow_square_enabled_ && footprint_default_.size() >= 3U) ?
    footprint_default_ : footprint_cache_;

  const FootprintCostFn cost_big = [this, &big](double x, double y, double yaw) {
      return collision_checker_->footprintCostAtPose(x, y, yaw, big);
    };
  FootprintCostFn cost_small{};
  if (narrow_square_enabled_ && footprint_narrow_.size() >= 3U) {
    cost_small = [this](double x, double y, double yaw) {
        return collision_checker_->footprintCostAtPose(x, y, yaw, footprint_narrow_);
      };
  }
  fn(cost_big, cost_small);
  return true;
}

bool ThreePhaseController::runPrealignQueries(
  const std::vector<PlanarPoint> & path,
  const PlanarPoint & robot,
  double robot_yaw,
  StrategyPreview & preview,
  bool & sweep_clear,
  double & sweep_worst_cost,
  double & big_footprint_cost)
{
  bool ok_sweep = false;
  double worst = 0.0;
  double big_cost = 0.0;
  StrategyPreview p;
  const bool got = withCostQueries(
    [&](const FootprintCostFn & cost_big, const FootprintCostFn & cost_small) {
      big_cost = cost_big(robot.x, robot.y, robot_yaw);
      p = previewNarrowStrategy(path, robot, robot_yaw, narrow_prealign_, cost_big, cost_small);
      if (p.strategy == NarrowStrategy::kAlignOnly ||
        p.strategy == NarrowStrategy::kAlignThenShrink)
      {
        // 承诺旋转之前必须过扫掠校验：开阔处 footprint<253 只保证**当前这一个朝向**
        // 不碰带，原地转会把八边形顶点扫进膨胀带甚至真障碍。
        // 用**大足迹**校验：转的时候身上还是大足迹。
        ok_sweep = sweepClearForRotation(
          robot, robot_yaw, p.target_yaw, narrow_prealign_sweep_step_rad_,
          narrow_trigger_.footprint_lethal_threshold, cost_big, worst);
      }
    });
  preview = p;
  sweep_clear = ok_sweep;
  sweep_worst_cost = worst;
  big_footprint_cost = big_cost;
  return got;
}

ThreePhaseController::PrealignDecision ThreePhaseController::evaluateSquare(
  const geometry_msgs::msg::PoseStamped & pose,
  const rclcpp::Time & now,
  const StrategyPreview & preview,
  double robot_yaw,
  const SquareExitProbe & exit)
{
  PrealignDecision none;
  if (!narrow_square_enabled_) {
    return none;
  }
  // ⚠️ 续租**不在**这里。它必须每拍无条件跑（见 renewFootprintLease 的注释）：
  //    写在这里就会在 evaluateNarrow 接管时被 early-return 跳过，
  //    也就是恰好在最需要小足迹的时候停止续租。

  // ================= 已请求但还没验证通过 =================
  if (square_pending_) {
    if (footprintVerified(footprint_narrow_.size())) {
      square_pending_ = false;
      square_active_ = true;
      square_activated_ = now;
      square_last_lease_ = now;
      square_exit_clear_hits_ = 0;
      ++square_switch_count_;
      RCLCPP_WARN(
        logger_,
        "[%s] 窄通道小足迹**已回读确认生效**(第 %d 次): global costmap 顶点数=%zu "
        "内切 %.4f->%.4f | 朝向锁定 %.3frad 最短驻留 %.1fs 硬超时 %.1fs",
        name_.c_str(), square_switch_count_, footprint_narrow_.size(),
        footprint_default_inscribed_, footprint_narrow_inscribed_,
        square_locked_yaw_, narrow_square_min_dwell_sec_, narrow_square_hard_timeout_sec_);
      return none;                  // 本拍交回内层，开始通过
    }
    if ((now - square_requested_).seconds() > narrow_square_verify_timeout_sec_) {
      // 🔴 回读没确认就**绝不**假定切成功。复原并放弃本次。
      ++square_verify_fail_count_;
      revertFootprint("回读验证超时：global costmap 没确认切到小足迹");
      RCLCPP_ERROR(
        logger_,
        "[%s] 小足迹切换**回读失败** %.1fs 内 global costmap 未确认"
        "(期望顶点 %zu，实际 %d)。已复原，本次不缩足迹。"
        "若持续出现请查 /global_costmap/footprint 是否真的有订阅者 | 累计失败=%d",
        name_.c_str(), narrow_square_verify_timeout_sec_, footprint_narrow_.size(),
        global_footprint_vertices_.load(), square_verify_fail_count_);
      return none;
    }
    // 等待期间原地不动，别带着未确认的足迹往窄处走。
    none.take_over = true;
    none.cmd = rotateOnly(shortestAngularDiff(robot_yaw, square_locked_yaw_), pose.header);
    return none;
  }

  // ================= 小足迹已生效 =================
  if (square_active_) {
    const double dwell_s = (now - square_activated_).seconds();
    if (!exit.from_path) {
      ++square_exit_degraded_;
    }

    // ---- 复原判据 1：切出窗口内大足迹连续 N 拍装得下，且已过最短驻留 ----
    //
    // 这里的 exit.clear 是**前视窗口**的判定（exit_preview_m，比切入窗口长），
    // 不是"当前位姿装得下"。旧判据只看当前位姿，而当前位姿必然装得下
    //（否则 evaluateNarrow 早接管了），所以旧判据在切入那一瞬间就成立 ——
    // 实测 121/123 次复原都发生在第 3 拍(=0.15s)，与中位驻留完全吻合。
    if (exit.clear) {
      ++square_exit_clear_hits_;
    } else {
      square_exit_clear_hits_ = 0;
    }
    if (square_exit_clear_hits_ >= narrow_square_exit_ticks_) {
      if (dwell_s < narrow_square_min_dwell_sec_) {
        // 最短驻留把这条**正常**复原路径压住。只压这一条 ——
        // 下面的硬超时不受它限制，安全通路永远优先。
        ++square_suppressed_dwell_;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 2000,
          "[%s] 切出判据已连续 %d 拍成立，但驻留仅 %.2fs < 最短 %.1fs ⇒ 暂不复原"
          "（防 0.15s 级抖动）| 累计被压住=%d",
          name_.c_str(), square_exit_clear_hits_, dwell_s, narrow_square_min_dwell_sec_,
          square_suppressed_dwell_);
      } else {
        ++square_revert_cleared_;
        revertFootprint(
          exit.from_path ?
          "切出窗口内大足迹已连续装得下" :
          "退化判据(仅当前位姿)下大足迹已连续装得下");
        return none;
      }
    }

    // ---- 复原判据 2：硬超时（红线：受限动作必须带超时）----
    // ⚠️ 故意放在最短驻留**之后**且不受它约束：它是安全兜底，不是调优项。
    if (dwell_s > narrow_square_hard_timeout_sec_) {
      ++square_revert_timeout_;
      revertFootprint("小足迹硬超时");
      RCLCPP_WARN(
        logger_,
        "[%s] 小足迹生效已超 %.1fs 仍未脱离窄处 ⇒ 强制复原。"
        "复原后机器人可能落在膨胀带内，由上层重规划处理 | 累计超时复原=%d",
        name_.c_str(), narrow_square_hard_timeout_sec_, square_revert_timeout_);
      return none;
    }

    // ---- 失去对齐：**保持小足迹**，平移置零、重新对齐 ----
    // 这是刻意的反直觉设计（已与用户确认）：复原足迹会让机器人瞬间落在 253 带里，
    // 而 `状态 -> ESCAPE` 实测 10 轮全为 0（ESCAPE 从不执行）——复原会困得更死。
    // 安全性由另一条兜底：MPPI 两个 critic 都是 consider_footprint:true、
    // 用真实多边形在真实位姿求值，且 254 红线不动。
    //
    // 🔴 迟滞：切入用 yaw_gate_rad、**切出用更严的 yaw_resume_rad**。
    // 两侧同一个数就是没有迟滞，保证在闸门上自激 —— 实测 749 次重新对齐、
    // 30s 全耗在原地摆头、最后被硬超时踢回八边形（"能进不能出"）。
    // 判据必须两侧同源同几何：这里两侧都是「robot_yaw 与 square_locked_yaw_ 的
    // 最短角差」，只有阈值不同。
    const double err = shortestAngularDiff(robot_yaw, square_locked_yaw_);
    if (square_realigning_) {
      // 已在接管旋转中：必须转到明显好于闸门才交回，否则继续转
      if (std::fabs(err) > narrow_limits_.yaw_resume_rad) {
        none.take_over = true;
        none.cmd = rotateOnly(err, pose.header);
        return none;
      }
      square_realigning_ = false;
      RCLCPP_INFO(
        logger_, "[%s] 重新对齐完成(误差 %.3frad <= 交回阈值 %.3frad) ⇒ 交回内层",
        name_.c_str(), std::fabs(err), narrow_limits_.yaw_resume_rad);
      return none;
    }
    if (std::fabs(err) > narrow_limits_.yaw_gate_rad) {
      ++square_realign_count_;
      square_realigning_ = true;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[%s] 小足迹生效期间失去对齐(误差 %.3frad > 闸门 %.3frad) ⇒ "
        "**保持小足迹**、平移置零、重新对齐到 %.3frad 才交回"
        "（复原足迹会把车困在膨胀带里）| 累计=%d",
        name_.c_str(), std::fabs(err), narrow_limits_.yaw_gate_rad,
        narrow_limits_.yaw_resume_rad, square_realign_count_);
      none.take_over = true;
      none.cmd = rotateOnly(err, pose.header);
      return none;
    }
    // ---- 对齐良好：交回**贴边通行层**继续推进，而不是交回内层 MPPI ----
    // 🔴 这里返回 take_over=false 只表示"本层（缩足迹层）这一拍没有指令"，
    // 真正接管的是 evaluateNarrow 的贴边通行 —— 它有横向扫描寻优、饱和检测、
    // 偏离路径保护和物理堵死判据，是本仓库既有的实现，不要在这里另写一份。
    //
    // 为什么必须由贴边通行层走完整个小足迹区间、不能交回 MPPI：
    // 小足迹生效的区间按定义就是"窄到八边形过不去"，而这种地方 MPPI 的代价场
    // 是**饱和**的。膨胀层的 inscribed_radius_ 随运行时换足迹一起变成正方形的
    // 0.32，多边形最外侧格到墙的距离 W/2-0.32 <= 0.32 时整个多边形恒 253：
    //     零梯度阈值 W <= 4 x 0.32 = 1.28m
    // 而缩足迹的用武之地本来就是 0.64 < W <= 0.86m —— 远小于 1.28m。
    // 即缩足迹把零梯度阈值从八边形的 1.72m 降到 1.28m，**没降到通道宽度以下**。
    // 实测（0.65~0.86m 通道）：costmap_raw 剖面 22 点里 253 占 15 个(68%)；
    // 交回 MPPI 的后果是累计行程 1.706m / 净位移 0.488m、|vx| 中位 0.0134、
    // 近零帧 46.4% —— 原地来回蹭到撞满 30s 硬超时（实测 11 次切换里 4 次）。
    // 恒 253 意味着"哪都一样坏"，没有方向信息可优化；方向只能来自**路径**。
    //
    // 联动（关键）：evaluateNarrow 开头有一条"便宜早退" ——
    // 中心格与足迹代价都低于阈值就直接放行。小足迹装得下之后 footprint_cost
    // 正好从 254 掉到 253、低于 254 阈值 ⇒ 早退 ⇒ 谁都不接管 ⇒ 落回 MPPI。
    // 所以那条早退必须加一个"小足迹生效期间不早退"的例外，否则本注释里的
    // 交接根本不会发生。该例外由 squareActive() 提供。
    return none;
  }

  // ================= 尚未请求：看要不要切 =================
  if (preview.strategy != NarrowStrategy::kAlignThenShrink) {
    return none;
  }
  // ---- 冷却期：刚复原过就不许马上再切 ----
  if (square_cooldown_valid_) {
    const double since = (now - square_cooldown_started_).seconds();
    if (since < narrow_square_cooldown_sec_) {
      ++square_suppressed_cooldown_;
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 2000,
        "[%s] 前方 %.2fm 处需缩足迹，但距上次复原仅 %.2fs < 冷却 %.1fs ⇒ 本次不切"
        "（防复原后立刻再切的自激）| 累计被挡=%d",
        name_.c_str(), preview.at_distance_m, since, narrow_square_cooldown_sec_,
        square_suppressed_cooldown_);
      return none;
    }
  }
  // 🔴 顺序不可颠倒：**必须先对齐到位**才允许缩足迹。
  //    小足迹把代价地图的朝向盲区放大到 0.133m，朝向没锁住就切等于让
  //    代价地图在一个它无法表达的姿态上说"可以过"。
  const double err = shortestAngularDiff(robot_yaw, preview.target_yaw);
  if (std::fabs(err) > narrow_limits_.yaw_gate_rad) {
    none.take_over = true;
    none.cmd = rotateOnly(err, pose.header);
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] 前方 %.2fm 处需缩足迹才过得去，先原地对齐到 %.3frad（还差 %.3frad）",
      name_.c_str(), preview.at_distance_m, preview.target_yaw, std::fabs(err));
    return none;
  }

  // 🔴 运行期断言：在**这个具体的目标朝向**上，小足迹必须真的更窄。
  //    启动守卫比的是 delta=0（理想对齐后）的半宽；这里比的是本次真正要
  //    锁定的朝向 —— 两者会不一致（朝向闸门允许 yaw_gate_rad 的偏差，
  //    而侧向半宽随 delta 变化）。事故当天缺的正是这一条：所有判据都在问
  //    "过不过得去"，没有一条在问"换了之后是不是真的更窄"。
  {
    const double delta = shortestAngularDiff(preview.target_yaw, preview.corridor_heading);
    std::vector<PlanarPoint> big;
    std::vector<PlanarPoint> small;
    big.reserve(footprint_default_.size());
    small.reserve(footprint_narrow_.size());
    for (const auto & q : footprint_default_) {big.push_back(PlanarPoint{q.x, q.y});}
    for (const auto & q : footprint_narrow_) {small.push_back(PlanarPoint{q.x, q.y});}
    const double lat_big = lateralHalfExtent(big, delta);
    const double lat_small = lateralHalfExtent(small, delta);
    if (!(lat_small < lat_big)) {
      ++square_refused_wider_;
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 3000,
        "[%s] 🔴 拒绝缩足迹：在目标朝向 %.3frad(相对通道 %.3frad)上，"
        "小足迹侧向半宽 %.4f **不小于** 大足迹 %.4f —— 切了会更宽，不是更窄。"
        "这通常意味着 narrow_favorable_period 配错（正方形必须 pi/2，"
        "pi/4 会把它转到对角朝墙 0.4384 > 面朝墙 0.3100）| 累计拒绝=%d",
        name_.c_str(), preview.target_yaw, delta, lat_small, lat_big,
        square_refused_wider_);
      return none;
    }
  }

  square_locked_yaw_ = preview.target_yaw;
  square_pending_ = true;
  square_requested_ = now;
  // ⚠️ 续租时刻必须在这里就初始化：renewFootprintLease 从下一拍起就会读它，
  //    而默认构造的 rclcpp::Time 是 RCL_SYSTEM_TIME，与节点时钟相减会**抛异常**。
  square_last_lease_ = now;
  requestFootprint(footprint_narrow_);
  RCLCPP_WARN(
    logger_,
    "[%s] 朝向已对齐(误差 %.3frad)，请求缩足迹: %zu 点 -> %zu 点，内切 %.4f -> %.4f "
    "(最窄通道 %.3fm -> %.3fm)。等待 global costmap 回读确认，最多 %.1fs",
    name_.c_str(), std::fabs(err), footprint_default_.size(), footprint_narrow_.size(),
    footprint_default_inscribed_, footprint_narrow_inscribed_,
    2.0 * footprint_default_inscribed_, 2.0 * footprint_narrow_inscribed_,
    narrow_square_verify_timeout_sec_);
  none.take_over = true;
  none.cmd = rotateOnly(err, pose.header);   // 等确认期间保持朝向、不前进
  return none;
}

void ThreePhaseController::disengageNarrow(const char * why)
{
  if (narrow_engaged_) {
    RCLCPP_INFO(logger_, "[%s] 窄通道接管退出：%s", name_.c_str(), why);
  }
  narrow_engaged_ = false;
  narrow_hits_ = 0;
  narrow_clear_hits_ = 0;
  narrow_via_saturation_ = false;
  // 🔴 脱离时必须清空无进展窗口。留着旧锚点会让下一次刚进饱和带就
  //    立刻满足"3s 没动"（锚点时刻是上一次的），等于第二入口没有消抖。
  narrow_stall_ = NarrowStallState{};
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

  // ---- 「代价场饱和 + 内层无进展」检测：必须跑在早退**之前** ----
  // 放在早退之后就永远跑不到 —— 早退命中的那些拍正是要观察的那些拍。
  // 本函数只在 Phase::kFollow 里被调用，所以 following 恒为 true；
  // 相位切换时窗口由 disengageNarrow / 新目标处清空。
  const bool saturated = footprint_cost >= narrow_trigger_.saturation_threshold;
  const bool saturated_stalled = updateSaturationStall(
    saturated, /*following=*/ true,
    pose.pose.position.x, pose.pose.position.y, now.seconds(),
    narrow_trigger_, narrow_stall_);
  if (saturated_stalled && !narrow_engaged_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] 代价场饱和(足迹 %.0f >= %.0f)且内层 %.1fs 内位移 < %.3fm ⇒ "
      "由第二入口接管、沿路径方向走。这一类通道 254 永不出现，"
      "只有实测无进展能测到",
      name_.c_str(), footprint_cost, narrow_trigger_.saturation_threshold,
      narrow_trigger_.saturation_stall_sec, narrow_trigger_.saturation_min_move_m);
  }

  // 🔴 未接管时也必须periodically报出这三个数，否则"该接管却没接管"完全不可观测。
  //    实测教训：机器人在通道口 (2.12,-5.80) 静止了 **90s**（净位移 0.041m、
  //    离锚点最大偏移 0.041m），期间控制器一个字都没打，第二入口直到 t+108s 才进。
  //    我拿外部探针读 /global_costmap/costmap_raw 得到"足迹 253 持续 90s"，
  //    据此以为两个合取项都成立 —— 但 readCosts 读的是 **controller 自己的
  //    costmap_ros_，即 local costmap**，与全局图不是同一张。判据用哪张图，
  //    诊断就必须报哪张图的数，否则永远在对着错的数推理。
  if (!narrow_engaged_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] 未接管诊断(local costmap): 足迹=%.0f(阈值 %.0f ⇒ 饱和=%s) 中心=%.0f "
      "无进展窗口: 有锚点=%s 已持续=%.2fs(阈值 %.1fs) 判据成立=%s",
      name_.c_str(), footprint_cost, narrow_trigger_.saturation_threshold,
      saturated ? "是" : "否", center_cost,
      narrow_stall_.has_anchor ? "是" : "否",
      narrow_stall_.has_anchor ? (now.seconds() - narrow_stall_.anchor_sec) : 0.0,
      narrow_trigger_.saturation_stall_sec,
      saturated_stalled ? "是" : "否");
  }

  // ---- 便宜的早退：开阔处直接放行，不去付路径变换/通道方向的开销 ----
  // 中心格致命与朝向无关，也在这里判掉。
  //
  // 🔴 例外一：**小足迹生效期间绝不早退**。
  // 缩足迹成功之后 footprint_cost 正好从 254 掉到 253（这就是"装得下了"的
  // 含义），于是它 < 254 阈值、这条早退命中、本层放行、控制权落回内层 MPPI。
  // 而小足迹生效的区间按定义就是"窄到八边形过不去"，那里 MPPI 的代价场是
  // **饱和**的：膨胀层 inscribed_radius_ 随换足迹变成正方形的 0.32，
  // 多边形最外侧格到墙 W/2-0.32 <= 0.32 时整个多边形恒 253，
  //     零梯度阈值 W <= 4 x 0.32 = 1.28m
  // 而用武之地是 0.64 < W <= 0.86m，远小于 1.28m —— 缩足迹把阈值从八边形的
  // 1.72m 降到 1.28m，没降到通道宽度以下。恒 253 = 哪都一样坏 = 无方向可优化。
  // 实测后果（0.65~0.86m 通道）：costmap_raw 剖面 253 占 68%，交回 MPPI 后
  // 累计行程 1.706m / 净位移 0.488m、|vx| 中位 0.0134、近零帧 46.4%，
  // 11 次切换里 4 次撞满 30s 硬超时。方向信息只能来自**路径** ——
  // 所以这一段必须由贴边通行层用 corridorHeadingFromPath 沿路径方向走完。
  //
  // 🔴 例外二：**饱和且内层实测无进展时不早退**（第二入口，2026-09-03 实测加上）。
  // 例外一只覆盖"已经切过小足迹"的情形。而实测发现整整一类通道连第一次
  // 切换都触发不了：八边形足迹恒 253、254 一次都不出现（净宽 1.05~1.80m，
  // 八边形只要 0.86m，本来就过得去），中心格最高 216 也够不着 253 ——
  // ①②④ 三条判据全不成立。挡路的是膨胀梯度本身而不是 253，
  // 详见 NarrowTriggerConfig::saturation_threshold 上方那段量级比较。
  //
  // 🔴 例外三（迟滞同源）：**由第二入口接管期间，脱离阈值也用 253 而不是 254**。
  // 否则接管的同一拍 footprint_cost(253) < 254 就成立，早退立刻命中，
  // 保证振荡 —— 迟滞两侧必须问同一个几何问题。
  const double clear_threshold =
    (narrow_via_saturation_ || saturated_stalled)
    ? narrow_trigger_.saturation_threshold
    : narrow_trigger_.footprint_lethal_threshold;
  // 🔴 例外四（迟滞同**变量**）：由第二入口接管期间，代价掉下 253 在最短驻留
  // 之内**不足以**退出。例外三只对齐了阈值，实测仍振荡 18 次：这一类通道的
  // 代价场是 229~253 的纹理而不是平的 253，机器人原地转的过程中当前位姿的
  // 八边形代价会自己掉到 233 < 253，于是 3 拍脱离命中、在**还没转正**时就
  // 交回 MPPI（13/18 次接管只活了 0.35~1.80s，而转正需要 1.5~3.3s）。
  // 入口问的是「有没有推进」，出口必须先给足能推进的时间再问代价。
  // ⚠️ narrow_started_ 未接管时是默认构造的 rclcpp::Time，时钟源是
  // RCL_SYSTEM_TIME，而 now 走 ROS 时钟（仿真下是 /clock）。两者相减会抛
  // "can't subtract times with different time sources [1 != 2]"。
  // 原来那处相减（绝对上限兜底）只在接管中执行，所以从没暴露；
  // 这条判据每拍都要问，必须先判 narrow_engaged_ 再读 narrow_started_。
  // 实测代价：漏了这一步，controller 每拍抛异常 -> follow_path 逐拍 Aborting，
  // 机器人停在 (-0.12,-0.31) 一步没走、24.3s 后 ABORTED，且零次接管。
  const double narrow_engaged_sec =
    narrow_engaged_ ? (now - narrow_started_).seconds() : 0.0;
  const bool saturation_dwell_holding = saturationDwellHolding(
    narrow_engaged_, narrow_via_saturation_,
    narrow_engaged_sec, narrow_saturation_min_dwell_sec_);
  if (!squareActive() && !saturation_dwell_holding &&
    center_cost < narrow_trigger_.center_lethal_threshold &&
    footprint_cost < clear_threshold)
  {
    // ⚠️ 计数只能有一处：接管中让下面 switch 的 kNone 分支去数（它问的是
    // footprint >= saturation_threshold，与本处 clear_threshold 在第二入口下
    // 恰好同值 253）。若两处都 ++，3 拍的脱离条件会在 1.5 拍就满足，
    // 迟滞带等于被砍掉一半 —— 这正是例外三/四想堵的那个洞。
    if (!narrow_engaged_) {
      ++narrow_clear_hits_;
      narrow_hits_ = 0;
      // 脱离窄通道即允许再次上报异常（若之后又遇到新的窄处）。
      narrow_threw_here_ = false;
      return none;
    }
    // 🔴 例外五的第三个泄漏口：已接管时原来也在这里 return none ⇒
    // 又一次 1~2 拍粒度的让位。接管中一律落到下面继续由本层驱动，
    // 退出只走 kNone 分支里那条"连续 N 拍脱离"的正规路径。
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
    center_cost, footprint_cost, favorable_cost, have_path, saturated_stalled,
    narrow_engaged_, narrow_trigger_);

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
      // 没有路径就定义不出「沿通道」这个轴 ⇒ 本层无从下手，必须让位。
      // 与 kNone **不能合并**：那一条是"还在通道里但代价掉了"，可以继续开。
      ++narrow_clear_hits_;
      narrow_hits_ = 0;
      return none;

    case NarrowVerdict::kNone:
      // 走到这里说明 footprint_cost 过阈但 favorable/center 都正常，
      // 而 kNone 只可能来自 footprint_cost 未过阈 —— 上面的早退已经处理过。
      // 保留分支是为了 switch 穷尽（编译器会盯着），不做额外动作。
      ++narrow_clear_hits_;
      narrow_hits_ = 0;
      // 🔴 例外五的第二个泄漏口：**已接管期间不许在这里让位**。
      // clear_hits 没满 narrow_clear_ticks 拍时原来直接 return none，
      // 于是每 1~2 拍就把方向盘交回 MPPI 一次 —— 标志位还亮着"已接管"，
      // 实际在开车的是内层。退出必须只走 disengageNarrow 那一条路径；
      // 计数照加，但车继续由本层开，落到下面的贴边计算。
      if (!narrow_engaged_) {
        return none;
      }
      // 🔴 第四个泄漏口（实测抓到）：最短驻留只挡了上面第 1884 行那条早退路径，
      //    **这一条 disengageNarrow 完全没问它**。于是 banner 打印
      //    「最短驻留=4.0s」而真实退出只需 narrow_clear_ticks=3 拍 = 0.15s。
      //
      //    实测证据（远端目标 5.88,-5.86 那一腿）：
      //      t=371.5s 第二入口接管(足迹 253、朝向误差 -0.680rad)
      //      t=371.9s 「足迹已连续脱离致命带」退出 —— 只活了 **0.45s**，
      //               而转正到最近有利朝向的算术下界是 3.93s，即它必然是在
      //               **还没转正**时就被赶出去的
      //      t=371.9~467.6s 交回 MPPI 后 4 次 Failed to make progress + 一次
      //               spin 恢复，**96s 没有进展**
      //      t=467.6s 再次接管，这次连续开了 41s，t=508.8 位置到达、成功
      //    也就是说：本层每次真正开够时间都能穿过去，穿不过去只是因为被
      //    提前赶出去。这正是 narrow_math.hpp 里 saturationDwellHolding 的
      //    文档注释预言的振荡（"转正之前就满足脱离条件、交回 MPPI、
      //    MPPI 再转回去再卡住 —— 保证振荡"），只是那道闸没接到这条出口上。
      //
      //    迟滞的同变量原则：入口问「有没有推进」，出口就不能只问「代价高不高」。
      //    驻留未满时计数照加（上面已 ++），但方向盘不交回去。
      if (saturation_dwell_holding) {
        ++narrow_suppressed_dwell_;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 1000,
          "[%s] 足迹已脱离致命带但第二入口最短驻留未满(%.2f/%.2fs)，继续由本层驱动",
          name_.c_str(), narrow_engaged_sec, narrow_saturation_min_dwell_sec_);
      }
      // 出口只有这一处，且驻留与拍数两项都由 narrowShouldDisengage 一起问 ——
      // 少问一项曾经让「最短驻留=4.0s」变成实际 0.15s（见上）。
      if (narrowShouldDisengage(
          narrow_clear_hits_, narrow_clear_ticks_, saturation_dwell_holding))
      {
        updateNarrowFailureCount(/*cleared_through=*/ true, narrow_engage_count_);
        disengageNarrow("足迹已连续脱离致命带(kNone 路径)");
        narrow_threw_here_ = false;
        return none;
      }
      break;      // 继续由本层驱动

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
    // 记录入口来源：迟滞的脱离阈值要跟着它走（见上方例外三）。
    narrow_via_saturation_ =
      footprint_cost < narrow_trigger_.footprint_lethal_threshold;
    if (narrow_via_saturation_) {
      ++narrow_saturation_engagements_;
    }
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

  // 🔴 续租放在**函数最顶端**，在任何相位判断、任何一层接管判断之前。
  //    第一轮实测的教训：原先写在 evaluateSquare 里，而 kFollow 在
  //    evaluateNarrow 接管时会 early-return，根本走不到 —— 于是恰好在窄通道
  //    接管期间停止续租，协调器侧看门狗 2.1s 时误判"进程死了"，
  //    把足迹从通道中途抢回默认值。续租是存活信号，与哪层在控制无关。
  renewFootprintLease(now);

  // 🔴 tick 时刻也必须在**任何** early-return / throw 之前记下来。
  //    它是 setPlan 里区分「周期重规划」与「同一目标的新一次下发」的唯一依据
  //    （见 isFreshFollowAttempt）。记漏一拍不致命，但如果把它挪到某个
  //    early-return 之后，窄通道接管期间就会停止更新 —— 那时空档会被越算越大，
  //    接管一结束的第一次重规划就被误判成"新尝试"、白白重置对齐预算。
  //    这与续租那条教训是同一个坑：存活信号不能藏在 early-return 后面。
  last_tick_time_ = now;
  has_tick_ = true;

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

      // 走到这里说明足迹还没碰致命带（开阔处）。此时看前方是否有「只有转正
      // 才过得去」的窄处 —— 有就趁现在原地转，而不是等撞进去再在里面转。
      // 顺序同样在调内层**之前**：接管这一拍内层不该出指令，也不该更新它的内部状态。
      const PrealignDecision prealign = evaluatePrealign(pose, now);
      if (prealign.take_over) {
        return prealign.cmd;
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
