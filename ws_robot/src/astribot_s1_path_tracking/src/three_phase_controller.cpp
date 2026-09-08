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

  // 2026-09-07 删除窄通道贴边通行层之后**再没有任何一层保留横移能力**：
  // 那一层自己算指令并在内层之前 return，是 vy 的唯一出口。现在
  // DiffDrive + vy_max/vy_std=0 + zero_vy_in_follow 三处一致地把 vy 归零，
  // 底盘物理上仍是全向的，但整条控制链不再使用横移。在线要能反证这一点，
  // 所以单独打一行 —— 否则"机器人不横移"分不清是配置如此还是某层坏了。
  RCLCPP_INFO(
    logger_,
    "[%s] 窄通道贴边通行层已删除(2026-09-07)：足迹代价全覆盖的通道完全交给内层"
    "控制器；跟踪段横移=%s，本层不再产生任何 vy",
    name_.c_str(), zero_vy_in_follow_ ? "禁止" : "允许");

  // 接近段限速单独打一行，理由同上：在线要能反证这一档到底开没开。
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

  // ---- 接近段限速参数校验 ----
  // 关掉时不校验：用不上的值不该成为拒绝启动的理由。
  if (approach_enabled_) {
    if (!(approach_dist_m_ > 0.0)) {
      throw nav2_core::PlannerException("ThreePhaseController: approach_dist 必须 > 0");
    }
    if (!(approach_v_min_ > 0.0)) {
      // 置 0 会让终段算出小到驱动不了底盘的速度而卡死在容差外
      // （实测底盘 0.02 m/s 才能平动）。
      throw nav2_core::PlannerException("ThreePhaseController: approach_v_min 必须 > 0");
    }
    if (!(approach_v_min_ < approach_dist_m_)) {
      // 量纲不同（m/s vs m）故这不是物理约束，纯粹为了捕获把两者写颠倒 ——
      // 颠倒后 D=0.05m 使限速段整个落在容差内、形同虚设，且下限 1.5m/s 超速。
      throw nav2_core::PlannerException(
        "ThreePhaseController: approach_v_min 必须 < approach_dist（疑似两者写颠倒）");
    }
    if (!(approach_dist_m_ > fallback_xy_tol_)) {
      // 收敛区必须比到位容差大，否则 nav2 判到位时机器人还没进过限速段，
      // 整个机制一次都不生效 —— 而日志上看不出任何异常。
      throw nav2_core::PlannerException(
        "ThreePhaseController: approach_dist 必须 > fallback_xy_tolerance，"
        "否则限速段整个落在到位容差内、形同虚设");
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
      warnIfGoalAlignInert(xy_tol, yaw_tol);
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

void ThreePhaseController::warnIfGoalAlignInert(double xy_tol, double yaw_tol)
{
  if (!align_goal_enabled_ || warned_goal_align_inert_) {
    return;
  }
  // ALIGN_GOAL 段拿不到执行机会的条件，用**实际取到的容差**判，不猜：
  //   · 进入 ALIGN_GOAL 需要 dist_to_goal <= xy_tol（advancePhase）
  //   · controller_server 判到位用的是同一个 GoalChecker；当它不卡朝向时，
  //     判到位 ≡ dist <= xy_tol —— 与上面**同一个数**
  // 两个条件同时成立 ⇒ 位置一进容差，nav2 当拍就结束 action，
  // ALIGN_GOAL 最多拿到一拍。这是既有缺陷（不是接近段限速引入的）。
  //
  // 判"不卡朝向"的门限取 pi：nav2 的 SimpleGoalChecker 用 yaw_goal_tolerance
  // 直接与 |yaw误差| 比，而 |yaw误差| <= pi 恒成立，故 >= pi 即等于不约束。
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

  // 🔴 tick 时刻必须在**任何** early-return / throw 之前记下来。
  //    它是 setPlan 里区分「周期重规划」与「同一目标的新一次下发」的唯一依据
  //    （见 isFreshFollowAttempt）。记漏一拍不致命，但如果把它挪到某个
  //    early-return 之后，那一段期间就会停止更新 —— 空档会被越算越大，
  //    下一次重规划就被误判成"新尝试"、白白重置对齐预算。
  //    教训归纳：存活信号不能藏在 early-return 后面。
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
      geometry_msgs::msg::TwistStamped cmd =
        inner_->computeVelocityCommands(pose, velocity, goal_checker);
      if (zero_vy_in_follow_) {
        // 前向为主：把横移分量掐掉。注意这会让内层控制器的输出与它自己的
        // 预测不一致（它以为能横移），所以内层应当配成非全向模型；
        // 这里掐掉只是最后一道保险。
        cmd.twist.linear.y = 0.0;
      }
      applyApproachCap(cmd, dist_to_goal);
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

void ThreePhaseController::applyApproachCap(
  geometry_msgs::msg::TwistStamped & cmd, double dist_to_goal_m)
{
  if (!approach_enabled_) {
    return;                       // 一键回退：接近段不限速（限速上线前的行为）
  }

  // 🔴 夹的是**旋转不变的模长** hypot(vx, vy)，并按同一比例缩放两个分量以保持
  //    行进方向不变；不许分别夹 |vx| 与 |vy|。分别夹会在斜向行进时把方向拗弯
  //    （教训：ownership-bound-must-be-rotation-invariant-norm）。
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
  // ⚠ 刻意**不动 wz**：终段仍要继续对齐路径方向，掐掉转向能力会让机器人
  //   直线冲过弯道末段。限的只是"多快到"，不是"往哪转"。

  // 本控制器没有 status 话题，日志是既有的上报通道（规范：不静默）。
  // 不在这里印"预期过冲"：那需要 τ，而 τ 是链路实测属性、不是本层的参数，
  // 硬编码进日志只会在链路变化后变成过期结论。
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "[%s] 接近段限速：d=%.3fm (D=%.2fm) ‖v‖ %.3f -> %.3f m/s",
    name_.c_str(), dist_to_goal_m, approach_dist_m_, speed, cap);
}

}  // namespace astribot_s1_path_tracking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  astribot_s1_path_tracking::ThreePhaseController, nav2_core::Controller)
