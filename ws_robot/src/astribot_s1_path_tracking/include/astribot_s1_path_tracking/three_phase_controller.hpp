// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <cmath>
#include <cstddef>

#include "astribot_s1_path_tracking/align_math.hpp"
#include "astribot_s1_path_tracking/controller_execution.hpp"
#include "astribot_s1_path_tracking/corner_stop_evidence.hpp"
#include "astribot_s1_path_tracking/layered_collision_reader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

namespace astribot_s1_path_tracking
{

class ThreePhaseController : public nav2_core::Controller
{
public:
  ThreePhaseController() = default;
  ~ThreePhaseController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /// 供测试/诊断读取当前相位。
  Phase phase() const {return phase_;}

protected:
  LayeredCollisionReader alignment_collision_;
  enum class PlanUpdate {NewExecution, EquivalentRefresh, RouteReplacement};
  virtual void applyPlan(const nav_msgs::msg::Path & path);
  void applyPendingPlan();
  void beginExecution();
  PlanUpdate planUpdate() const {return plan_update_;}
  virtual double cornerStoppingDistance(double) const {return 0.;}
  virtual double cornerSettleDuration() const {return .6;}
  virtual double cornerSourceMaxGap() const {return .5;}
  bool cornerReanchorReady(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,const rclcpp::Time & now);
  void recordCornerPause(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,double now);
  double translationApproachLimit(double distance,double maximum) const;
  geometry_msgs::msg::TwistStamped policyAlignment(
    double error, double measured_wz, const std_msgs::msg::Header & header) {
    return rotateOnly(error, measured_wz, header);
  }
  void preparePolicyTakeover() {
    if(start_heading_valid_ && phase_!=Phase::kAlignStart) {
      enterPhase(Phase::kAlignStart,clock_->now(),"validated policy takeover",true);
    }
  }
  virtual bool hasTerminalRefinement() const {return false;}
  bool terminalRefinementAllowed(const geometry_msgs::msg::PoseStamped & pose,
    double capture_radius) const;
  void accountPolicyPause(double seconds, const rclcpp::Time & now) {
    // A replacement can create this phase after the paused interval began.
    const double overlap = std::min(std::max(0.0, seconds),
      std::max(0.0, (budget_clock_.now() - phase_started_).seconds()));
    phase_started_ = phase_started_ + rclcpp::Duration::from_seconds(overlap);
    last_tick_time_ = now;
  }

  void observeCornerPose(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, double now_s);

private:
  friend class ThreePhaseControllerTestPeer;
  /// 读取参数，越界即抛（禁止静默回落到"看起来正常"的默认值）。
  void declareAndLoadParams();

  /// 加载内层控制器。失败即抛 —— 没有内层控制器就没有跟踪能力，
  /// 不能降级成"只会原地转"。
  void loadInnerController(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);

  /// 从 goal_checker 取位置/朝向容差；取不到则用配置的兜底值并告警一次。
  void resolveTolerances(nav2_core::GoalChecker * goal_checker, double & xy_tol, double & yaw_tol);

  /// 纯旋转指令：只出 wz，vx/vy 严格为 0。
  ///
  /// wz_now 是**实测**角速度，用于惯性补偿（align_inertia_enabled_=false 时忽略）。
  /// 本函数**不是 const**：它维护滑行闩锁与标定统计，两者都必须跨拍。
  geometry_msgs::msg::TwistStamped rotateOnly(
    double error_rad, double wz_now, const std_msgs::msg::Header & header);

  /// 接近段限速：就地按距终点距离夹住 cmd 的线速度模长。
  ///
  /// 只在 kFollow 且**内层已出指令之后**调用。两个对齐段是纯旋转，
  /// 没有线速度可夹，天然不经过这里。
  void applyApproachCap(geometry_msgs::msg::TwistStamped & cmd, double dist_to_goal_m);

  /// Detect and execute standard-angle vertices without changing smooth
  /// curvature tracking.  The approach phase holds zero angular velocity;
  /// the following alignment phase uses the existing inertial-aware rotator.
  bool maybeStartCorner(
    const geometry_msgs::msg::PoseStamped & pose, double robot_yaw, double goal_xy_tolerance,
    double measured_speed=0.);
  bool cornerStopped(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,double now);
  void recordCornerCommand(const geometry_msgs::msg::Twist & command,double now);
  enum class CornerStage {Idle, Approach, SettlingBeforeTurn, Turning, SettlingAfterTurn, Recovering, ReanchorSettling};
  void setCornerStage(CornerStage stage);
  void emitCornerState(const char * reason);
  void updateCornerSegment();
  geometry_msgs::msg::TwistStamped cornerRotationCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, double error);
  geometry_msgs::msg::TwistStamped cornerApproachCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & phase_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker,
    double distance_to_corner);
  void resetCornerState(bool clear_detection);

  /// 若 ALIGN_GOAL 段实际拿不到执行机会，告警一次（只在真实取到容差后调用）。
  /// 存在的理由：`align_goal_enabled: true` 会让人以为终点对齐在工作，
  /// 而它取决于 GoalChecker 是否卡朝向 —— 一个看着开着其实关着的开关，
  /// 必须显式说出来，不能只写在 yaml 注释里。
  void warnIfGoalAlignInert(double xy_tol, double yaw_tol);

  /// 若 GoalChecker 的朝向容差**紧到本层物理上到不了**，告警一次。
  void warnIfYawToleranceUnreachable(double yaw_tol);

  /// 一次性说明"终点航向的交付精度是哪个数"，并在两个数**离得太远**时告警。
  void noteDeliveredYawAccuracy(double yaw_tol);

  /// 本层能保证的最好朝向落点 [rad] = align_tolerance + 松手后的余转。
  double bestAchievableYawLanding() const;

  /// 相位超时检查，超时抛异常。
  void checkPhaseTimeout(const rclcpp::Time & now);

  /// 进入相位；新目标须传 restart_timer=true，即使相位未变也重置超时计时。
  void enterPhase(
    Phase p, const rclcpp::Time & now, const char * why, bool restart_timer = false);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThreePhaseController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> inner_loader_;
  nav2_core::Controller::Ptr inner_;

  nav_msgs::msg::Path plan_;
  std::shared_ptr<ControllerExecution> execution_;
  uint64_t execution_generation_{0};
  int64_t accepted_plan_stamp_ns_{0};
  std::optional<nav_msgs::msg::Path> pending_plan_;
  PlanUpdate plan_update_{PlanUpdate::NewExecution};
  Phase phase_{Phase::kDone};
  rclcpp::Clock budget_clock_{RCL_STEADY_TIME};
  rclcpp::Time phase_started_{0, 0, RCL_STEADY_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  double start_heading_{0.0};
  bool start_heading_valid_{false};
  bool start_alignment_engaged_{false};
  /// 上一条路径的终点，用于区分「新目标」与「同一目标的周期性重规划」。
  PlanarPoint last_goal_end_{};
  bool has_last_goal_{false};
  bool warned_tolerance_fallback_{false};
  /// ALIGN_GOAL 段实际不生效的告警只打一次（20Hz 刷屏会把真问题淹掉）。
  bool warned_goal_align_inert_{false};
  /// 朝向容差紧到到不了的告警同样只打一次。
  bool warned_yaw_unreachable_{false};
  /// "交付精度是哪个数"的一次性说明（含两数拉开过大的告警）。
  bool noted_delivered_accuracy_{false};
  /// 用控制周期空档区分周期重规划与新 FollowPath 尝试，见 isFreshFollowAttempt。
  /// 必须使用 RCL_ROS_TIME，与 clock_->now() 保持一致。
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  bool has_tick_{false};

  std::string inner_plugin_;
  bool align_start_enabled_{true};
  bool align_goal_enabled_{true};
  bool zero_vy_in_follow_{true};
  double align_kp_{1.0};
  double align_max_vel_{0.6};
  double align_floor_vel_{0.05};
  double align_tol_rad_{0.05};
  double start_min_angle_rad_{0.20};  double lookahead_m_{0.5};
  double align_timeout_sec_{15.0};
  double follow_timeout_sec_{0.0};      // 0 = 不限（交给 nav2 的 progress_checker）
  double fallback_xy_tol_{0.18};
  double fallback_yaw_tol_{0.20};
  /// 终点位移小于此值即视为同一目标（默认行为树 1Hz 重规划会反复调 setPlan）。
  double new_goal_epsilon_m_{0.25};
  /// tick 空档超过此值 => 上一个 FollowPath action 已结束，这次 setPlan 是新一次
  /// 尝试。默认 0.5s = 20Hz 下 10 拍：远大于单拍抖动，又远小于 align_timeout(15s)。
  double new_attempt_gap_sec_{0.5};

  /// 接近段限速开关；false 时不施加本层接近限速。
  bool approach_enabled_{true};
  /// 收敛区长度 D(m)：d < D 时线速度模长上限按 d/D 收敛，以减小响应滞后造成的过冲。
  double approach_dist_m_{1.50};
  /// 速度下限(m/s)：保证终段还能动（实测底盘 0.02 m/s 即可平动）。
  double approach_v_min_{0.05};

  /// Standard-angle turn mode.  It is opt-in in the controller defaults and
  /// enabled explicitly by the simulation/hardware navigation profiles.
  bool corner_turn_enabled_{false};
  double corner_min_angle_rad_{0.61};
  double corner_max_angle_rad_{2.75};
  double corner_min_segment_m_{0.25};
  double corner_approach_distance_m_{0.25};
  double corner_capture_radius_m_{0.04};
  double corner_lateral_tolerance_m_{0.15};
  double corner_approach_speed_mps_{0.08};
  double corner_approach_min_speed_mps_{0.02};
  double corner_approach_kp_{0.8};
  /// Minimum post-corner distance reserved for the final GoalChecker decision.
  double corner_terminal_guard_m_{0.10};
  double corner_timeout_sec_{15.0};

  std::vector<PathCorner> corners_;
  std::size_t corner_cursor_{0U};
  std::size_t corner_completed_count_{0U};
  bool corner_active_{false};
  double corner_started_s_{0.0};
  double corner_heading_{0.0};
  PlanarPoint corner_position_{};
  geometry_msgs::msg::PoseStamped last_corner_pose_;
  bool has_corner_pose_{false};
  double last_corner_pose_time_s_{0.};
  double corner_feedback_linear_bound_mps_{std::sqrt(2.)};
  double corner_feedback_angular_bound_radps_{2.};
  bool corner_reanchor_pending_{false};
  bool corner_position_recovering_{false};
  CornerStopEvidence corner_stop_;
  CornerStage corner_stage_{CornerStage::Idle};
  uint64_t corner_event_sequence_{0};
  bool corner_braking_{false};
  std::size_t corner_plan_revision_{0U};

  /// 角速度惯性补偿开关。关闭时需重新放宽 yaw_goal_tolerance，覆盖未补偿余转。
  bool align_inertia_enabled_{true};
  /// 指令→实际的角速度死时间 [s]。
  double align_coast_lag_{0.03};
  /// 松手后的等效角减速度 [rad/s^2]。
  double align_coast_decel_{7.0};
  /// 停稳角速度阈值(rad/s)，须小于 floor 指令下的实测角速度，避免提前解除滑停锁定。
  double align_settled_wz_{0.02};

  /// 预测到位后锁定零速直到停稳，避免边界反复反转；进入相位时复位。
  bool align_coasting_{false};
  /// 停止指令时的误差、实测角速度及预测余转，用于记录预测/实测差异并标定 lag/decel。
  double coast_err_at_stop_{0.0};
  double coast_wz_at_stop_{0.0};
  double coast_predicted_{0.0};

  /// 每个目标只记录一次到达误差；仅新目标重置，周期重规划不重置。
  bool arrival_logged_{false};
  /// 跨目标累计量。用户要的是"统计"，单条读数不够：单个目标的误差落在容差内
  /// 说明不了系统性偏置，n 条的均值/最大值才能。进程内累计，重启即清零。
  int arrival_count_{0};
  double arrival_xy_sum_{0.0};
  double arrival_xy_max_{0.0};
  double arrival_yaw_sum_{0.0};
  double arrival_yaw_max_{0.0};
};

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
