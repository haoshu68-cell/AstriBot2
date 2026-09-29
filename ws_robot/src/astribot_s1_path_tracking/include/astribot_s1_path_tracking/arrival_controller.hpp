// Copyright 2026 Astribot
#ifndef ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_CONTROLLER_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_CONTROLLER_HPP_

#include <mutex>
#include "astribot_s1_path_tracking/envelope_guard.hpp"
#include "astribot_s1_path_tracking/arrival_progress.hpp"
#include "astribot_s1_path_tracking/arrival_braking.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
#include "astribot_bridge_msgs/msg/bridge_status.hpp"
#include "astribot_s1_path_tracking/policy_lease.hpp"
#include "astribot_s1_path_tracking/corridor_lease.hpp"
#include "astribot_navigation_msgs/msg/corridor_alignment.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdint>
#include "astribot_s1_path_tracking/three_phase_controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "astribot_s1_path_tracking/tracking_metrics.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace astribot_s1_path_tracking
{
class ArrivalGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void reset() override {ready_ = false; ++generation_;}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override;
  bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &) override;
  void report(bool ready);
  uint64_t generation() const {return generation_;}
  double xyTolerance() const {return xy_;}
  double yawTolerance() const {return yaw_;}
  double stoppedLinear() const {return stopped_v_;}
  double stoppedAngular() const {return stopped_w_;}
private:
  friend class ArrivalControllerTestPeer;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time reported_{0, 0, RCL_ROS_TIME};
  bool ready_{false};
  uint64_t generation_{0};
  double xy_{0.03}, yaw_{0.02617993877991494};
  double stopped_v_{0.01}, stopped_w_{0.01};
};

class ArrivalController : public ThreePhaseController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void deactivate() override;
  void setSpeedLimit(const double &, const bool &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override;
protected:
  void applyPlan(const nav_msgs::msg::Path &) override;
  double cornerStoppingDistance(double speed) const override;
  double cornerSettleDuration() const override {return settle_time_;}
  bool hasTerminalRefinement() const override {return true;}
private:
  friend class ArrivalControllerTestPeer;
  geometry_msgs::msg::TwistStamped computeCommand(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *);
  void limitCornerTranslation(geometry_msgs::msg::Twist & command);
  EnvelopeGuard geometry_guard_;
  std::shared_ptr<ArrivalProgress> arrival_progress_;
  PolicyLease policy_lease_;
  rclcpp::Subscription<astribot_bridge_msgs::msg::BridgeStatus>::SharedPtr bridge_status_sub_;
  std::mutex bridge_status_mutex_;
  std::string bridge_error_;
  int64_t bridge_status_stamp_{-1};
  rclcpp::Subscription<PolicyLease::Message>::SharedPtr policy_sub_;
  bool policy_takeover_{false};
  bool policy_enabled_{false}, policy_paused_{false};
  double policy_tick_{-1};
  std::mutex speed_limit_mutex_;
  double nominal_speed_{0.}, external_speed_limit_{0.};
  bool external_speed_percentage_{false};
  using CorridorAlignment = astribot_navigation_msgs::msg::CorridorAlignment;
  rclcpp::Subscription<CorridorAlignment>::SharedPtr corridor_alignment_sub_;
  CorridorAlignment::ConstSharedPtr corridor_alignment_;
  std::mutex corridor_mutex_;
  std::chrono::steady_clock::time_point corridor_received_;
  bool corridor_simulated_{false};
  nav_msgs::msg::Path tracking_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
  double localSharpPathLimit(const geometry_msgs::msg::PoseStamped & pose) const;
  void publishPhase(const char * phase);
  void logMetrics(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, const ArrivalGoalChecker &, const char * phase);
  MetricSamplingClock metrics_clock_;
  std::optional<double> metrics_anchor_;
  uint64_t plan_revision_{0};
  double metrics_rate_{2.0}, metrics_terminal_radius_{0.5};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  void resetAttempt();
  [[noreturn]] void fail(const std::string &);
  geometry_msgs::msg::PoseStamped inFrame(const geometry_msgs::msg::PoseStamped &,
    const std::string &);
  bool safeCommand(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, const geometry_msgs::msg::Twist &);
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_sub_;
  std::mutex observation_mutex_;
  geometry_msgs::msg::PoseStamped observation_, goal_, anchor_;
  std::string source_, error_;
  bool observed_{false}, has_goal_{false}, refining_{false}, started_{false};
  bool holding_{false}, completion_logged_{false};
  bool xy_held_{false}, yaw_held_{false};
  bool corridor_terminal_coast_{false};
  double stop_ratio_{1.0}, resume_ratio_{1.0}, settle_drift_ratio_{1.0};
  rclcpp::Logger logger_{rclcpp::get_logger("ArrivalController")};
  uint64_t generation_{0};
  ArrivalGoalChecker * checker_{nullptr};
  double last_tick_{-1};
  double started_at_{0}, progress_at_{0}, refine_at_{0}, best_error_{0};
  double capture_{0.30}, refine_timeout_{45}, total_timeout_{300};
  double progress_timeout_{15}, settle_time_{0.6}, kp_xy_{0.5}, kp_yaw_{0.8};
  double max_v_{0.06}, max_w_{0.15}, speed_scale_{1.0};
  double min_v_{0.0}, min_w_{0.0}, coarse_yaw_{0.0};
  double braking_xy_{0.0}, braking_yaw_{0.0};
  std::string braking_model_{"constant_time"};
  std::string angular_braking_model_{"constant_time"};
  double normal_acceleration_{0.25},normal_jerk_{0.5};
  double angular_acceleration_{0.6},angular_jerk_{1.2};
  SettledOffsetCurve linear_offset_,angular_offset_;
  ArrivalCoast<2> xy_coast_;
  ArrivalCoast<1> yaw_coast_;
  ArrivalSettling<2> xy_settling_;
  ArrivalSettling<1> yaw_settling_;
};
}  // namespace astribot_s1_path_tracking
#endif
