// Copyright 2026 Astribot
#ifndef ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_CONTROLLER_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_CONTROLLER_HPP_

#include <mutex>
#include "astribot_s1_path_tracking/policy_lease.hpp"
#include "astribot_navigation_msgs/msg/corridor_alignment.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdint>
#include "astribot_s1_path_tracking/three_phase_controller.hpp"
#include "nav2_core/goal_checker.hpp"
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
private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time reported_{0, 0, RCL_ROS_TIME};
  bool ready_{false};
  uint64_t generation_{0};
  double xy_{0.03}, yaw_{0.02617993877991494};
};

class ArrivalController : public ThreePhaseController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path &) override;
  void setSpeedLimit(const double &, const bool &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override;
protected:
  bool hasTerminalRefinement() const override {return true;}
private:
  PolicyLease policy_lease_;
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
  nav_msgs::msg::Path tracking_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
  double localSharpPathLimit(const geometry_msgs::msg::PoseStamped & pose) const;
  void publishPhase(const char * phase);
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
  rclcpp::Logger logger_{rclcpp::get_logger("ArrivalController")};
  uint64_t generation_{0};
  ArrivalGoalChecker * checker_{nullptr};
  double last_tick_{-1};
  double started_at_{0}, progress_at_{0}, refine_at_{0}, hold_at_{0}, best_error_{0};
  double capture_{0.30}, pose_timeout_{0.5}, refine_timeout_{45}, total_timeout_{300};
  double progress_timeout_{15}, settle_time_{0.6}, kp_xy_{0.5}, kp_yaw_{0.8};
  double max_v_{0.06}, max_w_{0.15}, speed_scale_{1.0};
};
}  // namespace astribot_s1_path_tracking
#endif
