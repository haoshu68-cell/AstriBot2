#ifndef ASTRIBOT_S1_PATH_TRACKING__SLIP_MONITOR_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__SLIP_MONITOR_HPP_

#include <mutex>
#include "astribot_s1_path_tracking/linear_slip.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace astribot_s1_path_tracking
{
class SlipMonitor
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &,const std::string &);
  void cleanup();
  void reset();
  void pause();
  bool enabled() const {return mode_!="off";}
  double maximum() const {return detector_.config().max_speed;}
  bool apply(geometry_msgs::msg::Twist &,double cap,bool eligible);
private:
  void pose(const nav_msgs::msg::Odometry &,bool slam);
  LinearSlip detector_;
  std::mutex mutex_;
  std::string mode_{"off"},slam_frame_,sdk_frame_,slam_child_,sdk_child_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("linear_slip")};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_sub_,sdk_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
};
}  // namespace astribot_s1_path_tracking
#endif
