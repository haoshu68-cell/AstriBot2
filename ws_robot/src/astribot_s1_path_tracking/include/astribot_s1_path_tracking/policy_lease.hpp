#ifndef ASTRIBOT_S1_PATH_TRACKING__POLICY_LEASE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__POLICY_LEASE_HPP_
#include <chrono>
#include <mutex>
#include <cmath>
#include <string>
#include <optional>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "astribot_navigation_msgs/msg/motion_constraint.hpp"
namespace astribot_s1_path_tracking {
class PolicyLease {
public:
  using Message = astribot_navigation_msgs::msg::MotionConstraint;
  enum class Status {Fresh, WaitForClock, Invalid};
  void receive(const Message & msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg.lease_s) || msg.lease_s<=0 || msg.lease_s>0.5 ||
        !std::isfinite(msg.max_linear_speed) || msg.max_linear_speed<0 ||
        !std::isfinite(msg.max_angular_speed) || msg.max_angular_speed<0) {return;}
    if (seen_ && msg.epoch==message_.epoch && msg.sequence<=message_.sequence) {return;}
    message_=msg; received_=std::chrono::steady_clock::now();seen_=true;
  }
  bool fresh(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_); return freshUnlocked(now);
  }
  Status status(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);return freshUnlocked(now)?Status::Fresh:Status::Invalid;
  }
  bool waitingForClock(const rclcpp::Time & now) const {return status(now)==Status::WaitForClock;}
  std::string freshnessDetail(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!seen_) {return "no_constraint_received";}
    const double age=(now-rclcpp::Time(message_.stamp,now.get_clock_type())).seconds();
    const double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count();
    return "source_age_s="+std::to_string(age)+" wall_age_s="+std::to_string(wall_age)+
      " lease_s="+std::to_string(message_.lease_s)+" epoch="+std::to_string(message_.epoch)+
      " sequence="+std::to_string(message_.sequence);
  }
  bool held(const rclcpp::Time & now, bool no_planning=false) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && message_.hold && (!no_planning || message_.planning==0);
  }
  bool alignmentRequired(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && message_.alignment_required;
  }
  bool centeringRequired(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && message_.centering_required;
  }
  bool corridorTrackingRequired(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && message_.corridor_tracking_required;
  }
  double linearSpeedLimit(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && !message_.hold ? message_.max_linear_speed : 0.;
  }
  double cornerAngularSpeedLimit(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && !message_.hold && !message_.centering_required &&
      !message_.alignment_required && !message_.corridor_tracking_required ? message_.max_angular_speed : 0.;
  }
  // Apply one coherent constraint snapshot. Scaling the complete planar twist
  // preserves curvature; callers must recheck a sweep if a forbidden axis changes.
  bool restrict(geometry_msgs::msg::Twist & command, const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before=command;
    if (!freshUnlocked(now) || message_.hold) {command={};return command!=before;}
    if (message_.alignment_required) {command.linear.x=command.linear.y=0.;}
    if (message_.centering_required) {command.angular.z=0.;}
    const double speed=std::hypot(command.linear.x,command.linear.y);
    const double angular=std::abs(command.angular.z);
    const double ratio=std::min({1.,speed>0. ? message_.max_linear_speed/speed : 1.,
      angular>0. ? message_.max_angular_speed/angular : 1.});
    command.linear.x*=ratio;command.linear.y*=ratio;command.angular.z*=ratio;
    return command!=before;
  }
private:
  bool freshUnlocked(const rclcpp::Time & now) const {
    (void)now;return seen_;
  }
  mutable std::mutex mutex_;
  Message message_;
  bool seen_{false};
  std::chrono::steady_clock::time_point received_;
};
}
#endif
