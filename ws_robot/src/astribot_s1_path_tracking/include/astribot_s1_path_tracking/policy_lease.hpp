#ifndef ASTRIBOT_S1_PATH_TRACKING__POLICY_LEASE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__POLICY_LEASE_HPP_
#include <chrono>
#include <mutex>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "astribot_navigation_msgs/msg/motion_constraint.hpp"
namespace astribot_s1_path_tracking {
class PolicyLease {
public:
  using Message = astribot_navigation_msgs::msg::MotionConstraint;
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
  bool held(const rclcpp::Time & now, bool no_planning=false) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshUnlocked(now) && message_.hold && (!no_planning || message_.planning==0);
  }
private:
  bool freshUnlocked(const rclcpp::Time & now) const {
    if (!seen_) {return false;}
    double age=(now-rclcpp::Time(message_.stamp,now.get_clock_type())).seconds();
    double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count();
    return age>=0 && age<=message_.lease_s && wall_age<=message_.lease_s;
  }
  mutable std::mutex mutex_;
  Message message_;
  bool seen_{false};
  std::chrono::steady_clock::time_point received_;
};
}
#endif
