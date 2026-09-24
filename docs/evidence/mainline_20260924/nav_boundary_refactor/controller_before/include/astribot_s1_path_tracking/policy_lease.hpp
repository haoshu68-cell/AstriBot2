#ifndef ASTRIBOT_S1_PATH_TRACKING__POLICY_LEASE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__POLICY_LEASE_HPP_
#include <chrono>
#include <mutex>
#include <cmath>
#include <string>
#include <optional>
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
  // Future-stamped input grants no motion. Allow a bounded zero-command wait
  // for independently delivered /clock and constraint messages to catch up.
  Status status(const rclcpp::Time & now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (freshUnlocked(now)) {return Status::Fresh;}
    if (!seen_) {return Status::Invalid;}
    const auto wall_now=std::chrono::steady_clock::now();
    const double age=(now-rclcpp::Time(message_.stamp,now.get_clock_type())).seconds();
    const double wall_age=std::chrono::duration<double>(wall_now-received_).count();
    if (!(age<0 && -age<=message_.lease_s && wall_age<=message_.lease_s)) {return Status::Invalid;}
    if (!clock_wait_started_) {clock_wait_started_=wall_now;}
    return std::chrono::duration<double>(wall_now-*clock_wait_started_).count()<=message_.lease_s ? Status::WaitForClock : Status::Invalid;
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
private:
  bool freshUnlocked(const rclcpp::Time & now) const {
    if (!seen_) {return false;}
    double age=(now-rclcpp::Time(message_.stamp,now.get_clock_type())).seconds();
    double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count();
    const bool valid=age>=0 && age<=message_.lease_s && wall_age<=message_.lease_s;
    if (valid) {clock_wait_started_.reset();}
    return valid;
  }
  mutable std::mutex mutex_;
  Message message_;
  bool seen_{false};
  std::chrono::steady_clock::time_point received_;
  mutable std::optional<std::chrono::steady_clock::time_point> clock_wait_started_;
};
}
#endif
