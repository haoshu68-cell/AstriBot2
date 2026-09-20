#ifndef ASTRIBOT_S1_PATH_TRACKING__START_MANEUVER_CHANNEL_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__START_MANEUVER_CHANNEL_HPP_
#include <chrono>
#include <cmath>
#include <mutex>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "astribot_navigation_msgs/msg/start_maneuver.hpp"
#include "astribot_navigation_msgs/msg/start_maneuver_request.hpp"

namespace astribot_s1_path_tracking {
class StartManeuverChannel {
public:
  using Reply=astribot_navigation_msgs::msg::StartManeuver;
  using Request=astribot_navigation_msgs::msg::StartManeuverRequest;
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) {
    clock_=node->get_clock();simulated_=node->get_parameter("use_sim_time").as_bool();
    request_id_=std::chrono::steady_clock::now().time_since_epoch().count();
    publisher_=rclcpp::create_publisher<Request>(node,"path_tracking/start_maneuver_request",1);
    subscriber_=node->create_subscription<Reply>("navigation_policy/start_maneuver",1,
      [this](Reply::ConstSharedPtr reply) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reply->request_id!=request_id_) {return;}
        reply_=reply;received_=std::chrono::steady_clock::now();
      });
  }
  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);++request_id_;reply_.reset();
  }
  void cleanup() {reset();subscriber_.reset();publisher_.reset();}
  Reply::ConstSharedPtr exchange(const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & pose,double heading) {
    std::lock_guard<std::mutex> lock(mutex_);
    Request request;request.stamp=clock_->now();request.request_id=request_id_;
    request.reference_path=path;request.current_pose=pose;request.target_heading_rad=heading;
    publisher_->publish(request);
    if (!reply_ || reply_->request_id!=request_id_ || !std::isfinite(reply_->lease_s) ||
        reply_->lease_s<=0 || reply_->lease_s>0.3 || reply_->mode>Reply::FAILED) {return nullptr;}
    const double age=(clock_->now()-rclcpp::Time(reply_->stamp,clock_->get_clock_type())).seconds();
    const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count();
    const auto & evaluated=reply_->evaluated_pose;
    const double distance=std::hypot(pose.pose.position.x-evaluated.pose.position.x,
                                    pose.pose.position.y-evaluated.pose.position.y);
    const double angle=std::abs(std::remainder(tf2::getYaw(pose.pose.orientation)-
                                              tf2::getYaw(evaluated.pose.orientation),2*M_PI));
    const double turn=std::remainder(heading-tf2::getYaw(evaluated.pose.orientation),2*M_PI);
    const double rotated=std::remainder(tf2::getYaw(pose.pose.orientation)-
                                        tf2::getYaw(evaluated.pose.orientation),2*M_PI);
    const double progress=std::copysign(1.0,turn)*rotated;
    const bool orientation_valid=reply_->mode==Reply::TURN ?
      progress>=-1e-6 && progress<=std::abs(turn)+1e-6 : angle<=0.03;
    if (age<0 || age>reply_->lease_s || (!simulated_ && wall>reply_->lease_s) ||
        evaluated.header.frame_id!=pose.header.frame_id || !std::isfinite(distance) ||
        !std::isfinite(angle) || distance>0.04 || !orientation_valid) {return nullptr;}
    const auto & c=reply_->command;
    if (!std::isfinite(c.linear.x) || !std::isfinite(c.linear.y) || !std::isfinite(c.angular.z) ||
        c.linear.x>0 || std::hypot(c.linear.x,c.linear.y)>.050000001 || std::abs(c.linear.y)>.01 ||
        c.linear.z!=0 || c.angular.x!=0 || c.angular.y!=0 || std::abs(c.angular.z)>.1 ||
        (c.linear.x==0 && (c.linear.y!=0 || c.angular.z!=0)) ||
        (reply_->mode!=Reply::REVERSE && (c.linear.x!=0 || c.linear.y!=0 || c.angular.z!=0))) {return nullptr;}
    return reply_;
  }
private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<Request>::SharedPtr publisher_;
  rclcpp::Subscription<Reply>::SharedPtr subscriber_;
  Reply::ConstSharedPtr reply_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_;
  uint64_t request_id_{0};
  bool simulated_{false};
};
}
#endif
