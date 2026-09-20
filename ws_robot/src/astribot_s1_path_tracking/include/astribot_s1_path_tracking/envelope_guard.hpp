#pragma once
#include <mutex>
#include <chrono>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "astribot_navigation_msgs/msg/navigation_envelope_v2.hpp"
#include "astribot_navigation_msgs/msg/envelope_apply_status.hpp"
#include "astribot_s1_robot_geometry/filled_collision.hpp"

namespace astribot_s1_path_tracking {
class EnvelopeGuard {
  using Message=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  using Status=astribot_navigation_msgs::msg::EnvelopeApplyStatus;
public:
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map,const std::string & consumer) {
    nav2_util::declare_parameter_if_not_declared(node,"navigation_geometry_mode",rclcpp::ParameterValue("legacy"));
    auto mode=node->get_parameter("navigation_geometry_mode").as_string();
    if(mode!="legacy" && mode!="fixed_v2") throw std::invalid_argument("invalid navigation_geometry_mode");
    enabled_=mode=="fixed_v2";if(!enabled_) return;
    map_=map;clock_=node->get_clock();consumer_=consumer;
    pub_=rclcpp::create_publisher<Status>(node,"/navigation/envelope_applied",10);
    sub_=node->create_subscription<Message>("/navigation/envelope_v2",10,[this](Message::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      if(msg_ && m->coordinator_session_id==msg_->coordinator_session_id && m->epoch<msg_->epoch) return;
      if(msg_ && m->coordinator_session_id==msg_->coordinator_session_id && m->epoch==msg_->epoch &&
         m->installed_geometry_hash!=msg_->installed_geometry_hash) {msg_.reset();return;}
      msg_=m;received_=std::chrono::steady_clock::now();
    });
    timer_=node->create_wall_timer(std::chrono::milliseconds(100),[this]() {
      std::lock_guard<std::mutex> lock(mutex_);if(!msg_) return;
      bool applied=installed();
      for(const auto & name:{consumer_,consumer_=="planner"?std::string("global_costmap"):std::string("local_costmap")}) {
        Status s;s.header.stamp=clock_->now();s.header.frame_id=map_->getBaseFrameID();
        s.coordinator_session_id=msg_->coordinator_session_id;s.consumer_id=name;s.envelope_epoch=msg_->epoch;
        s.installed_geometry_hash=msg_->installed_geometry_hash;s.applied=applied;
        s.reason=applied?"ACTUAL_COSTMAP_FOOTPRINT_APPLIED":"FOOTPRINT_OR_LEASE_MISMATCH";pub_->publish(s);
      }
    });
  }
  bool enabled() const {return enabled_;}
  bool ready() {
    if(!enabled_) return true;
    std::lock_guard<std::mutex> lock(mutex_);
    return installed() && msg_->navigation_allowed && msg_->limits.transport_ready && clock_->now()<rclcpp::Time(msg_->valid_until);
  }
  void cleanup() {timer_.reset();sub_.reset();pub_.reset();std::lock_guard<std::mutex> lock(mutex_);msg_.reset();map_.reset();}
private:
  bool installed() {
    if(!msg_ || msg_->mode!=Message::FIXED_POSTURE || msg_->header.frame_id!=map_->getBaseFrameID() ||
       msg_->coordinator_session_id.empty() || msg_->installed_geometry_hash.empty()) return false;
    double age=(clock_->now()-rclcpp::Time(msg_->header.stamp)).seconds();
    if(age<0 || age>.3 || std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>.5) return false;
    auto actual=map_->getRobotFootprint();const auto & wanted=msg_->installed_footprint.points;
    if(!astribot_s1_robot_geometry::convex(actual)||actual.size()!=wanted.size()) return false;
    for(auto & q:actual) {bool found=false;for(auto & w:wanted) if(std::hypot(q.x-w.x,q.y-w.y)<2e-6) found=true;if(!found) return false;}
    return true;
  }
  bool enabled_{false};std::string consumer_;std::mutex mutex_;
  Message::ConstSharedPtr msg_;std::chrono::steady_clock::time_point received_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<Status>::SharedPtr pub_;rclcpp::Subscription<Message>::SharedPtr sub_;rclcpp::TimerBase::SharedPtr timer_;
};
}
