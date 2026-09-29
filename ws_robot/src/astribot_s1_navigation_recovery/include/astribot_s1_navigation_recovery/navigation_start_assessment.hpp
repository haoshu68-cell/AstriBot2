#pragma once
#include "astribot_s1_navigation_recovery/departure_path.hpp"
#include "astribot_s1_path_tracking/envelope_guard.hpp"
#include "astribot_s1_path_tracking/layered_collision_reader.hpp"
#include <astribot_navigation_msgs/srv/assess_navigation_start.hpp>

namespace astribot_s1_navigation_recovery {
using namespace astribot_s1_path_tracking;
// Geometry query, collocated with the planner server that owns the global
// costmap. Fixed geometry facts are independent of installation/authority ACKs;
// all collision checks use one snapshot's envelope. Recovery decisions/alarms
// belong to the BT, never to this service.
class NavigationStartAssessment {
public:
  using Service=astribot_navigation_msgs::srv::AssessNavigationStart;
  NavigationStartAssessment(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map,EnvelopeGuard &guard,LayeredCollisionReader &reader);
private:
  void assess(const Service::Request &,Service::Response &);
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  EnvelopeGuard &guard_;LayeredCollisionReader &reader_;
  rclcpp::Service<Service>::SharedPtr service_;
};
} // namespace astribot_s1_navigation_recovery
