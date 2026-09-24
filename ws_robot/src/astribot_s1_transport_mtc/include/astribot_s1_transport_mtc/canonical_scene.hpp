#pragma once
#include <moveit_msgs/msg/planning_scene.hpp>
#include <set>
#include <string>

namespace astribot::transport {
// Fixed-base transport accepts world/map geometry in the planning frame and
// attached geometry local to a known model link. Under that contract no external
// fixed transform is consumed, so remove it from both planning and comparison.
// Preserve every other field; callers apply their existing signature rules.
moveit_msgs::msg::PlanningScene canonicalScene(
  moveit_msgs::msg::PlanningScene scene, const std::string& planning_frame,
  const std::set<std::string>& known_links);
}
