#pragma once
#include <moveit_msgs/msg/planning_scene.hpp>
#include <string>
#include <utility>
namespace astribot::transport {
// Joint samples change during execution and are checked against expected_start
// separately. All collision geometry, attachments, ACM and fixed transforms bind
// this first-stage plan; occupancy probabilities use the existing MTC kernel.
using SceneBinding=std::pair<moveit_msgs::msg::PlanningScene,std::string>;
SceneBinding bind_scene(moveit_msgs::msg::PlanningScene scene);
}
