#pragma once
#include <moveit_msgs/msg/planning_scene.hpp>
#include <string>
#include <set>
#include <utility>
namespace astribot::transport {
// Joint samples change during execution and are checked against expected_start
// separately. Canonical collision geometry, attachments and ACM bind this plan;
// occupancy probabilities use the existing MTC kernel. External frames are not
// supported, and attachments require the caller's actual model link names.
using SceneBinding=std::pair<moveit_msgs::msg::PlanningScene,std::string>;
SceneBinding bind_scene(moveit_msgs::msg::PlanningScene scene,
                        const std::set<std::string>& known_links = {});
}
