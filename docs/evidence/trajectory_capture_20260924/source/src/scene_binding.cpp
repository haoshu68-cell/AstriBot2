#include "astribot_s1_transport_native/scene_binding.hpp"
#include <astribot_s1_transport_mtc/canonical_octomap.hpp>
#include <astribot_s1_transport_mtc/canonical_scene.hpp>
#include <algorithm>
#include <stdexcept>
namespace astribot::transport {
SceneBinding bind_scene(moveit_msgs::msg::PlanningScene scene,
                        const std::set<std::string>& known_links) {
 scene=canonicalScene(std::move(scene),"astribot_torso_base",known_links);
 scene.robot_state.joint_state=sensor_msgs::msg::JointState();scene.robot_state.multi_dof_joint_state=sensor_msgs::msg::MultiDOFJointState();
 auto &objects=scene.world.collision_objects;
 std::sort(objects.begin(),objects.end(),[](const auto&a,const auto&b){return a.id<b.id;});
 for(auto &object:objects)object.header.stamp=builtin_interfaces::msg::Time();
 auto &attached=scene.robot_state.attached_collision_objects;
 std::sort(attached.begin(),attached.end(),[](const auto&a,const auto&b){return a.object.id<b.object.id;});
 for(auto &object:attached)object.object.header.stamp=builtin_interfaces::msg::Time();
 auto &octomap=scene.world.octomap.octomap;std::string occupancy;
 if(!octomap.data.empty())occupancy=canonical_octomap(std::string(octomap.data.begin(),octomap.data.end()),octomap.binary,octomap.resolution,octomap.id);
 octomap.data.clear();return {std::move(scene),std::move(occupancy)};
}
}
