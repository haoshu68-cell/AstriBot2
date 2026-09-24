#include "astribot_s1_transport_mtc/canonical_scene.hpp"
#include <stdexcept>

namespace astribot::transport {
moveit_msgs::msg::PlanningScene canonicalScene(
  moveit_msgs::msg::PlanningScene scene, const std::string& planning_frame,
  const std::set<std::string>& known_links) {
  if(scene.is_diff || scene.robot_state.is_diff)
    throw std::runtime_error("MTC_FULL_SCENE_REQUIRED");
  if(planning_frame!="astribot_torso_base")
    throw std::runtime_error("MTC_FIXED_BASE_FRAME_REQUIRED");
  const auto& multi=scene.robot_state.multi_dof_joint_state;
  if(!multi.joint_names.empty() || !multi.transforms.empty() || !multi.twist.empty() || !multi.wrench.empty())
    throw std::runtime_error("MTC_MULTIDOF_STATE_UNSUPPORTED");
  for(const auto& object:scene.world.collision_objects)
    if(object.header.frame_id!=planning_frame)
      throw std::runtime_error("MTC_WORLD_FRAME_UNSUPPORTED:"+object.id);
  const auto& map=scene.world.octomap;
  if(map.header.frame_id!=planning_frame ||
     (!map.octomap.header.frame_id.empty() && map.octomap.header.frame_id!=planning_frame))
    throw std::runtime_error("MTC_OCTOMAP_FRAME_UNSUPPORTED");
  for(const auto& object:scene.robot_state.attached_collision_objects) {
    if(!known_links.count(object.link_name))
      throw std::runtime_error("MTC_ATTACHED_LINK_UNKNOWN:"+object.link_name);
    if(object.object.header.frame_id!=object.link_name)
      throw std::runtime_error("MTC_ATTACHED_FRAME_UNSUPPORTED:"+object.object.id);
  }
  scene.fixed_frame_transforms.clear();
  return scene;
}
}
