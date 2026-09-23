#pragma once
#include "astribot_s1_gazebo_bringup/kinematic_inventory.hpp"
#include <moveit_msgs/msg/attached_collision_object.hpp>
namespace astribot::simulation {
ignition::math::Pose3d fixed_parent_from_tcp(const std::string &urdf,const std::string &parent,const std::string &tcp);
moveit_msgs::msg::AttachedCollisionObject observed_payload(const ignition::gazebo::EntityComponentManager &,uint64_t model,
  const PayloadExecution &,const std::string &object_id,const std::string &tcp,const ignition::math::Pose3d &parent_from_tcp);
}
