#pragma once
#include <astribot_s1_manipulation_perception/pick_planning_client.hpp>
#include <astribot_s1_manipulation/gripper_commander.hpp>

namespace astribot::perception_planning {
// Restricted candidate geometry, NOT a grasp acceptance or contact certificate.
// GripperCommander must be configured with the same RobotModel and left-gripper
// defaults as the MTC deployment (including preload). Own it outside this library.
// Requires one registered BOX, face-aligned grasp axes, supported model depth,
// and a box centered between the native fingers. Rejects unsupported candidates.
// Reads actual pad mesh/FK; calls only the commander's width inversion/read API.
GraspMapping map_known_box_grasp(
  const moveit_msgs::msg::CollisionObject &registered_geometry,
  const Candidate &, const Pose::Result &,
  const moveit::core::RobotModelConstPtr &,
  const astribot_s1_manipulation::GripperCommander &);
} // namespace astribot::perception_planning
