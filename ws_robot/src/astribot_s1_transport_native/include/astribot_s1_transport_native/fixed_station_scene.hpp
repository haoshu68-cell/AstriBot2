#pragma once
#include <array>
#include <set>
#include <string>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace astribot::transport {
struct FixedStationObservation {
  std::string id;
  geometry_msgs::msg::Pose world_pose;
  std::array<double, 3> size_xyz;
};
using FixedStations = std::array<FixedStationObservation, 2>;

// The caller binds fresh, same-capture Gazebo model observations and
// PayloadFrames::world_from_base to its stopped, navigation-revoked transaction.
// Gazebo world is not MoveIt's fixed world. Only these two stations are changed.
// Each registered BOX is centered and aligned at its physical model origin.
moveit_msgs::msg::PlanningScene fixed_station_scene_diff(
  const moveit_msgs::msg::PlanningScene &before, const FixedStations &stations,
  const Eigen::Isometry3d &world_from_base, const std::set<std::string> &known_links);

void validate_fixed_station_scene(
  const moveit_msgs::msg::PlanningScene &before,
  const moveit_msgs::msg::PlanningScene &after,
  const moveit_msgs::msg::PlanningScene &diff,
  const std::set<std::string> &known_links);

// Converts an already determined world target; does not infer a grasp offset
// or alter the actual attached object's TCP transform. Output frame is torso_base.
geometry_msgs::msg::Pose fixed_station_target_in_base(
  const geometry_msgs::msg::Pose &world_target,
  const Eigen::Isometry3d &world_from_base);
}
