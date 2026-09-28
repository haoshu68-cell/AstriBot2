#pragma once

#include <string>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace astribot_s1_manipulation {

// Samples the same linear/cubic/quintic position curve as JTC. Each accepted
// interval has a Bernstein-hull joint span <= maximum_joint_span. This bounds
// joint interpolation, not the Cartesian swept volume between samples.
bool sampleControllerTrajectory(
  const robot_trajectory::RobotTrajectory& trajectory,
  double maximum_joint_span, std::size_t maximum_samples,
  std::vector<moveit::core::RobotState>& states, std::string& error);

}  // namespace astribot_s1_manipulation
