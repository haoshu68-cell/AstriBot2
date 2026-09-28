#pragma once

#include <string>
#include <limits>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>

namespace astribot_s1_manipulation {

// Samples the same linear/cubic/quintic position curve as JTC. Each accepted
// interval has a Bernstein-hull joint span <= maximum_joint_span. This bounds
// joint interpolation, not the Cartesian swept volume between samples.
bool sampleControllerTrajectory(
  const robot_trajectory::RobotTrajectory& trajectory,
  double maximum_joint_span, std::size_t maximum_samples,
  std::vector<moveit::core::RobotState>& states, std::string& error);

enum class SweepVerdict {CLEAR, RISK, UNKNOWN};
struct ControllerSweepResult {
  SweepVerdict verdict{SweepVerdict::UNKNOWN};
  std::string reason;
  std::size_t checked_states{0};
  // Only CLEAR provides a certified bound within the supplied fixed scene.
  double clearance_lower_bound{std::numeric_limits<double>::quiet_NaN()};
};

// Offline/shadow check of the whole explicit spline, including attached bodies
// and the other arm. Bernstein hulls plus kinematic displacement bounds certify
// each interval against full-robot distance queries. This does not certify
// perception coverage, scene freshness, the measured-to-first-point transition,
// moving obstacles, tracking error or the physical stopping envelope.
ControllerSweepResult checkControllerSweep(
  const planning_scene::PlanningSceneConstPtr& scene,
  const robot_trajectory::RobotTrajectory& trajectory,
  double clearance_m, std::size_t maximum_states, double budget_seconds);

}  // namespace astribot_s1_manipulation
