#pragma once
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <astribot_transport_msgs/srv/revalidate_payload_transition.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace astribot_s1_transport_mtc {
struct CachedStage {
  std::string id;
  planning_scene::PlanningSceneConstPtr scene;
  robot_trajectory::RobotTrajectoryPtr trajectory;
};

struct PayloadTransitionBinding {
  std::string object_id;
  std::string operation;
  moveit_msgs::msg::PlanningScene input_scene;
  std::string transaction_id;
  moveit_msgs::msg::PlanningScene confirmed_scene;
  bool transport_replanned{false};
};

using TransportReplanner = std::function<robot_trajectory::RobotTrajectoryPtr(
  const planning_scene::PlanningSceneConstPtr&, const moveit::core::RobotState&,
  const moveit::core::JointModelGroup*, std::chrono::steady_clock::time_point)>;

struct PayloadTransitionResult {
  std::vector<CachedStage> stages;
  bool transport_replanned{false};
  std::chrono::steady_clock::time_point deadline;
};

// Builds a separate candidate cache. Throws on any rejected scene/path; the
// caller commits the returned stages and transaction only after full success.
// Only a first PICK transaction may replace a colliding TRANSPORT_POSTURE after
// the actual-body LIFT passes. Every other path retains its original timing.
PayloadTransitionResult revalidatePayloadTransition(
  const astribot_transport_msgs::srv::RevalidatePayloadTransition::Request& request,
  const std::string& cached_context, const PayloadTransitionBinding& binding,
  const std::vector<CachedStage>& stages,
  std::chrono::steady_clock::time_point cached_at,
  double velocity_scaling, double acceleration_scaling,
  const TransportReplanner& replan_transport = {},
  double time_scaling = 1., double joint_margin = .1);
}
