#pragma once
#include <cmath>
#include <limits>
#include <sstream>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <rclcpp/logging.hpp>
#include "collision_validator.hpp"
#include "controller_trajectory.hpp"
#include "singularity_monitor.hpp"
#include "trajectory_time_optimizer.hpp"

namespace astribot_s1_manipulation {
// Read-only revalidation of the actual controller spline. The bounded joint
// sampling is not a proof of the Cartesian swept volume between samples.
inline bool validateExternalTrajectoryGeometry(
  const planning_scene::PlanningSceneConstPtr& scene,
  const robot_trajectory::RobotTrajectory& trajectory, std::string& error)
{
  const auto* group = trajectory.getGroup();
  if (!group || trajectory.getWayPointCount() == 0) {error="EMPTY_EXTERNAL_TRAJECTORY"; return false;}
  std::vector<moveit::core::RobotState> states;
  if (!sampleControllerTrajectory(trajectory,.025,10000,states,error)) return false;
  for (const auto& state:states) {
    for(auto idx:group->getVariableIndexList())
      if(!std::isfinite(state.getVariablePositions()[idx])) {error="INVALID_EXTERNAL_JOINT_VALUE"; return false;}
    if (!state.satisfiesBounds(group,1e-6)) {error="EXTERNAL_JOINT_LIMIT"; return false;}
  }
  CollisionValidator collision;
  if(!collision.configure(scene,CollisionParams{},error)) return false;
  size_t first_bad_index=0;
  auto report=collision.checkStates(states,group->getName(),&first_bad_index);
  if(report.collision) {
    error="EXTERNAL_COLLISION:"+report.reason;
    moveit_msgs::msg::RobotState first,bad,last;
    moveit::core::robotStateToRobotStateMsg(states.front(),first,true);
    moveit::core::robotStateToRobotStateMsg(states[first_bad_index],bad,true);
    moveit::core::robotStateToRobotStateMsg(states.back(),last,true);
    std::ostringstream snapshots;snapshots.precision(std::numeric_limits<double>::max_digits10);
    snapshots<<"first=";moveit_msgs::msg::to_flow_style_yaml(first,snapshots);
    snapshots<<"\nbad=";moveit_msgs::msg::to_flow_style_yaml(bad,snapshots);
    snapshots<<"\nlast=";moveit_msgs::msg::to_flow_style_yaml(last,snapshots);
    RCLCPP_ERROR(rclcpp::get_logger("astribot_s1_manipulation.external_trajectory"),
      "EXTERNAL_COLLISION_SNAPSHOT frame=%s group=%s first_bad_index=%zu checked_states=%zu trajectory_waypoints=%zu reason=%s\n%s",
      scene->getPlanningFrame().c_str(),group->getName().c_str(),first_bad_index,states.size(),trajectory.getWayPointCount(),
      report.reason.c_str(),snapshots.str().c_str());
    return false;
  }
  if(group->getName()=="arm_left" || group->getName()=="arm_right") {
    SingularityMonitor singularity;
    if(!singularity.configure(SingularityParams{},error)) return false;
    auto singular=singularity.checkStates(states,group,"astribot_"+group->getName()+"_tcp_link");
    if(!singular.valid || singular.singular) {error="EXTERNAL_SINGULARITY:"+singular.reason;return false;}
  }
  return true;
}

// Parameterize once before validation, serialization and cache binding. Uniform
// time scaling preserves the position curve, including JTC derivative splines.
inline bool validateExternalTrajectory(
  const planning_scene::PlanningSceneConstPtr& scene,
  robot_trajectory::RobotTrajectory& trajectory, std::string& error,
  double velocity_scaling=0.3, double acceleration_scaling=0.3,
  double time_scaling=1.)
{
  if(!std::isfinite(time_scaling) || time_scaling<1.) {error="INVALID_EXTERNAL_TIME_SCALING";return false;}
  if(!trajectory.getGroup() || trajectory.getWayPointCount()==0) {error="EMPTY_EXTERNAL_TRAJECTORY";return false;}
  TrajectoryTimeOptimizer timing;
  TimeOptimizerParams params;params.enable_optimization=false;
  params.baseline_velocity_scaling=velocity_scaling;
  params.baseline_acceleration_scaling=acceleration_scaling;
  OptimizationResult metrics;
  if(!timing.configure(params,error) || timing.optimize(trajectory,metrics)!=PlanErrorCode::kSuccess) {
    error="EXTERNAL_TIME_PARAMETERIZATION:"+error;return false;
  }
  if(time_scaling!=1.) {
    double duration=0.;
    for(size_t i=0;i<trajectory.getWayPointCount();++i) {
      duration+=trajectory.getWayPointDurationFromPrevious(i)*time_scaling;
      if(!std::isfinite(duration) || duration>std::numeric_limits<int32_t>::max()) {
        error="EXTERNAL_TIME_SCALING_DURATION_OVERFLOW";return false;
      }
    }
    for(size_t i=0;i<trajectory.getWayPointCount();++i) {
      trajectory.setWayPointDurationFromPrevious(i,trajectory.getWayPointDurationFromPrevious(i)*time_scaling);
      auto& state=*trajectory.getWayPointPtr(i);
      for(size_t joint=0;joint<state.getVariableCount();++joint) {
        if(state.hasVelocities())state.getVariableVelocities()[joint]/=time_scaling;
        if(state.hasAccelerations())state.getVariableAccelerations()[joint]=
          state.getVariableAccelerations()[joint]/time_scaling/time_scaling;
      }
    }
  }
  return validateExternalTrajectoryGeometry(scene,trajectory,error);
}
}
