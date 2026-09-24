#pragma once
#include <cmath>
#include <limits>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "collision_validator.hpp"
#include "singularity_monitor.hpp"
#include "trajectory_time_optimizer.hpp"

namespace astribot_s1_manipulation {
// Read-only geometry revalidation of the bound path. The existing position
// interpolation checks do not prove the controller spline's swept volume.
inline bool validateExternalTrajectoryGeometry(
  const planning_scene::PlanningSceneConstPtr& scene,
  const robot_trajectory::RobotTrajectory& trajectory, std::string& error)
{
  const auto* group = trajectory.getGroup();
  if (!group || trajectory.getWayPointCount() == 0) {error="EMPTY_EXTERNAL_TRAJECTORY"; return false;}
  std::vector<moveit::core::RobotState> states;
  for (size_t i=0; i<trajectory.getWayPointCount(); ++i) {
    const auto& state=trajectory.getWayPoint(i);
    for(auto idx:group->getVariableIndexList())
      if(!std::isfinite(state.getVariablePositions()[idx])) {error="INVALID_EXTERNAL_JOINT_VALUE"; return false;}
    if (!state.satisfiesBounds(group,1e-6)) {error="EXTERNAL_JOINT_LIMIT"; return false;}
    if (i) {
      const auto& previous=trajectory.getWayPoint(i-1);
      double step=0.;
      for (auto idx:group->getVariableIndexList()) step=std::max(step,std::abs(state.getVariablePositions()[idx]-previous.getVariablePositions()[idx]));
      if (!std::isfinite(step) || step>100.) {error="INVALID_EXTERNAL_JOINT_VALUE"; return false;}
      const int count=std::max(1,int(std::ceil(step/.025)));
      for(int j=1;j<count;++j) {
        moveit::core::RobotState sample(previous);
        previous.interpolate(state,double(j)/count,sample);sample.update();states.push_back(sample);
      }
    }
    states.push_back(state);
    if(states.size()>10000) {error="EXTERNAL_PATH_TOO_LARGE"; return false;}
  }
  CollisionValidator collision;
  if(!collision.configure(scene,CollisionParams{},error)) return false;
  auto report=collision.checkStates(states,group->getName());
  if(report.collision) {error="EXTERNAL_COLLISION:"+report.reason;return false;}
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
