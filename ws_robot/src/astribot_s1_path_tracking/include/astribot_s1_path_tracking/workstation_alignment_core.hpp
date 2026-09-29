#pragma once
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "astribot_s1_path_tracking/layered_collision.hpp"

namespace astribot_s1_path_tracking {
struct WorkstationPose {double x{},y{},yaw{};};
struct WorkstationMotionState {
  std::array<double,3> velocity{},acceleration{};
  bool xy_held{false},yaw_held{false};
};
struct WorkstationConfig {
  double max_linear_speed{.06},max_angular_speed{.15},kp_xy{.5},kp_yaw{.8};
  double min_linear_speed{},min_angular_speed{};
  double normal_acceleration{.25},normal_jerk{.5};
  double normal_angular_acceleration{.6},normal_angular_jerk{1.2};
  double xy_goal_tolerance{.002},yaw_goal_tolerance{.0017453292519943296};
  double stopped_linear_velocity{.001},stopped_angular_velocity{.001};
  double settle_time{.6},settle_drift_ratio{.2};
  double stop_tolerance_ratio{.65},resume_tolerance_ratio{.9};
  double refine_timeout{135.},control_frequency{20.};
};
// Both planning BT and controller read the same launch-injected namespace.
template<class Node>
WorkstationConfig loadWorkstationConfig(const std::shared_ptr<Node>& node,
    const std::string& prefix="WorkstationAlign") {
  WorkstationConfig cfg;
  const auto read=[&](const char *key,double &value,bool allow_zero=false) {
    const auto name=prefix+"."+key;
    if(!node->has_parameter(name))node->declare_parameter(name,value);
    value=node->get_parameter(name).as_double();
    if(!std::isfinite(value)||value<0.||(!allow_zero&&value==0.))
      throw std::invalid_argument("WORKSTATION_INVALID_PARAMETER:"+name);
  };
#define WORKSTATION_READ(field) read(#field,cfg.field)
  WORKSTATION_READ(max_linear_speed);WORKSTATION_READ(max_angular_speed);
  read("min_linear_speed",cfg.min_linear_speed,true);
  read("min_angular_speed",cfg.min_angular_speed,true);
  WORKSTATION_READ(kp_xy);WORKSTATION_READ(kp_yaw);
  WORKSTATION_READ(normal_acceleration);WORKSTATION_READ(normal_jerk);
  WORKSTATION_READ(normal_angular_acceleration);WORKSTATION_READ(normal_angular_jerk);
  WORKSTATION_READ(xy_goal_tolerance);WORKSTATION_READ(yaw_goal_tolerance);
  WORKSTATION_READ(stopped_linear_velocity);WORKSTATION_READ(stopped_angular_velocity);
  WORKSTATION_READ(settle_time);WORKSTATION_READ(settle_drift_ratio);
  WORKSTATION_READ(stop_tolerance_ratio);WORKSTATION_READ(resume_tolerance_ratio);
  WORKSTATION_READ(refine_timeout);WORKSTATION_READ(control_frequency);
#undef WORKSTATION_READ
  if(cfg.min_linear_speed>cfg.max_linear_speed||cfg.min_angular_speed>cfg.max_angular_speed)
    throw std::invalid_argument("WORKSTATION_MINIMUM_SPEED_EXCEEDS_MAXIMUM");
  if(cfg.stop_tolerance_ratio>cfg.resume_tolerance_ratio||cfg.resume_tolerance_ratio>1.||
      cfg.settle_drift_ratio>1.)throw std::invalid_argument("WORKSTATION_INVALID_SETTLING_RATIOS");
  return cfg;
}
struct WorkstationEvaluation {
  bool clear{false};std::string reason;double prediction_s{};std::size_t steps{};
};
std::array<double,3> workstationCommand(const WorkstationPose&,const WorkstationPose&,
  WorkstationMotionState&,const WorkstationConfig&,double dt,bool stopping=false);
WorkstationPose workstationIntegrate(const WorkstationPose&,const std::array<double,3>&,double dt);
bool workstationStoppingClear(const LayeredCollisionSnapshot&,const WorkstationPose&,
  WorkstationMotionState,const WorkstationConfig&);
WorkstationEvaluation evaluateWorkstationAlignment(const LayeredCollisionSnapshot&,
  const WorkstationPose&,const WorkstationPose&,const WorkstationConfig&);
} // namespace astribot_s1_path_tracking
