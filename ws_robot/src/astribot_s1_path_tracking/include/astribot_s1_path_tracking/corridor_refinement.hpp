#pragma once
#include <algorithm>
#include <cmath>

namespace astribot_s1_path_tracking {
inline bool directCandidateHeadingCompatible(double pose_heading,double tangent,bool fixed_geometry) {
  // Candidate generation uses the fixed-posture approach heading window.
  // Execution still needs the full sweep, source leases and corridor permit.
  // Legacy generation retains its original one-degree window.
  return std::isfinite(pose_heading) && std::isfinite(tangent) &&
    std::abs(std::remainder(pose_heading-tangent,2*M_PI))<=(fixed_geometry ? .05 : M_PI/180.);
}
// A small heading error can still be corrected while approaching the goal.
// A zero command is not a settled pose. Only a fresh, stopped observation
// window can make the terminal heading residual an unreachable result.
inline bool corridorRefinementReachable(double goal_axis_error,double yaw_error,
  bool terminal_settled,double yaw_tolerance,double axis_tolerance=.05) {
  return std::isfinite(goal_axis_error) && std::isfinite(yaw_error) &&
    std::isfinite(yaw_tolerance) && yaw_tolerance>0. &&
    std::isfinite(axis_tolerance) && axis_tolerance>0. &&
    std::abs(goal_axis_error)<=axis_tolerance &&
    (!terminal_settled || std::abs(yaw_error)<=yaw_tolerance);
}
inline bool corridorTerminalWithinBounds(double xy,double yaw,double stop_xy,double stop_yaw,
    double xy_tolerance,double yaw_tolerance) {
  return std::isfinite(xy) && std::isfinite(yaw) && std::isfinite(stop_xy) && std::isfinite(stop_yaw) &&
    std::isfinite(xy_tolerance) && std::isfinite(yaw_tolerance) && xy>=0. && stop_xy>=0. && stop_yaw>=0. &&
    xy_tolerance>0. && yaw_tolerance>0. && xy+stop_xy<=xy_tolerance && std::abs(yaw)+stop_yaw<=yaw_tolerance;
}
inline bool corridorAngularCorrectionAllowed(double commanded_speed,double measured_speed) {
  return std::isfinite(commanded_speed) && std::isfinite(measured_speed) &&
    commanded_speed>=.005 && measured_speed>=.003;
}
inline double corridorApproachHeadingGain(double distance,double commanded_speed,
    double measured_speed,double base_gain,double position_tolerance) {
  // Finish heading convergence before the translational stop. A constant yaw
  // gain has no knowledge of the remaining travel and can consume that travel
  // before reaching the heading bound. The 2*v/d term gives quadratic error
  // decay with distance in the ideal translating model. Keep a bounded gain;
  // measured braking, angular caps, motion gating and swept checks still apply.
  if(!std::isfinite(distance) || !std::isfinite(commanded_speed) ||
      !std::isfinite(measured_speed) || !std::isfinite(base_gain) ||
      !std::isfinite(position_tolerance) || distance<0. || commanded_speed<0. ||
      measured_speed<0. || base_gain<=0. || position_tolerance<=0.) {
    return base_gain;
  }
  const double available=std::max(position_tolerance,distance-position_tolerance);
  return std::max(base_gain,std::min(3.,2.*std::max(commanded_speed,measured_speed)/available));
}
}
