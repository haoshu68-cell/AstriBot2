#pragma once

#include <string>
#include <vector>

namespace astribot_trajectory_bridge_native {

using JointVector = std::vector<double>;
using JointTrajectory = std::vector<JointVector>;

JointVector interpolate_trajectory(const std::vector<double> &times,
                                   const JointTrajectory &positions,
                                   const JointTrajectory &velocities,
                                   double t, bool use_cubic);
double max_abs_error(const JointVector &actual, const JointVector &target);
double clamp_cmd(double cmd);
bool is_cmd_in_range(double cmd);
double cmd_to_rad(double cmd, bool clamp);
double rad_to_cmd(double rad, bool clamp);
double opening_fraction_to_cmd(double fraction);
double cmd_to_opening_fraction(double cmd);
double horizontal_reach(double x, double y);
double reach_activity(double max_reach, double reach_folded, double reach_full);
double scale_from_activity(double activity, double min_speed_scale);
bool is_extended_by_reach(double max_reach, double extended_reach,
                          double hysteresis, bool was_extended);

}  // namespace astribot_trajectory_bridge_native
