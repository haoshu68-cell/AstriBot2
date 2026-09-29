#include "astribot_trajectory_bridge_native/arm_math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astribot_trajectory_bridge_native {
namespace {

double lerp(double a, double b, double s) { return a + (b - a) * s; }

double cubic_hermite(double p0, double v0, double p1, double v1, double s,
                     double segment_dt) {
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  return h00 * p0 + h10 * segment_dt * v0 + h01 * p1 + h11 * segment_dt * v1;
}

}  // namespace

JointVector interpolate_trajectory(const std::vector<double> &times,
                                   const JointTrajectory &positions,
                                   const JointTrajectory &velocities,
                                   double t, bool use_cubic) {
  if (times.empty() || positions.empty() || times.size() != positions.size()) {
    throw std::invalid_argument("invalid trajectory dimensions");
  }
  if (t <= times.front()) return positions.front();
  if (t >= times.back()) return positions.back();
  std::size_t hi = 1;
  while (hi < times.size() && t > times[hi]) ++hi;
  const std::size_t lo = hi - 1;
  const double segment_dt = times[hi] - times[lo];
  const double s = (t - times[lo]) / segment_dt;
  JointVector result;
  result.reserve(positions[lo].size());
  for (std::size_t j = 0; j < positions[lo].size(); ++j) {
    if (use_cubic) {
      result.push_back(cubic_hermite(positions[lo][j], velocities[lo][j],
                                     positions[hi][j], velocities[hi][j], s,
                                     segment_dt));
    } else {
      result.push_back(lerp(positions[lo][j], positions[hi][j], s));
    }
  }
  return result;
}

double max_abs_error(const JointVector &actual, const JointVector &target) {
  if (actual.size() != target.size()) {
    throw std::invalid_argument("joint vector dimensions differ");
  }
  double worst = 0.0;
  for (std::size_t i = 0; i < actual.size(); ++i) {
    worst = std::max(worst, std::abs(actual[i] - target[i]));
  }
  return worst;
}

double clamp_cmd(double cmd) { return std::max(0.0, std::min(100.0, cmd)); }

bool is_cmd_in_range(double cmd) { return cmd >= 0.0 && cmd <= 100.0; }

double cmd_to_rad(double cmd, bool clamp) {
  return (clamp ? clamp_cmd(cmd) : cmd) * 0.0093;
}

double rad_to_cmd(double rad, bool clamp) {
  const double cmd = rad / 0.0093;
  return clamp ? clamp_cmd(cmd) : cmd;
}

double opening_fraction_to_cmd(double fraction) {
  const double f = std::max(0.0, std::min(1.0, fraction));
  return (1.0 - f) * 100.0;
}

double cmd_to_opening_fraction(double cmd) { return 1.0 - clamp_cmd(cmd) / 100.0; }

double horizontal_reach(double x, double y) { return std::hypot(x, y); }

double reach_activity(double max_reach, double reach_folded, double reach_full) {
  const double ratio = (max_reach - reach_folded) / (reach_full - reach_folded);
  return std::max(0.0, std::min(1.0, ratio));
}

double scale_from_activity(double activity, double min_speed_scale) {
  const double raw = 1.0 - activity * (1.0 - min_speed_scale);
  return std::max(min_speed_scale, std::min(1.0, raw));
}

bool is_extended_by_reach(double max_reach, double extended_reach,
                          double hysteresis, bool was_extended) {
  return max_reach > (extended_reach - (was_extended ? hysteresis : 0.0));
}

}  // namespace astribot_trajectory_bridge_native
