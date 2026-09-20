#pragma once
#include <cmath>
#include <stdexcept>

namespace astribot_s1_transport_mtc {
struct JointInterval {
  double lower, upper;
  bool contains(double position) const {
    return std::isfinite(position) && position >= lower - 1e-9 && position <= upper + 1e-9;
  }
};
inline JointInterval planningInterval(double lower, double upper, double margin) {
  if (!std::isfinite(lower) || !std::isfinite(upper) || !std::isfinite(margin) ||
      margin <= 0. || margin > .2 || upper - lower <= 2. * margin)
    throw std::invalid_argument("INVALID_JOINT_PLANNING_MARGIN");
  return {lower + margin, upper - margin};
}
}
