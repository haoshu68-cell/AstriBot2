#pragma once
#include <stdexcept>

namespace astribot_s1_transport_mtc {
struct BaseMotionLimits {
  double translation_m{.02};
  double rotation_rad{.02};
  double linear_speed_m_s{.02};
  double angular_speed_rad_s{.03};
};

inline BaseMotionLimits baseMotionLimits(bool relaxed, bool simulated) {
  if (relaxed && !simulated) {
    throw std::invalid_argument("SIMULATION_RELAXED_BASE_MOTION_REQUIRES_USE_SIM_TIME");
  }
  BaseMotionLimits limits;
  if (relaxed) {
    limits.rotation_rad = .05;
    limits.angular_speed_rad_s = .10;
  }
  return limits;
}
}
