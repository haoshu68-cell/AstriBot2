#ifndef ASTRIBOT_S1_PATH_TRACKING__CORRIDOR_LEASE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__CORRIDOR_LEASE_HPP_

#include <cmath>

namespace astribot_s1_path_tracking
{
inline bool corridorRequestFresh(bool simulated, double age, double wall_age, double lease)
{
  (void)simulated;(void)age;(void)wall_age;
  return std::isfinite(lease)&&lease>0.&&lease<=.3;
}
}
#endif
