#ifndef ASTRIBOT_S1_PATH_TRACKING__CORRIDOR_LEASE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__CORRIDOR_LEASE_HPP_

#include <cmath>

namespace astribot_s1_path_tracking
{
inline bool corridorRequestFresh(bool simulated, double age, double wall_age, double lease)
{
  // The final constraint and command writer retain independent wall watchdogs.
  // A ROS-time corridor timer must age its request in the same physical time.
  return std::isfinite(age) && std::isfinite(wall_age) && std::isfinite(lease) &&
         lease > 0. && lease <= .3 && age >= 0. && age <= lease && wall_age >= 0. &&
         (simulated || wall_age <= lease);
}
}
#endif
