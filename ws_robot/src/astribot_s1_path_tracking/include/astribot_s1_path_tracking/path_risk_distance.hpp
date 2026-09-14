#pragma once
#include <algorithm>
#include <cstddef>
#include <cmath>
#include "nav_msgs/msg/path.hpp"

namespace astribot_s1_path_tracking {
inline double pathRiskDistance(const nav_msgs::msg::Path & path, size_t first,
  size_t blocked, double start_offset)
{
  if (first > blocked || blocked >= path.poses.size() ||
      !std::isfinite(start_offset) || start_offset < 0) {return 0.;}
  double distance=-start_offset;
  for (size_t i=first; i<blocked; ++i) {
    const auto & a=path.poses[i].pose.position;
    const auto & b=path.poses[i+1].pose.position;
    const double segment=std::hypot(b.x-a.x,b.y-a.y);
    if (!std::isfinite(segment)) {return 0.;}
    distance+=segment;
    if (!std::isfinite(distance)) {return 0.;}
  }
  return std::max(0.,distance);
}
}
