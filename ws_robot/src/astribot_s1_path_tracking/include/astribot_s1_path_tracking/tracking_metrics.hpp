// Copyright 2026 Astribot
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include "nav_msgs/msg/path.hpp"

namespace astribot_s1_path_tracking
{
struct TrackingProjection
{
  double cross_track_m, tangent_rad, progress_m, curvature;
};

// Diagnostic geometry follows tools/run_waypoint_route.py; it never alters the path.
inline std::optional<TrackingProjection> projectTrackingPose(
  const nav_msgs::msg::Path & path, double x, double y,
  std::optional<double> anchor = std::nullopt)
{
  std::optional<TrackingProjection> result;
  double arc = 0.0, best = std::numeric_limits<double>::infinity();
  const auto & points = path.poses;
  auto separation = [&](size_t a, size_t b) {
      return std::hypot(points[a].pose.position.x - points[b].pose.position.x,
        points[a].pose.position.y - points[b].pose.position.y);
    };
  auto heading = [&](size_t a, size_t b) {
      return std::atan2(points[b].pose.position.y - points[a].pose.position.y,
        points[b].pose.position.x - points[a].pose.position.x);
    };
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & a = points[i].pose.position;
    const auto & b = points[i+1].pose.position;
    const double dx = b.x-a.x, dy = b.y-a.y, length = std::hypot(dx, dy);
    if (length < 1e-8) {continue;}
    if (anchor && (arc < *anchor-1.0 || arc > *anchor+3.0)) {arc += length; continue;}
    const double f = std::clamp(((x-a.x)*dx+(y-a.y)*dy)/(length*length), 0.0, 1.0);
    const double px = a.x+f*dx, py = a.y+f*dy;
    const double distance = std::hypot(x-px, y-py);
    if (distance < best) {
      const double tangent = std::atan2(dy, dx);
      size_t lo = i, hi = i+1;
      while (lo > 0 && separation(lo, i) < 0.25) {--lo;}
      while (hi+1 < points.size() && separation(hi, i+1) < 0.25) {++hi;}
      const double left = lo < i ? heading(lo, i) : tangent;
      const double right = hi > i+1 ? heading(i+1, hi) : tangent;
      const double curvature = std::remainder(right-left, 2*M_PI) /
        std::max(0.1, separation(lo, hi));
      const double sign = dx*(y-a.y)-dy*(x-a.x) >= 0.0 ? 1.0 : -1.0;
      result = TrackingProjection{sign*distance, tangent, arc+f*length, curvature};
      best = distance;
    }
    arc += length;
  }
  return result;
}

class MetricSamplingClock
{
public:
  bool due(double now, double rate_hz)
  {
    if (rate_hz <= 0.0) {return false;}
    if (last_ && now >= *last_ && now-*last_+1e-9 < 1.0/rate_hz) {return false;}
    last_ = now;
    return true;
  }
private:
  std::optional<double> last_;
};
}  // namespace astribot_s1_path_tracking
