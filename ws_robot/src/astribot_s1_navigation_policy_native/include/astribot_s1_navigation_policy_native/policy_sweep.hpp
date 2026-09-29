#pragma once

#include <array>
#include <optional>
#include <vector>

namespace astribot::navigation::policy {
using Point = std::array<double, 2>;
using Polygon = std::vector<Point>;
using PlanarPose = std::array<double, 3>;
struct Bounds { Point lower, upper; };
// Values are copied from the validated envelope at a decision boundary.
struct SweepProfile {
  double half_length_m, half_width_m, clearance_margin_m, payload_extra_margin_m;
  std::optional<Polygon> footprint_xy;
};

// The adapter expands broadcast inputs before entering these typed row APIs.
std::vector<double> clearance_many(const std::vector<double>& x,
  const std::vector<double>& y, const std::vector<double>& yaw,
  const std::vector<Bounds>& boxes, const SweepProfile& profile,
  double sampling_margin = 0.);
std::vector<double> motion_clearance(const PlanarPose& command,
  const std::vector<double>& begin, const std::vector<double>& end,
  const std::vector<Bounds>& boxes, const SweepProfile& profile,
  const PlanarPose& origin = {0., 0., 0.});
}  // namespace astribot::navigation::policy
