#include "astribot_s1_navigation_policy_native/policy_sweep.hpp"
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include "astribot_s1_navigation_policy_native/policy_numeric.hpp"
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace astribot::navigation::policy {
namespace {
std::pair<std::vector<double>, std::vector<double>> flatten(const std::vector<Bounds>& boxes) {
  std::vector<double> lower, upper;
  lower.reserve(2 * boxes.size()); upper.reserve(2 * boxes.size());
  for (const auto& b : boxes) {
    lower.insert(lower.end(), b.lower.begin(), b.lower.end());
    upper.insert(upper.end(), b.upper.begin(), b.upper.end());
  }
  return {std::move(lower), std::move(upper)};
}
bool finite(const PlanarPose& values) {
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}
}

std::vector<double> clearance_many(const std::vector<double>& x,
  const std::vector<double>& y, const std::vector<double>& yaw,
  const std::vector<Bounds>& boxes, const SweepProfile& profile, double sampling_margin) {
  if (x.size() != boxes.size() || y.size() != boxes.size() || yaw.size() != boxes.size())
    throw std::invalid_argument("clearance rows have inconsistent lengths");
  const double margin = profile.clearance_margin_m + profile.payload_extra_margin_m + sampling_margin;
  if (!profile.footprint_xy) {
    const auto [lower, upper] = flatten(boxes);
    return navigation::clearance_many_rect(x, y, yaw, lower, upper,
      profile.half_length_m, profile.half_width_m, margin);
  }
  const auto& footprint = *profile.footprint_xy;
  if (footprint.empty()) throw std::invalid_argument("empty footprint");
  double radius = 0.;
  for (const auto& p : footprint) radius = std::max(radius, std::sqrt(p[0]*p[0]+p[1]*p[1]));
  std::vector<double> result(boxes.size());
  std::vector<std::array<double, 7>> close;
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    const auto& b = boxes[i];
    const double dx = std::max({b.lower[0]-x[i], 0., x[i]-b.upper[0]});
    const double dy = std::max({b.lower[1]-y[i], 0., y[i]-b.upper[1]});
    const double bound = std::hypot(dx, dy) - radius;
    result[i] = bound - margin - 1e-12;
    if (bound <= margin + 1e-12) {
      indices.push_back(i);
      close.push_back({x[i], y[i], yaw[i], b.lower[0], b.lower[1], b.upper[0], b.upper[1]});
    }
  }
  if (!close.empty()) {
    const auto exact = astribot_s1_robot_geometry::boxDistances(footprint, close);
    for (std::size_t i = 0; i < close.size(); ++i) result[indices[i]] = exact[i] - margin - 1e-12;
  }
  return result;
}

std::vector<double> motion_clearance(const PlanarPose& command,
  const std::vector<double>& begin, const std::vector<double>& end,
  const std::vector<Bounds>& boxes, const SweepProfile& profile, const PlanarPose& origin) {
  if (!finite(command) || !finite(origin)) throw std::invalid_argument("finite motion required");
  if (begin.size() != end.size() || begin.size() != boxes.size())
    throw std::invalid_argument("sweep rows have inconsistent lengths");
  if (!profile.footprint_xy) {
    const auto [lower, upper] = flatten(boxes);
    return navigation::motion_clearance_rect({command.begin(), command.end()}, begin, end, lower, upper,
      profile.half_length_m, profile.half_width_m, profile.clearance_margin_m,
      profile.payload_extra_margin_m, {origin.begin(), origin.end()});
  }
  for (std::size_t i = 0; i < begin.size(); ++i) {
    const auto& b = boxes[i];
    if (!std::isfinite(begin[i]) || !std::isfinite(end[i]) || begin[i] < 0. || end[i] < begin[i] ||
      !std::isfinite(b.lower[0]) || !std::isfinite(b.lower[1]) || !std::isfinite(b.upper[0]) ||
      !std::isfinite(b.upper[1]) || b.lower[0] > b.upper[0] || b.lower[1] > b.upper[1])
      throw std::invalid_argument("finite ordered sweep intervals and bounds required");
  }
  std::vector<double> result(begin.size(), std::numeric_limits<double>::infinity());
  std::vector<double> start = begin, finish = end;
  std::vector<std::size_t> owners(begin.size());
  std::iota(owners.begin(), owners.end(), 0);
  const double speed = euclidean_norm(command[0], command[1]);
  const double radius = euclidean_norm(profile.half_length_m, profile.half_width_m);
  const double boundary_speed = speed + radius * std::abs(command[2]);
  const double margin = profile.clearance_margin_m + profile.payload_extra_margin_m;
  const double c = std::cos(origin[2]), s = std::sin(origin[2]);
  const std::vector<double> twist(command.begin(), command.end());
  // Match the bounded breadth-first subdivision, including per-owner rejection
  // before choosing the next frontier. A depth limit never grants clearance.
  for (int depth = 0; depth <= 10 && !owners.empty(); ++depth) {
    std::vector<double> mid(owners.size()), half(owners.size()), x(owners.size()), y(owners.size()), yaw(owners.size());
    std::vector<Bounds> current;
    current.reserve(owners.size());
    for (std::size_t i = 0; i < owners.size(); ++i) {
      mid[i] = (start[i]+finish[i])/2.; half[i] = (finish[i]-start[i])/2.;
      const auto pose = navigation::body_pose(twist, mid[i]);
      x[i] = origin[0]+c*pose[0]-s*pose[1]; y[i] = origin[1]+s*pose[0]+c*pose[1];
      yaw[i] = origin[2]+pose[2]; current.push_back(boxes[owners[i]]);
    }
    const auto point = clearance_many(x, y, yaw, current, profile);
    std::vector<bool> split(owners.size(), false);
    for (std::size_t i = 0; i < owners.size(); ++i) {
      const auto& b = current[i];
      const double dx = std::max({b.lower[0]-x[i], 0., x[i]-b.upper[0]});
      const double dy = std::max({b.lower[1]-y[i], 0., y[i]-b.upper[1]});
      const double circle = std::hypot(dx, dy)-radius-margin-speed*half[i]-1e-12;
      const double bound = std::max(point[i]-boundary_speed*half[i], circle);
      const bool safe = bound > 0.;
      const bool rejected = !safe && (point[i] <= 0. || depth == 10);
      if (safe || rejected) result[owners[i]] = std::min(result[owners[i]], bound);
      split[i] = !safe && !rejected;
    }
    std::vector<std::size_t> next;
    std::vector<double> next_start, next_finish;
    for (std::size_t i = 0; i < owners.size(); ++i) {
      if (!split[i] || result[owners[i]] <= 0.) continue;
      next.insert(next.end(), {owners[i], owners[i]});
      next_start.insert(next_start.end(), {start[i], mid[i]});
      next_finish.insert(next_finish.end(), {mid[i], finish[i]});
    }
    owners.swap(next); start.swap(next_start); finish.swap(next_finish);
  }
  return result;
}
}  // namespace astribot::navigation::policy
