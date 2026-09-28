#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <vector>

namespace astribot_s1_robot_geometry {
// Path XY is in the corridor frame; polygon XY is in the held body frame.
// This reserves the passage through a standard corner rotation. It is not
// a replacement for the planner's occupied-cell swept-polygon check.
inline bool corridorTurnsOutside(
    const std::vector<std::array<double, 2>> & polygon,
    const std::vector<std::array<double, 2>> & path,
    double length, double width, double margin, double heading_limit) {
  constexpr double pi = 3.14159265358979323846;
  bool have_previous = false;
  double previous = 0.;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & at = path[i - 1];
    const double dx = path[i][0] - at[0], dy = path[i][1] - at[1];
    if (std::hypot(dx, dy) <= 1e-9) {continue;}
    const double heading = std::atan2(dy, dx);
    const double turn = std::remainder(heading - previous, 2. * pi);
    if (have_previous && std::abs(turn) > 1e-9 &&
        std::max(std::abs(previous), std::abs(heading)) > heading_limit) {
      const double low = std::min(previous, previous + turn);
      const double high = std::max(previous, previous + turn);
      const double xmin = -margin, xmax = length + margin;
      const double ymin = -width / 2. - margin, ymax = width / 2. + margin;
      double radius = 0.;
      for (const auto & p : polygon) {radius = std::max(radius, std::hypot(p[0], p[1]));}
      const double gap = std::hypot(std::max({xmin - at[0], 0., at[0] - xmax}),
                                    std::max({ymin - at[1], 0., at[1] - ymax}));
      if (gap <= radius + 1e-10) {
        std::vector<double> angles{low, high};
        // Intersection can first occur at a vertex/edge contact. Enumerate
        // every such angle, then use the existing filled-polygon distance.
        // This avoids treating unrelated extrema of a swept AABB as contact.
        const auto contacts = [&](double a, double b, double value) {
          const double amplitude = std::hypot(a, b);
          if (amplitude == 0. || std::abs(value) > amplitude + 1e-12) {return;}
          const double phase = std::atan2(b, a);
          const double offset = std::acos(std::clamp(value / amplitude, -1., 1.));
          for (double root : {phase - offset, phase + offset}) {
            const int first = static_cast<int>(std::ceil((low - root) / (2. * pi)));
            const int last = static_cast<int>(std::floor((high - root) / (2. * pi)));
            for (int k = first; k <= last; ++k) {angles.push_back(root + k * 2. * pi);}
          }
        };
        for (std::size_t j = 0; j < polygon.size(); ++j) {
          const auto & p = polygon[j];
          for (double x : {xmin, xmax}) {contacts(p[0], -p[1], x - at[0]);}
          for (double y : {ymin, ymax}) {contacts(p[1], p[0], y - at[1]);}
          const auto & next = polygon[(j + 1) % polygon.size()];
          const double nx = p[1] - next[1], ny = next[0] - p[0];
          for (double x : {xmin - at[0], xmax - at[0]}) {
            for (double y : {ymin - at[1], ymax - at[1]}) {
              contacts(nx * x + ny * y, nx * y - ny * x, nx * p[0] + ny * p[1]);
            }
          }
        }
        std::vector<std::array<double, 7>> queries;
        queries.reserve(angles.size());
        for (double angle : angles) {queries.push_back({at[0],at[1],angle,xmin,ymin,xmax,ymax});}
        const auto distances = boxDistances(polygon, queries);
        if (std::any_of(distances.begin(), distances.end(), [](double distance) {
            return distance <= 1e-10;
          })) {return false;}
      }
    }
    previous = heading;have_previous = true;
  }
  return true;
}
}  // namespace astribot_s1_robot_geometry
