#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace astribot::simulation {

struct SliceVertex { double x, y, z; };
using SliceTriangle = std::array<SliceVertex, 3>;

struct SliceRaster {
  std::size_t width, height;
  double resolution, origin_x, origin_y;
  std::vector<std::int8_t> data;
};

// World-coordinate triangles, a half-open z slab [lower_z, upper_z),
// and a row-major raster. Unknown (-1) cells remain unknown. Geometry marks
// only known cells occupied (100); this function never creates free space.
// Infinite slab endpoints are supported. Closed, non-self-intersecting connected
// components use even/odd interiors, combined by union. A fully nested separate
// inner shell is therefore conservatively occupied, not interpreted as a cavity.
// Open/nonmanifold components contribute their actual surface projection only.
// Coincident mesh vertices are welded by exact coordinates, independently of
// original mesh indices (which may be split for normals or texture coordinates).
namespace height_slice_detail {

inline double coordinate(const SliceVertex & p, unsigned axis) {
  return axis == 0 ? p.x : axis == 1 ? p.y : p.z;
}

inline std::vector<SliceVertex> clip(const std::vector<SliceVertex> & input,
                                   unsigned axis, double bound, bool above) {
  std::vector<SliceVertex> output;
  if (input.empty()) return output;
  auto previous = input.back();
  bool previous_inside = above ? coordinate(previous, axis) >= bound
                               : coordinate(previous, axis) <= bound;
  for (const auto & current : input) {
    const bool inside = above ? coordinate(current, axis) >= bound
                              : coordinate(current, axis) <= bound;
    if (inside != previous_inside) {
      const double t = (bound - coordinate(previous, axis)) /
                       (coordinate(current, axis) - coordinate(previous, axis));
      SliceVertex p{previous.x + t * (current.x - previous.x),
                    previous.y + t * (current.y - previous.y),
                    previous.z + t * (current.z - previous.z)};
      if (axis == 0) p.x = bound;
      else if (axis == 1) p.y = bound;
      else p.z = bound;
      output.push_back(p);
    }
    if (inside) output.push_back(current);
    previous = current;
    previous_inside = inside;
  }
  return output;
}

inline std::pair<std::size_t, std::size_t> cells(double low, double high,
    double origin, double resolution, std::size_t count) {
  const double first = std::max(0., std::floor((low - origin) / resolution));
  const double end = std::min(static_cast<double>(count),
                              std::floor((high - origin) / resolution) + 1.);
  if (first >= end) return {0, 0};
  return {static_cast<std::size_t>(first), static_cast<std::size_t>(end)};
}

inline void project_face(const std::vector<SliceVertex> & polygon,
                         double upper_z, SliceRaster & raster) {
  if (polygon.empty()) return;
  double min_x = polygon.front().x, max_x = min_x;
  double min_y = polygon.front().y, max_y = min_y;
  for (const auto & p : polygon) {
    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
  }
  const auto columns = cells(min_x, max_x, raster.origin_x, raster.resolution, raster.width);
  const auto rows = cells(min_y, max_y, raster.origin_y, raster.resolution, raster.height);
  for (auto y = rows.first; y < rows.second; ++y) {
    const double bottom = raster.origin_y + y * raster.resolution;
    for (auto x = columns.first; x < columns.second; ++x) {
      auto & value = raster.data[y * raster.width + x];
      if (value < 0 || value == 100) continue;
      const double left = raster.origin_x + x * raster.resolution;
      auto overlap = clip(polygon, 0, left, true);
      overlap = clip(overlap, 0, left + raster.resolution, false);
      overlap = clip(overlap, 1, bottom, true);
      overlap = clip(overlap, 1, bottom + raster.resolution, false);
      // The clipped polygon represents a closed upper face for computation.
      // A cell touching only that excluded z boundary must remain unchanged.
      if (std::any_of(overlap.begin(), overlap.end(),
                     [upper_z](const SliceVertex & p) { return p.z < upper_z; }))
        value = 100;
    }
  }
}

inline std::size_t component(std::vector<std::size_t> & parents, std::size_t i) {
  while (parents[i] != i) { parents[i] = parents[parents[i]]; i = parents[i]; }
  return i;
}

inline void fill_section(const std::vector<std::array<SliceVertex, 2>> & segments,
                         SliceRaster & raster) {
  if (segments.empty()) return;
  double min_y = segments.front()[0].y, max_y = min_y;
  for (const auto & segment : segments) for (const auto & p : segment) {
    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
  }
  const auto rows = cells(min_y, max_y, raster.origin_y, raster.resolution, raster.height);
  std::vector<double> intersections;
  for (auto y = rows.first; y < rows.second; ++y) {
    intersections.clear();
    const double center_y = raster.origin_y + (y + .5) * raster.resolution;
    for (const auto & segment : segments) {
      auto a = segment[0], b = segment[1];
      if (a.y > b.y) std::swap(a, b);
      // Half-open scanline crossing counts a shared vertex exactly once.
      if (a.y <= center_y && center_y < b.y)
        intersections.push_back(a.x + (center_y - a.y) * (b.x - a.x) / (b.y - a.y));
    }
    std::sort(intersections.begin(), intersections.end());
    if (intersections.size() % 2 != 0)
      throw std::runtime_error("Closed mesh has an ambiguous height-slice section");
    for (std::size_t i = 0; i < intersections.size(); i += 2) {
      const double left = intersections[i], right = intersections[i + 1];
      const auto columns = cells(left, right, raster.origin_x, raster.resolution, raster.width);
      for (auto x = columns.first; x < columns.second; ++x) {
        const double center_x = raster.origin_x + (x + .5) * raster.resolution;
        auto & value = raster.data[y * raster.width + x];
        if (value >= 0 && left <= center_x && center_x < right) value = 100;
      }
    }
  }
}

}  // namespace height_slice_detail

inline void rasterize_mesh(const std::vector<SliceTriangle> & triangles,
                           double lower_z, double upper_z, SliceRaster & raster) {
  if (std::isnan(lower_z) || std::isnan(upper_z) || lower_z >= upper_z ||
      !std::isfinite(raster.resolution) || raster.resolution <= 0 ||
      !std::isfinite(raster.origin_x) || !std::isfinite(raster.origin_y) ||
      (raster.height && raster.width > std::numeric_limits<std::size_t>::max() / raster.height) ||
      raster.data.size() != raster.width * raster.height)
    throw std::invalid_argument("Invalid height-slice raster or slab");
  for (const auto & triangle : triangles) for (const auto & p : triangle)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::invalid_argument("Nonfinite collision mesh vertex");
  if (triangles.empty() || !raster.width || !raster.height) return;

  using namespace height_slice_detail;
  std::map<std::array<double, 3>, std::size_t> vertices;
  std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>> edges;
  std::vector<std::size_t> parents(triangles.size());
  std::iota(parents.begin(), parents.end(), 0);
  std::vector<bool> valid(triangles.size(), true);
  double min_z = triangles.front()[0].z, max_z = min_z;
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    const auto & triangle = triangles[i];
    std::array<std::size_t, 3> ids;
    for (unsigned j = 0; j < 3; ++j) {
      const auto & p = triangle[j];
      min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
      ids[j] = vertices.emplace(std::array<double, 3>{p.x, p.y, p.z}, vertices.size()).first->second;
    }
    const SliceVertex a{triangle[1].x - triangle[0].x, triangle[1].y - triangle[0].y,
                        triangle[1].z - triangle[0].z};
    const SliceVertex b{triangle[2].x - triangle[0].x, triangle[2].y - triangle[0].y,
                        triangle[2].z - triangle[0].z};
    if (a.y*b.z == a.z*b.y && a.z*b.x == a.x*b.z && a.x*b.y == a.y*b.x) {
      valid[i] = false; continue;
    }
    for (unsigned j = 0; j < 3; ++j) {
      const auto edge = std::minmax(ids[j], ids[(j + 1) % 3]);
      auto & incidents = edges[{edge.first, edge.second}];
      if (!incidents.empty()) parents[component(parents, i)] = component(parents, incidents.front());
      incidents.push_back(i);
    }
    std::vector<SliceVertex> polygon(triangle.begin(), triangle.end());
    polygon = clip(polygon, 2, lower_z, true);
    polygon = clip(polygon, 2, upper_z, false);
    project_face(polygon, upper_z, raster);
  }

  std::vector<bool> closed(triangles.size(), true);
  for (const auto & edge : edges) if (edge.second.size() != 2)
    closed[component(parents, edge.second.front())] = false;
  const double low = std::max(lower_z, min_z), high = std::min(upper_z, max_z);
  if (low >= high) return;
  const double section_z = .5 * low + .5 * high;
  std::map<std::size_t, std::vector<std::array<SliceVertex, 2>>> sections;
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    const auto root = component(parents, i);
    if (!valid[i] || !closed[root]) continue;
    std::vector<SliceVertex> crossings;
    for (unsigned j = 0; j < 3; ++j) {
      auto a = triangles[i][j], b = triangles[i][(j + 1) % 3];
      if (a.z > b.z) std::swap(a, b);
      if (a.z <= section_z && section_z < b.z) {
        const double t = (section_z - a.z) / (b.z - a.z);
        crossings.push_back({a.x + t*(b.x - a.x), a.y + t*(b.y - a.y), section_z});
      }
    }
    if (crossings.size() == 2) sections[root].push_back({crossings[0], crossings[1]});
  }
  for (const auto & section : sections) fill_section(section.second, raster);
}

}  // namespace astribot::simulation
