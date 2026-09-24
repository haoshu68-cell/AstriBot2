#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace astribot_s1_mapping {
// Endpoint evidence only: ray origin / occlusion coverage is not in saved keyframes.
class HeightSliceGrid {
 public:
  using Cell = std::pair<int, int>;
  double resolution, ground_z;
  std::vector<double> edges;
  std::vector<std::set<Cell>> occupied;
  std::vector<uint64_t> counts;
  uint64_t below=0, above=0, nonfinite=0;
  HeightSliceGrid(double cell_size, double ground, std::vector<double> heights)
      : resolution(cell_size), ground_z(ground), edges(std::move(heights)) {
    if (!std::isfinite(resolution) || resolution<=0 || !std::isfinite(ground_z) || edges.size()<2)
      throw std::invalid_argument("invalid height grid configuration");
    for (size_t i=0;i<edges.size();++i)
      if (!std::isfinite(edges[i]) || (i && edges[i]<=edges[i-1]))
        throw std::invalid_argument("height edges must increase strictly");
    occupied.resize(edges.size()-1);counts.resize(edges.size()-1);
  }
  void add(double x,double y,double z) {
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) {++nonfinite;return;}
    const double h=z-ground_z;
    if (h<edges.front()) {++below;return;}
    if (h>=edges.back()) {++above;return;}
    const auto i=static_cast<size_t>(std::upper_bound(edges.begin(),edges.end(),h)-edges.begin()-1);
    const double cx=std::floor(x/resolution),cy=std::floor(y/resolution);
    if (cx<std::numeric_limits<int>::min() || cx>std::numeric_limits<int>::max() ||
        cy<std::numeric_limits<int>::min() || cy>std::numeric_limits<int>::max())
      throw std::invalid_argument("point exceeds representable grid coordinates");
    occupied[i].emplace(static_cast<int>(cx),static_cast<int>(cy));++counts[i];
  }
};
}  // namespace astribot_s1_mapping
