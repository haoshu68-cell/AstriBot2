// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/costmap_adapter.hpp"

#include <limits>
#include <sstream>

namespace astribot_s1_autonomy
{

bool validateCostmapAdapterParams(const CostmapAdapterParams & params, std::string & error)
{
  error.clear();
  if (params.unknown_cost < 0 || params.unknown_cost > 255) {
    std::ostringstream oss;
    oss << "unknown_cost 必须在 [0,255] 内，当前 " << params.unknown_cost;
    error = oss.str();
    return false;
  }
  if (params.lethal_cost_threshold < 1 || params.lethal_cost_threshold > 255) {
    std::ostringstream oss;
    oss << "lethal_cost_threshold 必须在 [1,255] 内，当前 " << params.lethal_cost_threshold;
    error = oss.str();
    return false;
  }
  if (params.lethal_cost_threshold > params.unknown_cost) {
    std::ostringstream oss;
    oss << "lethal_cost_threshold(" << params.lethal_cost_threshold
        << ") > unknown_cost(" << params.unknown_cost
        << ")，将导致没有任何栅格被判成致命障碍";
    error = oss.str();
    return false;
  }
  return true;
}

bool costmapToGridMap(
  unsigned int size_x,
  unsigned int size_y,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<uint8_t> & data,
  const CostmapAdapterParams & params,
  GridMap & out,
  std::string & error)
{
  error.clear();
  out = GridMap{};

  std::string param_error;
  if (!validateCostmapAdapterParams(params, param_error)) {
    error = "代价地图转换参数非法: " + param_error;
    return false;
  }
  if (size_x == 0U || size_y == 0U) {
    std::ostringstream oss;
    oss << "代价地图尺寸为 0 (" << size_x << "x" << size_y << ")";
    error = oss.str();
    return false;
  }
  if (!(resolution > 0.0)) {
    std::ostringstream oss;
    oss << "代价地图分辨率非法(" << resolution << ")";
    error = oss.str();
    return false;
  }

  const uint64_t expected =
    static_cast<uint64_t>(size_x) * static_cast<uint64_t>(size_y);
  if (expected > static_cast<uint64_t>(std::numeric_limits<std::size_t>::max())) {
    error = "代价地图尺寸乘积溢出 size_t";
    return false;
  }
  if (data.size() != static_cast<std::size_t>(expected)) {
    std::ostringstream oss;
    oss << "代价地图 data 长度 " << data.size() << " 与 size_x*size_y=" << expected
        << " 不一致(拒绝转换以防越界读)";
    error = oss.str();
    return false;
  }

  const auto unknown_cost = static_cast<uint8_t>(params.unknown_cost);
  const auto lethal_threshold = static_cast<uint8_t>(params.lethal_cost_threshold);

  out.width = size_x;
  out.height = size_y;
  out.resolution = resolution;
  out.origin_x = origin_x;
  out.origin_y = origin_y;
  out.data.resize(static_cast<std::size_t>(expected));

  for (std::size_t i = 0; i < out.data.size(); ++i) {
    const uint8_t cost = data[i];
    if (cost == unknown_cost) {
      out.data[i] = static_cast<int8_t>(-1);
    } else if (cost >= lethal_threshold) {
      out.data[i] = static_cast<int8_t>(100);
    } else {
      out.data[i] = static_cast<int8_t>(0);
    }
  }
  return true;
}

CostmapGridStats summarizeGrid(const GridMap & grid)
{
  CostmapGridStats stats;
  if (!grid.consistent()) {
    return stats;
  }
  stats.total = grid.data.size();
  for (const int8_t v : grid.data) {
    if (v < 0) {
      ++stats.unknown;
    } else if (v >= static_cast<int8_t>(100)) {
      ++stats.lethal;
    } else {
      ++stats.free;
    }
  }
  return stats;
}

}  // namespace astribot_s1_autonomy
