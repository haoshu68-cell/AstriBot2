// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/path_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace astribot_s1_autonomy
{

bool PathValidator::configure(const PathValidatorParams & params, std::string & error)
{
  configured_ = false;
  std::ostringstream oss;

  if (params.free_threshold < 0 || params.occupied_threshold > 100 ||
    params.free_threshold >= params.occupied_threshold)
  {
    oss << "要求 0 <= free_threshold < occupied_threshold <= 100，当前 free="
        << params.free_threshold << " occupied=" << params.occupied_threshold;
    error = oss.str();
    return false;
  }
  if (params.goal_clearance_radius < 0.0) {
    error = "goal_clearance_radius 不能为负";
    return false;
  }
  if (params.goal_unknown_clearance_radius < 0.0) {
    error = "goal_unknown_clearance_radius 不能为负";
    return false;
  }
  if (params.goal_unknown_clearance_radius > params.goal_clearance_radius) {
    oss << "goal_unknown_clearance_radius(" << params.goal_unknown_clearance_radius
        << ") 不应大于 goal_clearance_radius(" << params.goal_clearance_radius
        << ")：前沿点天生紧贴未知区，这样设会让所有候选点都不合法";
    error = oss.str();
    return false;
  }
  if (!(params.path_endpoint_tolerance > 0.0)) {
    error = "path_endpoint_tolerance 必须为正";
    return false;
  }
  if (params.max_samples == 0U) {
    error = "max_samples 必须为正";
    return false;
  }

  params_ = params;
  configured_ = true;
  error.clear();
  return true;
}

double PathValidator::effectiveStep(const GridMap & map) const
{
  if (params_.path_sample_step > 0.0) {
    return params_.path_sample_step;
  }
  return map.resolution * 0.5;
}

bool PathValidator::isKnownFree(const GridMap & map, double wx, double wy) const
{
  if (!configured_) {
    return false;
  }
  unsigned int mx = 0U;
  unsigned int my = 0U;
  if (!map.worldToMap(wx, wy, mx, my)) {
    return false;   // 地图外一律按不合法处理（保守）
  }
  const int8_t v = map.data[map.index(mx, my)];
  if (v < 0) {
    return false;   // 未知格 —— 这就是本文件存在的理由
  }
  return v <= static_cast<int8_t>(params_.free_threshold);
}

ValidationResult PathValidator::validateGoal(
  const GridMap & map, double gx, double gy) const
{
  ValidationResult r;
  if (!configured_) {
    r.reason = "PathValidator 未配置";
    return r;
  }
  if (!map.consistent()) {
    r.reason = "地图为空或 data 长度与尺寸不一致";
    return r;
  }

  unsigned int mx = 0U;
  unsigned int my = 0U;
  if (!map.worldToMap(gx, gy, mx, my)) {
    std::ostringstream oss;
    oss << "目标点(" << gx << ", " << gy << ")落在地图范围外";
    r.reason = oss.str();
    r.first_bad_point = {gx, gy};
    return r;
  }

  const int8_t v = map.data[map.index(mx, my)];
  if (v < 0) {
    r.reason = "目标点落在未知栅格上";
    r.first_bad_point = {gx, gy};
    return r;
  }
  if (v > static_cast<int8_t>(params_.free_threshold)) {
    std::ostringstream oss;
    oss << "目标点所在栅格不是空闲(占据值=" << static_cast<int>(v) << ")";
    r.reason = oss.str();
    r.first_bad_point = {gx, gy};
    return r;
  }

  const double occupied_radius = params_.goal_clearance_radius;
  const double unknown_radius = params_.goal_unknown_clearance_radius;
  const double max_radius = std::max(occupied_radius, unknown_radius);
  if (max_radius > 0.0 && map.resolution > 0.0) {
    const int radius_cells = static_cast<int>(std::ceil(max_radius / map.resolution));
    const double occ_r_sq = (occupied_radius / map.resolution) * (occupied_radius / map.resolution);
    const double unk_r_sq = (unknown_radius / map.resolution) * (unknown_radius / map.resolution);
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const double d_sq = static_cast<double>((dx * dx) + (dy * dy));
        const bool in_occ = d_sq <= occ_r_sq;
        const bool in_unk = d_sq <= unk_r_sq;
        if (!in_occ && !in_unk) {
          continue;
        }
        const int nx = static_cast<int>(mx) + dx;
        const int ny = static_cast<int>(my) + dy;
        ++r.samples_checked;
        if (nx < 0 || ny < 0 ||
          nx >= static_cast<int>(map.width) || ny >= static_cast<int>(map.height))
        {
          r.reason = "目标点净空邻域伸出地图边界";
          r.first_bad_point = {map.worldX(mx), map.worldY(my)};
          return r;
        }
        const std::size_t idx = map.index(
          static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
        const int8_t nv = map.data[idx];
        if (in_unk && nv < 0) {
          std::ostringstream oss;
          oss << "目标点未知净空半径(" << unknown_radius << "m)内存在未知栅格";
          r.reason = oss.str();
          r.first_bad_point = {
            map.worldX(static_cast<unsigned int>(nx)),
            map.worldY(static_cast<unsigned int>(ny))};
          return r;
        }
        if (in_occ && nv >= static_cast<int8_t>(params_.occupied_threshold)) {
          std::ostringstream oss;
          oss << "目标点净空半径(" << occupied_radius << "m)内存在占据栅格";
          r.reason = oss.str();
          r.first_bad_point = {
            map.worldX(static_cast<unsigned int>(nx)),
            map.worldY(static_cast<unsigned int>(ny))};
          return r;
        }
      }
    }
  }

  r.valid = true;
  r.reason = "目标点合法";
  return r;
}

ValidationResult PathValidator::validatePath(
  const GridMap & map,
  const std::vector<PlanarPoint> & path,
  const PlanarPoint & requested_goal) const
{
  ValidationResult r;
  if (!configured_) {
    r.reason = "PathValidator 未配置";
    return r;
  }
  if (!map.consistent()) {
    r.reason = "地图为空或 data 长度与尺寸不一致";
    return r;
  }
  if (path.empty()) {
    r.reason = "路径为空(规划失败)";
    return r;
  }
  if (path.size() < 2U) {
    r.reason = "路径只有 1 个点，无法证明可通行";
    r.first_bad_point = path.front();
    return r;
  }

  const PlanarPoint & endp = path.back();
  const double endpoint_err = std::hypot(endp.x - requested_goal.x, endp.y - requested_goal.y);
  if (endpoint_err > params_.path_endpoint_tolerance) {
    std::ostringstream oss;
    oss << "规划终点与请求目标相差 " << endpoint_err << "m，超过上限 "
        << params_.path_endpoint_tolerance << "m(规划器可能截断了不可达目标)";
    r.reason = oss.str();
    r.first_bad_point = endp;
    r.bad_segment_index = static_cast<int>(path.size()) - 1;
    return r;
  }

  const double step = effectiveStep(map);
  if (!(step > 0.0)) {
    r.reason = "采样步长非正(地图分辨率异常?)";
    return r;
  }

  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    const PlanarPoint & a = path[i];
    const PlanarPoint & b = path[i + 1U];
    const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
    const auto steps = static_cast<std::size_t>(
      std::max(1.0, std::ceil(seg_len / step)));

    for (std::size_t k = 0; k < steps; ++k) {
      if (r.samples_checked >= params_.max_samples) {
        std::ostringstream oss;
        oss << "采样点数超过上限 " << params_.max_samples << "，拒绝该路径(防御性拦截)";
        r.reason = oss.str();
        r.bad_segment_index = static_cast<int>(i);
        return r;
      }
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      const double sx = a.x + ((b.x - a.x) * t);
      const double sy = a.y + ((b.y - a.y) * t);
      ++r.samples_checked;
      if (!isKnownFree(map, sx, sy)) {
        unsigned int mx = 0U;
        unsigned int my = 0U;
        const bool in_map = map.worldToMap(sx, sy, mx, my);
        std::ostringstream oss;
        if (!in_map) {
          oss << "路径第 " << i << " 段采样点越出地图范围";
        } else {
          const int8_t v = map.data[map.index(mx, my)];
          if (v < 0) {
            oss << "路径第 " << i << " 段穿过未知栅格";
          } else {
            oss << "路径第 " << i << " 段穿过非空闲栅格(占据值="
                << static_cast<int>(v) << ")";
          }
        }
        r.reason = oss.str();
        r.first_bad_point = {sx, sy};
        r.bad_segment_index = static_cast<int>(i);
        return r;
      }
    }
  }

  ++r.samples_checked;
  if (!isKnownFree(map, path.back().x, path.back().y)) {
    r.reason = "路径终点所在栅格不是已知空闲";
    r.first_bad_point = path.back();
    r.bad_segment_index = static_cast<int>(path.size()) - 1;
    return r;
  }

  r.valid = true;
  std::ostringstream oss;
  oss << "路径合法(检查 " << r.samples_checked << " 个采样点, 步长 " << step << "m)";
  r.reason = oss.str();
  return r;
}


std::size_t nearestPathIndex(const std::vector<PlanarPoint> & path, const PlanarPoint & robot)
{
  if (path.empty()) {
    return 0U;
  }
  std::size_t best = 0U;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i].x - robot.x;
    const double dy = path[i].y - robot.y;
    const double d2 = (dx * dx) + (dy * dy);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

double pathDeviation(const std::vector<PlanarPoint> & path, const PlanarPoint & robot)
{
  if (path.empty()) {
    return -1.0;              // 见头文件：绝不能返回 0
  }
  const std::size_t i = nearestPathIndex(path, robot);
  return std::hypot(path[i].x - robot.x, path[i].y - robot.y);
}

std::vector<PlanarPoint> remainingPath(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot)
{
  if (path.empty()) {
    return {};
  }
  const std::size_t i = nearestPathIndex(path, robot);
  return std::vector<PlanarPoint>(path.begin() + static_cast<std::ptrdiff_t>(i), path.end());
}

}  // namespace astribot_s1_autonomy
