// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <utility>

namespace astribot_s1_autonomy
{

namespace
{
/// BFS/膨胀等循环的迭代上限倍数：任何一次遍历最多访问 kMaxVisitFactor 倍格数。
/// 正常算法每格只会被访问一次，这个上限纯粹是「禁止无限 while 死循环」的兜底。
constexpr std::size_t kMaxVisitFactor = 4U;
/// 地图格数上限（约 4000x4000）。超过就拒绝处理，防止一次 search 吃掉几百 MB。
constexpr std::size_t kMaxCellCount = 16000000U;
/// 寻找 BFS 种子时，允许在机器人周围搜索的最大半径（格）。
constexpr int kMaxSeedSearchRadiusCells = 40;

/// 8 邻域偏移。
constexpr int kNeighborDx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int kNeighborDy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
/// 4 邻域取前述数组里的偶数下标（正右、正上、正左、正下）。
constexpr int kNeighborDx4[4] = {1, 0, -1, 0};
constexpr int kNeighborDy4[4] = {0, 1, 0, -1};
}  // namespace

bool GridMap::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!(resolution > 0.0)) {
    return false;
  }
  const double fx = (wx - origin_x) / resolution;
  const double fy = (wy - origin_y) / resolution;
  if (fx < 0.0 || fy < 0.0) {
    return false;
  }
  const auto ix = static_cast<int64_t>(std::floor(fx));
  const auto iy = static_cast<int64_t>(std::floor(fy));
  if (ix >= static_cast<int64_t>(width) || iy >= static_cast<int64_t>(height)) {
    return false;
  }
  mx = static_cast<unsigned int>(ix);
  my = static_cast<unsigned int>(iy);
  return true;
}

bool FrontierSearch::configure(const FrontierSearchParams & params, std::string & error)
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
  if (params.obstacle_inflation_radius < 0.0) {
    error = "obstacle_inflation_radius 不能为负";
    return false;
  }
  if (params.min_frontier_cells < 1) {
    error = "min_frontier_cells 必须 >= 1";
    return false;
  }
  if (params.min_samples_per_cluster < 1 ||
    params.max_samples_per_cluster < params.min_samples_per_cluster)
  {
    oss << "要求 1 <= min_samples_per_cluster <= max_samples_per_cluster，当前 min="
        << params.min_samples_per_cluster << " max=" << params.max_samples_per_cluster;
    error = oss.str();
    return false;
  }
  if (params.min_goal_distance < 0.0) {
    error = "min_goal_distance 不能为负";
    return false;
  }
  if (params.max_goal_distance > 0.0 && params.max_goal_distance <= params.min_goal_distance) {
    oss << "max_goal_distance(" << params.max_goal_distance
        << ") 必须大于 min_goal_distance(" << params.min_goal_distance << ")，或 <=0 表示不限制";
    error = oss.str();
    return false;
  }

  params_ = params;
  configured_ = true;
  error.clear();
  return true;
}

void FrontierSearch::buildObstacleMask(const GridMap & map)
{
  const std::size_t cell_count = map.data.size();
  const int width = static_cast<int>(map.width);
  const int height = static_cast<int>(map.height);

  std::vector<uint8_t> raw_occupied(cell_count, 0U);
  for (std::size_t i = 0; i < cell_count; ++i) {
    if (map.data[i] >= static_cast<int8_t>(params_.occupied_threshold)) {
      raw_occupied[i] = 1U;
    }
  }

  if (params_.min_obstacle_cluster_cells > 1) {
    visited_.assign(cell_count, 0U);
    std::vector<std::size_t> component;
    for (std::size_t seed = 0; seed < cell_count; ++seed) {
      if (raw_occupied[seed] == 0U || visited_[seed] != 0U) {
        continue;
      }
      component.clear();
      bfs_queue_.clear();
      bfs_queue_.push_back(seed);
      visited_[seed] = 1U;
      std::size_t head = 0U;
      std::size_t guard = 0U;
      const std::size_t guard_limit = cell_count * kMaxVisitFactor;
      while (head < bfs_queue_.size() && guard++ < guard_limit) {
        const std::size_t cur = bfs_queue_[head++];
        component.push_back(cur);
        const int cx = static_cast<int>(cur % map.width);
        const int cy = static_cast<int>(cur / map.width);
        for (int k = 0; k < 8; ++k) {
          const int nx = cx + kNeighborDx8[k];
          const int ny = cy + kNeighborDy8[k];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const std::size_t nidx = map.index(
            static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
          if (raw_occupied[nidx] != 0U && visited_[nidx] == 0U) {
            visited_[nidx] = 1U;
            bfs_queue_.push_back(nidx);
          }
        }
      }
      if (component.size() < static_cast<std::size_t>(params_.min_obstacle_cluster_cells)) {
        for (const std::size_t idx : component) {
          raw_occupied[idx] = 0U;   // 判定为噪声斑块，抹除
        }
      }
    }
  }

  inflated_occupied_ = raw_occupied;
  const int radius_cells = (params_.obstacle_inflation_radius > 0.0 && map.resolution > 0.0) ?
    static_cast<int>(std::ceil(params_.obstacle_inflation_radius / map.resolution)) : 0;
  if (radius_cells > 0) {
    const double radius_sq_cells =
      (params_.obstacle_inflation_radius / map.resolution) *
      (params_.obstacle_inflation_radius / map.resolution);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t idx = map.index(
          static_cast<unsigned int>(x), static_cast<unsigned int>(y));
        if (raw_occupied[idx] == 0U) {
          continue;
        }
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (static_cast<double>((dx * dx) + (dy * dy)) > radius_sq_cells) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
              continue;
            }
            inflated_occupied_[map.index(
                static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))] = 1U;
          }
        }
      }
    }
  }
}

bool FrontierSearch::buildReachableMask(const GridMap & map, double robot_x, double robot_y)
{
  const std::size_t cell_count = map.data.size();
  reachable_.assign(cell_count, 0U);

  unsigned int rmx = 0U;
  unsigned int rmy = 0U;
  if (!map.worldToMap(robot_x, robot_y, rmx, rmy)) {
    return false;   // 机器人在地图外，无法确定可达域
  }

  const int width = static_cast<int>(map.width);
  const int height = static_cast<int>(map.height);
  auto isFreeCell = [&](std::size_t idx) {
      const int8_t v = map.data[idx];
      return v >= 0 && v <= static_cast<int8_t>(params_.free_threshold) &&
             inflated_occupied_[idx] == 0U;
    };

  std::size_t seed = 0U;
  bool seed_found = false;
  for (int r = 0; r <= kMaxSeedSearchRadiusCells && !seed_found; ++r) {
    for (int dy = -r; dy <= r && !seed_found; ++dy) {
      for (int dx = -r; dx <= r && !seed_found; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const int nx = static_cast<int>(rmx) + dx;
        const int ny = static_cast<int>(rmy) + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const std::size_t idx = map.index(
          static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
        if (isFreeCell(idx)) {
          seed = idx;
          seed_found = true;
        }
      }
    }
  }
  if (!seed_found) {
    return false;   // 机器人周围完全没有可用自由格
  }

  bfs_queue_.clear();
  bfs_queue_.push_back(seed);
  reachable_[seed] = 1U;
  std::size_t head = 0U;
  std::size_t guard = 0U;
  const std::size_t guard_limit = cell_count * kMaxVisitFactor;
  while (head < bfs_queue_.size() && guard++ < guard_limit) {
    const std::size_t cur = bfs_queue_[head++];
    const int cx = static_cast<int>(cur % map.width);
    const int cy = static_cast<int>(cur / map.width);
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kNeighborDx8[k];
      const int ny = cy + kNeighborDy8[k];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const std::size_t nidx = map.index(
        static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      if (reachable_[nidx] != 0U) {
        continue;
      }
      if (!isFreeCell(nidx)) {
        continue;
      }
      reachable_[nidx] = 1U;
      bfs_queue_.push_back(nidx);
    }
  }
  return true;
}

std::size_t FrontierSearch::extractFrontierCells(const GridMap & map)
{
  const std::size_t cell_count = map.data.size();
  is_frontier_.assign(cell_count, 0U);
  const int width = static_cast<int>(map.width);
  const int height = static_cast<int>(map.height);
  const int neighbor_count = params_.use_eight_connectivity ? 8 : 4;

  std::size_t frontier_count = 0U;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t idx = map.index(
        static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      const int8_t v = map.data[idx];
      if (v < 0 || v > static_cast<int8_t>(params_.free_threshold)) {
        continue;
      }
      if (inflated_occupied_[idx] != 0U) {
        continue;
      }
      bool touches_unknown = false;
      for (int k = 0; k < neighbor_count && !touches_unknown; ++k) {
        const int nx = x + (params_.use_eight_connectivity ? kNeighborDx8[k] : kNeighborDx4[k]);
        const int ny = y + (params_.use_eight_connectivity ? kNeighborDy8[k] : kNeighborDy4[k]);
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        if (map.data[map.index(
            static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))] < 0)
        {
          touches_unknown = true;
        }
      }
      if (touches_unknown) {
        is_frontier_[idx] = 1U;
        ++frontier_count;
      }
    }
  }
  return frontier_count;
}

std::size_t FrontierSearch::countUnknownAround(
  const GridMap & map, unsigned int mx, unsigned int my) const
{
  if (!(map.resolution > 0.0) || !(params_.gain_window_radius > 0.0)) {
    return 0U;
  }
  const int r = static_cast<int>(std::ceil(params_.gain_window_radius / map.resolution));
  const int width = static_cast<int>(map.width);
  const int height = static_cast<int>(map.height);
  std::size_t unknown = 0U;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const int nx = static_cast<int>(mx) + dx;
      const int ny = static_cast<int>(my) + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      if (map.data[map.index(
          static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))] < 0)
      {
        ++unknown;
      }
    }
  }
  return unknown;
}

bool FrontierSearch::hasClearance(const GridMap & map, unsigned int mx, unsigned int my) const
{
  if (!(params_.required_clearance_radius > 0.0) || !(map.resolution > 0.0)) {
    return true;
  }
  const int r = static_cast<int>(std::ceil(params_.required_clearance_radius / map.resolution));
  const double r_sq = (params_.required_clearance_radius / map.resolution) *
    (params_.required_clearance_radius / map.resolution);
  const int width = static_cast<int>(map.width);
  const int height = static_cast<int>(map.height);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (static_cast<double>((dx * dx) + (dy * dy)) > r_sq) {
        continue;
      }
      const int nx = static_cast<int>(mx) + dx;
      const int ny = static_cast<int>(my) + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        return false;   // 净空窗口伸出地图边界，保守判为净空不足
      }
      const std::size_t idx = map.index(
        static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      if (inflated_occupied_[idx] != 0U) {
        return false;
      }
    }
  }
  return true;
}

double FrontierSearch::visitPenaltyAt(
  double x, double y, const std::vector<VisitRecord> & history) const
{
  if (!(params_.visit_penalty_radius > 0.0)) {
    return 0.0;
  }
  const double radius_sq = params_.visit_penalty_radius * params_.visit_penalty_radius;
  double penalty = 0.0;
  for (const VisitRecord & rec : history) {
    const double dx = x - rec.x;
    const double dy = y - rec.y;
    const double d_sq = (dx * dx) + (dy * dy);
    if (d_sq > radius_sq) {
      continue;
    }
    const double closeness = 1.0 - (std::sqrt(d_sq) / params_.visit_penalty_radius);
    penalty += closeness * static_cast<double>(rec.count);
  }
  return penalty;
}

void FrontierSearch::clusterFrontiers(
  const GridMap & map, double robot_x, double robot_y, Result & out)
{
  const std::size_t cell_count = map.data.size();
  visited_.assign(cell_count, 0U);
  const int width = static_cast<int>(map.width);
  const int height = static_cast<int>(map.height);

  for (std::size_t seed = 0; seed < cell_count; ++seed) {
    if (is_frontier_[seed] == 0U || visited_[seed] != 0U) {
      continue;
    }

    FrontierCluster cluster;
    bfs_queue_.clear();
    bfs_queue_.push_back(seed);
    visited_[seed] = 1U;
    std::size_t head = 0U;
    std::size_t guard = 0U;
    const std::size_t guard_limit = cell_count * kMaxVisitFactor;
    double sum_x = 0.0;
    double sum_y = 0.0;
    bool any_reachable = false;

    while (head < bfs_queue_.size() && guard++ < guard_limit) {
      const std::size_t cur = bfs_queue_[head++];
      cluster.cells.push_back(cur);
      const auto cx = static_cast<unsigned int>(cur % map.width);
      const auto cy = static_cast<unsigned int>(cur / map.width);
      sum_x += map.worldX(cx);
      sum_y += map.worldY(cy);
      if (reachable_[cur] != 0U) {
        any_reachable = true;
      }
      for (int k = 0; k < 8; ++k) {
        const int nx = static_cast<int>(cx) + kNeighborDx8[k];
        const int ny = static_cast<int>(cy) + kNeighborDy8[k];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const std::size_t nidx = map.index(
          static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
        if (is_frontier_[nidx] != 0U && visited_[nidx] == 0U) {
          visited_[nidx] = 1U;
          bfs_queue_.push_back(nidx);
        }
      }
    }

    cluster.size = cluster.cells.size();
    if (cluster.size == 0U) {
      continue;
    }
    cluster.centroid_x = sum_x / static_cast<double>(cluster.size);
    cluster.centroid_y = sum_y / static_cast<double>(cluster.size);
    cluster.distance_to_robot = std::hypot(
      cluster.centroid_x - robot_x, cluster.centroid_y - robot_y);

    unsigned int gmx = 0U;
    unsigned int gmy = 0U;
    if (map.worldToMap(cluster.centroid_x, cluster.centroid_y, gmx, gmy)) {
      cluster.unknown_gain = countUnknownAround(map, gmx, gmy);
    }

    if (cluster.size < static_cast<std::size_t>(params_.min_frontier_cells)) {
      cluster.accepted = false;
      cluster.reject_reason = "面积过小";
    } else if (!any_reachable) {
      cluster.accepted = false;
      cluster.reject_reason = "被障碍物包围/不可达";
    } else {
      cluster.accepted = true;
      ++out.accepted_cluster_count;
    }
    out.clusters.push_back(std::move(cluster));
  }
}

void FrontierSearch::search(
  const GridMap & map,
  double robot_x,
  double robot_y,
  const std::vector<VisitRecord> & history,
  Result & out)
{
  out.clusters.clear();
  out.candidates.clear();
  out.best_candidate_index = -1;
  out.raw_frontier_cell_count = 0U;
  out.accepted_cluster_count = 0U;
  out.summary.clear();

  if (!configured_) {
    out.summary = "FrontierSearch 未配置";
    return;
  }
  if (!map.consistent()) {
    out.summary = "地图为空或 data 长度与 width*height 不一致";
    return;
  }
  if (map.data.size() > kMaxCellCount) {
    std::ostringstream oss;
    oss << "地图格数 " << map.data.size() << " 超过上限 " << kMaxCellCount << "，拒绝处理";
    out.summary = oss.str();
    return;
  }

  buildObstacleMask(map);

  const bool reachable_ok = buildReachableMask(map, robot_x, robot_y);
  if (!reachable_ok) {
    reachable_.assign(map.data.size(), 1U);
    out.summary = "警告: 机器人周围找不到自由格(可能贴障碍或位姿异常)，本次跳过可达性过滤; ";
  }

  out.raw_frontier_cell_count = extractFrontierCells(map);
  if (out.raw_frontier_cell_count == 0U) {
    out.summary += "没有任何前沿格";
    return;
  }

  clusterFrontiers(map, robot_x, robot_y, out);
  if (out.accepted_cluster_count == 0U) {
    out.summary += "有前沿格但没有任何前沿块通过过滤";
    return;
  }

  std::size_t max_gain = 1U;
  for (const FrontierCluster & c : out.clusters) {
    if (c.accepted) {
      max_gain = std::max(max_gain, c.unknown_gain);
    }
  }

  std::mt19937 rng(params_.random_seed);
  double best_cost = 0.0;

  for (std::size_t ci = 0; ci < out.clusters.size(); ++ci) {
    const FrontierCluster & cluster = out.clusters[ci];
    if (!cluster.accepted) {
      continue;
    }

    const auto raw_samples = static_cast<int64_t>(
      std::ceil(static_cast<double>(cluster.size) * params_.adaptive_sample_gain));
    const auto sample_count = static_cast<std::size_t>(
      std::clamp<int64_t>(
        raw_samples,
        params_.min_samples_per_cluster,
        std::min<int64_t>(
          params_.max_samples_per_cluster, static_cast<int64_t>(cluster.size))));

    const std::size_t stride = std::max<std::size_t>(1U, cluster.size / sample_count);
    std::uniform_int_distribution<std::size_t> jitter(0U, stride > 1U ? (stride - 1U) : 0U);

    for (std::size_t s = 0; s < sample_count; ++s) {
      const std::size_t base = s * stride;
      if (base >= cluster.size) {
        break;
      }
      const std::size_t pick = std::min(cluster.size - 1U, base + jitter(rng));
      const std::size_t cell = cluster.cells[pick];
      const auto mx = static_cast<unsigned int>(cell % map.width);
      const auto my = static_cast<unsigned int>(cell / map.width);

      GoalCandidate cand;
      cand.cluster_index = ci;
      cand.x = map.worldX(mx);
      cand.y = map.worldY(my);
      const double to_centroid_x = cluster.centroid_x - cand.x;
      const double to_centroid_y = cluster.centroid_y - cand.y;
      constexpr double kYawDegenerateEps = 1e-6;
      if (std::hypot(to_centroid_x, to_centroid_y) > kYawDegenerateEps) {
        cand.yaw = std::atan2(to_centroid_y, to_centroid_x);
      } else {
        cand.yaw = std::atan2(cand.y - robot_y, cand.x - robot_x);
      }

      cand.distance = std::hypot(cand.x - robot_x, cand.y - robot_y);

      if (inflated_occupied_[cell] != 0U) {
        cand.reject_reason = "落在障碍物/膨胀区内";
      } else if (reachable_[cell] == 0U) {
        cand.reject_reason = "不可达";
      } else if (cand.distance < params_.min_goal_distance) {
        cand.reject_reason = "距离机器人过近";
      } else if (params_.max_goal_distance > 0.0 && cand.distance > params_.max_goal_distance) {
        cand.reject_reason = "距离机器人过远";
      } else if (!hasClearance(map, mx, my)) {
        cand.reject_reason = "净空不足";
      } else {
        cand.valid = true;
      }

      if (cand.valid) {
        cand.gain_normalized = static_cast<double>(cluster.unknown_gain) /
          static_cast<double>(max_gain);
        cand.visit_penalty = visitPenaltyAt(cand.x, cand.y, history);
        cand.cost = (params_.weight_distance * cand.distance) +
          (params_.weight_visit_penalty * cand.visit_penalty) -
          (params_.weight_gain * cand.gain_normalized);

        if (out.best_candidate_index < 0 || cand.cost < best_cost) {
          best_cost = cand.cost;
          out.best_candidate_index = static_cast<int>(out.candidates.size());
        }
      }
      out.candidates.push_back(std::move(cand));
    }
  }

  std::ostringstream oss;
  oss << "前沿格=" << out.raw_frontier_cell_count
      << " 前沿块=" << out.clusters.size()
      << "(通过 " << out.accepted_cluster_count << ")"
      << " 候选点=" << out.candidates.size();
  if (out.best_candidate_index >= 0) {
    const GoalCandidate & best = out.candidates[static_cast<std::size_t>(out.best_candidate_index)];
    oss << " 最优代价=" << best.cost
        << " 距离=" << best.distance
        << " 增益=" << best.gain_normalized;
  } else {
    oss << " 无有效候选";
  }
  out.summary += oss.str();
}

}  // namespace astribot_s1_autonomy
