// Copyright 2026 Astribot.

#include "astribot_s1_autonomy/escape_logic.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace astribot_s1_autonomy
{

namespace
{

/// 把任意角归一化到 (-pi, pi]。与 align_math::normalizeAngle 同一约定。
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

/// a 到 b 的最短有向角差。
double shortestAngularDiff(double from, double to)
{
  return normalizeAngle(to - from);
}

/// 取线段采样步长。cfg 未指定时用 resolution/2 —— 半格保证不会跳过任何一格。
double resolveStep(const GridMap & map, const EscapeSearchConfig & cfg)
{
  if (cfg.segment_step_m > 0.0) {
    return cfg.segment_step_m;
  }
  return map.resolution > 0.0 ? (map.resolution * 0.5) : 0.025;
}

/// 读一张三态图在世界坐标处的值。越界返回 false。
bool readTri(const GridMap & g, double wx, double wy, int & value)
{
  if (!g.consistent()) {
    return false;
  }
  unsigned int mx = 0U;
  unsigned int my = 0U;
  if (!g.worldToMap(wx, wy, mx, my)) {
    return false;
  }
  value = static_cast<int>(g.data[g.index(mx, my)]);
  return true;
}

/// 点到线段的距离**平方**。返回平方是为了不开方 —— 扫掠里这是最内层调用。
double distSqPointToSegment(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double vv = (vx * vx) + (vy * vy);
  double t = 0.0;
  if (vv > 0.0) {
    t = std::max(0.0, std::min(1.0, ((wx * vx) + (wy * vy)) / vv));
  }
  const double dx = wx - (t * vx);
  const double dy = wy - (t * vy);
  return (dx * dx) + (dy * dy);
}

/// 足迹环带扫掠：线段外扩 footprint_radius_m 的区域内是否存在物理占据格。
///
/// 用「bbox 枚举 + 点到线段距离」而不是「沿线段每点扫一个圆盘」：
/// 后者相邻采样圆盘重叠率极高，同一格会被查上百次。实测量级差异是
/// 每段 ~1300 格 vs ~95000 格 —— 本项目已有一次「小样本单测全绿、
/// 真实规模直接卡死」的记录，这里不再重犯。
///
/// @return true = 环带内无物理占据（可通行）
bool footprintAnnulusClear(
  const PlanarPoint & a,
  const PlanarPoint & b,
  const GridMap & map,
  const EscapeSearchConfig & cfg)
{
  const double radius = cfg.footprint_radius_m;
  if (radius <= 0.0) {
    return true;      // 显式关闭（旧行为）
  }
  if (map.resolution <= 0.0) {
    return false;
  }

  const double min_wx = std::min(a.x, b.x) - radius;
  const double max_wx = std::max(a.x, b.x) + radius;
  const double min_wy = std::min(a.y, b.y) - radius;
  const double max_wy = std::max(a.y, b.y) + radius;

  const auto to_index = [&map](double w, double origin, unsigned int extent) -> long {
      const double raw = std::floor((w - origin) / map.resolution);
      if (raw < 0.0) {
        return 0L;
      }
      const long hi = static_cast<long>(extent) - 1L;
      return (raw > static_cast<double>(hi)) ? hi : static_cast<long>(raw);
    };

  const long i0 = to_index(min_wx, map.origin_x, map.width);
  const long i1 = to_index(max_wx, map.origin_x, map.width);
  const long j0 = to_index(min_wy, map.origin_y, map.height);
  const long j1 = to_index(max_wy, map.origin_y, map.height);

  const long cells = (i1 - i0 + 1L) * (j1 - j0 + 1L);
  if (cells <= 0L || static_cast<std::size_t>(cells) > cfg.max_footprint_cells) {
    return false;     // 超上限判不可通行，不降分辨率
  }

  const double r_sq = radius * radius;
  for (long j = j0; j <= j1; ++j) {
    for (long i = i0; i <= i1; ++i) {
      const auto mx = static_cast<unsigned int>(i);
      const auto my = static_cast<unsigned int>(j);
      const int tri = static_cast<int>(map.data[map.index(mx, my)]);
      if (!isPhysicallyOccupied(tri, cfg.thresholds)) {
        continue;
      }
      const double cx = map.worldX(mx);
      const double cy = map.worldY(my);
      if (distSqPointToSegment(cx, cy, a.x, a.y, b.x, b.y) <= r_sq) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

const char * toString(EscapeVerdict v)
{
  switch (v) {
    case EscapeVerdict::kNone:
      return "NONE";
    case EscapeVerdict::kEscape:
      return "ESCAPE";
    case EscapeVerdict::kBlockedPhysically:
      return "BLOCKED_PHYSICALLY";
    case EscapeVerdict::kDataInsufficient:
      return "DATA_INSUFFICIENT";
  }
  return "UNKNOWN";
}

bool isPlannerLethal(int tri_value, const EscapeGridThresholds & th)
{
  return tri_value >= 0 && tri_value >= th.occupied_threshold;
}

bool isPhysicallyOccupied(int tri_value, const EscapeGridThresholds & th)
{
  return tri_value >= 0 && tri_value >= th.occupied_threshold;
}

bool isUnknownCell(int tri_value)
{
  return tri_value < 0;
}

EscapeVerdict evaluateEscapeTrigger(
  const CellReading & reading,
  int consecutive_failures,
  const EscapeTriggerConfig & cfg,
  std::string & why)
{
  std::ostringstream oss;

  if (reading.map_valid && isPhysicallyOccupied(reading.map_tri, cfg.thresholds)) {
    oss << "物理真堵：/map 在机器人所在格判占据(值 " << reading.map_tri
        << " >= " << cfg.thresholds.occupied_threshold << ")，禁止脱困";
    why = oss.str();
    return EscapeVerdict::kBlockedPhysically;
  }

  if (!reading.costmap_valid || !reading.map_valid) {
    oss << "数据不足：costmap_valid=" << (reading.costmap_valid ? "true" : "false")
        << " map_valid=" << (reading.map_valid ? "true" : "false")
        << "，保守拒绝脱困";
    why = oss.str();
    return EscapeVerdict::kDataInsufficient;
  }
  if (isUnknownCell(reading.map_tri)) {
    oss << "物理状态未知：/map 在机器人所在格为未知(-1)，未知不等于可通行，拒绝脱困";
    why = oss.str();
    return EscapeVerdict::kDataInsufficient;
  }

  if (!isPlannerLethal(reading.costmap_tri, cfg.thresholds)) {
    oss << "起点在 costmap 上非致命(值 " << reading.costmap_tri
        << ")，规划失败原因在目标侧，应换候选点而非脱困";
    why = oss.str();
    return EscapeVerdict::kNone;
  }

  if (consecutive_failures < cfg.trigger_failures) {
    oss << "连续规划失败 " << consecutive_failures << " 次，未达触发阈值 "
        << cfg.trigger_failures << "，暂不脱困";
    why = oss.str();
    return EscapeVerdict::kNone;
  }

  oss << "起点落在膨胀带(costmap 致命，值 " << reading.costmap_tri
      << ")但 /map 判空闲(值 " << reading.map_tri << ")，连续规划失败 "
      << consecutive_failures << " 次 ⇒ 允许脱困";
  why = oss.str();
  return EscapeVerdict::kEscape;
}

bool segmentPhysicallyClear(
  const PlanarPoint & a,
  const PlanarPoint & b,
  const GridMap & map,
  const EscapeSearchConfig & cfg)
{
  if (!map.consistent()) {
    return false;      // 无图 ⇒ 不可通行（禁止「无数据当成安全」）
  }
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  const double step = resolveStep(map, cfg);
  if (step <= 0.0) {
    return false;
  }

  std::size_t n = static_cast<std::size_t>(std::ceil(len / step)) + 1U;
  if (n > cfg.max_segment_samples) {
    return false;
  }
  if (n < 1U) {
    n = 1U;
  }

  for (std::size_t i = 0U; i < n; ++i) {
    const double t = (n == 1U) ? 0.0 : (static_cast<double>(i) / static_cast<double>(n - 1U));
    const double x = a.x + (dx * t);
    const double y = a.y + (dy * t);
    int tri = 0;
    if (!readTri(map, x, y, tri)) {
      return false;    // 出图 ⇒ 不可通行
    }
    if (isUnknownCell(tri) || isPhysicallyOccupied(tri, cfg.thresholds)) {
      return false;
    }
  }

  return footprintAnnulusClear(a, b, map, cfg);
}

bool pickBreadcrumbTarget(
  const std::vector<Breadcrumb> & trail,
  const PlanarPoint & robot,
  const GridMap & costmap,
  const GridMap & map,
  const EscapeSearchConfig & cfg,
  PlanarPoint & target,
  std::string & why)
{
  if (trail.empty()) {
    why = "来路轨迹为空(通常是刚启动)，改用最近可规划格搜索";
    return false;
  }

  std::vector<std::size_t> order(trail.size());
  for (std::size_t i = 0U; i < trail.size(); ++i) {
    order[i] = trail.size() - 1U - i;
  }

  std::size_t checked = 0U;
  std::size_t rejected_far = 0U;
  std::size_t rejected_lethal = 0U;
  std::size_t rejected_map = 0U;
  std::size_t rejected_blocked = 0U;

  for (const std::size_t idx : order) {
    const PlanarPoint & p = trail[idx].p;
    ++checked;

    const double dist = std::hypot(p.x - robot.x, p.y - robot.y);
    if (dist <= 1e-6 || dist > cfg.search_radius_m) {
      ++rejected_far;
      continue;
    }

    int cost_tri = 0;
    if (!readTri(costmap, p.x, p.y, cost_tri) ||
      isPlannerLethal(cost_tri, cfg.thresholds))
    {
      ++rejected_lethal;
      continue;      // 这个历史点现在也在带里（或已出图），换下一个
    }

    int map_tri = 0;
    if (!readTri(map, p.x, p.y, map_tri) || isUnknownCell(map_tri) ||
      isPhysicallyOccupied(map_tri, cfg.thresholds))
    {
      ++rejected_map;
      continue;
    }
    if (cfg.require_target_map_free && map_tri > cfg.thresholds.free_threshold) {
      ++rejected_map;
      continue;
    }

    if (!segmentPhysicallyClear(robot, p, map, cfg)) {
      ++rejected_blocked;
      continue;      // 直线被墙挡住 —— 直驱没有规划器，必须查这一条
    }

    target = p;
    std::ostringstream oss;
    oss << "来路脱困：取轨迹第 " << idx << " 点 (" << p.x << ", " << p.y
        << ")，距 " << dist << "m（扫了 " << checked << " 个点）";
    why = oss.str();
    return true;
  }

  std::ostringstream oss;
  oss << "来路轨迹 " << trail.size() << " 点全部不可用（超距/出图 " << rejected_far
      << "、仍在膨胀带 " << rejected_lethal << "、物理不可站 " << rejected_map
      << "、直线被挡 " << rejected_blocked << "），改用最近可规划格搜索";
  why = oss.str();
  return false;
}

bool findNearestPlannableCell(
  const PlanarPoint & robot,
  double ref_heading,
  const GridMap & costmap,
  const GridMap & map,
  const EscapeSearchConfig & cfg,
  PlanarPoint & target,
  bool & used_relaxed_pass,
  std::string & why)
{
  used_relaxed_pass = false;
  if (!costmap.consistent() || !map.consistent()) {
    why = "costmap 或 /map 不可用，无法搜索脱困目标";
    return false;
  }
  if (costmap.resolution <= 0.0) {
    why = "costmap 分辨率非法，无法搜索";
    return false;
  }

  unsigned int cx = 0U;
  unsigned int cy = 0U;
  if (!costmap.worldToMap(robot.x, robot.y, cx, cy)) {
    why = "机器人位置不在 costmap 范围内，无法搜索脱困目标";
    return false;
  }
  const int max_r = static_cast<int>(std::floor(cfg.search_radius_m / costmap.resolution));
  if (max_r < 1) {
    why = "search_radius_m 小于一个栅格，无法搜索";
    return false;
  }

  std::size_t n_scanned = 0U;      // 扫过的格子总数
  std::size_t rej_range = 0U;      // 超出 search_radius_m 或距离为 0
  std::size_t rej_heading = 0U;    // 方向约束（仅趟 1）
  std::size_t rej_lethal = 0U;     // costmap 致命/未知 ⇒ 规划器不接受
  std::size_t rej_map = 0U;        // /map 上未知或物理占据
  std::size_t rej_map_free = 0U;   // require_target_map_free：/map 不是明确空闲
  std::size_t rej_blocked = 0U;    // 直线段物理被挡

  for (int pass = 0; pass < 2; ++pass) {
    const bool constrain = (pass == 0);
    for (int r = 1; r <= max_r; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {
            continue;      // 不在这一环上
          }
          const int nx = static_cast<int>(cx) + dx;
          const int ny = static_cast<int>(cy) + dy;
          if (nx < 0 || ny < 0 ||
            nx >= static_cast<int>(costmap.width) || ny >= static_cast<int>(costmap.height))
          {
            continue;
          }
          const auto ux = static_cast<unsigned int>(nx);
          const auto uy = static_cast<unsigned int>(ny);
          const double wx = costmap.worldX(ux);
          const double wy = costmap.worldY(uy);
          ++n_scanned;

          const double dist = std::hypot(wx - robot.x, wy - robot.y);
          if (dist <= 1e-6 || dist > cfg.search_radius_m) {
            ++rej_range;
            continue;
          }

          if (constrain) {
            const double bearing = std::atan2(wy - robot.y, wx - robot.x);
            if (std::fabs(shortestAngularDiff(ref_heading, bearing)) > cfg.heading_tol_rad) {
              ++rej_heading;
              continue;
            }
          }

          const int cost_tri = static_cast<int>(costmap.data[costmap.index(ux, uy)]);
          if (isPlannerLethal(cost_tri, cfg.thresholds) || isUnknownCell(cost_tri)) {
            ++rej_lethal;
            continue;
          }

          int map_tri = 0;
          if (!readTri(map, wx, wy, map_tri) || isUnknownCell(map_tri) ||
            isPhysicallyOccupied(map_tri, cfg.thresholds))
          {
            ++rej_map;
            continue;
          }
          if (cfg.require_target_map_free && map_tri > cfg.thresholds.free_threshold) {
            ++rej_map_free;
            continue;
          }

          PlanarPoint cand{wx, wy};
          if (!segmentPhysicallyClear(robot, cand, map, cfg)) {
            ++rej_blocked;
            continue;
          }

          target = cand;
          used_relaxed_pass = !constrain;
          std::ostringstream oss;
          oss << "最近可规划格：(" << wx << ", " << wy << ")，距 " << dist
              << "m，环半径 " << r << " 格，"
              << (constrain ? "方向约束内" : "**已放开方向约束**(趟1无解)");
          why = oss.str();
          return true;
        }
      }
    }
  }

  std::ostringstream oss;
  oss << "半径 " << cfg.search_radius_m << "m 内没有任何「规划器可接受且物理可直达」的格子，"
      << "脱困无解（扫 " << n_scanned << " 格：超距 " << rej_range
      << "、方向约束 " << rej_heading << "、膨胀带/未知 " << rej_lethal
      << "、物理占据/未知 " << rej_map << "、非明确空闲 " << rej_map_free
      << "、直线被挡 " << rej_blocked << "）";
  why = oss.str();
  return false;
}

EscapeCommand escapeVelocity(
  const PlanarPoint & robot,
  double robot_yaw,
  const PlanarPoint & target,
  const EscapeLimits & lim)
{
  EscapeCommand cmd;
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double dist = std::hypot(dx, dy);

  if (dist <= lim.arrive_tol_m) {
    cmd.arrived = true;
    return cmd;      // 全零 + arrived
  }

  const double bearing_world = std::atan2(dy, dx);
  const double bearing_body = shortestAngularDiff(robot_yaw, bearing_world);
  const double speed = std::min(lim.max_linear, std::max(0.0, lim.max_linear));
  cmd.vx = speed * std::cos(bearing_body);
  cmd.vy = speed * std::sin(bearing_body);

  if (std::fabs(bearing_body) > lim.align_tol_rad) {
    const double wz = std::copysign(lim.max_angular, bearing_body);
    cmd.wz = std::max(-lim.max_angular, std::min(lim.max_angular, wz));
  }
  return cmd;
}

bool escapeCleared(int consecutive_clear_ticks, int need)
{
  if (need <= 0) {
    return false;      // need<=0 是配置错误，按「永不判出带」处理而不是「立刻出带」
  }
  return consecutive_clear_ticks >= need;
}

}  // namespace astribot_s1_autonomy

