// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/align_math.hpp"

#include <algorithm>
#include <cmath>

namespace astribot_s1_path_tracking
{

const char * toString(Phase p)
{
  switch (p) {
    case Phase::kAlignStart:
      return "ALIGN_START";
    case Phase::kFollow:
      return "FOLLOW";
    case Phase::kAlignGoal:
      return "ALIGN_GOAL";
    case Phase::kDone:
      return "DONE";
  }
  return "UNKNOWN";
}

double normalizeAngle(double a)
{
  // atan2(sin, cos) 天然落在 (-pi, pi]，比手写循环减 2pi 稳（不会因大角度死循环）。
  return std::atan2(std::sin(a), std::cos(a));
}

double shortestAngularDiff(double from, double to)
{
  return normalizeAngle(to - from);
}

bool pathStartHeading(
  const std::vector<PlanarPoint> & path, double lookahead_m, double & heading)
{
  if (path.size() < 2U || !(lookahead_m > 0.0)) {
    return false;
  }
  const PlanarPoint & p0 = path.front();
  double acc = 0.0;
  std::size_t idx = 0U;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    acc += std::hypot(path[i].x - path[i - 1U].x, path[i].y - path[i - 1U].y);
    idx = i;
    if (acc >= lookahead_m) {
      break;
    }
  }
  const double dx = path[idx].x - p0.x;
  const double dy = path[idx].y - p0.y;
  // 路径整体退化成一个点（所有顶点重合）时无方向可言 —— 明确失败，
  // 不返回 atan2(0,0)=0 冒充「朝 +x」。
  if (std::hypot(dx, dy) < 1e-9) {
    return false;
  }
  heading = std::atan2(dy, dx);
  return true;
}

bool needsStartAlign(double heading_error_rad, double min_angle_rad)
{
  return std::fabs(heading_error_rad) > min_angle_rad;
}

double alignAngularVelocity(
  double error_rad, double kp, double max_vel, double floor_vel, double tol_rad)
{
  if (std::fabs(error_rad) <= tol_rad) {
    return 0.0;
  }
  double v = kp * error_rad;
  const double mag = std::fabs(v);
  const double sign = (error_rad >= 0.0) ? 1.0 : -1.0;
  // 先压下限再压上限：floor 只保证「能动」，绝不允许因此超过 max。
  double out = std::max(mag, std::fabs(floor_vel));
  out = std::min(out, std::fabs(max_vel));
  return sign * out;
}

Phase advancePhase(
  Phase current,
  double start_error_rad,
  double goal_error_rad,
  double dist_to_goal_m,
  double xy_tol_m,
  double align_tol_rad,
  double start_min_rad,
  bool align_goal_enabled)
{
  switch (current) {
    case Phase::kAlignStart:
      // 已经对上（或本来就不值得转）就进跟踪段。
      if (!needsStartAlign(start_error_rad, start_min_rad) ||
        std::fabs(start_error_rad) <= align_tol_rad)
      {
        return Phase::kFollow;
      }
      return Phase::kAlignStart;

    case Phase::kFollow:
      if (dist_to_goal_m > xy_tol_m) {
        return Phase::kFollow;
      }
      // 位置到了。探索场景刻意不对齐终点姿态 —— 直接结束。
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return (std::fabs(goal_error_rad) <= align_tol_rad) ? Phase::kDone : Phase::kAlignGoal;

    case Phase::kAlignGoal:
      // 防御：本相位在 align_goal_enabled=false 时不该出现；
      // 万一出现（例如运行期改参数），立即收敛到 kDone 而不是继续转。
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return (std::fabs(goal_error_rad) <= align_tol_rad) ? Phase::kDone : Phase::kAlignGoal;

    case Phase::kDone:
      return Phase::kDone;
  }
  return Phase::kDone;
}

}  // namespace astribot_s1_path_tracking
