// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/align_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
  if (std::hypot(dx, dy) < 1e-9) {
    return false;
  }
  heading = std::atan2(dy, dx);
  return true;
}

bool isSameGoal(const PlanarPoint & prev_end, const PlanarPoint & cur_end, double eps_m)
{
  if (!(eps_m > 0.0)) {
    return false;
  }
  return std::hypot(cur_end.x - prev_end.x, cur_end.y - prev_end.y) <= eps_m;
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
  double out = std::max(mag, std::fabs(floor_vel));
  out = std::min(out, std::fabs(max_vel));
  return sign * out;
}

double coastAngle(double wz, double lag_s, double decel_rad_s2)
{
  if (!(lag_s >= 0.0) || !(decel_rad_s2 > 0.0)) {
    return 0.0;
  }
  const double w = std::fabs(wz);
  if (!(w > 0.0)) {              // NaN 也走这一支
    return 0.0;
  }
  return w * lag_s + w * w / (2.0 * decel_rad_s2);
}

double predictedHeadingError(double error_rad, double wz, double lag_s, double decel_rad_s2)
{
  const double coast = coastAngle(wz, lag_s, decel_rad_s2);
  if (!(coast > 0.0)) {
    return error_rad;
  }
  const double dir = (wz > 0.0) ? 1.0 : -1.0;      // wz==0 已被上面挡住
  return error_rad - dir * coast;
}

double alignAngularVelocityWithInertia(
  double error_rad, double wz, double kp, double max_vel, double floor_vel, double tol_rad,
  double lag_s, double decel_rad_s2)
{
  const double e_pred = predictedHeadingError(error_rad, wz, lag_s, decel_rad_s2);
  if (std::fabs(e_pred) <= tol_rad) {
    return 0.0;                  // 剩下的角度正好被惯性吃掉：现在就松手
  }
  if (e_pred * error_rad <= 0.0) {
    return 0.0;
  }
  return alignAngularVelocity(e_pred, kp, max_vel, floor_vel, tol_rad);
}

bool headingSettled(double error_rad, double wz, double tol_rad, double settled_wz)
{
  if (!(std::fabs(error_rad) <= tol_rad)) {
    return false;                // NaN 走这一支：判"没到位"，保守方向
  }
  if (!(settled_wz > 0.0)) {
    return true;
  }
  return std::fabs(wz) <= settled_wz;
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
  return advancePhase(
    current, start_error_rad, goal_error_rad, dist_to_goal_m, xy_tol_m, align_tol_rad,
    start_min_rad, align_goal_enabled, 0.0, std::numeric_limits<double>::infinity());
}

Phase advancePhase(
  Phase current,
  double start_error_rad,
  double goal_error_rad,
  double dist_to_goal_m,
  double xy_tol_m,
  double align_tol_rad,
  double start_min_rad,
  bool align_goal_enabled,
  double wz,
  double settled_wz)
{
  switch (current) {
    case Phase::kAlignStart:
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
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return headingSettled(goal_error_rad, wz, align_tol_rad, settled_wz) ?
             Phase::kDone : Phase::kAlignGoal;

    case Phase::kAlignGoal:
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return headingSettled(goal_error_rad, wz, align_tol_rad, settled_wz) ?
             Phase::kDone : Phase::kAlignGoal;

    case Phase::kDone:
      if (dist_to_goal_m > xy_tol_m) {
        return Phase::kFollow;
      }
      if (align_goal_enabled &&
        !headingSettled(goal_error_rad, wz, align_tol_rad, settled_wz))
      {
        return Phase::kAlignGoal;
      }
      return Phase::kDone;
  }
  return Phase::kDone;
}

double approachSpeedCap(
  double dist_to_goal_m, double approach_dist_m, double v_min, double nominal_speed)
{
  if (!(approach_dist_m > 0.0) || !(v_min >= 0.0) || !(nominal_speed > 0.0)) {
    return nominal_speed;
  }
  if (!(dist_to_goal_m < approach_dist_m)) {
    return nominal_speed;
  }
  const double d = std::max(0.0, dist_to_goal_m);
  const double linear = nominal_speed * (d / approach_dist_m);
  return std::min(std::max(linear, v_min), nominal_speed);
}

bool shouldRestartPhaseTimer(Phase before, Phase requested, bool is_new_goal)
{
  if (before != requested) {
    return true;                 // 相位变了，计时器天然要从头算
  }
  return is_new_goal;
}

bool isFreshFollowAttempt(bool has_prev_tick, double idle_gap_sec, double gap_threshold_sec)
{
  if (!has_prev_tick) {
    return true;                 // 从没 tick 过 = 这是第一次下发，当然是新尝试
  }
  if (!(gap_threshold_sec > 0.0)) {
    return false;
  }
  return idle_gap_sec > gap_threshold_sec;
}

}  // namespace astribot_s1_path_tracking
