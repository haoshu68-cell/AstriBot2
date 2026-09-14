// Copyright 2026 Astribot. Apache-2.0.
#include "astribot_s1_path_tracking/curvature_speed_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace astribot_s1_path_tracking
{
namespace curvature_speed
{

bool validate(const Limits & lim, std::string & why)
{
  if (!(lim.a_lat_max > 0.0)) {
    why = "a_lat_max 必须 > 0（当前 " + std::to_string(lim.a_lat_max) +
      "）；置 0 等于要求侧向加速度恒为 0，任何转弯都会被无限罚。";
    return false;
  }
  if (!(lim.soft_ratio > 0.0) || lim.soft_ratio > 1.0) {
    why = "soft_ratio 必须 in (0, 1]（当前 " + std::to_string(lim.soft_ratio) +
      "）；<=0 会让直行也吃罚，>1 会让惩罚永不触发。";
    return false;
  }
  if (!(lim.v_max > 0.0)) {
    why = "v_max 必须 > 0（当前 " + std::to_string(lim.v_max) + "）。";
    return false;
  }
  if (lim.v_min_turn < 0.0 || lim.v_min_turn > lim.v_max) {
    why = "v_min_turn 必须 in [0, v_max]（当前 " + std::to_string(lim.v_min_turn) +
      " vs v_max " + std::to_string(lim.v_max) + "）。";
    return false;
  }
  return true;
}

bool validateBrake(double a_brake, std::string & why)
{

  if (!(a_brake > 0.0) || !std::isfinite(a_brake)) {
    why = "path_brake_accel 必须 > 0 且有限（当前 " + std::to_string(a_brake) +
      "）；<=0 会让远处曲率不打折，逐字退回旧的无权重 max 行为（弯一进窗口就阶跃降速"
      "并平推整个窗口）。要那个行为请显式写一个很小的正数，不要写 0。";
    return false;
  }
  return true;
}

bool validate(const Lookahead & la, std::string & why)
{
  if (!(la.t_react > 0.0)) {
    why = "path_lookahead_t_react 必须 > 0（当前 " + std::to_string(la.t_react) +
      "）；置 0 会把前视一路压到下限，前馈项只看锚点脚下那一小段。";
    return false;
  }
  if (!(la.min_dist > 0.0)) {
    why = "path_lookahead_min_dist 必须 > 0（当前 " + std::to_string(la.min_dist) +
      "）；见 curvature_speed_math.hpp 文件头关于 0.05m 栅格量化噪声那一段。";
    return false;
  }
  if (!(la.horizon_s > 0.0)) {
    why = "horizon_s 必须 > 0（当前 " + std::to_string(la.horizon_s) +
      "）；它等于 time_steps * model_dt，为 0 说明 CriticData 里的时域是空的。";
    return false;
  }

  return validateBrake(la.a_brake, why);
}

LookaheadResult lookaheadDistance(double v_ref, const Lookahead & la)
{
  LookaheadResult r;

  if (!(v_ref > 0.0) || !std::isfinite(v_ref)) {
    r.dist = la.min_dist;
    r.clamped_to_min = true;
    return r;
  }

  double d = v_ref * la.t_react;

  const double horizon_dist = v_ref * la.horizon_s;
  if (d > horizon_dist) {
    d = horizon_dist;
    r.clamped_to_horizon = true;
  }
  if (d < la.min_dist) {
    d = la.min_dist;
    r.clamped_to_min = true;
    r.clamped_to_horizon = false;
  }

  r.dist = d;
  return r;
}

double speedForLateralAccel(double a_budget, double kappa, const Limits & lim)
{
  if (!(kappa > 0.0) || !(a_budget > 0.0)) {
    return lim.v_max;
  }
  const double v = std::sqrt(a_budget / kappa);
  return std::clamp(v, lim.v_min_turn, lim.v_max);
}



double softSpeedCap(double kappa, const Limits & lim)
{
  return speedForLateralAccel(lim.soft_ratio * lim.a_lat_max, kappa, lim);
}







double mengerCurvature(
  double x0, double y0,
  double x1, double y1,
  double x2, double y2)
{
  const double ax = x1 - x0, ay = y1 - y0;
  const double bx = x2 - x1, by = y2 - y1;
  const double cx = x2 - x0, cy = y2 - y0;

  const double la = std::hypot(ax, ay);
  const double lb = std::hypot(bx, by);
  const double lc = std::hypot(cx, cy);

  if (la < kEpsLen || lb < kEpsLen || lc < kEpsLen) {
    return 0.0;
  }

  const double cross = ax * by - ay * bx;
  return 2.0 * std::fabs(cross) / (la * lb * lc);
}



double brakingWindowDepth(const Limits & lim, double a_brake)
{
  if (!(a_brake > 0.0) || !std::isfinite(a_brake)) {

    return std::numeric_limits<double>::infinity();
  }

  const double num = lim.v_max * lim.v_max - lim.v_min_turn * lim.v_min_turn;
  if (!(num > 0.0)) {
    return 0.0;
  }
  return num / (2.0 * a_brake);
}

double discountCurvatureByBrakingDistance(
  double kappa, double arc, const Limits & lim, double a_brake)
{
  if (!(kappa > 0.0)) {
    return 0.0;
  }

  if (!(arc > 0.0) || !(a_brake > 0.0) || !std::isfinite(arc) || !std::isfinite(a_brake)) {
    return kappa;
  }

  const double a_soft = lim.soft_ratio * lim.a_lat_max;

  const double v_cap = softSpeedCap(kappa, lim);
  const double v_allow = std::sqrt(v_cap * v_cap + 2.0 * a_brake * arc);

  if (v_allow >= lim.v_max) {
    return 0.0;
  }
  if (!(a_soft > 0.0) || !(v_allow > 0.0)) {
    return kappa;
  }

  return a_soft / (v_allow * v_allow);
}

WindowCurvature bindingCurvatureInWindow(
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  std::size_t start_idx,
  double lookahead_dist,
  const Limits & lim,
  double a_brake)
{
  WindowCurvature out;

  const std::size_t n = std::min(xs.size(), ys.size());
  if (n < 3 || start_idx + 2 >= n) {
    return out;
  }

  const double depth = brakingWindowDepth(lim, a_brake);
  const double scan_dist = std::min(lookahead_dist, depth);

  double arc = std::hypot(xs[start_idx + 1] - xs[start_idx], ys[start_idx + 1] - ys[start_idx]);

  for (std::size_t i = start_idx + 1; i + 1 < n; ++i) {
    const double kappa = mengerCurvature(
      xs[i - 1], ys[i - 1], xs[i], ys[i], xs[i + 1], ys[i + 1]);
    out.kappa_raw_max = std::max(out.kappa_raw_max, kappa);

    const double kappa_eff = discountCurvatureByBrakingDistance(kappa, arc, lim, a_brake);
    if (kappa_eff > out.kappa_effective) {
      out.kappa_effective = kappa_eff;
      out.kappa_raw = kappa;
      out.bind_arc = arc;
      out.bind_idx = i;
    }

    if (arc >= scan_dist) {
      break;
    }
    arc += std::hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]);
  }
  return out;
}

}  // namespace curvature_speed
}  // namespace astribot_s1_path_tracking
