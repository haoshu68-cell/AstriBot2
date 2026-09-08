// Copyright 2026 Astribot. Apache-2.0.
// 曲率-速度耦合纯函数层实现。推导与取值理由见同名 .hpp 文件头。

#include "astribot_s1_path_tracking/curvature_speed_math.hpp"

#include <algorithm>
#include <cmath>
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

double speedForLateralAccel(double a_budget, double kappa, const Limits & lim)
{
  if (!(kappa > 0.0) || !(a_budget > 0.0)) {
    return lim.v_max;
  }
  const double v = std::sqrt(a_budget / kappa);
  return std::clamp(v, lim.v_min_turn, lim.v_max);
}

double allowedSpeed(double kappa, const Limits & lim)
{
  return speedForLateralAccel(lim.a_lat_max, kappa, lim);
}

double softSpeedCap(double kappa, const Limits & lim)
{
  return speedForLateralAccel(lim.soft_ratio * lim.a_lat_max, kappa, lim);
}

double trajectoryCurvature(double v, double wz)
{
  const double s = std::fabs(v);
  if (s < kEpsSpeed) {
    // 原地旋转：a_lat = v*|wz| 本来就趋 0，这里显式返回 0 是为了不让
    // kappa 变成一个巨大的数去污染日志与前馈项。
    return 0.0;
  }
  return std::fabs(wz) / s;
}

double lateralAccelExcess(double v, double wz, const Limits & lim)
{
  // 注意这里用恒等式 a_lat = v*|wz|，**不经过 kappa**：
  // 既省一次除法，也让 v -> 0 时天然为 0，不需要 eps 保护。
  const double a_lat = std::fabs(v) * std::fabs(wz);
  const double a_soft = lim.soft_ratio * lim.a_lat_max;
  const double excess = a_lat - a_soft;
  if (excess <= 0.0) {
    return 0.0;
  }
  // 归一化到 a_lat_max，使代价量纲无关、权重可跨机型迁移。
  return excess / lim.a_lat_max;
}

double speedExcessOverPathCap(double v, double kappa_path, const Limits & lim)
{
  const double cap = softSpeedCap(kappa_path, lim);
  const double excess = std::fabs(v) - cap;
  return excess > 0.0 ? excess : 0.0;
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

  // 重复点/退化三角形。nav2 的 Path 里确实有重复点，不设这道闸会得到 inf。
  if (la < kEpsLen || lb < kEpsLen || lc < kEpsLen) {
    return 0.0;
  }

  const double cross = ax * by - ay * bx;
  return 2.0 * std::fabs(cross) / (la * lb * lc);
}

double maxCurvatureInWindow(
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  std::size_t start_idx,
  double lookahead_dist)
{
  const std::size_t n = std::min(xs.size(), ys.size());
  if (n < 3 || start_idx + 2 >= n) {
    return 0.0;
  }

  double kappa_max = 0.0;
  double arc = 0.0;
  for (std::size_t i = start_idx + 1; i + 1 < n; ++i) {
    kappa_max = std::max(
      kappa_max,
      mengerCurvature(xs[i - 1], ys[i - 1], xs[i], ys[i], xs[i + 1], ys[i + 1]));

    arc += std::hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]);
    if (arc >= lookahead_dist) {
      break;
    }
  }
  return kappa_max;
}

}  // namespace curvature_speed
}  // namespace astribot_s1_path_tracking
