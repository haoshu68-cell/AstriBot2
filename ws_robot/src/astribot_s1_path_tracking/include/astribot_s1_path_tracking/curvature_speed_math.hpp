// Copyright 2026 Astribot. Apache-2.0.
#ifndef ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_MATH_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_MATH_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace astribot_s1_path_tracking
{
namespace curvature_speed
{

constexpr double kEpsSpeed = 1e-3;

constexpr double kEpsLen = 1e-6;

struct Limits
{

  double a_lat_max{0.35};

  double soft_ratio{0.6};

  double v_min_turn{0.08};

  double v_max{1.0};
};

bool validate(const Limits & lim, std::string & why);

struct Lookahead
{

  double t_react{2.0};

  double min_dist{0.20};

  double horizon_s{2.8};

  double a_brake{0.25};
};

struct LookaheadResult
{
  double dist{0.0};

  bool clamped_to_horizon{false};

  bool clamped_to_min{false};
};

bool validate(const Lookahead & la, std::string & why);

bool validateBrake(double a_brake, std::string & why);

LookaheadResult lookaheadDistance(double v_ref, const Lookahead & la);

double speedForLateralAccel(double a_budget, double kappa, const Limits & lim);


double softSpeedCap(double kappa, const Limits & lim);




double mengerCurvature(
  double x0, double y0,
  double x1, double y1,
  double x2, double y2);


double discountCurvatureByBrakingDistance(
  double kappa, double arc, const Limits & lim, double a_brake);

double brakingWindowDepth(const Limits & lim, double a_brake);

struct WindowCurvature
{

  double kappa_effective{0.0};

  double kappa_raw{0.0};

  double bind_arc{0.0};

  std::size_t bind_idx{0};

  double kappa_raw_max{0.0};
};

WindowCurvature bindingCurvatureInWindow(
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  std::size_t start_idx,
  double lookahead_dist,
  const Limits & lim,
  double a_brake);

}  // namespace curvature_speed
}  // namespace astribot_s1_path_tracking
#endif
