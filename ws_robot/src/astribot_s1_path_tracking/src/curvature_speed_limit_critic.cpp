// Copyright 2026 Astribot. Apache-2.0.
#include "astribot_s1_path_tracking/curvature_speed_limit_critic.hpp"

#include <stdexcept>
#include <xtensor/xmath.hpp>
#include "astribot_s1_path_tracking/cost_accumulate.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
void CurvatureSpeedLimitCritic::initialize()
{
  auto get = parameters_handler_->getParamGetter(name_);
  get(power_, "cost_power", 2, ParameterType::Static);
  get(weight_, "cost_weight", 6.0, ParameterType::Static);
  get(path_weight_, "path_cost_weight", 6.0, ParameterType::Static);
  get(t_react_, "path_lookahead_t_react", 1.4, ParameterType::Static);
  get(lookahead_min_dist_, "path_lookahead_min_dist", 0.2, ParameterType::Static);
  get(brake_accel_, "path_brake_accel", 0.25, ParameterType::Static);
  get(threshold_to_consider_, "threshold_to_consider", 0.5, ParameterType::Static);
  get(limits_.a_lat_max, "a_lat_max", 0.35, ParameterType::Static);
  get(limits_.soft_ratio, "soft_ratio", 0.6, ParameterType::Static);
  get(limits_.v_min_turn, "v_min_turn", 0.08, ParameterType::Static);
  parameters_handler_->getParamGetter(parent_name_)(limits_.v_max, "vx_max", 1.0, ParameterType::Static);
  std::string why;
  curvature_speed::Lookahead lookahead{t_react_, lookahead_min_dist_, 1.0, brake_accel_};
  if (!curvature_speed::validate(limits_, why) || !curvature_speed::validate(lookahead, why)) {
    throw std::invalid_argument(name_ + ": " + why);
  }
  if (power_ == 0 || !std::isfinite(weight_) || weight_ < 0 ||
    !std::isfinite(path_weight_) || path_weight_ < 0)
  {throw std::invalid_argument(name_ + ": invalid cost power/weight");}
}

void CurvatureSpeedLimitCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_ || utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path)) {return;}
  const size_t steps = data.state.vx.shape(1);
  if (steps == 0) {return;}
  const auto speed = xt::sqrt(xt::square(data.state.vx) + xt::square(data.state.vy));
  const auto excess = xt::maximum(
    speed * xt::abs(data.state.wz) - static_cast<float>(limits_.soft_ratio * limits_.a_lat_max),
    0.0f) / static_cast<float>(limits_.a_lat_max);
  const xt::xtensor<float, 1> lateral_cost = xt::pow(
    xt::sum(excess, {1}, immediate) * data.model_dt * weight_, power_);
  astribot_s1_path_tracking::cost_accumulate::accumulateInPlace(data.costs, lateral_cost);
  const size_t n = data.path.x.shape(0);
  if (path_weight_ <= 0 || n < 3) {return;}
  std::vector<double> xs(n), ys(n);
  for (size_t i = 0; i < n; ++i) {xs[i] = data.path.x(i); ys[i] = data.path.y(i);}
  const curvature_speed::Lookahead lookahead{
    t_react_, lookahead_min_dist_, steps * data.model_dt, brake_accel_};
  const auto window = curvature_speed::bindingCurvatureInWindow(
    xs, ys, utils::findPathTrajectoryInitialPoint(data),
    curvature_speed::lookaheadDistance(limits_.v_max, lookahead).dist, limits_, brake_accel_);
  if (window.kappa_effective <= 0) {return;}
  const float cap = curvature_speed::softSpeedCap(window.kappa_effective, limits_);
  const xt::xtensor<float, 1> path_cost = xt::pow(
    xt::sum(xt::maximum(speed - cap, 0.0f), {1}, immediate) * data.model_dt * path_weight_, power_);
  astribot_s1_path_tracking::cost_accumulate::accumulateInPlace(data.costs, path_cost);
}
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::CurvatureSpeedLimitCritic, mppi::critics::CriticFunction)
