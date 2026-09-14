// Copyright 2026 Astribot. Apache-2.0.
#ifndef ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_LIMIT_CRITIC_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_LIMIT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "astribot_s1_path_tracking/curvature_speed_math.hpp"

namespace mppi::critics
{
namespace curvature_speed = astribot_s1_path_tracking::curvature_speed;

class CurvatureSpeedLimitCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;
private:
  unsigned int power_{2};
  float weight_{6.0f}, path_weight_{6.0f}, threshold_to_consider_{0.5f};
  double t_react_{1.4}, lookahead_min_dist_{0.2}, brake_accel_{0.25};
  curvature_speed::Limits limits_;
};
}
#endif
