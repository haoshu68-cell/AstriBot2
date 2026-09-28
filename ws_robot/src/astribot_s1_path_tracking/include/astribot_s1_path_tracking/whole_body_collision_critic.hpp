#pragma once
#include "nav2_mppi_controller/critic_function.hpp"
#include "astribot_s1_path_tracking/layered_collision_reader.hpp"

namespace mppi::critics {
// Fixed posture: transform cached height polygons by base rollouts; no joint FK.
class WholeBodyCollisionCritic : public CriticFunction {
public:
  void initialize() override;
  void score(CriticData &data) override;
private:
  astribot_s1_path_tracking::LayeredCollisionReader reader_;
};
}
