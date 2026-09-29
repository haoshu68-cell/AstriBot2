#pragma once
#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include "astribot_s1_navigation_policy_native/policy_sweep.hpp"
#include <limits>

namespace astribot::navigation::policy {
struct RobotState {
  const double x, y, yaw, vx, vy, wz;
  RobotState(double x, double y, double yaw, double vx=0., double vy=0., double wz=0.);
};
struct RiskProfile {
  SweepProfile sweep;
  double max_speed_m_s, reaction_time_s, brake_deceleration_m_s2;
  double angular_brake_deceleration_rad_s2, linear_stop_delay_s, prediction_horizon_s;
};
struct Risk {
  bool blocked, immediate;
  double clearance_m, conflict_time_s;
  std::vector<std::string> obstacle_ids;
  bool moving, uncertain;
  std::vector<std::string> immediate_obstacle_ids;
};
struct PredictionRow {
  std::size_t owner;
  std::int64_t offset_ns;
  Bounds bounds;
};
Bounds obstacle_bounds(const MetricBox& box, const MetricBox* previous=nullptr);
std::vector<PredictionRow> prediction_rows(const WorldSnapshot& world,
  bool include_current=false, bool swept=false);
bool has_predictions(const TrackedObstacle& track);
std::size_t prediction_count(const TrackedObstacle& track);
MetricBox final_prediction(const TrackedObstacle& track);
Risk evaluate_risk(const WorldSnapshot& world, const RobotState& robot,
  const Polygon& path, const RiskProfile& profile,
  std::optional<double> speed_limit=std::nullopt);
}  // namespace astribot::navigation::policy
