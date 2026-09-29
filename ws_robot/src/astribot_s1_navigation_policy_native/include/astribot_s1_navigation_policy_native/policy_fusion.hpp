#pragma once
#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include <array>
#include <functional>
#include <map>
#include <memory>

namespace astribot::navigation::policy {
// Startup configuration copied and validated by ConservativeFusion. Runtime code
// consumes typed values; profile parsing belongs to its caller.
struct FusionProfile {
  double sensor_timeout_s=.5, track_memory_s=1., association_distance_m=.5;
  double velocity_confirmation_s=.4, velocity_fit_window_s=1.;
  double velocity_fit_max_residual_m=.06, min_tracked_speed_m_s=.1;
  double max_obstacle_speed_m_s=2., stationary_velocity_variance_m2_s2=.0004;
  double prediction_horizon_s=2.8, prediction_step_s=.2;
  double half_length_m=.31, half_width_m=.31, clearance_margin_m=.08, payload_extra_margin_m=0.;
  void validate() const;
};
using FusionSample=std::array<double,3>;  // capture seconds, x metres, y metres
struct FusionTrack {
  const std::string identifier;
  const Observation observation;
  const Vec3 velocity;
  const std::vector<FusionSample> samples;
  FusionTrack(std::string identifier, Observation observation, Vec3 velocity, std::vector<FusionSample> samples);
};
Covariance3 expanded_covariance(const Covariance3& covariance,double extra_variance);
MetricBox translate(const MetricBox& box,const Vec3& velocity,double dt,double extra_variance=0.);
Vec3 fitted_velocity(const std::vector<FusionSample>& samples,const FusionProfile& profile);
class ConservativeFusion {
 public:
  const FusionProfile profile;
  const std::string frame_id;
  using FreeAt=std::function<bool(const MetricBox&)>;
  using FreeMany=std::function<std::vector<bool>(const std::vector<MetricBox>&)>;
  explicit ConservativeFusion(FusionProfile profile, std::string frame_id="odom");
  ~ConservativeFusion();
  ConservativeFusion(const ConservativeFusion&)=delete;
  ConservativeFusion& operator=(const ConservativeFusion&)=delete;
  void ingest(const std::vector<Observation>& observations,const Stamp& now);
  WorldSnapshot update(const std::vector<Observation>& observations,const Stamp& now);
  WorldSnapshot snapshot(const Stamp& now,std::optional<std::array<double,3>> region=std::nullopt);
  void resolve_unassociated(const std::string& sensor_id,const std::vector<std::string>& measurement_ids,
    const Stamp& capture,const Stamp& now);
  void clear_observed_free(const Stamp& now,FreeAt free_at={},FreeMany free_many={});
  // Called with accepted effective envelope extents; preserves tracks and clocks.
  void set_envelope_bounds(double half_length_m,double half_width_m);
  void set_version(const Version& version);
  void set_sensors(std::vector<SensorHealth> sensors);
  const Version& version() const;
  const std::vector<SensorHealth>& sensors() const;
  const std::map<std::string,std::shared_ptr<const FusionTrack>>& tracks() const;
  std::int64_t sequence() const;
 private:
  struct State;
  std::unique_ptr<State> state_;
};
}  // namespace astribot::navigation::policy
