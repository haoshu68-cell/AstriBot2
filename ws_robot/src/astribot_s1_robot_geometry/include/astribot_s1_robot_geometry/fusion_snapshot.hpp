#pragma once
// Per-track derivation of the conservative fusion snapshot. This replicates the
// arithmetic of astribot_s1_navigation_policy.fusion.ConservativeFusion.snapshot
// (translate, covariance inflation, stationary detection, prediction-model
// variance and region relevance) without owning clocks, authority or leases.
//
// The caller keeps the Python ConservativeFusion state (tracks, epoch,
// sequence, sensors). The kernel only derives the per-track values so the thin
// Python adapter can reconstruct MetricBox / PredictionModel / TrackedObstacle
// objects without repeating the redundant validation already performed at
// ingest time. Semantics (capture time, geometry version, covariance retention,
// unknown/occlusion handling, ownership boundaries) stay unchanged.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace astribot_s1_robot_geometry {

struct SnapshotParams {
  // Optional caller arithmetic contract. Null retains existing binding behavior.
  double (*norm2)(double,double) = nullptr;
  double track_memory_s = 1.0;
  double velocity_confirmation_s = 0.4;
  double min_tracked_speed_m_s = 0.1;
  double velocity_fit_max_residual_m = 0.06;
  double stationary_velocity_variance_m2_s2 = 0.0004;
  // Region relevance filter (empty when has_region is false).
  bool has_region = false;
  double region_x = 0.0, region_y = 0.0, region_travel = 0.0;
  double prediction_horizon_s = 2.8;
  double half_length_m = 0.31, half_width_m = 0.31;
  double clearance_margin_m = 0.08, payload_extra_margin_m = 0.0;
};

struct SnapshotSample { double t, x, y; };

struct SnapshotTrackInput {
  int64_t capture_ns = 0;
  double center[3] = {0., 0., 0.};
  double size[3] = {0., 0., 0.};
  double pos_cov[9] = {0.};
  double track_velocity[3] = {0., 0., 0.};
  bool spatial_occupancy = false;
  bool velocity_observable = true;
  bool has_velocity_m_s = false;      // observation carries a directly measured velocity
  bool has_velocity_covariance = false;
  double velocity_covariance[9] = {0.};  // meaningful only when has_velocity_covariance
  std::vector<SnapshotSample> samples;
};

struct SnapshotTrackOutput {
  double center[3] = {0., 0., 0.};
  double size[3] = {0., 0., 0.};
  double covariance[9] = {0.};
  double velocity[3] = {0., 0., 0.};
  double variance = 0.0;
  bool relevant = true;
};

inline void require(bool ok, const char * reason) {
  if (!ok) { throw std::invalid_argument(reason); }
}

inline void snapshotCovariance(const double * values) {
  double scale=0.;
  for(int k=0;k<9;++k) {
    require(std::isfinite(values[k]),"finite snapshot covariance required");
    scale=std::max(scale,std::abs(values[k]));
  }
  double a[9];for(int k=0;k<9;++k)a[k]=scale>0.?values[k]/scale:values[k];
  for(int i=0;i<3;++i) {
    require(a[i*3+i]>=0.,"nonnegative covariance diagonal required");
    for(int j=i+1;j<3;++j) {
      require(std::abs(a[i*3+j]-a[j*3+i])<=1e-10,"symmetric covariance required");
      require(a[i*3+i]*a[j*3+j]-a[i*3+j]*a[i*3+j]>=-1e-12,"PSD covariance required");
    }
  }
  const double det=a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);
  require(det>=-1e-12,"PSD covariance required");
}

// Derive every track in place. Throws std::invalid_argument on non-finite input
// (matching the frozen Python contracts) or on a malformed sample index.
inline void snapshotTracks(const std::vector<SnapshotTrackInput> & inputs,
                           int64_t now_ns,
                           const SnapshotParams & p,
                           std::vector<SnapshotTrackOutput> & outputs) {
  require(now_ns >= 0, "snapshot now must be nonnegative");
  const auto norm2=[&](double x,double y){return p.norm2?p.norm2(x,y):std::hypot(x,y);};
  for(double value:{p.track_memory_s,p.velocity_confirmation_s,p.min_tracked_speed_m_s,
      p.velocity_fit_max_residual_m,p.stationary_velocity_variance_m2_s2,
      p.prediction_horizon_s,p.half_length_m,p.half_width_m,p.clearance_margin_m,
      p.payload_extra_margin_m})require(std::isfinite(value),"finite snapshot profile required");
  require(p.track_memory_s > 0. && p.velocity_confirmation_s > 0. &&
          p.min_tracked_speed_m_s > 0. && p.velocity_fit_max_residual_m > 0. &&
          p.stationary_velocity_variance_m2_s2 >= 0. && p.prediction_horizon_s > 0.,
          "invalid snapshot profile values");
  require(p.half_length_m>0. && p.half_width_m>0. && p.clearance_margin_m>=0. &&
    p.payload_extra_margin_m>=0.,"nonnegative snapshot safety geometry required");
  require(!p.has_region || (std::isfinite(p.region_x) && std::isfinite(p.region_y) &&
          std::isfinite(p.region_travel) && p.region_travel >= 0.),
          "invalid snapshot region");
  outputs.resize(inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto & in = inputs[i];
    require(in.capture_ns>=0,"nonnegative capture timestamp required");
    for (int k = 0; k < 3; ++k) {
      require(std::isfinite(in.center[k]) && std::isfinite(in.size[k]) &&
              std::isfinite(in.track_velocity[k]), "finite snapshot geometry required");
      require(in.size[k] > 0., "positive snapshot box size required");
    }
    snapshotCovariance(in.pos_cov);
    if (in.has_velocity_covariance) {
      snapshotCovariance(in.velocity_covariance);
    }
    for (const auto & s : in.samples) {
      require(std::isfinite(s.t) && std::isfinite(s.x) && std::isfinite(s.y),
              "finite snapshot sample required");
    }

    const double age = std::max(0.0, static_cast<double>(now_ns - in.capture_ns) * 1e-9);
    const bool old = age > p.track_memory_s;

    // Prediction velocity is zeroed once the track exceeds memory, exactly as
    // the reference zeroes `velocity` while translate still uses track velocity.
    double velocity[3] = {old ? 0.0 : in.track_velocity[0],
                          old ? 0.0 : in.track_velocity[1],
                          old ? 0.0 : in.track_velocity[2]};

    const bool occupancy_only = !in.velocity_observable && !in.has_velocity_m_s;

    // stationary: enough samples, a long-enough confirmation window, a
    // sub-threshold speed, and every sample within 2x the fit residual of the
    // most recent sample.
    bool stationary = in.samples.size() >= 5 &&
        (in.samples.back().t - in.samples.front().t) >= p.velocity_confirmation_s - 1e-8 &&
        norm2(in.track_velocity[0], in.track_velocity[1]) < p.min_tracked_speed_m_s;
    if (stationary) {
      const double last_x = in.samples.back().x, last_y = in.samples.back().y;
      for (const auto & s : in.samples) {
        if (norm2(s.x - last_x, s.y - last_y) > 2.0 * p.velocity_fit_max_residual_m) {
          stationary = false;
          break;
        }
      }
    }

    double variance = (occupancy_only || stationary)
        ? p.stationary_velocity_variance_m2_s2 : 0.01;
    if (in.has_velocity_covariance) {
      variance = std::max(variance, in.velocity_covariance[0]);
      variance = std::max(variance, in.velocity_covariance[4]);
      variance = std::max(variance, in.velocity_covariance[8]);
    }
    if (in.spatial_occupancy) { variance = 0.0; }
    const double age_variance = occupancy_only ? variance : std::max(variance, 0.04);

    // Translated geometry. A fixed occupied cell is a statement about space and
    // is never moved or inflated by its own variance.
    double center[3] = {in.center[0], in.center[1], in.center[2]};
    double covariance[9];
    for (int k = 0; k < 9; ++k) { covariance[k] = in.pos_cov[k]; }
    if (!in.spatial_occupancy) {
      const double dt = std::min(age, p.track_memory_s);
      const double clamped = std::min(age, 3.0);
      const double extra = clamped * clamped * age_variance;
      const bool zero_velocity =
          in.track_velocity[0] == 0.0 && in.track_velocity[1] == 0.0 && in.track_velocity[2] == 0.0;
      if (!zero_velocity) {
        center[0] += in.track_velocity[0] * dt;
        center[1] += in.track_velocity[1] * dt;
        center[2] += in.track_velocity[2] * dt;
      }
      covariance[0] += extra;
      covariance[4] += extra;
      covariance[8] += extra;
    }

    bool relevant = true;
    for(double value:center)require(std::isfinite(value),"snapshot center overflow");
    snapshotCovariance(covariance);
    require(std::isfinite(variance),"snapshot variance overflow");
    if (p.has_region) {
      const double t = p.prediction_horizon_s;
      const double center_cov = std::max(covariance[0] + t * t * variance,
                                         covariance[4] + t * t * variance);
      const double radius = p.region_travel + p.half_length_m + p.half_width_m +
          p.clearance_margin_m + p.payload_extra_margin_m +
          norm2(in.size[0], in.size[1]) / 2.0 +
          2.0 * std::sqrt(2.0 * center_cov) +
          norm2(velocity[0], velocity[1]) * t;
      require(std::isfinite(radius) && std::isfinite(center_cov),"snapshot region overflow");
      relevant = norm2(center[0] - p.region_x, center[1] - p.region_y) <= radius;
    }

    auto & out = outputs[i];
    out.center[0] = center[0]; out.center[1] = center[1]; out.center[2] = center[2];
    out.size[0] = in.size[0]; out.size[1] = in.size[1]; out.size[2] = in.size[2];
    for (int k = 0; k < 9; ++k) { out.covariance[k] = covariance[k]; }
    out.velocity[0] = velocity[0]; out.velocity[1] = velocity[1]; out.velocity[2] = velocity[2];
    out.variance = variance;
    out.relevant = relevant;
  }
}

}  // namespace astribot_s1_robot_geometry
