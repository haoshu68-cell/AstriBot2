#ifndef ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_SETTLING_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_SETTLING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace astribot_s1_path_tracking
{
// Only observations newer than a zero command can certify a stopped axis.
template<std::size_t N>
class ArrivalSettling
{
public:
  struct Evidence
  {
    bool stopped{false};
    double span{0.}, speed{0.}, drift{0.};
    std::size_t samples{0};
  };

  void reset() {samples_.clear(); zero_since_ = -1.; evidence_ = {};}
  void command(bool zero, double now)
  {
    if (!zero) {reset();}
    else if (zero_since_ < 0.) {zero_since_ = now;}
  }
  const Evidence & evidence() const {return evidence_;}

  void observe(double stamp, double now, std::array<double, N> value,
    double duration, double max_gap, double stopped_speed, double max_drift,
    bool angular = false)
  {
    if (!std::isfinite(stamp) || !std::isfinite(now) || now < zero_since_ ||
      now-stamp < -0.05 || now-stamp > max_gap ||
      !std::all_of(value.begin(), value.end(), [](double v) {return std::isfinite(v);}))
    {reset(); return;}
    if (zero_since_ < 0.) {return;}
    if (stamp < zero_since_) {
      if (!samples_.empty()) {reset(); zero_since_ = now;}
      return;
    }
    if (!samples_.empty()) {
      const auto & last = samples_.back();
      if (angular) {
        value[0] = last.value[0] + std::remainder(value[0]-last.value[0], 2.*std::acos(-1.));
      }
      if (stamp <= last.stamp) {
        if (stamp < last.stamp || value != last.value) {reset(); zero_since_ = now;}
        return;
      }
      if (stamp-last.stamp > max_gap) {samples_.clear(); evidence_ = {};}
    }
    samples_.push_back({stamp, value});
    while (samples_.size() > 2 && samples_[1].stamp <= stamp-duration) {samples_.pop_front();}
    // Bound memory even if a malformed source advances its stamps by tiny increments.
    if (samples_.size() > 512) {samples_.pop_front();}
    evidence_ = {};
    evidence_.samples = samples_.size();
    evidence_.span = stamp-samples_.front().stamp;
    if (samples_.size() < 3 || evidence_.span+1e-9 < duration) {return;}

    std::array<double, N> mean{}, low = value, high = value;
    double mean_t = 0.;
    for (const auto & sample : samples_) {
      mean_t += sample.stamp-samples_.front().stamp;
      for (std::size_t i = 0; i < N; ++i) {
        mean[i] += sample.value[i]-samples_.front().value[i];
        low[i] = std::min(low[i], sample.value[i]);
        high[i] = std::max(high[i], sample.value[i]);
      }
    }
    mean_t /= samples_.size();
    for (auto & v : mean) {v /= samples_.size();}
    double variance = 0., slope_sq = 0., endpoint_sq = 0., drift_sq = 0.;
    std::array<double, N> covariance{};
    for (const auto & sample : samples_) {
      const double t = sample.stamp-samples_.front().stamp-mean_t;
      variance += t*t;
      for (std::size_t i = 0; i < N; ++i) {
        covariance[i] += t*(sample.value[i]-samples_.front().value[i]-mean[i]);
      }
    }
    if (variance <= 0.) {return;}
    for (std::size_t i = 0; i < N; ++i) {
      const double slope = covariance[i]/variance;
      const double endpoint = (value[i]-samples_.front().value[i])/evidence_.span;
      slope_sq += slope*slope; endpoint_sq += endpoint*endpoint;
      drift_sq += (high[i]-low[i])*(high[i]-low[i]);
    }
    evidence_.speed = std::sqrt(std::max(slope_sq, endpoint_sq));
    evidence_.drift = std::sqrt(drift_sq);
    evidence_.stopped = evidence_.speed <= stopped_speed && evidence_.drift <= max_drift;
  }

private:
  struct Sample {double stamp; std::array<double, N> value;};
  std::deque<Sample> samples_;
  double zero_since_{-1.};
  Evidence evidence_;
};
}  // namespace astribot_s1_path_tracking
#endif
