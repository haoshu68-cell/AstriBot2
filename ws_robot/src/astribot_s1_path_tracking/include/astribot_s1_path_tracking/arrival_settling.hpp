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

  void reset() {samples_.clear(); source_stamp_ = -1.; zero_since_ = -1.; evidence_ = {};}
  void command(bool zero, double now)
  {
    if (!zero) {reset();}
    else if (zero_since_ < 0.) {zero_since_ = now;}
  }
  const Evidence & evidence() const {return evidence_;}

  void observe(double stamp, double now, std::array<double, N> value,
    double duration, double stopped_speed, double max_drift,
    bool angular = false)
  {
    if (!std::isfinite(stamp) || !std::isfinite(now) || now < zero_since_ ||
      !std::all_of(value.begin(), value.end(), [](double v) {return std::isfinite(v);}))
    {reset(); return;}
    if (zero_since_ < 0.) {return;}
    if(stamp<=source_stamp_)return;
    source_stamp_=stamp;
    if (!samples_.empty() && angular) {
      const auto & last=samples_.back();
      value[0]=last.value[0]+std::remainder(value[0]-last.value[0],2.*std::acos(-1.));
    }
    stamp=now;  // The observation stamp orders samples; the stop window is steady time.
    samples_.push_back({stamp, value});
    while (samples_.size() > 2 && samples_[1].stamp <= stamp-duration) {samples_.pop_front();}
    // Bound memory even if a malformed source advances its stamps by tiny increments.
    if (samples_.size() > 512) {samples_.pop_front();}
    evidence_ = {};
    evidence_.samples = samples_.size();
    evidence_.span = stamp-samples_.front().stamp;
    if (samples_.size() < 2 || evidence_.span+1e-9 < duration) {return;}

    // Signed adjacent increments telescope to displacement from the window
    // anchor. Check every sample's net displacement; never sum absolute motion.
    double endpoint_sq=0.,peak_sq=0.;
    for(const auto &sample:samples_) {
      double net_sq=0.;
      for(std::size_t i=0;i<N;++i) {
        const double delta=sample.value[i]-samples_.front().value[i];
        net_sq+=delta*delta;
      }
      peak_sq=std::max(peak_sq,net_sq);
    }
    for(std::size_t i=0;i<N;++i) {
      const double delta=value[i]-samples_.front().value[i];endpoint_sq+=delta*delta;
    }
    evidence_.speed=std::sqrt(endpoint_sq)/evidence_.span;
    evidence_.drift=std::sqrt(peak_sq);
    evidence_.stopped = evidence_.speed <= stopped_speed && evidence_.drift <= max_drift;
  }

private:
  struct Sample {double stamp; std::array<double, N> value;};
  std::deque<Sample> samples_;
  double zero_since_{-1.}, source_stamp_{-1.};
  Evidence evidence_;
};
}  // namespace astribot_s1_path_tracking
#endif
