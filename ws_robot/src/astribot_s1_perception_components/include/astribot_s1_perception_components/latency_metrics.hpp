#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace astribot::vision {
struct LatencySummary {
  uint64_t samples=0,overflow_samples=0;
  double p50_upper_ms=0,p95_upper_ms=0,p99_upper_ms=0,max_ms=0;
};
// Fixed memory, cumulative samples. Quantiles are conservative 0.05 ms bin
// upper bounds, not exact percentiles. Overflow quantiles return lifetime max.
class LatencyHistogram {
public:
  void add_ms(double value) {
    if(!std::isfinite(value)||value<0)throw std::invalid_argument("invalid latency");
    ++samples_;maximum_=std::max(maximum_,value);
    if(value>=bin_ms*bins_.size())++overflow_;
    else ++bins_[static_cast<size_t>(value/bin_ms)];
  }
  LatencySummary summary()const {
    return {samples_,overflow_,upper(.50),upper(.95),upper(.99),maximum_};
  }
private:
  double upper(double fraction)const {
    if(!samples_)return 0.;
    const auto rank=static_cast<uint64_t>(std::ceil(fraction*samples_));uint64_t total=0;
    for(size_t i=0;i<bins_.size();++i) {
      total+=bins_[i];if(total>=rank)return std::min(maximum_,(i+1)*bin_ms);
    }
    return maximum_;
  }
  static constexpr double bin_ms=.05;
  std::array<uint64_t,8192>bins_{};
  uint64_t samples_=0,overflow_=0;double maximum_=0;
};
}
