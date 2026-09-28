#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace astribot_s1_robot_geometry {
// Opaque context contains the complete authorization except renewal timestamps
// and source sample sequence. Payloads remain with the message adapter.
struct LeaseSample {
  int64_t stamp_ns, limits_stamp_ns, valid_until_ns;
  double lease_s;
  bool allowed;
  std::string context;
};
class LeaseSelector {
public:
  int select(const std::vector<LeaseSample> & samples, int64_t now) {
    if (now < 0 || (last_now_ >= 0 && now < last_now_)) clock_fault_ = true;
    last_now_ = now;
    if (clock_fault_ || samples.empty()) return -1;
    // Negative authority is actionable immediately, even before /clock catches
    // up. The caller receives its original payload and explicit reason.
    if (!samples.back().allowed) return static_cast<int>(samples.size() - 1);
    std::size_t begin = 0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
      const auto & a = samples[i - 1];
      const auto & b = samples[i];
      // Revocations, new geometry/context and shorter evidence deadlines are
      // barriers, even when the incoming heartbeat precedes /clock.
      if (b.context != a.context || !a.allowed || !b.allowed ||
        b.valid_until_ns < a.valid_until_ns || b.lease_s < a.lease_s)
      {
        begin = i;
      }
    }
    int selected = -1;
    int64_t newest = -1;
    for (std::size_t i = begin; i < samples.size(); ++i) {
      const auto & sample = samples[i];
      if (sample.stamp_ns <= 0 || sample.limits_stamp_ns <= 0 ||
        sample.stamp_ns > now || sample.limits_stamp_ns > now ||
        sample.valid_until_ns <= now || !std::isfinite(sample.lease_s) ||
        sample.lease_s <= 0 || sample.lease_s > .5) continue;
      const auto age = now - sample.limits_stamp_ns;
      if (age > static_cast<int64_t>(std::llround(sample.lease_s * 1e9))) continue;
      if (sample.stamp_ns >= newest) {
        selected = static_cast<int>(i); newest = sample.stamp_ns;
      }
    }
    return selected;
  }
private:
  int64_t last_now_{-1};
  bool clock_fault_{false};
};
}  // namespace astribot_s1_robot_geometry
