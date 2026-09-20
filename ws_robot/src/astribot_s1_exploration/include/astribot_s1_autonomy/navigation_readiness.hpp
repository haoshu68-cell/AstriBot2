#pragma once
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <string>

namespace astribot_s1_autonomy {
inline std::string fixedEnvelopeReadiness(
    const astribot_navigation_msgs::msg::NavigationEnvelopeV2 * envelope,
    int64_t now_ns, size_t publishers) {
  if (publishers != 1) return "ENVELOPE_V2_SOURCE_NOT_UNIQUE";
  if (!envelope) return "ENVELOPE_V2_NOT_RECEIVED";
  const auto & e = *envelope;
  if (e.header.stamp.sec < 0 || e.valid_until.sec < 0 ||
      e.header.stamp.nanosec >= 1000000000u || e.valid_until.nanosec >= 1000000000u)
    return "ENVELOPE_V2_INVALID_TIME";
  const auto stamp = int64_t(e.header.stamp.sec) * 1000000000 + e.header.stamp.nanosec;
  const auto until = int64_t(e.valid_until.sec) * 1000000000 + e.valid_until.nanosec;
  if (stamp <= 0 || stamp > now_ns || until <= now_ns || until - stamp > 500000000)
    return "ENVELOPE_V2_EXPIRED";
  if (e.mode != e.FIXED_POSTURE || !e.navigation_allowed || !e.limits.transport_ready ||
      e.coordinator_session_id.empty() || e.hold_id.empty() || e.epoch == 0)
    return "ENVELOPE_V2_NOT_READY: " + e.reason;
  return {};
}
}
