#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace astribot::navigation
{
struct PublicationLease
{
  bool allowed{false};
  double lease_s{0.0};
  std::int64_t deadline_ns{0};
  std::string reason;
};

// Recover a deadline from an upstream floating lease without extending it.
// Multiplication can round upward onto an integer, so shrink one ULP before
// floor. This may discard one additional nanosecond even for an exact product.
// A sub-nanosecond lease has no representable positive integer-ns deadline.
inline std::optional<std::int64_t> conservative_source_deadline(
  std::int64_t stamp_ns, double duration_s)
{
  if (stamp_ns < 0 || !std::isfinite(duration_s) || duration_s <= 0.0) {
    return std::nullopt;
  }
  const double scaled_ns = duration_s * 1000000000.0;
  // 2^63 is exactly representable, whereas double(INT64_MAX) rounds up to it.
  // Reject outside the declared signed-nanosecond range before integer casts.
  if (!std::isfinite(scaled_ns) || scaled_ns >= 0x1p63) {return std::nullopt;}
  const double ns = std::floor(std::nextafter(scaled_ns, 0.0));
  if (ns < 1.0) {return std::nullopt;}
  // Validated product is finite, positive, and strictly less than 2^63.
  const auto lease_ns = static_cast<std::int64_t>(ns);
  if (stamp_ns > std::numeric_limits<std::int64_t>::max() - lease_ns) {
    return std::nullopt;
  }
  return stamp_ns + lease_ns;
}

// MotionConstraint publication leases additionally have the protocol's 0.5 s
// cap; arbitrary source freshness thresholds retain their configured duration.
inline std::optional<std::int64_t> conservative_publication_deadline(
  std::int64_t stamp_ns, double lease_s)
{
  if (lease_s > 0.5) {return std::nullopt;}
  return conservative_source_deadline(stamp_ns, lease_s);
}

// Pure publication-time admission. All times are exact ROS nanoseconds in
// [0, INT64_MAX]; epochs span [0, UINT64_MAX]. Intervals contain capture/deadline
// from the SAME evidence used for the decision, never newly refreshed evidence.
// The caller supplies the actual publication-time now and performs no blocking
// work between this check and publication. This function neither renews sources
// nor verifies their identities: evidence_valid represents that upstream proof.
// Cost is O(required_intervals.size()), with no clocks, ROS calls, or waiting.
inline PublicationLease evaluate_publication_lease(
  std::int64_t now_ns, std::int64_t decision_ns,
  std::uint64_t decision_epoch, std::uint64_t current_epoch,
  double lease_cap_s, const std::vector<std::array<std::int64_t, 2>> & required_intervals,
  bool evidence_valid)
{
  auto deny = [](const char * reason, std::int64_t deadline = 0) {
      return PublicationLease{false, 0.0, deadline, reason};
    };
  if (!evidence_valid) {return deny("EVIDENCE_INVALID");}
  if (required_intervals.empty()) {return deny("MISSING_REQUIRED_INTERVALS");}
  if (!std::isfinite(lease_cap_s) || lease_cap_s <= 0.0 || lease_cap_s > 0.5) {
    return deny("INVALID_LEASE_CAP");
  }
  if (decision_epoch != current_epoch) {return deny("EPOCH_MISMATCH");}
  if (now_ns < 0 || decision_ns < 0) {return deny("NEGATIVE_TIMESTAMP");}
  if (now_ns < decision_ns) {return deny("CLOCK_ROLLBACK");}
  auto deadline_ns = std::numeric_limits<std::int64_t>::max();
  for (const auto & interval : required_intervals) {
    const auto capture_ns = interval[0];
    const auto source_deadline_ns = interval[1];
    if (capture_ns < 0 || source_deadline_ns < 0) {return deny("NEGATIVE_TIMESTAMP");}
    if (source_deadline_ns <= capture_ns) {return deny("INVALID_INTERVAL");}
    if (capture_ns > now_ns) {return deny("FUTURE_CAPTURE");}
    deadline_ns = std::min(deadline_ns, source_deadline_ns);
  }
  if (now_ns >= deadline_ns) {return deny("SOURCE_EXPIRED", deadline_ns);}
  // Both operands are nonnegative and deadline > now: subtraction cannot
  // overflow signed int64, including the full [0, INT64_MAX] input range.
  const auto remaining_ns = deadline_ns - now_ns;
  // For the only branch that divides, remaining_ns <= 500,000,000, so integer
  // conversion is exact in binary64. One nextafter toward zero then removes
  // any upward rounding from division, and conservatively shrinks a cap too.
  const double remaining_s = remaining_ns > 500000000 ? 0.5 :
    static_cast<double>(remaining_ns) / 1000000000.0;
  const double lease_s = std::nextafter(std::min(lease_cap_s, remaining_s), 0.0);
  if (!(lease_s > 0.0)) {return deny("LEASE_NOT_REPRESENTABLE", deadline_ns);}
  return {true, lease_s, deadline_ns, "OK"};
}
}  // namespace astribot::navigation
