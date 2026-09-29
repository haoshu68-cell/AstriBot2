#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "astribot_s1_navigation_policy_native/publication_lease.hpp"

using astribot::navigation::conservative_publication_deadline;
using astribot::navigation::evaluate_publication_lease;

TEST(FinalPublicationLease, ForwardingCannotExtendUpstreamAbsoluteDeadline) {
  // Real R1-shaped publication: scan captured 48.800, valid through 49.100.
  // An upstream publication at 49.050 may grant at most 50 ms, not a new 300 ms.
  const auto upstream = evaluate_publication_lease(49050000000LL, 49000000000LL,
    1, 1, .3, {{48800000000LL, 49100000000LL}}, true);
  ASSERT_TRUE(upstream.allowed);
  const auto deadline = conservative_publication_deadline(49050000000LL, upstream.lease_s);
  ASSERT_TRUE(deadline.has_value());
  ASSERT_LE(*deadline, 49100000000LL);
  auto forwarded = evaluate_publication_lease(49080000000LL, 49060000000LL,
    8, 8, .3, {{49050000000LL, *deadline}}, true);
  ASSERT_TRUE(forwarded.allowed);
  auto forwarded_deadline = conservative_publication_deadline(49080000000LL, forwarded.lease_s);
  ASSERT_TRUE(forwarded_deadline.has_value());
  EXPECT_LE(*forwarded_deadline, *deadline);
  // Repeating the same proposal cannot roll its deadline forward.
  auto later = evaluate_publication_lease(49099000000LL, 49095000000LL,
    8, 8, .3, {{49050000000LL, *deadline}}, true);
  ASSERT_TRUE(later.allowed);
  EXPECT_LT(later.lease_s, forwarded.lease_s);
  auto late_deadline = conservative_publication_deadline(49099000000LL, later.lease_s);
  ASSERT_TRUE(late_deadline.has_value());
  EXPECT_LE(*late_deadline, *deadline);
}

TEST(FinalPublicationLease, SourceExpiryDuringFinalProtectionComputationsRevokesMotion) {
  const auto deadline = conservative_publication_deadline(49050000000LL, .05);
  ASSERT_TRUE(deadline.has_value());
  for (const auto publication : std::array<std::int64_t, 3>{*deadline, 49105000000LL, 49122000000LL}) {
    const auto output = evaluate_publication_lease(publication, 49090000000LL,
      8, 8, .3, {{49050000000LL, *deadline}}, true);
    EXPECT_FALSE(output.allowed);
    EXPECT_EQ(output.lease_s, 0.0);
    EXPECT_EQ(output.reason, "SOURCE_EXPIRED");
  }
}

TEST(FinalPublicationLease, RewindAndEpochChangeDuringForwardingReject) {
  EXPECT_FALSE(evaluate_publication_lease(90, 100, 8, 8, .3, {{50, 150}}, true).allowed);
  EXPECT_FALSE(evaluate_publication_lease(110, 100, 8, 9, .3, {{50, 150}}, true).allowed);
}

TEST(FinalPublicationLease, MalformedProposalCannotBecomeAnAbsoluteDeadline) {
  for (const auto lease : {0.0, -.1, .500000001,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(conservative_publication_deadline(100, lease).has_value());
  EXPECT_FALSE(conservative_publication_deadline(-1, .1).has_value());
  EXPECT_FALSE(conservative_publication_deadline(
    std::numeric_limits<std::int64_t>::max() - 1, .1).has_value());
  EXPECT_FALSE(conservative_publication_deadline(0, std::numeric_limits<double>::denorm_min()).has_value());
}

TEST(FinalPublicationLease, IndependentScanAndOdomUsedForSweepBoundTheFinalLease) {
  // Proposal remains live through .5, but the exact scan used for the sweep
  // expires at .3 during the computation begun at .29. No new scan is used.
  for (const auto expired_sensor : {0, 1}) {
    std::vector<std::array<std::int64_t, 2>> inputs{{250000000, 500000000},
      {0, 450000000}, {0, 450000000}};
    inputs[1 + expired_sensor][1] = 300000000;
    const auto result = evaluate_publication_lease(360000000, 290000000, 8, 8, .3, inputs, true);
    EXPECT_FALSE(result.allowed); EXPECT_EQ(result.reason, "SOURCE_EXPIRED");
  }
}

TEST(FinalPublicationLease, CallerInvalidatesEvidenceUnderExistingTimeDomainRules) {
  // false is the caller freshness result; this does not add a simulated-wall TTL.
  const auto result = evaluate_publication_lease(290000000, 290000000, 8, 8, .3,
    {{250000000, 500000000}, {0, 300000000}, {0, 300000000}}, false);
  EXPECT_FALSE(result.allowed); EXPECT_EQ(result.lease_s, 0.0);
  EXPECT_EQ(result.reason, "EVIDENCE_INVALID");
}
