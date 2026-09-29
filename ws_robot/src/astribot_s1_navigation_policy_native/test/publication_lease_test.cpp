#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#if __has_include("astribot_s1_navigation_policy_native/publication_lease.hpp")
#include "astribot_s1_navigation_policy_native/publication_lease.hpp"
using astribot::navigation::evaluate_publication_lease;
using astribot::navigation::conservative_publication_deadline;
using astribot::navigation::conservative_source_deadline;
using Intervals = std::vector<std::array<std::int64_t, 2>>;

TEST(PublicationLease, PublicationAfterDecisionMustStillPrecedeEveryDeadline) {
  for (auto now : {300000000LL, 305000000LL, 322000000LL}) {
    auto result = evaluate_publication_lease(now, 250000000, 4, 4, 0.2,
      {{0, 300000000}}, true);
    EXPECT_FALSE(result.allowed); EXPECT_EQ(result.lease_s, 0.0);
    EXPECT_EQ(result.reason, "SOURCE_EXPIRED");
    EXPECT_EQ(result.deadline_ns, 300000000);
  }
}

TEST(PublicationLease, OneNanosecondRemainingIsPositiveAndConservative) {
  auto result = evaluate_publication_lease(299999999, 250000000, 4, 4, 0.2,
    {{0, 300000000}}, true);
  ASSERT_TRUE(result.allowed);
  EXPECT_GT(result.lease_s, 0);
  EXPECT_LE(static_cast<long double>(result.lease_s) * 1000000000.0L, 1.0L);
  EXPECT_EQ(result.deadline_ns, 300000000);
}

TEST(PublicationLease, MultipleSourcesUseEarliestDeadlineAndCap) {
  auto result = evaluate_publication_lease(200000000, 190000000, 0, 0, 0.3,
    {{50000000, 550000000}, {0, 300000000}, {100000000, 400000000}}, true);
  ASSERT_TRUE(result.allowed); EXPECT_EQ(result.deadline_ns, 300000000);
  EXPECT_GT(result.lease_s, 0.099999999);
  EXPECT_LE(static_cast<long double>(result.lease_s), 0.1L);
  result = evaluate_publication_lease(0, 0, 0, 0, 0.125, {{0, 1000000000}}, true);
  ASSERT_TRUE(result.allowed); EXPECT_LT(result.lease_s, 0.125);
}

TEST(PublicationLease, RepeatedEvaluationCannotRenewTheSourceDeadline) {
  const auto early = evaluate_publication_lease(100, 90, 8, 8, 0.2, {{1, 200}}, true);
  const auto late = evaluate_publication_lease(190, 90, 8, 8, 0.2, {{1, 200}}, true);
  ASSERT_TRUE(early.allowed); ASSERT_TRUE(late.allowed);
  EXPECT_EQ(early.deadline_ns, late.deadline_ns);
  EXPECT_LT(late.lease_s, early.lease_s);
}

TEST(PublicationLease, MissingOrInvalidEvidenceNeverAuthorizes) {
  EXPECT_EQ(evaluate_publication_lease(10, 9, 0, 0, 0.2, {}, true).reason, "MISSING_REQUIRED_INTERVALS");
  auto result = evaluate_publication_lease(10, 9, 0, 0, 0.2, {{1, 20}}, false);
  EXPECT_FALSE(result.allowed); EXPECT_EQ(result.reason, "EVIDENCE_INVALID");
  EXPECT_EQ(result.lease_s, 0.0);
}

TEST(PublicationLease, EpochMismatchAndClockRollbackDeny) {
  EXPECT_EQ(evaluate_publication_lease(10, 9, 1, 2, 0.2, {{1, 20}}, true).reason, "EPOCH_MISMATCH");
  EXPECT_EQ(evaluate_publication_lease(8, 9, 2, 2, 0.2, {{1, 20}}, true).reason, "CLOCK_ROLLBACK");
}

TEST(PublicationLease, FutureCaptureAndMalformedDeadlineDeny) {
  EXPECT_EQ(evaluate_publication_lease(10, 9, 0, 0, 0.2, {{11, 20}}, true).reason, "FUTURE_CAPTURE");
  for (auto deadline : {1LL, 0LL}) {
    EXPECT_EQ(evaluate_publication_lease(10, 9, 0, 0, 0.2, {{1, deadline}}, true).reason, "INVALID_INTERVAL");
  }
  EXPECT_FALSE(evaluate_publication_lease(10, 9, 0, 0, 0.2, {{1, 20}, {11, 30}}, true).allowed);
}

TEST(PublicationLease, CapMustBeFinitePositiveAndAtMostHalfASecond) {
  for (double cap : {0.0, -0.1, 0.500000000001,
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    const auto result = evaluate_publication_lease(10, 9, 0, 0, cap, {{1, 20}}, true);
    EXPECT_FALSE(result.allowed); EXPECT_EQ(result.reason, "INVALID_LEASE_CAP");
  }
  EXPECT_TRUE(evaluate_publication_lease(10, 9, 0, 0, 0.5, {{1, 1000000000}}, true).allowed);
  const auto tiny = evaluate_publication_lease(10, 9, 0, 0,
    std::numeric_limits<double>::denorm_min(), {{1, 20}}, true);
  EXPECT_FALSE(tiny.allowed); EXPECT_EQ(tiny.lease_s, 0.0);
}

TEST(PublicationLease, FullNonnegativeInt64RangeHasNoOverflow) {
  const auto hi = std::numeric_limits<std::int64_t>::max();
  auto result = evaluate_publication_lease(hi - 1, hi - 2, UINT64_MAX, UINT64_MAX,
    0.5, {{hi - 3, hi}}, true);
  ASSERT_TRUE(result.allowed); EXPECT_EQ(result.deadline_ns, hi);
  EXPECT_LE(static_cast<long double>(result.lease_s) * 1000000000.0L, 1.0L);
  result = evaluate_publication_lease(0, 0, 0, 0, 0.5, {{0, hi}}, true);
  ASSERT_TRUE(result.allowed); EXPECT_LT(result.lease_s, 0.5);
  EXPECT_FALSE(evaluate_publication_lease(hi, hi, 0, 0, 0.5, {{0, hi}}, true).allowed);
}

TEST(PublicationLease, NegativeAndMinimumInt64TimestampsDenyBeforeSubtraction) {
  for (const auto negative : {-1LL, std::numeric_limits<long long>::min()}) {
    EXPECT_FALSE(evaluate_publication_lease(negative, 0, 0, 0, 0.2, {{0, 20}}, true).allowed);
    EXPECT_FALSE(evaluate_publication_lease(10, negative, 0, 0, 0.2, {{0, 20}}, true).allowed);
    EXPECT_FALSE(evaluate_publication_lease(10, 0, 0, 0, 0.2, {{negative, 20}}, true).allowed);
    EXPECT_FALSE(evaluate_publication_lease(10, 0, 0, 0, 0.2, {{0, negative}}, true).allowed);
  }
}

TEST(PublicationLease, RoundingNeverExtendsNanosecondRemainingTime) {
  for (auto remaining : {1LL, 3LL, 7LL, 999LL, 99999999LL, 100000000LL,
    299999999LL, 499999999LL, 500000000LL})
  {
    auto result = evaluate_publication_lease(0, 0, 0, 0, 0.5, {{0, remaining}}, true);
    ASSERT_TRUE(result.allowed);
    EXPECT_LE(static_cast<long double>(result.lease_s) * 1000000000.0L,
      static_cast<long double>(remaining));
  }
}

TEST(PublicationLease, UpstreamFloatLeaseNeverRoundsItsDeadlineOutward) {
  for (double lease : {0.1, 0.2, 0.3, 0.5, 0.000000002, 0.123456789}) {
    const auto deadline = conservative_publication_deadline(100, lease);
    ASSERT_TRUE(deadline);
    EXPECT_LE(static_cast<long double>(*deadline - 100),
      static_cast<long double>(lease) * 1000000000.0L);
    EXPECT_GT(*deadline, 100);
  }
}

TEST(PublicationLease, UpstreamLeaseRejectsInvalidAndOverflowRatherThanSaturating) {
  const auto hi = std::numeric_limits<std::int64_t>::max();
  for (double lease : {0.0, -0.1, 0.50001,
    std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_FALSE(conservative_publication_deadline(0, lease));
  }
  EXPECT_FALSE(conservative_publication_deadline(-1, 0.1));
  EXPECT_FALSE(conservative_publication_deadline(hi, 0.1));
  EXPECT_FALSE(conservative_publication_deadline(hi - 1, 0.1));
  EXPECT_TRUE(conservative_publication_deadline(hi - 1000000000, 0.5));
}

TEST(PublicationLease, SourceTimeoutKeepsConfiguredValuesAbovePublicationCap) {
  const auto deadline = conservative_source_deadline(100, 10.0);
  ASSERT_TRUE(deadline);
  EXPECT_EQ(*deadline, 100 + 9999999999LL);
  EXPECT_FALSE(conservative_publication_deadline(100, 10.0));
  EXPECT_FALSE(conservative_source_deadline(0, std::numeric_limits<double>::max()));
  EXPECT_FALSE(conservative_source_deadline(0, 0x1p63 / 1000000000.0));
  EXPECT_FALSE(conservative_source_deadline(std::numeric_limits<std::int64_t>::max(), 10.0));
}
#else
TEST(PublicationLease, RequiredNativeImplementationExists) {
  FAIL() << "publication_lease.hpp is not implemented";
}
#endif
