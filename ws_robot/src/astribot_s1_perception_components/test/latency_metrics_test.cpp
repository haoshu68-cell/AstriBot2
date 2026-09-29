#include <gtest/gtest.h>
#include <limits>
#include "astribot_s1_perception_components/latency_metrics.hpp"
using astribot::vision::LatencyHistogram;

TEST(LatencyMetrics, EmptyHistogramHasNoSamples) {
  const auto result=LatencyHistogram{}.summary();
  EXPECT_EQ(result.samples,0u);EXPECT_EQ(result.overflow_samples,0u);
  EXPECT_EQ(result.max_ms,0.);EXPECT_EQ(result.p99_upper_ms,0.);
}
TEST(LatencyMetrics, QuantilesBoundTheObservedDurations) {
  LatencyHistogram histogram;
  for(int n=1;n<=100;++n)histogram.add_ms(n*.1);
  const auto result=histogram.summary();
  EXPECT_EQ(result.samples,100u);EXPECT_EQ(result.overflow_samples,0u);
  EXPECT_GE(result.p50_upper_ms,5.);EXPECT_LE(result.p50_upper_ms,5.05+1.e-9);
  EXPECT_GE(result.p95_upper_ms,9.5);EXPECT_LE(result.p95_upper_ms,9.55+1.e-9);
  EXPECT_GE(result.p99_upper_ms,9.9);EXPECT_LE(result.p99_upper_ms,9.95+1.e-9);
  EXPECT_DOUBLE_EQ(result.max_ms,10.);
}
TEST(LatencyMetrics, LongOutliersAreCountedAndNeverClipped) {
  LatencyHistogram histogram;histogram.add_ms(1.);histogram.add_ms(800.);histogram.add_ms(1200.);
  const auto result=histogram.summary();EXPECT_EQ(result.samples,3u);EXPECT_EQ(result.overflow_samples,2u);
  EXPECT_DOUBLE_EQ(result.p99_upper_ms,1200.);EXPECT_DOUBLE_EQ(result.max_ms,1200.);
}
TEST(LatencyMetrics, InvalidMeasurementsCannotPoisonSummary) {
  LatencyHistogram histogram;
  EXPECT_THROW(histogram.add_ms(-1),std::invalid_argument);
  EXPECT_THROW(histogram.add_ms(std::numeric_limits<double>::infinity()),std::invalid_argument);
  EXPECT_THROW(histogram.add_ms(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  EXPECT_EQ(histogram.summary().samples,0u);
}
