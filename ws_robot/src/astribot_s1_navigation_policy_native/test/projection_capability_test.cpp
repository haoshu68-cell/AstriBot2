#include <gtest/gtest.h>
#include "astribot_s1_navigation_policy_native/projection_capability.hpp"
using namespace astribot::navigation;
namespace {
CameraCapabilitySample sample(int64_t at,std::string camera="head",std::string epoch="one") {
  return {camera,camera+"_optical",epoch,true,at,at,at+250000000};
}
void pair(ProjectionCapabilityGate& gate,int64_t at,double wall,std::string camera="head",std::string epoch="one") {
  auto h=sample(at,camera,epoch);gate.raw(h,at,wall);gate.processing(h,at,wall);
}
}
TEST(ProjectionCapability, UnconfiguredPathKeepsOriginalAdmission) {
  ProjectionCapabilityGate gate({});EXPECT_TRUE(gate.evaluate(1000000000,1.).ready);
}
TEST(ProjectionCapability, RequiresRawAndProcessingForEveryConfiguredCamera) {
  ProjectionCapabilityGate gate({"head","torso"});auto h=sample(1000000000);
  gate.raw(h,1000000000,1.);EXPECT_FALSE(gate.evaluate(1000000000,1.).ready);
  gate.processing(h,1000000000,1.);EXPECT_FALSE(gate.evaluate(1000000000,1.).ready);
  pair(gate,1000000001,1.,"torso");EXPECT_TRUE(gate.evaluate(1000000001,1.).ready);
}
TEST(ProjectionCapability, LatestCaptureSurvivesSourceAndReceiptAge) {
  ProjectionCapabilityGate gate({"head"});pair(gate,1000000000,1.);
  EXPECT_TRUE(gate.evaluate(1249999999,1.249999999).ready);
  EXPECT_TRUE(gate.evaluate(1250000000,1.25).ready);
  pair(gate,1300000000,1.3);ASSERT_TRUE(gate.evaluate(1300000000,1.3).ready);
  EXPECT_TRUE(gate.evaluate(1300000000,1.56).ready);
}
TEST(ProjectionCapability, RepeatedHeartbeatRetainsLatestCapture) {
  ProjectionCapabilityGate gate({"head"});pair(gate,1000000000,1.);ASSERT_TRUE(gate.evaluate(1000000000,1.).ready);
  auto h=sample(1000000000);gate.raw(h,1000000000,1.2);gate.processing(h,1000000000,1.2);
  EXPECT_TRUE(gate.evaluate(1000000000,1.26).ready);
}
TEST(ProjectionCapability, ShortFaultCannotDisappearBeforeEvaluation) {
  ProjectionCapabilityGate gate({"head"});pair(gate,1000000000,1.);auto before=gate.evaluate(1000000000,1.);ASSERT_TRUE(before.ready);
  auto bad=sample(1050000000);bad.valid=false;gate.processing(bad,1050000000,1.05);
  auto old=sample(1040000000);old.header_ns=1050000001;gate.processing(old,1050000001,1.051);
  EXPECT_FALSE(gate.evaluate(1050000001,1.051).ready);
  pair(gate,1100000000,1.1);auto recovered=gate.evaluate(1100000000,1.1);
  EXPECT_TRUE(recovered.ready);EXPECT_GT(recovered.revision,before.revision);EXPECT_EQ(recovered.released_ns,1100000000);
}
TEST(ProjectionCapability, FrameAndEpochChangesUpdateContext) {
  ProjectionCapabilityGate gate({"head"});pair(gate,1000000000,1.);ASSERT_TRUE(gate.evaluate(1000000000,1.).ready);
  auto p=sample(1050000000);p.frame="wrong";gate.processing(p,1050000000,1.05);
  EXPECT_FALSE(gate.evaluate(1050000000,1.05).ready);
  pair(gate,1100000000,1.1);ASSERT_TRUE(gate.evaluate(1100000000,1.1).ready);
  p=sample(1150000000,"head","new");gate.processing(p,1150000000,1.15);
  EXPECT_TRUE(gate.evaluate(1150000000,1.15).ready);
  auto raw=sample(1200000000);gate.raw(raw,1200000000,1.2);
  p.capture_ns=p.header_ns=1200000000;p.until_ns=1450000000;gate.processing(p,1200000000,1.2);
  EXPECT_TRUE(gate.evaluate(1200000000,1.2).ready);
}
TEST(ProjectionCapability, LocalRollbackDoesNotDiscardLatestHistory) {
  for(bool raw_first:{false,true}) {
    ProjectionCapabilityGate gate({"head"});pair(gate,100000000000,1.);ASSERT_TRUE(gate.evaluate(100000000000,1.).ready);
    auto h=sample(50000000000);
    if(raw_first)gate.raw(h,50000000000,1.1);else gate.processing(h,50000000000,1.1);
    EXPECT_TRUE(gate.evaluate(50000000000,1.1).ready);
    pair(gate,50100000000,1.2);EXPECT_TRUE(gate.evaluate(50100000000,1.2).ready);
  }
}
TEST(ProjectionCapability, InvalidAndUnboundedSourceConfigurationRejected) {
  EXPECT_THROW(ProjectionCapabilityGate({"head","head"}),std::invalid_argument);
  EXPECT_THROW(ProjectionCapabilityGate({"../head"}),std::invalid_argument);
  EXPECT_THROW(ProjectionCapabilityGate({""}),std::invalid_argument);
}
TEST(ProjectionCapability, FutureHeartbeatIsAcceptedAsLatest) {
  for(bool raw_source:{false,true}) {
    ProjectionCapabilityGate gate({"head"});pair(gate,1000000000,1.);
    auto before=gate.evaluate(1000000000,1.);ASSERT_TRUE(before.ready);
    auto future=sample(1050000000);future.header_ns=1061000000;
    if(raw_source)gate.raw(future,1060000000,1.06);else gate.processing(future,1060000000,1.06);
    auto during=gate.evaluate(1060000000,1.06);
    EXPECT_TRUE(during.ready);EXPECT_EQ(during.revision,before.revision);
    EXPECT_EQ(during.future_samples_rejected,0u);
    // A same-version message ahead of local /clock is discarded, never
    // admitted or used to refresh either the capture or its wall-clock lease.
    EXPECT_TRUE(gate.evaluate(1250000000,1.25).ready);
    pair(gate,1300000000,1.3);EXPECT_TRUE(gate.evaluate(1300000000,1.3).ready);
  }
}
TEST(ProjectionCapability, FutureRevocationAndContextChangeStillRevokeImmediately) {
  for(bool change_context:{false,true}) {
    ProjectionCapabilityGate gate({"head"});pair(gate,1000000000,1.);ASSERT_TRUE(gate.evaluate(1000000000,1.).ready);
    auto future=sample(1050000000);future.header_ns=1061000000;
    if(change_context)future.epoch="replacement";else future.valid=false;
    gate.processing(future,1060000000,1.06);
    EXPECT_EQ(gate.evaluate(1060000000,1.06).ready,change_context);
  }
}
