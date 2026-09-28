#include <gtest/gtest.h>
#include "astribot_s1_transport_native/arm_hold.hpp"
#include <cmath>
#include <limits>
using namespace astribot::transport;
namespace {
constexpr int64_t T=1000000000,MS=1000000;
builtin_interfaces::msg::Time stamp(int64_t n) {builtin_interfaces::msg::Time t;t.sec=n/1000000000;t.nanosec=n%1000000000;return t;}
ResourceGrant grant() {return {"task","lease","resource_boot",{"left","right"},T,T+2000*MS,T,T};}
HoldCompletion complete() {return {"task","lease","resource_boot","posture_action",T,true,true};}
Geometry geometry(int64_t at,uint64_t seq,double left=0.) {
  Geometry g;g.header.stamp=stamp(at);g.header.frame_id="base";g.valid_until=stamp(at+300*MS);
  g.source_id="geometry_boot";g.sequence=seq;g.model_revision="model";g.attachment_revision="payload_epoch_1";
  g.complete=g.attachment_state_confirmed=true;g.joints.name={"left","right"};g.joints.position={left,0.};
  g.joint_position_error_bounds={.004,.004};g.joint_source_stamps={g.header.stamp,g.header.stamp};return g;
}
std::vector<controller_manager_msgs::msg::ControllerState> controllers() {
  controller_manager_msgs::msg::ControllerState c;c.name="arms";c.state="active";c.type="astribot_s1_manipulation/OwnedTrajectoryController";
  c.claimed_interfaces={"left/position","right/position"};return {c};
}
class Fixture:public ::testing::Test {
protected:
  ArmHold h;
  void ready() {
    h.begin("hold",grant(),complete(),T,T);
    for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.controllers(controllers(),at,at,at,at);h.geometry(geometry(at,i+1),at,at);}
    ASSERT_TRUE(h.status(T+600*MS,T+600*MS).hold_confirmed);
  }
};
TEST_F(Fixture, ZeroMotionWithoutTaskAuthorityNeverConfirms) {
  h.controllers(controllers(),T,T,T,T);h.geometry(geometry(T,1),T,T);EXPECT_FALSE(h.status(T,T).hold_confirmed);
}
TEST_F(Fixture, MissingOrUnfinishedExecutorProofRejected) {
  auto proof=complete();proof.terminal=false;EXPECT_THROW(h.begin("hold",grant(),proof,T,T),std::invalid_argument);
  proof=complete();proof.owner_id="other";EXPECT_THROW(h.begin("hold",grant(),proof,T,T),std::invalid_argument);
  auto resource=grant();resource.lease_id="";EXPECT_THROW(h.begin("hold",resource,complete(),T,T),std::invalid_argument);
}
TEST_F(Fixture, SustainedMeasuredSettlingAndClaimsProduceBoundedHold) {
  ready();auto status=h.status(T+600*MS,T+600*MS);EXPECT_EQ(status.owner_id,"task");EXPECT_EQ(status.hold_id,"hold");
  EXPECT_EQ(status.attachment_revision,"payload_epoch_1");EXPECT_GT(status.lease_s,0.);EXPECT_LE(status.lease_s,.3);
}
TEST_F(Fixture, SingleFrameOrDuplicateDoesNotProveSettling) {
  h.begin("hold",grant(),complete(),T,T);auto g=geometry(T+100*MS,1);
  h.controllers(controllers(),T+100*MS,T+100*MS,T+100*MS,T+100*MS);
  for(int i=0;i<10;++i)h.geometry(g,T+100*MS,T+100*MS);
  EXPECT_FALSE(h.status(T+100*MS,T+100*MS).hold_confirmed);
}
TEST_F(Fixture, SettlingBudgetIsQuarterOfHoldError) {
  h.begin("hold",grant(),complete(),T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.controllers(controllers(),at,at,at,at);h.geometry(geometry(at,i+1,i==1?.0011:0.),at,at);}
  EXPECT_FALSE(h.status(T+600*MS,T+600*MS).hold_confirmed);
}
TEST_F(Fixture, ClaimLossRevokesAndCannotAutomaticallyRecover) {
  ready();auto c=controllers();c[0].claimed_interfaces={"left/position"};h.controllers(c,T+610*MS,T+610*MS,T+610*MS,T+610*MS);
  EXPECT_FALSE(h.status(T+610*MS,T+610*MS).hold_confirmed);
  h.controllers(controllers(),T+620*MS,T+620*MS,T+620*MS,T+620*MS);h.geometry(geometry(T+620*MS,4),T+620*MS,T+620*MS);
  EXPECT_FALSE(h.status(T+620*MS,T+620*MS).hold_confirmed);
}
TEST_F(Fixture, DuplicateControllerClaimsAreRejected) {
  ready();auto c=controllers();c.push_back(c.front());h.controllers(c,T+610*MS,T+610*MS,T+610*MS,T+610*MS);
  EXPECT_FALSE(h.status(T+610*MS,T+610*MS).hold_confirmed);
}
TEST_F(Fixture, UnownedJointCannotReceiveHoldAuthorization) {
  auto resource=grant();resource.joints={"left"};h.begin("hold",resource,complete(),T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.controllers(controllers(),at,at,at,at);h.geometry(geometry(at,i+1),at,at);}
  EXPECT_FALSE(h.status(T+600*MS,T+600*MS).hold_confirmed);
}
TEST_F(Fixture, AttachmentChangeAndReturnCannotReviveOldHold) {
  ready();auto g=geometry(T+610*MS,4);g.attachment_revision="payload_epoch_2";h.geometry(g,T+610*MS,T+610*MS);
  EXPECT_FALSE(h.status(T+610*MS,T+610*MS).hold_confirmed);
  h.geometry(geometry(T+620*MS,5),T+620*MS,T+620*MS);EXPECT_FALSE(h.status(T+620*MS,T+620*MS).hold_confirmed);
}
TEST_F(Fixture, DriftOutsideOriginalReservationRevokes) {
  ready();h.geometry(geometry(T+610*MS,4,.004001),T+610*MS,T+610*MS);EXPECT_FALSE(h.status(T+610*MS,T+610*MS).hold_confirmed);
}
TEST_F(Fixture, CancelClearsRequestAndHoldCannotReuseId) {
  ready();h.cancel();EXPECT_FALSE(h.status(T+610*MS,T+610*MS).hold_confirmed);
  EXPECT_THROW(h.request("r",astribot_navigation_msgs::msg::RobotEnvelope(),T+610*MS,T+610*MS),std::invalid_argument);
  EXPECT_THROW(h.begin("hold",grant(),complete(),T+610*MS,T+610*MS),std::invalid_argument);
}
TEST_F(Fixture, SourceAndFrozenClockWallExpiryRevoke) {
  ready();EXPECT_FALSE(h.status(T+900*MS,T+900*MS).hold_confirmed);
  ArmHold second;second.begin("fresh",grant(),complete(),T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;second.controllers(controllers(),at,at,at,at);second.geometry(geometry(at,i+1),at,at);}
  EXPECT_FALSE(second.status(T+600*MS,T+900*MS).hold_confirmed);
}
TEST_F(Fixture, FutureSameVersionDoesNotDisplaceCurrentEvidence) {
  ready();h.geometry(geometry(T+700*MS,4),T+600*MS,T+601*MS);EXPECT_TRUE(h.status(T+600*MS,T+601*MS).hold_confirmed);
}
TEST_F(Fixture, MalformedJointArraysAndNanCannotConfirm) {
  h.begin("hold",grant(),complete(),T,T);auto g=geometry(T+100*MS,1);g.joints.position[0]=NAN;
  EXPECT_FALSE(h.geometry(g,T+100*MS,T+100*MS));EXPECT_FALSE(h.status(T+100*MS,T+100*MS).hold_confirmed);
}
TEST_F(Fixture, RequestReferencesCurrentGeometryAndSameHold) {
  ready();astribot_navigation_msgs::msg::RobotEnvelope limits;limits.posture_id="nonhome";limits.frame_id="base";
  auto request=h.request("request",limits,T+600*MS,T+600*MS);
  EXPECT_EQ(request.request_id,"request");EXPECT_EQ(request.hold_id,"hold");EXPECT_EQ(request.geometry_sequence,3u);
  EXPECT_EQ(request.limits.posture_id,"nonhome");
}

TEST_F(Fixture, FreshResourceRenewalMaintainsOwnershipButNewOwnerRevokes) {
  ready();auto renewed=grant();renewed.issued_at=T+600*MS;renewed.received_at=renewed.issued_at;
  renewed.received_steady=renewed.issued_at;renewed.valid_until=T+2600*MS;
  EXPECT_TRUE(h.resource(renewed,T+600*MS,T+600*MS));
  renewed.owner_id="other";EXPECT_FALSE(h.resource(renewed,T+610*MS,T+610*MS));
  EXPECT_FALSE(h.status(T+610*MS,T+610*MS).hold_confirmed);
}
TEST_F(Fixture, ExpiredHoldCannotReviveWhenTimerDidNotObserveGap) {
  ready();h.controllers(controllers(),T+1000*MS,T+1000*MS,T+1000*MS,T+1000*MS);
  h.geometry(geometry(T+1000*MS,4),T+1000*MS,T+1000*MS);
  EXPECT_FALSE(h.status(T+1000*MS,T+1000*MS).hold_confirmed);
}
TEST_F(Fixture, SameCaptureTightenedLeaseCannotBeIgnored) {
  ready();auto g=geometry(T+600*MS,4);g.valid_until=stamp(T+650*MS);
  h.geometry(g,T+610*MS,T+610*MS);
  EXPECT_LE(h.status(T+620*MS,T+620*MS).lease_s,.030000001);
  EXPECT_FALSE(h.status(T+650*MS,T+650*MS).hold_confirmed);
}
TEST_F(Fixture, ResourceQueueDelayDoesNotRenewWallLease) {
  auto resource=grant();resource.issued_at=0;resource.received_at=T;resource.valid_until=T+1000*MS;resource.received_steady=T;
  h.begin("hold",resource,complete(),T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.controllers(controllers(),at,at,at,at);h.geometry(geometry(at,i+1),at,at);}
  for(int i=0;i<3;++i) {auto wall=T+(750+125*i)*MS;auto ros=T+(650+50*i)*MS;
    h.controllers(controllers(),ros,wall,ros,wall);h.geometry(geometry(ros,4+i),ros,wall);}
  EXPECT_FALSE(h.status(T+750*MS,T+1000*MS).hold_confirmed);
}

TEST_F(Fixture, OneSequenceCannotRepresentThreeDifferentCaptures) {
  h.begin("hold",grant(),complete(),T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.controllers(controllers(),at,at,at,at);h.geometry(geometry(at,1),at,at);}
  EXPECT_FALSE(h.status(T+600*MS,T+600*MS).hold_confirmed);
}
TEST_F(Fixture, ResourceGapDuringSettlingCannotBeHiddenByRenewal) {
  auto resource=grant();resource.valid_until=T+400*MS;h.begin("hold",resource,complete(),T,T);
  for(int i=0;i<2;++i) {auto at=T+(100+250*i)*MS;h.controllers(controllers(),at,at,at,at);h.geometry(geometry(at,i+1),at,at);}
  resource.issued_at=resource.received_at=resource.received_steady=T+450*MS;resource.valid_until=T+2000*MS;
  EXPECT_FALSE(h.resource(resource,T+450*MS,T+450*MS));
  h.controllers(controllers(),T+600*MS,T+600*MS,T+600*MS,T+600*MS);h.geometry(geometry(T+600*MS,3),T+600*MS,T+600*MS);
  EXPECT_FALSE(h.status(T+600*MS,T+600*MS).hold_confirmed);
}
TEST_F(Fixture, FirstControllerResponseAfterSettlingCanCompleteInitialization) {
  h.begin("hold",grant(),complete(),T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.geometry(geometry(at,i+1),at,at);}
  EXPECT_FALSE(h.status(T+600*MS,T+600*MS).hold_confirmed);
  h.controllers(controllers(),T+600*MS,T+600*MS,T+600*MS,T+600*MS);
  EXPECT_TRUE(h.status(T+600*MS,T+600*MS).hold_confirmed);
}
TEST_F(Fixture, ZeroSequenceRevokesInvalidCapture) {
  h.begin("hold",grant(),complete(),T,T);
  EXPECT_FALSE(h.geometry(geometry(T+100*MS,0),T+100*MS,T+100*MS));
  EXPECT_EQ(h.reason(),"GEOMETRY_SEQUENCE_INVALID");
}
TEST_F(Fixture, SameSequenceCannotChangeAttachmentIdentity) {
  h.begin("hold",grant(),complete(),T,T);const auto at=T+100*MS;
  ASSERT_TRUE(h.geometry(geometry(at,1),at,at));auto changed=geometry(at,1);changed.attachment_revision="changed";
  EXPECT_FALSE(h.geometry(changed,at,at));EXPECT_EQ(h.reason(),"HOLD_CONFLICTING_SEQUENCE");
}
TEST_F(Fixture, ClaimsMayRefreshBeforeFirstConfirmationButNotAcrossConfirmedGap) {
  h.begin("hold",grant(),complete(),T,T);h.controllers(controllers(),T,T,T,T);
  for(int i=0;i<3;++i) {auto at=T+(100+250*i)*MS;h.geometry(geometry(at,i+1),at,at);}
  EXPECT_FALSE(h.status(T+600*MS,T+600*MS).hold_confirmed);
  h.controllers(controllers(),T+600*MS,T+600*MS,T+600*MS,T+600*MS);
  EXPECT_TRUE(h.status(T+600*MS,T+600*MS).hold_confirmed);
}
TEST_F(Fixture, CoordinatorObservationAcceptsFirstAndLaterFreshPositive) {
  ready();const auto first=T+600*MS;
  auto observed=h.status(first,first);const ArmHold &hold=h;
  EXPECT_TRUE(hold.observationMatches(observed,first,first,first,first,first));
  observed=h.status(first+40*MS,first+40*MS);
  EXPECT_TRUE(hold.observationMatches(observed,first,first+50*MS,first+50*MS,first+60*MS,first+60*MS));
  EXPECT_TRUE(h.status(first+60*MS,first+60*MS).hold_confirmed);
}
TEST_F(Fixture, CoordinatorObservationRejectsFalseOldAndMismatchedHold) {
  ready();const auto first=T+600*MS;const auto valid=h.status(first,first);
  auto changed=valid;changed.hold_confirmed=false;
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  changed=valid;changed.owner_id="other";
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  changed=valid;changed.hold_id="previous";
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  changed=valid;changed.attachment_revision="previous_payload";
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  changed=valid;changed.header.stamp=stamp(first-1);
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  changed=valid;changed.header.stamp=stamp(first+1);
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  changed=valid;changed.header.stamp.nanosec=1000000000;
  EXPECT_FALSE(h.observationMatches(changed,first,first,first,first,first));
  h.cancel();EXPECT_FALSE(h.observationMatches(valid,first,first,first,first,first));
}
TEST_F(Fixture, CoordinatorObservationEnforcesSourceAndSteadyLeaseBoundaries) {
  ready();const auto first=T+600*MS;auto observed=h.status(first,first);observed.lease_s=.1;
  const auto received=first+20*MS,expires=first+100*MS;
  EXPECT_TRUE(h.observationMatches(observed,first,received,received,expires-1,expires-1));
  EXPECT_FALSE(h.observationMatches(observed,first,received,received,expires,expires-1));
  // A frozen ROS clock cannot extend the source's remaining lease at receipt.
  EXPECT_TRUE(h.observationMatches(observed,first,received,received,received,expires-1));
  EXPECT_FALSE(h.observationMatches(observed,first,received,received,received,expires));
  EXPECT_FALSE(h.observationMatches(observed,first,received,received,received-1,received));
  EXPECT_FALSE(h.observationMatches(observed,first,received,received,received,received-1));
  for(const auto lease:{0.,-.1,.500000001,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    observed.lease_s=lease;
    EXPECT_FALSE(h.observationMatches(observed,first,received,received,received,received));
  }
}
}
