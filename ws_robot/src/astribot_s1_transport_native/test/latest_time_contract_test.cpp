#include <gtest/gtest.h>
#include "astribot_s1_transport_native/arm_hold.hpp"
#include "astribot_s1_transport_native/resource_authority.hpp"
#include "astribot_s1_transport_native/payload_command.hpp"
#include "astribot_s1_transport_native/payload_frames.hpp"
#include "astribot_s1_transport_native/slam_pose_stop_window.hpp"
using namespace astribot::transport;
namespace {
constexpr int64_t S=1000000000,MS=1000000;
builtin_interfaces::msg::Time stamp(int64_t n){builtin_interfaces::msg::Time t;t.sec=n/S;t.nanosec=n%S;return t;}
Geometry geometry(int64_t at,uint64_t sequence){
 Geometry g;g.header.stamp=stamp(at);g.header.frame_id="base";g.valid_until=stamp(1);
 g.source_id="source";g.sequence=sequence;g.model_revision="model";g.attachment_revision="payload";
 g.complete=g.attachment_state_confirmed=true;g.joints.name={"left"};g.joints.position={0.};
 g.joint_position_error_bounds={.004};g.joint_source_stamps={g.header.stamp};return g;
}
std::vector<controller_manager_msgs::msg::ControllerState> claims(){
 controller_manager_msgs::msg::ControllerState c;c.state="active";c.type="astribot_s1_manipulation/OwnedTrajectoryController";c.claimed_interfaces={"left/position"};return {c};
}
class LatestHold:public ::testing::Test {
protected:
 ArmHold hold;
 void settle(){
  hold.begin("hold",{"task","lease","epoch",{"left"},S,3*S,S,S},{"task","lease","epoch","child",S,true,true},S,S);
  for(int i=0;i<3;++i){const auto steady=S+(100+250*i)*MS;
   hold.controllers(claims(),S,steady,S/2,steady);
   ASSERT_TRUE(hold.geometry(geometry(20*S+i,1+i),S/2,steady));}
  ASSERT_TRUE(hold.status(S/2,S+600*MS).hold_confirmed);
 }
};
TEST_F(LatestHold, LeadingAndFormerlyExpiredObservationUsesSteadySettling){settle();}
TEST_F(LatestHold, CurrentGeometryDoesNotExpireAndClaimsWatchdogRemains){
 settle();hold.controllers(claims(),S,S+1000*MS,100*S,S+1000*MS);
 EXPECT_TRUE(hold.status(100*S,S+1000*MS).hold_confirmed);
 EXPECT_FALSE(hold.status(100*S,S+1500*MS).hold_confirmed);
 EXPECT_EQ(hold.reason(),"HOLD_CONTROLLER_STATE_EXPIRED");
}
TEST_F(LatestHold, OldGeometryCannotOverrideLatestButExplicitInvalidRevokes){
 settle();auto old=geometry(20*S,1);old.joints.position={.5};
 EXPECT_FALSE(hold.geometry(old,S/2,S+610*MS));EXPECT_TRUE(hold.status(S/2,S+610*MS).hold_confirmed);
 old.sequence=4;
 EXPECT_FALSE(hold.geometry(old,S/2,S+615*MS));EXPECT_TRUE(hold.status(S/2,S+615*MS).hold_confirmed);
 auto bad=geometry(20*S+3,4);bad.complete=false;
 EXPECT_FALSE(hold.geometry(bad,S/2,S+620*MS));EXPECT_FALSE(hold.status(S/2,S+620*MS).hold_confirmed);
}
TEST_F(LatestHold, CancelStillRevokes){settle();hold.cancel();EXPECT_FALSE(hold.status(S/2,S+610*MS).hold_confirmed);}
TEST(LatestResource, RosRollbackAndJumpCannotEndLeaseButSteadyExpiryDoes){
 ResourceAuthority a("epoch",{"left"},[](const auto&){});ASSERT_TRUE(a.acquire("task","request",S,S).accepted);
 EXPECT_TRUE(a.grant(S/2,S+MS));EXPECT_TRUE(a.grant(100*S,S+2*MS));
 EXPECT_FALSE(a.grant(S/2,3*S));EXPECT_EQ(a.reason(),"RESOURCE_LEASE_EXPIRED");
}
TEST(LatestResource, RenewalAndTerminalStopRetainAuthorityBoundaries){
 ResourceAuthority a("epoch",{"left"},[](const auto&){});ASSERT_TRUE(a.acquire("task","request",S,S).accepted);
 const auto g=*a.grant(S,S);a.submitted(S,S);
 ASSERT_TRUE(a.renew(g.lease_id,g.epoch,1,S/2,S+MS));EXPECT_FALSE(a.renew(g.lease_id,g.epoch,1,S/2,S+2*MS));
 a.stop("CANCELED",S/2,S+3*MS);EXPECT_FALSE(a.grant(S/2,S+3*MS));
 EXPECT_FALSE(a.release(false,true,S/2,S+4*MS));EXPECT_FALSE(a.release(true,false,S/2,S+5*MS));
 EXPECT_TRUE(a.release(true,true,S/2,S+6*MS));
}
nlohmann::json inventory(bool applied=false){
 return {{"stamp_ns",20*S+(applied?1:0)},{"source_epoch","source"},{"clock_epoch",2},{"revision",applied?11:10},
 {"reason",applied?"ATTACHED_INVENTORY_OBSERVED":"EMPTY_INVENTORY_OBSERVED"},{"models",{{41,"fixture"}}},
 {"execution",{{{"entity",41},{"epoch","plugin"},{"clock_epoch",3},{"accepted",applied?8:7},{"applied",applied?8:7},{"attached",applied}}}}};
}
TEST(LatestPayload, SourceClockDoesNotExpireTransactionEvidence){
 PayloadCommand command("fixture",true,inventory(),{S,S},S,S);
 EXPECT_TRUE(command.applied(inventory(true),{S/2,S+MS},S/2,100*S));
 EXPECT_FALSE(command.applied(inventory(),{S/2,S+MS},S/2,100*S));
 auto wrong=inventory(true);wrong["source_epoch"]="different";
 EXPECT_THROW(command.applied(wrong,{S,S},S,S),std::runtime_error);
}
TEST(LatestPayload, RebindingRejectsPreviousCallbackGeneration){
 PayloadWorldInbox inbox;inbox.bind(10*S);const auto previous=inbox.ticket();inbox.bind(20*S);
 ignition::msgs::Pose_V v;v.mutable_header()->mutable_stamp()->set_sec(1);
 inbox.receive(v,{S,S},previous);EXPECT_TRUE(inbox.take().empty());
 inbox.receive(v,{S,S},inbox.ticket());EXPECT_EQ(inbox.take().size(),1u);
}
geometry_msgs::msg::Pose pose(double x,double angle=0.) {
 geometry_msgs::msg::Pose p;p.position.x=x;p.orientation.z=std::sin(angle/2.);p.orientation.w=std::cos(angle/2.);return p;
}
TEST(SlamPoseStop, MissingOrInsufficientObservedWindowDoesNotConfirmStop){
 SlamPoseStopWindow window;EXPECT_FALSE(window.stopped());
 window.observe(pose(0.),S);EXPECT_FALSE(window.stopped());
 window.observe(pose(0.),S+500*MS);EXPECT_FALSE(window.stopped());
 window.observe(pose(0.),S+600*MS);EXPECT_TRUE(window.stopped());
 window.clear();EXPECT_FALSE(window.stopped());
}
TEST(SlamPoseStop, ReversingJitterDoesNotAccumulateAbsoluteTravel){
 SlamPoseStopWindow window;
 for(int i=0;i<=60;++i)window.observe(pose(i%2?.002:0.,i%2?.004:0.),S+i*10*MS);
 // Absolute travel is 0.12 m / 0.24 rad, while signed net deviation stays small.
 EXPECT_TRUE(window.stopped());
}
TEST(SlamPoseStop, NetDriftAndOutOfWindowExcursionCannotHideByReturning){
 SlamPoseStopWindow translation;translation.observe(pose(0.),S);translation.observe(pose(.006),S+600*MS);EXPECT_FALSE(translation.stopped());
 SlamPoseStopWindow rotation;rotation.observe(pose(0.),S);rotation.observe(pose(0.,.011),S+600*MS);EXPECT_FALSE(rotation.stopped());
 SlamPoseStopWindow returning;returning.observe(pose(0.),S);returning.observe(pose(.006),S+300*MS);returning.observe(pose(0.),S+600*MS);EXPECT_FALSE(returning.stopped());
}
TEST(SlamPoseStop, WrappedYawCrossingDoesNotInventFullRotation){
 SlamPoseStopWindow window;const auto pi=std::acos(-1.);
 for(int i=0;i<=60;++i)window.observe(pose(0.,i%2?-pi+.003:pi-.003),S+i*10*MS);
 EXPECT_TRUE(window.stopped());
}
}
