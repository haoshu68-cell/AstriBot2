#include <gtest/gtest.h>
#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include "astribot_s1_transport_native/arm_hold.hpp"
#include "astribot_s1_robot_geometry/attachment_geometry.hpp"
namespace nav=astribot::navigation;
namespace task=astribot::transport;
namespace {
constexpr int64_t T=10000000000LL,MS=1000000;
geometry_msgs::msg::Polygon square(float r) {
  geometry_msgs::msg::Polygon p;
  for(auto xy:std::vector<std::pair<float,float>>{{-r,-r},{r,-r},{r,r},{-r,r}}) {
    geometry_msgs::msg::Point32 v;v.x=xy.first;v.y=xy.second;p.points.push_back(v);
  }return p;
}
class Flow:public ::testing::Test {
protected:
  nlohmann::json profile=nav::load_policy_profile(PROFILE_FILE,true);
  nav::FixedEnvelopeCore coordinator{profile,"coordinator",1};task::ArmHold hold;
  task::Geometry g;
  astribot_navigation_msgs::msg::RobotEnvelope limits;
  nav::FixedEnvelopeCore::Ack ack;
  int64_t now=T+600*MS;
  virtual void attachments(int64_t,uint64_t) {}
  virtual double payload_mass() const {return profile.at("payload_mass_kg").get<double>();}
  void SetUp() override {
    task::ResourceGrant grant{"task","lease","resource_boot",{"left","right"},T,T+2000*MS,T,T};
    task::HoldCompletion completed{"task","lease","resource_boot","hold_action",T,true,true};hold.begin("hold",grant,completed,T,T);
    g.header.frame_id=profile.at("base_frame");g.source_id="geometry";g.model_revision="model";g.attachment_revision="empty_1";
    g.complete=g.attachment_state_confirmed=true;g.joints.name={"left","right"};g.joints.position={0.,0.};g.joint_position_error_bounds={.003,.003};
    g.physical_footprint=square(.3F);g.reserved_footprint=square(.5F);g.height_m=1.7;
    controller_manager_msgs::msg::ControllerState controller;controller.name="arms";controller.state="active";
    controller.type="joint_trajectory_controller/JointTrajectoryController";controller.claimed_interfaces={"left/position","right/position"};
    for(int i=0;i<3;++i) {
      auto at=T+(100+250*i)*MS;g.sequence=i+1;g.header.stamp=nav::fixed_stamp(at);g.joint_source_stamps={g.header.stamp,g.header.stamp};g.valid_until=nav::fixed_stamp(at+300*MS);
      attachments(at,i+1);
      hold.controllers({controller},at,at,at,at);ASSERT_TRUE(hold.geometry(g,at,at));coordinator.state(g);
    }
    limits.frame_id=profile.at("base_frame");limits.posture_id="nonhome";limits.lease_s=.3;
#define FIELD(name) limits.name=profile.at(#name).get<double>()
    FIELD(half_length_m);FIELD(half_width_m);FIELD(height_m);FIELD(payload_mass_kg);FIELD(max_speed_m_s);
    FIELD(max_angular_speed_rad_s);FIELD(max_acceleration_m_s2);FIELD(brake_deceleration_m_s2);
#undef FIELD
    limits.payload_mass_kg=payload_mass();
    coordinator.hold(hold.status(now,now));coordinator.propose(hold.request("request",limits,now,now),now,true);
    ack.coordinator_session_id=coordinator.session();ack.envelope_epoch=coordinator.epoch();
    ack.installed_geometry_hash=coordinator.output()->installed_geometry_hash;ack.applied=true;ack.header.stamp=nav::fixed_stamp(now);
  }
  void acknowledge(const std::string &consumer) {ack.consumer_id=consumer;coordinator.acknowledge(ack,now);}
  void all() {for(const auto *name:{"controller","global_costmap","local_costmap","planner","policy","protection"})acknowledge(name);}
};
TEST_F(Flow, MissingControllerAndLocalCostmapStayExplicitlyBlocked) {
  for(const auto *name:{"global_costmap","planner","policy","protection"})acknowledge(name);
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  EXPECT_EQ(coordinator.output()->reason,"WAITING_FOR:controller,local_costmap");
  acknowledge("controller");EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  acknowledge("local_costmap");EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(Flow, CancelThenLateAckCannotRestoreMotion) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  hold.cancel();coordinator.hold(hold.status(now,now));EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(Flow, AttachmentChangeBetweenTimerTicksRevokesImmediately) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  auto changed=g;changed.sequence=4;changed.attachment_revision="loaded_2";coordinator.state(changed);
  EXPECT_FALSE(coordinator.output()->navigation_allowed);
  EXPECT_FALSE(coordinator.output()->limits.transport_ready);
  changed.sequence=5;changed.attachment_revision="empty_1";coordinator.state(changed);
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(Flow, SamePolygonOldEpochAckCannotAuthorizeNewRequest) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);const auto old_ack=ack;
  coordinator.propose(hold.request("request_2",limits,now,now),now,true);
  for(const auto *name:{"controller","global_costmap","local_costmap","planner","policy","protection"}) {
    auto stale=old_ack;stale.consumer_id=name;coordinator.acknowledge(stale,now);
  }
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  ack.envelope_epoch=coordinator.epoch();all();EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(Flow, NegativeAckAlwaysRemovesCurrentPermission) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);ack.applied=false;acknowledge("controller");
  EXPECT_FALSE(coordinator.output()->navigation_allowed);
  EXPECT_FALSE(coordinator.output()->limits.transport_ready);
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  now+=MS;ack.header.stamp=nav::fixed_stamp(now);ack.applied=true;acknowledge("controller");EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}

TEST_F(Flow, PreRevocationAndSameStampPositiveAcksCannotRestorePermission) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  const auto old=ack;
  now+=10*MS;ack.header.stamp=nav::fixed_stamp(now);ack.applied=false;acknowledge("controller");
  ASSERT_FALSE(coordinator.tick(now)->navigation_allowed);
  ack=old;acknowledge("controller");
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  ack.header.stamp=nav::fixed_stamp(now);acknowledge("controller");
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  now+=MS;ack.header.stamp=nav::fixed_stamp(now);acknowledge("controller");
  EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}

TEST_F(Flow, ReorderedNegativeAckCannotLowerTheRecoveryBoundary) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  const auto base=now;now+=21*MS;ack.applied=false;
  ack.header.stamp=nav::fixed_stamp(base+20*MS);acknowledge("controller");
  ack.header.stamp=nav::fixed_stamp(base+10*MS);acknowledge("controller");
  ack.applied=true;ack.header.stamp=nav::fixed_stamp(base+15*MS);acknowledge("controller");
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  ack.header.stamp=nav::fixed_stamp(base+21*MS);acknowledge("controller");
  EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}

TEST_F(Flow, NewEnvelopeEpochDoesNotReuseTheOldNegativeAckBoundary) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  ack.applied=false;ack.header.stamp=nav::fixed_stamp(now+100*MS);acknowledge("controller");
  ASSERT_FALSE(coordinator.tick(now)->navigation_allowed);
  coordinator.propose(hold.request("new_request",limits,now,now),now,true);
  ack.envelope_epoch=coordinator.epoch();ack.applied=true;ack.header.stamp=nav::fixed_stamp(now);
  all();EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}

TEST_F(Flow, HoldRevocationBetweenTicksRemainsLatched) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  const auto positive=hold.status(now,now);auto negative=positive;negative.hold_confirmed=false;
  coordinator.hold(negative);EXPECT_FALSE(coordinator.output()->navigation_allowed);
  EXPECT_FALSE(coordinator.output()->limits.transport_ready);
  coordinator.hold(positive);EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(Flow, ClockEpochWithResetSequenceInvalidatesOldReservation) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  auto changed=g;changed.clock_epoch++;changed.sequence=1;coordinator.state(changed);
  EXPECT_FALSE(coordinator.output()->navigation_allowed);
  EXPECT_EQ(coordinator.fault(),"GEOMETRY_VERSION_CHANGED");
}
TEST_F(Flow, ConflictingSameSequenceCannotHideAttachmentRevocation) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  auto changed=g;changed.attachment_state_confirmed=false;coordinator.state(changed);
  EXPECT_FALSE(coordinator.output()->navigation_allowed);
  coordinator.state(g);EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}

namespace payload=astribot::payload;
namespace geometry=astribot_s1_robot_geometry;
// Deterministic inputs, not a ROS integration test or a physical inventory provider.
class FullChain:public Flow {
protected:
  payload::Config config{"simulation","fixture_session","fixture_inventory","fixture_ledger",{"left_tcp","right_tcp"}};
  payload::Ledger ledger{config,[](const auto &) {}};
  payload::Consumer consumer{config};
  payload::Observation observation;
  virtual payload::Objects initial_objects() {return {};}
  double payload_mass() const override {
    double mass=0.;for(const auto &object:observation.objects)mass+=object.weight;return mass;
  }
  void attachments(int64_t at,uint64_t seq) override {
    observation.environment=config.environment;observation.session_id=config.session;observation.source_id=config.source;
    observation.source_epoch="fixture_source";observation.sequence=seq;observation.revision=1;
    observation.objects=initial_objects();observation.full_inventory=true;
    observation.status=observation.objects.empty()?observation.EMPTY:observation.ATTACHED;observation.transaction_id="fixture_init";
    observation.observed_at=payload::stamp(at);observation.valid_until=payload::stamp(at+300*MS);
    ASSERT_TRUE(ledger.observe(observation,at,at));reconcile(at);consume(at);
  }
  void reconcile(int64_t at,bool mismatch=false) {
    auto ticket=ledger.request(at,at);ASSERT_TRUE(ticket);
    moveit_msgs::msg::PlanningScene scene;scene.robot_state.attached_collision_objects=observation.objects;
    if(mismatch)scene.is_diff=true;
    EXPECT_EQ(ledger.reconcile(*ticket,scene,at,at),!mismatch);
  }
  void consume(int64_t at) {
    auto s=ledger.state(at,at);g.sequence=observation.sequence;g.header.stamp=payload::stamp(at);
    g.joint_source_stamps={g.header.stamp,g.header.stamp};
    if(!consumer.receive(s,at,at)) {
      g.complete=g.attachment_state_confirmed=false;g.reason=consumer.reason();
      return;
    }
    const auto a=geometry::confirmedAttachments(consumer,at,at);
    g.complete=g.attachment_state_confirmed=true;g.attachment_revision=a.revision;g.attachment_ids=a.ids;
    g.valid_until=payload::stamp(a.valid_until);
    // Known fixture TCP-to-base transforms are identity; production uses URDF FK.
    double x=.3,y=.3;
    for(const auto &shape:a.shapes) {
      x=std::max({x,shape.support({1,0,0},geometry::Transform::Identity()),shape.support({-1,0,0},geometry::Transform::Identity())});
      y=std::max({y,shape.support({0,1,0},geometry::Transform::Identity()),shape.support({0,-1,0},geometry::Transform::Identity())});
    }
    g.physical_footprint=square(static_cast<float>(std::max(x,y)));
    g.reserved_footprint=square(static_cast<float>(std::max(x,y)+.2));
  }
  void changed(uint8_t status,payload::Objects objects,bool mismatch=false) {
    now+=10*MS;observation.sequence++;observation.revision++;observation.status=status;observation.objects=std::move(objects);
    observation.transaction_id="fixture_change_"+std::to_string(observation.revision);
    observation.observed_at=payload::stamp(now);observation.valid_until=payload::stamp(now+300*MS);
    ASSERT_TRUE(ledger.observe(observation,now,now));reconcile(now,mismatch);consume(now);
    hold.geometry(g,now,now);coordinator.state(g);coordinator.hold(hold.status(now,now));
  }
  payload::Objects dual_load() {
    payload::Objects objects;
    for(const auto *side:{"left","right"}) {
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name=std::string(side)+"_tcp";a.object.header.frame_id=a.link_name;
      a.object.id=side;a.object.pose.orientation.w=1.;a.object.pose.position.y=std::string(side)=="left"?.7:-.7;a.weight=1.;
      shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.2,.2,.2};a.object.primitives={box};
      geometry_msgs::msg::Pose pose;pose.orientation.w=1.;a.object.primitive_poses={pose};objects.push_back(a);
    }return objects;
  }
};
TEST_F(FullChain, ConfirmedEmptyVersionReachesAllConsumers) {
  const auto state=ledger.state(now,now);ASSERT_TRUE(state.confirmed);EXPECT_EQ(g.attachment_revision,state.attachment_revision);
  EXPECT_EQ(coordinator.output()->attachment_revision,state.attachment_revision);
  EXPECT_TRUE(g.attachment_ids.empty());EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  all();EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(FullChain, DistinctDualLoadsExpandGeometryAndRevokeOldEmptyPermission) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);const auto empty=g.attachment_revision;
  changed(observation.ATTACHED,dual_load());EXPECT_TRUE(g.complete);EXPECT_EQ(g.attachment_ids.size(),2u);
  EXPECT_NE(g.attachment_revision,empty);EXPECT_LE(g.physical_footprint.points[0].x,-.8F);
  EXPECT_GE(std::abs(g.physical_footprint.points[0].y),.8F);
  EXPECT_FALSE(hold.status(now,now).hold_confirmed);all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(FullChain, EmptyLoadedEmptyUsesNewVersionAndLateAcksCannotRestoreOldRequest) {
  const auto first=g.attachment_revision;changed(observation.ATTACHED,dual_load());const auto loaded=g.attachment_revision;
  changed(observation.EMPTY,{});EXPECT_TRUE(g.complete);EXPECT_TRUE(g.attachment_ids.empty());
  EXPECT_NE(g.attachment_revision,first);EXPECT_NE(g.attachment_revision,loaded);
  all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(FullChain, IncompleteSceneReadbackStopsTheWholeAdmissionChain) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  changed(observation.ATTACHED,dual_load(),true);EXPECT_FALSE(g.complete);EXPECT_FALSE(g.attachment_state_confirmed);
  EXPECT_FALSE(hold.status(now,now).hold_confirmed);all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(FullChain, ExpiredSourceCannotBeRenewedByPublishingTheLedger) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  now+=300*MS;observation.sequence++;consume(now);EXPECT_FALSE(g.complete);
  hold.geometry(g,now,now);coordinator.state(g);coordinator.hold(hold.status(now,now));
  all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
class LoadedChain:public FullChain {
protected: payload::Objects initial_objects() override {return dual_load();}
};
TEST_F(LoadedChain, StableDualLoadRequiresTheSameSixConsumerConfirmations) {
  ASSERT_EQ(g.attachment_ids.size(),2u);ASSERT_TRUE(hold.status(now,now).hold_confirmed);
  EXPECT_EQ(coordinator.output()->limits.payload_mass_kg,2.);
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);all();EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(LoadedChain, LoadedGeometryCannotProposeAnUnknownMassEnvelope) {
  limits.payload_mass_kg=0.;
  EXPECT_THROW(coordinator.propose(hold.request("unknown_mass",limits,now,now),now,true),std::invalid_argument);
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
}
