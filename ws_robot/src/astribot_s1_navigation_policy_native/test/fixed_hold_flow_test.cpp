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
  int64_t payload_steady_offset=0;
  FullChain() {coordinator.payload_source([this](int64_t at){return consumer.current(at,at+payload_steady_offset);});}
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
    if(!consumer.receive(s,at,at+payload_steady_offset)) {
      g.complete=g.attachment_state_confirmed=false;g.reason=consumer.reason();
      return;
    }
    const auto a=geometry::confirmedAttachments(consumer,at,at+payload_steady_offset);
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
TEST_F(FullChain, LedgerAttachmentChangeRevokesBeforeGeometryUpdate) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  now+=MS;observation.sequence++;observation.revision++;observation.status=observation.ATTACHED;observation.objects=dual_load();
  observation.transaction_id="attach_before_geometry";
  observation.observed_at=payload::stamp(now);observation.valid_until=payload::stamp(now+300*MS);
  ASSERT_TRUE(ledger.observe(observation,now,now));reconcile(now);
  ASSERT_TRUE(consumer.receive(ledger.state(now,now),now,now));
  // No geometry callback has occurred. An old explicit EMPTY cannot cover loads.
  EXPECT_TRUE(g.attachment_ids.empty());EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  EXPECT_EQ(coordinator.fault(),"PAYLOAD_MASS_REVISION_MISMATCH");
}
class LoadedChain:public FullChain {
protected:
  payload::Objects initial_objects() override {return dual_load();}
  std::string mass_case(double left,double right,double requested) {
    payload::Ledger independent{config,[](const auto&) {}};
    auto o=observation;o.objects=dual_load();o.objects[0].weight=left;o.objects[1].weight=right;
    if(!independent.observe(o,now,now))return "FIXTURE_OBSERVATION_REJECTED";
    auto ticket=independent.request(now,now);if(!ticket)return "FIXTURE_TICKET_UNAVAILABLE";
    moveit_msgs::msg::PlanningScene scene;scene.robot_state.attached_collision_objects=o.objects;
    if(!independent.reconcile(*ticket,scene,now,now))return "FIXTURE_RECONCILIATION_REJECTED";
    payload::Consumer confirmed{config};auto state=independent.state(now,now);
    if(!confirmed.receive(state,now,now))return confirmed.reason();
    auto geometry=g;geometry.attachment_revision=state.attachment_revision;
    auto status=hold.status(now,now);status.attachment_revision=state.attachment_revision;
    nav::FixedEnvelopeCore candidate{profile,"numeric_mass",1};
    candidate.payload_source([&confirmed](int64_t at){return confirmed.current(at,at);});
    candidate.state(geometry);candidate.hold(status);
    auto request=hold.request("numeric_mass",limits,now,now);request.limits.payload_mass_kg=requested;
    try {candidate.propose(request,now,true);return "ACCEPTED";}
    catch(const std::invalid_argument& e){return e.what();}
  }
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
TEST_F(LoadedChain, PositiveButUnderreportedMassMustBeRejected) {
  limits.payload_mass_kg=.001;
  EXPECT_THROW(coordinator.propose(hold.request("underreported",limits,now,now),now,true),std::invalid_argument);
}
TEST_F(LoadedChain, ExactAndConservativelyHigherMassRemainAdmissible) {
  for(double mass:{2.,2.5}) {
    limits.payload_mass_kg=mass;
    EXPECT_NO_THROW(coordinator.propose(hold.request("mass_"+std::to_string(mass),limits,now,now),now,true));
    EXPECT_EQ(coordinator.output()->limits.payload_mass_kg,mass);
  }
}
TEST_F(LoadedChain, NonfiniteMassCannotBeProposed) {
  for(double mass:std::vector<double>{NAN,INFINITY,-1.}) {
    limits.payload_mass_kg=mass;
    EXPECT_THROW(coordinator.propose(hold.request("bad_mass",limits,now,now),now,true),std::invalid_argument);
  }
}
TEST_F(LoadedChain, ConsumerMustMatchTheGeometryAttachmentRevision) {
  auto old_geometry=g;
  changed(observation.ATTACHED,dual_load());
  nav::FixedEnvelopeCore candidate{profile,"mismatched",1};
  candidate.payload_source([this](int64_t at){return consumer.current(at,at);});
  old_geometry.header.stamp=nav::fixed_stamp(now);old_geometry.valid_until=nav::fixed_stamp(now+300*MS);
  old_geometry.joint_source_stamps={old_geometry.header.stamp,old_geometry.header.stamp};
  auto h=nav::FixedEnvelopeCore::Hold{};h.header.stamp=old_geometry.header.stamp;h.owner_id="task";h.hold_id="hold";
  h.attachment_revision=old_geometry.attachment_revision;h.hold_confirmed=true;h.lease_s=.3;
  candidate.state(old_geometry);candidate.hold(h);
  nav::FixedEnvelopeCore::Request request;request.request_id="mismatch";request.hold_id="hold";request.geometry_sequence=old_geometry.sequence;request.limits=limits;
  try {candidate.propose(request,now,true);FAIL()<<"revision mismatch accepted";}
  catch(const std::invalid_argument& e){EXPECT_STREQ(e.what(),"PAYLOAD_MASS_REVISION_MISMATCH");}
}
TEST_F(LoadedChain, MissingOrAdditionalObjectIdentityIsRejected) {
  for(auto ids:std::vector<std::vector<std::string>>{{"left"},{"left","right","extra"},{"left","left"}}) {
    auto wrong=g;wrong.sequence++;wrong.attachment_ids=ids;
    nav::FixedEnvelopeCore candidate{profile,"mismatched_ids",1};
    candidate.payload_source([this](int64_t at){return consumer.current(at,at);});
    candidate.state(wrong);candidate.hold(hold.status(now,now));
    auto request=hold.request("ids",limits,now,now);request.geometry_sequence=wrong.sequence;
    EXPECT_THROW(candidate.propose(request,now,true),std::invalid_argument);
  }
}
TEST_F(LoadedChain, MissingConfiguredProviderDoesNotFallBackToEmpty) {
  nav::FixedEnvelopeCore candidate{profile,"no_source",1};candidate.state(g);candidate.hold(hold.status(now,now));
  try {candidate.propose(hold.request("source_missing",limits,now,now),now,true);FAIL()<<"missing source accepted";}
  catch(const std::invalid_argument& e){EXPECT_STREQ(e.what(),"PAYLOAD_SOURCE_NOT_CONFIGURED");}
}
TEST_F(LoadedChain, RepeatedLedgerCannotRenewMassWhenRosClockFreezes) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  auto old=ledger.state(now,now);
  payload_steady_offset=200*MS;EXPECT_TRUE(consumer.receive(old,now,now+payload_steady_offset));
  payload_steady_offset=301*MS;
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  EXPECT_EQ(coordinator.fault(),"PAYLOAD_MASS_EVIDENCE_UNAVAILABLE");
  EXPECT_FALSE(consumer.receive(old,now,now+payload_steady_offset));
  all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(LoadedChain, FreshMassAfterExpiryNeedsANewEnvelopeRequest) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);const auto old_epoch=coordinator.epoch();
  payload_steady_offset=301*MS;ASSERT_FALSE(coordinator.tick(now)->navigation_allowed);
  now+=MS;observation.sequence++;observation.observed_at=payload::stamp(now);observation.valid_until=payload::stamp(now+300*MS);
  ASSERT_TRUE(ledger.observe(observation,now,now));reconcile(now);consume(now);ASSERT_TRUE(g.complete);
  ASSERT_TRUE(hold.geometry(g,now,now));coordinator.state(g);coordinator.hold(hold.status(now,now));
  all();EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  EXPECT_NO_THROW(coordinator.propose(hold.request("mass_recovered",limits,now,now),now,true));
  EXPECT_GT(coordinator.epoch(),old_epoch);
  ack.envelope_epoch=coordinator.epoch();ack.installed_geometry_hash=coordinator.output()->installed_geometry_hash;ack.header.stamp=nav::fixed_stamp(now);
  all();EXPECT_TRUE(coordinator.tick(now)->navigation_allowed);
}
TEST_F(LoadedChain, DetachLedgerCannotRenewAnOldLoadedGeometry) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  now+=MS;observation.sequence++;observation.revision++;observation.status=observation.EMPTY;observation.objects.clear();
  observation.transaction_id="detach_before_geometry";
  observation.observed_at=payload::stamp(now);observation.valid_until=payload::stamp(now+300*MS);
  ASSERT_TRUE(ledger.observe(observation,now,now));reconcile(now);
  ASSERT_TRUE(consumer.receive(ledger.state(now,now),now,now));
  EXPECT_EQ(g.attachment_ids.size(),2u);EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  EXPECT_EQ(coordinator.fault(),"PAYLOAD_MASS_REVISION_MISMATCH");
}
TEST_F(LoadedChain, ChangedMassWithoutLedgerRevisionRevokes) {
  all();ASSERT_TRUE(coordinator.tick(now)->navigation_allowed);
  auto forged=ledger.state(now,now);forged.observation.objects[0].weight=10.;
  forged.geometry_digest=payload::digest(payload::canonical(forged.observation.objects,config.allowed_links).dump());
  EXPECT_FALSE(consumer.receive(forged,now,now));
  EXPECT_EQ(consumer.reason(),"ATTACHMENT_CONTENT_CHANGED_WITHOUT_REVISION");
  EXPECT_FALSE(coordinator.tick(now)->navigation_allowed);
  EXPECT_EQ(coordinator.fault(),"PAYLOAD_MASS_EVIDENCE_UNAVAILABLE");
}
TEST_F(LoadedChain, ZeroPhysicalWeightIsUnknownEvenWithPositiveRequestedMass) {
  EXPECT_EQ(mass_case(0.,1.,2.),"PAYLOAD_MASS_INVALID_WEIGHT");
}
TEST_F(LoadedChain, FiniteIndividualMassesCannotOverflowTheSum) {
  const auto maximum=std::numeric_limits<double>::max();
  EXPECT_EQ(mass_case(maximum,maximum,maximum),"PAYLOAD_MASS_SUM_OVERFLOW");
}
TEST_F(LoadedChain, DecimalSummationUsesOnlyOneRepresentableStep) {
  EXPECT_EQ(mass_case(.1,.2,.3),"ACCEPTED");
  EXPECT_EQ(mass_case(.1,.2,std::nextafter(.3,0.)),"PAYLOAD_MASS_UNDERREPORTED");
  const auto one_below=std::nextafter(2.,0.);
  EXPECT_EQ(mass_case(1.,1.,one_below),"ACCEPTED");
  EXPECT_EQ(mass_case(1.,1.,std::nextafter(one_below,0.)),"PAYLOAD_MASS_UNDERREPORTED");
}
}
