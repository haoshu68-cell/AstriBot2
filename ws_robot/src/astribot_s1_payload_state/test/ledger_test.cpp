#include <gtest/gtest.h>
#include "astribot_s1_payload_state/ledger.hpp"
#include <filesystem>
#include <fstream>
#include <unistd.h>
using namespace astribot::payload;
namespace {
constexpr int64_t T=1000000000;
Config config() {return {"simulation","session","physical_inventory","ledger_boot",{"left_tcp","right_tcp"}};}
Observation empty(uint64_t seq=1,uint64_t rev=1,int64_t at=T) {
  Observation o;o.environment="simulation";o.session_id="session";o.source_id="physical_inventory";
  o.source_epoch="boot_a";o.sequence=seq;o.revision=rev;o.observed_at=stamp(at);o.valid_until=stamp(at+kLease);
  o.full_inventory=true;o.status=Observation::EMPTY;o.transaction_id="reconcile_startup";return o;
}
moveit_msgs::msg::AttachedCollisionObject box(std::string id="load",std::string link="left_tcp") {
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name=link;a.object.header.frame_id=link;a.object.id=id;
  a.object.pose.orientation.w=1.;shape_msgs::msg::SolidPrimitive p;p.type=p.BOX;p.dimensions={.2,.1,.3};
  a.object.primitives.push_back(p);geometry_msgs::msg::Pose pose;pose.orientation.w=1.;pose.position.y=.25;
  a.object.primitive_poses.push_back(pose);return a;
}
Observation loaded(uint64_t seq=1,uint64_t rev=1,int64_t at=T) {
  auto o=empty(seq,rev,at);o.status=Observation::ATTACHED;o.objects={box()};return o;
}
moveit_msgs::msg::PlanningScene scene(const Observation &o) {moveit_msgs::msg::PlanningScene s;s.robot_state.attached_collision_objects=o.objects;return s;}
class Fixture: public ::testing::Test {
protected:
  std::vector<nlohmann::json> records;
  Ledger ledger{config(),[this](const auto &r){records.push_back(r);}};
  void confirm(const Observation &o=empty(),int64_t at=T,int64_t wall=T) {
    ASSERT_TRUE(ledger.observe(o,at,wall));auto request=ledger.request(at,wall);ASSERT_TRUE(request);
    ASSERT_TRUE(ledger.reconcile(*request,scene(o),at,wall));ASSERT_TRUE(ledger.state(at,wall).confirmed);
  }
};
TEST_F(Fixture, StartupAndSceneAloneCannotConfirmEmpty) {
  EXPECT_FALSE(ledger.state(T,T).confirmed);EXPECT_FALSE(ledger.request(T,T));
  EXPECT_FALSE(ledger.reconcile({0,T,T},scene(empty()),T,T));
}
TEST_F(Fixture, ExplicitEmptyNeedsReadbackAndDurableRecord) {
  ASSERT_TRUE(ledger.observe(empty(),T,T));EXPECT_FALSE(ledger.state(T,T).confirmed);
  auto ticket=ledger.request(T,T);ASSERT_TRUE(ticket);ASSERT_TRUE(ledger.reconcile(*ticket,scene(empty()),T,T));
  auto s=ledger.state(T,T);EXPECT_TRUE(s.confirmed);EXPECT_EQ(s.observation.status,Observation::EMPTY);
  EXPECT_FALSE(s.attachment_revision.empty());EXPECT_GT(s.ledger_revision,0u);EXPECT_FALSE(records.empty());
}
TEST_F(Fixture, MissingInventoryOrWrongSourceCannotClearLoad) {
  confirm(loaded());auto bad=empty(2,2,T+1);bad.full_inventory=false;
  EXPECT_FALSE(ledger.observe(bad,T+1,T+1));EXPECT_FALSE(ledger.state(T+1,T+1).confirmed);
  EXPECT_FALSE(ledger.state(T+1,T+1).observation.objects.empty());
}
TEST_F(Fixture, ForeignSessionDoesNotReplaceConfirmedSource) {
  confirm();auto bad=empty(2,1,T+100);bad.session_id="foreign";EXPECT_FALSE(ledger.observe(bad,T+100,T+100));
  EXPECT_TRUE(ledger.state(T+100,T+100).confirmed);EXPECT_TRUE(ledger.state(T+kLease,T+kLease).confirmed);
}
TEST_F(Fixture, ObservationDoesNotExpireBySourceOrSteadyAge) {
  confirm();EXPECT_TRUE(ledger.state(T+kLease-1,T+kLease-1).confirmed);
  EXPECT_TRUE(ledger.state(T+kLease,T+kLease).confirmed);
}
TEST_F(Fixture, FrozenRosTimeRetainsLatestObservation) {
  confirm();EXPECT_TRUE(ledger.state(T,T+kLease).confirmed);
}
TEST_F(Fixture, RepeatedEvidenceRetainsDiagnosticCaptureDeadline) {
  confirm();auto ticket=ledger.request(T+100,T+100);ASSERT_TRUE(ticket);
  EXPECT_FALSE(ledger.observe(empty(),T+100,T+100));
  EXPECT_TRUE(ledger.reconcile(*ticket,scene(empty()),T+100,T+100));
  EXPECT_TRUE(ledger.state(T+kLease,T+kLease).confirmed);
}
TEST_F(Fixture, EmptyLoadedEmptyNeverReusesVersion) {
  confirm();auto first=ledger.state(T,T);confirm(loaded(2,2,T+10),T+10,T+10);auto second=ledger.state(T+10,T+10);
  confirm(empty(3,3,T+20),T+20,T+20);auto third=ledger.state(T+20,T+20);
  EXPECT_NE(first.attachment_revision,third.attachment_revision);EXPECT_EQ(first.geometry_digest,third.geometry_digest);
  EXPECT_GT(third.ledger_revision,second.ledger_revision);
}
TEST_F(Fixture, ContentChangeWithoutSourceRevisionIsRejected) {
  confirm();EXPECT_FALSE(ledger.observe(loaded(2,1,T+1),T+1,T+1));EXPECT_FALSE(ledger.state(T+1,T+1).confirmed);
}
TEST_F(Fixture, DelayedSceneCannotConfirmNewRevision) {
  ASSERT_TRUE(ledger.observe(empty(),T,T));auto old=ledger.request(T,T);ASSERT_TRUE(old);
  ASSERT_TRUE(ledger.observe(loaded(2,2,T+1),T+1,T+1));
  EXPECT_FALSE(ledger.reconcile(*old,scene(empty()),T+2,T+2));EXPECT_FALSE(ledger.state(T+2,T+2).confirmed);
}
TEST_F(Fixture, SceneMismatchOrDiffCannotConfirm) {
  ASSERT_TRUE(ledger.observe(loaded(),T,T));auto ticket=ledger.request(T,T);ASSERT_TRUE(ticket);
  EXPECT_FALSE(ledger.reconcile(*ticket,scene(empty()),T,T));
  auto s=scene(loaded());s.is_diff=true;EXPECT_FALSE(ledger.reconcile(*ticket,s,T,T));
}
TEST_F(Fixture, ExpiredReadbackCannotConfirmFreshObservation) {
  ASSERT_TRUE(ledger.observe(empty(),T,T));auto ticket=ledger.request(T,T);ASSERT_TRUE(ticket);
  ASSERT_TRUE(ledger.observe(empty(2,1,T+kLease),T+kLease,T+kLease));
  EXPECT_FALSE(ledger.reconcile(*ticket,scene(empty()),T+kLease,T+kLease));
}
TEST_F(Fixture, SourceRestartNeedsReadbackAndRetiredEpochCannotReturn) {
  confirm();auto b=empty();b.source_epoch="boot_b";b.observed_at=stamp(T+1);b.valid_until=stamp(T+1+kLease);
  ASSERT_TRUE(ledger.observe(b,T+1,T+1));EXPECT_FALSE(ledger.state(T+1,T+1).confirmed);
  auto ticket=ledger.request(T+1,T+1);ASSERT_TRUE(ticket);ASSERT_TRUE(ledger.reconcile(*ticket,scene(b),T+1,T+1));
  auto old=empty(100,100,T+2);EXPECT_FALSE(ledger.observe(old,T+2,T+2));
  EXPECT_EQ(ledger.state(T+2,T+2).observation.source_epoch,"boot_b");
}
TEST_F(Fixture, LocalClockRollbackDoesNotChangeSourceEpoch) {
  confirm();EXPECT_TRUE(ledger.state(T-1,T+1).confirmed);
  EXPECT_TRUE(ledger.observe(empty(2,1,T),T,T+2));
  auto o=empty(3,2,T);o.clock_epoch=1;ASSERT_TRUE(ledger.observe(o,T,T+3));
  EXPECT_FALSE(ledger.state(T,T+3).confirmed);auto ticket=ledger.request(T,T+3);ASSERT_TRUE(ticket);
  EXPECT_TRUE(ledger.reconcile(*ticket,scene(o),T,T+3));
}
TEST_F(Fixture, PendingReleaseCannotPublishEmpty) {
  confirm(loaded());auto pending=loaded(2,2,T+1);pending.status=Observation::TRANSITION;
  EXPECT_TRUE(ledger.observe(pending,T+1,T+1));EXPECT_FALSE(ledger.state(T+1,T+1).confirmed);
  EXPECT_FALSE(ledger.request(T+1,T+1));
}
TEST_F(Fixture, FutureObservationAcceptedButConflictingDuplicateRejected) {
  EXPECT_TRUE(ledger.observe(empty(1,1,T+1),T,T));auto o=empty();o.valid_until=stamp(T+kLease+1);
  EXPECT_FALSE(ledger.observe(o,T,T));EXPECT_FALSE(ledger.state(T,T).confirmed);
}
TEST_F(Fixture, DualDistinctPayloadsAcceptedAndDuplicateObjectRejected) {
  auto o=loaded();o.objects.push_back(box("right_load","right_tcp"));confirm(o);
  auto bad=o;bad.sequence=2;bad.revision=2;bad.objects[1].object.id="load";
  EXPECT_FALSE(ledger.observe(bad,T,T));EXPECT_FALSE(ledger.state(T,T).confirmed);
}
TEST_F(Fixture, InvalidDimensionsPoseAndUnknownLinkAreRejected) {
  for(int type=0;type<4;++type) {
    auto o=loaded();if(type==0)o.objects[0].object.primitives[0].dimensions[0]=-1;
    if(type==1)o.objects[0].object.pose.orientation.w=0;
    if(type==2)o.objects[0].link_name="foreign_link";
    if(type==3)o.objects[0].object.primitive_poses[0].position.x=NAN;
    EXPECT_FALSE(ledger.observe(o,T,T));EXPECT_FALSE(ledger.state(T,T).confirmed);
  }
}
TEST_F(Fixture, UnsupportedMeshAndEmptyLoadedGeometryAreRejected) {
  auto o=loaded();o.objects[0].object.meshes.emplace_back();EXPECT_FALSE(ledger.observe(o,T,T));
  o=loaded();o.objects[0].object.primitives.clear();o.objects[0].object.primitive_poses.clear();EXPECT_FALSE(ledger.observe(o,T,T));
}
TEST_F(Fixture, GeometryOrderDoesNotChangeDigest) {
  auto a=loaded();a.objects.push_back(box("right_load","right_tcp"));confirm(a);auto before=ledger.state(T,T);
  std::reverse(a.objects.begin(),a.objects.end());a.sequence=2;a.observed_at=stamp(T+1);a.valid_until=stamp(T+1+kLease);
  ASSERT_TRUE(ledger.observe(a,T+1,T+1));EXPECT_EQ(before.geometry_digest,ledger.state(T+1,T+1).geometry_digest);
  EXPECT_EQ(before.attachment_revision,ledger.state(T+1,T+1).attachment_revision);
}
TEST(Storage, CommitFailureLatchesWithoutGrantingPermission) {
  Ledger l(config(),[](const auto &){throw std::runtime_error("disk full");});
  EXPECT_FALSE(l.observe(empty(),T,T));EXPECT_FALSE(l.state(T,T).confirmed);
  EXPECT_FALSE(l.observe(empty(2,2,T+1),T+1,T+1));
}
TEST(Storage, RestartDoesNotRestoreAuthorization) {
  Ledger l(config(),[](const auto &){});EXPECT_FALSE(l.state(T,T).confirmed);
}
TEST(Storage, JournalPersistsAndExclusiveOwnerIsRequired) {
  auto path=std::filesystem::temp_directory_path()/("astribot_payload_"+std::to_string(getpid())+".jsonl");
  std::filesystem::remove(path);
  {Journal j(path.string());j.append({{"revision",7},{"status","EMPTY"}});EXPECT_THROW(Journal duplicate(path.string()),std::exception);}
  std::ifstream stream(path);std::string line;std::getline(stream,line);ASSERT_FALSE(line.empty());
  EXPECT_EQ(nlohmann::json::parse(line).at("revision"),7);std::filesystem::remove(path);
}
TEST(Storage, InvalidJournalPathFailsAtStartup) {EXPECT_THROW(Journal j("/nonexistent_parent_astribot/state.jsonl"),std::exception);}
TEST_F(Fixture, InvalidPhysicalEvidenceCannotBeHealedBySceneAlone) {
  confirm(loaded());auto bad=empty(2,2,T+1);bad.full_inventory=false;
  ASSERT_FALSE(ledger.observe(bad,T+1,T+1));
  EXPECT_FALSE(ledger.request(T+2,T+2));
}
TEST_F(Fixture, DiagnosticDeadlineDoesNotControlConfirmation) {
  auto o=empty();o.valid_until=stamp(T+200);confirm(o);
  o.sequence=2;o.valid_until=stamp(T+400);
  ASSERT_TRUE(ledger.observe(o,T+100,T+100));
  EXPECT_TRUE(ledger.state(T+200,T+200).confirmed);
}
TEST_F(Fixture, MalformedStateCannotBeClearedByIdenticalGoodReplay) {
  confirm();auto bad=loaded(2,1,T+1);ASSERT_FALSE(ledger.observe(bad,T+1,T+1));
  EXPECT_FALSE(ledger.observe(empty(),T+2,T+2));EXPECT_FALSE(ledger.request(T+2,T+2));
}

TEST_F(Fixture, QueuedObservationRemainsValidAndSceneRequestIsTimed) {
  ASSERT_TRUE(ledger.observe(empty(),T,T+400000000,Receipt{T,T}));
  auto ticket=ledger.request(T,T+400000000);ASSERT_TRUE(ticket);
  ASSERT_TRUE(ledger.reconcile(*ticket,scene(empty()),T,T+400000000));
  EXPECT_TRUE(ledger.state(T,T+kLease).confirmed);
}
TEST_F(Fixture, OverflowFaultCannotBeHealedBySceneReadback) {
  confirm();ledger.source_fault("INPUT_QUEUE_OVERFLOW",T,T);
  EXPECT_FALSE(ledger.state(T+1,T+1).confirmed);EXPECT_FALSE(ledger.request(T+1,T+1));
}
TEST_F(Fixture, NewLedgerStartsWithIdentifiableUnknownState) {
  const auto s=ledger.state(T,T);EXPECT_FALSE(s.confirmed);
  EXPECT_EQ(s.observation.session_id,"session");EXPECT_EQ(s.observation.source_id,"physical_inventory");
  EXPECT_EQ(s.observation.environment,"simulation");EXPECT_EQ(s.ledger_epoch,"ledger_boot");
}
TEST_F(Fixture, FutureSameContextPacketReplacesOlderObservation) {
  confirm();auto future=empty(2,1,T+1);
  EXPECT_TRUE(ledger.observe(future,T,T+100));EXPECT_TRUE(ledger.state(T,T+100).confirmed);
  future.status=Observation::UNKNOWN;
  EXPECT_FALSE(ledger.observe(future,T,T+101));EXPECT_FALSE(ledger.state(T,T+101).confirmed);
}
TEST_F(Fixture, SourceDeadlineIsPreservedAsDiagnostic) {
  auto o=empty();o.valid_until=stamp(T+200);confirm(o);
  o.sequence=2;o.valid_until=stamp(T+400);ASSERT_TRUE(ledger.observe(o,T+100,T+100));
  EXPECT_EQ(ns(ledger.state(T+100,T+100).valid_until),T+400);
}

TEST_F(Fixture, FaultRecoveryRequiresNewCaptureNotMerelyNewSequence) {
  confirm();ledger.source_fault("INPUT_QUEUE_OVERFLOW",T,T);
  EXPECT_FALSE(ledger.observe(empty(2,1,T),T+1,T+1));EXPECT_FALSE(ledger.request(T+1,T+1));
  confirm(empty(3,1,T+2),T+2,T+2);
}

TEST_F(Fixture, ExpiredOutOfOrderPacketDoesNotRevokeFreshState) {
  confirm();confirm(empty(2,1,T+400000000),T+400000000,T+400000000);
  EXPECT_FALSE(ledger.observe(empty(),T+600000000,T+600000000));
  EXPECT_TRUE(ledger.state(T+600000000,T+600000000).confirmed);
}
}

TEST_F(Fixture, HumbleSceneRoundtripPreservesPhysicalMassAndDigest) {
  auto o=loaded();o.objects[0].weight=.75;o.objects[0].object.pose.position.y=-.2;
  ASSERT_TRUE(ledger.observe(o,T,T));const auto expected=ledger.state(T,T).geometry_digest;
  auto ticket=ledger.request(T,T);ASSERT_TRUE(ticket);auto returned=scene(o);
  returned.robot_state.attached_collision_objects[0].weight=0.;
  returned.robot_state.attached_collision_objects[0].object.pose.position.y-=4e-16;
  returned.robot_state.attached_collision_objects[0].object.pose.orientation.x=3e-18;
  returned.robot_state.attached_collision_objects[0].object.pose.orientation.w+=7e-16;
  ASSERT_TRUE(ledger.reconcile(*ticket,returned,T,T));
  EXPECT_EQ(ledger.state(T,T).geometry_digest,expected);
  EXPECT_DOUBLE_EQ(ledger.state(T,T).observation.objects[0].weight,.75);
  auto changed=o;changed.sequence=2;changed.objects[0].weight=1.;
  EXPECT_FALSE(ledger.observe(changed,T,T));EXPECT_FALSE(ledger.state(T,T).confirmed);
}
TEST_F(Fixture, SceneRoundtripToleranceDoesNotHideCollisionOrMassChanges) {
  auto o=loaded();o.objects[0].weight=.75;
  for(int field=0;field<7;++field) {
    Ledger candidate(config(),[](const auto &){});
    ASSERT_TRUE(candidate.observe(o,T,T));auto ticket=candidate.request(T,T);ASSERT_TRUE(ticket);
    auto returned=scene(o);auto &v=returned.robot_state.attached_collision_objects[0];v.weight=0;
    if(field==0)v.object.pose.position.x=1e-8;
    if(field==1)v.object.primitive_poses[0].position.y+=1e-8;
    if(field==2)v.object.primitives[0].dimensions[0]-=1e-8;
    if(field==3)v.touch_links={"foreign"};
    if(field==4)v.weight=.8;
    if(field==5)v.weight=-1.;
    if(field==6)v.weight=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(candidate.reconcile(*ticket,returned,T,T))<<field;
  }
}
TEST_F(Fixture, SceneQuaternionSignNearHalfTurnIsEquivalent) {
  auto o=loaded();o.objects[0].weight=.75;o.objects[0].object.pose.orientation.w=0;
  o.objects[0].object.pose.orientation.x=1.;ASSERT_TRUE(ledger.observe(o,T,T));
  auto ticket=ledger.request(T,T);ASSERT_TRUE(ticket);auto returned=scene(o);
  returned.robot_state.attached_collision_objects[0].weight=0;
  returned.robot_state.attached_collision_objects[0].object.pose.orientation.w=-1e-16;
  EXPECT_TRUE(ledger.reconcile(*ticket,returned,T,T));
}
TEST_F(Fixture, FaultRecoveryUsesSourceOrderingNotLocalReceiveTime) {
  confirm();ledger.source_fault("INVENTORY_CHANGED",T+100000000,T+100000000);
  EXPECT_TRUE(ledger.observe(empty(2,1,T+80000000),T+160000000,T+160000000));
  EXPECT_FALSE(ledger.state(T+160000000,T+160000000).confirmed);
  auto fresh=empty(3,1,T+120000000);
  ASSERT_TRUE(ledger.observe(fresh,T+200000000,T+200000000));
  auto ticket=ledger.request(T+200000000,T+200000000);ASSERT_TRUE(ticket);
  EXPECT_TRUE(ledger.reconcile(*ticket,scene(fresh),T+200000000,T+200000000));
}
