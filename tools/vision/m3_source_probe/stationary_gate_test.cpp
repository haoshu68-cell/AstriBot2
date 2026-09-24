#include "stationary_gate.hpp"
#include <gtest/gtest.h>
#include <functional>
#include <limits>
#include <vector>

using namespace m3_probe;
namespace {
builtin_interfaces::msg::Time source_time(int64_t ns) {
  builtin_interfaces::msg::Time t;t.sec=int32_t(ns/1000000000);t.nanosec=uint32_t(ns%1000000000);return t;
}
struct Fixture {
  StationaryGate gate{"owner","hold","coordinator"};
  Time now{10000000000LL,20000000000LL};
  Geometry geometry;
  Envelope envelope;
  Hold hold;
  Odom odom;
  Command command;
  Fixture() {
    geometry.source_id="geometry";geometry.model_revision="robot";geometry.attachment_revision="empty";
    geometry.complete=true;geometry.attachment_state_confirmed=true;
    envelope.coordinator_session_id="coordinator";envelope.hold_id="hold";envelope.epoch=7;
    envelope.model_revision="robot";envelope.attachment_revision="empty";envelope.installed_geometry_hash="shape";
    envelope.mode=Envelope::FIXED_POSTURE;envelope.navigation_allowed=true;envelope.limits.transport_ready=true;
    hold.owner_id="owner";hold.hold_id="hold";hold.attachment_revision="empty";hold.lease_s=.3;hold.hold_confirmed=true;
    odom.header.frame_id="odom";odom.child_frame_id="base";odom.pose.pose.orientation.w=1;
  }
  Ack ack(const std::string& consumer)const {
    Ack a;a.header.stamp=source_time(now.ros);a.coordinator_session_id="coordinator";a.consumer_id=consumer;
    a.envelope_epoch=envelope.epoch;a.installed_geometry_hash=envelope.installed_geometry_hash;a.applied=true;return a;
  }
  void advance(int64_t duration=50*ms){now.ros+=duration;now.steady+=duration;}
  void refresh(const std::string& missing_ack={}) {
    geometry.header.stamp=envelope.header.stamp=hold.header.stamp=odom.header.stamp=source_time(now.ros);
    geometry.valid_until=envelope.valid_until=source_time(now.ros+300*ms);
    gate.receive(geometry,now);gate.receive(envelope,now);gate.receive(hold,now);
    for(const auto& c:StationaryGate::consumers)if(c!=missing_ack)gate.receive(ack(c),now);
    gate.receive(command,now);gate.receive(odom,now);
  }
  std::string arm() {
    std::string why;
    for(int i=0;i<20;++i) {
      refresh();why=gate.arm(envelope,geometry,now);
      if(why.empty())return why;
      if(!gate.fault().empty())return gate.fault();
      advance();
    }
    return why;
  }
};
std::deque<StopSample> stopped_rows() {
  std::deque<StopSample> rows;
  for(int i=0;i<13;++i) {
    const int64_t t=1000*ms+i*50*ms;
    rows.push_back({t,{t,t+1000*ms},{t,t+1000*ms},0,0,0,0,0,0,0});
  }
  return rows;
}
StopMetrics stop(const std::deque<StopSample>& rows) {return measured_stop(rows,999*ms,rows.back().received);}

TEST(StationaryGate, ReadyFixedEnvelopeAllowsNavigationButDoesNotAuthorizeProbeMotion) {
  Fixture f;ASSERT_TRUE(f.arm().empty());EXPECT_TRUE(f.gate.armed());
  EXPECT_TRUE(f.envelope.navigation_allowed);EXPECT_EQ(f.gate.acks().size(),6u);
  EXPECT_TRUE(f.gate.metrics().passed);EXPECT_GE(f.gate.metrics().samples,12u);
  EXPECT_TRUE(f.gate.navigation().empty()); // Owner evidence, not silence, supplies cold-start authority.
}
TEST(StationaryGate, Scene03RecordedEnvelopeTransitionsKeepOriginalReceipt) {
  // Exact projected fields from first_stage_evidence_extract.json /data/envelopes[0:11].
  // Source SHA256: 5fa7e6cba969009a2ecc0dac932de79405c6414157a6ac4ef3ab9867b7021aa5.
  // This replays envelope callbacks only, not a complete Hold/scene admission.
  struct Row {int64_t source,ros,steady,until;bool allowed;const char* reason;};
  const Row rows[]={
    {71432000000LL,71433000000LL,82906866882056LL,71579527357LL,false,"WAITING_FOR:controller,global_costmap,local_costmap,planner,policy,protection"},
    {71434000000LL,71434000000LL,82906868529011LL,71579527357LL,false,"WAITING_FOR:controller,global_costmap,local_costmap,planner,policy"},
    {71440000000LL,71441000000LL,82906880844205LL,71579527357LL,false,"WAITING_FOR:controller,global_costmap,local_costmap,planner,policy"},
    {71454000000LL,71454000000LL,82906888485218LL,71681965180LL,false,"WAITING_FOR:controller,global_costmap,local_costmap,planner,policy"},
    {71464000000LL,71464000000LL,82906898536168LL,71681965180LL,false,"WAITING_FOR:global_costmap,local_costmap,planner,policy"},
    {71464000000LL,71465000000LL,82906899594094LL,71681965180LL,false,"WAITING_FOR:global_costmap,planner,policy"},
    {71477000000LL,71477000000LL,82906911734781LL,71681965180LL,false,"WAITING_FOR:global_costmap,policy"},
    {71477000000LL,71477000000LL,82906912208798LL,71681965180LL,false,"WAITING_FOR:policy"},
    {71500000000LL,71500000000LL,82906933823400LL,71681965180LL,false,"WAITING_FOR:policy"},
    {71504000000LL,71504000000LL,82906938080828LL,71681965180LL,false,"WAITING_FOR:policy"},
    {71504000000LL,71505000000LL,82906939191541LL,71681965180LL,true,"READY_FIXED"}
  };
  Envelope e;e.coordinator_session_id="a7bd2d14-98bb-481f-90b8-0e9a0f033481";
  e.hold_id="17129c5c-9572-4f2b-846d-093ae6a791f4_1_hold";e.epoch=82832289353709;
  e.request_id="mass_fixed_d76559df4d2e45258d933c33127324c5";
  e.installed_geometry_hash="9615f819027cc6c29e630b5d4b7949e8dade3e3acfab54f02911715696ae1cd1";
  e.mode=Envelope::FIXED_POSTURE;
  StationaryGate gate{"unused_for_envelope_replay",e.hold_id,e.coordinator_session_id};
  int64_t previous_source=0;Time original;
  for(const auto& row:rows) {
    if(row.source!=previous_source)original={row.ros,row.steady};
    e.header.stamp=source_time(row.source);e.valid_until=source_time(row.until);
    e.navigation_allowed=e.limits.transport_ready=row.allowed;e.reason=row.reason;
    gate.receive(e,{row.ros,row.steady});ASSERT_TRUE(gate.fault().empty())<<row.reason;
    EXPECT_EQ(gate.envelope()->at.ros,original.ros);EXPECT_EQ(gate.envelope()->at.steady,original.steady);
    EXPECT_EQ(gate.envelope()->value.navigation_allowed,row.allowed);previous_source=row.source;
  }
  EXPECT_EQ(gate.envelope()->value.reason,"READY_FIXED");EXPECT_FALSE(gate.armed());
}
TEST(StationaryGate, MissingAckRevokedEnvelopeAndWrongHoldNeverArm) {
  Fixture missing;missing.refresh("protection");
  EXPECT_EQ(missing.gate.arm(missing.envelope,missing.geometry,missing.now),"ACK_MISSING:protection");
  Fixture revoked;revoked.envelope.navigation_allowed=false;revoked.refresh();
  EXPECT_EQ(revoked.gate.arm(revoked.envelope,revoked.geometry,revoked.now),"FIXED_ENVELOPE_NOT_READY");
  Fixture owner;owner.hold.owner_id="other";owner.refresh();
  EXPECT_EQ(owner.gate.arm(owner.envelope,owner.geometry,owner.now),"TYPED_HOLD_MISMATCH");
  EXPECT_FALSE(missing.gate.armed());EXPECT_FALSE(revoked.gate.armed());EXPECT_FALSE(owner.gate.armed());
}
TEST(StationaryGate, BadThenGoodCallbacksLatchFirstFailure) {
  const std::vector<std::pair<std::string,std::function<void(Fixture&)>>> cases={
    {"NONZERO_CHASSIS_COMMAND",[](auto& f){f.command.linear.x=.1;f.gate.receive(f.command,f.now);f.command.linear.x=0;f.gate.receive(f.command,f.now);}},
    {"OWNER_TYPED_HOLD_CHANGED",[](auto& f){f.hold.header.stamp=source_time(f.now.ros);f.hold.hold_confirmed=false;f.gate.receive(f.hold,f.now);f.hold.hold_confirmed=true;f.gate.receive(f.hold,f.now);}},
    {"CURRENT_ACK_REVOKED",[](auto& f){auto a=f.ack("controller");a.applied=false;f.gate.receive(a,f.now);a.applied=true;f.gate.receive(a,f.now);}},
    {"OWNER_FIXED_ENVELOPE_CHANGED",[](auto& f){f.envelope.navigation_allowed=false;f.gate.receive(f.envelope,f.now);f.envelope.navigation_allowed=true;f.gate.receive(f.envelope,f.now);}},
    {"OWNER_GEOMETRY_CHANGED",[](auto& f){f.geometry.header.stamp=source_time(f.now.ros);f.geometry.complete=false;f.gate.receive(f.geometry,f.now);f.geometry.complete=true;f.gate.receive(f.geometry,f.now);}}
  };
  for(const auto& c:cases) {SCOPED_TRACE(c.first);Fixture f;ASSERT_TRUE(f.arm().empty());f.advance();c.second(f);
    EXPECT_EQ(f.gate.check(f.envelope,f.geometry,f.now),c.first);}
}
TEST(StationaryGate, FrozenContextCannotMigrate) {
  const std::vector<std::function<void(Envelope&)>> changes={
    [](auto& e){++e.epoch;},[](auto& e){++e.clock_epoch;},[](auto& e){e.hold_id="new";},
    [](auto& e){e.coordinator_session_id="new";},[](auto& e){e.model_revision="new";},
    [](auto& e){e.attachment_revision="new";},[](auto& e){e.installed_geometry_hash="new";},
    [](auto& e){e.request_id="new";}
  };
  for(size_t i=0;i<changes.size();++i) {SCOPED_TRACE(i);Fixture f;ASSERT_TRUE(f.arm().empty());f.advance();
    changes[i](f.envelope);f.gate.receive(f.envelope,f.now);EXPECT_EQ(f.gate.fault(),"OWNER_FIXED_ENVELOPE_CHANGED");}
}
TEST(StationaryGate, ActiveNavigationCannotBeErasedByTerminalBeforePolling) {
  Fixture f;ASSERT_TRUE(f.arm().empty());f.advance();Navigation n;n.stamp=source_time(f.now.ros);
  n.task_id="navigation";n.source="arbiter";n.sequence=1;n.state="ACCEPTED";f.gate.receive(n,f.now);
  n.sequence=2;n.state="SUCCEEDED";f.gate.receive(n,f.now);
  EXPECT_EQ(f.gate.check(f.envelope,f.geometry,f.now),"NAVIGATION_TASK_ACTIVE");
  EXPECT_EQ(f.gate.navigation().at("navigation").state,"ACCEPTED");
}
TEST(StationaryGate, ExistingObservedTaskBlocksSettlingUntilTerminal) {
  Fixture f;f.refresh();Navigation n;n.stamp=source_time(f.now.ros);n.task_id="old";n.sequence=1;n.state="EXECUTING";
  f.gate.receive(n,f.now);EXPECT_EQ(f.gate.arm(f.envelope,f.geometry,f.now),"NAVIGATION_TASK_ACTIVE");
  f.advance();n.stamp=source_time(f.now.ros);n.sequence=2;n.state="CANCELED";f.gate.receive(n,f.now);
  EXPECT_TRUE(f.arm().empty());
}
TEST(StationaryGate, DuplicateSourcesNeverRenewAnchorsOrAddSamples) {
  Fixture f;ASSERT_TRUE(f.arm().empty());const auto at=f.now;const auto count=f.gate.metrics().samples;
  const auto hold=f.hold;const auto odom=f.odom;const auto ack=f.ack("controller");
  const auto envelope_deadline=f.gate.envelope_deadline();
  f.now.steady+=100*ms;f.gate.receive(hold,f.now);f.gate.receive(odom,f.now);f.gate.receive(ack,f.now);
  f.envelope.reason="READY_FIXED";f.envelope.valid_until=source_time(f.now.ros+1000*ms);
  f.gate.receive(f.envelope,f.now);f.gate.receive(f.geometry,f.now);
  EXPECT_EQ(f.gate.envelope()->at.steady,at.steady);EXPECT_EQ(f.gate.geometry()->at.steady,at.steady);
  EXPECT_EQ(f.gate.envelope_deadline(),envelope_deadline);
  EXPECT_EQ(f.gate.hold()->at.steady,at.steady);EXPECT_EQ(f.gate.odom()->at.steady,at.steady);
  EXPECT_EQ(f.gate.acks().at("controller").at.steady,at.steady);
  EXPECT_EQ(f.gate.metrics().samples,count);
  f.gate.receive(f.command,f.now);EXPECT_EQ(f.gate.command()->at.steady,f.now.steady);
  f.now.steady=at.steady+300*ms;
  f.gate.receive(hold,f.now);
  EXPECT_EQ(f.gate.check(f.envelope,f.geometry,f.now),"TYPED_HOLD_EXPIRED");
}
TEST(StationaryGate, SameStampConflictAndRollbackAreExplicit) {
  Fixture conflict;ASSERT_TRUE(conflict.arm().empty());conflict.hold.owner_id="other";
  conflict.gate.receive(conflict.hold,conflict.now);EXPECT_EQ(conflict.gate.fault(),"HOLD_SOURCE_CONFLICT");
  Fixture backward;ASSERT_TRUE(backward.arm().empty());auto a=backward.ack("controller");a.header.stamp=source_time(backward.now.ros-1);
  backward.gate.receive(a,backward.now);EXPECT_EQ(backward.gate.fault(),"ACK_SOURCE_ROLLBACK");
  Fixture ros;ASSERT_TRUE(ros.arm().empty());--ros.now.ros;
  EXPECT_EQ(ros.gate.check(ros.envelope,ros.geometry,ros.now),"STATIONARY_CLOCK_ROLLBACK");
}
TEST(StationaryGate, LateDeliveryConsumesLeaseAndFutureInputFails) {
  const Time received{1200*ms,8000*ms};
  EXPECT_TRUE(fresh(1000*ms,1300*ms,received,{1200*ms,8099*ms}));
  EXPECT_FALSE(fresh(1000*ms,1300*ms,received,{1200*ms,8100*ms}));
  Fixture future;future.hold.header.stamp=source_time(future.now.ros+1);future.gate.receive(future.hold,future.now);
  EXPECT_EQ(future.gate.fault(),"STATIONARY_SOURCE_INVALID");
}
TEST(StationaryGate, RenewalCannotHideAnExpiredLeaseBetweenPolls) {
  Fixture f;ASSERT_TRUE(f.arm().empty());f.advance(300*ms);f.hold.header.stamp=source_time(f.now.ros);
  f.gate.receive(f.hold,f.now);EXPECT_EQ(f.gate.fault(),"TYPED_HOLD_EXPIRED");
}
TEST(StationaryGate, GeometryAndEnvelopeRenewalCannotEraseExpiry) {
  for(bool geometry:{true,false})for(bool paused:{true,false}) {
    SCOPED_TRACE(std::to_string(geometry)+":"+std::to_string(paused));Fixture f;ASSERT_TRUE(f.arm().empty());
    if(!paused)f.now.ros+=301*ms;
    f.now.steady+=301*ms;
    if(geometry) {
      f.geometry.header.stamp=source_time(f.now.ros);f.geometry.valid_until=source_time(f.now.ros+300*ms);
      f.gate.receive(f.geometry,f.now);EXPECT_EQ(f.gate.fault(),"OWNER_GEOMETRY_EXPIRED");
    } else {
      f.envelope.header.stamp=source_time(f.now.ros);f.envelope.valid_until=source_time(f.now.ros+300*ms);
      f.gate.receive(f.envelope,f.now);EXPECT_EQ(f.gate.fault(),"OWNER_FIXED_ENVELOPE_EXPIRED");
    }
  }
}
TEST(StationaryGate, GeometryAndEnvelopeExpiredThenGoodCannotRecover) {
  for(bool geometry:{true,false}) {
    Fixture f;ASSERT_TRUE(f.arm().empty());f.advance();
    if(geometry) {
      f.geometry.header.stamp=source_time(f.now.ros);f.geometry.valid_until=source_time(f.now.ros);
      f.gate.receive(f.geometry,f.now);f.geometry.valid_until=source_time(f.now.ros+300*ms);f.gate.receive(f.geometry,f.now);
      EXPECT_EQ(f.gate.fault(),"OWNER_GEOMETRY_CHANGED");
    } else {
      f.envelope.header.stamp=source_time(f.now.ros);f.envelope.valid_until=source_time(f.now.ros);
      f.gate.receive(f.envelope,f.now);f.envelope.valid_until=source_time(f.now.ros+300*ms);f.gate.receive(f.envelope,f.now);
      EXPECT_EQ(f.gate.fault(),"OWNER_FIXED_ENVELOPE_CHANGED");
    }
  }
}
TEST(StationaryGate, CommandLossIsNotEvidenceOfZeroCommand) {
  Fixture f;ASSERT_TRUE(f.arm().empty());
  for(int i=0;i<6;++i) {
    f.advance();f.hold.header.stamp=source_time(f.now.ros);f.gate.receive(f.hold,f.now);
    for(const auto& c:StationaryGate::consumers)f.gate.receive(f.ack(c),f.now);
    f.odom.header.stamp=source_time(f.now.ros);f.gate.receive(f.odom,f.now);
  }
  EXPECT_EQ(f.gate.fault(),"COMMAND_STALE_AT_ODOM");
}
TEST(StationaryGate, FixedReferenceDetectsSlowAccumulationBeyondRollingWindow) {
  Fixture f;ASSERT_TRUE(f.arm().empty());const auto reference=f.gate.reference()->pose.pose.position.x;
  for(int i=1;i<=102&&f.gate.fault().empty();++i) {
    f.advance();f.odom.pose.pose.position.x=reference+i*.0002;f.refresh();
    if(i<100){EXPECT_TRUE(f.gate.check(f.envelope,f.geometry,f.now).empty());}
  }
  EXPECT_EQ(f.gate.fault(),"BASE_REFERENCE_MOVED");
  EXPECT_LE(f.gate.metrics().drift,.005);EXPECT_LE(f.gate.metrics().fitted_speed,.01);
  EXPECT_EQ(f.gate.reference()->pose.pose.position.x,reference);
}
TEST(StationaryGate, QuaternionSignDoesNotChangePoseAndFrameChangeFails) {
  Fixture f;ASSERT_TRUE(f.arm().empty());f.advance();f.odom.pose.pose.orientation.w=-1;f.refresh();
  EXPECT_TRUE(f.gate.check(f.envelope,f.geometry,f.now).empty());
  f.advance();f.odom.header.stamp=source_time(f.now.ros);f.odom.child_frame_id="other";f.gate.receive(f.odom,f.now);
  EXPECT_EQ(f.gate.fault(),"ODOM_FRAME_CHANGED");
}
TEST(StationaryGate, InvalidDeviceNumbersAreRejected) {
  Fixture nan;nan.odom.pose.pose.position.x=std::numeric_limits<double>::quiet_NaN();nan.refresh();EXPECT_EQ(nan.gate.fault(),"ODOM_INVALID");
  Fixture quaternion;quaternion.odom.pose.pose.orientation.w=2;quaternion.refresh();EXPECT_EQ(quaternion.gate.fault(),"ODOM_INVALID");
  Fixture infinity;infinity.command.angular.z=std::numeric_limits<double>::infinity();infinity.refresh();EXPECT_EQ(infinity.gate.fault(),"COMMAND_INVALID");
}
TEST(MeasuredStop, NumericThresholdsBelowAtAndAbove) {
  const std::vector<std::pair<double,std::function<void(StopSample&,double)>>> limits={
    {.01,[](auto& s,double v){s.vx=v;}},{.02,[](auto& s,double v){s.wz=v;}},
    {1e-6,[](auto& s,double v){s.command=v;}},{.005,[](auto& s,double v){s.x=v;}},
    {.01,[](auto& s,double v){s.yaw=v;}}
  };
  for(size_t i=0;i<limits.size();++i)for(int side:{-1,0,1}) {
    SCOPED_TRACE(std::to_string(i)+":"+std::to_string(side));const auto& c=limits[i];auto rows=stopped_rows();
    const double v=side==0?c.first:std::nextafter(c.first,side<0?0.:std::numeric_limits<double>::infinity());
    c.second(rows.back(),v);EXPECT_EQ(stop(rows).passed,side<=0);
  }
}
TEST(MeasuredStop, SpanCountContinuityAndStrictAgeBoundaries) {
  auto rows=stopped_rows();ASSERT_TRUE(stop(rows).passed);
  auto short_span=rows;--short_span.back().source;EXPECT_FALSE(stop(short_span).passed);
  auto count12=rows;count12.erase(count12.begin()+1);EXPECT_TRUE(stop(count12).passed);
  auto count11=count12;count11.erase(count11.begin()+2);EXPECT_FALSE(stop(count11).passed);
  for(int64_t delta:{-1LL,0LL,1LL}) {
    auto gap=rows;gap[1].source=gap[0].source+120*ms+delta;gap[2].source=gap[1].source+1;
    EXPECT_EQ(stop(gap).passed,delta<=0);
    auto command_age=rows;command_age.back().command_received.steady-=300*ms+delta;
    EXPECT_EQ(stop(command_age).passed,delta<0);
    auto now=rows.back().received;now.ros+=300*ms+delta;now.steady+=300*ms+delta;
    EXPECT_EQ(measured_stop(rows,999*ms,now).passed,delta<0);
  }
  auto duplicate=rows;duplicate[1].source=duplicate[0].source;EXPECT_FALSE(stop(duplicate).passed);
}
TEST(MeasuredStop, FittedDriftAndYawUnwrap) {
  auto rows=stopped_rows();
  for(size_t i=0;i<rows.size();++i)rows[i].x=i<6?0:.0045;
  EXPECT_FALSE(stop(rows).passed);EXPECT_GT(stop(rows).fitted_speed,.01);
  for(auto& row:rows)row.x*=.8;
  EXPECT_TRUE(stop(rows).passed);
  rows=stopped_rows();for(auto& row:rows)row.yaw=std::acos(-1.)-.001;
  rows.back().yaw=-std::acos(-1.)+.001;
  EXPECT_TRUE(stop(rows).passed);EXPECT_NEAR(stop(rows).rotation,.002,1e-12);
}
} // namespace
