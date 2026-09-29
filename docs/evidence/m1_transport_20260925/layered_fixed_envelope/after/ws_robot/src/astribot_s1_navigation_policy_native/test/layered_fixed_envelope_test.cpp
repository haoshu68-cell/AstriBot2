#include <gtest/gtest.h>
#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include "astribot_s1_robot_geometry/layered_envelope.hpp"

namespace nav=astribot::navigation;
namespace geom=astribot_s1_robot_geometry;
namespace {
constexpr int64_t T=10000000000LL;
geometry_msgs::msg::Polygon square(float r) {
  geometry_msgs::msg::Polygon result;
  for(auto xy:geom::Polygon2{{-r,-r},{r,-r},{r,r},{-r,r}}) {
    geometry_msgs::msg::Point32 p;p.x=xy[0];p.y=xy[1];result.points.push_back(p);
  }
  return result;
}
class LayeredFixedEnvelope:public ::testing::Test {
protected:
  nlohmann::json profile=nav::load_policy_profile(PROFILE_FILE,true);
  nav::FixedEnvelopeCore core{profile,"layered-test",1};
  nav::FixedEnvelopeCore::State s;
  nav::FixedEnvelopeCore::Hold h;
  nav::FixedEnvelopeCore::Request request;
  void SetUp() override {
    s.header.frame_id=profile.at("base_frame");s.header.stamp=nav::fixed_stamp(T);s.valid_until=nav::fixed_stamp(T+300000000);
    s.source_id="measured-model";s.sequence=1;s.model_revision="model";s.attachment_revision="empty";
    s.complete=s.attachment_state_confirmed=true;s.joints.name={"arm"};s.joints.position={0.};
    s.joint_source_stamps={s.header.stamp};s.joint_position_error_bounds={.003};
    s.physical_footprint=square(.3F);s.reserved_footprint=square(.5F);s.height_m=1.7;
    s.height_profile_revision=geom::sha256("unit-profile");s.ground_in_base_m=-.095;
    const std::vector<double> edges{-.045,.155,.585,1.085,1.535,2.205};
    for(std::size_t i=1;i<edges.size();++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice layer;layer.z_min_m=edges[i-1];layer.z_max_m=edges[i];
      if(i!=3)layer.footprint=square(.4F);
      s.height_slices.push_back(layer);
    }
    h.header.stamp=s.header.stamp;h.owner_id="task";h.hold_id="measured-hold";h.attachment_revision=s.attachment_revision;h.hold_confirmed=true;h.lease_s=.3;
    request.request_id="fixed-request";request.hold_id=h.hold_id;request.geometry_sequence=s.sequence;
    request.limits.frame_id=s.header.frame_id;request.limits.posture_id="fixed";request.limits.lease_s=.3;
#define COPY(field) request.limits.field=profile.at(#field).get<double>()
    COPY(half_length_m);COPY(half_width_m);COPY(height_m);COPY(payload_mass_kg);COPY(max_speed_m_s);
    COPY(max_angular_speed_rad_s);COPY(max_acceleration_m_s2);COPY(brake_deceleration_m_s2);
#undef COPY
  }
  void propose() {core.state(s);core.hold(h);core.propose(request,T,true);}
  void ackAll(const std::string &hash) {
    nav::FixedEnvelopeCore::Ack ack;ack.header.stamp=nav::fixed_stamp(T);ack.coordinator_session_id=core.session();
    ack.envelope_epoch=core.epoch();ack.installed_geometry_hash=hash;ack.applied=true;
    for(const auto *id:{"global_costmap","local_costmap","planner","controller","policy"}) {ack.consumer_id=id;core.acknowledge(ack,T);}
  }
};
TEST_F(LayeredFixedEnvelope, ReferenceLayersInflateExactlyOnceAndPreserveEmptyLayer) {
  propose();const auto before=*core.output();
  EXPECT_EQ(before.reference_state_sequence,s.sequence);EXPECT_EQ(before.height_profile_revision,s.height_profile_revision);
  EXPECT_EQ(before.ground_in_base_m,s.ground_in_base_m);ASSERT_EQ(before.height_slices.size(),5u);
  for(std::size_t i=0;i<5;++i) {
    EXPECT_EQ(before.height_slices[i].z_min_m,s.height_slices[i].z_min_m);
    EXPECT_EQ(before.height_slices[i].z_max_m,s.height_slices[i].z_max_m);
    if(i==2)EXPECT_TRUE(before.height_slices[i].footprint.points.empty());
    else EXPECT_EQ(geom::envelopePolygonPoints(before.height_slices[i].footprint),
      geom::serializedPolygon(geom::inflatePolygon(geom::envelopePolygonPoints(s.height_slices[i].footprint),before.clearance_m)));
  }
  EXPECT_NO_THROW(geom::validateLayeredEnvelope(before));
  for(int i=0;i<20;++i)core.tick(T+i);
  EXPECT_EQ(core.output()->height_slices,before.height_slices);
  EXPECT_EQ(core.output()->installed_geometry_hash,before.installed_geometry_hash);
  EXPECT_EQ(core.output()->height_geometry_hash,before.height_geometry_hash);
}
TEST_F(LayeredFixedEnvelope, MissingLayersRejectAlignmentWithoutChangingTraditionalAdmission) {
  s.height_slices.clear();core.state(s);core.hold(h);
  EXPECT_NO_THROW(core.propose(request,T,true));ASSERT_TRUE(core.output());
  EXPECT_THROW(geom::validateLayeredEnvelope(*core.output()),std::invalid_argument);
  ackAll(core.output()->installed_geometry_hash);EXPECT_TRUE(core.tick(T)->navigation_allowed);
}
TEST_F(LayeredFixedEnvelope, MetadataChangesDoNotAddTraditionalNavigationRevocation) {
  propose();ackAll(core.output()->installed_geometry_hash);ASSERT_TRUE(core.tick(T)->navigation_allowed);
  const auto reference=*core.output();
  for(int change=0;change<3;++change) {
    auto changed=s;changed.sequence=2+change;
    if(change==0)changed.height_profile_revision=geom::sha256("replacement-profile");
    else if(change==1)changed.ground_in_base_m-=.001;
    else {changed.height_slices[0].z_max_m+=.001;changed.height_slices[1].z_min_m+=.001;}
    core.state(changed);EXPECT_TRUE(core.fault().empty());EXPECT_TRUE(core.tick(T)->navigation_allowed);
    EXPECT_EQ(core.output()->height_profile_revision,reference.height_profile_revision);
    EXPECT_EQ(core.output()->ground_in_base_m,reference.ground_in_base_m);
    EXPECT_EQ(core.output()->height_slices,reference.height_slices);
  }
}
TEST_F(LayeredFixedEnvelope, RenewalDoesNotReplaceFixedReferenceLayerSnapshot) {
  propose();ackAll(core.output()->installed_geometry_hash);ASSERT_TRUE(core.tick(T)->navigation_allowed);
  const auto original=*core.output();
  auto missing=s;missing.sequence=2;missing.height_slices.clear();core.state(missing);
  EXPECT_TRUE(core.tick(T)->navigation_allowed);EXPECT_EQ(core.output()->height_slices,original.height_slices);
  EXPECT_EQ(core.output()->height_geometry_hash,original.height_geometry_hash);
}
TEST_F(LayeredFixedEnvelope, MetadataComesFromRequestedReferenceRatherThanLatestState) {
  core.state(s);auto changed=s;changed.sequence=2;changed.height_profile_revision=geom::sha256("other");core.state(changed);core.hold(h);
  EXPECT_NO_THROW(core.propose(request,T,true));ASSERT_TRUE(core.output());
  EXPECT_EQ(core.output()->height_profile_revision,s.height_profile_revision);
  EXPECT_EQ(core.output()->reference_state_sequence,s.sequence);EXPECT_EQ(core.output()->source_state_sequence,changed.sequence);
}
TEST_F(LayeredFixedEnvelope, SmallValidHoldDriftDoesNotRequireReservedInsideReserved) {
  propose();ackAll(core.output()->installed_geometry_hash);ASSERT_TRUE(core.tick(T)->navigation_allowed);
  const auto installed=core.output()->height_slices;
  ++s.sequence;s.joints.position[0]=.001;s.reserved_footprint=square(.501F);
  for(auto &layer:s.height_slices)if(!layer.footprint.points.empty())layer.footprint=square(.401F);
  core.state(s);EXPECT_TRUE(core.tick(T)->navigation_allowed);EXPECT_EQ(core.output()->height_slices,installed);
}
TEST_F(LayeredFixedEnvelope, LayerOnlyChangeChangesAlignmentHashAndPreservesTraditionalHash) {
  propose();const auto old_hash=core.output()->installed_geometry_hash;
  const auto old_height_hash=core.output()->height_geometry_hash;const auto aggregate=core.output()->installed_footprint;
  ++s.sequence;s.height_slices[0].footprint=square(.41F);request.geometry_sequence=s.sequence;request.request_id="second";
  core.state(s);core.propose(request,T,true);
  EXPECT_EQ(core.output()->installed_footprint,aggregate);EXPECT_EQ(core.output()->installed_geometry_hash,old_hash);
  EXPECT_NE(core.output()->height_geometry_hash,old_height_hash);
  ackAll(core.output()->installed_geometry_hash);EXPECT_TRUE(core.tick(T)->navigation_allowed);
}
TEST_F(LayeredFixedEnvelope, HeightOutsideProfileRejectsAlignmentWithoutChangingTraditionalAdmission) {
  s.height_m=std::nextafter(s.height_slices.back().z_max_m,INFINITY);core.state(s);core.hold(h);
  EXPECT_NO_THROW(core.propose(request,T,true));ASSERT_TRUE(core.output());
  EXPECT_THROW(geom::validateLayeredEnvelope(*core.output()),std::invalid_argument);
  ackAll(core.output()->installed_geometry_hash);EXPECT_TRUE(core.tick(T)->navigation_allowed);
}
} // namespace
