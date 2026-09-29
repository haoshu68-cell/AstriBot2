#include <gtest/gtest.h>
#include "astribot_s1_robot_geometry/attachment_geometry.hpp"
#include "astribot_s1_robot_geometry/geometry_state_core.hpp"
using namespace astribot_s1_robot_geometry;
namespace p=astribot::payload;
namespace {
constexpr int64_t T=1000000000;
p::Config config() {return {"simulation","session","inventory","ledger",{"tcp"}};}
p::State confirmed(bool loaded=true,int64_t at=T) {
  p::Observation o;o.environment="simulation";o.session_id="session";o.source_id="inventory";o.source_epoch="source_boot";
  o.sequence=o.revision=1;o.status=loaded?o.ATTACHED:o.EMPTY;o.full_inventory=true;o.transaction_id="startup";
  o.observed_at=p::stamp(at);o.valid_until=p::stamp(at+p::kLease);
  if(loaded) {
    moveit_msgs::msg::AttachedCollisionObject a;a.link_name="tcp";a.object.id="offset";a.object.header.frame_id="tcp";
    a.object.pose.orientation.w=1.;a.object.pose.position.x=.4;
    shape_msgs::msg::SolidPrimitive b;b.type=b.BOX;b.dimensions={.2,.4,.6};a.object.primitives={b};
    geometry_msgs::msg::Pose pose;pose.orientation.w=1.;pose.position.y=.3;a.object.primitive_poses={pose};o.objects={a};
  }
  p::Ledger l(config(),[](const auto &){});
  if(!l.observe(o,at,at))throw std::runtime_error("fixture observation");auto ticket=l.request(at,at);
  moveit_msgs::msg::PlanningScene s;s.robot_state.attached_collision_objects=o.objects;
  if(!ticket || !l.reconcile(*ticket,s,at,at))throw std::runtime_error("fixture scene");return l.state(at,at);
}
TEST(AttachmentGeometry, MissingConfirmationIsNotAnEmptyBody) {
  p::Consumer c(config());EXPECT_THROW(confirmedAttachments(c,T,T),std::invalid_argument);
}
TEST(AttachmentGeometry, ExplicitEmptyRetainsVersionAndSourceDeadline) {
  p::Consumer c(config());auto s=confirmed(false);ASSERT_TRUE(c.receive(s,T,T));
  const auto g=confirmedAttachments(c,T,T);EXPECT_TRUE(g.shapes.empty());EXPECT_FALSE(g.revision.empty());
  EXPECT_EQ(g.valid_until,T+p::kLease);EXPECT_EQ(g.observed_at,T);
}
TEST(AttachmentGeometry, OffsetPayloadIsConservativelyIncluded) {
  p::Consumer c(config());ASSERT_TRUE(c.receive(confirmed(),T,T));const auto g=confirmedAttachments(c,T,T);
  ASSERT_EQ(g.shapes.size(),1u);EXPECT_EQ(g.ids,std::vector<std::string>{"offset"});
  const auto &shape=g.shapes[0];EXPECT_NEAR(shape.support({1,0,0},Transform::Identity()),.5,1e-12);
  EXPECT_NEAR(shape.support({0,1,0},Transform::Identity()),.5,1e-12);
  EXPECT_NEAR(shape.support({0,0,1},Transform::Identity()),.3,1e-12);
}
TEST(AttachmentGeometry, ExpiredOrRevokedInputCannotFinishAsyncGeometry) {
  p::Consumer c(config());auto s=confirmed();ASSERT_TRUE(c.receive(s,T,T));auto a=confirmedAttachments(c,T,T);
  const GeometryWorkContext captured{c.generation(),1,a.revision,0};
  auto bad=s;bad.confirmed=false;ASSERT_FALSE(c.receive(bad,T+1,T+1));
  EXPECT_THROW(confirmedAttachments(c,T+1,T+1),std::invalid_argument);
  auto fresh=confirmed(true,T+2);ASSERT_TRUE(c.receive(fresh,T+2,T+2));auto b=confirmedAttachments(c,T+2,T+2);
  const GeometryWorkContext current{c.generation(),1,b.revision,0};
  EXPECT_THROW(validateGeometryCompletion(captured,current,true,T,T+p::kLease,T+2,1.,2.,T,b.revision,0.),std::invalid_argument);
  EXPECT_THROW(confirmedAttachments(c,T+2,T+2+p::kLease),std::invalid_argument);
}

TEST(AttachmentGeometry, CompletionClampsToTightenedCurrentAttachmentLease) {
  p::Consumer c(config());auto s=confirmed();ASSERT_TRUE(c.receive(s,T,T));
  const auto original=T+300000000;
  s.valid_until=p::stamp(T+100000000);s.published_at=p::stamp(T+10000000);
  ASSERT_TRUE(c.receive(s,T+10000000,T+10000000));
  EXPECT_EQ(attachmentCompletionDeadline(c,original,T+50000000,T+50000000),T+100000000);
  EXPECT_THROW(attachmentCompletionDeadline(c,original,T+100000000,T+100000000),std::invalid_argument);
}
}
