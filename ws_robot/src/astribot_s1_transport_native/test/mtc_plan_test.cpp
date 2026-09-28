#include <gtest/gtest.h>
#include "astribot_s1_transport_native/mtc_plan.hpp"
#include <limits>
#include <cmath>
using namespace astribot::transport;
namespace {
using Stage=astribot_transport_msgs::msg::ManipulationStage;
std::vector<Stage> stages(){
 std::vector<Stage> result;
 for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"PREGRASP","ARM"},{"GRASP_APPROACH","ARM"},{"GRASP_CONFIRM","GRIPPER"},{"ATTACH_CONFIRM","ATTACH"},{"LIFT","ARM"},{"TRANSPORT_POSTURE","ARM"}}){
  Stage s;s.stage_id=pair.first;s.kind=pair.second;s.expected_start.joint_state.name={"arm","hand"};s.expected_start.joint_state.position={0.,.2};result.push_back(s);
 }
 return result;
}
sensor_msgs::msg::JointState actual(){sensor_msgs::msg::JointState s;s.name={"hand","arm"};s.position={.2,0.};return s;}
}
TEST(MtcPlan, OrderedSingleUseBoundToPlannerContext){
 auto s=stages();MtcPlan p("PICK",s,"task-context","task-context",100,{"arm","hand"});
 EXPECT_EQ(p.check("PREGRASP","ARM",100,actual(),{}).stage_id,"PREGRASP");
 EXPECT_THROW(p.check("GRASP_APPROACH","ARM",100,actual(),{}),std::runtime_error);
 p.acknowledge();EXPECT_EQ(p.check("GRASP_APPROACH","ARM",100,actual(),{}).stage_id,"GRASP_APPROACH");
 EXPECT_THROW(MtcPlan("PICK",s,"task-context","other",100,{"arm","hand"}),std::runtime_error);
}
TEST(MtcPlan, RejectsMissingReorderedOrWrongKindStages){
 auto s=stages();s.pop_back();EXPECT_THROW(MtcPlan("PICK",s,"c","c",0,{"arm","hand"}),std::runtime_error);
 s=stages();std::swap(s[0],s[1]);EXPECT_THROW(MtcPlan("PICK",s,"c","c",0,{"arm","hand"}),std::runtime_error);
 s=stages();s[2].kind="ARM";EXPECT_THROW(MtcPlan("PICK",s,"c","c",0,{"arm","hand"}),std::runtime_error);
}
TEST(MtcPlan, MeasuredStartToleranceAndOrder){
 MtcPlan p("PICK",stages(),"c","c",0,{"arm","hand"});auto a=actual();
 a.position[1]=.025;EXPECT_NO_THROW(p.check("PREGRASP","ARM",0,a,{}));
 a.position[1]=std::nextafter(.025,1.);EXPECT_THROW(p.check("PREGRASP","ARM",0,a,{}),std::runtime_error);
 a.position[1]=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(p.check("PREGRASP","ARM",0,a,{}),std::runtime_error);
 a=actual();a.name.pop_back();a.position.pop_back();EXPECT_THROW(p.check("PREGRASP","ARM",0,a,{}),std::runtime_error);
}
TEST(MtcPlan, RequiresEveryOwnedJointInExpectedStart){
 auto s=stages();s[0].expected_start.joint_state.name={"arm"};s[0].expected_start.joint_state.position={0.};
 MtcPlan p("PICK",s,"c","c",0,{"arm","hand"});EXPECT_THROW(p.check("PREGRASP","ARM",0,actual(),{}),std::runtime_error);
}
TEST(MtcPlan, OriginalPlanExpiryInclusiveAndClockRewindRejected){
 MtcPlan p("PICK",stages(),"c","c",100,{"arm","hand"});
 EXPECT_THROW(p.check("PREGRASP","ARM",99,actual(),{}),std::runtime_error);
 EXPECT_NO_THROW(p.check("PREGRASP","ARM",120000000100LL,actual(),{}));
 EXPECT_THROW(p.check("PREGRASP","ARM",120000000101LL,actual(),{}),std::runtime_error);
}
TEST(MtcPlan, AttachmentStateAndRetreatAlternativeAreBound){
 auto s=stages();moveit_msgs::msg::AttachedCollisionObject body;body.object.id="box";s[0].expected_start.attached_collision_objects={body};
 MtcPlan p("PICK",s,"c","c",0,{"arm","hand"});
 EXPECT_THROW(p.check("PREGRASP","ARM",0,actual(),{}),std::runtime_error);
 EXPECT_NO_THROW(p.check("PREGRASP","ARM",0,actual(),{"box"}));
 EXPECT_THROW(p.check("PREGRASP","ARM",0,actual(),{"box","other_hand_object"}),std::runtime_error);
 std::vector<std::pair<std::string,std::string>> order={{"PREPLACE","ARM"},{"PLACE_APPROACH","ARM"},{"RELEASE","GRIPPER"},{"DETACH_CONFIRM","DETACH"},{"RETREAT:2","ARM"},{"STOW","ARM"}};
 s=stages();for(size_t i=0;i<s.size();++i){s[i].stage_id=order[i].first;s[i].kind=order[i].second;}
 MtcPlan place("PLACE",s,"p","p",0,{"arm","hand"});
 for(const auto& pair:order){EXPECT_NO_THROW(place.check(pair.first.substr(0,pair.first.find(':')),pair.second,0,actual(),{}));place.acknowledge();}
 EXPECT_TRUE(place.complete());EXPECT_THROW(place.check("STOW","ARM",0,actual(),{}),std::runtime_error);
}

namespace {
class MtcPayloadSuffix : public ::testing::Test {
protected:
 static constexpr int64_t created=100;
 std::vector<Stage> original;

 void SetUp() override {
  original=stages();
  moveit_msgs::msg::AttachedCollisionObject body;
  body.object.id="box";body.link_name="arm_tcp";body.object.header.frame_id=body.link_name;
  body.touch_links={body.link_name};body.object.pose.orientation.w=1.;
  body.object.primitives.resize(1);body.object.primitives[0].type=body.object.primitives[0].BOX;
  body.object.primitives[0].dimensions={.06,.06,.06};
  body.object.primitive_poses.resize(1);body.object.primitive_poses[0].orientation.w=1.;
  for(size_t i=4;i<6;++i) {
   original[i].expected_start.attached_collision_objects={body};
   auto& path=original[i].trajectory.joint_trajectory;path.joint_names={"arm"};
   for(int point=0;point<3;++point) {
    trajectory_msgs::msg::JointTrajectoryPoint sample;
    sample.positions={i==4?.2*point:.4+.3*point};
    sample.velocities={0.};sample.accelerations={0.};sample.time_from_start.sec=point;
    path.points.push_back(sample);
   }
  }
  original[5].expected_start.joint_state.position[0]=.4;
 }

 MtcPlan atPayload() const {
  MtcPlan plan("PICK",original,"c","c",created,{"arm","hand"});
  for(int i=0;i<3;++i)plan.acknowledge();
  return plan;
 }

 std::vector<Stage> confirmedSuffix(bool replanned) const {
  std::vector<Stage> suffix{original[4],original[5]};
  for(auto& stage:suffix)
   stage.expected_start.attached_collision_objects[0].object.primitives[0].dimensions[0]=.065;
  if(replanned)suffix[1].trajectory.joint_trajectory.points[1].positions[0]=.6;
  return suffix;
 }

 void expectRejectedUnchanged(const std::vector<Stage>& suffix,bool replanned=true,
                              int64_t steady=created+1) const {
  auto plan=atPayload();auto before=plan;
  EXPECT_THROW(plan.adoptPayloadSuffix(suffix,replanned,steady),std::runtime_error);
  ASSERT_EQ(plan.index(),3u);
  while(!before.complete()) {
   ASSERT_FALSE(plan.complete());
   EXPECT_EQ(plan.current(),before.current());
   plan.acknowledge();before.acknowledge();
  }
  EXPECT_TRUE(plan.complete());
 }
};
}

TEST_F(MtcPayloadSuffix, ReplannedPickAdoptsCompleteSuffixWithoutAcknowledgingBarrier){
 auto plan=atPayload();const auto suffix=confirmedSuffix(true);
 ASSERT_NE(suffix[1].trajectory,original[5].trajectory);
 ASSERT_EQ(suffix[0].trajectory,original[4].trajectory);
 ASSERT_NO_THROW(plan.adoptPayloadSuffix(suffix,true,created+1));
 EXPECT_EQ(plan.index(),3u);EXPECT_EQ(plan.current(),original[3]);
 plan.acknowledge();EXPECT_EQ(plan.current(),suffix[0]);
 plan.acknowledge();EXPECT_EQ(plan.current(),suffix[1]);
}

TEST_F(MtcPayloadSuffix, AdoptionRetainsOriginalLifetimeAndRejectsRewindOrExpiredInput){
 auto plan=atPayload();const auto suffix=confirmedSuffix(true);
 ASSERT_NO_THROW(plan.adoptPayloadSuffix(suffix,true,created+119000000000LL));
 plan.acknowledge();
 EXPECT_NO_THROW(plan.check("LIFT","ARM",created+120000000000LL,actual(),{"box"}));
 EXPECT_THROW(plan.check("LIFT","ARM",created+120000000001LL,actual(),{"box"}),std::runtime_error);
 expectRejectedUnchanged(suffix,true,created-1);
 expectRejectedUnchanged(suffix,true,created+120000000001LL);
}

TEST_F(MtcPayloadSuffix, UnchangedPathsAcceptConfirmedAttachmentAndRejectHiddenReplanning){
 auto plan=atPayload();const auto suffix=confirmedSuffix(false);
 ASSERT_NO_THROW(plan.adoptPayloadSuffix(suffix,false,created+1));
 EXPECT_EQ(plan.index(),3u);
 plan.acknowledge();EXPECT_EQ(plan.current(),suffix[0]);
 plan.acknowledge();EXPECT_EQ(plan.current(),suffix[1]);
 expectRejectedUnchanged(confirmedSuffix(true),false);
}

TEST_F(MtcPayloadSuffix, RejectsWrongStageIdentityKindOrSuffixCountAtomically){
 auto suffix=confirmedSuffix(true);
 suffix[1].stage_id+=":other";expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);suffix[1].kind="GRIPPER";expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);suffix.pop_back();expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);suffix.push_back(original[5]);expectRejectedUnchanged(suffix);
}

TEST_F(MtcPayloadSuffix, RejectsChangedExpectedJointStartOrAttachmentIdentityAtomically){
 auto suffix=confirmedSuffix(true);
 suffix[1].expected_start.joint_state.position[0]+=.001;expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);suffix[1].expected_start.joint_state.name[0]="other_arm";
 expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);suffix[1].expected_start.multi_dof_joint_state.joint_names={"floating_base"};
 expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);suffix[1].expected_start.attached_collision_objects[0].object.id="other_box";
 expectRejectedUnchanged(suffix);
}

TEST_F(MtcPayloadSuffix, RejectsLiftTrajectoryChangeEvenWhenTransportWasReplanned){
 auto suffix=confirmedSuffix(true);
 suffix[0].trajectory.joint_trajectory.points[1].positions[0]+=.001;
 expectRejectedUnchanged(suffix);
}

TEST_F(MtcPayloadSuffix, ReplanningCannotBeAppliedToPlaceOperation){
 auto place=original;
 const std::vector<std::pair<std::string,std::string>> order={{"PREPLACE","ARM"},{"PLACE_APPROACH","ARM"},
  {"RELEASE","GRIPPER"},{"DETACH_CONFIRM","DETACH"},{"RETREAT:2","ARM"},{"STOW","ARM"}};
 for(size_t i=0;i<place.size();++i){place[i].stage_id=order[i].first;place[i].kind=order[i].second;}
 place[4].expected_start.attached_collision_objects.clear();place[5].expected_start.attached_collision_objects.clear();
 MtcPlan plan("PLACE",place,"c","c",created,{"arm","hand"});
 for(int i=0;i<3;++i)plan.acknowledge();
 const std::vector<Stage> suffix{place[4],place[5]};
 EXPECT_THROW(plan.adoptPayloadSuffix(suffix,true,created+1),std::runtime_error);
 ASSERT_EQ(plan.index(),3u);EXPECT_EQ(plan.current(),place[3]);
 // The same PLACE suffix remains admissible when no path is replanned.
 ASSERT_NO_THROW(plan.adoptPayloadSuffix(suffix,false,created+1));
 EXPECT_EQ(plan.index(),3u);plan.acknowledge();EXPECT_EQ(plan.current(),place[4]);
 plan.acknowledge();EXPECT_EQ(plan.current(),place[5]);
}

TEST_F(MtcPayloadSuffix, RejectsNonIncreasingTrajectoryTimeAtomically){
 auto suffix=confirmedSuffix(true);
 suffix[1].trajectory.joint_trajectory.points[1].time_from_start=
  suffix[1].trajectory.joint_trajectory.points[0].time_from_start;
 expectRejectedUnchanged(suffix);
}

TEST_F(MtcPayloadSuffix, RejectsStartOrEndBoundaryDriftAtomically){
 auto suffix=confirmedSuffix(true);
 suffix[1].trajectory.joint_trajectory.points.front().positions[0]+=.000001;
 expectRejectedUnchanged(suffix);
 suffix=confirmedSuffix(true);
 suffix[1].trajectory.joint_trajectory.points.back().positions[0]+=.001;
 expectRejectedUnchanged(suffix);
}
