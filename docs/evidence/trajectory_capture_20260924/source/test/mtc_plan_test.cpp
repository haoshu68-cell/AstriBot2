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
