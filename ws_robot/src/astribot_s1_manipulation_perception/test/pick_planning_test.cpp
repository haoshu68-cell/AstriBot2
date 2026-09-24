#include <gtest/gtest.h>
#include <astribot_s1_manipulation_perception/pick_planning_client.hpp>
#include <cmath>
#include "../src/pending_action.hpp"

namespace pp = astribot::perception_planning;
class PickPlanning : public ::testing::Test {
protected:
  pp::Request r;
  pp::Pose::Result pose;
  pp::Candidate candidate;
  void SetUp() override {
    r.context={"object_17","inventory_v3","torso","optical","camera_boot","projector_boot",
               "cad_a","cad_hash:visibility_hash","grasp_hash",2,7,9,1,"actual_scene_signature","",""};
    r.task_id="pick_17";r.context_id="owner_2:scene_7";r.detection_id="frame_42:0";
    r.segmentation_source="single_instance_fixture";r.visible_instances=1;
    r.object_cloud.header.frame_id="optical";r.object_cloud.header.stamp.sec=10;
    r.valid_until.sec=15;
    r.admission_deadline_steady=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    r.result_deadline_steady=r.admission_deadline_steady+std::chrono::milliseconds(4500);
    r.base_from_camera.header=r.object_cloud.header;
    r.base_from_camera.header.frame_id="astribot_torso_base";
    r.base_from_camera.child_frame_id="optical";
    auto &t=r.base_from_camera.transform;
    t.translation.x=1;t.translation.y=2;t.translation.z=3;
    t.rotation.z=std::sqrt(.5);t.rotation.w=std::sqrt(.5);
    r.model_geometry.id="object_17";r.model_geometry.header.frame_id="cad_a";
    r.model_geometry.pose.orientation.w=1;
    shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.04,.03,.06};
    r.model_geometry.primitives.push_back(box);
    geometry_msgs::msg::Pose local;local.orientation.w=1;local.position.x=1;
    r.model_geometry.primitive_poses.push_back(local);
    r.geometry_revision="cad_geometry_hash";
    r.scene.world.collision_objects.push_back(r.model_geometry);
    r.map_grasp=[](const pp::Candidate &,const pp::Pose::Result &) {
      pp::GraspMapping m;m.grasp_from_tcp.rotation.w=1;m.grasp_from_tcp.translation.x=.1;
      m.physical_object_width_m=.03;return m;
    };
    r.grasp_registration_revision="synthetic_test_only";
    r.grasp_registration_evidence="test/pick_planning_test.cpp";
    r.minimum_width_m=.01;r.maximum_width_m=.08;r.approach_m=.03;r.lift_m=.1;
    r.touch_links={"left_pad","right_pad"};
    pose.success=true;auto &o=pose.observation;o.header=r.object_cloud.header;
    o.object_id=r.context.object_instance;o.source_camera_id=r.context.camera_id;
    o.source_epoch=r.context.source_epoch;o.source_model=r.context.model_id;
    o.model_revision=r.context.pose_model_revision;o.calibration_revision=2;
    o.planning_scene_revision=7;o.envelope_epoch=9;o.valid_until=r.valid_until;
    o.position_valid=o.orientation_valid=true;
    o.pose.pose.orientation.w=1;o.pose.pose.position.z=1;
    candidate.header=candidate.grasp_pose.header=r.object_cloud.header;
    candidate.candidate_id="proposal_8";candidate.object_id="object_17";
    candidate.camera_id="torso";candidate.source_epoch="camera_boot";candidate.arm_id="left";
    candidate.model_name="graspnet_baseline_torchscript";candidate.model_revision="grasp_hash";
    candidate.calibration_revision=2;candidate.planning_scene_revision=7;candidate.envelope_epoch=9;
    candidate.valid_until=r.valid_until;candidate.geometry_valid=true;
    candidate.grasp_pose.pose.orientation.w=1;candidate.grasp_pose.pose.position.z=1;
    candidate.gripper_width_m=.04;
  }
  pp::Plan::Goal goal() {return pp::planning_goal(r,pose,candidate,r.context,11000000000LL);}
};

TEST_F(PickPlanning, BothGoalsKeepOneCaptureAndIdentity) {
  auto g=pp::inference_goals(r,r.context,10200000000LL);
  EXPECT_EQ(g.pose.header,g.grasps.header);EXPECT_EQ(g.pose.object_id,"object_17");
  EXPECT_EQ(g.pose.processing_epoch,"projector_boot");EXPECT_EQ(g.pose.model_id,"cad_a");
  EXPECT_EQ(g.grasps.arm_id,"left");EXPECT_EQ(g.grasps.valid_until,r.valid_until);
  EXPECT_EQ(g.grasps.planning_scene_revision,7u);
  EXPECT_THROW(pp::inference_goals(r,r.context,10600000000LL),std::runtime_error);
}
TEST_F(PickPlanning, RejectsChangedIdentitySourceProcessingClockAndVersions) {
  for(int field=0;field<11;++field) {
    auto c=r.context;
    switch(field) {
      case 0:c.object_instance="same_class_other_object";break;
      case 1:c.identity_revision="reassigned";break;
      case 2:c.source_epoch="reboot";break;
      case 3:c.processing_epoch="new_projector";break;
      case 4:++c.clock_epoch;break;case 5:++c.calibration_revision;break;
      case 6:++c.scene_revision;break;case 7:++c.envelope_epoch;break;
      case 8:c.pose_model_revision="replaced";break;
      case 9:c.grasp_model_revision="replaced";break;
      case 10:c.scene_signature="geometry_changed_same_revision";break;
    }
    EXPECT_THROW(pp::check_context(r,c,11000000000LL),std::runtime_error)<<field;
  }
  EXPECT_THROW(pp::check_context(r,r.context,15000000000LL),std::runtime_error);
  EXPECT_THROW(pp::check_context(r,r.context,9000000000LL),std::runtime_error);
}
TEST_F(PickPlanning, RejectsBoxAsInstanceAndAmbiguousFixture) {
  r.segmentation_source="yolo_bbox";
  EXPECT_THROW(pp::inference_goals(r,r.context,10100000000LL),std::runtime_error);
  r.segmentation_source="single_instance_fixture";r.visible_instances=2;
  EXPECT_THROW(pp::inference_goals(r,r.context,10100000000LL),std::runtime_error);
}
TEST_F(PickPlanning, TransformsNativeGraspAndCadShapeIntoBase) {
  auto g=goal();EXPECT_EQ(g.operation,"PICK");EXPECT_EQ(g.object_id,"object_17");
  EXPECT_NEAR(g.target.pose.position.x,1,1e-10);EXPECT_NEAR(g.target.pose.position.y,2.1,1e-10);
  EXPECT_NEAR(g.target.pose.position.z,4,1e-10);EXPECT_NEAR(g.pre_target.pose.position.y,2.07,1e-10);
  EXPECT_NEAR(g.exit_targets.at(0).pose.position.z,4.1,1e-10);
  const auto &object=g.scene.world.collision_objects.at(0);
  EXPECT_EQ(object.header.frame_id,"astribot_torso_base");
  EXPECT_NEAR(object.primitive_poses.at(0).position.y,3,1e-10);
  EXPECT_NEAR(object.primitive_poses.at(0).position.z,4,1e-10);
  EXPECT_DOUBLE_EQ(object.pose.orientation.w,1);
  EXPECT_EQ(g.target.header.stamp,r.object_cloud.header.stamp); // never refresh
  EXPECT_DOUBLE_EQ(g.grasp_width_m,.03); // not the network's .04 m opening
  EXPECT_LE(g.timeout_s,4.0);
}
TEST_F(PickPlanning, RequiresMeasuredTfAndExplicitRegistration) {
  r.base_from_camera.header.stamp.sec=0;EXPECT_THROW(goal(),std::runtime_error);
  r.base_from_camera.header.stamp.sec=10;r.grasp_registration_evidence.clear();
  EXPECT_THROW(goal(),std::runtime_error);
}
TEST_F(PickPlanning, RejectsWrongObjectModelFrameAndPartialPose) {
  pose.observation.object_id="object_18";EXPECT_THROW(goal(),std::runtime_error);
  pose.observation.object_id="object_17";pose.observation.orientation_valid=false;
  EXPECT_THROW(goal(),std::runtime_error);pose.observation.orientation_valid=true;
  candidate.model_revision="old";EXPECT_THROW(goal(),std::runtime_error);
  candidate.model_revision="grasp_hash";candidate.grasp_pose.header.frame_id="base";
  EXPECT_THROW(goal(),std::runtime_error);
}
TEST_F(PickPlanning, RejectsUnsupportedWidthArmAndInventedExpiry) {
  candidate.gripper_width_m=.081;EXPECT_THROW(goal(),std::runtime_error);
  candidate.gripper_width_m=.04;candidate.arm_id="right";EXPECT_THROW(goal(),std::runtime_error);
  candidate.arm_id="left";candidate.valid_until.sec=16;EXPECT_THROW(goal(),std::runtime_error);
}
TEST_F(PickPlanning, RequiresFullWorldTargetUnattached) {
  r.scene.is_diff=true;EXPECT_THROW(goal(),std::runtime_error);r.scene.is_diff=false;
  r.scene.world.collision_objects.clear();EXPECT_THROW(goal(),std::runtime_error);
}
TEST_F(PickPlanning, RequiresCompleteMtcPickIncludingLiftAndTransport) {
  auto g=goal();pp::Plan::Result p;p.success=true;p.context_id=g.context_id;
  const std::vector<std::pair<std::string,std::string>> seq={
    {"PREGRASP","ARM"},{"GRASP_APPROACH","ARM"},{"GRASP_CONFIRM","GRIPPER"},
    {"ATTACH_CONFIRM","ATTACH"},{"LIFT","ARM"},{"TRANSPORT_POSTURE","ARM"}};
  for(const auto &[id,kind]:seq) {astribot_transport_msgs::msg::ManipulationStage s;s.stage_id=id;s.kind=kind;p.stages.push_back(s);}
  EXPECT_NO_THROW(pp::check_plan(g,p));p.stages.pop_back();
  EXPECT_THROW(pp::check_plan(g,p),std::runtime_error);
  p.context_id="other";EXPECT_THROW(pp::check_plan(g,p),std::runtime_error);
}
TEST(PendingAction, ReadyUnknownResultDoesNotProveTerminal) {
  for(const auto code:{rclcpp_action::ResultCode::UNKNOWN,rclcpp_action::ResultCode::SUCCEEDED,
      rclcpp_action::ResultCode::ABORTED,rclcpp_action::ResultCode::CANCELED}) {
    pp::detail::Pending<pp::Pose> pending;pending.sent=true;
    std::promise<rclcpp_action::ClientGoalHandle<pp::Pose>::WrappedResult> promise;
    pending.result=promise.get_future().share();
    rclcpp_action::ClientGoalHandle<pp::Pose>::WrappedResult result;result.code=code;
    promise.set_value(result);ASSERT_TRUE(pending.received());
    EXPECT_EQ(pending.terminal(),code!=rclcpp_action::ResultCode::UNKNOWN);
  }
}
