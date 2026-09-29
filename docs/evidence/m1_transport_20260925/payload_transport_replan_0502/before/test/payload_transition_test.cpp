#include "astribot_s1_transport_mtc/payload_transition.hpp"
#include <astribot_s1_manipulation/external_trajectory_validator.hpp>
#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace transport = astribot_s1_transport_mtc;
using Request = astribot_transport_msgs::srv::RevalidatePayloadTransition::Request;

class PayloadTransition : public ::testing::Test {
protected:
  moveit::core::RobotModelPtr model;
  transport::PayloadTransitionBinding binding;
  std::vector<transport::CachedStage> stages;
  Request request;
  std::chrono::steady_clock::time_point created = std::chrono::steady_clock::now();

  static moveit_msgs::msg::CollisionObject box(const std::string& id, double size,
                                              double x=0., double y=0.) {
    moveit_msgs::msg::CollisionObject object;
    object.id=id;object.header.frame_id="astribot_torso_base";
    object.pose.orientation.w=1.;object.pose.position.x=x;object.pose.position.y=y;
    shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={size,size,size};
    object.primitives={shape};geometry_msgs::msg::Pose pose;pose.orientation.w=1.;
    object.primitive_poses={pose};return object;
  }

  void SetUp() override {
    const auto urdf=urdf::parseURDF(R"(<robot name="transition_test">
      <link name="astribot_torso_base"/><link name="tcp"><collision><geometry>
      <box size="0.02 0.02 0.02"/></geometry></collision></link>
      <joint name="slide" type="prismatic"><parent link="astribot_torso_base"/>
      <child link="tcp"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint></robot>)");
    ASSERT_TRUE(urdf);
    auto srdf=std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf,R"(<robot name="transition_test"><group name="slide_group">
      <joint name="slide"/></group></robot>)"));
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    auto scene=std::make_shared<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();
    ASSERT_TRUE(scene->processCollisionObjectMsg(box("payload",.05)));
    ASSERT_TRUE(scene->processCollisionObjectMsg(box("wall",.05,.2,.22)));
    scene->getPlanningSceneMsg(binding.input_scene);
    auto& map=binding.input_scene.world.octomap;
    map.header.frame_id="astribot_torso_base";map.origin.orientation.w=1.;
    octomap::OcTree tree(.005);tree.updateNode(octomap::point3d(3.,3.,3.),true);
    ASSERT_TRUE(octomap_msgs::binaryMapToMsg(tree,map.octomap));
    map.octomap.header.frame_id="astribot_torso_base";
    binding.object_id="payload";binding.operation="PICK";
    request.context_id="context";request.transaction_id="attach-1";request.start_index=4;
    request.scene=binding.input_scene;
    request.scene.world.collision_objects.erase(request.scene.world.collision_objects.begin());
    // World serialization is ordered by id: payload precedes wall.
    ASSERT_EQ(request.scene.world.collision_objects.front().id,"wall");
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.object=box("payload",.08);attached.object.header.frame_id="tcp";
    attached.link_name="tcp";attached.touch_links={"tcp"};
    request.scene.robot_state.attached_collision_objects={attached};
    auto predicted=planning_scene::PlanningScene::clone(scene);
    attached.object.primitives.front().dimensions={.05,.05,.05};
    ASSERT_TRUE(predicted->processAttachedCollisionObjectMsg(attached));
    for(const auto& id:{"PREGRASP","GRASP_APPROACH","GRASP_CONFIRM","ATTACH_CONFIRM","LIFT","TRANSPORT_POSTURE"}) {
      transport::CachedStage stage;stage.id=id;stage.scene=predicted;
      if(stages.size()>=4) {
        stage.trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(model,"slide_group");
        auto state=predicted->getCurrentState();state.setVariablePosition("slide",0.);state.update();
        stage.trajectory->addSuffixWayPoint(state,0.);
        state.setVariablePosition("slide",.4);state.update();stage.trajectory->addSuffixWayPoint(state,4.);
      }
      stages.push_back(std::move(stage));
    }
  }
  std::vector<transport::CachedStage> validate() {
    return transport::revalidatePayloadTransition(request,"context",binding,stages,created,.1,.1);
  }
  void expectRejected(const std::string& expected) {
    try {validate();FAIL()<<"accepted invalid transition";}
    catch(const std::runtime_error& error) {EXPECT_NE(std::string(error.what()).find(expected),std::string::npos)<<error.what();}
  }
};

TEST_F(PayloadTransition, AttachReplacesEveryWaypointAndKeepsPathAndStagePolicy) {
  auto scene=planning_scene::PlanningScene::clone(stages[4].scene);
  scene->getAllowedCollisionMatrixNonConst().setEntry("payload","support",true);stages[4].scene=scene;
  moveit_msgs::msg::RobotTrajectory original;stages[4].trajectory->getRobotTrajectoryMsg(original);
  auto result=validate();
  EXPECT_EQ(result[4].scene->getCurrentState().getAttachedBody("payload")->getTouchLinks(),std::set<std::string>{"tcp"});
  collision_detection::AllowedCollision::Type allowed;
  ASSERT_TRUE(result[4].scene->getAllowedCollisionMatrix().getEntry("payload","support",allowed));
  EXPECT_EQ(allowed,collision_detection::AllowedCollision::ALWAYS);
  for(size_t i=0;i<result[4].trajectory->getWayPointCount();++i) {
    auto body=result[4].trajectory->getWayPoint(i).getAttachedBody("payload");ASSERT_NE(body,nullptr);
    const auto* shape=dynamic_cast<const shapes::Box*>(body->getShapes().front().get());ASSERT_NE(shape,nullptr);
    EXPECT_DOUBLE_EQ(shape->size[0],.08);
    EXPECT_NEAR(body->getGlobalPose().translation().x(),result[4].trajectory->getWayPoint(i).getVariablePosition("slide"),1e-9);
  }
  moveit_msgs::msg::RobotTrajectory updated;result[4].trajectory->getRobotTrajectoryMsg(updated);
  EXPECT_EQ(original,updated); // The already-issued joint path and timing are unchanged.
  EXPECT_NE(result[4].trajectory.get(),stages[4].trajectory.get());
}

TEST_F(PayloadTransition, InflatedPayloadRejectsPreviouslyClearPathWithoutMutatingCache) {
  std::string reason;robot_trajectory::RobotTrajectory old(*stages[4].trajectory,true);
  ASSERT_TRUE(astribot_s1_manipulation::validateExternalTrajectory(stages[4].scene,old,reason,.1,.1))<<reason;
  request.scene.robot_state.attached_collision_objects.front().object.primitives.front().dimensions={.5,.5,.5};
  expectRejected("EXTERNAL_COLLISION");
  const auto* body=stages[4].trajectory->getWayPoint(0).getAttachedBody("payload");
  EXPECT_DOUBLE_EQ(dynamic_cast<const shapes::Box*>(body->getShapes().front().get())->size[0],.05);
  EXPECT_TRUE(binding.transaction_id.empty());
}

TEST_F(PayloadTransition, DetachRemovesStaleTrajectoryBodyAndUsesActualWorldPose) {
  binding.operation="PLACE";binding.input_scene=request.scene;
  request.scene.robot_state.attached_collision_objects.clear();
  request.scene.world.collision_objects.push_back(box("payload",.08,0.,.6));
  stages[3].id="DETACH_CONFIRM";
  auto result=validate();
  for(size_t i=4;i<result.size();++i) {
    EXPECT_FALSE(result[i].scene->getCurrentState().hasAttachedBody("payload"));
    ASSERT_TRUE(result[i].scene->getWorld()->hasObject("payload"));
    EXPECT_NEAR(result[i].scene->getWorld()->getObject("payload")->pose_.translation().y(),.6,1e-9);
    for(size_t j=0;j<result[i].trajectory->getWayPointCount();++j)
      EXPECT_FALSE(result[i].trajectory->getWayPoint(j).hasAttachedBody("payload"));
  }
}

TEST_F(PayloadTransition, RejectsStaticWorldAndOtherAttachmentChanges) {
  request.scene.world.collision_objects.front().pose.position.x+=.1;expectRejected("SCENE_CHANGED");
  request.scene=binding.input_scene;
  auto other=moveit_msgs::msg::AttachedCollisionObject();other.object=box("other",.03);
  other.object.header.frame_id="tcp";other.link_name="tcp";
  request.scene.robot_state.attached_collision_objects.push_back(other);expectRejected("SCENE_CHANGED");
}

TEST_F(PayloadTransition, RejectsContextIndexExpiryAndWrongTransition) {
  request.context_id="old";expectRejected("CONTEXT_UNKNOWN");request.context_id="context";
  request.start_index=3;expectRejected("INDEX_INVALID");request.start_index=4;
  created-=std::chrono::seconds(121);expectRejected("CONTEXT_EXPIRED");created=std::chrono::steady_clock::now();
  stages[3].id="DETACH_CONFIRM";expectRejected("STAGE_INVALID");
}

TEST_F(PayloadTransition, RejectsUnboundOrEmptyTargetBody) {
  auto& target=request.scene.robot_state.attached_collision_objects.front();
  target.object.header.frame_id="astribot_torso_base";expectRejected("ATTACHED_FRAME_UNSUPPORTED");
  target.object.header.frame_id="tcp";target.object.primitives.clear();target.object.primitive_poses.clear();
  expectRejected("TARGET_GEOMETRY_INVALID");
}

TEST_F(PayloadTransition, RejectsAcmOctomapOriginAndMetadataChanges) {
  const auto valid=request.scene;
  request.scene.allowed_collision_matrix.default_entry_names.push_back("wall");
  request.scene.allowed_collision_matrix.default_entry_values.push_back(true);expectRejected("SCENE_CHANGED");
  request.scene=valid;request.scene.world.octomap.origin.position.x=.000001;expectRejected("SCENE_CHANGED");
  request.scene=valid;request.scene.world.octomap.octomap.resolution=.01;expectRejected("SCENE_CHANGED");
}

TEST_F(PayloadTransition, RejectsMissingDuplicateAndAmbiguousTarget) {
  const auto attached=request.scene.robot_state.attached_collision_objects.front();
  request.scene.robot_state.attached_collision_objects.clear();expectRejected("TARGET_STATE_INVALID");
  request.scene.robot_state.attached_collision_objects={attached,attached};expectRejected("TARGET_STATE_INVALID");
  request.scene.robot_state.attached_collision_objects={attached};
  request.scene.world.collision_objects.push_back(box("payload",.08));expectRejected("TARGET_STATE_INVALID");
}

TEST_F(PayloadTransition, RejectsMalformedShapeAndWrongLink) {
  auto& attached=request.scene.robot_state.attached_collision_objects.front();
  attached.object.primitives.front().dimensions[0]=-1.;expectRejected("TARGET_GEOMETRY_INVALID");
  attached.object.primitives.front().dimensions[0]=.08;
  attached.link_name="astribot_torso_base";attached.object.header.frame_id=attached.link_name;
  expectRejected("TARGET_LINK_CHANGED");
}

TEST_F(PayloadTransition, RepeatRequiresSameTransactionAndActualScene) {
  auto confirmed=validate();binding.transaction_id=request.transaction_id;binding.confirmed_scene=request.scene;stages=confirmed;
  EXPECT_NO_THROW(validate());request.transaction_id="other";expectRejected("TRANSACTION_CHANGED");
  request.transaction_id=binding.transaction_id;
  request.scene.robot_state.attached_collision_objects.front().object.pose.position.y=.05;
  expectRejected("CONFIRMED_SCENE_CHANGED");
}

TEST_F(PayloadTransition, LaterOccupancyCheckCannotRecoverOldSmallBody) {
  auto confirmed=validate();auto scene=planning_scene::PlanningScene::clone(confirmed[4].scene);
  // A later occupied voxel clears the predicted 5cm body but intersects the confirmed 8cm body.
  auto map=request.scene.world.octomap;octomap::OcTree tree(.005);
  tree.updateNode(octomap::point3d(.2,.036,.002),true);
  ASSERT_TRUE(octomap_msgs::binaryMapToMsg(tree,map.octomap));scene->processOctomapMsg(map);
  auto predicted_scene=planning_scene::PlanningScene::clone(stages[4].scene);predicted_scene->processOctomapMsg(map);
  robot_trajectory::RobotTrajectory predicted(*stages[4].trajectory,true);std::string reason;
  ASSERT_TRUE(astribot_s1_manipulation::validateExternalTrajectory(predicted_scene,predicted,reason,.1,.1))<<reason;
  robot_trajectory::RobotTrajectory trajectory(*confirmed[4].trajectory,true);
  EXPECT_FALSE(astribot_s1_manipulation::validateExternalTrajectory(scene,trajectory,reason,.1,.1));
  EXPECT_NE(reason.find("EXTERNAL_COLLISION"),std::string::npos)<<reason;
}

TEST_F(PayloadTransition, UnreferencedTransformDriftAllowsAttachAndConfirmedReadback) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="aft_mapped";transform.child_frame_id="astribot_torso_base";
  transform.transform.rotation.w=1.;transform.transform.translation.x=.000627574547;
  binding.input_scene.fixed_frame_transforms={transform};
  transform.transform.translation.x=.000594694690;
  request.scene.fixed_frame_transforms={transform};
  auto confirmed=validate();
  binding.transaction_id=request.transaction_id;binding.confirmed_scene=request.scene;stages=confirmed;
  request.scene.fixed_frame_transforms.front().transform.translation.x=.00058;
  EXPECT_NO_THROW(validate());
}

TEST_F(PayloadTransition, RejectsUnchangedExternalFrameOnWorldGeometry) {
  binding.input_scene.world.collision_objects.back().header.frame_id="aft_mapped";
  request.scene.world.collision_objects.front().header.frame_id="aft_mapped";
  expectRejected("WORLD_FRAME_UNSUPPORTED");
}

TEST_F(PayloadTransition, RejectsExternalFrameBeforeOmittingAuthorizedPayload) {
  binding.input_scene.world.collision_objects.front().header.frame_id="aft_mapped";
  expectRejected("WORLD_FRAME_UNSUPPORTED");
}

TEST_F(PayloadTransition, RejectsUnchangedUnknownAttachmentLink) {
  moveit_msgs::msg::AttachedCollisionObject other;
  other.object=box("other",.02,0.,2.);other.link_name="external_link";
  other.object.header.frame_id=other.link_name;
  binding.input_scene.robot_state.attached_collision_objects.push_back(other);
  request.scene.robot_state.attached_collision_objects.push_back(other);
  expectRejected("ATTACHED_LINK_UNKNOWN");
}

TEST_F(PayloadTransition, RevalidationPreservesScaledOwnerAndCacheMessages) {
  std::vector<moveit_msgs::msg::RobotTrajectory> owner;
  for(size_t i=4;i<stages.size();++i) {
    std::string error;
    ASSERT_TRUE(astribot_s1_manipulation::validateExternalTrajectory(
      stages[i].scene,*stages[i].trajectory,error,.1,.1,2.5))<<error;
    moveit_msgs::msg::RobotTrajectory message;stages[i].trajectory->getRobotTrajectoryMsg(message);
    owner.push_back(message);
  }
  const auto confirmed=validate();
  binding.transaction_id=request.transaction_id;binding.confirmed_scene=request.scene;
  stages=confirmed;const auto repeated=validate();
  for(size_t i=4;i<stages.size();++i) {
    moveit_msgs::msg::RobotTrajectory first,second;
    confirmed[i].trajectory->getRobotTrajectoryMsg(first);repeated[i].trajectory->getRobotTrajectoryMsg(second);
    EXPECT_EQ(owner[i-4],first);EXPECT_EQ(owner[i-4],second);
  }
}
