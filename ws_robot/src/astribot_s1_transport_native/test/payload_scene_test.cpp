#include <gtest/gtest.h>
#include "astribot_s1_transport_native/payload_scene.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <limits>
using namespace astribot::transport;
namespace {
const std::set<std::string> known_links={"astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"};
astribot::payload::Observation physical() {
 astribot::payload::Observation o;o.full_inventory=true;o.status=o.ATTACHED;
 moveit_msgs::msg::AttachedCollisionObject body;body.link_name="astribot_arm_left_tcp_link";body.touch_links={body.link_name};body.weight=.2;
 body.object.id="box";body.object.header.frame_id=body.link_name;body.object.pose.orientation.w=1.;
 shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.071,.071,.131};body.object.primitives={box};
 geometry_msgs::msg::Pose p;p.orientation.w=1.;body.object.primitive_poses={p};o.objects={body};return o;
}
}
TEST(PayloadScene, RegisteredSizeAcceptsNominalAndActualScene45ConservativeBox) {
 auto shape=physical().objects.front().object.primitives.front();
 shape.dimensions={.06,.06,.12};
 EXPECT_TRUE(payload_registered_box_size_matches(shape,{.06,.06,.12}));
 shape.dimensions={.07146969384566991,.07146969384566991,.1314696938456699};
 EXPECT_TRUE(payload_registered_box_size_matches(shape,{.06,.06,.12}));
}
TEST(PayloadScene, RegisteredSizeRejectsMixedAxesRepeatedPaddingAndRealSizeChanges) {
 auto shape=physical().objects.front().object.primitives.front();
 const std::vector<double> nominal{.06,.06,.12};
 const std::vector<double> conservative{.07146969384566991,.07146969384566991,.1314696938456699};
 for(unsigned mask=1;mask<7;++mask) {
  for(size_t i=0;i<3;++i)shape.dimensions[i]=(mask&(1u<<i))?conservative[i]:nominal[i];
  EXPECT_FALSE(payload_registered_box_size_matches(shape,nominal))<<"mixed axes "<<mask;
 }
 for(size_t i=0;i<3;++i)shape.dimensions[i]=conservative[i]+(conservative[i]-nominal[i]);
 EXPECT_FALSE(payload_registered_box_size_matches(shape,nominal));
 // Recomputing the producer margin from an already inflated BOX is also invalid.
 shape.dimensions={.08312800969847545,.08312800969847545,.14312800969847544};
 EXPECT_FALSE(payload_registered_box_size_matches(shape,nominal));
 for(const auto &base:{nominal,conservative})for(size_t i=0;i<3;++i)for(double change:{-.001,.001}) {
  shape.dimensions.assign(base.begin(),base.end());shape.dimensions[i]+=change;
  EXPECT_FALSE(payload_registered_box_size_matches(shape,nominal));
 }
}
TEST(PayloadScene, RegisteredSizeKeepsNanometerComparisonAndRejectsInvalidDimensions) {
 auto shape=physical().objects.front().object.primitives.front();
 for(const std::vector<double> base: {std::vector<double>{.06,.06,.12},
      std::vector<double>{.07146969384566991,.07146969384566991,.1314696938456699}}) {
  for(size_t i=0;i<3;++i)for(double sign:{-1.,1.}) {
   shape.dimensions.assign(base.begin(),base.end());shape.dimensions[i]+=sign*5e-10;
   EXPECT_TRUE(payload_registered_box_size_matches(shape,{.06,.06,.12}));
   shape.dimensions.assign(base.begin(),base.end());shape.dimensions[i]+=sign*2e-9;
   EXPECT_FALSE(payload_registered_box_size_matches(shape,{.06,.06,.12}));
  }
 }
 for(double invalid:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-1.}) {
  shape.dimensions={.06,.06,.12};shape.dimensions[1]=invalid;
  EXPECT_FALSE(payload_registered_box_size_matches(shape,{.06,.06,.12}));
 }
 shape.dimensions={.06,.06};EXPECT_FALSE(payload_registered_box_size_matches(shape,{.06,.06,.12}));
}
TEST(PayloadScene, CopiesActualConservativeBodyAndRejectsUnrelatedSceneChanges) {
 moveit_msgs::msg::PlanningScene before;before.world.octomap.header.frame_id="astribot_torso_base";auto raw=physical().objects.front().object;raw.header.frame_id="astribot_torso_base";raw.primitives[0].dimensions={.06,.06,.12};
 before.world.collision_objects={raw};auto observed=physical();geometry_msgs::msg::Pose pose;pose.orientation.w=1.;
 const auto diff=payload_scene_diff(before,observed,"box",true,pose,"astribot_torso_base",known_links);
 ASSERT_EQ(diff.robot_state.attached_collision_objects.size(),1u);
 EXPECT_DOUBLE_EQ(diff.robot_state.attached_collision_objects[0].object.primitives[0].dimensions[0],.071);
 auto after=before;after.world.collision_objects.clear();after.robot_state.attached_collision_objects=observed.objects;
 after.robot_state.attached_collision_objects[0].weight=0.;
 EXPECT_NO_THROW(validate_payload_scene(before,after,observed,"box",true,pose,"astribot_torso_base",known_links));
 after.allowed_collision_matrix.default_entry_names={"obstacle"};after.allowed_collision_matrix.default_entry_values={true};
 EXPECT_THROW(validate_payload_scene(before,after,observed,"box",true,pose,"astribot_torso_base",known_links),std::runtime_error);
}
TEST(PayloadScene, DetachPreservesPhysicalGeometryAtMeasuredPoseAndFullInventory) {
 moveit_msgs::msg::PlanningScene before;before.world.octomap.header.frame_id="astribot_torso_base";before.robot_state.attached_collision_objects=physical().objects;
 auto observed=physical();auto other=observed.objects.front();other.object.id="other";observed.objects={other};before.robot_state.attached_collision_objects.push_back(other);
 geometry_msgs::msg::Pose pose;pose.orientation.w=1.;pose.position.z=.97;
 auto diff=payload_scene_diff(before,observed,"box",false,pose,"astribot_torso_base",known_links);
 ASSERT_EQ(diff.world.collision_objects.size(),1u);EXPECT_EQ(diff.world.collision_objects[0].header.frame_id,"astribot_torso_base");
 EXPECT_DOUBLE_EQ(diff.world.collision_objects[0].pose.position.z,.97);
 auto after=before;after.robot_state.attached_collision_objects=observed.objects;after.world.collision_objects=diff.world.collision_objects;
 EXPECT_NO_THROW(validate_payload_scene(before,after,observed,"box",false,pose,"astribot_torso_base",known_links));
 after.world.collision_objects[0].primitives[0].dimensions[0]=.06;
 EXPECT_THROW(validate_payload_scene(before,after,observed,"box",false,pose,"astribot_torso_base",known_links),std::runtime_error);
}

TEST(PayloadScene, RealMoveItAttachAndDetachPreserveGeometryAndUnrelatedObjects) {
 const auto urdf_model=urdf::parseURDF(R"(<robot name="payload_scene_fixture">
  <link name="astribot_torso_base"/>
  <link name="astribot_arm_left_tcp_link"/>
  <link name="astribot_arm_right_tcp_link"/>
  <joint name="left" type="fixed"><parent link="astribot_torso_base"/><child link="astribot_arm_left_tcp_link"/></joint>
  <joint name="right" type="fixed"><parent link="astribot_torso_base"/><child link="astribot_arm_right_tcp_link"/></joint>
 </robot>)");
 ASSERT_TRUE(urdf_model);auto srdf_model=std::make_shared<srdf::Model>();
 ASSERT_TRUE(srdf_model->initString(*urdf_model,"<robot name=\"payload_scene_fixture\"/>"));
 planning_scene::PlanningScene scene(urdf_model,srdf_model);
 const std::set<std::string> model_links={"astribot_torso_base","astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"};
 auto observed=physical();observed.objects[0].object.pose.position.y=-.1;
 observed.objects[0].object.primitives[0].dimensions={.07146969384566991,.07146969384566991,.1314696938456699};
 auto world=observed.objects[0].object;world.header.frame_id="astribot_torso_base";
 world.primitives[0].dimensions={.06,.06,.12};ASSERT_TRUE(scene.processCollisionObjectMsg(world));
 ASSERT_TRUE(payload_registered_box_size_matches(world.primitives[0],{.06,.06,.12}));
 auto obstacle=world;obstacle.id="obstacle";obstacle.pose.position.x=2.;
 ASSERT_TRUE(scene.processCollisionObjectMsg(obstacle));
 auto other=physical().objects.front();other.object.id="other";
 other.link_name=other.object.header.frame_id="astribot_arm_right_tcp_link";other.touch_links={other.link_name};
 ASSERT_TRUE(scene.processAttachedCollisionObjectMsg(other));observed.objects.push_back(other);
 moveit_msgs::msg::PlanningScene before;scene.getPlanningSceneMsg(before);
 geometry_msgs::msg::Pose pose;pose.orientation.w=1.;
 const auto diff=payload_scene_diff(before,observed,"box",true,pose,"astribot_torso_base",model_links);

 // Reproduce the old non-atomic failure in the actual installed MoveIt core.
 planning_scene::PlanningScene old_scene(urdf_model,srdf_model);ASSERT_TRUE(old_scene.setPlanningSceneMsg(before));
 auto redundant=diff;moveit_msgs::msg::CollisionObject removed;removed.id="box";removed.operation=removed.REMOVE;
 redundant.world.collision_objects={removed};
 EXPECT_FALSE(old_scene.setPlanningSceneDiffMsg(redundant));
 EXPECT_TRUE(old_scene.getCurrentState().hasAttachedBody("box"));EXPECT_FALSE(old_scene.getWorld()->hasObject("box"));

 ASSERT_TRUE(scene.setPlanningSceneDiffMsg(diff));
 moveit_msgs::msg::PlanningScene attached;scene.getPlanningSceneMsg(attached);
 EXPECT_NO_THROW(validate_payload_scene(before,attached,observed,"box",true,pose,"astribot_torso_base",model_links));
 EXPECT_FALSE(scene.getWorld()->hasObject("box"));EXPECT_TRUE(scene.getWorld()->hasObject("obstacle"));
 EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("other"));
 const auto body=std::find_if(attached.robot_state.attached_collision_objects.begin(),attached.robot_state.attached_collision_objects.end(),
  [](const auto &value){return value.object.id=="box";});
 ASSERT_NE(body,attached.robot_state.attached_collision_objects.end());
 ASSERT_EQ(body->object.primitives.size(),1u);EXPECT_EQ(body->object.primitives[0].dimensions,observed.objects[0].object.primitives[0].dimensions);

 auto detached=observed;detached.objects.erase(detached.objects.begin());
 pose.position.x=.2;pose.position.y=-.3;pose.position.z=.97;
 const auto detach_diff=payload_scene_diff(attached,detached,"box",false,pose,"astribot_torso_base",model_links);
 ASSERT_TRUE(scene.setPlanningSceneDiffMsg(detach_diff));
 moveit_msgs::msg::PlanningScene placed;scene.getPlanningSceneMsg(placed);
 EXPECT_NO_THROW(validate_payload_scene(attached,placed,detached,"box",false,pose,"astribot_torso_base",model_links));
 EXPECT_FALSE(scene.getCurrentState().hasAttachedBody("box"));EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("other"));
 ASSERT_TRUE(scene.getWorld()->hasObject("box"));
 EXPECT_NEAR(scene.getWorld()->getObject("box")->pose_.translation().x(),.2,1e-12);
 EXPECT_NEAR(scene.getWorld()->getObject("box")->pose_.translation().y(),-.3,1e-12);
 EXPECT_NEAR(scene.getWorld()->getObject("box")->pose_.translation().z(),.97,1e-12);
 // A second PICK consumes the conservative world geometry produced by DETACH.
 // Each authoritative attachment is generated from the same nominal body;
 // the actual MoveIt roundtrip must neither grow nor shrink its dimensions.
 const auto placed_box=std::find_if(placed.world.collision_objects.begin(),placed.world.collision_objects.end(),
   [](const auto &value){return value.id=="box";});
 ASSERT_NE(placed_box,placed.world.collision_objects.end());
 ASSERT_TRUE(payload_registered_box_size_matches(placed_box->primitives[0],{.06,.06,.12}));
 EXPECT_EQ(placed_box->primitives[0].dimensions,observed.objects[0].object.primitives[0].dimensions);
 const auto second_attach=payload_scene_diff(placed,observed,"box",true,pose,"astribot_torso_base",model_links);
 ASSERT_TRUE(scene.setPlanningSceneDiffMsg(second_attach));
 moveit_msgs::msg::PlanningScene attached_again;scene.getPlanningSceneMsg(attached_again);
 EXPECT_NO_THROW(validate_payload_scene(placed,attached_again,observed,"box",true,pose,"astribot_torso_base",model_links));
 const auto second_detach=payload_scene_diff(attached_again,detached,"box",false,pose,"astribot_torso_base",model_links);
 ASSERT_TRUE(scene.setPlanningSceneDiffMsg(second_detach));scene.getPlanningSceneMsg(placed);
 EXPECT_NO_THROW(validate_payload_scene(attached_again,placed,detached,"box",false,pose,"astribot_torso_base",model_links));
 const auto placed_again=std::find_if(placed.world.collision_objects.begin(),placed.world.collision_objects.end(),
   [](const auto &value){return value.id=="box";});
 ASSERT_NE(placed_again,placed.world.collision_objects.end());
 EXPECT_EQ(placed_again->primitives[0].dimensions,observed.objects[0].object.primitives[0].dimensions);
 obstacle.pose.position.x+=.1;ASSERT_TRUE(scene.processCollisionObjectMsg(obstacle));
 scene.getPlanningSceneMsg(placed);
 EXPECT_THROW(validate_payload_scene(attached,placed,detached,"box",false,pose,"astribot_torso_base",model_links),std::runtime_error);
}

TEST(PayloadScene, ChecksTargetFramesBeforeRemovingAuthorizedObjectFromComparison) {
 moveit_msgs::msg::PlanningScene before;before.world.octomap.header.frame_id="astribot_torso_base";auto observed=physical();
 auto raw=observed.objects.front().object;raw.header.frame_id="map";
 before.world.collision_objects={raw};geometry_msgs::msg::Pose pose;pose.orientation.w=1.;
 EXPECT_THROW(payload_scene_diff(before,observed,"box",true,pose,"astribot_torso_base",known_links),std::runtime_error);
 before.world.collision_objects[0].header.frame_id="astribot_torso_base";
 auto after=before;after.world.collision_objects.clear();after.robot_state.attached_collision_objects=observed.objects;
 EXPECT_THROW(validate_payload_scene(before,after,observed,"box",true,pose,"astribot_torso_base",{}),std::runtime_error);
 after.robot_state.attached_collision_objects[0].object.header.frame_id="map";
 EXPECT_THROW(validate_payload_scene(before,after,observed,"box",true,pose,"astribot_torso_base",known_links),std::runtime_error);
}

TEST(PayloadScene, WaitsForDelayedLedgerButRejectsUnexpectedRevisionOrEpoch) {
 auto expected=physical();expected.source_epoch="source";expected.clock_epoch=3;expected.revision=12;
 astribot::payload::State state;state.ledger_epoch="ledger";state.observation=expected;
 state.observation.revision=11;
 EXPECT_FALSE(payload_revision_ready(state,expected,"ledger"));
 state.observation.revision=12;
 EXPECT_TRUE(payload_revision_ready(state,expected,"ledger"));
 state.observation.revision=13;
 EXPECT_THROW(payload_revision_ready(state,expected,"ledger"),std::runtime_error);
 state.observation=expected;state.observation.clock_epoch=4;
 EXPECT_THROW(payload_revision_ready(state,expected,"ledger"),std::runtime_error);
 state.observation=expected;state.observation.source_epoch="restarted";
 EXPECT_THROW(payload_revision_ready(state,expected,"ledger"),std::runtime_error);
 state.observation=expected;
 EXPECT_THROW(payload_revision_ready(state,expected,"restarted_ledger"),std::runtime_error);
}

TEST(PayloadScene, KnownUnpairedRawInvalidatesFreshOldLedgerBeforeRemainingMotion) {
 auto bound=physical();bound.source_epoch="source";bound.clock_epoch=1;bound.revision=2;bound.sequence=10;
 astribot::payload::State ledger;ledger.ledger_epoch="ledger";ledger.observation=bound;ledger.confirmed=true;
 // Previous criterion still authorizes this ledger revision, independent of
 // whether any diagnostic exists for the newly received raw observation.
 ASSERT_TRUE(payload_revision_ready(ledger,bound,"ledger"));
 auto latest=bound;latest.sequence=11;latest.revision=3;
 EXPECT_THROW(require_payload_observation_bound(bound,latest),std::runtime_error);
 latest=bound;latest.sequence=11;latest.status=latest.UNKNOWN;latest.full_inventory=false;latest.objects.clear();
 EXPECT_THROW(require_payload_observation_bound(bound,latest),std::runtime_error);
 latest=bound;latest.sequence=11;latest.objects[0].object.primitives[0].dimensions[0]+=.001;
 EXPECT_THROW(require_payload_observation_bound(bound,latest),std::runtime_error);
 latest=bound;latest.sequence=11;
 EXPECT_NO_THROW(require_payload_observation_bound(bound,latest));
 ledger.observation.revision=1;
 EXPECT_FALSE(payload_revision_ready(ledger,bound,"ledger")); // legitimate lag still waits
}
