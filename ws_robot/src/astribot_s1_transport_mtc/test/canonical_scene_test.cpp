#include <gtest/gtest.h>
#include "astribot_s1_transport_mtc/canonical_scene.hpp"

using astribot::transport::canonicalScene;

class CanonicalScene : public ::testing::Test {
protected:
  moveit_msgs::msg::PlanningScene scene;
  const std::set<std::string> links{"astribot_torso_base","tcp"};
  void SetUp() override {
    scene.world.octomap.header.frame_id="astribot_torso_base";
    scene.world.octomap.origin.orientation.w=1.;
  }
  moveit_msgs::msg::PlanningScene canonical() {
    return canonicalScene(scene,"astribot_torso_base",links);
  }
};

TEST_F(CanonicalScene, PreservesGeometryAndSamplesWhileRemovingUnusedTransforms) {
  scene.robot_state.joint_state.name={"joint"};scene.robot_state.joint_state.position={.25};
  moveit_msgs::msg::CollisionObject object;object.id="box";object.header.frame_id="astribot_torso_base";
  object.pose.position.x=.123456789;scene.world.collision_objects={object};
  scene.world.octomap.origin.position.z=.0123456789;
  scene.world.octomap.octomap.data={1,2,3};
  scene.allowed_collision_matrix.default_entry_names={"box"};
  scene.allowed_collision_matrix.default_entry_values={false};
  auto expected=scene;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="aft_mapped";transform.child_frame_id="astribot_torso_base";
  transform.transform.rotation.w=1.;transform.transform.translation.x=.0006;
  scene.fixed_frame_transforms={transform};
  EXPECT_EQ(canonical(),expected);
  EXPECT_EQ(scene.fixed_frame_transforms.size(),1u);
}

TEST_F(CanonicalScene, RejectsDiffScenesAndNonFixedPlanningFrame) {
  scene.is_diff=true;EXPECT_THROW(canonical(),std::runtime_error);scene.is_diff=false;
  scene.robot_state.is_diff=true;EXPECT_THROW(canonical(),std::runtime_error);scene.robot_state.is_diff=false;
  EXPECT_THROW(canonicalScene(scene,"map",links),std::runtime_error);
}

TEST_F(CanonicalScene, RejectsMultiDofStateInsteadOfDiscardingIt) {
  scene.robot_state.multi_dof_joint_state.joint_names={"floating"};
  EXPECT_THROW(canonical(),std::runtime_error);
  scene.robot_state.multi_dof_joint_state=sensor_msgs::msg::MultiDOFJointState();
  scene.robot_state.multi_dof_joint_state.transforms.emplace_back();
  EXPECT_THROW(canonical(),std::runtime_error);
  scene.robot_state.multi_dof_joint_state=sensor_msgs::msg::MultiDOFJointState();
  scene.robot_state.multi_dof_joint_state.twist.emplace_back();
  EXPECT_THROW(canonical(),std::runtime_error);
  scene.robot_state.multi_dof_joint_state=sensor_msgs::msg::MultiDOFJointState();
  scene.robot_state.multi_dof_joint_state.wrench.emplace_back();
  EXPECT_THROW(canonical(),std::runtime_error);
}

TEST_F(CanonicalScene, RejectsExternalWorldAndMapFrames) {
  moveit_msgs::msg::CollisionObject object;object.id="box";object.header.frame_id="aft_mapped";
  scene.world.collision_objects={object};EXPECT_THROW(canonical(),std::runtime_error);
  scene.world.collision_objects.clear();
  scene.world.octomap.header.frame_id="aft_mapped";EXPECT_THROW(canonical(),std::runtime_error);
  scene.world.octomap.header.frame_id="astribot_torso_base";
  scene.world.octomap.octomap.header.frame_id="aft_mapped";EXPECT_THROW(canonical(),std::runtime_error);
  scene.world.octomap.octomap.header.frame_id="astribot_torso_base";EXPECT_NO_THROW(canonical());
  scene.world.octomap.octomap.header.frame_id.clear();EXPECT_NO_THROW(canonical());
}

TEST_F(CanonicalScene, RequiresKnownLinkLocalAttachmentGeometry) {
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object.id="payload";attached.link_name="tcp";attached.object.header.frame_id="tcp";
  scene.robot_state.attached_collision_objects={attached};
  EXPECT_EQ(canonical().robot_state.attached_collision_objects,scene.robot_state.attached_collision_objects);
  EXPECT_THROW(canonicalScene(scene,"astribot_torso_base",{}),std::runtime_error);
  scene.robot_state.attached_collision_objects.front().object.header.frame_id="astribot_torso_base";
  EXPECT_THROW(canonical(),std::runtime_error);
  scene.robot_state.attached_collision_objects.front().link_name="external_link";
  scene.robot_state.attached_collision_objects.front().object.header.frame_id="external_link";
  EXPECT_THROW(canonical(),std::runtime_error);
}
