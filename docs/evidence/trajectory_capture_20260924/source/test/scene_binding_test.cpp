#include <gtest/gtest.h>
#include "astribot_s1_transport_native/scene_binding.hpp"
using namespace astribot::transport;
TEST(SceneBinding, RejectsDiffAndBindsWorldAttachmentsAndCollisionPolicy) {
 moveit_msgs::msg::PlanningScene a;a.world.octomap.header.frame_id="astribot_torso_base";
 auto b=a;b.is_diff=true;EXPECT_THROW(bind_scene(b),std::runtime_error);
 b=a;b.robot_state.is_diff=true;EXPECT_THROW(bind_scene(b),std::runtime_error);
 moveit_msgs::msg::CollisionObject object;object.id="box";object.header.frame_id="astribot_torso_base";
 b=a;b.world.collision_objects.push_back(object);EXPECT_NE(bind_scene(a),bind_scene(b));
 b=a;moveit_msgs::msg::AttachedCollisionObject body;body.object=object;body.link_name="tcp";body.object.header.frame_id="tcp";
 b.robot_state.attached_collision_objects.push_back(body);EXPECT_NE(bind_scene(a,{"tcp"}),bind_scene(b,{"tcp"}));
 auto changed=b;changed.robot_state.attached_collision_objects.front().object.pose.position.x=.000001;
 EXPECT_NE(bind_scene(b,{"tcp"}),bind_scene(changed,{"tcp"}));
 EXPECT_THROW(bind_scene(b),std::runtime_error);
 b=a;b.allowed_collision_matrix.default_entry_names={"box"};b.allowed_collision_matrix.default_entry_values={true};EXPECT_NE(bind_scene(a),bind_scene(b));
}
TEST(SceneBinding, SourceStampsOrderingAndIndependentJointSamplesDoNotChangeGeometry) {
 moveit_msgs::msg::PlanningScene a;a.world.octomap.header.frame_id="astribot_torso_base";
 moveit_msgs::msg::CollisionObject first,second;first.id="first";second.id="second";
 first.header.frame_id=second.header.frame_id="astribot_torso_base";a.world.collision_objects={first,second};
 auto b=a;std::swap(b.world.collision_objects[0],b.world.collision_objects[1]);b.world.collision_objects[0].header.stamp.sec=3;
 b.robot_state.joint_state.name={"arm"};b.robot_state.joint_state.position={.1};
 EXPECT_EQ(bind_scene(a),bind_scene(b));
 b.world.collision_objects[0].pose.position.x=.1;EXPECT_NE(bind_scene(a),bind_scene(b));
}
TEST(SceneBinding, ExactMapOriginAndLinkPaddingStillBind) {
 moveit_msgs::msg::PlanningScene a;a.world.octomap.header.frame_id="astribot_torso_base";
 auto b=a;b.world.octomap.origin.position.x=.000001;EXPECT_NE(bind_scene(a),bind_scene(b));
 b=a;moveit_msgs::msg::LinkPadding padding;padding.link_name="tcp";padding.padding=.000001;
 b.link_padding={padding};EXPECT_NE(bind_scene(a),bind_scene(b));
}
TEST(SceneBinding, IgnoresUnreferencedExternalTransformDrift) {
 moveit_msgs::msg::PlanningScene a;a.world.octomap.header.frame_id="astribot_torso_base";
 geometry_msgs::msg::TransformStamped transform;
 transform.header.frame_id="aft_mapped";transform.child_frame_id="astribot_torso_base";
 transform.transform.rotation.w=1.;transform.transform.translation.x=.000627574547;
 a.fixed_frame_transforms={transform};auto b=a;
 b.fixed_frame_transforms.front().transform.translation.x=.000594694690;
 EXPECT_EQ(bind_scene(a),bind_scene(b));
}
TEST(SceneBinding, RejectsReferencedExternalFrameAndUnknownAttachedLink) {
 moveit_msgs::msg::PlanningScene scene;scene.world.octomap.header.frame_id="astribot_torso_base";
 moveit_msgs::msg::CollisionObject object;object.id="box";object.header.frame_id="aft_mapped";
 scene.world.collision_objects={object};EXPECT_THROW(bind_scene(scene),std::runtime_error);
 scene.world.collision_objects.clear();
 moveit_msgs::msg::AttachedCollisionObject body;body.object=object;
 body.link_name="external_link";body.object.header.frame_id=body.link_name;
 scene.robot_state.attached_collision_objects={body};EXPECT_THROW(bind_scene(scene),std::runtime_error);
}
