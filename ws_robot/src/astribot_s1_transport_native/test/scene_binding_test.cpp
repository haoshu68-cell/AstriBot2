#include <gtest/gtest.h>
#include "astribot_s1_transport_native/scene_binding.hpp"
using namespace astribot::transport;
TEST(SceneBinding, RejectsDiffAndBindsWorldAttachmentsAndCollisionPolicy) {
 moveit_msgs::msg::PlanningScene a;
 auto b=a;b.is_diff=true;EXPECT_THROW(bind_scene(b),std::runtime_error);
 b=a;b.robot_state.is_diff=true;EXPECT_THROW(bind_scene(b),std::runtime_error);
 moveit_msgs::msg::CollisionObject object;object.id="box";
 b=a;b.world.collision_objects.push_back(object);EXPECT_NE(bind_scene(a),bind_scene(b));
 b=a;moveit_msgs::msg::AttachedCollisionObject body;body.object=object;b.robot_state.attached_collision_objects.push_back(body);EXPECT_NE(bind_scene(a),bind_scene(b));
 b=a;b.allowed_collision_matrix.default_entry_names={"box"};b.allowed_collision_matrix.default_entry_values={true};EXPECT_NE(bind_scene(a),bind_scene(b));
}
TEST(SceneBinding, SourceStampsOrderingAndIndependentJointSamplesDoNotChangeGeometry) {
 moveit_msgs::msg::PlanningScene a;
 moveit_msgs::msg::CollisionObject first,second;first.id="first";second.id="second";a.world.collision_objects={first,second};
 auto b=a;std::swap(b.world.collision_objects[0],b.world.collision_objects[1]);b.world.collision_objects[0].header.stamp.sec=3;
 b.robot_state.joint_state.name={"arm"};b.robot_state.joint_state.position={.1};
 EXPECT_EQ(bind_scene(a),bind_scene(b));
 b.world.collision_objects[0].pose.position.x=.1;EXPECT_NE(bind_scene(a),bind_scene(b));
}
