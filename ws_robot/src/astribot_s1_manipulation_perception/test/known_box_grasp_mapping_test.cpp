#include <astribot_s1_manipulation_perception/known_box_grasp_mapping.hpp>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/robot_state/robot_state.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>

namespace pp=astribot::perception_planning;
namespace manipulation=astribot_s1_manipulation;
class BoxRegistration : public ::testing::Test {
protected:
  static inline rclcpp::Node::SharedPtr node;
  static inline moveit::core::RobotModelPtr model;
  static inline std::unique_ptr<manipulation::GripperCommander> gripper;
  moveit_msgs::msg::CollisionObject box;
  pp::Candidate candidate;
  pp::Pose::Result pose;
  static void SetUpTestSuite() {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"100");
    ASSERT_NE(std::getenv("M3_TEST_URDF"),nullptr);ASSERT_NE(std::getenv("M3_TEST_SRDF"),nullptr);
    std::ifstream u(std::getenv("M3_TEST_URDF")),s(std::getenv("M3_TEST_SRDF"));
    std::stringstream ut,st;ut<<u.rdbuf();st<<s.rdbuf();
    auto urdf=urdf::parseURDF(ut.str());ASSERT_TRUE(urdf);
    auto srdf=std::make_shared<srdf::Model>();ASSERT_TRUE(srdf->initString(*urdf,st.str()));
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    rclcpp::init(0,nullptr);node=std::make_shared<rclcpp::Node>("m3_registration_geometry_only");
    gripper=std::make_unique<manipulation::GripperCommander>(node);
    manipulation::GripperConfig c;c.group_name="gripper_left";
    c.action_name="/m3_unused_gripper_controller/follow_joint_trajectory";
    c.tcp_link="astribot_arm_left_tcp_link";c.left_pad_link="astribot_gripper_left_Link_L11";
    c.right_pad_link="astribot_gripper_left_Link_R11";std::string why;
    ASSERT_EQ(gripper->configure(model,c,why),manipulation::PlanErrorCode::kSuccess)<<why;
  }
  static void TearDownTestSuite() {gripper.reset();node.reset();model.reset();rclcpp::shutdown();}
  void SetUp() override {
    box.pose.orientation.w=1;shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;
    shape.dimensions={.04,.06,.04};box.primitives={shape};geometry_msgs::msg::Pose p;p.orientation.w=1;
    box.primitive_poses={p};pose.observation.pose.pose.orientation.w=1;
    candidate.grasp_pose.pose.orientation.w=1;candidate.gripper_width_m=.08;
    candidate.gripper_depth_m=.01;candidate.gripper_height_m=.02;
  }
  pp::GraspMapping mapping() {return pp::map_known_box_grasp(box,candidate,pose,model,*gripper);}
};
TEST_F(BoxRegistration, UsesCadWidthAndActualCommanderClosure) {
  const auto m=mapping();double q;std::string why;
  ASSERT_EQ(gripper->graspAngleForWidth(.06,q,why),manipulation::PlanErrorCode::kSuccess);
  EXPECT_DOUBLE_EQ(m.physical_object_width_m,.06);EXPECT_DOUBLE_EQ(m.commanded_joint_angle_rad,q);
  EXPECT_NEAR(m.grasp_from_tcp.translation.y,-.006,1e-8);
  EXPECT_NEAR(m.grasp_from_tcp.translation.x,-.0564122794,5e-5);
  std::cout<<std::setprecision(12)<<"M3_BOX_GEOMETRY width="<<m.physical_object_width_m
    <<" q="<<q<<" open_gap="<<gripper->jawWidthAtAngle(gripper->openAngle())
    <<" tcp_translation="<<m.grasp_from_tcp.translation.x<<","<<m.grasp_from_tcp.translation.y
    <<","<<m.grasp_from_tcp.translation.z<<std::endl;
}
TEST_F(BoxRegistration, CandidateDepthChangesTcpAlongApproachByThirtyMillimeters) {
  const auto near=mapping();candidate.gripper_depth_m=.04;const auto far=mapping();
  EXPECT_NEAR(far.grasp_from_tcp.translation.x-near.grasp_from_tcp.translation.x,.03,1e-10);
  EXPECT_DOUBLE_EQ(far.commanded_joint_angle_rad,near.commanded_joint_angle_rad);
}
TEST_F(BoxRegistration, ChangedCadWidthUsesNewClosureAndFk) {
  const auto wide=mapping();box.primitives[0].dimensions[1]=.04;const auto narrow=mapping();
  EXPECT_GT(narrow.commanded_joint_angle_rad,wide.commanded_joint_angle_rad);
  EXPECT_GT(std::abs(narrow.grasp_from_tcp.translation.x-wide.grasp_from_tcp.translation.x),.001);
}
TEST_F(BoxRegistration, RejectsSlantedOffCenterTooWideAndUnknownDepth) {
  candidate.grasp_pose.pose.orientation.z=std::sin(.1);candidate.grasp_pose.pose.orientation.w=std::cos(.1);
  EXPECT_THROW(mapping(),std::runtime_error);candidate.grasp_pose.pose.orientation.z=0;candidate.grasp_pose.pose.orientation.w=1;
  candidate.grasp_pose.pose.position.y=.01;EXPECT_THROW(mapping(),std::runtime_error);
  candidate.grasp_pose.pose.position.y=0;box.primitives[0].dimensions[1]=.1;candidate.gripper_width_m=.1;
  EXPECT_THROW(mapping(),std::runtime_error);box.primitives[0].dimensions[1]=.06;
  candidate.gripper_depth_m=.015;EXPECT_THROW(mapping(),std::runtime_error);
}
TEST_F(BoxRegistration, RotatedCameraDoesNotChangeRelativeBoxRegistration) {
  const auto original=mapping();
  // Identical 90 degree camera rotation and translation on both observed poses.
  const double v=std::sqrt(.5);
  pose.observation.pose.pose.orientation.z=candidate.grasp_pose.pose.orientation.z=v;
  pose.observation.pose.pose.orientation.w=candidate.grasp_pose.pose.orientation.w=v;
  pose.observation.pose.pose.position.x=candidate.grasp_pose.pose.position.x=.2;
  pose.observation.pose.pose.position.y=candidate.grasp_pose.pose.position.y=-.1;
  pose.observation.pose.pose.position.z=candidate.grasp_pose.pose.position.z=1;
  const auto rotated=mapping();
  EXPECT_NEAR(rotated.physical_object_width_m,original.physical_object_width_m,1e-12);
  EXPECT_NEAR(rotated.commanded_joint_angle_rad,original.commanded_joint_angle_rad,1e-12);
  EXPECT_NEAR(rotated.grasp_from_tcp.translation.x,original.grasp_from_tcp.translation.x,1e-12);
}
