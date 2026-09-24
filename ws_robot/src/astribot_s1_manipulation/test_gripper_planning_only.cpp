#include "astribot_s1_manipulation/gripper_commander.hpp"
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>

using namespace astribot_s1_manipulation;
class GripperPlanningOnly : public ::testing::Test {
protected:
  moveit::core::RobotModelPtr model;
  rclcpp::Node::SharedPtr node;
  void SetUp() override {
    const char* urdf_path=std::getenv("ASTRIBOT_GRIPPER_TEST_URDF");
    const char* srdf_path=std::getenv("ASTRIBOT_GRIPPER_TEST_SRDF");
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    if(!urdf_path || !*urdf_path || !srdf_path || !*srdf_path || !domain || !*domain)
      GTEST_SKIP()<<"Optional integration test requires ASTRIBOT_GRIPPER_TEST_URDF, "
                   "ASTRIBOT_GRIPPER_TEST_SRDF and an isolated ROS_DOMAIN_ID=181";
    ASSERT_STREQ(domain,"181");
    std::ifstream u(urdf_path),s(srdf_path);
    std::stringstream ut,st;ut<<u.rdbuf();st<<s.rdbuf();
    auto urdf=urdf::parseURDF(ut.str());ASSERT_TRUE(urdf);
    auto srdf=std::make_shared<srdf::Model>();ASSERT_TRUE(srdf->initString(*urdf,st.str()));
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    rclcpp::init(0,nullptr);
    node=std::make_shared<rclcpp::Node>("gripper_planning_only_probe","/gripper_planning_test");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for(const auto& name:node->get_node_names())
      ASSERT_EQ(name,node->get_fully_qualified_name())<<"Domain 181 has another node; do not use this graph";
  }
  void TearDown() override {
    if(node){node.reset();rclcpp::shutdown();}
    model.reset();
  }
  GripperConfig config() const {
    GripperConfig c;c.group_name="gripper_left";
    c.action_name="/gripper_left_controller/follow_joint_trajectory";
    c.tcp_link="astribot_arm_left_tcp_link";c.left_pad_link="astribot_gripper_left_Link_L11";
    c.right_pad_link="astribot_gripper_left_Link_R11";return c;
  }
  bool hasControllerClient() const {
    const auto clients=node->get_node_graph_interface()->get_client_names_and_types_by_node(node->get_name(),node->get_namespace());
    for(const auto& entry:clients)
      if(entry.first.find("/gripper_left_controller/follow_joint_trajectory/")==0)return true;
    return false;
  }
  bool hasJointSubscription() const {
    const auto subscriptions=node->get_node_graph_interface()->get_subscriber_names_and_types_by_node(node->get_name(),node->get_namespace());
    return subscriptions.count("/joint_states")!=0;
  }
};

TEST_F(GripperPlanningOnly, PlanningDoesNotCreateControllerEndpoints) {
  GripperCommander gripper(node);std::string detail;
  ASSERT_EQ(gripper.configureForPlanning(model,config(),detail),PlanErrorCode::kSuccess)<<detail;
  EXPECT_FALSE(hasControllerClient());
  EXPECT_FALSE(hasJointSubscription());
}

TEST_F(GripperPlanningOnly, GeometryMatchesDefaultExecutionConfiguration) {
  GripperCommander original(node),planning(node);std::string detail;
  ASSERT_EQ(original.configure(model,config(),detail),PlanErrorCode::kSuccess)<<detail;
  EXPECT_TRUE(hasControllerClient());EXPECT_TRUE(hasJointSubscription());
  ASSERT_EQ(planning.configureForPlanning(model,config(),detail),PlanErrorCode::kSuccess)<<detail;
  EXPECT_EQ(planning.jointName(),original.jointName());
  EXPECT_DOUBLE_EQ(planning.openAngle(),original.openAngle());
  EXPECT_DOUBLE_EQ(planning.closedAngle(),original.closedAngle());
  for(int i=0;i<=60;++i) {
    const double angle=original.openAngle()+(original.closedAngle()-original.openAngle())*i/60.;
    EXPECT_DOUBLE_EQ(planning.jawWidthAtAngle(angle),original.jawWidthAtAngle(angle));
  }
  for(double width:{.03,.04,.05,.06,.07,.08}) {
    double original_angle=0.,planning_angle=0.;std::string original_detail;
    EXPECT_EQ(planning.graspAngleForWidth(width,planning_angle,detail),
              original.graspAngleForWidth(width,original_angle,original_detail));
    EXPECT_DOUBLE_EQ(planning_angle,original_angle);EXPECT_EQ(detail,original_detail);
  }
}

TEST_F(GripperPlanningOnly, AllExecutionEntrypointsRejectWithoutDiscoveryOrSending) {
  GripperCommander gripper(node);std::string detail;
  ASSERT_EQ(gripper.configureForPlanning(model,config(),detail),PlanErrorCode::kSuccess)<<detail;
  const auto started=std::chrono::steady_clock::now();
  for(const auto& outcome:{gripper.moveTo(gripper.openAngle()),gripper.open(),gripper.close(),gripper.closeToWidth(.06)}) {
    EXPECT_EQ(outcome.code,PlanErrorCode::kInvalidInput);
    EXPECT_NE(outcome.detail.find("GRIPPER_PLANNING_ONLY"),std::string::npos)<<outcome.detail;
  }
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count(),.5);
  EXPECT_FALSE(hasControllerClient());EXPECT_FALSE(hasJointSubscription());
}

TEST_F(GripperPlanningOnly, ReconfiguringForPlanningRemovesPreviousExecutionEndpoints) {
  GripperCommander gripper(node);std::string detail;
  ASSERT_EQ(gripper.configure(model,config(),detail),PlanErrorCode::kSuccess)<<detail;
  EXPECT_TRUE(hasControllerClient());EXPECT_TRUE(hasJointSubscription());
  ASSERT_EQ(gripper.configureForPlanning(model,config(),detail),PlanErrorCode::kSuccess)<<detail;
  EXPECT_FALSE(hasControllerClient());EXPECT_FALSE(hasJointSubscription());
  double measured;EXPECT_FALSE(gripper.measuredAngle(measured));
  EXPECT_EQ(gripper.moveTo(gripper.openAngle()).code,PlanErrorCode::kInvalidInput);
}
