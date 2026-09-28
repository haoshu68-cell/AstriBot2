#include <gtest/gtest.h>
#include <thread>
#define main grasp_gate_program_main
#include "src/grasp_candidate_gate_node.cpp"
#undef main
TEST(GraspPose, RejectsNonUnitQuaternion) {
  geometry_msgs::msg::PoseStamped p; p.pose.orientation.w=2.; EXPECT_FALSE(finite_pose(p));
}
TEST(GraspPose, RejectsOverflowQuaternion) {
  geometry_msgs::msg::PoseStamped p; p.pose.orientation.w=1e308; EXPECT_FALSE(finite_pose(p));
}
TEST(GraspPose, AcceptsUnitQuaternion) { geometry_msgs::msg::PoseStamped p; p.pose.orientation.w=1.; EXPECT_TRUE(finite_pose(p)); }
TEST(GraspGate, MissingContextMustNotPass) {
  rclcpp::init(0,nullptr);
  auto gate=std::make_shared<GraspCandidateGate>(); auto probe=std::make_shared<rclcpp::Node>("grasp_test");
  using Array=astribot_perception_msgs::msg::GraspCandidateArray;
  Array::ConstSharedPtr received;
  auto pub=probe->create_publisher<Array>("/perception/grasp_candidates",10);
  auto sub=probe->create_subscription<Array>("/manipulation/valid_grasp_candidates",10,[&](Array::ConstSharedPtr p){received=p;});
  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(gate); exec.add_node(probe);
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<until) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  Array input; input.task_id="test"; input.source_epoch="epoch";
  input.header.frame_id=input.source_frame="camera"; input.header.stamp=probe->now();
  astribot_perception_msgs::msg::GraspCandidate c;
  c.header=input.header; c.candidate_id="g1";c.object_id="o1";c.arm_id="right_arm";c.camera_id="head_rgbd";
  c.grasp_pose.header=c.header;c.grasp_pose.pose.orientation.w=1;c.score=.8;
  c.gripper_width_m=.05;c.gripper_height_m=.1;c.gripper_depth_m=.1;
  c.collision_checked=c.collision_free=c.geometry_valid=true;
  c.valid_until=probe->now()+rclcpp::Duration::from_seconds(.4);input.candidates.push_back(c);
  pub->publish(input);
  while(!received && std::chrono::steady_clock::now()<until) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  EXPECT_TRUE(received); if(received) {EXPECT_TRUE(received->candidates.empty());}
  exec.remove_node(gate);exec.remove_node(probe); gate.reset();probe.reset();rclcpp::shutdown();
}
class ConfiguredGraspGate : public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp() override {
    rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("required_calibration_revision",1),
      rclcpp::Parameter("required_planning_scene_revision",2),rclcpp::Parameter("required_envelope_epoch",3)});
    gate=std::make_shared<GraspCandidateGate>(opts);probe=std::make_shared<rclcpp::Node>("configured_grasp_probe");
    pub=probe->create_publisher<Array>("/perception/grasp_candidates",10);
    sub=probe->create_subscription<Array>("/manipulation/valid_grasp_candidates",10,[this](Array::ConstSharedPtr p){received=p;});
    exec.add_node(gate);exec.add_node(probe);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    input.task_id="test";input.source_epoch="epoch";input.header.frame_id=input.source_frame="camera";input.header.stamp=probe->now();
    astribot_perception_msgs::msg::GraspCandidate c;c.header=input.header;c.grasp_pose.header=c.header;c.grasp_pose.pose.orientation.w=1.;
    c.candidate_id="g1";c.object_id="o1";c.arm_id="right";c.camera_id="head";c.model_name="fixture";c.model_revision="rev1";
    c.calibration_revision=1;c.planning_scene_revision=2;c.envelope_epoch=3;c.score=.8;
    c.gripper_width_m=.05;c.gripper_height_m=.1;c.gripper_depth_m=.1;c.geometry_valid=c.collision_checked=c.collision_free=true;
    c.valid_until=probe->now()+rclcpp::Duration::from_seconds(.4);input.candidates.push_back(c);
  }
  void TearDown() override {exec.remove_node(gate);exec.remove_node(probe);gate.reset();probe.reset();}
  void send() {pub->publish(input);const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
    while(!received && std::chrono::steady_clock::now()<end){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    ASSERT_TRUE(received);
  }
  using Array=astribot_perception_msgs::msg::GraspCandidateArray;
  std::shared_ptr<GraspCandidateGate> gate;rclcpp::Node::SharedPtr probe;rclcpp::executors::SingleThreadedExecutor exec;
  Array input;Array::ConstSharedPtr received;rclcpp::Publisher<Array>::SharedPtr pub;rclcpp::Subscription<Array>::SharedPtr sub;
};
TEST_F(ConfiguredGraspGate, ScreensMatchingCandidateWithoutExecutionPermission) { send();ASSERT_TRUE(received);EXPECT_EQ(received->candidates.size(),1u);EXPECT_EQ(received->reason_code,"CANDIDATES_SCREENED_REQUIRES_MTC_VALIDATION"); }
TEST_F(ConfiguredGraspGate, RejectsSceneMismatch) {input.candidates[0].planning_scene_revision=1;send();ASSERT_TRUE(received);EXPECT_TRUE(received->candidates.empty());}
TEST_F(ConfiguredGraspGate, RejectsEnvelopeMismatch) {input.candidates[0].envelope_epoch=1;send();ASSERT_TRUE(received);EXPECT_TRUE(received->candidates.empty());}
TEST_F(ConfiguredGraspGate, RejectsUncheckedCollision) {input.candidates[0].collision_checked=false;send();ASSERT_TRUE(received);EXPECT_TRUE(received->candidates.empty());}
TEST_F(ConfiguredGraspGate, RejectsArrayFrameMismatch) {input.source_frame="other";send();ASSERT_TRUE(received);EXPECT_TRUE(received->candidates.empty());}
TEST_F(ConfiguredGraspGate, PinnedContextCannotPretendToUpdate) {EXPECT_FALSE(gate->set_parameters_atomically({rclcpp::Parameter("required_planning_scene_revision",9)}).successful);}
