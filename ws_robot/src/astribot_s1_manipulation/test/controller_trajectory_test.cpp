#include <gtest/gtest.h>
#include <astribot_s1_manipulation/controller_trajectory.hpp>
#include <astribot_s1_manipulation/external_trajectory_validator.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace manipulation = astribot_s1_manipulation;

class ControllerTrajectory : public ::testing::Test {
protected:
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  std::vector<moveit::core::RobotState> samples;
  std::string error;

  void SetUp() override {
    auto urdf = urdf::parseURDF(R"(<robot name="spline_test">
      <link name="base"/><link name="tip"><collision><geometry><box size="0.02 0.02 0.02"/>
      </geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/>
      <child link="tip"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="5"/>
      </joint></robot>)");
    ASSERT_TRUE(urdf);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, R"(<robot name="spline_test"><group name="slide_group">
      <joint name="slide"/></group></robot>)"));
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene = std::make_shared<planning_scene::PlanningScene>(model);
  }

  robot_trajectory::RobotTrajectory path(double start, double end, int degree,
    double v0 = 0, double v1 = 0, double a0 = 0, double a1 = 0, double duration = 1) {
    robot_trajectory::RobotTrajectory result(model, "slide_group");
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePosition("slide", start);
    if (degree >= 3) state.setVariableVelocity("slide", v0);
    if (degree == 5) state.setVariableAcceleration("slide", a0);
    state.update();
    result.addSuffixWayPoint(state, 0);
    state.setVariablePosition("slide", end);
    if (degree >= 3) state.setVariableVelocity("slide", v1);
    if (degree == 5) state.setVariableAcceleration("slide", a1);
    state.update();
    result.addSuffixWayPoint(state, duration);
    return result;
  }

  void obstacle(double x) {
    moveit_msgs::msg::CollisionObject object;
    object.id = "obstacle"; object.header.frame_id = "base";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX; box.dimensions = {.02, .04, .04};
    geometry_msgs::msg::Pose pose; pose.orientation.w = 1; pose.position.x = x;
    object.primitives = {box}; object.primitive_poses = {pose};
    ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  }
};

TEST_F(ControllerTrajectory, LinearSamplesPreserveEndpointsAndJointStep) {
  auto trajectory = path(-.3, .3, 1);
  ASSERT_TRUE(manipulation::sampleControllerTrajectory(trajectory, .025, 10000, samples, error)) << error;
  ASSERT_GT(samples.size(), 2u);
  EXPECT_DOUBLE_EQ(samples.front().getVariablePosition("slide"), -.3);
  EXPECT_DOUBLE_EQ(samples.back().getVariablePosition("slide"), .3);
  for (std::size_t i = 1; i < samples.size(); ++i)
    EXPECT_LE(std::abs(samples[i].getVariablePosition("slide") - samples[i - 1].getVariablePosition("slide")), .025);
}

TEST_F(ControllerTrajectory, CubicOvershootHitsObstacleOutsideWaypointLine) {
  auto trajectory = path(0, .2, 3, 2, -2);
  obstacle(.6);  // q(0.5)=0.6; the old endpoint line only covers [0,0.2].
  EXPECT_FALSE(scene->isStateColliding(trajectory.getFirstWayPoint()));
  EXPECT_FALSE(scene->isStateColliding(trajectory.getLastWayPoint()));
  EXPECT_FALSE(manipulation::validateExternalTrajectoryGeometry(scene, trajectory, error));
  EXPECT_EQ(error.find("EXTERNAL_COLLISION:"), 0u) << error;
}

TEST_F(ControllerTrajectory, QuinticExcursionWithIdenticalPositionsAndVelocities) {
  auto trajectory = path(0, 0, 5, 0, 0, 10, 10);
  obstacle(.3125);  // Exact midpoint of the quintic; both endpoints are zero.
  EXPECT_FALSE(manipulation::validateExternalTrajectoryGeometry(scene, trajectory, error));
  EXPECT_EQ(error.find("EXTERNAL_COLLISION:"), 0u) << error;
}

TEST_F(ControllerTrajectory, SplineJointLimitOvershootIsRejected) {
  auto trajectory = path(0, 0, 3, 8, -8);
  EXPECT_FALSE(manipulation::validateExternalTrajectoryGeometry(scene, trajectory, error));
  EXPECT_EQ(error, "EXTERNAL_JOINT_LIMIT");
}

TEST_F(ControllerTrajectory, BoundUsesWholeCurveInsteadOfEndpointOrMidpointDelta) {
  auto trajectory = path(0, 0, 3, 3, 3);  // q(0)=q(0.5)=q(1)=0, but q(0.25)>0.
  ASSERT_TRUE(manipulation::sampleControllerTrajectory(trajectory, .025, 10000, samples, error)) << error;
  double minimum = 0, maximum = 0;
  for (const auto& sample : samples) {
    minimum = std::min(minimum, sample.getVariablePosition("slide"));
    maximum = std::max(maximum, sample.getVariablePosition("slide"));
  }
  EXPECT_GT(maximum, .28);
  EXPECT_LT(minimum, -.28);
}

TEST_F(ControllerTrajectory, OddNanosecondIntervalsAndStationarySegments) {
  for (double duration : {1e-9, .100000001, 1.000000003}) {
    auto trajectory = path(.1, .1, 5, 0, 0, 0, 0, duration);
    ASSERT_TRUE(manipulation::sampleControllerTrajectory(trajectory, .025, 10000, samples, error)) << error;
    EXPECT_EQ(samples.size(), 2u);
    EXPECT_NEAR(samples.back().getVariablePosition("slide"), .1, 1e-12);
  }
  auto trajectory = path(-.3, .3, 3, 1, -1, 0, 0, .100000001);
  EXPECT_TRUE(manipulation::sampleControllerTrajectory(trajectory, .025, 10000, samples, error)) << error;
}

TEST_F(ControllerTrajectory, SampleBudgetExhaustionIsFailure) {
  auto trajectory = path(-.5, .5, 1);
  EXPECT_FALSE(manipulation::sampleControllerTrajectory(trajectory, .025, 2, samples, error));
  EXPECT_EQ(error, "EXTERNAL_PATH_TOO_LARGE");
}

TEST_F(ControllerTrajectory, InvalidTimeAndNonfiniteDerivativeAreRejected) {
  auto trajectory = path(0, .2, 3, 0, 0, 0, 0, 0);
  EXPECT_FALSE(manipulation::sampleControllerTrajectory(trajectory, .025, 10000, samples, error));
  EXPECT_EQ(error, "CONTROLLER_TRAJECTORY_TIME_NOT_INCREASING");
  trajectory = path(0, .2, 3, std::numeric_limits<double>::quiet_NaN(), 0);
  EXPECT_FALSE(manipulation::sampleControllerTrajectory(trajectory, .025, 10000, samples, error));
  EXPECT_EQ(error, "INVALID_EXTERNAL_JOINT_VALUE");
}
