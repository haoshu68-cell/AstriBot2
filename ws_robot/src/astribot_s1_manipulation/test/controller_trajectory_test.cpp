#include <gtest/gtest.h>
#include <astribot_s1_manipulation/controller_trajectory.hpp>
#include <astribot_s1_manipulation/external_trajectory_validator.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <random>

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
      </joint><link name="other_arm"><collision><geometry><box size="0.02 0.02 0.02"/>
      </geometry></collision></link><joint name="other_arm_mount" type="fixed"><parent link="base"/>
      <child link="other_arm"/><origin xyz="0 0.5 0"/></joint></robot>)");
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

TEST_F(ControllerTrajectory, FixedSeedHermiteExtremaAndInputPreservation) {
  std::mt19937 generator(20260928);
  std::uniform_real_distribution<double> value(-1., 1.);
  for (int trial = 0; trial < 100; ++trial) {
    const int degree = trial % 2 ? 3 : 5;
    const double q0=value(generator),q1=value(generator),v0=3*value(generator),v1=3*value(generator);
    const double a0=8*value(generator),a1=8*value(generator),duration=1.1+value(generator);
    auto trajectory=path(q0,q1,degree,v0,v1,a0,a1,duration);
    moveit_msgs::msg::RobotTrajectory before,after;
    trajectory.getRobotTrajectoryMsg(before);
    ASSERT_TRUE(manipulation::sampleControllerTrajectory(trajectory,.025,10000,samples,error)) << trial << ": " << error;
    trajectory.getRobotTrajectoryMsg(after);
    EXPECT_EQ(before,after);  // Validation must not change the approved path.
    double low=q0,high=q0;
    for(const auto& sample:samples) {
      low=std::min(low,sample.getVariablePosition("slide"));
      high=std::max(high,sample.getVariablePosition("slide"));
    }
    // Independent normalized Hermite polynomial, not the JTC sampler or its
    // Bernstein subdivision. Dense extrema comparison is regression evidence,
    // not a continuous Cartesian collision proof.
    double c[6]={q0,duration*v0,0,0,0,0};
    if(degree==3) {
      c[2]=3*(q1-q0)-duration*(2*v0+v1);
      c[3]=2*(q0-q1)+duration*(v0+v1);
    }else {
      c[2]=duration*duration*a0/2;
      const double dq=q1-c[0]-c[1]-c[2],dv=duration*v1-c[1]-2*c[2],da=duration*duration*a1-2*c[2];
      c[3]=10*dq-4*dv+da/2;c[4]=-15*dq+7*dv-da;c[5]=6*dq-3*dv+da/2;
    }
    for(int i=0;i<=1000;++i) {
      const double u=i/1000.;double q=c[5];
      for(int k=4;k>=0;--k)q=q*u+c[k];
      EXPECT_GE(q,low-.025-1e-7) << trial;
      EXPECT_LE(q,high+.025+1e-7) << trial;
    }
  }
}

TEST_F(ControllerTrajectory, SweepFindsThinObstacleBetweenSafeSamples) {
  auto trajectory=path(0,.024,1);
  moveit_msgs::msg::CollisionObject object;object.id="thin";object.header.frame_id="base";
  shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.0002,.04,.04};
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.x=.012;
  object.primitives={box};object.primitive_poses={pose};ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  ASSERT_TRUE(manipulation::validateExternalTrajectoryGeometry(scene,trajectory,error)) << error;
  auto result=manipulation::checkControllerSweep(scene,trajectory,0.,10000,1.);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::RISK) << result.reason;
  EXPECT_TRUE(std::isnan(result.clearance_lower_bound));
}

TEST_F(ControllerTrajectory, SweepCertifiesClearanceAndReportsBudgetExhaustion) {
  auto trajectory=path(0,.2,1);obstacle(.8);
  auto result=manipulation::checkControllerSweep(scene,trajectory,.05,10000,1.);
  ASSERT_EQ(result.verdict,manipulation::SweepVerdict::CLEAR) << result.reason;
  EXPECT_GT(result.clearance_lower_bound,.05);
  EXPECT_LE(result.clearance_lower_bound,.48); // Fixed other arm limits global clearance.
  result=manipulation::checkControllerSweep(scene,trajectory,.05,1,1.);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::UNKNOWN);
  EXPECT_EQ(result.reason,"SWEEP_BUDGET_EXHAUSTED");
  result=manipulation::checkControllerSweep(scene,trajectory,.05,10000,1e-12);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::UNKNOWN);
}

TEST_F(ControllerTrajectory, SweepChecksOtherArmOutsideTheTrajectoryGroup) {
  auto trajectory=path(0,.2,1);
  moveit_msgs::msg::CollisionObject object;object.id="side_obstacle";object.header.frame_id="base";
  shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.02,.02,.02};
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.y=.5;
  object.primitives={box};object.primitive_poses={pose};ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  auto result=manipulation::checkControllerSweep(scene,trajectory,0.,10000,1.);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::RISK) << result.reason;
}

TEST_F(ControllerTrajectory, SweepIncludesAttachedPayload) {
  moveit_msgs::msg::AttachedCollisionObject attached;attached.link_name="tip";attached.touch_links={"tip"};
  attached.object.id="payload";attached.object.header.frame_id="tip";
  shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.04,.04,.04};
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.x=.3;
  attached.object.primitives={box};attached.object.primitive_poses={pose};
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  auto trajectory=path(0,.2,1);obstacle(.5);
  for(size_t i=0;i<trajectory.getWayPointCount();++i) {
    const double q=trajectory.getWayPoint(i).getVariablePosition("slide");
    *trajectory.getWayPointPtr(i)=scene->getCurrentState();
    trajectory.getWayPointPtr(i)->setVariablePosition("slide",q);
  }
  auto result=manipulation::checkControllerSweep(scene,trajectory,0.,10000,1.);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::RISK) << result.reason;
}

TEST_F(ControllerTrajectory, RevoluteOffsetAndMimicBodiesAreBounded) {
  auto urdf=urdf::parseURDF(R"(<robot name="rotor_test"><link name="base"/>
    <link name="tip"><collision><origin xyz="1 0 0"/><geometry><sphere radius="0.005"/></geometry></collision></link>
    <joint name="slide" type="revolute"><parent link="base"/><child link="tip"/><origin xyz="0.2 0 0"/>
      <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="5"/></joint>
    <link name="finger"><collision><origin xyz="0.3 0 0"/><geometry><sphere radius="0.005"/></geometry></collision></link>
    <joint name="finger_joint" type="revolute"><parent link="base"/><child link="finger"/><origin xyz="0 2 0"/>
      <axis xyz="0 0 1"/><limit lower="-6" upper="6" effort="10" velocity="10"/>
      <mimic joint="slide" multiplier="2" offset="0"/></joint></robot>)");
  ASSERT_TRUE(urdf);auto srdf=std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf,R"(<robot name="rotor_test"><group name="slide_group"><joint name="slide"/></group></robot>)"));
  model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  scene=std::make_shared<planning_scene::PlanningScene>(model);
  auto trajectory=path(-1,1,1);
  auto result=manipulation::checkControllerSweep(scene,trajectory,.01,10000,1.);
  ASSERT_EQ(result.verdict,manipulation::SweepVerdict::CLEAR) << result.reason;
  obstacle(1.2);
  EXPECT_FALSE(scene->isStateColliding(trajectory.getFirstWayPoint()));
  EXPECT_FALSE(scene->isStateColliding(trajectory.getLastWayPoint()));
  result=manipulation::checkControllerSweep(scene,trajectory,0.,10000,1.);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::RISK) << result.reason;
  scene=std::make_shared<planning_scene::PlanningScene>(model);
  moveit_msgs::msg::CollisionObject object;object.id="mimic_obstacle";object.header.frame_id="base";
  shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.001,.001,.001};
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;pose.position.x=.3;pose.position.y=2;
  object.primitives={box};object.primitive_poses={pose};ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  result=manipulation::checkControllerSweep(scene,trajectory,0.,10000,1.);
  EXPECT_EQ(result.verdict,manipulation::SweepVerdict::RISK) << result.reason;
}
