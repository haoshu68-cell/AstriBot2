#include <gtest/gtest.h>
#include <astribot_s1_manipulation/external_trajectory_validator.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <limits>

namespace manipulation=astribot_s1_manipulation;

class TrajectoryTimeScaling : public ::testing::Test {
protected:
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  void SetUp() override {
    const auto urdf=urdf::parseURDF(R"(<robot name="time_scaling_test">
      <link name="base"/><link name="tip"><collision><geometry><box size="0.02 0.02 0.02"/>
      </geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/>
      <child link="tip"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint></robot>)");
    ASSERT_TRUE(urdf);auto srdf=std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf,R"(<robot name="time_scaling_test"><group name="slide_group">
      <joint name="slide"/></group></robot>)"));
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    scene=std::make_shared<planning_scene::PlanningScene>(model);
  }
  robot_trajectory::RobotTrajectory path() const {
    robot_trajectory::RobotTrajectory result(model,"slide_group");
    moveit::core::RobotState state(model);state.setToDefaultValues();
    for(double position:{0.,.15,-.05,.35}) {
      state.setVariablePosition("slide",position);state.update();result.addSuffixWayPoint(state,0.);
    }
    return result;
  }
  static moveit_msgs::msg::RobotTrajectory message(const robot_trajectory::RobotTrajectory& path) {
    moveit_msgs::msg::RobotTrajectory result;path.getRobotTrajectoryMsg(result);return result;
  }
};

TEST_F(TrajectoryTimeScaling, DefaultAndExplicitOnePreserveExistingParameterization) {
  auto baseline=path(),defaulted=path(),explicit_one=path();
  manipulation::TrajectoryTimeOptimizer optimizer;manipulation::TimeOptimizerParams params;
  params.enable_optimization=false;params.baseline_velocity_scaling=.1;params.baseline_acceleration_scaling=.1;
  std::string error;manipulation::OptimizationResult metrics;
  ASSERT_TRUE(optimizer.configure(params,error));
  ASSERT_EQ(optimizer.optimize(baseline,metrics),manipulation::PlanErrorCode::kSuccess);
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,defaulted,error,.1,.1))<<error;
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,explicit_one,error,.1,.1,1.))<<error;
  EXPECT_EQ(message(baseline),message(defaulted));EXPECT_EQ(message(baseline),message(explicit_one));
}

TEST_F(TrajectoryTimeScaling, ScalesTimeAndDerivativesWithoutChangingPositions) {
  auto baseline=path(),scaled=path();std::string error;
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,baseline,error,.1,.1))<<error;
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,scaled,error,.1,.1,2.5))<<error;
  for(size_t i=0;i<baseline.getWayPointCount();++i) {
    const auto& a=baseline.getWayPoint(i);const auto& b=scaled.getWayPoint(i);
    EXPECT_DOUBLE_EQ(b.getVariablePosition("slide"),a.getVariablePosition("slide"));
    EXPECT_DOUBLE_EQ(scaled.getWayPointDurationFromPrevious(i),baseline.getWayPointDurationFromPrevious(i)*2.5);
    ASSERT_TRUE(a.hasVelocities());ASSERT_TRUE(a.hasAccelerations());
    EXPECT_DOUBLE_EQ(b.getVariableVelocity("slide"),a.getVariableVelocity("slide")/2.5);
    EXPECT_DOUBLE_EQ(b.getVariableAcceleration("slide"),a.getVariableAcceleration("slide")/6.25);
  }
}

TEST_F(TrajectoryTimeScaling, RejectsInvalidFactorsBeforeMutatingTrajectory) {
  for(double factor:{0.,.999,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    auto value=path();const auto before=message(value);std::string error;
    EXPECT_FALSE(manipulation::validateExternalTrajectory(scene,value,error,.1,.1,factor));
    EXPECT_EQ(error,"INVALID_EXTERNAL_TIME_SCALING");EXPECT_EQ(message(value),before);
  }
}

TEST_F(TrajectoryTimeScaling, RejectsScaledDurationThatCannotBeRepresented) {
  auto value=path();std::string error;
  EXPECT_FALSE(manipulation::validateExternalTrajectory(
    scene,value,error,.1,.1,std::numeric_limits<double>::max()));
  EXPECT_EQ(error,"EXTERNAL_TIME_SCALING_DURATION_OVERFLOW");
}

TEST_F(TrajectoryTimeScaling, GeometryRevalidationNeverRetimesOrRescalesTheBoundMessage) {
  auto scaled=path();std::string error;
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,scaled,error,.1,.1,2.5))<<error;
  const auto cached=message(scaled);
  EXPECT_TRUE(manipulation::validateExternalTrajectoryGeometry(scene,scaled,error))<<error;
  EXPECT_TRUE(manipulation::validateExternalTrajectoryGeometry(scene,scaled,error))<<error;
  EXPECT_EQ(message(scaled),cached);
}

TEST_F(TrajectoryTimeScaling, InstalledJtcQuinticSplineHasTheSamePositionCurveAtScaledTime) {
  auto baseline=path(),scaled=path();std::string error;
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,baseline,error,.1,.1))<<error;
  ASSERT_TRUE(manipulation::validateExternalTrajectory(scene,scaled,error,.1,.1,2.5))<<error;
  const auto original=message(baseline).joint_trajectory,slower=message(scaled).joint_trajectory;
  joint_trajectory_controller::Trajectory sampler;
  for(size_t i=1;i<original.points.size();++i) {
    const auto& a=original.points[i-1];const auto& b=original.points[i];
    const auto& slow_a=slower.points[i-1];const auto& slow_b=slower.points[i];
    const int64_t ta=rclcpp::Duration(a.time_from_start).nanoseconds(),tb=rclcpp::Duration(b.time_from_start).nanoseconds();
    const int64_t sa=rclcpp::Duration(slow_a.time_from_start).nanoseconds(),sb=rclcpp::Duration(slow_b.time_from_start).nanoseconds();
    for(double fraction:{.125,.25,.5,.75,.875}) {
      trajectory_msgs::msg::JointTrajectoryPoint fast,slow;
      sampler.interpolate_between_points(rclcpp::Time(ta),a,rclcpp::Time(tb),b,
        rclcpp::Time(ta+int64_t((tb-ta)*fraction)),fast);
      sampler.interpolate_between_points(rclcpp::Time(sa),slow_a,rclcpp::Time(sb),slow_b,
        rclcpp::Time(sa+int64_t((sb-sa)*fraction)),slow);
      ASSERT_EQ(fast.positions.size(),1u);ASSERT_EQ(slow.positions.size(),1u);
      EXPECT_NEAR(slow.positions[0],fast.positions[0],1e-9);
      EXPECT_NEAR(slow.velocities[0],fast.velocities[0]/2.5,1e-9);
      EXPECT_NEAR(slow.accelerations[0],fast.accelerations[0]/6.25,1e-9);
    }
  }
}
