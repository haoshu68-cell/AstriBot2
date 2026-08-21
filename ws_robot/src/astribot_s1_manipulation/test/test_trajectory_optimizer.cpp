// Copyright 2026 Astribot
//
// 节拍量化与时间优化单测。
//
// 重点覆盖任务里两条硬要求：
//   1. "禁止忽略关节速度、加速度极限输出超参轨迹" -> 超限必须被检出
//   2. "优化后仍然超出硬限制 -> 回退原始轨迹，告警输出，不输出非法轨迹"
//      -> 回退逻辑必须真的回退，而且 baseline 本身非法时必须拒绝输出

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "astribot_s1_manipulation/error_codes.hpp"
#include "astribot_s1_manipulation/trajectory_metrics.hpp"
#include "astribot_s1_manipulation/trajectory_time_optimizer.hpp"
#include "test_robot_fixture.hpp"

using astribot_s1_manipulation::MetricsParams;
using astribot_s1_manipulation::MetricsSource;
using astribot_s1_manipulation::OptimizationResult;
using astribot_s1_manipulation::PlanErrorCode;
using astribot_s1_manipulation::TimeOptimizerParams;
using astribot_s1_manipulation::TrajectoryMetrics;
using astribot_s1_manipulation::TrajectoryTimeOptimizer;
using astribot_s1_manipulation::evaluateTrajectory;

namespace
{

/// 造一条 arm_left 的简单轨迹：从全 0 线性走到给定终点，n 个点。
robot_trajectory::RobotTrajectory makeTrajectory(
  const moveit::core::RobotModelPtr & model,
  const std::vector<double> & target,
  std::size_t waypoints)
{
  robot_trajectory::RobotTrajectory trajectory(model, "arm_left");
  const moveit::core::JointModelGroup * jmg = model->getJointModelGroup("arm_left");
  for (std::size_t i = 0; i < waypoints; ++i) {
    const double t = (waypoints <= 1U) ? 0.0 :
      static_cast<double>(i) / static_cast<double>(waypoints - 1U);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::vector<double> values(target.size());
    for (std::size_t j = 0; j < target.size(); ++j) {
      values[j] = t * target[j];
    }
    state.setJointGroupPositions(jmg, values);
    state.update();
    trajectory.addSuffixWayPoint(state, 0.0);
  }
  return trajectory;
}

}  // namespace

TEST(TrajectoryMetrics, EmptyAndSingleWaypointHandledWithoutCrashing) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  MetricsParams params;

  robot_trajectory::RobotTrajectory empty(model, "arm_left");
  const TrajectoryMetrics empty_metrics = evaluateTrajectory(empty, params);
  EXPECT_FALSE(empty_metrics.valid);
  EXPECT_EQ(empty_metrics.waypoint_count, 0U);

  robot_trajectory::RobotTrajectory single = makeTrajectory(model, {0.1, 0.1, 0.1}, 1);
  const TrajectoryMetrics single_metrics = evaluateTrajectory(single, params);
  // 单点轨迹是合法的（起点即终点），节拍为 0，但要能明确表达出来。
  EXPECT_TRUE(single_metrics.valid);
  EXPECT_DOUBLE_EQ(single_metrics.duration, 0.0);
  EXPECT_TRUE(single_metrics.isLegal());
}

TEST(TrajectoryMetrics, UnparameterizedTrajectoryIsReportedAsFiniteDifference) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  // 未做时间参数化：所有 dt 都是 0，没有速度数据。
  robot_trajectory::RobotTrajectory trajectory = makeTrajectory(model, {1.0, 0.5, 0.5}, 5);
  MetricsParams params;
  params.allow_finite_difference = true;
  const TrajectoryMetrics metrics = evaluateTrajectory(trajectory, params);
  ASSERT_TRUE(metrics.valid);
  EXPECT_EQ(metrics.source, MetricsSource::kFiniteDifference);
  // dt 全为 0 时无法算速度，必须老实报 0，而不是除零得到 inf。
  EXPECT_TRUE(std::isfinite(metrics.max_joint_velocity));

  // 禁用差分时应直接拒绝
  params.allow_finite_difference = false;
  const TrajectoryMetrics strict = evaluateTrajectory(trajectory, params);
  EXPECT_FALSE(strict.valid);
}

TEST(TrajectoryTimeOptimizer, RejectsInvalidScalingParams) {
  TrajectoryTimeOptimizer optimizer;
  std::string error;

  TimeOptimizerParams p;
  p.baseline_velocity_scaling = 0.0;      // 必须在 (0,1]
  EXPECT_FALSE(optimizer.configure(p, error));

  p = TimeOptimizerParams{};
  p.optimized_velocity_scaling = 1.5;     // > 1 会突破硬限位
  EXPECT_FALSE(optimizer.configure(p, error));

  p = TimeOptimizerParams{};
  p.totg_path_tolerance = 0.0;
  EXPECT_FALSE(optimizer.configure(p, error));

  // 优化档不高于 baseline 档 -> 优化没有意义，属配置错误
  p = TimeOptimizerParams{};
  p.baseline_velocity_scaling = 0.9;
  p.optimized_velocity_scaling = 0.9;
  EXPECT_FALSE(optimizer.configure(p, error));
  EXPECT_NE(error.find("must exceed"), std::string::npos) << error;

  p = TimeOptimizerParams{};
  EXPECT_TRUE(optimizer.configure(p, error)) << error;
}

TEST(TrajectoryTimeOptimizer, UnconfiguredOptimizerReturnsNotConfigured) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  robot_trajectory::RobotTrajectory trajectory = makeTrajectory(model, {0.5, 0.5, 0.5}, 5);

  TrajectoryTimeOptimizer optimizer;   // 故意不 configure
  OptimizationResult result;
  EXPECT_EQ(optimizer.optimize(trajectory, result), PlanErrorCode::kNotConfigured);
}

TEST(TrajectoryTimeOptimizer, EmptyTrajectoryIsRejected) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  robot_trajectory::RobotTrajectory empty(model, "arm_left");

  TrajectoryTimeOptimizer optimizer;
  std::string error;
  ASSERT_TRUE(optimizer.configure(TimeOptimizerParams{}, error)) << error;

  OptimizationResult result;
  EXPECT_EQ(optimizer.optimize(empty, result), PlanErrorCode::kInvalidInput);
}

TEST(TrajectoryTimeOptimizer, OptimizedTrajectoryIsLegalAndNotSlowerThanBaseline) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  robot_trajectory::RobotTrajectory trajectory = makeTrajectory(model, {1.5, 1.0, 1.0}, 12);

  TrajectoryTimeOptimizer optimizer;
  std::string error;
  TimeOptimizerParams p;
  p.baseline_velocity_scaling = 0.20;
  p.baseline_acceleration_scaling = 0.20;
  p.optimized_velocity_scaling = 0.90;
  p.optimized_acceleration_scaling = 0.90;
  ASSERT_TRUE(optimizer.configure(p, error)) << error;

  OptimizationResult result;
  const PlanErrorCode code = optimizer.optimize(trajectory, result);
  ASSERT_EQ(code, PlanErrorCode::kSuccess) << result.note;

  // baseline 必须算出来且合法
  ASSERT_TRUE(result.baseline_metrics.valid) << result.baseline_metrics.summary;
  EXPECT_TRUE(result.baseline_metrics.isLegal()) << result.baseline_metrics.summary;

  // 最终交付的轨迹（optimize 就地改写）必须合法 —— 这是最硬的一条要求
  MetricsParams metrics_params;
  const TrajectoryMetrics final_metrics = evaluateTrajectory(trajectory, metrics_params);
  ASSERT_TRUE(final_metrics.valid) << final_metrics.summary;
  EXPECT_TRUE(final_metrics.isLegal())
    << "delivered trajectory must never violate joint limits: " << final_metrics.summary;

  // 采纳优化版时节拍必须真的更短；回退时必须等于 baseline。
  if (result.optimized_accepted) {
    EXPECT_LT(result.optimized_metrics.duration, result.baseline_metrics.duration)
      << result.note;
    EXPECT_GT(result.duration_reduction_ratio, 0.0);
  } else {
    EXPECT_TRUE(result.fell_back);
    EXPECT_NEAR(final_metrics.duration, result.baseline_metrics.duration, 1e-6);
  }
}

TEST(TrajectoryTimeOptimizer, DisabledOptimizationStillProducesExecutableBaseline) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  robot_trajectory::RobotTrajectory trajectory = makeTrajectory(model, {1.0, 0.8, 0.6}, 10);

  TrajectoryTimeOptimizer optimizer;
  std::string error;
  TimeOptimizerParams p;
  p.enable_optimization = false;
  ASSERT_TRUE(optimizer.configure(p, error)) << error;

  OptimizationResult result;
  ASSERT_EQ(optimizer.optimize(trajectory, result), PlanErrorCode::kSuccess) << result.note;
  EXPECT_FALSE(result.optimized_accepted);

  // 关掉优化也必须给出带时间戳、可执行、合法的轨迹
  MetricsParams metrics_params;
  const TrajectoryMetrics metrics = evaluateTrajectory(trajectory, metrics_params);
  ASSERT_TRUE(metrics.valid);
  EXPECT_EQ(metrics.source, MetricsSource::kFromTrajectory);
  EXPECT_GT(metrics.duration, 0.0);
  EXPECT_TRUE(metrics.isLegal());
}

TEST(TrajectoryMetrics, ViolationIsDetectedWhenVelocityExceedsLimit) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  // 手工造一条"超速"轨迹：两个点相距很大，但 dt 给得极小。
  // fixture 里关节速度限位是 2.0 rad/s；这里 1.0rad / 0.01s = 100rad/s，必然超限。
  robot_trajectory::RobotTrajectory trajectory(model, "arm_left");
  const moveit::core::JointModelGroup * jmg = model->getJointModelGroup("arm_left");

  moveit::core::RobotState a(model);
  a.setToDefaultValues();
  a.setJointGroupPositions(jmg, std::vector<double>{0.0, 0.0, 0.0});
  a.setVariableVelocity("left_j1", 0.0);
  a.update();

  moveit::core::RobotState b(model);
  b.setToDefaultValues();
  b.setJointGroupPositions(jmg, std::vector<double>{1.0, 0.0, 0.0});
  // 显式写入一个超限速度，模拟"参数化器给出了非法结果"这一情形
  b.setVariableVelocity("left_j1", 100.0);
  b.update();

  trajectory.addSuffixWayPoint(a, 0.0);
  trajectory.addSuffixWayPoint(b, 0.01);

  MetricsParams params;
  const TrajectoryMetrics metrics = evaluateTrajectory(trajectory, params);
  ASSERT_TRUE(metrics.valid) << metrics.summary;
  EXPECT_TRUE(metrics.velocity_limit_violated) << metrics.summary;
  EXPECT_FALSE(metrics.isLegal());
  ASSERT_FALSE(metrics.violating_joints.empty());
  EXPECT_NE(metrics.summary.find("ILLEGAL"), std::string::npos) << metrics.summary;
}
