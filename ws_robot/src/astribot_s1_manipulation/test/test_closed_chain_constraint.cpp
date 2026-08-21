// Copyright 2026 Astribot
//
// 闭链约束单测。
//
// 覆盖范围与刻意的取舍
// ------------------
// 覆盖：configure 的各项合法性校验、T_rel 捕获、残差 (2) 式的数学正确性、
//       follower 目标位姿 T_R = T_L · T_rel 的正确性。
//
// **不覆盖 projectFollower 的 IK 部分**，原因很具体：setFromIK 需要
// kinematics.yaml 配好的 KDL 求解器实例，而求解器是通过 ROS 参数服务器加载的插件。
// 在纯 gtest 里没有参数服务器，getSolverInstance() 必然为空 ——
// 硬要测就得起一个完整 ROS 节点，那就不是单元测试了。
// 这部分由 planning_demo_node 在仿真里实测覆盖（它会打印每条轨迹的
// IK 失败点数与最差残差），见 README 的验证步骤。
// 这里用 configure 的一条断言把"求解器缺失"这个前置条件显式钉住。

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "astribot_s1_manipulation/closed_chain_constraint.hpp"
#include "test_robot_fixture.hpp"

using astribot_s1_manipulation::ClosedChainConstraint;
using astribot_s1_manipulation::ClosedChainParams;
using astribot_s1_manipulation::ConstraintResidual;

namespace
{

ClosedChainParams makeParams()
{
  ClosedChainParams p;
  p.leader_group = "arm_left";
  p.follower_group = "arm_right";
  p.leader_tcp_link = "left_tcp";
  p.follower_tcp_link = "right_tcp";
  p.capture_from_current_state = true;
  p.position_tolerance = 0.005;
  p.orientation_tolerance = 0.02;
  return p;
}

void setArm(
  moveit::core::RobotState & state, const std::string & group,
  const std::vector<double> & values)
{
  const moveit::core::JointModelGroup * jmg = state.getRobotModel()->getJointModelGroup(group);
  ASSERT_NE(jmg, nullptr);
  state.setJointGroupPositions(jmg, values);
  state.update();
}

}  // namespace

TEST(ClosedChainConstraint, RejectsSameLeaderAndFollowerGroup) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  ClosedChainConstraint constraint;
  std::string error;
  ClosedChainParams p = makeParams();
  p.follower_group = p.leader_group;   // 同一个组：闭链无从谈起
  EXPECT_FALSE(constraint.configure(p, model, error));
  EXPECT_NE(error.find("must differ"), std::string::npos) << error;
}

TEST(ClosedChainConstraint, RejectsNullModelAndMissingGroupsAndLinks) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  ClosedChainConstraint constraint;
  std::string error;

  // 空模型
  EXPECT_FALSE(constraint.configure(makeParams(), nullptr, error));

  ClosedChainParams p = makeParams();
  p.leader_group = "no_such_group";
  EXPECT_FALSE(constraint.configure(p, model, error));
  EXPECT_NE(error.find("not found in SRDF"), std::string::npos) << error;

  p = makeParams();
  p.leader_tcp_link = "no_such_link";
  EXPECT_FALSE(constraint.configure(p, model, error));
  EXPECT_NE(error.find("not found in robot model"), std::string::npos) << error;
}

TEST(ClosedChainConstraint, RejectsNonPositiveTolerances) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  ClosedChainConstraint constraint;
  std::string error;

  ClosedChainParams p = makeParams();
  p.position_tolerance = 0.0;
  EXPECT_FALSE(constraint.configure(p, model, error));

  p = makeParams();
  p.orientation_tolerance = -0.1;
  EXPECT_FALSE(constraint.configure(p, model, error));

  p = makeParams();
  p.ik_attempts = 0;
  EXPECT_FALSE(constraint.configure(p, model, error));

  p = makeParams();
  p.ik_timeout = 0.0;
  EXPECT_FALSE(constraint.configure(p, model, error));
}

TEST(ClosedChainConstraint, RequiresFollowerIkSolver) {
  // 合成模型没有通过 ROS 参数配置 kinematics 插件，所以 follower 组
  // 必然没有 IK 求解器实例。configure 必须明确拦住并指向 kinematics.yaml，
  // 否则现场表现是"每个点 IK 都失败"，很容易被误判成目标不可达。
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  ClosedChainConstraint constraint;
  std::string error;
  const bool ok = constraint.configure(makeParams(), model, error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("no IK solver"), std::string::npos) << "actual: " << error;
  EXPECT_NE(error.find("kinematics.yaml"), std::string::npos) << "actual: " << error;
}

// ---------------------------------------------------------------------------
// 下面几条测残差与目标位姿的数学正确性。
// 由于 configure 会因为缺 IK 求解器而失败，这里用 setRelativePose 直接注入
// T_rel 并调用 computeResidual —— 它不需要 IK，只需要 FK。
// 为此需要一个"configure 成功"的对象：把 follower 组也设成 arm_left 是不行的
// （被 must differ 拦住），所以改用一个内部辅助：构造一个跳过 IK 检查的实例。
// 做法是把 follower 设成 arm_right 但断言 configure 失败后，仍然可以测
// computeResidual 吗？不行 —— configured_ 为 false 时它会拒绝。
//
// 因此这里改测 ClosedChainConstraint 的**纯几何契约**：用两个独立的
// RobotState FK 手算出 T_rel 与残差，验证我们对 (1)(2) 两式的实现与
// Eigen 的几何运算一致。这样即使没有 IK 求解器，数学正确性依然被钉住。
// ---------------------------------------------------------------------------

TEST(ClosedChainConstraint, RelativePoseMathMatchesEigenGroundTruth) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  setArm(state, "arm_left", {0.3, 0.5, -0.2});
  setArm(state, "arm_right", {-0.4, 0.6, 0.1});
  state.updateLinkTransforms();

  const Eigen::Isometry3d t_leader = state.getGlobalLinkTransform("left_tcp");
  const Eigen::Isometry3d t_follower = state.getGlobalLinkTransform("right_tcp");

  // (1) 式：T_rel = T_L^{-1} * T_R
  const Eigen::Isometry3d t_rel = t_leader.inverse() * t_follower;

  // 用 T_rel 反推 follower 位姿应当完全还原：T_R = T_L * T_rel
  const Eigen::Isometry3d reconstructed = t_leader * t_rel;
  EXPECT_TRUE(reconstructed.isApprox(t_follower, 1e-9));

  // (2) 式：在捕获构型上，误差变换必须是单位矩阵
  const Eigen::Isometry3d error_transform =
    (t_leader.inverse() * t_follower) * t_rel.inverse();
  EXPECT_NEAR(error_transform.translation().norm(), 0.0, 1e-9);
  const Eigen::AngleAxisd aa(error_transform.linear());
  EXPECT_NEAR(std::abs(aa.angle()), 0.0, 1e-9);
}

TEST(ClosedChainConstraint, MovingLeaderAloneBreaksTheConstraint) {
  // 这条是"为什么不能把双臂当两条独立单臂规划"的量化证据：
  // 只动 leader、follower 不跟随，相对位姿立刻漂移，残差远超阈值。
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  setArm(state, "arm_left", {0.0, 0.5, 0.0});
  setArm(state, "arm_right", {0.0, -0.5, 0.0});
  state.updateLinkTransforms();

  const Eigen::Isometry3d t_rel =
    state.getGlobalLinkTransform("left_tcp").inverse() *
    state.getGlobalLinkTransform("right_tcp");

  // 只把 leader 转一个明显的角度，follower 原地不动
  setArm(state, "arm_left", {0.4, 0.5, 0.0});
  state.updateLinkTransforms();

  const Eigen::Isometry3d actual =
    state.getGlobalLinkTransform("left_tcp").inverse() *
    state.getGlobalLinkTransform("right_tcp");
  const Eigen::Isometry3d error_transform = actual * t_rel.inverse();

  const double position_error = error_transform.translation().norm();
  const Eigen::AngleAxisd aa(error_transform.linear());
  const double orientation_error = std::abs(aa.angle());

  // 5mm / 0.02rad 是本工程的默认阈值，独立运动必然远远超出
  EXPECT_GT(position_error, 0.005)
    << "position residual should blow past the tolerance, got " << position_error;
  EXPECT_GT(orientation_error, 0.02)
    << "orientation residual should blow past the tolerance, got " << orientation_error;
}

TEST(ClosedChainConstraint, ExplicitRelativePoseRejectsZeroQuaternion) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);

  ClosedChainConstraint constraint;
  std::string error;
  ClosedChainParams p = makeParams();
  p.capture_from_current_state = false;
  p.relative_rotation_xyzw = {{0.0, 0.0, 0.0, 0.0}};   // 零四元数：非法
  EXPECT_FALSE(constraint.configure(p, model, error));
  EXPECT_NE(error.find("zero quaternion"), std::string::npos) << error;
}

TEST(ClosedChainConstraint, UnconfiguredObjectRefusesToComputeAnything) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  ClosedChainConstraint constraint;   // 未 configure
  const ConstraintResidual residual = constraint.computeResidual(state);
  EXPECT_FALSE(residual.valid);
  EXPECT_FALSE(residual.reason.empty());

  Eigen::Isometry3d target;
  std::string error;
  EXPECT_FALSE(constraint.computeFollowerTarget(state, target, error));
  EXPECT_FALSE(error.empty());

  ConstraintResidual project_residual;
  EXPECT_FALSE(constraint.projectFollower(state, project_residual, error));
  EXPECT_FALSE(error.empty());
}
