// Copyright 2026 Astribot
//
// 奇异点检测单测。
//
// 核心断言思路：平面 3R 臂完全伸直（所有关节 0）是教科书上的奇异构型 ——
// 此时末端只能沿垂直于臂的方向瞬时运动，沿臂轴方向的运动需要无穷大关节速度，
// 雅可比丢秩。弯曲构型则远离奇异。所以必然有
//     sigma_min(伸直) << sigma_min(弯曲)
// 这个关系与具体数值无关，是几何决定的，可以放心断言。

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "astribot_s1_manipulation/singularity_monitor.hpp"
#include "test_robot_fixture.hpp"

using astribot_s1_manipulation::SingularityMonitor;
using astribot_s1_manipulation::SingularityParams;
using astribot_s1_manipulation::SingularityReport;

namespace
{

/// 把某条臂设成给定关节角。
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

TEST(SingularityMonitor, RejectsInvalidParams) {
  SingularityMonitor monitor;
  std::string error;

  SingularityParams p;
  p.min_singular_value = 0.0;   // 必须 > 0
  EXPECT_FALSE(monitor.configure(p, error));
  EXPECT_FALSE(error.empty());

  p = SingularityParams{};
  p.max_condition_number = 1.0;  // 条件数天然 >= 1，阈值给 1 等于全判奇异
  EXPECT_FALSE(monitor.configure(p, error));

  p = SingularityParams{};
  p.degenerate_jacobian_epsilon = -1.0;
  EXPECT_FALSE(monitor.configure(p, error));

  // 合法参数必须通过
  p = SingularityParams{};
  EXPECT_TRUE(monitor.configure(p, error)) << error;
  EXPECT_TRUE(monitor.isConfigured());
}

TEST(SingularityMonitor, UnconfiguredMonitorReportsInvalid) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  SingularityMonitor monitor;   // 故意不 configure
  const SingularityReport report =
    monitor.check(state, model->getJointModelGroup("arm_left"), "left_tcp");
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.reason.empty());
}

TEST(SingularityMonitor, NullGroupAndMissingLinkAreRejectedWithoutCrashing) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  SingularityMonitor monitor;
  std::string error;
  ASSERT_TRUE(monitor.configure(SingularityParams{}, error)) << error;

  // 空指针组
  EXPECT_FALSE(monitor.check(state, nullptr, "left_tcp").valid);
  // 空 tip 名
  EXPECT_FALSE(monitor.check(state, model->getJointModelGroup("arm_left"), "").valid);
  // 不存在的 link
  EXPECT_FALSE(
    monitor.check(state, model->getJointModelGroup("arm_left"), "no_such_link").valid);
}

TEST(SingularityMonitor, NonChainGroupIsRejected) {
  // 这条很重要：dual_arm 组不是运动链，对它求雅可比没有数学定义。
  // 必须显式拦住，否则 MoveIt 内部会返回垃圾数据或断言失败。
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  const moveit::core::JointModelGroup * dual = model->getJointModelGroup("dual_arm");
  ASSERT_NE(dual, nullptr);
  ASSERT_FALSE(dual->isChain()) << "fixture's dual_arm group should not be a chain";

  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  SingularityMonitor monitor;
  std::string error;
  ASSERT_TRUE(monitor.configure(SingularityParams{}, error)) << error;

  const SingularityReport report = monitor.check(state, dual, "left_tcp");
  EXPECT_FALSE(report.valid);
  EXPECT_NE(report.reason.find("not a kinematic chain"), std::string::npos)
    << "actual reason: " << report.reason;
}

TEST(SingularityMonitor, StraightArmIsMoreSingularThanBentArm) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  const moveit::core::JointModelGroup * left = model->getJointModelGroup("arm_left");
  ASSERT_NE(left, nullptr);
  ASSERT_TRUE(left->isChain());

  SingularityMonitor monitor;
  std::string error;
  // 阈值先给极宽，这一步只比较数值大小，不做判定。
  SingularityParams p;
  p.min_singular_value = 1e-12;
  p.max_condition_number = 1e12;
  ASSERT_TRUE(monitor.configure(p, error)) << error;

  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  // 完全伸直：平面 3R 的经典奇异构型
  setArm(state, "arm_left", {0.0, 0.0, 0.0});
  const SingularityReport straight = monitor.check(state, left, "left_tcp");
  ASSERT_TRUE(straight.valid) << straight.reason;

  // 明显弯曲：远离奇异
  setArm(state, "arm_left", {0.0, 1.0, 1.0});
  const SingularityReport bent = monitor.check(state, left, "left_tcp");
  ASSERT_TRUE(bent.valid) << bent.reason;

  EXPECT_LT(straight.min_singular_value, bent.min_singular_value)
    << "straight sigma_min=" << straight.min_singular_value
    << " bent sigma_min=" << bent.min_singular_value;
  // 条件数方向相反：伸直时更病态
  EXPECT_GT(straight.condition_number, bent.condition_number);
}

TEST(SingularityMonitor, ExactlySingularArmIsFlaggedRegardlessOfThreshold) {
  // 完全伸直的平面 3R 臂是**精确**奇异的，不是"接近"奇异：
  // 实测 sigma_min = 2.27e-17（数值零），低于 degenerate_jacobian_epsilon，
  // 于是走"退化雅可比"分支，条件数报 inf。
  // 这种构型无论阈值给多宽都必须被判奇异 —— 这条断言比"阈值卡在实测值之间"
  // 更强，因为它不依赖任何具体数值。
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  const moveit::core::JointModelGroup * left = model->getJointModelGroup("arm_left");

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  setArm(state, "arm_left", {0.0, 0.0, 0.0});

  SingularityMonitor monitor;
  std::string error;
  SingularityParams wide;
  wide.min_singular_value = 1e-12;    // 极宽
  wide.max_condition_number = 1e12;   // 极宽
  ASSERT_TRUE(monitor.configure(wide, error)) << error;

  const SingularityReport report = monitor.check(state, left, "left_tcp");
  ASSERT_TRUE(report.valid) << report.reason;
  EXPECT_TRUE(report.singular)
    << "a fully extended planar 3R arm is exactly rank-deficient; sigma_min="
    << report.min_singular_value << " cond=" << report.condition_number;
  // 除零保护必须生效：sigma_min 近零时条件数报 inf，而不是一个巨大的假有限值。
  EXPECT_FALSE(std::isfinite(report.condition_number))
    << "condition number should be infinite for an exactly singular jacobian";
}

TEST(SingularityMonitor, ThresholdActuallyFlagsNearSingularArm) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  const moveit::core::JointModelGroup * left = model->getJointModelGroup("arm_left");

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  // 用"几乎伸直"而不是"精确伸直"：实测 sigma_min = 2.45e-3，是有限正数，
  // 这样才能验证"阈值卡在实测值之上就必须报奇异"这条判定逻辑本身。
  setArm(state, "arm_left", {0.0, 0.02, 0.0});

  // 先量出实际 sigma_min，断言不依赖硬编码数值。
  SingularityMonitor loose;
  std::string error;
  SingularityParams wide;
  wide.min_singular_value = 1e-12;
  wide.max_condition_number = 1e12;
  ASSERT_TRUE(loose.configure(wide, error)) << error;
  const SingularityReport measured = loose.check(state, left, "left_tcp");
  ASSERT_TRUE(measured.valid) << measured.reason;
  ASSERT_GT(measured.min_singular_value, 0.0);
  ASSERT_TRUE(std::isfinite(measured.condition_number));
  EXPECT_FALSE(measured.singular)
    << "a slightly bent arm should pass wide thresholds; sigma_min="
    << measured.min_singular_value;

  // 把 sigma_min 阈值卡到实测值之上 -> 必须报奇异，且原因指明是 sigma_min。
  SingularityMonitor strict;
  SingularityParams tight;
  tight.min_singular_value = measured.min_singular_value * 2.0;
  tight.max_condition_number = 1e12;   // 只让 sigma_min 这条判据起作用
  ASSERT_TRUE(strict.configure(tight, error)) << error;
  const SingularityReport flagged = strict.check(state, left, "left_tcp");
  ASSERT_TRUE(flagged.valid) << flagged.reason;
  EXPECT_TRUE(flagged.singular);
  EXPECT_NE(flagged.reason.find("sigma_min"), std::string::npos) << flagged.reason;

  // 反过来只让条件数判据起作用，也必须能独立触发。
  SingularityMonitor by_condition;
  SingularityParams cond_only;
  cond_only.min_singular_value = 1e-12;   // 这条不触发
  cond_only.max_condition_number = measured.condition_number * 0.5;
  ASSERT_TRUE(by_condition.configure(cond_only, error)) << error;
  const SingularityReport cond_flagged = by_condition.check(state, left, "left_tcp");
  ASSERT_TRUE(cond_flagged.valid) << cond_flagged.reason;
  EXPECT_TRUE(cond_flagged.singular);
  EXPECT_NE(cond_flagged.reason.find("condition_number"), std::string::npos)
    << cond_flagged.reason;
}

TEST(SingularityMonitor, DisabledMonitorStillReportsNumbersButNeverFlags) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  const moveit::core::JointModelGroup * left = model->getJointModelGroup("arm_left");

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  setArm(state, "arm_left", {0.0, 0.0, 0.0});

  SingularityMonitor monitor;
  std::string error;
  SingularityParams p;
  p.enabled = false;
  p.min_singular_value = 1e9;   // 极端阈值，开启时必然判奇异
  ASSERT_TRUE(monitor.configure(p, error)) << error;

  const SingularityReport report = monitor.check(state, left, "left_tcp");
  EXPECT_TRUE(report.valid);
  EXPECT_FALSE(report.singular) << "disabled monitor must never flag";
  // 关掉判定但仍要给出数值，便于观察轨迹的奇异余量。
  EXPECT_GT(report.max_singular_value, 0.0);
}

TEST(SingularityMonitor, CheckStatesReturnsFirstSingularWaypoint) {
  auto model = astribot_test::makeRobotModel();
  ASSERT_TRUE(model != nullptr);
  const moveit::core::JointModelGroup * left = model->getJointModelGroup("arm_left");

  moveit::core::RobotState bent(model);
  bent.setToDefaultValues();
  setArm(bent, "arm_left", {0.0, 1.0, 1.0});

  moveit::core::RobotState straight(model);
  straight.setToDefaultValues();
  setArm(straight, "arm_left", {0.0, 0.0, 0.0});

  // 先测出弯曲/伸直的实际值，把阈值卡在两者之间。
  SingularityMonitor loose;
  std::string error;
  SingularityParams wide;
  wide.min_singular_value = 1e-12;
  wide.max_condition_number = 1e12;
  ASSERT_TRUE(loose.configure(wide, error)) << error;
  const double sv_bent = loose.check(bent, left, "left_tcp").min_singular_value;
  const double sv_straight = loose.check(straight, left, "left_tcp").min_singular_value;
  ASSERT_LT(sv_straight, sv_bent);

  SingularityMonitor monitor;
  SingularityParams p;
  p.min_singular_value = 0.5 * (sv_bent + sv_straight);
  p.max_condition_number = 1e12;
  ASSERT_TRUE(monitor.configure(p, error)) << error;

  // 第 2 个点(下标 2)是奇异的，应该被精确定位出来。
  const std::vector<moveit::core::RobotState> states = {bent, bent, straight, bent};
  std::size_t worst_index = 0;
  const SingularityReport report =
    monitor.checkStates(states, left, "left_tcp", &worst_index);
  EXPECT_TRUE(report.singular);
  EXPECT_EQ(worst_index, 2U);

  // 全是非奇异点时不应报奇异
  const std::vector<moveit::core::RobotState> good = {bent, bent, bent};
  const SingularityReport ok = monitor.checkStates(good, left, "left_tcp", &worst_index);
  EXPECT_FALSE(ok.singular);
  EXPECT_TRUE(ok.valid);

  // 空输入不能崩
  const SingularityReport empty = monitor.checkStates({}, left, "left_tcp", nullptr);
  EXPECT_FALSE(empty.valid);
}
