// Copyright 2026 Astribot
//
// OptimizingPlannerWrapper 的单元测试。
//
// 这里**不**加载真实机器人模型、不起 move_group：直接在 OMPL 的
// RealVectorStateSpace 上造一个最小规划问题。理由是本包装层的行为
// （何时停止优化、委托前等不等目标状态）与机器人无关，纯粹是 OMPL
// 的 solve()/PTC/GoalLazySamples 语义，用合成问题能测得更准也更快。

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "astribot_s1_manipulation/optimizing_planner_wrapper.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using astribot_s1_manipulation::OptimizingPlannerPolicy;
using astribot_s1_manipulation::OptimizingPlannerWrapper;
using astribot_s1_manipulation::extractOptimizingPolicy;

namespace
{

rclcpp::Logger testLogger()
{
  return rclcpp::get_logger("test_optimizing_planner_wrapper");
}

/// 一个 4 维、无障碍的规划问题。无障碍是刻意的：RRT* 会在头几次迭代就
/// 拿到近乎最优的直线解，之后再也改进不了 —— 这正是要复现的"首解即最优、
/// 于是烧完超时"的场景。
class OpenSpaceProblem
{
public:
  OpenSpaceProblem()
  {
    auto space = std::make_shared<ob::RealVectorStateSpace>(kDim);
    ob::RealVectorBounds bounds(kDim);
    bounds.setLow(-3.14);
    bounds.setHigh(3.14);
    space->setBounds(bounds);

    setup_ = std::make_shared<og::SimpleSetup>(space);
    setup_->setStateValidityChecker(
      [](const ob::State *) {return true;});           // 全空间无碰撞

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    for (unsigned int i = 0; i < kDim; ++i) {
      start[i] = 0.0;
      goal[i] = 1.0;
    }
    setup_->setStartAndGoalStates(start, goal, 1e-3);
    setup_->setOptimizationObjective(
      std::make_shared<ob::PathLengthOptimizationObjective>(setup_->getSpaceInformation()));
  }

  ob::SpaceInformationPtr spaceInformation() const {return setup_->getSpaceInformation();}
  ob::ProblemDefinitionPtr problemDefinition() const {return setup_->getProblemDefinition();}

  /// 挂上规划器并 solve，返回实际耗时（秒）。
  double solveFor(const ob::PlannerPtr & planner, double timeout_sec)
  {
    planner->setProblemDefinition(setup_->getProblemDefinition());
    planner->setup();
    const auto t0 = std::chrono::steady_clock::now();
    last_status_ = planner->solve(ob::timedPlannerTerminationCondition(timeout_sec));
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t1 - t0).count();
  }

  ob::PlannerStatus lastStatus() const {return last_status_;}

private:
  static constexpr unsigned int kDim = 4U;
  og::SimpleSetupPtr setup_;
  ob::PlannerStatus last_status_{ob::PlannerStatus::UNKNOWN};
};

}  // namespace

// ---------------------------------------------------------------------------
// yaml 键的解析与摘除
// ---------------------------------------------------------------------------

TEST(ExtractOptimizingPolicy, ConsumesItsOwnKeysAndLeavesTheRest)
{
  std::map<std::string, std::string> config = {
    {"type", "geometric::RRTstar"},
    {"goal_bias", "0.05"},
    {"optimization_budget_sec", "0.5"},
    {"goal_state_wait_sec", "0.1"},
  };

  const OptimizingPlannerPolicy policy =
    extractOptimizingPolicy(config, "RRTstarConfig", testLogger());

  EXPECT_DOUBLE_EQ(policy.optimization_budget_sec, 0.5);
  EXPECT_DOUBLE_EQ(policy.goal_state_wait_sec, 0.1);
  EXPECT_TRUE(policy.enabled());

  // 必须从 config 里删掉：留着会被喂给 OMPL 的 ParamSet，触发
  // "has no parameter named" WARN，污染那条本来用来抓拼写错误的告警。
  EXPECT_EQ(config.count("optimization_budget_sec"), 0U);
  EXPECT_EQ(config.count("goal_state_wait_sec"), 0U);
  // 其余键一个都不能动。
  EXPECT_EQ(config.size(), 2U);
  EXPECT_EQ(config.at("type"), "geometric::RRTstar");
  EXPECT_EQ(config.at("goal_bias"), "0.05");
}

TEST(ExtractOptimizingPolicy, MissingKeysMeanDisabled)
{
  std::map<std::string, std::string> config = {{"goal_bias", "0.05"}};
  const OptimizingPlannerPolicy policy =
    extractOptimizingPolicy(config, "BITstarConfig", testLogger());

  EXPECT_DOUBLE_EQ(policy.optimization_budget_sec, 0.0);
  EXPECT_DOUBLE_EQ(policy.goal_state_wait_sec, 0.0);
  EXPECT_FALSE(policy.enabled());
  EXPECT_EQ(config.size(), 1U);
}

TEST(ExtractOptimizingPolicy, GarbageAndNegativeValuesDegradeToDisabledWithoutThrowing)
{
  // 写错值不能抛异常（会穿出 allocator 直接打死 move_group），
  // 也不能悄悄用一个"看起来合理"的数 —— 只能退回不启用并打 WARN。
  for (const char * bad : {"abc", "0.5s", "", "-1.0"}) {
    std::map<std::string, std::string> config = {{"optimization_budget_sec", bad}};
    OptimizingPlannerPolicy policy;
    EXPECT_NO_THROW(
      policy = extractOptimizingPolicy(config, "RRTstarConfig", testLogger()))
      << "value: " << bad;
    EXPECT_DOUBLE_EQ(policy.optimization_budget_sec, 0.0) << "value: " << bad;
    // 无论解析成功还是失败，键都要被摘掉。
    EXPECT_EQ(config.count("optimization_budget_sec"), 0U) << "value: " << bad;
  }
}

// ---------------------------------------------------------------------------
// 优化窗口：这是本包装层存在的主要理由
// ---------------------------------------------------------------------------

TEST(OptimizingPlannerWrapper, NativeRRTstarBurnsTheWholeTimeout)
{
  // 先把"病症"钉下来，否则下一个用例的加速无从对比。
  // 这条同时是一个回归哨兵：哪天 OMPL 改了默认行为让 RRT* 自己提前返回，
  // 这条会失败，提醒我们包装层可能已经不需要了。
  OpenSpaceProblem problem;
  auto planner = std::make_shared<og::RRTstar>(problem.spaceInformation());

  constexpr double kTimeout = 1.0;
  const double elapsed = problem.solveFor(planner, kTimeout);

  EXPECT_TRUE(problem.lastStatus());
  EXPECT_GE(elapsed, kTimeout * 0.9)
    << "native RRT* is expected to keep optimizing until the timeout fires";
}

TEST(OptimizingPlannerWrapper, OptimizationBudgetReturnsLongBeforeTheTimeout)
{
  OpenSpaceProblem problem;
  OptimizingPlannerPolicy policy;
  policy.optimization_budget_sec = 0.15;

  auto planner = std::make_shared<OptimizingPlannerWrapper<og::RRTstar>>(
    problem.spaceInformation(), policy, testLogger());

  // 超时给 5s，但首解只需毫秒级，所以应当在 0.15s 出头就返回。
  constexpr double kTimeout = 5.0;
  const double elapsed = problem.solveFor(planner, kTimeout);

  EXPECT_TRUE(problem.lastStatus()) << "the budget must not cost us the solution";
  EXPECT_GE(elapsed, policy.optimization_budget_sec)
    << "must actually spend the optimization window, not bail at the first solution";
  // 上界给足余量（PTC 只在每次迭代边界求值，一次迭代可能较长）。
  EXPECT_LT(elapsed, 1.0)
    << "elapsed=" << elapsed << "s: the budget did not cut the run short";
}

TEST(OptimizingPlannerWrapper, ZeroBudgetIsIndistinguishableFromNativeBehaviour)
{
  // 策略两项都不配时必须逐字节退回原生行为 —— 这是"默认不改变任何东西"
  // 的保证，也是为什么 allocator 可以无条件套包装类。
  OpenSpaceProblem problem;
  OptimizingPlannerPolicy policy;   // 全默认 = 不启用
  ASSERT_FALSE(policy.enabled());

  auto planner = std::make_shared<OptimizingPlannerWrapper<og::RRTstar>>(
    problem.spaceInformation(), policy, testLogger());

  constexpr double kTimeout = 1.0;
  const double elapsed = problem.solveFor(planner, kTimeout);

  EXPECT_TRUE(problem.lastStatus());
  EXPECT_GE(elapsed, kTimeout * 0.9)
    << "a disabled policy must not shorten the run";
}

// ---------------------------------------------------------------------------
// 惰性目标采样等待
// ---------------------------------------------------------------------------

TEST(OptimizingPlannerWrapper, WaitsForTheLazyGoalSamplerToProduceItsFirstState)
{
  OpenSpaceProblem problem;
  const ob::SpaceInformationPtr si = problem.spaceInformation();

  // 造一个"慢启动"的惰性目标采样器：第一个状态延迟 80ms 才产出。
  // 这就是实测里 informed 采样器抛
  // "There must be at least 1 start and and 1 goal state" 的那个窗口。
  auto sampled = std::make_shared<std::atomic<int>>(0);
  auto lazy_goal = std::make_shared<ob::GoalLazySamples>(
    si,
    [sampled](const ob::GoalLazySamples * gls, ob::State * state) -> bool {
      if (sampled->load() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
      if (gls->getStateCount() >= 3U) {
        return false;                       // 采够了，结束采样线程
      }
      auto * rv = state->as<ob::RealVectorStateSpace::StateType>();
      for (unsigned int i = 0; i < 4U; ++i) {
        rv->values[i] = 1.0;
      }
      sampled->fetch_add(1);
      return true;
    },
    false);                                  // 先不自动启动，下面手动 start

  problem.problemDefinition()->setGoal(lazy_goal);

  OptimizingPlannerPolicy policy;
  policy.goal_state_wait_sec = 1.0;          // 远大于 80ms
  auto planner = std::make_shared<OptimizingPlannerWrapper<og::RRTstar>>(si, policy, testLogger());

  lazy_goal->startSampling();
  ASSERT_EQ(lazy_goal->getStateCount(), 0U) << "the fixture must start with zero goal states";

  problem.solveFor(planner, 1.0);

  // 关键断言：solve() 被委托下去之前，目标状态已经就位。
  EXPECT_GT(sampled->load(), 0)
    << "the wrapper must not delegate to solve() while the goal has no states";
  lazy_goal->stopSampling();
}

TEST(OptimizingPlannerWrapper, GivesUpWaitingInsteadOfHangingOnAnUnreachableGoal)
{
  // 目标采样器永远产不出状态时，绝不能死等 —— 必须超时后照常委托给 OMPL，
  // 让它用自己的路径报"无解"，而不是在这里把失败吞掉或者挂死。
  OpenSpaceProblem problem;
  const ob::SpaceInformationPtr si = problem.spaceInformation();

  auto lazy_goal = std::make_shared<ob::GoalLazySamples>(
    si,
    [](const ob::GoalLazySamples *, ob::State *) -> bool {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      return false;                          // 永不产出状态
    },
    false);
  problem.problemDefinition()->setGoal(lazy_goal);

  OptimizingPlannerPolicy policy;
  policy.goal_state_wait_sec = 0.05;
  auto planner = std::make_shared<OptimizingPlannerWrapper<og::RRTstar>>(si, policy, testLogger());

  lazy_goal->startSampling();
  const auto t0 = std::chrono::steady_clock::now();
  problem.solveFor(planner, 0.2);
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_LT(elapsed, 1.0) << "elapsed=" << elapsed << "s: the bounded wait did not bound";
  lazy_goal->stopSampling();
}

TEST(OptimizingPlannerWrapper, NonLazyGoalNeedsNoWait)
{
  // 显式 GoalStates 不存在这个竞争，包装层应当直接放行、不产生任何延迟。
  OpenSpaceProblem problem;
  const ob::SpaceInformationPtr si = problem.spaceInformation();

  OptimizingPlannerPolicy policy;
  policy.goal_state_wait_sec = 5.0;          // 故意给一个很大的值
  policy.optimization_budget_sec = 0.05;
  auto planner = std::make_shared<OptimizingPlannerWrapper<og::RRTstar>>(si, policy, testLogger());

  const double elapsed = problem.solveFor(planner, 2.0);

  EXPECT_TRUE(problem.lastStatus());
  EXPECT_LT(elapsed, 1.0)
    << "elapsed=" << elapsed << "s: a non-lazy goal must not trigger the 5s wait";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
