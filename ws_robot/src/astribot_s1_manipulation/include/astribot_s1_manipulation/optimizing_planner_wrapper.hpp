// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_MANIPULATION__OPTIMIZING_PLANNER_WRAPPER_HPP_
#define ASTRIBOT_S1_MANIPULATION__OPTIMIZING_PLANNER_WRAPPER_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <rclcpp/rclcpp.hpp>

namespace astribot_s1_manipulation
{

/// 渐进最优规划器的收敛策略。取值全部来自 ompl_planning.yaml 的
/// planner_configs.<配置名> 段，不在代码里给业务默认值 ——
/// 这里的成员默认值是"不启用"，即完全退回 OMPL 原生行为。
struct OptimizingPlannerPolicy
{
  /// yaml 键名：首解出现后允许继续优化的秒数。<=0 表示不限制（原生行为）。
  static constexpr const char * kOptimizationBudgetKey = "optimization_budget_sec";
  /// yaml 键名：委托 solve() 前等待惰性目标采样器产出首个目标状态的秒数。
  /// <=0 表示不等待（原生行为）。
  static constexpr const char * kGoalStateWaitKey = "goal_state_wait_sec";

  double optimization_budget_sec{0.0};
  double goal_state_wait_sec{0.0};

  /// 是否有任何一项被启用。两项都不启用时不必套包装类。
  bool enabled() const
  {
    return optimization_budget_sec > 0.0 || goal_state_wait_sec > 0.0;
  }
};

/// 从 yaml 配置映射里取出本包自己的策略键，并**从 config 中删除**它们。
inline OptimizingPlannerPolicy extractOptimizingPolicy(
  std::map<std::string, std::string> & config,
  const std::string & planner_config_name,
  const rclcpp::Logger & logger)
{
  OptimizingPlannerPolicy policy;

  const auto take = [&config, &planner_config_name, &logger](
    const char * key, double * out) {
      const auto it = config.find(key);
      if (it == config.end()) {
        return;
      }
      const std::string raw = it->second;
      config.erase(it);
      try {
        std::size_t consumed = 0U;
        const double parsed = std::stod(raw, &consumed);
        if (consumed != raw.size()) {
          throw std::invalid_argument("trailing characters");
        }
        if (parsed < 0.0) {
          RCLCPP_WARN(
            logger,
            "planner config '%s': %s = %s is negative, treated as disabled",
            planner_config_name.c_str(), key, raw.c_str());
          return;
        }
        *out = parsed;
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          logger,
          "planner config '%s': %s = '%s' is not a number (%s), treated as disabled",
          planner_config_name.c_str(), key, raw.c_str(), e.what());
      }
    };

  take(OptimizingPlannerPolicy::kOptimizationBudgetKey, &policy.optimization_budget_sec);
  take(OptimizingPlannerPolicy::kGoalStateWaitKey, &policy.goal_state_wait_sec);
  return policy;
}

/// 给任意 OMPL geometric 规划器套上上面两条策略。
///
/// 只重写 solve(const PlannerTerminationCondition&) —— 这是 ob::Planner 里
/// 唯一的纯虚 solve；solve(double) / solve(double,double) 是非虚包装，
/// 内部都会转到这个虚函数，所以重写一个就全覆盖了。
template<typename PlannerT>
class OptimizingPlannerWrapper : public PlannerT
{
public:
  OptimizingPlannerWrapper(
    const ompl::base::SpaceInformationPtr & si,
    OptimizingPlannerPolicy policy,
    rclcpp::Logger logger)
  : PlannerT(si), policy_(policy), logger_(std::move(logger))
  {
  }

  ompl::base::PlannerStatus solve(
    const ompl::base::PlannerTerminationCondition & ptc) override
  {
    waitForFirstLazyGoalState();

    if (policy_.optimization_budget_sec <= 0.0) {
      return PlannerT::solve(ptc);
    }

    const ompl::base::ProblemDefinitionPtr pdef = PlannerT::getProblemDefinition();
    if (!pdef) {
      return PlannerT::solve(ptc);
    }

    const ompl::base::Planner::PlannerProgressProperty best_cost = findBestCostProperty();
    if (!best_cost) {
      RCLCPP_WARN(
        logger_,
        "planner '%s': no '%s' progress property, cannot detect the first solution; "
        "%s = %.3f has no effect (native behaviour: optimize until timeout)",
        PlannerT::getName().c_str(), kBestCostProperty,
        OptimizingPlannerPolicy::kOptimizationBudgetKey, policy_.optimization_budget_sec);
      return PlannerT::solve(ptc);
    }

    auto first_solution_ns = std::make_shared<std::atomic<int64_t>>(0);
    const double budget = policy_.optimization_budget_sec;

    const ompl::base::PlannerTerminationCondition budget_ptc(
      [pdef, best_cost, budget, first_solution_ns]() -> bool {
        if (!hasSolutionYet(pdef, best_cost)) {
          return false;
        }
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        int64_t expected = 0;
        if (first_solution_ns->compare_exchange_strong(expected, now_ns)) {
          return false;
        }
        const double elapsed_sec =
          static_cast<double>(now_ns - first_solution_ns->load()) * 1e-9;
        return elapsed_sec >= budget;
      });

    const ompl::base::PlannerStatus status =
      PlannerT::solve(ompl::base::plannerOrTerminationCondition(ptc, budget_ptc));

    if (first_solution_ns->load() != 0) {
      RCLCPP_DEBUG(
        logger_,
        "planner '%s': first solution seen, kept optimizing for up to %s = %.3fs",
        PlannerT::getName().c_str(),
        OptimizingPlannerPolicy::kOptimizationBudgetKey, budget);
    }
    return status;
  }

private:
  /// OMPL 各优化型规划器上报"当前最优解代价"的 progress property 名。
  /// 这个字符串是 OMPL 的既成约定（RRTstar / BITstar / AITstar 都用它注册，
  /// InformedRRTstar / SORRTstar / ABITstar 从各自基类继承），
  /// 不是本包发明的，所以拿它当通用信号源是安全的。
  static constexpr const char * kBestCostProperty = "best cost REAL";

  ompl::base::Planner::PlannerProgressProperty findBestCostProperty() const
  {
    const auto & properties = PlannerT::getPlannerProgressProperties();
    const auto it = properties.find(kBestCostProperty);
    if (it == properties.end()) {
      return ompl::base::Planner::PlannerProgressProperty();
    }
    return it->second;
  }

  /// 判断"规划器现在已经拿到解了吗"。
  static bool hasSolutionYet(
    const ompl::base::ProblemDefinitionPtr & pdef,
    const ompl::base::Planner::PlannerProgressProperty & best_cost)
  {
    if (pdef->hasExactSolution()) {
      return true;
    }
    try {
      return std::isfinite(std::stod(best_cost()));
    } catch (const std::exception &) {
      return false;
    }
  }

  /// 有界等待惰性目标采样器产出第一个目标状态。见文件头"问题 2"。
  void waitForFirstLazyGoalState()
  {
    if (policy_.goal_state_wait_sec <= 0.0) {
      return;
    }
    const ompl::base::ProblemDefinitionPtr pdef = PlannerT::getProblemDefinition();
    if (!pdef) {
      return;
    }
    const auto lazy_goal =
      std::dynamic_pointer_cast<ompl::base::GoalLazySamples>(pdef->getGoal());
    if (!lazy_goal) {
      return;
    }
    if (lazy_goal->getStateCount() > 0U) {
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(policy_.goal_state_wait_sec));
    while (lazy_goal->getStateCount() == 0U) {
      if (!lazy_goal->isSampling()) {
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (lazy_goal->getStateCount() == 0U) {
      RCLCPP_WARN(
        logger_,
        "planner '%s': lazy goal sampler produced no goal state within %.3fs; "
        "delegating to OMPL anyway (informed samplers may throw)",
        PlannerT::getName().c_str(), policy_.goal_state_wait_sec);
    } else {
      RCLCPP_DEBUG(
        logger_,
        "planner '%s': waited for the lazy goal sampler, %zu goal state(s) ready",
        PlannerT::getName().c_str(), lazy_goal->getStateCount());
    }
  }

  OptimizingPlannerPolicy policy_;
  rclcpp::Logger logger_;
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__OPTIMIZING_PLANNER_WRAPPER_HPP_
