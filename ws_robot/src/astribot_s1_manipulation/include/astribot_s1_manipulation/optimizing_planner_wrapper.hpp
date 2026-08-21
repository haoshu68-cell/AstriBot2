// Copyright 2026 Astribot
//
// 渐进最优规划器（RRT* / Informed RRT* / SORRT* ...）的行为包装层。
//
// 解决两个 Gazebo 实测问题，两者都只能在 solve() 这一层拦，所以合成一个包装类。
//
// 问题 1：渐进最优规划器把 allowed_planning_time 全部烧掉
// -----------------------------------------------------
// 实测日志（arm_left，从 ready 位姿到一个近处关节目标）：
//   Info: Found an initial solution with a cost of 0.60 in 4 iterations
//         (5 vertices in the graph)
//   Info: Created 6508 new states. Checked 21160774 rewire options.
//         Final solution cost 0.600
//   Info: Solution found in 5.001364 seconds
// 也就是：**第 4 次迭代（毫秒级）就拿到了最终解**，剩下 4.99s 做了 2116 万次
// rewire 检查，代价一点没降（0.600 -> 0.600）。demo 里 5 次规划有 5 次这样，
// 白等 25s。
//
// 这不是 bug，是 RRT* 家族的定义行为：它们只在
//   ① 终止条件(PTC)成立，或 ② 解的代价达到 optimization objective 的
//      cost threshold
// 时返回。MoveIt 默认用 PathLengthOptimizationObjective，其 cost threshold 是 0
// （日志里那句 "Seeking a solution better than 0.00000"），永远达不到，
// 于是唯一出口就是超时。BIT* 不受影响（它有 batch 收敛判据，实测 0.015s 返回）。
//
// 为什么不用 MoveIt 官方的 termination_condition 键
// ------------------------------------------------
// MoveIt Humble 确实支持在 ompl_planning.yaml 里写
//   termination_condition: CostConvergence[solutions_window,epsilon]
//                        | ExactSolution | Iteration[num]
// （见 moveit/ompl_interface/model_based_planning_context.h 的
//  constructPlannerTerminationCondition 注释）。本包把该键放进
//  moveitReservedKeys()，所以它照常透传给 MoveIt，需要时可以直接用。
// 但它治不了上面这个场景：
//   * CostConvergence 靠 pdef 的 intermediate-solution 回调计数，只在
//     **发现更好的解**时才推进（见 OMPL 的
//     CostConvergenceTerminationCondition::processNewSolution）。上面这个查询
//     首解即最优、之后一次都没改进过，solutions 计数停在 1，永远凑不满
//     solutions_window，条件永不成立。
//   * ExactSolution 一有可行解就停，等于把 RRT* 退化成 RRT，
//     直接放弃了渐进最优性 —— 而任务要求的就是最优规划器。
// 本包要的语义是两者之间的第三种：**首解出现后再优化一个固定的时间窗**。
// 这个 PTC 在 OMPL 里没有现成实现，所以在这里自己合成。
// 好处是节拍可预测（总耗时 <= 首解耗时 + 窗口），又保留了 rewiring 带来的
// 代价下降机会。窗口长度写在 yaml 里，不硬编码。
//
// 问题 2：informed 采样器与 MoveIt 的惰性目标采样存在启动竞争
// ---------------------------------------------------------
// 实测日志（InformedRRTstarConfig，一次规划刷 13 条 ERROR）：
//   [ERROR] Exception caught executing adapter 'Fix Start State Path
//   Constraints': PathLengthDirectInfSampler: There must be at least 1 start
//   and and 1 goal state when the informed sampler is created.
//   Warning: Goal sampling thread never did any work.
//   Debug: Stopped goal sampling thread after 0 sampling attempts
// 原因：MoveIt 用 ob::GoalLazySamples 表达目标 —— 目标状态由一个后台采样线程
// 陆续产出。而 RRTstar::solve() 一进来就 allocSampler()，informed 采样器
// (PathLengthDirectInfSampler) 在构造时就要求 pdef 里已经有 >=1 个目标状态。
// 采样线程刚 start 还没排上 CPU 时，状态数是 0，于是抛异常。
// 异常沿 MoveIt 的整条 request-adapter 链层层重抛，每个 adapter 报一条 ERROR。
// 它会"自愈"（adapter 被跳过，后续尝试往往已经有目标状态了，实测第 14 次拿到
// 10 个目标状态并出解），所以不影响成功率，但日志噪声极大，
// 而且真正的规划失败会被埋在这堆 ERROR 里看不见。
//
// 修法：委托 solve() 之前，如果目标是 GoalLazySamples 且状态数为 0，
// 就有界地等它产出第一个状态。等到了立刻返回；等不到（超时或采样线程已结束）
// 就照常委托下去，让 OMPL 用自己的路径报错，绝不在这里吞掉失败。

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
///
/// 必须删除：剩下的键会被逐个喂给 OMPL 的 planner->params().setParam()，
/// 而这两个键不是任何 OMPL 规划器的参数，留着会触发本包的
/// "has no parameter named" WARN —— 那个 WARN 是用来抓拼写错误的，
/// 不能被自己的键污染。
///
/// 解析失败（写了非数字、负数）时打 WARN 并按"不启用"处理，不抛异常。
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
      // 没有问题定义时不做任何加工，让 OMPL 自己走它的报错路径。
      return PlannerT::solve(ptc);
    }

    const ompl::base::Planner::PlannerProgressProperty best_cost = findBestCostProperty();
    if (!best_cost) {
      // 不装死也不猜：明确告知窗口不生效，退回原生行为。
      RCLCPP_WARN(
        logger_,
        "planner '%s': no '%s' progress property, cannot detect the first solution; "
        "%s = %.3f has no effect (native behaviour: optimize until timeout)",
        PlannerT::getName().c_str(), kBestCostProperty,
        OptimizingPlannerPolicy::kOptimizationBudgetKey, policy_.optimization_budget_sec);
      return PlannerT::solve(ptc);
    }

    // 共享状态：首解出现的时刻（steady_clock 纪元纳秒，0 = 还没出现）。
    // 用 steady_clock 而不是 system_clock，避免系统时间被 NTP 调整时窗口算错。
    // 用 atomic 是防御性的：PTC 目前只在规划器主线程里求值，但
    // OMPL 的 PTC 也有"独立线程周期求值"的实现形式，不想依赖这个细节。
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
        // 首次看到解：记下时刻，本次不终止，让优化窗口从现在开始计时。
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
  ///
  /// 为什么不能只用 pdef->hasExactSolution()（第一版就是这么写的，跑不通）：
  /// RRTstar 在**整个 solve() 循环里都不往 pdef 里塞解** —— 它把最优的
  /// goal motion 存在自己的 bestGoalMotion_ 里，只在循环结束后才
  /// addSolutionPath()。所以在 PTC 求值的那个时刻，hasExactSolution()
  /// 恒为 false，窗口永远不会开始计时，实测就是烧满 5.0s。
  /// （这也是 OMPL 的 CostConvergenceTerminationCondition 要挂
  ///  intermediate-solution 回调而不是查 pdef 的原因。）
  ///
  /// 这里改用 progress property "best cost REAL"：它读的就是规划器内部的
  /// bestCost_，循环中实时更新。没有解时是 inf（个别规划器 setup 前是 nan），
  /// 所以判据就是"能解析成有限数"。
  ///
  /// 不用 pdef->setIntermediateSolutionCallback()：那个槽位只有一个，
  /// MoveIt 的 termination_condition: CostConvergence[...] 也要用它，
  /// 抢过来会静默破坏用户配的官方终止条件。progress property 是只读的，
  /// 不存在这个冲突。
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
      // 属性值不是数字（不该发生）。当成"还没有解"，即窗口不启动、
      // 退回超时兜底 —— 宁可慢也不能提前砍掉一次本来能成功的规划。
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
      // 不是惰性目标（例如显式 GoalStates），不存在这个竞争。
      return;
    }
    if (lazy_goal->getStateCount() > 0U) {
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(policy_.goal_state_wait_sec));
    while (lazy_goal->getStateCount() == 0U) {
      // 采样线程已经结束了就别白等 —— 不会再有新状态产出。
      if (!lazy_goal->isSampling()) {
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (lazy_goal->getStateCount() == 0U) {
      // 不在这里当失败处理：目标可能确实不可达，那该由 OMPL/MoveIt 报出来。
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
