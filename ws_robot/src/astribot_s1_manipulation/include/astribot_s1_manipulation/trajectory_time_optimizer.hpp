// Copyright 2026 Astribot
//
// 轨迹时间优化（节拍压缩）。
//
// 优化的是什么
// ------------
// OMPL 给出的是几何路径，路径**形状**已定。时间优化不改形状，只改"沿这条路径
// 走多快"——即重新分配每个 waypoint 的时间戳。所以它压缩的是纯粹的
// "无效等待时间"，不会让机器人走一条不同的路（不会引入新的碰撞风险）。
//
// 两种参数化器的区别（这是选型的关键）
// ----------------------------------
// IPTP (IterativeParabolicTimeParameterization)：
//   MoveIt 的老牌默认实现。按梯形速度曲线逐段迭代，实现简单、鲁棒，
//   但它是**保守**的：为了保证不超限，很多段没有把速度跑满，
//   总时长明显长于理论最优。它是本工程的 baseline。
//
// TOTG (TimeOptimalTrajectoryGeneration)：
//   基于 Kunz & Stilman 的时间最优路径参数化算法。在给定路径与关节
//   速度/加速度限位下求解**时间最优**的速度剖面，理论上就是节拍下界。
//   它会把限位真正跑满，所以节拍明显短于 IPTP —— 这正是任务要的"压缩节拍"。
//   代价：① 对路径点密度敏感，重复点/过密点会让它数值上出问题；
//        ② 它会重采样路径点（resample_dt），输出的 waypoint 数与输入不同。
//
// 为什么必须自己复核限位并保留回退
// ------------------------------
// 任务明确要求"优化后仍然超出关节速度/加速度硬限制 -> 回退原始规划轨迹，
// 告警输出，不输出非法轨迹"。实测 TOTG 在路径含重复点时确实会给出超限结果，
// 所以不能信任"参数化器应该遵守限位"，必须用 trajectory_metrics 独立复核，
// 不合法就回退。这是本模块的核心契约。

#ifndef ASTRIBOT_S1_MANIPULATION__TRAJECTORY_TIME_OPTIMIZER_HPP_
#define ASTRIBOT_S1_MANIPULATION__TRAJECTORY_TIME_OPTIMIZER_HPP_

#include <string>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include "astribot_s1_manipulation/error_codes.hpp"
#include "astribot_s1_manipulation/trajectory_metrics.hpp"

namespace astribot_s1_manipulation
{

struct TimeOptimizerParams
{
  /// 时间优化总开关。关掉后只做 baseline 参数化（IPTP），
  /// 仍然会输出可执行轨迹，只是节拍不压缩。
  bool enable_optimization{true};

  /// baseline(IPTP) 用的缩放系数。取小值是刻意的：
  /// baseline 的作用是"安全可执行的参考"，同时作为节拍对比的分母。
  double baseline_velocity_scaling{0.30};
  double baseline_acceleration_scaling{0.30};

  /// 优化(TOTG) 用的缩放系数。这才是"把限位跑满"的那一档。
  /// 不建议直接给 1.0：真机上留 10% 余量给跟踪误差，
  /// 否则控制器一旦滞后就立刻超限报警。
  double optimized_velocity_scaling{0.90};
  double optimized_acceleration_scaling{0.90};

  /// TOTG 的路径几何容差(rad)。它允许在拐角处稍微"抹圆"以获得更快通过速度。
  /// 给大了会偏离原路径（有碰撞风险，因为碰撞是在原路径上校验的）；
  /// 给 0 会让拐角处速度被迫降到 0，节拍反而变差。0.1 是官方推荐量级。
  double totg_path_tolerance{0.1};

  /// TOTG 输出的重采样时间步(s)。0 表示不重采样（保留原 waypoint 时刻）。
  /// 给 0.05s(20Hz) 与 JointTrajectoryController 的 100Hz 更新率匹配良好：
  /// 控制器会在点之间自己插值，不需要给到 100Hz 那么密。
  double totg_resample_dt{0.05};

  /// TOTG 判定"两个路径点是否重复"的最小关节角变化(rad)。
  /// 低于此值的点会被合并 —— 这一条很关键：OMPL 的路径经常包含几乎重合的点，
  /// 不合并会让 TOTG 在这些点上算出巨大的速度（除以接近 0 的距离）。
  double totg_min_angle_change{0.001};

  /// 是否再叠加 Ruckig 平滑。默认关闭：Ruckig 需要 jerk 限位，
  /// 而 URDF 和手册都没有本机器人的 jerk 数据，编一个数字进去
  /// 等于给后续调参埋一个来源不明的常量（见 joint_limits.yaml 的说明）。
  bool enable_ruckig_smoothing{false};

  /// 限位复核的相对容差，透传给 evaluateTrajectory。
  double limit_tolerance_ratio{1e-3};
};

struct OptimizationResult
{
  /// 优化版本是否被采纳。false 表示用的是 baseline（原因见 note）。
  bool optimized_accepted{false};
  /// 是否发生了回退（优化结果超限或参数化失败）。
  bool fell_back{false};

  TrajectoryMetrics baseline_metrics;
  TrajectoryMetrics optimized_metrics;

  /// 节拍缩短比例，(baseline - optimized) / baseline，范围通常 (0, 1)。
  /// 负值意味着"优化"反而更慢 —— 这种结果也会被拒绝并回退。
  double duration_reduction_ratio{0.0};

  std::string note;
};

/// 轨迹时间参数化与节拍优化器。
///
/// 用法：configure() 一次，之后可对多条轨迹反复调用 optimize()。
/// optimize() 会**就地修改**传入的轨迹为最终采纳的版本。
class TrajectoryTimeOptimizer
{
public:
  TrajectoryTimeOptimizer() = default;

  bool configure(const TimeOptimizerParams & params, std::string & error);

  bool isConfigured() const noexcept
  {
    return configured_;
  }

  const TimeOptimizerParams & params() const noexcept
  {
    return params_;
  }

  /// 对轨迹做时间参数化并尝试压缩节拍。
  ///
  /// 流程：
  ///   1. 复制一份做 IPTP -> baseline（必须成功，否则整个轨迹不可执行）
  ///   2. 复制一份做 TOTG -> optimized
  ///   3. 用 evaluateTrajectory 独立复核两者的限位合法性
  ///   4. optimized 合法且确实更短 -> 采纳；否则回退 baseline 并 WARN
  ///   5. baseline 也非法 -> 返回 kJointLimitViolation，**不输出轨迹**
  ///
  /// @param[in,out] trajectory 输入原始几何路径；输出为最终采纳的带时轨迹。
  ///                失败时不保证内容有意义，调用方应据返回码决定是否使用。
  /// @param[out] result 优化过程与两版指标，供日志与上层判断。
  /// @return kSuccess / kTimeParameterizationFailed / kJointLimitViolation /
  ///         kInvalidInput / kNotConfigured / kExceptionCaught
  PlanErrorCode optimize(
    robot_trajectory::RobotTrajectory & trajectory,
    OptimizationResult & result) const;

private:
  /// 跑 IPTP。失败返回 false（不抛异常）。
  bool applyIptp(
    robot_trajectory::RobotTrajectory & trajectory,
    double velocity_scaling,
    double acceleration_scaling,
    std::string & error) const;

  /// 跑 TOTG。失败返回 false（不抛异常）。
  bool applyTotg(
    robot_trajectory::RobotTrajectory & trajectory,
    double velocity_scaling,
    double acceleration_scaling,
    std::string & error) const;

  TimeOptimizerParams params_;
  bool configured_{false};
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__TRAJECTORY_TIME_OPTIMIZER_HPP_
