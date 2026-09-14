// Copyright 2026 Astribot

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
  /// @param[in,out] trajectory 输入原始几何路径；输出为最终采纳的带时轨迹。
  /// @param[out] result 优化过程与两版指标，供日志与上层判断。
  /// @return kSuccess / kTimeParameterizationFailed / kJointLimitViolation /
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
