// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_MANIPULATION__TRAJECTORY_METRICS_HPP_
#define ASTRIBOT_S1_MANIPULATION__TRAJECTORY_METRICS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace astribot_s1_manipulation
{

/// 单关节的峰值统计。
struct JointPeak
{
  std::string joint_name;
  double max_velocity{0.0};          ///< rad/s（绝对值）
  double max_acceleration{0.0};      ///< rad/s^2（绝对值）
  double velocity_limit{0.0};        ///< 该关节的限位，来自 RobotModel（URDF + joint_limits.yaml）
  double acceleration_limit{0.0};
  bool velocity_violated{false};
  bool acceleration_violated{false};
  /// 速度用满了限位的百分之多少。节拍优化的核心指标：
  /// 这个值越接近 100% 说明越"跑满"，节拍越紧。
  double velocity_utilization{0.0};
};

/// 速度/加速度数据来源。混用两种来源做对比会得出错误结论，所以显式标明。
enum class MetricsSource
{
  kFromTrajectory,   ///< 直接读 waypoint 里的速度/加速度（已时间参数化）
  kFiniteDifference, ///< 由位置差分估算（原始几何路径，无时间信息）
  kUnavailable,      ///< 无法计算
};

struct TrajectoryMetrics
{
  bool valid{false};
  MetricsSource source{MetricsSource::kUnavailable};

  double duration{0.0};              ///< 总时长，秒。这就是"节拍"
  std::size_t waypoint_count{0};

  double max_joint_velocity{0.0};    ///< 全轨迹全关节的速度峰值 rad/s
  std::string max_velocity_joint;
  double max_joint_acceleration{0.0};
  std::string max_acceleration_joint;

  /// 全关节里"速度利用率"最高者的利用率。判断节拍是否还有压缩余量：
  /// 远小于 100% 说明还能更快；接近 100% 说明已经贴着限位了。
  double peak_velocity_utilization{0.0};

  std::vector<JointPeak> per_joint;

  bool velocity_limit_violated{false};
  bool acceleration_limit_violated{false};
  std::vector<std::string> violating_joints;

  std::string summary;               ///< 一行可打印摘要

  /// 轨迹是否合法（不超任何硬限位）。任务禁止输出非法轨迹，
  /// 这就是最终的放行判据。
  bool isLegal() const noexcept
  {
    return valid && !velocity_limit_violated && !acceleration_limit_violated;
  }
};

/// 评估参数。
struct MetricsParams
{
  /// 限位判定的相对容差。浮点计算和时间参数化器内部的离散化会让结果
  /// 在限位上下浮动极小的量，完全零容差会把合法轨迹误判为超限。
  /// 1e-3 = 允许超出限位 0.1%。
  double limit_tolerance_ratio{1e-3};

  /// 是否允许在没有速度数据时退化为差分估算。
  /// false 时遇到未参数化轨迹直接返回 valid=false。
  bool allow_finite_difference{true};
};

/// 评估一条轨迹的节拍指标。
///
/// @param trajectory 待评估轨迹。空轨迹或单点轨迹返回 valid=false。
/// @param params 评估参数。
/// @return 指标。任何情况下都不抛异常。
TrajectoryMetrics evaluateTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const MetricsParams & params);

/// 把两次评估结果格式化成对比字符串，用于"优化前 vs 优化后"的日志输出。
/// 任务要求"优化后同等任务运动节拍相比原始规划有缩短"，这个函数就是它的证据输出。
std::string formatComparison(
  const TrajectoryMetrics & baseline,
  const TrajectoryMetrics & optimized);

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__TRAJECTORY_METRICS_HPP_
