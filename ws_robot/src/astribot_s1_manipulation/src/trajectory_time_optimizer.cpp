// Copyright 2026 Astribot

#include "astribot_s1_manipulation/trajectory_time_optimizer.hpp"

#include <exception>
#include <sstream>
#include <string>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/logging.hpp>

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.time_optimizer";

bool isValidScaling(double value)
{
  // MoveIt 的缩放系数定义域是 (0, 1]。给 0 会让轨迹永远走不完，
  // 给 >1 会直接突破 joint_limits.yaml 里的硬限位。
  return value > 0.0 && value <= 1.0;
}
}  // namespace

bool TrajectoryTimeOptimizer::configure(
  const TimeOptimizerParams & params, std::string & error)
{
  error.clear();
  configured_ = false;

  if (!isValidScaling(params.baseline_velocity_scaling)) {
    error = "baseline_velocity_scaling must be in (0, 1], got " +
      std::to_string(params.baseline_velocity_scaling);
    return false;
  }
  if (!isValidScaling(params.baseline_acceleration_scaling)) {
    error = "baseline_acceleration_scaling must be in (0, 1], got " +
      std::to_string(params.baseline_acceleration_scaling);
    return false;
  }
  if (!isValidScaling(params.optimized_velocity_scaling)) {
    error = "optimized_velocity_scaling must be in (0, 1], got " +
      std::to_string(params.optimized_velocity_scaling);
    return false;
  }
  if (!isValidScaling(params.optimized_acceleration_scaling)) {
    error = "optimized_acceleration_scaling must be in (0, 1], got " +
      std::to_string(params.optimized_acceleration_scaling);
    return false;
  }
  if (!(params.totg_path_tolerance > 0.0)) {
    error = "totg_path_tolerance must be > 0";
    return false;
  }
  if (params.totg_resample_dt < 0.0) {
    error = "totg_resample_dt must be >= 0 (0 means no resampling)";
    return false;
  }
  if (!(params.totg_min_angle_change > 0.0)) {
    error = "totg_min_angle_change must be > 0";
    return false;
  }
  if (params.limit_tolerance_ratio < 0.0) {
    error = "limit_tolerance_ratio must be >= 0";
    return false;
  }

  // 优化档的缩放不比 baseline 大的话，"优化"没有任何意义 ——
  // 这是配置错误，早点拦住比让用户困惑"为什么节拍没缩短"要好。
  if (params.enable_optimization &&
    params.optimized_velocity_scaling <= params.baseline_velocity_scaling)
  {
    error = "optimized_velocity_scaling must exceed baseline_velocity_scaling "
      "for optimization to shorten the cycle time";
    return false;
  }

  params_ = params;
  configured_ = true;
  return true;
}

bool TrajectoryTimeOptimizer::applyIptp(
  robot_trajectory::RobotTrajectory & trajectory,
  double velocity_scaling,
  double acceleration_scaling,
  std::string & error) const
{
  error.clear();
  try {
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling)) {
      error = "IterativeParabolicTimeParameterization::computeTimeStamps returned false";
      return false;
    }
    return true;
  } catch (const std::exception & e) {
    error = std::string("exception in IPTP: ") + e.what();
    return false;
  }
}

bool TrajectoryTimeOptimizer::applyTotg(
  robot_trajectory::RobotTrajectory & trajectory,
  double velocity_scaling,
  double acceleration_scaling,
  std::string & error) const
{
  error.clear();
  try {
    // path_tolerance / min_angle_change 通过构造函数给；
    // resample_dt 也在构造函数里（0 表示不重采样）。
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(
      params_.totg_path_tolerance,
      params_.totg_resample_dt,
      params_.totg_min_angle_change);
    if (!totg.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling)) {
      error = "TimeOptimalTrajectoryGeneration::computeTimeStamps returned false";
      return false;
    }
    return true;
  } catch (const std::exception & e) {
    // TOTG 在路径含重复点/退化段时确实会抛异常。捕获后按"优化失败"处理，
    // 上层会回退到 baseline，而不是让整个规划崩掉。
    error = std::string("exception in TOTG: ") + e.what();
    return false;
  }
}

PlanErrorCode TrajectoryTimeOptimizer::optimize(
  robot_trajectory::RobotTrajectory & trajectory,
  OptimizationResult & result) const
{
  result = OptimizationResult{};

  if (!configured_) {
    result.note = "TrajectoryTimeOptimizer not configured";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kNotConfigured;
  }
  if (trajectory.getWayPointCount() == 0U) {
    result.note = "empty trajectory";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kInvalidInput;
  }
  if (trajectory.getGroup() == nullptr) {
    result.note = "trajectory has no joint model group";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kInvalidInput;
  }

  MetricsParams metrics_params;
  metrics_params.limit_tolerance_ratio = params_.limit_tolerance_ratio;
  metrics_params.allow_finite_difference = true;

  // ---- 第一步：baseline (IPTP) ----
  // 用副本做，这样失败时原始轨迹还在。
  robot_trajectory::RobotTrajectory baseline(trajectory);
  std::string iptp_error;
  if (!applyIptp(
      baseline, params_.baseline_velocity_scaling,
      params_.baseline_acceleration_scaling, iptp_error))
  {
    result.note = "baseline time parameterization failed: " + iptp_error;
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kTimeParameterizationFailed;
  }
  result.baseline_metrics = evaluateTrajectory(baseline, metrics_params);
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName), "baseline (IPTP, scaling %.2f/%.2f): %s",
    params_.baseline_velocity_scaling, params_.baseline_acceleration_scaling,
    result.baseline_metrics.summary.c_str());

  if (!result.baseline_metrics.isLegal()) {
    // baseline 就超限：说明 joint_limits.yaml 与路径本身矛盾（例如路径里有
    // 关节位置跳变）。此时**不输出任何轨迹** —— 任务禁止输出非法轨迹。
    result.note = "baseline trajectory violates joint limits: " +
      result.baseline_metrics.summary;
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "baseline itself is illegal, refusing to output any trajectory: %s",
      result.baseline_metrics.summary.c_str());
    return PlanErrorCode::kJointLimitViolation;
  }

  // 优化关闭：直接采纳 baseline。
  if (!params_.enable_optimization) {
    trajectory = baseline;
    result.optimized_accepted = false;
    result.optimized_metrics = result.baseline_metrics;
    result.note = "optimization disabled by configuration; baseline adopted";
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kSuccess;
  }

  // ---- 第二步：优化版 (TOTG) ----
  robot_trajectory::RobotTrajectory optimized(trajectory);
  std::string totg_error;
  if (!applyTotg(
      optimized, params_.optimized_velocity_scaling,
      params_.optimized_acceleration_scaling, totg_error))
  {
    trajectory = baseline;
    result.fell_back = true;
    result.optimized_metrics = result.baseline_metrics;
    result.note = "TOTG failed, fell back to baseline: " + totg_error;
    RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kSuccess;
  }

  // 可选的 Ruckig 平滑。失败不致命：平滑只是锦上添花，
  // 失败就用未平滑的 TOTG 结果，但要告警说明。
  if (params_.enable_ruckig_smoothing) {
    try {
      if (!trajectory_processing::RuckigSmoothing::applySmoothing(
          optimized, params_.optimized_velocity_scaling,
          params_.optimized_acceleration_scaling))
      {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "Ruckig smoothing returned false; using unsmoothed TOTG result");
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "exception in Ruckig smoothing (%s); using unsmoothed TOTG result", e.what());
    }
  }

  result.optimized_metrics = evaluateTrajectory(optimized, metrics_params);
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName), "optimized (TOTG, scaling %.2f/%.2f): %s",
    params_.optimized_velocity_scaling, params_.optimized_acceleration_scaling,
    result.optimized_metrics.summary.c_str());

  // ---- 第三步：独立复核限位，非法则回退 ----
  // 这是本模块最重要的一条契约：绝不因为"参数化器说它遵守了限位"就相信它。
  if (!result.optimized_metrics.isLegal()) {
    trajectory = baseline;
    result.fell_back = true;
    std::ostringstream oss;
    oss << "optimized trajectory violates joint limits ("
        << result.optimized_metrics.summary << "), fell back to baseline";
    result.note = oss.str();
    RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kSuccess;
  }

  // ---- 第四步：确认真的更快 ----
  if (result.baseline_metrics.duration > 1e-9) {
    result.duration_reduction_ratio =
      (result.baseline_metrics.duration - result.optimized_metrics.duration) /
      result.baseline_metrics.duration;
  }
  if (result.optimized_metrics.duration >= result.baseline_metrics.duration) {
    // "优化"后反而更慢/一样：没有采纳的理由，回退到 baseline。
    // 这种情况在路径极短（只有两三个点）时会出现，属正常。
    trajectory = baseline;
    result.fell_back = true;
    std::ostringstream oss;
    oss << "optimized duration " << result.optimized_metrics.duration
        << "s is not shorter than baseline " << result.baseline_metrics.duration
        << "s, keeping baseline";
    result.note = oss.str();
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
    return PlanErrorCode::kSuccess;
  }

  trajectory = optimized;
  result.optimized_accepted = true;
  result.note = formatComparison(result.baseline_metrics, result.optimized_metrics);
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "%s", result.note.c_str());
  return PlanErrorCode::kSuccess;
}

}  // namespace astribot_s1_manipulation
