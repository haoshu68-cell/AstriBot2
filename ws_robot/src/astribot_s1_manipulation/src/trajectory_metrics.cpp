// Copyright 2026 Astribot

#include "astribot_s1_manipulation/trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

namespace astribot_s1_manipulation
{

namespace
{

/// 从 RobotModel 取某个变量的速度/加速度限位。
/// MoveIt 会把 URDF 的限位和 joint_limits.yaml 的限位合并到 VariableBounds，
/// 所以这里读到的就是"最终生效"的硬限位 —— 正是判超限该用的值。
void readBounds(
  const moveit::core::JointModel * joint_model,
  const std::string & variable_name,
  double * velocity_limit,
  double * acceleration_limit)
{
  *velocity_limit = 0.0;
  *acceleration_limit = 0.0;
  if (joint_model == nullptr) {
    return;
  }
  const moveit::core::VariableBounds & bounds = joint_model->getVariableBounds(variable_name);
  if (bounds.velocity_bounded_) {
    *velocity_limit = std::max(std::abs(bounds.max_velocity_), std::abs(bounds.min_velocity_));
  }
  if (bounds.acceleration_bounded_) {
    *acceleration_limit =
      std::max(std::abs(bounds.max_acceleration_), std::abs(bounds.min_acceleration_));
  }
}

}  // namespace

TrajectoryMetrics evaluateTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const MetricsParams & params)
{
  TrajectoryMetrics metrics;

  const std::size_t count = trajectory.getWayPointCount();
  metrics.waypoint_count = count;

  if (count == 0U) {
    metrics.summary = "empty trajectory";
    return metrics;
  }
  if (count == 1U) {
    metrics.valid = true;
    metrics.source = MetricsSource::kFromTrajectory;
    metrics.duration = 0.0;
    metrics.summary = "single-waypoint trajectory: no motion";
    return metrics;
  }

  const moveit::core::JointModelGroup * jmg = trajectory.getGroup();
  if (jmg == nullptr) {
    metrics.summary = "trajectory has no joint model group";
    return metrics;
  }

  metrics.duration = trajectory.getDuration();

  const std::vector<std::string> & variable_names = jmg->getVariableNames();
  if (variable_names.empty()) {
    metrics.summary = "joint model group '" + jmg->getName() + "' has no variables";
    return metrics;
  }

  const moveit::core::RobotState & probe = trajectory.getWayPoint(count > 1U ? 1U : 0U);
  const bool has_velocities = probe.hasVelocities();
  const bool has_accelerations = probe.hasAccelerations();

  if (!has_velocities) {
    if (!params.allow_finite_difference) {
      metrics.summary = "trajectory has no velocity data and finite difference is disabled";
      return metrics;
    }
    metrics.source = MetricsSource::kFiniteDifference;
  } else {
    metrics.source = MetricsSource::kFromTrajectory;
  }

  metrics.per_joint.reserve(variable_names.size());

  for (const std::string & variable_name : variable_names) {
    JointPeak peak;
    peak.joint_name = variable_name;

    const moveit::core::JointModel * joint_model =
      trajectory.getRobotModel()->getJointOfVariable(variable_name);
    readBounds(joint_model, variable_name, &peak.velocity_limit, &peak.acceleration_limit);

    for (std::size_t i = 0; i < count; ++i) {
      const moveit::core::RobotState & state = trajectory.getWayPoint(i);

      double velocity = 0.0;
      if (has_velocities) {
        velocity = state.getVariableVelocity(variable_name);
      } else if (i + 1U < count) {
        const double dt = trajectory.getWayPointDurationFromPrevious(i + 1U);
        if (dt > 1e-9) {
          const double q0 = state.getVariablePosition(variable_name);
          const double q1 = trajectory.getWayPoint(i + 1U).getVariablePosition(variable_name);
          velocity = (q1 - q0) / dt;
        }
      }
      if (std::isfinite(velocity)) {
        peak.max_velocity = std::max(peak.max_velocity, std::abs(velocity));
      }

      if (has_accelerations) {
        const double acceleration = state.getVariableAcceleration(variable_name);
        if (std::isfinite(acceleration)) {
          peak.max_acceleration = std::max(peak.max_acceleration, std::abs(acceleration));
        }
      }
    }

    if (peak.velocity_limit > 0.0) {
      const double allowed = peak.velocity_limit * (1.0 + params.limit_tolerance_ratio);
      peak.velocity_violated = peak.max_velocity > allowed;
      peak.velocity_utilization = 100.0 * peak.max_velocity / peak.velocity_limit;
    }
    if (peak.acceleration_limit > 0.0 && has_accelerations) {
      const double allowed = peak.acceleration_limit * (1.0 + params.limit_tolerance_ratio);
      peak.acceleration_violated = peak.max_acceleration > allowed;
    }

    if (peak.max_velocity > metrics.max_joint_velocity) {
      metrics.max_joint_velocity = peak.max_velocity;
      metrics.max_velocity_joint = variable_name;
    }
    if (peak.max_acceleration > metrics.max_joint_acceleration) {
      metrics.max_joint_acceleration = peak.max_acceleration;
      metrics.max_acceleration_joint = variable_name;
    }
    metrics.peak_velocity_utilization =
      std::max(metrics.peak_velocity_utilization, peak.velocity_utilization);

    if (peak.velocity_violated) {
      metrics.velocity_limit_violated = true;
      metrics.violating_joints.push_back(variable_name + "(velocity)");
    }
    if (peak.acceleration_violated) {
      metrics.acceleration_limit_violated = true;
      metrics.violating_joints.push_back(variable_name + "(acceleration)");
    }

    metrics.per_joint.push_back(peak);
  }

  metrics.valid = true;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "duration=" << metrics.duration << "s"
      << " waypoints=" << metrics.waypoint_count
      << " max_vel=" << metrics.max_joint_velocity << "rad/s"
      << " (" << (metrics.max_velocity_joint.empty() ? "-" : metrics.max_velocity_joint) << ")"
      << " max_acc=" << metrics.max_joint_acceleration << "rad/s^2"
      << " (" << (metrics.max_acceleration_joint.empty() ? "-" : metrics.max_acceleration_joint)
      << ")"
      << " peak_vel_util=" << std::setprecision(1) << metrics.peak_velocity_utilization << "%"
      << " source=" << (metrics.source == MetricsSource::kFromTrajectory ?
    "trajectory" : "finite-difference");
  if (!metrics.isLegal()) {
    oss << " ILLEGAL[";
    for (std::size_t i = 0; i < metrics.violating_joints.size(); ++i) {
      if (i != 0U) {
        oss << ",";
      }
      oss << metrics.violating_joints[i];
    }
    oss << "]";
  }
  metrics.summary = oss.str();
  return metrics;
}

std::string formatComparison(
  const TrajectoryMetrics & baseline,
  const TrajectoryMetrics & optimized)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "节拍对比: ";
  if (!baseline.valid || !optimized.valid) {
    oss << "对比不可用 (baseline valid=" << (baseline.valid ? "yes" : "no")
        << ", optimized valid=" << (optimized.valid ? "yes" : "no") << ")";
    return oss.str();
  }

  oss << "时长 " << baseline.duration << "s -> " << optimized.duration << "s";
  if (baseline.duration > 1e-9) {
    const double ratio = 100.0 * (baseline.duration - optimized.duration) / baseline.duration;
    oss << " (缩短 " << std::setprecision(1) << ratio << "%)" << std::setprecision(3);
  }
  oss << " | 最大关节速度 " << baseline.max_joint_velocity
      << " -> " << optimized.max_joint_velocity << " rad/s";
  oss << " | 速度利用率 " << std::setprecision(1)
      << baseline.peak_velocity_utilization << "% -> "
      << optimized.peak_velocity_utilization << "%";
  oss << " | 优化后合法性: " << (optimized.isLegal() ? "合法" : "超限");
  return oss.str();
}

}  // namespace astribot_s1_manipulation
