// Copyright 2026 Astribot

#include "astribot_s1_manipulation/singularity_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.singularity";
}  // namespace

bool SingularityMonitor::configure(const SingularityParams & params, std::string & error)
{
  error.clear();

  if (!(params.min_singular_value > 0.0)) {
    error = "min_singular_value must be > 0, got " + std::to_string(params.min_singular_value);
    return false;
  }
  if (!(params.max_condition_number > 1.0)) {
    error = "max_condition_number must be > 1.0, got " +
      std::to_string(params.max_condition_number);
    return false;
  }
  if (!(params.degenerate_jacobian_epsilon > 0.0)) {
    error = "degenerate_jacobian_epsilon must be > 0, got " +
      std::to_string(params.degenerate_jacobian_epsilon);
    return false;
  }

  params_ = params;
  configured_ = true;
  return true;
}

SingularityReport SingularityMonitor::check(
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tip_link_name) const
{
  SingularityReport report;

  if (!configured_) {
    report.reason = "SingularityMonitor not configured";
    return report;
  }
  if (jmg == nullptr) {
    report.reason = "joint model group pointer is null";
    return report;
  }
  if (tip_link_name.empty()) {
    report.reason = "tip link name is empty";
    return report;
  }

  if (!jmg->isChain()) {
    report.reason = "group '" + jmg->getName() +
      "' is not a kinematic chain; check each arm separately";
    return report;
  }

  const moveit::core::RobotModelConstPtr & model = state.getRobotModel();
  if (!model) {
    report.reason = "robot model is null";
    return report;
  }
  const moveit::core::LinkModel * tip_link = model->getLinkModel(tip_link_name);
  if (tip_link == nullptr) {
    report.reason = "tip link '" + tip_link_name + "' not found in robot model";
    return report;
  }

  moveit::core::RobotState local_state(state);
  local_state.updateLinkTransforms();

  Eigen::MatrixXd jacobian;
  const bool ok = local_state.getJacobian(
    jmg, tip_link, Eigen::Vector3d::Zero(), jacobian, false);
  if (!ok) {
    report.reason = "getJacobian failed for group '" + jmg->getName() +
      "' tip '" + tip_link_name + "'";
    return report;
  }
  if (jacobian.rows() == 0 || jacobian.cols() == 0) {
    report.reason = "jacobian is empty";
    return report;
  }
  if (!jacobian.allFinite()) {
    report.reason = "jacobian contains non-finite values";
    return report;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const Eigen::VectorXd & sv = svd.singularValues();
  if (sv.size() == 0) {
    report.reason = "SVD produced no singular values";
    return report;
  }

  report.max_singular_value = sv(0);
  report.min_singular_value = sv(sv.size() - 1);
  report.valid = true;

  if (report.max_singular_value < params_.degenerate_jacobian_epsilon) {
    report.condition_number = std::numeric_limits<double>::infinity();
    report.singular = true;
    std::ostringstream oss;
    oss << "degenerate jacobian: sigma_max=" << report.max_singular_value
        << " < epsilon=" << params_.degenerate_jacobian_epsilon;
    report.reason = oss.str();
    return report;
  }

  if (report.min_singular_value < params_.degenerate_jacobian_epsilon) {
    report.condition_number = std::numeric_limits<double>::infinity();
  } else {
    report.condition_number = report.max_singular_value / report.min_singular_value;
  }

  if (!params_.enabled) {
    report.singular = false;
    report.reason = "singularity check disabled by configuration";
    return report;
  }

  std::ostringstream oss;
  if (report.min_singular_value < params_.min_singular_value) {
    report.singular = true;
    oss << "sigma_min=" << report.min_singular_value
        << " below threshold " << params_.min_singular_value;
    report.reason = oss.str();
    return report;
  }
  if (report.condition_number > params_.max_condition_number) {
    report.singular = true;
    oss << "condition_number=" << report.condition_number
        << " above threshold " << params_.max_condition_number;
    report.reason = oss.str();
    return report;
  }

  report.singular = false;
  oss << "ok: sigma_min=" << report.min_singular_value
      << " cond=" << report.condition_number;
  report.reason = oss.str();
  return report;
}

SingularityReport SingularityMonitor::checkStates(
  const std::vector<moveit::core::RobotState> & states,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tip_link_name,
  std::size_t * worst_index) const
{
  SingularityReport worst;
  if (states.empty()) {
    worst.reason = "no states to check";
    return worst;
  }

  double worst_sigma = std::numeric_limits<double>::infinity();
  std::size_t worst_i = 0;
  bool any_valid = false;
  bool still_escaping = params_.allow_singular_start;

  for (std::size_t i = 0; i < states.size(); ++i) {
    const SingularityReport r = check(states[i], jmg, tip_link_name);
    if (!r.valid) {
      if (worst_index != nullptr) {
        *worst_index = i;
      }
      return r;
    }
    any_valid = true;

    if (!r.singular) {
      still_escaping = false;
    } else if (still_escaping) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "waypoint %zu is still singular (%s) but exempt as part of the escape "
        "from a singular start; the trajectory is allowed to move away from it",
        i, r.reason.c_str());
    } else {
      if (worst_index != nullptr) {
        *worst_index = i;
      }
      return r;
    }

    if (r.min_singular_value < worst_sigma) {
      worst_sigma = r.min_singular_value;
      worst_i = i;
      worst = r;
    }
  }

  if (!any_valid) {
    worst.valid = false;
    worst.reason = "no valid jacobian on any state";
  }
  if (worst_index != nullptr) {
    *worst_index = worst_i;
  }

  if (worst.valid && worst.singular) {
    std::ostringstream oss;
    oss << "ok (start waypoint exempt): worst sigma_min=" << worst.min_singular_value
        << " at index " << worst_i
        << ", no non-exempt waypoint is singular";
    worst.reason = oss.str();
    worst.singular = false;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger(kLoggerName),
    "checked %zu states, worst sigma_min=%.6f at index %zu",
    states.size(), worst.min_singular_value, worst_i);
  return worst;
}

}  // namespace astribot_s1_manipulation
