// Copyright 2026 Astribot

#include "astribot_s1_manipulation/closed_chain_constraint.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include <rclcpp/logging.hpp>

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.closed_chain";

/// 检查一个组是否存在且可用于 IK（必须是链）。
bool validateGroupForIk(
  const moveit::core::RobotModelConstPtr & model,
  const std::string & group_name,
  const moveit::core::JointModelGroup ** out_group,
  std::string & error)
{
  if (group_name.empty()) {
    error = "group name is empty";
    return false;
  }
  const moveit::core::JointModelGroup * jmg = model->getJointModelGroup(group_name);
  if (jmg == nullptr) {
    error = "planning group '" + group_name + "' not found in SRDF";
    return false;
  }
  if (!jmg->isChain()) {
    error = "planning group '" + group_name +
      "' is not a kinematic chain, cannot be used for IK";
    return false;
  }
  *out_group = jmg;
  return true;
}
}  // namespace

bool ClosedChainConstraint::configure(
  const ClosedChainParams & params,
  const moveit::core::RobotModelConstPtr & robot_model,
  std::string & error)
{
  error.clear();
  configured_ = false;
  relative_pose_known_ = false;

  if (!robot_model) {
    error = "robot model pointer is null";
    return false;
  }

  if (params.leader_group == params.follower_group) {
    error = "leader_group and follower_group must differ (both are '" +
      params.leader_group + "')";
    return false;
  }

  const moveit::core::JointModelGroup * leader = nullptr;
  const moveit::core::JointModelGroup * follower = nullptr;
  if (!validateGroupForIk(robot_model, params.leader_group, &leader, error)) {
    return false;
  }
  if (!validateGroupForIk(robot_model, params.follower_group, &follower, error)) {
    return false;
  }

  if (!robot_model->hasLinkModel(params.leader_tcp_link)) {
    error = "leader TCP link '" + params.leader_tcp_link + "' not found in robot model";
    return false;
  }
  if (!robot_model->hasLinkModel(params.follower_tcp_link)) {
    error = "follower TCP link '" + params.follower_tcp_link + "' not found in robot model";
    return false;
  }

  if (!(params.position_tolerance > 0.0)) {
    error = "position_tolerance must be > 0";
    return false;
  }
  if (!(params.orientation_tolerance > 0.0)) {
    error = "orientation_tolerance must be > 0";
    return false;
  }
  if (!(params.ik_timeout > 0.0)) {
    error = "ik_timeout must be > 0";
    return false;
  }
  if (params.ik_attempts < 1) {
    error = "ik_attempts must be >= 1";
    return false;
  }

  Eigen::Isometry3d explicit_relative_pose = Eigen::Isometry3d::Identity();
  if (!params.capture_from_current_state) {
    Eigen::Quaterniond q(
      params.relative_rotation_xyzw[3],   // w
      params.relative_rotation_xyzw[0],   // x
      params.relative_rotation_xyzw[1],   // y
      params.relative_rotation_xyzw[2]);  // z
    const double norm = q.norm();
    if (!(norm > 1e-9)) {
      error = "relative_rotation_xyzw is a zero quaternion";
      return false;
    }
    q.normalize();
    explicit_relative_pose.linear() = q.toRotationMatrix();
    explicit_relative_pose.translation() = Eigen::Vector3d(
      params.relative_translation[0],
      params.relative_translation[1],
      params.relative_translation[2]);
  }

  if (follower->getSolverInstance() == nullptr) {
    error = "follower group '" + params.follower_group +
      "' has no IK solver configured; check kinematics.yaml";
    return false;
  }

  params_ = params;
  robot_model_ = robot_model;
  leader_group_ = leader;
  follower_group_ = follower;

  if (!params.capture_from_current_state) {
    relative_pose_ = explicit_relative_pose;
    relative_pose_known_ = true;
  }

  configured_ = true;
  return true;
}

void ClosedChainConstraint::setRelativePose(const Eigen::Isometry3d & relative_pose) noexcept
{
  relative_pose_ = relative_pose;
  relative_pose_known_ = true;
}

bool ClosedChainConstraint::captureRelativePose(
  const moveit::core::RobotState & state, std::string & error)
{
  error.clear();
  if (!configured_) {
    error = "ClosedChainConstraint not configured";
    return false;
  }

  moveit::core::RobotState local_state(state);
  local_state.updateLinkTransforms();

  const Eigen::Isometry3d & t_leader =
    local_state.getGlobalLinkTransform(params_.leader_tcp_link);
  const Eigen::Isometry3d & t_follower =
    local_state.getGlobalLinkTransform(params_.follower_tcp_link);

  relative_pose_ = t_leader.inverse() * t_follower;
  relative_pose_known_ = true;

  const Eigen::Vector3d & p = relative_pose_.translation();
  const Eigen::AngleAxisd aa(relative_pose_.linear());
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "captured closed-chain T_rel from current state: "
    "translation=(%.4f, %.4f, %.4f) m, rotation=%.4f rad about (%.3f, %.3f, %.3f), "
    "|translation|=%.4f m",
    p.x(), p.y(), p.z(), aa.angle(), aa.axis().x(), aa.axis().y(), aa.axis().z(), p.norm());
  return true;
}

bool ClosedChainConstraint::computeFollowerTarget(
  const moveit::core::RobotState & state,
  Eigen::Isometry3d & follower_target,
  std::string & error) const
{
  error.clear();
  if (!configured_) {
    error = "ClosedChainConstraint not configured";
    return false;
  }
  if (!relative_pose_known_) {
    error = "relative pose T_rel unknown; call captureRelativePose() first "
      "or set capture_from_current_state=false with explicit values";
    return false;
  }

  moveit::core::RobotState local_state(state);
  local_state.updateLinkTransforms();
  const Eigen::Isometry3d & t_leader =
    local_state.getGlobalLinkTransform(params_.leader_tcp_link);

  follower_target = t_leader * relative_pose_;
  return true;
}

ConstraintResidual ClosedChainConstraint::computeResidual(
  const moveit::core::RobotState & state) const
{
  ConstraintResidual residual;

  if (!configured_) {
    residual.reason = "ClosedChainConstraint not configured";
    return residual;
  }
  if (!relative_pose_known_) {
    residual.reason = "relative pose T_rel unknown";
    return residual;
  }

  moveit::core::RobotState local_state(state);
  local_state.updateLinkTransforms();
  const Eigen::Isometry3d & t_leader =
    local_state.getGlobalLinkTransform(params_.leader_tcp_link);
  const Eigen::Isometry3d & t_follower =
    local_state.getGlobalLinkTransform(params_.follower_tcp_link);

  const Eigen::Isometry3d actual_relative = t_leader.inverse() * t_follower;
  const Eigen::Isometry3d error_transform = actual_relative * relative_pose_.inverse();

  residual.position_error = error_transform.translation().norm();

  const Eigen::Matrix3d & rot = error_transform.linear();
  if (!rot.allFinite()) {
    residual.reason = "error transform contains non-finite values";
    return residual;
  }
  const Eigen::AngleAxisd angle_axis(rot);
  residual.orientation_error = std::abs(angle_axis.angle());
  if (!std::isfinite(residual.orientation_error) ||
    !std::isfinite(residual.position_error))
  {
    residual.reason = "residual is not finite";
    return residual;
  }

  residual.valid = true;
  residual.within_tolerance =
    residual.position_error <= params_.position_tolerance &&
    residual.orientation_error <= params_.orientation_tolerance;

  std::ostringstream oss;
  oss << "e_p=" << residual.position_error << "m (tol " << params_.position_tolerance
      << "), e_r=" << residual.orientation_error << "rad (tol "
      << params_.orientation_tolerance << ")";
  residual.reason = oss.str();
  return residual;
}

bool ClosedChainConstraint::projectFollower(
  moveit::core::RobotState & state,
  ConstraintResidual & residual,
  std::string & error) const
{
  error.clear();
  residual = ConstraintResidual{};

  if (!configured_) {
    error = "ClosedChainConstraint not configured";
    return false;
  }
  if (!relative_pose_known_) {
    error = "relative pose T_rel unknown";
    return false;
  }
  if (follower_group_ == nullptr) {
    error = "follower group pointer is null";
    return false;
  }

  Eigen::Isometry3d follower_target;
  if (!computeFollowerTarget(state, follower_target, error)) {
    return false;
  }

  const bool ik_ok = state.setFromIK(
    follower_group_,
    follower_target,
    params_.follower_tcp_link,
    params_.ik_timeout);

  if (!ik_ok) {
    const Eigen::Vector3d & p = follower_target.translation();
    std::ostringstream oss;
    oss << "follower IK failed for target translation ("
        << p.x() << ", " << p.y() << ", " << p.z() << ")"
        << " with timeout " << params_.ik_timeout << "s";
    error = oss.str();
    return false;
  }

  state.update();
  residual = computeResidual(state);
  if (!residual.valid) {
    error = "residual computation failed after IK: " + residual.reason;
    return false;
  }
  if (!residual.within_tolerance) {
    error = "closed-chain residual out of tolerance after IK: " + residual.reason;
    return false;
  }

  return true;
}

}  // namespace astribot_s1_manipulation
