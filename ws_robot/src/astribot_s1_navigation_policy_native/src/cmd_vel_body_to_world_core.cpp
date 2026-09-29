#include "astribot_s1_navigation_policy_native/cmd_vel_body_to_world_core.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

#include "astribot_s1_navigation_policy_native/navigation_math.hpp"

namespace astribot::navigation {
namespace {

bool finite_all(const std::initializer_list<double> values) {
  return std::all_of(values.begin(), values.end(),
                     [](const double value) { return std::isfinite(value); });
}

}  // namespace

CmdVelBodyToWorldCore::CmdVelBodyToWorldCore(BodyToWorldConfig config)
    : config_(config) {
  validate_config(config_);
  attitude_samples_.reserve(64U);
}

void CmdVelBodyToWorldCore::validate_config(const BodyToWorldConfig& config) {
  if (!finite_all({config.odom_timeout_s, config.cmd_timeout_s,
                   config.normal_height, config.max_height_deviation,
                   config.max_tilt_rad}) ||
      config.odom_timeout_s <= 0.0 || config.cmd_timeout_s <= 0.0 ||
      config.max_height_deviation < 0.0 || config.max_tilt_rad < 0.0) {
    throw std::invalid_argument(
        "body-to-world settings must be finite; timeouts must be positive and "
        "posture tolerances nonnegative");
  }
}

void CmdVelBodyToWorldCore::update_config(const BodyToWorldConfig& config) {
  validate_config(config);
  config_ = config;
}

bool CmdVelBodyToWorldCore::ready(const double ros_now_s,
                                  const double steady_now_s) const {
  if (!config_.enable_body_to_world && !config_.enable_posture_monitor) {
    return true;
  }
  if (!has_odom_ || !odom_valid_ || !finite_all({ros_now_s, steady_now_s})) {
    return false;
  }
  return true;
}

OdomStatus CmdVelBodyToWorldCore::odom(
    const double qx, const double qy, const double qz, const double qw,
    const double z, const double stamp_s, const double ros_now_s,
    const double steady_now_s) {
  OdomStatus status;
  if(has_odom_ && std::isfinite(stamp_s) && stamp_s<last_odom_stamp_s_)return status;
  has_odom_ = true;
  last_odom_received_s_ = steady_now_s;
  last_odom_stamp_s_ = stamp_s;
  const double norm = qx * qx + qy * qy + qz * qz + qw * qw;
  odom_valid_ = finite_all({qx, qy, qz, qw, z, stamp_s}) &&
                std::abs(norm - 1.0) < 0.01;
  if (!odom_valid_ || !ready(ros_now_s, steady_now_s)) {
    status.publish_zero = config_.enable_body_to_world ||
                          config_.enable_posture_monitor;
    return status;
  }

  current_yaw_ = std::atan2(2.0 * (qw * qz + qy * qx),
                            1.0 - 2.0 * (qy * qy + qz * qz));
  if (!config_.enable_posture_monitor || safety_tripped_) return status;

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
  const double pitch = std::asin(sinp);
  if (attitude_samples_.size() < 64U) {
    attitude_samples_.push_back({z, roll, pitch});
  }
  const auto evaluation = evaluate_posture(
      true, attitude_samples_, z, roll, pitch, config_.normal_height,
      config_.max_height_deviation, config_.max_tilt_rad, 20U);
  if (evaluation.first == "trip") {
    safety_tripped_ = true;
    status.publish_zero = true;
    status.tripped = true;
    status.reason = evaluation.second;
  }
  return status;
}

CommandStatus CmdVelBodyToWorldCore::command(
    const double linear_x, const double linear_y, const double linear_z,
    const double angular_x, const double angular_y, const double angular_z,
    const double steady_now_s, const double ros_now_s) {
  has_cmd_ = true;
  last_cmd_received_s_ = steady_now_s;
  CommandStatus status;
  const bool valid = finite_all({linear_x, linear_y, linear_z, angular_x,
                                 angular_y, angular_z, steady_now_s,
                                 ros_now_s});
  if (safety_tripped_ || !valid || !ready(ros_now_s, steady_now_s)) {
    return status;
  }
  status.angular_z = angular_z;
  if (config_.enable_body_to_world) {
    const auto world = body_to_world_xy(linear_x, linear_y, current_yaw_);
    status.linear_x = world[0];
    status.linear_y = world[1];
  } else {
    status.linear_x = linear_x;
    status.linear_y = linear_y;
  }
  return status;
}

bool CmdVelBodyToWorldCore::watchdog(const double steady_now_s,
                                     const double ros_now_s) {
  if (!has_cmd_) return false;
  if (safety_tripped_ || !ready(ros_now_s, steady_now_s) ||
      !std::isfinite(steady_now_s) ||
      steady_now_s - last_cmd_received_s_ > config_.cmd_timeout_s) {
    has_cmd_ = false;
    return true;
  }
  return false;
}

}  // namespace astribot::navigation
