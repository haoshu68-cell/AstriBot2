#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "astribot_s1_chassis_effort_drive_native/wheel_math.hpp"

namespace astribot::chassis_effort {
namespace {

constexpr std::array<const char *, 4> kWheelNames = {
    "wheel_RF_Joint", "wheel_LF_Joint", "wheel_RR_Joint", "wheel_LR_Joint"};
constexpr std::array<const char *, 4> kWheelLabels = {"RF", "LF", "RR", "LR"};

template <typename T>
T parameter_or(rclcpp::Node *node, const std::string &name, const T &fallback) {
  try {
    return node->get_parameter(name).get_value<T>();
  } catch (const std::exception &) {
    return fallback;
  }
}

double clamp_value(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(upper, value));
}

class OmniEffortDriveNode final : public rclcpp::Node {
public:
  OmniEffortDriveNode() : Node("omni_effort_drive_node") {
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("joint_states_topic", "/joint_states");
    declare_parameter("effort_command_topic", "/wheel_effort_controller/commands");
    declare_parameter("wheel_radius", 0.08);
    declare_parameter("wheel_coeff_vx", std::vector<double>{-0.7071, 0.7071, -0.7071, 0.7071});
    declare_parameter("wheel_coeff_vy", std::vector<double>{-0.7071, -0.7071, 0.7071, 0.7071});
    declare_parameter("wheel_coeff_wz", std::vector<double>{-0.3060, -0.3060, -0.3024, -0.3024});
    declare_parameter("kinematics_global_sign", 1.0);
    declare_parameter("pid_kp", 0.4);
    declare_parameter("pid_ki", 0.1);
    declare_parameter("pid_kd", 0.0);
    declare_parameter("friction_coulomb_nm", 0.1);
    declare_parameter("friction_viscous_nm_s", 1.0);
    declare_parameter("friction_deadband_rad_s", 0.05);
    declare_parameter("wheel_effort_limit_nm", 15.0);
    declare_parameter("wheel_velocity_limit_rad_s", 40.0);
    declare_parameter("idle_position_hold", true);
    declare_parameter("idle_position_kp", 3.0);
    declare_parameter("cmd_vel_timeout_sec", 0.5);
    declare_parameter("joint_state_timeout_sec", 0.3);
    declare_parameter("control_period_sec", 0.01);
    declare_parameter("warn_effort_ratio", 0.9);
    declare_parameter("tracking_error_error_rad_s", 3.0);
    declare_parameter("tracking_error_error_duration_sec", 1.0);
    declare_parameter("auto_slowdown_on_tracking_error", false);
    declare_parameter("auto_slowdown_scale", 0.5);
    declare_parameter("log_throttle_sec", 2.0);

    const auto cmd_topic = parameter_or<std::string>(this, "cmd_vel_topic", "/cmd_vel");
    const auto joint_topic = parameter_or<std::string>(this, "joint_states_topic", "/joint_states");
    const auto effort_topic = parameter_or<std::string>(
        this, "effort_command_topic", "/wheel_effort_controller/commands");
    const double period = safe_double("control_period_sec", 0.01);
    const double actual_period = period > 0.0 ? period : 0.01;

    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic, 10, std::bind(&OmniEffortDriveNode::on_cmd, this, std::placeholders::_1));
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10, std::bind(&OmniEffortDriveNode::on_joint_state, this,
                                   std::placeholders::_1));
    effort_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(effort_topic, 10);
    for (std::size_t i = 0; i < kWheelNames.size(); ++i) {
      setpoint_publishers_[i] = create_publisher<std_msgs::msg::Float64>(
          std::string("/wheel_velocity_setpoint/") + kWheelLabels[i], 10);
      effort_debug_publishers_[i] = create_publisher<std_msgs::msg::Float64>(
          std::string("/wheel_effort/") + kWheelLabels[i], 10);
    }
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(actual_period)),
        std::bind(&OmniEffortDriveNode::control_step, this));
  }

private:
  double safe_double(const std::string &name, const double fallback) const {
    try {
      const double value = get_parameter(name).as_double();
      return std::isfinite(value) ? value : fallback;
    } catch (const std::exception &) {
      return fallback;
    }
  }

  bool safe_bool(const std::string &name, const bool fallback) const {
    try {
      return get_parameter(name).as_bool();
    } catch (const std::exception &) {
      return fallback;
    }
  }

  std::vector<double> safe_doubles(const std::string &name,
                                   const std::vector<double> &fallback) const {
    try {
      return get_parameter(name).as_double_array();
    } catch (const std::exception &) {
      return fallback;
    }
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr message) {
    try {
      target_vx_ = message->linear.x;
      target_vy_ = message->linear.y;
      target_wz_ = message->angular.z;
      last_cmd_time_ = get_clock()->now();
      have_cmd_ = true;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "failed to parse cmd_vel: %s", error.what());
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message) {
    try {
      std::unordered_map<std::string, double> velocities;
      std::unordered_map<std::string, double> positions;
      if (message->velocity.size() == message->name.size()) {
        for (std::size_t i = 0; i < message->name.size(); ++i) {
          velocities[message->name[i]] = message->velocity[i];
        }
      }
      if (message->position.size() == message->name.size()) {
        for (std::size_t i = 0; i < message->name.size(); ++i) {
          positions[message->name[i]] = message->position[i];
        }
      }
      wheel_positions_valid_ = true;
      bool updated = false;
      for (std::size_t i = 0; i < kWheelNames.size(); ++i) {
        const auto velocity = velocities.find(kWheelNames[i]);
        const auto position = positions.find(kWheelNames[i]);
        if (velocity != velocities.end()) {
          wheel_velocity_[i] = velocity->second;
          updated = true;
        }
        if (position != positions.end()) {
          wheel_position_[i] = position->second;
        }
        if (position == positions.end() || velocity == velocities.end() ||
            !std::isfinite(wheel_position_[i]) || !std::isfinite(wheel_velocity_[i])) {
          wheel_positions_valid_ = false;
        }
      }
      if (updated) {
        last_joint_time_ = get_clock()->now();
        have_joint_ = true;
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "failed to parse joint_states: %s", error.what());
    }
  }

  std::array<double, 4> inverse_kinematics(const double vx, const double vy,
                                            const double wz) const {
    const double radius = safe_double("wheel_radius", 0.08);
    const auto coeff_vx = safe_doubles("wheel_coeff_vx", {-0.7071, 0.7071, -0.7071, 0.7071});
    const auto coeff_vy = safe_doubles("wheel_coeff_vy", {-0.7071, -0.7071, 0.7071, 0.7071});
    const auto coeff_wz = safe_doubles("wheel_coeff_wz", {-0.3060, -0.3060, -0.3024, -0.3024});
    const double global_sign = safe_double("kinematics_global_sign", 1.0);
    std::array<double, 4> targets{};
    if (radius <= 1e-6 || coeff_vx.size() != 4 || coeff_vy.size() != 4 ||
        coeff_wz.size() != 4 || !std::isfinite(vx) || !std::isfinite(vy) ||
        !std::isfinite(wz) || !std::isfinite(radius) || !std::isfinite(global_sign)) {
      return targets;
    }
    std::array<double, 4> vx_coeff{}, vy_coeff{}, wz_coeff{};
    std::copy(coeff_vx.begin(), coeff_vx.end(), vx_coeff.begin());
    std::copy(coeff_vy.begin(), coeff_vy.end(), vy_coeff.begin());
    std::copy(coeff_wz.begin(), coeff_wz.end(), wz_coeff.begin());
    return astribot::chassis_effort::inverse_kinematics(
        vx, vy, wz, radius, vx_coeff, vy_coeff, wz_coeff, global_sign);
  }

  bool stale(const rclcpp::Time &now, const rclcpp::Time &stamp,
             const bool available, const double timeout) const {
    return !available || (now - stamp).seconds() > timeout;
  }

  void control_step() {
    const rclcpp::Time now = get_clock()->now();
    const double dt = safe_double("control_period_sec", 0.01);
    double vx = target_vx_;
    double vy = target_vy_;
    double wz = target_wz_;
    if (stale(now, last_cmd_time_, have_cmd_, safe_double("cmd_vel_timeout_sec", 0.5))) {
      vx = vy = wz = 0.0;
    }
    const bool feedback_stale = stale(
        now, last_joint_time_, have_joint_, safe_double("joint_state_timeout_sec", 0.3));
    const auto targets = inverse_kinematics(vx, vy, wz);
    const double velocity_limit = safe_double("wheel_velocity_limit_rad_s", 40.0);
    const double kp = safe_double("pid_kp", 0.4);
    const double ki = safe_double("pid_ki", 0.1);
    const double kd = safe_double("pid_kd", 0.0);
    const double tau_c = safe_double("friction_coulomb_nm", 0.1);
    const double tau_v = safe_double("friction_viscous_nm_s", 1.0);
    const double deadband = safe_double("friction_deadband_rad_s", 0.05);
    const double tau_max = safe_double("wheel_effort_limit_nm", 15.0);
    const double warn_ratio = safe_double("warn_effort_ratio", 0.9);

    const bool idle = safe_bool("idle_position_hold", true) &&
                      std::max({std::abs(vx), std::abs(vy), std::abs(wz)}) <= 1e-9 &&
                      !feedback_stale && wheel_positions_valid_ &&
                      std::all_of(wheel_position_.begin(), wheel_position_.end(),
                                  [](const double value) { return std::isfinite(value); });
    if (!idle) {
      idle_reference_.reset();
    } else if (!idle_reference_.has_value() &&
               std::all_of(wheel_velocity_.begin(), wheel_velocity_.end(),
                           [](const double value) { return std::abs(value) < 0.05; })) {
      idle_reference_ = wheel_position_;
    }

    std_msgs::msg::Float64MultiArray effort_message;
    effort_message.data.reserve(kWheelNames.size());
    for (std::size_t i = 0; i < kWheelNames.size(); ++i) {
      const double target = targets[i];
      const double clamped_target = clamp_value(target, -velocity_limit, velocity_limit);
      if (std::abs(clamped_target - target) > 1e-6) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%s target %.2f exceeds +/- %.2f; clamped",
                             kWheelNames[i], target, velocity_limit);
      }
      if (feedback_stale) {
        effort_message.data.push_back(0.0);
        loops_[i].reset();
        continue;
      }
      const auto result = loops_[i].update(
          clamped_target, wheel_velocity_[i], kp, ki, kd, tau_c, tau_v,
          deadband, tau_max, dt);
      double effort = result[0];
      const double error = result[1];
      if (idle_reference_.has_value()) {
        double gain = safe_double("idle_position_kp", 3.0);
        if (!std::isfinite(gain) || gain < 0.0 || gain > 10.0) gain = 0.0;
        effort = clamp_value(effort + gain * (idle_reference_->at(i) - wheel_position_[i]),
                             -tau_max, tau_max);
      }
      effort_message.data.push_back(effort);
      if (std::abs(effort) > warn_ratio * tau_max) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%s effort %.2f is near +/- %.2f", kWheelNames[i], effort, tau_max);
      }
      std_msgs::msg::Float64 setpoint_message;
      setpoint_message.data = clamped_target;
      setpoint_publishers_[i]->publish(setpoint_message);
      std_msgs::msg::Float64 debug_message;
      debug_message.data = effort;
      effort_debug_publishers_[i]->publish(debug_message);
      check_tracking_error(i, error, now);
    }
    if (feedback_stale && have_joint_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "joint_states stale; all wheel efforts set to zero");
    }
    effort_publisher_->publish(effort_message);
  }

  void check_tracking_error(const std::size_t index, const double error,
                            const rclcpp::Time &now) {
    const double threshold = safe_double("tracking_error_error_rad_s", 3.0);
    const double duration = safe_double("tracking_error_error_duration_sec", 1.0);
    if (std::abs(error) <= threshold) {
      error_high_since_[index].reset();
      return;
    }
    if (!error_high_since_[index].has_value()) {
      error_high_since_[index] = now;
      return;
    }
    if ((now - *error_high_since_[index]).seconds() >= duration) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "%s tracking error %.2f exceeds %.2f", kWheelNames[index],
                            error, threshold);
      if (safe_bool("auto_slowdown_on_tracking_error", false)) {
        const double scale = safe_double("auto_slowdown_scale", 0.5);
        target_vx_ *= scale;
        target_vy_ *= scale;
        target_wz_ *= scale;
      }
    }
  }

  double target_vx_{0.0};
  double target_vy_{0.0};
  double target_wz_{0.0};
  std::array<double, 4> wheel_velocity_{};
  std::array<double, 4> wheel_position_{};
  bool wheel_positions_valid_{false};
  bool have_cmd_{false};
  bool have_joint_{false};
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_joint_time_;
  std::array<WheelLoop, 4> loops_;
  std::array<std::optional<rclcpp::Time>, 4> error_high_since_;
  std::optional<std::array<double, 4>> idle_reference_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> setpoint_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> effort_debug_publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace astribot::chassis_effort

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astribot::chassis_effort::OmniEffortDriveNode>());
  rclcpp::shutdown();
  return 0;
}
