#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <nav2_msgs/msg/speed_limit.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "astribot_trajectory_bridge_native/arm_math.hpp"

namespace astribot::trajectory {
namespace {

const std::vector<std::string> kArmJoints = [] {
  std::vector<std::string> result;
  for (const auto& side : {std::string("left"), std::string("right")}) {
    for (int index = 1; index <= 7; ++index) {
      result.push_back("astribot_arm_" + side + "_joint_" +
                       std::to_string(index));
    }
  }
  return result;
}();

const std::vector<std::string> kMonitoredLinks = {
    "astribot_arm_left_tcp_link", "astribot_arm_right_tcp_link",
    "astribot_gripper_left_Link_L11", "astribot_gripper_right_Link_R11"};

class ArmSpeedLimiterNode final : public rclcpp::Node {
public:
  ArmSpeedLimiterNode() : Node("arm_speed_limiter_node") {
    declare_parameter("joint_states_topic", "/joint_states");
    declare_parameter("speed_limit_topic", "/speed_limit");
    declare_parameter("check_period", 0.5);
    declare_parameter("extension_metric", "horizontal_reach");
    declare_parameter("chassis_base_frame", "astribot_torso_base");
    declare_parameter("reach_tf_timeout_sec", 0.5);
    declare_parameter("monitored_links", kMonitoredLinks);
    declare_parameter("extended_reach_m", 0.64);
    declare_parameter("extended_reach_hysteresis_m", 0.03);
    declare_parameter("folded_reference_rad", std::vector<double>(14, 0.0));
    declare_parameter("extended_threshold_rad", 0.5);
    declare_parameter("extended_speed_limit_pct", 50.0);

    const double tf_timeout = get_parameter("reach_tf_timeout_sec").as_double();
    const double period = get_parameter("check_period").as_double();
    if (!std::isfinite(tf_timeout) || tf_timeout <= 0.0 ||
        !std::isfinite(period) || period <= 0.0) {
      throw std::invalid_argument("reach_tf_timeout_sec/check_period must be positive");
    }
    metric_ = get_parameter("extension_metric").as_string();
    if (metric_ != "horizontal_reach" && metric_ != "joint_deviation") {
      RCLCPP_ERROR(get_logger(), "invalid extension_metric; fallback to joint_deviation");
      metric_ = "joint_deviation";
    }
    const double extended_reach_m = get_parameter("extended_reach_m").as_double();
    const double hysteresis_m = get_parameter("extended_reach_hysteresis_m").as_double();
    if (metric_ == "horizontal_reach" &&
        (!(extended_reach_m > 0.0) || hysteresis_m < 0.0 ||
         hysteresis_m >= extended_reach_m)) {
      RCLCPP_ERROR(get_logger(), "invalid reach threshold; fallback to joint_deviation");
      metric_ = "joint_deviation";
    }

    publisher_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
        get_parameter("speed_limit_topic").as_string(), 10);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_states_topic").as_string(), 10,
        std::bind(&ArmSpeedLimiterNode::on_joint_state, this,
                  std::placeholders::_1));
    if (metric_ == "horizontal_reach") {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(period)),
                               std::bind(&ArmSpeedLimiterNode::republish, this));
    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&ArmSpeedLimiterNode::validate_parameter_update, this,
                  std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult validate_parameter_update(
      const std::vector<rclcpp::Parameter>& parameters) const {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    const auto changed = [&parameters](const std::string& name) {
      return std::any_of(parameters.begin(), parameters.end(),
                         [&name](const auto& p) { return p.get_name() == name; });
    };
    const auto candidate = [this, &parameters](const std::string& name) {
      // ROS atomic updates use the last occurrence of a repeated name.
      const auto found = std::find_if(parameters.rbegin(), parameters.rend(),
          [&name](const auto& p) { return p.get_name() == name; });
      return found == parameters.rend() ? get_parameter(name) : *found;
    };
    try {
      if (changed("extended_reach_m") || changed("extended_reach_hysteresis_m")) {
        const double reach = candidate("extended_reach_m").as_double();
        const double hysteresis = candidate("extended_reach_hysteresis_m").as_double();
        if (!std::isfinite(reach) || !std::isfinite(hysteresis) ||
            reach <= 0.0 || hysteresis < 0.0 || hysteresis >= reach) {
          result.reason = "reach must be finite and positive; hysteresis must be finite in [0, reach)";
          return result;
        }
      }
      if (changed("extended_threshold_rad")) {
        const double threshold = candidate("extended_threshold_rad").as_double();
        if (!std::isfinite(threshold) || threshold < 0.0) {
          result.reason = "extended_threshold_rad must be finite and nonnegative";
          return result;
        }
      }
      if (changed("folded_reference_rad")) {
        const auto reference = candidate("folded_reference_rad").as_double_array();
        if (!std::all_of(reference.begin(), reference.end(),
                         [](double value) { return std::isfinite(value); })) {
          result.reason = "folded_reference_rad values must be finite";
          return result;
        }
      }
      if (changed("extended_speed_limit_pct")) {
        const double percentage = candidate("extended_speed_limit_pct").as_double();
        // Nav2 interprets zero as release, not as a zero-speed restriction.
        if (!std::isfinite(percentage) || percentage <= 0.0 || percentage > 100.0) {
          result.reason = "extended_speed_limit_pct must be finite in (0, 100]; zero means release";
          return result;
        }
      }
      if (changed("reach_tf_timeout_sec")) {
        const double timeout = candidate("reach_tf_timeout_sec").as_double();
        if (!std::isfinite(timeout) || timeout <= 0.0) {
          result.reason = "reach_tf_timeout_sec must be finite and positive";
          return result;
        }
      }
    } catch (const std::exception& error) {
      result.reason = std::string("invalid parameter type: ") + error.what();
      return result;
    }
    // Validation never mutates cached classification or commits parameters.
    // ROS commits the complete batch only after every callback accepts it.
    result.successful = true;
    return result;
  }

  void republish() {
    if (!last_extended_.has_value()) return;
    publish_limit(*last_extended_);
  }

  std::optional<bool> horizontal_extended() {
    const auto links = get_parameter("monitored_links").as_string_array();
    const auto base_frame = get_parameter("chassis_base_frame").as_string();
    double best = -1.0;
    bool failed = links.empty();
    for (const auto& link : links) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
            base_frame, link, tf2::TimePointZero);
        const double stamp = static_cast<double>(transform.header.stamp.sec) +
                             static_cast<double>(transform.header.stamp.nanosec) * 1e-9;
        const double age = get_clock()->now().seconds() - stamp;
        const auto& t = transform.transform.translation;
        if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
            ((transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0) &&
             (age < 0.0 || age > get_parameter("reach_tf_timeout_sec").as_double()))) {
          failed = true;
          continue;
        }
        best = std::max(best, astribot_trajectory_bridge_native::horizontal_reach(t.x, t.y));
      } catch (const tf2::TransformException&) {
        failed = true;
      }
    }
    if (failed || best < 0.0) return std::nullopt;
    return astribot_trajectory_bridge_native::is_extended_by_reach(
        best, get_parameter("extended_reach_m").as_double(),
        get_parameter("extended_reach_hysteresis_m").as_double(),
        last_extended_.value_or(false));
  }

  bool joint_extended(const sensor_msgs::msg::JointState& message) const {
    const auto folded_reference = get_parameter("folded_reference_rad").as_double_array();
    std::unordered_map<std::string, double> positions;
    for (std::size_t i = 0; i < message.name.size() && i < message.position.size(); ++i) {
      positions[message.name[i]] = message.position[i];
    }
    double max_deviation = 0.0;
    for (std::size_t i = 0; i < kArmJoints.size() && i < folded_reference.size(); ++i) {
      const auto it = positions.find(kArmJoints[i]);
      if (it != positions.end()) {
        max_deviation = std::max(max_deviation,
                                 std::abs(it->second - folded_reference[i]));
      }
    }
    return max_deviation > get_parameter("extended_threshold_rad").as_double();
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message) {
    bool extended = true;
    if (metric_ == "horizontal_reach") {
      const auto value = horizontal_extended();
      extended = value.value_or(true);
    } else {
      extended = joint_extended(*message);
    }
    if (last_extended_.has_value() && last_extended_.value() == extended) return;
    last_extended_ = extended;
    publish_limit(extended);
  }

  void publish_limit(const bool extended) {
    nav2_msgs::msg::SpeedLimit limit;
    limit.header.stamp = get_clock()->now();
    limit.percentage = true;
    limit.speed_limit = extended ? get_parameter("extended_speed_limit_pct").as_double() : 0.0;
    publisher_->publish(limit);
  }

  std::string metric_;
  std::optional<bool> last_extended_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace
}  // namespace astribot::trajectory

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<astribot::trajectory::ArmSpeedLimiterNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("arm_speed_limiter_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
