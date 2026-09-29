#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "astribot_s1_navigation_policy_native/cmd_vel_body_to_world_core.hpp"
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"

namespace astribot::navigation {
namespace {

double steady_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

class CmdVelBodyToWorldNode final : public rclcpp::Node {
public:
  CmdVelBodyToWorldNode() : Node("cmd_vel_body_to_world_node") {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Set at startup; changing this setting requires a node restart.";
    declare_parameter("input_topic", "/cmd_vel_nav_body", startup_only);
    declare_parameter("output_topic", "/cmd_vel", startup_only);
    declare_parameter("odom_topic", "/odom", startup_only);
    declare_parameter("normal_height", 0.134);
    declare_parameter("max_height_deviation", 0.06);
    declare_parameter("max_tilt_rad", 0.12);
    declare_parameter("enable_body_to_world", false);
    declare_parameter("enable_posture_monitor", true, startup_only);
    declare_parameter("odom_timeout_sec", 0.5);
    declare_parameter("cmd_timeout_sec", 0.5);

    const auto config = runtime_config();
    core_ = std::make_unique<CmdVelBodyToWorldCore>(config);
    parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          try {
            auto candidate = runtime_config();
            for (const auto& parameter : parameters) {
              const auto& name = parameter.get_name();
              if (name == "enable_body_to_world") candidate.enable_body_to_world = parameter.as_bool();
              else if (name == "normal_height") candidate.normal_height = parameter.as_double();
              else if (name == "max_height_deviation") candidate.max_height_deviation = parameter.as_double();
              else if (name == "max_tilt_rad") candidate.max_tilt_rad = parameter.as_double();
              else if (name == "odom_timeout_sec") candidate.odom_timeout_s = parameter.as_double();
              else if (name == "cmd_timeout_sec") candidate.cmd_timeout_s = parameter.as_double();
            }
            CmdVelBodyToWorldCore::validate_config(candidate);
            result.successful = true;
          } catch (const std::exception& error) {
            result.successful = false;
            result.reason = error.what();
          }
          // The parameter store commits after callbacks accept the whole batch.
          // Control callbacks read that committed store; validation has no side effects.
          return result;
        });

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic, 10,
        std::bind(&CmdVelBodyToWorldNode::on_command, this,
                  std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&CmdVelBodyToWorldNode::on_odom, this,
                  std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CmdVelBodyToWorldNode::on_watchdog, this));
    RCLCPP_INFO(get_logger(),
                "cmd_vel_body_to_world_node C++ direct runtime started: %s + %s -> %s, body_to_world=%s; %s",
                input_topic.c_str(), odom_topic.c_str(), output_topic.c_str(),
                config.enable_body_to_world ? "ON" : "OFF",
                describe_monitor_state(config.enable_posture_monitor, false,
                                       false)
                    .c_str());
  }

  void stop() {
    if (publisher_) publisher_->publish(geometry_msgs::msg::Twist());
  }

private:
  BodyToWorldConfig runtime_config() const {
    BodyToWorldConfig config;
    config.normal_height = get_parameter("normal_height").as_double();
    config.max_height_deviation = get_parameter("max_height_deviation").as_double();
    config.max_tilt_rad = get_parameter("max_tilt_rad").as_double();
    config.enable_body_to_world = get_parameter("enable_body_to_world").as_bool();
    config.enable_posture_monitor = get_parameter("enable_posture_monitor").as_bool();
    config.odom_timeout_s = get_parameter("odom_timeout_sec").as_double();
    config.cmd_timeout_s = get_parameter("cmd_timeout_sec").as_double();
    return config;
  }

  double ros_seconds() { return get_clock()->now().seconds(); }

  void publish_zero() { publisher_->publish(geometry_msgs::msg::Twist()); }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
    core_->update_config(runtime_config());
    const auto& q = message->pose.pose.orientation;
    const auto status = core_->odom(
        q.x, q.y, q.z, q.w, message->pose.pose.position.z,
        stamp_seconds(message->header.stamp), ros_seconds(), steady_seconds());
    if (status.publish_zero) publish_zero();
    if (status.tripped) {
      RCLCPP_ERROR(get_logger(),
                   "%s trigger=%s; safety_tripped requires node restart",
                   describe_monitor_state(true, false, true).c_str(),
                   status.reason.c_str());
    }
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr message) {
    core_->update_config(runtime_config());
    const auto output = core_->command(
        message->linear.x, message->linear.y, message->linear.z,
        message->angular.x, message->angular.y, message->angular.z,
        steady_seconds(), ros_seconds());
    geometry_msgs::msg::Twist converted;
    converted.linear.x = output.linear_x;
    converted.linear.y = output.linear_y;
    converted.angular.z = output.angular_z;
    publisher_->publish(converted);
  }

  void on_watchdog() {
    core_->update_config(runtime_config());
    if (core_->watchdog(steady_seconds(), ros_seconds())) publish_zero();
  }

  std::unique_ptr<CmdVelBodyToWorldCore> core_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace astribot::navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<astribot::navigation::CmdVelBodyToWorldNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception& error) {
    result = 1;
    RCLCPP_FATAL(rclcpp::get_logger("cmd_vel_body_to_world_node"), "%s",
                 error.what());
  }
  rclcpp::shutdown();
  return result;
}
