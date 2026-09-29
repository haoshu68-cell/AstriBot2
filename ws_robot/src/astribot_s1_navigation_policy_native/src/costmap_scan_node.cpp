#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "astribot_s1_navigation_policy_native/navigation_math.hpp"

namespace astribot::navigation {
namespace {

class CostmapScanNode final : public rclcpp::Node {
public:
  CostmapScanNode() : Node("navigation_costmap_scan_adapter") {
    declare_parameter("scan_topic", "/scan_from_cloud");
    declare_parameter("max_marking_range_m", 5.5);
    const auto scan_topic = get_parameter("scan_topic").as_string();
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/navigation_policy/costmap_scan", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&CostmapScanNode::on_scan, this, std::placeholders::_1));
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message) {
    try {
      auto converted = *message;
      const std::vector<double> ranges(message->ranges.begin(),
                                       message->ranges.end());
      const auto cleared = costmap_clearing_ranges(
          ranges, message->range_max, get_parameter("max_marking_range_m").as_double());
      converted.ranges.assign(cleared.begin(), cleared.end());
      publisher_->publish(converted);
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "invalid LaserScan: %s", error.what());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace
}  // namespace astribot::navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<astribot::navigation::CostmapScanNode>());
  } catch (const std::exception& error) {
    result = 1;
    RCLCPP_FATAL(rclcpp::get_logger("navigation_costmap_scan_adapter"), "%s",
                 error.what());
  }
  rclcpp::shutdown();
  return result;
}
