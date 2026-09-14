// Copyright 2026 Astribot.
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/pointcloud_slice_scan_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(false);
    auto node = std::make_shared<astribot_s1_autonomy::PointcloudSliceScanNode>(options);
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
