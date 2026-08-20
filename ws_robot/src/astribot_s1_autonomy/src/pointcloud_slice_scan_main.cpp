// Copyright 2026 Astribot.
//
// 感知节点的独立进程入口。
// 组件方式（component_container）和独立进程方式共用同一份实现，
// 这里只是把组件包一层 main，便于单独调试、单独 gdb。
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/pointcloud_slice_scan_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    rclcpp::NodeOptions options;
    // 允许通过命令行 --ros-args -p 覆盖参数。
    options.allow_undeclared_parameters(false);
    auto node = std::make_shared<astribot_s1_autonomy::PointcloudSliceScanNode>(options);
    // 本节点的重活在自己的工作线程里，单线程执行器足够；
    // 用多线程执行器也无害，这里保持简单。
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
