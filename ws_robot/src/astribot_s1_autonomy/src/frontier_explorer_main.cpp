// Copyright 2026 Astribot.
//
// 探索节点的独立进程入口。
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/frontier_explorer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(false);
    auto node = std::make_shared<astribot_s1_autonomy::FrontierExplorerNode>(options);
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
