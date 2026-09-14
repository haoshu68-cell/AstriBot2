// Copyright 2026 Astribot.
#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<astribot_s1_autonomy::ExplorationCoordinatorNode>(
      rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[exploration_coordinator_node] 启动或运行失败: %s\n", e.what());
    std::fprintf(
      stderr,
      "[exploration_coordinator_node] 若是参数问题，上面那句已给出具体是哪一项；"
      "若是 'context is invalid'，通常是启动瞬间被 Ctrl-C 打断，重跑即可。\n");
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;                     // 明确的失败码，不要 SIGABRT
  }

  rclcpp::shutdown();
  return 0;
}
