// Copyright 2026 Astribot.
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/livox_custom_to_pc2_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(false);
    try {
      auto node =
        std::make_shared<astribot_s1_autonomy::LivoxCustomToPc2Node>(options);
      rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(), 3);
      exec.add_node(node);
      exec.spin();
    } catch (const astribot_s1_autonomy::LivoxConvertConfigError & e) {
      RCLCPP_ERROR(rclcpp::get_logger("livox_custom_to_pc2"),
                   "参数被拒绝：%s", e.what());
      code = 2;
    }
  }
  rclcpp::shutdown();
  return code;
}
