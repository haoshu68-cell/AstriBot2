// Copyright 2026 Astribot.
//
// CustomMsg → PointCloud2 转换节点的独立进程入口。
//
// 用多线程执行器：两路输入各约 10Hz、每帧约两万点，转换有实打实的计算量。
// 单线程执行器会把两路串行化，front 的转换会推迟 back 的回调，
// 抖动时表现成两路交替丢帧 —— 而 BEST_EFFORT 丢帧是静默的。
// 线程数给 3（两路 + 上报定时器）。
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
      // 参数被拒绝就不要起来。静默按错参数跑的后果是假点团进 costmap。
      RCLCPP_ERROR(rclcpp::get_logger("livox_custom_to_pc2"),
                   "参数被拒绝：%s", e.what());
      code = 2;
    }
  }
  rclcpp::shutdown();
  return code;
}
