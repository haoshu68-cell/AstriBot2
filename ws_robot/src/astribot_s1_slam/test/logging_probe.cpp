#include "log.hpp"
#include <iostream>
#include <thread>
#include <rcl_interfaces/msg/log.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("slam_logging_probe");
  vxlm_log::init(node->get_logger());
  bool rosout_received = false;
  auto subscription = node->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::RosoutQoS(),
      [&rosout_received](rcl_interfaces::msg::Log::ConstSharedPtr message) {
        if(message->msg == "[SYS] ROSOUT_MARK") rosout_received = true;
      });
  LOG_ERROR(SYS, "ROSOUT_MARK");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while(!rosout_received && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if(!rosout_received) return 2;
  int evaluated = 0;
  LOG_DEBUG(EKF, "DEBUG_MARK {}", ++evaluated);
  LOG_INFO(INIT, "INFO_MARK 中文 {}", 42);
  LOG_STARTUP(SYS, "STARTUP_MARK");
  LOG_WARN(LOOP, "WARN_MARK");
  LOG_ERROR(MAP, "ERROR_MARK");
  LOG_DECISION(BACKEND, "DECISION_MARK");
  for(int i = 0; i < 5; ++i) LOG_INFO_THROTTLE(PERF, 2, "THROTTLE_MARK {}", i);
  std::thread worker([] { LOG_WARN(IMU, "THREAD_MARK"); });
  worker.join();
  std::cout << "debug_evaluated=" << evaluated << std::endl;
  LOG_WARN(SYS, "TAIL_MARK");
  node.reset();
  rclcpp::shutdown();
}
