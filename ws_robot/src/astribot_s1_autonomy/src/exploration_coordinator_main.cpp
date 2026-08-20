// Copyright 2026 Astribot.
//
// 独立进程入口。协调器的 action 回调、订阅回调、定时器要能并发推进，
// 所以必须用多线程执行器：单线程执行器下，定时器里跑前沿搜索的那几十毫秒
// 会把 action 结果回调压在队列后面，导航结果的处理被拖慢。
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<astribot_s1_autonomy::ExplorationCoordinatorNode>(
    rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
