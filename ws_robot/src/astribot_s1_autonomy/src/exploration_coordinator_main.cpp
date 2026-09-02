// Copyright 2026 Astribot.
//
// 独立进程入口。协调器的 action 回调、订阅回调、定时器要能并发推进，
// 所以必须用多线程执行器：单线程执行器下，定时器里跑前沿搜索的那几十毫秒
// 会把 action 结果回调压在队列后面，导航结果的处理被拖慢。
#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 构造与 spin 都要包在 try 里。两类异常会从这里冒出来，若不接住，
  // 表现是 `terminate called after throwing` + 退出码 -6(SIGABRT) ——
  // 在 launch 日志里和"节点崩了"完全区分不开，而实际原因往往只是参数写错：
  //
  //   1) loadParameters() 对非法参数**刻意抛异常**拒绝启动（replan_policy 拼错、
  //      replan_check_period_sec<=0、max_invalid_replan_attempts=0 …）。
  //      需求要求"非法参数拒绝启动"，抛是对的，但必须给出可读的原因和干净的退出码。
  //   2) 启动瞬间收到 SIGINT 时 context 会在构造中途失效，create_publisher
  //      抛 "rcl node's context is invalid"。实测：launch 起来后立刻 Ctrl-C，
  //      协调器就以 -6 退出，日志里一片 `terminate called after throwing`。
  //      这不是缺陷，但不该长得像缺陷。
  try {
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<astribot_s1_autonomy::ExplorationCoordinatorNode>(
      rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    // 用 fprintf 而不是 RCLCPP_ERROR：异常可能来自节点构造过程中，
    // 那时 logger 未必可用，而这条信息恰恰是排查的全部线索，绝不能丢。
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
