#pragma once

#include <atomic>
#include <astribot_transport_msgs/msg/execution_heartbeat.hpp>
#include <astribot_transport_msgs/msg/controller_trajectory_start.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>

namespace astribot_s1_manipulation {

// JTC with one additional execution requirement: the task owner must remain
// alive. Expiry stops the accepted trajectory and latches the affected lease.
// Position hold uses JTC's existing stop semantics; it is not a brake model.
class OwnedTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
public:
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  using Heartbeat = astribot_transport_msgs::msg::ExecutionHeartbeat;
  static constexpr int64_t lease_duration_ns = 300000000;
  static int64_t steady_now();
  bool permitted() const;
  void heartbeat(const Heartbeat&);
  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Publisher<Heartbeat>::SharedPtr acknowledgement_;
  // -1: no heartbeat yet; 0: irrevocably expired/revoked for this lease.
  std::atomic<int64_t> deadline_{-1};
  std::atomic<bool> cycle_ready_{false};
  std::string lease_id_;
  uint64_t sequence_{0};
  int64_t source_stamp_{0};
  FollowJTrajAction::Result::SharedPtr expired_result_;
  using Start = astribot_transport_msgs::msg::ControllerTrajectoryStart;
  std::unique_ptr<realtime_tools::RealtimePublisher<Start>> start_publisher_;
  Start start_record_;
  trajectory_msgs::msg::JointTrajectoryPoint previous_command_;
  bool start_pending_ = false;
  // Action callbacks share JTC's mutually exclusive group; update is separate.
  // Odd means a callback is changing the goal/trajectory handoff.
  std::atomic_uint64_t goal_transition_{0};
};
}  // namespace astribot_s1_manipulation
