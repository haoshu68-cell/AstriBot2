#include "astribot_s1_manipulation/owned_trajectory_controller.hpp"

#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/create_server.hpp>

namespace astribot_s1_manipulation {
int64_t OwnedTrajectoryController::steady_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool OwnedTrajectoryController::permitted() const {
  return steady_now() < deadline_.load();
}

controller_interface::CallbackReturn OwnedTrajectoryController::on_configure(
  const rclcpp_lifecycle::State& state) {
  const auto result = JointTrajectoryController::on_configure(state);
  if (result != controller_interface::CallbackReturn::SUCCESS) return result;
  expired_result_ = std::make_shared<FollowJTrajAction::Result>();
  expired_result_->error_code = FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED;
  expired_result_->error_string = "EXECUTION_OWNER_EXPIRED";
  // This task stack admits motion through FJT only. Remove the unused topic
  // command entry instead of retaining an unleased compatibility path.
  joint_command_subscriber_.reset();
  action_server_.reset();
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node(), "~/follow_joint_trajectory",
    [this](const auto& id, const auto goal) {
      return permitted() ? goal_received_callback(id, goal) : rclcpp_action::GoalResponse::REJECT;
    },
    [this](const auto goal) { return goal_cancelled_callback(goal); },
    [this](const auto goal) { goal_accepted_callback(goal); });
  acknowledgement_ = get_node()->create_publisher<Heartbeat>("~/execution_heartbeat_ack", 10);
  heartbeat_subscription_ = get_node()->create_subscription<Heartbeat>(
    "/transport/execution_heartbeat", rclcpp::QoS(10),
    [this](Heartbeat::ConstSharedPtr value) {
      heartbeat(*value);
      Heartbeat acknowledgement;
      acknowledgement.stamp = get_node()->now(); acknowledgement.lease_id = lease_id_;
      acknowledgement.sequence = sequence_; acknowledgement.active = permitted();
      acknowledgement_->publish(acknowledgement);
    });
  return result;
}

controller_interface::CallbackReturn OwnedTrajectoryController::on_activate(
  const rclcpp_lifecycle::State& state) {
  deadline_ = -1; lease_id_.clear(); sequence_ = 0; source_stamp_ = 0;
  return JointTrajectoryController::on_activate(state);
}

void OwnedTrajectoryController::heartbeat(const Heartbeat& value) {
  const auto received = steady_now();
  const auto ros = get_node()->now().nanoseconds();
  const auto stamp = int64_t(value.stamp.sec) * 1000000000 + value.stamp.nanosec;
  if (value.lease_id.empty() || value.sequence == 0 || value.stamp.nanosec >= 1000000000u ||
      stamp <= 0 || stamp > ros || ros - stamp >= lease_duration_ns) return;
  if (value.lease_id != lease_id_) {
    // Existing goals must terminate before a new task can establish authority.
    if (*rt_active_goal_.readFromNonRT() || rt_has_pending_goal_.load()) {
      deadline_ = 0; return;
    }
    lease_id_ = value.lease_id; sequence_ = 0; source_stamp_ = 0; deadline_ = -1;
  }
  if (!value.active) { deadline_ = 0; return; }
  auto previous = deadline_.load();
  if (previous == 0 || value.sequence <= sequence_ || stamp < source_stamp_) return;
  if (previous > 0 && received >= previous) {
    deadline_.compare_exchange_strong(previous, 0); return;
  }
  // The realtime expiry and callback renewal share one atomic transition;
  // a callback racing an expiry cannot revive that expired lease.
  if (deadline_.compare_exchange_strong(previous, received + lease_duration_ns - (ros - stamp))) {
    sequence_ = value.sequence; source_stamp_ = stamp;
  }
}

controller_interface::return_type OwnedTrajectoryController::update(
  const rclcpp::Time& time, const rclcpp::Duration& period) {
  auto deadline = deadline_.load();
  if (deadline > 0 && steady_now() >= deadline) deadline_.compare_exchange_strong(deadline, 0);
  if (!permitted()) {
    const auto goal = *rt_active_goal_.readFromRT();
    if (goal) {
      goal->setAborted(expired_result_);
      // Same realtime goal/hold handoff used by JTC's tolerance-stop path.
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      rt_has_pending_goal_ = false;
    }
    if (goal || !rt_is_holding_.load()) {
      read_state_from_state_interfaces(state_current_);
      traj_msg_external_point_ptr_.reset();
      traj_msg_external_point_ptr_.initRT(set_hold_position());
    }
  }
  return JointTrajectoryController::update(time, period);
}
}  // namespace astribot_s1_manipulation

PLUGINLIB_EXPORT_CLASS(astribot_s1_manipulation::OwnedTrajectoryController,
  controller_interface::ControllerInterface)
