#include <moveit/move_group/move_group_capability.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <future>
#include <thread>

namespace astribot_s1_manipulation {
// Keep MoveIt's trajectory execution manager as the sole controller owner.
// Waiting for execution must not occupy the action server callback group.
class CancellableExecution : public move_group::MoveGroupCapability {
  using Action = moveit_msgs::action::ExecuteTrajectory;
  using Goal = rclcpp_action::ServerGoalHandle<Action>;
  std::atomic<bool> busy_{false}, cancel_{false}, shutdown_{false}, faulted_{false};
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::thread worker_;

  void execute(const std::shared_ptr<Goal>& goal) {
    auto result = std::make_shared<Action::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    try {
      auto manager = context_->trajectory_execution_manager_;
      if (manager && !cancel_ && !shutdown_ && manager->push(goal->get_goal()->trajectory)) {
        RCLCPP_INFO(context_->moveit_cpp_->getNode()->get_logger(), "OWNED_EXECUTION_STARTED");
        manager->execute();
        auto done = std::async(std::launch::async, [manager] { return manager->waitForExecution(); });
        bool stop_sent = false;
        while (done.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
          if ((cancel_ || shutdown_) && !stop_sent) {
            RCLCPP_INFO(context_->moveit_cpp_->getNode()->get_logger(), "OWNED_EXECUTION_STOP_REQUESTED");
            manager->stopExecution(true);
            stop_sent = true;
          }
        }
        const auto status = done.get();
        if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        else if (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      }
      // The cancel callback sets intent before rclcpp changes the goal state.
      while (cancel_ && !goal->is_canceling() && !shutdown_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (goal->is_canceling()) {
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
        goal->canceled(result);
      } else if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        goal->succeed(result);
      } else {
        goal->abort(result);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(context_->moveit_cpp_->getNode()->get_logger(), "EXECUTION_TRANSACTION_FAILED: %s", e.what());
      faulted_ = true;  // An uncertain execution requires process-level reconciliation.
      try {
        if (context_->trajectory_execution_manager_) context_->trajectory_execution_manager_->stopExecution(true);
        if (goal->is_active()) goal->abort(result);
      } catch (const std::exception& stopping) {
        RCLCPP_ERROR(context_->moveit_cpp_->getNode()->get_logger(), "EXECUTION_STOP_UNCONFIRMED: %s", stopping.what());
      }
    }
    busy_ = false;
  }
public:
  CancellableExecution() : MoveGroupCapability("AstribotCancellableExecution") {}
  ~CancellableExecution() override {
    shutdown_ = true;
    if (worker_.joinable()) worker_.join();
  }
  void initialize() override {
    server_ = rclcpp_action::create_server<Action>(context_->moveit_cpp_->getNode(), "execute_trajectory",
      [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Action::Goal>&) {
        bool idle = false;
        if (shutdown_ || faulted_ || !busy_.compare_exchange_strong(idle, true))
          return rclcpp_action::GoalResponse::REJECT;
        cancel_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<Goal>&) {
        RCLCPP_INFO(context_->moveit_cpp_->getNode()->get_logger(), "OWNED_EXECUTION_CANCEL_ACCEPTED");
        cancel_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<Goal>& goal) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, goal] { execute(goal); });
      });
  }
};
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_manipulation::CancellableExecution, move_group::MoveGroupCapability)
