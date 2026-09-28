#pragma once
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <thread>

using Follow = control_msgs::action::FollowJointTrajectory;
using Handle = rclcpp_action::ClientGoalHandle<Follow>;

// Real JTC action/lifecycle/update implementation; ideal position interfaces.
// This measures command semantics, not physical stopping distance.
template<class Controller>
class ControllerFixture : public ::testing::Test {
protected:
  Controller controller;
  rclcpp::Node::SharedPtr owner;
  rclcpp_action::Client<Follow>::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  double position = 0, velocity = 0, command = 0;
  std::vector<double> update_ns, dispatch_and_update_ns;
  hardware_interface::CommandInterface command_interface{"joint", "position", &command};
  hardware_interface::StateInterface position_interface{"joint", "position", &position};
  hardware_interface::StateInterface velocity_interface{"joint", "velocity", &velocity};

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
      rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"})});
    ASSERT_EQ(controller.init("stop_probe", "", options), controller_interface::return_type::OK);
    executor.add_node(controller.get_node()->get_node_base_interface());
    ASSERT_EQ(controller.get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(command_interface);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(position_interface); states.emplace_back(velocity_interface);
    controller.assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(controller.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    owner = std::make_shared<rclcpp::Node>("stop_probe_owner");
    client = rclcpp_action::create_client<Follow>(owner, "/stop_probe/follow_joint_trajectory");
    executor.add_node(owner);
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  }

  void TearDown() override {
    if (!update_ns.empty()) {
      for (auto entry : {std::make_pair("update_ns", &update_ns),
                         std::make_pair("dispatch_and_update_ns", &dispatch_and_update_ns)}) {
        auto& values = *entry.second;
        std::sort(values.begin(), values.end());
        double sum = 0; for (auto value : values) sum += value;
        RecordProperty(std::string(entry.first) + "_mean", std::to_string(sum / values.size()));
        RecordProperty(std::string(entry.first) + "_p99", std::to_string(values[std::min(values.size()-1, size_t(std::ceil(values.size()*.99)-1))]));
        RecordProperty(std::string(entry.first) + "_max", std::to_string(values.back()));
      }
      RecordProperty("update_samples", int(update_ns.size()));
    }
    if (controller.get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      controller.get_node()->deactivate();
    controller.release_interfaces();
    executor.remove_node(controller.get_node()->get_node_base_interface());
    if (owner) executor.remove_node(owner);
    client.reset(); owner.reset();
  }

  virtual void beforeUpdate() {}

  void advance(double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      const auto dispatch_start = std::chrono::steady_clock::now();
      beforeUpdate();
      executor.spin_some();
      const auto update_start = std::chrono::steady_clock::now();
      ASSERT_EQ(controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.01)),
        controller_interface::return_type::OK);
      const auto update_end = std::chrono::steady_clock::now();
      update_ns.push_back(std::chrono::duration<double, std::nano>(update_end-update_start).count());
      dispatch_and_update_ns.push_back(std::chrono::duration<double, std::nano>(update_end-dispatch_start).count());
      const auto previous = position;
      position = command; velocity = (position - previous) / .01;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  Handle::SharedPtr send() {
    Follow::Goal goal;
    goal.trajectory.joint_names = {"joint"};
    trajectory_msgs::msg::JointTrajectoryPoint start, end;
    start.positions = {0}; end.positions = {1}; end.time_from_start.sec = 2;
    goal.trajectory.points = {start, end};
    auto future = client->async_send_goal(goal);
    if (executor.spin_until_future_complete(future, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("JTC_GOAL_RESPONSE_TIMEOUT");
    return future.get();
  }
};
