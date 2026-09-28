#include "controller_fixture.hpp"

class ControllerStop : public ControllerFixture<joint_trajectory_controller::JointTrajectoryController> {};

TEST_F(ControllerStop, UpdateCycleBenchmark) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.5);
  EXPECT_GT(position, .2); EXPECT_LT(position, .3);
}

TEST_F(ControllerStop, AcceptedTrajectoryContinuesAfterOwnerDisappears) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.25);
  const double at_owner_loss = position;
  executor.remove_node(owner); client.reset(); owner.reset(); goal.reset();
  advance(.4);
  // A red prerequisite for runtime recovery: standard JTC does not enforce the
  // lifetime of the submitting task. PASS confirms the gap, not safe stopping.
  EXPECT_GT(position - at_owner_loss, .12);
  RecordProperty("position_at_owner_loss", std::to_string(at_owner_loss));
  RecordProperty("position_after_400ms", std::to_string(position));
}

TEST_F(ControllerStop, CancelStopsNewReferencesOnIdealPositionInterfaces) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.25);
  auto canceled = client->async_cancel_goal(goal);
  ASSERT_EQ(executor.spin_until_future_complete(canceled, std::chrono::seconds(3)), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(canceled.get()->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  advance(.1);
  const double after_cancel = position;
  advance(.3);
  EXPECT_NEAR(position, after_cancel, 1e-12);
  EXPECT_NEAR(velocity, 0, 1e-12);
  RecordProperty("held_position", std::to_string(position));
}
