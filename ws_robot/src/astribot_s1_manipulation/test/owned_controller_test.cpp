#include "controller_fixture.hpp"
#include <astribot_s1_manipulation/owned_trajectory_controller.hpp>

class OwnedController : public ControllerFixture<astribot_s1_manipulation::OwnedTrajectoryController> {
protected:
  using Heartbeat = astribot_transport_msgs::msg::ExecutionHeartbeat;
  using Base = ControllerFixture<astribot_s1_manipulation::OwnedTrajectoryController>;
  rclcpp::Publisher<Heartbeat>::SharedPtr publisher;
  bool publish_heartbeat = true, lease_active = true;
  std::string lease = "owner_lease_1";
  uint64_t sequence = 0;

  void SetUp() override {
    Base::SetUp();
    publisher = owner->create_publisher<Heartbeat>("/transport/execution_heartbeat", 10);
    advance(.15);
  }

  void beforeUpdate() override {
    if (!publish_heartbeat) return;
    Heartbeat message;
    message.stamp = owner->now(); message.lease_id = lease;
    message.sequence = ++sequence; message.active = lease_active;
    publisher->publish(message);
  }
};

TEST_F(OwnedController, FreshOwnerKeepsOriginalTrajectory) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.6);
  EXPECT_GT(position, .25); EXPECT_LT(position, .4);
}

TEST_F(OwnedController, UpdateCycleBenchmark) {
  auto goal = send(); ASSERT_TRUE(goal);
  update_ns.clear(); dispatch_and_update_ns.clear();
  advance(.5);
  EXPECT_GT(position, .2); EXPECT_LT(position, .3);
}

TEST_F(OwnedController, LostOwnerAbortsAndHoldsInsteadOfCompletingOldTrajectory) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.25); publish_heartbeat = false;
  const double at_loss = position;
  advance(.5);
  const double stopped = position;
  EXPECT_LE(stopped - at_loss, .17);
  advance(.3); EXPECT_NEAR(position, stopped, 1e-12);
  auto result = client->async_get_result(goal);
  ASSERT_EQ(executor.spin_until_future_complete(result, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->error_string, "EXECUTION_OWNER_EXPIRED");
  RecordProperty("position_at_owner_loss", std::to_string(at_loss));
  RecordProperty("held_position", std::to_string(stopped));
}

TEST_F(OwnedController, ExpiredLeaseCannotBeRevivedByLateHeartbeat) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.2); publish_heartbeat = false; advance(.4);
  publish_heartbeat = true; advance(.15);
  EXPECT_FALSE(send());
}

TEST_F(OwnedController, ExplicitRevocationStopsBeforeLeaseTimeout) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.2); lease_active = false;
  const double at_revoke = position;
  advance(.1);
  EXPECT_LT(position - at_revoke, .03);
  EXPECT_FALSE(send());
}

TEST_F(OwnedController, DuplicateSequenceCannotExtendAuthority) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.2); publish_heartbeat = false;
  Heartbeat message; message.stamp = owner->now(); message.lease_id = lease;
  message.sequence = sequence; message.active = true;
  for (int i = 0; i < 6; ++i) {publisher->publish(message); advance(.1);}
  const auto stopped = position;
  advance(.2); EXPECT_NEAR(position, stopped, 1e-12);
  EXPECT_FALSE(send());
}

TEST_F(OwnedController, DirectTrajectoryTopicIsNotAnAlternateEntry) {
  auto direct = owner->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/stop_probe/joint_trajectory", 10);
  advance(.1);
  EXPECT_EQ(direct->get_subscription_count(), 0u);
}

TEST_F(OwnedController, NewLeaseCannotTakeOverAnExecutingGoal) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.2); lease = "different_owner";
  const auto before = position;
  advance(.1);
  EXPECT_LT(position - before, .03);
  auto result = client->async_get_result(goal);
  ASSERT_EQ(executor.spin_until_future_complete(result, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(OwnedController, StaleOrFutureSourceCannotExtendLease) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.2); publish_heartbeat = false;
  for (int i = 0; i < 6; ++i) {
    Heartbeat message;
    message.stamp = owner->now() + rclcpp::Duration::from_seconds(i % 2 ? 1. : -1.);
    message.lease_id = lease; message.sequence = ++sequence; message.active = true;
    publisher->publish(message); advance(.1);
  }
  const auto stopped = position;
  advance(.1); EXPECT_NEAR(position, stopped, 1e-12);
  EXPECT_FALSE(send());
}

TEST_F(OwnedController, NormalCompletionPreservesEndpoint) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(2.2);
  auto result = client->async_get_result(goal);
  ASSERT_EQ(executor.spin_until_future_complete(result, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(position, 1., 1e-12);
}
