#include "controller_fixture.hpp"
#include <astribot_s1_manipulation/owned_trajectory_controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <map>
#include <set>
#include <tuple>

class InspectableOwnedController : public astribot_s1_manipulation::OwnedTrajectoryController {
public:
  auto sampled_command() const { return traj_external_point_ptr_->get_trajectory_msg(); }
};

class OwnedController : public ControllerFixture<InspectableOwnedController> {
protected:
  using Heartbeat = astribot_transport_msgs::msg::ExecutionHeartbeat;
  using Base = ControllerFixture<InspectableOwnedController>;
  rclcpp::Publisher<Heartbeat>::SharedPtr publisher;
  bool publish_heartbeat = true, lease_active = true;
  std::string lease = "owner_lease_1";
  uint64_t sequence = 0;
  using Start = astribot_transport_msgs::msg::ControllerTrajectoryStart;
  rclcpp::Subscription<Start>::SharedPtr starts_subscription;
  std::vector<Start> starts;

  void SetUp() override {
    Base::SetUp();
    publisher = owner->create_publisher<Heartbeat>("/transport/execution_heartbeat", 10);
    starts_subscription = owner->create_subscription<Start>("/stop_probe/trajectory_start",
      rclcpp::QoS(1).reliable().transient_local(), [this](Start::ConstSharedPtr value) { starts.push_back(*value); });
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

TEST_F(OwnedController, StartUsesFirstControlCycleAfterDelayedAcceptance) {
  Follow::Goal command;
  command.trajectory.joint_names = {"joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {.8}; point.velocities = {0}; point.time_from_start.sec = 2;
  command.trajectory.points = {point};
  auto sent = client->async_send_goal(command);
  ASSERT_EQ(executor.spin_until_future_complete(sent, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  auto goal = sent.get(); ASSERT_TRUE(goal);
  const auto accepted = owner->now();
  // Keep lease fresh while no control cycle occurs. Neither callback time nor
  // the old hardware sample is the interpolation's initial point.
  for(int i=0;i<8;++i) { beforeUpdate(); executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
  EXPECT_TRUE(starts.empty());
  position = .17; velocity = .23;
  const auto sampled = controller.get_node()->now();
  ASSERT_EQ(controller.update(sampled, rclcpp::Duration::from_seconds(.01)), controller_interface::return_type::OK);
  advance(.08);
  ASSERT_EQ(starts.size(), 1u);
  EXPECT_EQ(starts[0].binding, Start::BOUND);
  EXPECT_EQ(starts[0].goal_id, goal->get_goal_id());
  EXPECT_EQ(rclcpp::Time(starts[0].sample_stamp), sampled);
  EXPECT_GT(rclcpp::Time(starts[0].sample_stamp), accepted);
  EXPECT_EQ(starts[0].joint_names, command.trajectory.joint_names);
  EXPECT_DOUBLE_EQ(starts[0].point_before.positions[0], .17);
  EXPECT_DOUBLE_EQ(starts[0].point_before.velocities[0], .23);
  EXPECT_EQ(rclcpp::Duration(starts[0].point_before.time_from_start).nanoseconds(), 0);
}

TEST_F(OwnedController, CancelHoldDoesNotReplaceRetainedStartWithAnotherGoal) {
  auto goal = send(); ASSERT_TRUE(goal);
  advance(.1); ASSERT_EQ(starts.size(), 1u);
  const auto first = starts.front();
  auto cancelled = client->async_cancel_goal(goal);
  ASSERT_EQ(executor.spin_until_future_complete(cancelled, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  advance(.12);
  ASSERT_EQ(starts.size(), 1u);
  EXPECT_EQ(starts[0], first);
  starts_subscription.reset(); starts.clear();
  starts_subscription = owner->create_subscription<Start>("/stop_probe/trajectory_start",
    rclcpp::QoS(1).reliable().transient_local(), [this](Start::ConstSharedPtr value) { starts.push_back(*value); });
  advance(.1);
  ASSERT_EQ(starts.size(), 1u);
  EXPECT_EQ(starts[0], first);
}

TEST_F(OwnedController, PreemptingCommandGetsItsOwnStartAndUuid) {
  auto first = send(); ASSERT_TRUE(first); advance(.15);
  ASSERT_EQ(starts.size(), 1u);
  auto second = send(); ASSERT_TRUE(second); advance(.1);
  ASSERT_EQ(starts.size(), 2u);
  EXPECT_EQ(starts[0].goal_id, first->get_goal_id());
  EXPECT_EQ(starts[1].goal_id, second->get_goal_id());
  EXPECT_NE(starts[0].sample_stamp, starts[1].sample_stamp);
  EXPECT_GT(starts[1].point_before.positions[0], 0.);
}

TEST_F(OwnedController, FutureHeaderIsNotReportedAsFirstSamplingTime) {
  Follow::Goal command;
  command.trajectory.joint_names = {"joint"};
  command.trajectory.header.stamp = owner->now() + rclcpp::Duration::from_seconds(.5);
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {.8}; point.time_from_start.sec = 1;
  command.trajectory.points = {point};
  auto sent = client->async_send_goal(command);
  ASSERT_EQ(executor.spin_until_future_complete(sent, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(sent.get());
  advance(.1);
  ASSERT_EQ(starts.size(), 1u);
  EXPECT_EQ(starts[0].binding, Start::BOUND);
  EXPECT_LT(rclcpp::Time(starts[0].sample_stamp), rclcpp::Time(command.trajectory.header.stamp));
  EXPECT_DOUBLE_EQ(starts[0].point_before.positions[0], 0.);
}

class OpenLoopOwnedController : public OwnedController {
protected:
  void SetUp() override { open_loop_control=true; OwnedController::SetUp(); }
};

TEST_F(OpenLoopOwnedController, UsesPreviousCommandInsteadOfMeasuredState) {
  ASSERT_TRUE(send()); advance(.2);
  ASSERT_EQ(starts.size(),1u);
  const double commanded=command;
  ASSERT_GT(commanded,0.);
  Follow::Goal goal;
  goal.trajectory.joint_names={"joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions={.8};point.time_from_start.sec=2;goal.trajectory.points={point};
  auto sent=client->async_send_goal(goal);
  ASSERT_EQ(executor.spin_until_future_complete(sent,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(sent.get());
  position=commanded+.3;
  advance(.08);
  ASSERT_EQ(starts.size(),2u);
  EXPECT_EQ(starts.back().goal_id,sent.get()->get_goal_id());
  EXPECT_DOUBLE_EQ(starts.back().point_before.positions[0],commanded);
}

TEST_F(OwnedController, ConcurrentGoalHandoffsNeverInventAStartState) {
  std::thread callbacks([this]{executor.spin();});
  std::map<int64_t, std::tuple<double,double,double>> states;
  std::map<rclcpp_action::GoalUUID,double> accepted;
  for (int trial=0;trial<25;++trial) {
    Follow::Goal command;
    command.trajectory.joint_names = {"joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {double(trial+1)/100.}; point.time_from_start.sec = 2;
    command.trajectory.points = {point};
    auto sent = client->async_send_goal(command);
    for(int cycle=0;cycle<40;++cycle) {
      beforeUpdate();
      const auto sampled = controller.get_node()->now();
      const auto before=std::make_pair(position,velocity);
      controller.update(sampled, rclcpp::Duration::from_seconds(.001));
      states[sampled.nanoseconds()] = {before.first,before.second,
        controller.sampled_command()->points.back().positions[0]};
      const auto previous=position;position=this->command;velocity=(position-previous)/.001;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (sent.wait_for(std::chrono::seconds(1))==std::future_status::ready && sent.get())
      accepted[sent.get()->get_goal_id()] = point.positions[0];
  }
  executor.cancel();callbacks.join();
  ASSERT_EQ(accepted.size(), 25u);
  ASSERT_FALSE(starts.empty());
  int bound=0, unknown=0;
  for (const auto& start: starts) {
    if(start.binding==Start::UNKNOWN) {++unknown;continue;}
    ++bound;
    ASSERT_TRUE(accepted.count(start.goal_id));
    const auto found=states.find(rclcpp::Time(start.sample_stamp).nanoseconds());
    ASSERT_NE(found, states.end());
    EXPECT_DOUBLE_EQ(start.point_before.positions[0], std::get<0>(found->second));
    EXPECT_DOUBLE_EQ(start.point_before.velocities[0], std::get<1>(found->second));
    EXPECT_DOUBLE_EQ(accepted.at(start.goal_id), std::get<2>(found->second));
  }
  EXPECT_GT(bound,0);
  RecordProperty("bound_start_records",bound);RecordProperty("unknown_start_records",unknown);
}

TEST(OwnedControllerDiscovery, InstalledPluginCanBeLoaded) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface","controller_interface::ControllerInterface");
  auto plugin=loader.createSharedInstance("astribot_s1_manipulation/OwnedTrajectoryController");
  EXPECT_NE(dynamic_cast<astribot_s1_manipulation::OwnedTrajectoryController*>(plugin.get()),nullptr);
}

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

TEST_F(OwnedController, HeartbeatCallbacksRaceRealtimeExpiryWithoutRevival) {
  auto goal=send();ASSERT_TRUE(goal);
  // Controller-manager callbacks and the hardware update loop use different
  // threads. Keep all fake hardware writes on this thread; spin only ROS on
  // the peer, so this tests the real callback/update concurrency boundary.
  std::thread callbacks([this]{executor.spin();});
  bool updates_ok=true;double lost_at=0,held_at=0;
  for(int cycle=0;cycle<100;++cycle) {
    if(cycle==25){publish_heartbeat=false;lost_at=position;}
    if(cycle==75){held_at=position;publish_heartbeat=true;}
    beforeUpdate();
    updates_ok=updates_ok&&controller.update(controller.get_node()->now(),rclcpp::Duration::from_seconds(.01))==controller_interface::return_type::OK;
    const auto previous=position;position=command;velocity=(position-previous)/.01;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto result=client->async_get_result(goal);
  const bool ready=result.wait_for(std::chrono::seconds(2))==std::future_status::ready;
  executor.cancel();callbacks.join();
  ASSERT_TRUE(updates_ok);ASSERT_TRUE(ready);
  EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_LE(held_at-lost_at,.17);
  EXPECT_NEAR(position,held_at,1e-12);
}

TEST_F(OwnedController, FreshLeaseWaitsForControlCycleBeforeAdmittingGoal) {
  lease="next_owner_lease";beforeUpdate();
  // Deliver the new owner's callback while the hardware cycle is paused.
  // ACK/admission before a cycle has consumed the lease can race an older
  // no-authority hold update and abort/overwrite the newly accepted goal.
  for(int i=0;i<10;++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  auto premature=send();EXPECT_FALSE(premature);
  if(premature)return;
  advance(.1);EXPECT_TRUE(send());
}
