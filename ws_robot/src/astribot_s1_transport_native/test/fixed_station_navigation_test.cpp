#include "astribot_s1_transport_native/fixed_station_navigation.hpp"
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gtest/gtest.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <filesystem>
#include <thread>

using astribot::transport::FixedStationNavigation;
namespace {
using Helper=FixedStationNavigation;
using Phase=Helper::Phase;
using Nav=nav2_msgs::action::NavigateToPose;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Ack=astribot_navigation_msgs::msg::EnvelopeApplyStatus;
using ServerHandle=rclcpp_action::ServerGoalHandle<Nav>;
using Clock=std::chrono::steady_clock;
class FixedStationNavigationTest:public testing::Test {
protected:
  rclcpp::Node::SharedPtr owner,fake;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::unique_ptr<Helper> helper;
  rclcpp::Service<Helper::Fixed>::SharedPtr fixed;
  rclcpp_action::Server<Nav>::SharedPtr nav;
  rclcpp::Publisher<Envelope>::SharedPtr envelope_pub;
  rclcpp::Publisher<Ack>::SharedPtr ack_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub;
  std::shared_ptr<ServerHandle> goal;
  Envelope envelope;
  bool fixed_received=false,omit_controller=false,legacy_protection=false,finish_cancel=true;
  bool publish_command=true,publish_slam_pose=true;
  bool controller_ack_applied=true,controller_ack_zero_stamp=false;
  double controller_ack_ahead_s=0.;
  double envelope_ahead_s=0.,slam_pose_ahead_s=0.;
  bool envelope_allowed=true,envelope_zero_stamp=false;
  int goals=0,cancels=0,commands=0;
  double x=0,speed=0;
  void SetUp()override {
    // Run only in an operator-assigned isolated domain, never the ambient stack.
    const auto expected=std::getenv("ASTRIBOT_FIXED_STATION_TEST_DOMAIN");
    const auto actual=std::getenv("ROS_DOMAIN_ID");
    ASSERT_NE(expected,nullptr);ASSERT_NE(actual,nullptr);ASSERT_STREQ(expected,actual);
    rclcpp::init(0,nullptr);
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    owner=std::make_shared<rclcpp::Node>("fixed_station_navigation_test_owner");
    owner->declare_parameter<std::string>("workstation_navigation_bt","/fixture/navigate_to_workstation.xml");
    fake=std::make_shared<rclcpp::Node>("fixed_station_navigation_test_endpoints");
    envelope_pub=fake->create_publisher<Envelope>("/navigation/envelope_v2",10);
    ack_pub=fake->create_publisher<Ack>("/navigation/envelope_applied",20);
    slam_pose_pub=fake->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS());
    command_pub=fake->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::SensorDataQoS());
    fixed=fake->create_service<Helper::Fixed>("/navigation/set_fixed_envelope",[this](const Helper::Fixed::Request::SharedPtr request,Helper::Fixed::Response::SharedPtr reply){
      fixed_received=true;reply->accepted=true;reply->epoch=7;
      envelope.coordinator_session_id="coordinator";envelope.epoch=7;envelope.request_id=request->request_id;
      envelope.hold_id=request->hold_id;envelope.reference_state_sequence=request->geometry_sequence;
      envelope.mode=Envelope::FIXED_POSTURE;envelope.navigation_allowed=true;
      envelope.model_revision="robot";envelope.attachment_revision="loaded";envelope.installed_geometry_hash="shape";
      envelope.limits=request->limits;envelope.limits.transport_ready=true;
    });
    nav=rclcpp_action::create_server<Nav>(fake,"/navigate_to_pose",
      [](const auto&,const auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto handle){++cancels;EXPECT_EQ(handle,goal);return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto handle){goal=handle;++goals;});
    helper=std::make_unique<Helper>(*owner,"/navigate_to_pose");
    executor->add_node(owner);executor->add_node(fake);
    auto client=rclcpp_action::create_client<Nav>(fake,"/navigate_to_pose");
    ASSERT_TRUE(wait([&]{return client->action_server_is_ready()&&envelope_pub->get_subscription_count()==1&&
      ack_pub->get_subscription_count()==1&&slam_pose_pub->get_subscription_count()==1&&command_pub->get_subscription_count()==1;},3.));
  }
  void TearDown()override {
    if(executor)executor->cancel();
    helper.reset();nav.reset();fixed.reset();goal.reset();
    if(owner)executor->remove_node(owner);
    if(fake)executor->remove_node(fake);
    command_pub.reset();slam_pose_pub.reset();ack_pub.reset();envelope_pub.reset();owner.reset();fake.reset();
    executor.reset();if(rclcpp::ok())rclcpp::shutdown();
  }
  Helper::Request request() {
    Helper::Request r;r.task_id="task";r.context_id="task_nav";r.lease_id="lease";r.resource_epoch="epoch";
    r.fixed.request_id="fixed";r.fixed.hold_id="hold";r.fixed.geometry_sequence=10;
    r.fixed.limits.frame_id="base";r.fixed.limits.payload_mass_kg=.2;
    auto& g=r.reference_geometry;g.complete=g.attachment_state_confirmed=true;g.source_id="geometry";
    g.sequence=10;g.model_revision="robot";g.attachment_revision="loaded";g.attachment_ids={"box"};g.header.frame_id="base";
    r.navigation_target.header.frame_id="map";r.navigation_target.pose.position.x=1.;r.navigation_target.pose.orientation.w=1.;
    r.robot_base_frame="base";r.position_tolerance_m=.002;r.yaw_tolerance_rad=std::acos(-1.)/1800.;return r;
  }
  void step() {
    const auto at=fake->now();
    if(fixed_received) {
      envelope.header.stamp=at+rclcpp::Duration::from_seconds(envelope_ahead_s);
      envelope.header.frame_id="base";envelope.valid_until=rclcpp::Time(envelope.header.stamp)+rclcpp::Duration::from_seconds(.3);
      envelope.navigation_allowed=envelope.limits.transport_ready=envelope_allowed;
      if(envelope_zero_stamp)envelope.header.stamp={};
      envelope_pub->publish(envelope);
      for(const auto* consumer:{"global_costmap","local_costmap","planner","controller","policy","protection"}) {
        if(std::string(consumer)=="protection"&&!legacy_protection)continue;
        if(omit_controller&&std::string(consumer)=="controller")continue;
        Ack a;a.header.stamp=at;a.coordinator_session_id="coordinator";a.envelope_epoch=7;
        a.installed_geometry_hash="shape";a.consumer_id=consumer;a.applied=true;
        if(a.consumer_id=="controller") {
          a.header.stamp=at+rclcpp::Duration::from_seconds(controller_ack_ahead_s);
          if(controller_ack_zero_stamp)a.header.stamp={};
          a.applied=controller_ack_applied;
        }
        ack_pub->publish(a);
      }
    }
    if(publish_command){geometry_msgs::msg::Twist command;command.linear.x=speed;command_pub->publish(command);++commands;}
    geometry_msgs::msg::PoseWithCovarianceStamped o;o.header.stamp=at+rclcpp::Duration::from_seconds(slam_pose_ahead_s);o.header.frame_id="map";
    o.pose.pose.position.x=x;o.pose.pose.orientation.w=1.;if(publish_slam_pose)slam_pose_pub->publish(o);
    // Drain the five ACKs sharing one subscription within a bounded cycle.
    executor->spin_all(std::chrono::milliseconds(5));helper->tick();
    if(goal&&goal->is_canceling()&&finish_cancel)goal->canceled(std::make_shared<Nav::Result>());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  bool wait(const std::function<bool()>& done,double seconds) {
    const auto deadline=Clock::now()+std::chrono::duration<double>(seconds);
    while(Clock::now()<deadline){step();if(done())return true;}return false;
  }
};
TEST_F(FixedStationNavigationTest, SameLeaseLegUsesSlamMapPoseAndPostTerminalStop) {
  publish_command=false;commands=0;
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;ASSERT_EQ(goals,1);
  EXPECT_EQ(goal->get_goal()->behavior_tree,"/fixture/navigate_to_workstation.xml");
  EXPECT_EQ(commands,0);
  EXPECT_EQ(goal->get_goal()->pose.header.frame_id,"map");EXPECT_DOUBLE_EQ(goal->get_goal()->pose.pose.position.x,1.);
  publish_command=true;speed=.04;step();
  x=1.;speed=0;step();publish_command=false;
  goal->succeed(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SETTLING;},1.));
  EXPECT_FALSE(helper->status().measured_stopped);EXPECT_FALSE(helper->status().cleanup_complete);
  const auto settling=Clock::now();
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SUCCEEDED;},2.))<<helper->status().reason;
  EXPECT_GE(std::chrono::duration<double>(Clock::now()-settling).count(),.6);
  EXPECT_TRUE(helper->status().nav_terminal);EXPECT_TRUE(helper->status().cleanup_complete);
  EXPECT_EQ(helper->status().nav_goal_uuid.size(),32u);EXPECT_EQ(helper->status().envelope_epoch,7u);
  EXPECT_LT(helper->status().position_error_m,1e-10);EXPECT_EQ(cancels,0);
}
TEST_F(FixedStationNavigationTest, LegacyProtectionCannotReplaceControllerAndCancellationStillRequiresStop) {
  publish_command=false;omit_controller=true;legacy_protection=true;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_EQ(helper->status().phase,Phase::WAIT_ACK);
  EXPECT_TRUE(helper->status().envelope_session.empty());EXPECT_TRUE(helper->status().geometry_hash.empty());
  helper->cancel("TASK_CANCELED");EXPECT_FALSE(helper->status().cleanup_complete);
  publish_command=true;
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(helper->status().phase,Phase::FAILED);EXPECT_TRUE(helper->status().nav_terminal);EXPECT_EQ(cancels,1);
}
TEST_F(FixedStationNavigationTest, CancelTargetsOnlyOwnedGoalAndTerminalDoesNotProveStopped) {
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  speed=.04;step();helper->cancel("TASK_CANCELED");
  ASSERT_TRUE(wait([&]{return helper->status().nav_terminal;},1.));
  EXPECT_FALSE(wait([&]{return helper->status().cleanup_complete;},.8));
  EXPECT_EQ(cancels,1);EXPECT_FALSE(helper->status().measured_stopped);
  const auto stopped_since=Clock::now();speed=0;step();publish_command=false;
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_GE(std::chrono::duration<double>(Clock::now()-stopped_since).count(),.6);
  EXPECT_EQ(helper->status().phase,Phase::FAILED);EXPECT_EQ(helper->status().reason,"TASK_CANCELED");
}
TEST_F(FixedStationNavigationTest, NonzeroSilenceCannotBeMistakenForFinalZero) {
  publish_command=false;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  publish_command=true;speed=.04;step();publish_command=false;speed=0;
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::CANCELING;},1.));
  EXPECT_EQ(helper->status().reason,"NAVIGATION_COMMAND_STALE");
  EXPECT_FALSE(wait([&]{return helper->status().cleanup_complete;},.8));
  EXPECT_TRUE(helper->status().nav_terminal);EXPECT_EQ(cancels,1);
  // A real late zero permits cleanup; silence and fresh stationary odometry did not.
  publish_command=true;step();publish_command=false;
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(helper->status().phase,Phase::FAILED);
  EXPECT_EQ(helper->status().reason,"NAVIGATION_COMMAND_STALE");
}
TEST_F(FixedStationNavigationTest, SuccessfulTerminalWithoutObservedZeroDoesNotProveStop) {
  publish_command=false;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  x=1.;goal->succeed(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SETTLING;},1.));
  EXPECT_FALSE(wait([&]{return helper->status().cleanup_complete;},.8));
  EXPECT_TRUE(helper->status().nav_terminal);EXPECT_FALSE(helper->status().measured_stopped);
  EXPECT_EQ(helper->status().phase,Phase::SETTLING);
  publish_command=true;step();publish_command=false;
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SUCCEEDED;},2.))<<helper->status().reason;
}
TEST_F(FixedStationNavigationTest, AbortedBeforeAnyCommandUsesPostTerminalSlamStop) {
  publish_command=false;commands=0;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  goal->abort(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::CANCELING;},1.));
  EXPECT_TRUE(helper->status().nav_terminal);EXPECT_EQ(commands,0);
  publish_slam_pose=false;
  EXPECT_FALSE(wait([&]{return helper->status().cleanup_complete;},.8));
  publish_slam_pose=true;
  const auto observed_since=Clock::now();
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_GE(std::chrono::duration<double>(Clock::now()-observed_since).count(),.6);
  EXPECT_EQ(helper->status().phase,Phase::FAILED);
  EXPECT_EQ(helper->status().reason,"NAVIGATION_NOT_SUCCEEDED");
  EXPECT_EQ(commands,0);EXPECT_EQ(cancels,0);
}
TEST_F(FixedStationNavigationTest, FinalZeroCannotReplaceMeasuredStopWindow) {
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.));
  x=1.;step();publish_command=false;goal->succeed(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SETTLING;},1.));publish_slam_pose=false;
  EXPECT_FALSE(wait([&]{return helper->status().cleanup_complete;},.8));
  EXPECT_EQ(helper->status().phase,Phase::SETTLING);
  publish_slam_pose=true;
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SUCCEEDED;},2.));
}
TEST_F(FixedStationNavigationTest, LeadingPositiveAckIsCurrentEvidence) {
  controller_ack_ahead_s=1.;helper->start(request());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING&&bool(goal);},3.))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_EQ(cancels,0);EXPECT_EQ(helper->status().envelope_session,"coordinator");
  helper->cancel("TASK_CANCELED");ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
}
TEST_F(FixedStationNavigationTest, FutureNegativeAckCannotAdmitNavigation) {
  controller_ack_ahead_s=1.;controller_ack_applied=false;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::WAIT_ACK;},.3))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_TRUE(helper->status().envelope_session.empty());
  controller_ack_ahead_s=0.;controller_ack_applied=true;
  // Older positives cannot erase the retained future revocation watermark.
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::WAIT_ACK;},.2))<<helper->status().reason;
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING;},2.))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_EQ(helper->status().envelope_session,"coordinator");
}
TEST_F(FixedStationNavigationTest, BoundAckRevocationBelongsToNav2AndUserCancelStillStops) {
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  controller_ack_ahead_s=1.;controller_ack_applied=false;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.6));
  EXPECT_EQ(cancels,0);helper->cancel("TASK_CANCELED");
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.))<<helper->status().reason;
  EXPECT_EQ(cancels,1);EXPECT_EQ(helper->status().phase,Phase::FAILED);
}
TEST_F(FixedStationNavigationTest, ZeroAckStampRemainsInvalid) {
  controller_ack_zero_stamp=true;helper->start(request());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::CANCELING;},1.));
  EXPECT_EQ(helper->status().reason,"NAVIGATION_ACK_INVALID");EXPECT_EQ(goals,0);
}
TEST_F(FixedStationNavigationTest, BoundAckExpiryDoesNotDuplicateNav2Admission) {
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  controller_ack_ahead_s=2.;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.15))
    <<helper->status().reason;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.8));
  EXPECT_EQ(cancels,0);helper->cancel("TASK_CANCELED");
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_EQ(cancels,1);
}
TEST_F(FixedStationNavigationTest, IntentPrecedesPermissionAndBindingDoesNotResendGoal) {
  omit_controller=true;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  const auto intent_id=goal->get_goal_id();
  EXPECT_EQ(helper->status().phase,Phase::WAIT_ACK);
  EXPECT_TRUE(helper->status().envelope_session.empty());EXPECT_TRUE(helper->status().geometry_hash.empty());
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::WAIT_ACK;},.3));
  omit_controller=false;
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING;},2.))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_EQ(goal->get_goal_id(),intent_id);
  EXPECT_EQ(helper->status().envelope_session,"coordinator");EXPECT_EQ(helper->status().geometry_hash,"shape");
  controller_ack_applied=false;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.6));
  helper->cancel("TASK_CANCELED");
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(cancels,1);EXPECT_EQ(goals,1);
}
TEST_F(FixedStationNavigationTest, SuccessBeforePermissionCannotCompleteNavigation) {
  omit_controller=true;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  ASSERT_EQ(helper->status().phase,Phase::WAIT_ACK);
  x=1.;goal->succeed(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::CANCELING;},1.));
  EXPECT_EQ(helper->status().reason,"NAVIGATION_SUCCEEDED_WITHOUT_PERMISSION");
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(helper->status().phase,Phase::FAILED);EXPECT_EQ(cancels,0);
}
TEST_F(FixedStationNavigationTest, LeadingEnvelopeAndSlamPoseCompleteWithoutTF) {
  envelope_ahead_s=5.;slam_pose_ahead_s=10.;helper->start(request());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING&&bool(goal);},3.))<<helper->status().reason;
  x=1.;goal->succeed(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SUCCEEDED;},2.))<<helper->status().reason;
  EXPECT_EQ(goals,1);EXPECT_EQ(cancels,0);EXPECT_LE(helper->status().position_error_m,.002);
}
TEST_F(FixedStationNavigationTest, BoundEnvelopeExpiryDoesNotDuplicateNav2Admission) {
  helper->start(request());ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING&&bool(goal);},3.));
  envelope_ahead_s=2.;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.1))<<helper->status().reason;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.5));
  EXPECT_EQ(cancels,0);goal->abort(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(helper->status().reason,"NAVIGATION_NOT_SUCCEEDED");EXPECT_EQ(cancels,0);
}
TEST_F(FixedStationNavigationTest, FutureNegativeEnvelopeCannotAdmitThenBoundMonitoringBelongsToNav2) {
  envelope_allowed=false;envelope_ahead_s=1.;helper->start(request());
  ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  EXPECT_EQ(helper->status().phase,Phase::WAIT_ACK);
  envelope_allowed=true;envelope_ahead_s=0.;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::WAIT_ACK;},.2));
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING;},2.))<<helper->status().reason;
  envelope_allowed=false;envelope_ahead_s=1.;
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.4));
  helper->cancel("TASK_CANCELED");
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));EXPECT_EQ(cancels,1);
}
TEST_F(FixedStationNavigationTest, BoundEnvelopeUpdatesDoNotCancelOwnedGoal) {
  helper->start(request());ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::NAVIGATING&&bool(goal);},3.));
  envelope_ahead_s=1.;envelope.installed_geometry_hash="changed";
  ASSERT_FALSE(wait([&]{return helper->status().phase!=Phase::NAVIGATING;},.4));
  helper->cancel("TASK_CANCELED");
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));EXPECT_EQ(cancels,1);
}
TEST_F(FixedStationNavigationTest, ZeroEnvelopeStampRemainsInvalid) {
  envelope_zero_stamp=true;helper->start(request());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::CANCELING;},1.));
  EXPECT_EQ(helper->status().reason,"NAVIGATION_ENVELOPE_INVALID");EXPECT_EQ(goals,0);
}
TEST_F(FixedStationNavigationTest, DefaultSelectsInstalledWorkstationTree) {
  auto default_owner=std::make_shared<rclcpp::Node>("workstation_default_configuration");
  Helper default_helper(*default_owner,"/navigate_to_pose");
  const std::filesystem::path path=default_owner->get_parameter("workstation_navigation_bt").as_string();
  EXPECT_EQ(path.filename(),"navigate_to_workstation.xml");
  EXPECT_TRUE(std::filesystem::is_regular_file(path))<<path;
}
} // namespace
