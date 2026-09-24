#include "astribot_s1_transport_native/fixed_station_navigation.hpp"
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <gtest/gtest.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  std::shared_ptr<ServerHandle> goal;
  Envelope envelope;
  bool fixed_received=false,omit_controller=false,legacy_protection=false,finish_cancel=true;
  bool publish_command=true,publish_odom=true;
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
    fake=std::make_shared<rclcpp::Node>("fixed_station_navigation_test_endpoints");
    envelope_pub=fake->create_publisher<Envelope>("/navigation/envelope_v2",10);
    ack_pub=fake->create_publisher<Ack>("/navigation/envelope_applied",20);
    odom_pub=fake->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());
    command_pub=fake->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::SensorDataQoS());
    broadcaster=std::make_unique<tf2_ros::StaticTransformBroadcaster>(fake);
    transform(0.);
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
      ack_pub->get_subscription_count()==1&&odom_pub->get_subscription_count()==1&&command_pub->get_subscription_count()==1;},3.));
  }
  void TearDown()override {
    if(executor)executor->cancel();
    helper.reset();broadcaster.reset();nav.reset();fixed.reset();goal.reset();
    if(owner)executor->remove_node(owner);
    if(fake)executor->remove_node(fake);
    command_pub.reset();odom_pub.reset();ack_pub.reset();envelope_pub.reset();owner.reset();fake.reset();
    executor.reset();if(rclcpp::ok())rclcpp::shutdown();
  }
  void transform(double map_from_odom_x) {
    geometry_msgs::msg::TransformStamped t;t.header.stamp=fake->now();t.header.frame_id="map";t.child_frame_id="odom";
    t.transform.translation.x=map_from_odom_x;t.transform.rotation.w=1.;broadcaster->sendTransform(t);
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
      envelope.header.stamp=at;envelope.header.frame_id="base";envelope.valid_until=at+rclcpp::Duration::from_seconds(.3);
      envelope_pub->publish(envelope);
      for(const auto* consumer:{"global_costmap","local_costmap","planner","controller","policy","protection"}) {
        if(std::string(consumer)=="protection"&&!legacy_protection)continue;
        if(omit_controller&&std::string(consumer)=="controller")continue;
        Ack a;a.header.stamp=at;a.coordinator_session_id="coordinator";a.envelope_epoch=7;
        a.installed_geometry_hash="shape";a.consumer_id=consumer;a.applied=true;ack_pub->publish(a);
      }
    }
    if(publish_command){geometry_msgs::msg::Twist command;command.linear.x=speed;command_pub->publish(command);++commands;}
    nav_msgs::msg::Odometry o;o.header.stamp=at;o.header.frame_id="odom";o.child_frame_id="base";
    o.pose.pose.position.x=x;o.pose.pose.orientation.w=1.;o.twist.twist.linear.x=speed;if(publish_odom)odom_pub->publish(o);
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
TEST_F(FixedStationNavigationTest, SameLeaseLegUsesCurrentMapTransformAndPostTerminalStop) {
  publish_command=false;commands=0;
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;ASSERT_EQ(goals,1);
  EXPECT_EQ(commands,0);
  EXPECT_EQ(goal->get_goal()->pose.header.frame_id,"map");EXPECT_DOUBLE_EQ(goal->get_goal()->pose.pose.position.x,1.);
  publish_command=true;speed=.04;step();
  transform(.2);x=.8;speed=0;step();publish_command=false;
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
  EXPECT_FALSE(wait([&]{return goals!=0;},1.5));EXPECT_EQ(goals,0);
  helper->cancel("TASK_CANCELED");EXPECT_FALSE(helper->status().cleanup_complete);
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(helper->status().phase,Phase::FAILED);EXPECT_TRUE(helper->status().nav_terminal);EXPECT_EQ(cancels,0);
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
TEST_F(FixedStationNavigationTest, FinalZeroDoesNotReplaceFreshOdometry) {
  helper->start(request());ASSERT_TRUE(wait([&]{return bool(goal);},3.))<<helper->status().reason;
  x=1.;step();publish_command=false;goal->succeed(std::make_shared<Nav::Result>());
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::SETTLING;},1.));
  publish_odom=false;
  ASSERT_TRUE(wait([&]{return helper->status().phase==Phase::CANCELING;},1.));
  EXPECT_EQ(helper->status().reason,"NAVIGATION_MOTION_OBSERVATION_STALE");
  EXPECT_FALSE(wait([&]{return helper->status().cleanup_complete;},.8));
  publish_odom=true;
  ASSERT_TRUE(wait([&]{return helper->status().cleanup_complete;},2.));
  EXPECT_EQ(helper->status().phase,Phase::FAILED);
  EXPECT_EQ(helper->status().reason,"NAVIGATION_MOTION_OBSERVATION_STALE");
}
} // namespace
