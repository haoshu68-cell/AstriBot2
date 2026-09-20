#include "astribot_s1_autonomy/exploration_coordinator_node.hpp"
#include <gtest/gtest.h>
#include <thread>
TEST(ExplorationReadiness, RequiresUniqueFreshConfirmedEnvelope) {
  using namespace astribot_s1_autonomy;
  astribot_navigation_msgs::msg::NavigationEnvelopeV2 e;
  e.header.stamp = rclcpp::Time(1000000000);
  e.valid_until = rclcpp::Time(1300000000);
  e.mode=e.FIXED_POSTURE; e.navigation_allowed=true; e.limits.transport_ready=true;
  e.coordinator_session_id="session";e.hold_id="hold";e.epoch=1;
  EXPECT_TRUE(fixedEnvelopeReadiness(&e,1100000000,1).empty());
  EXPECT_FALSE(fixedEnvelopeReadiness(nullptr,1100000000,1).empty());
  EXPECT_FALSE(fixedEnvelopeReadiness(&e,1100000000,0).empty());
  EXPECT_FALSE(fixedEnvelopeReadiness(&e,1100000000,2).empty());
  EXPECT_FALSE(fixedEnvelopeReadiness(&e,999999999,1).empty());
  EXPECT_FALSE(fixedEnvelopeReadiness(&e,1300000000,1).empty());
  e.navigation_allowed=false;
  EXPECT_FALSE(fixedEnvelopeReadiness(&e,1100000000,1).empty());
  e.navigation_allowed=true;e.hold_id.clear();
  EXPECT_FALSE(fixedEnvelopeReadiness(&e,1100000000,1).empty());
  e.hold_id="hold";e.header.stamp.sec=-1;
  EXPECT_EQ(fixedEnvelopeReadiness(&e,1100000000,1),"ENVELOPE_V2_INVALID_TIME");
}
TEST(ExplorationReadiness, MissingEnvelopeBlocksSelectionAndReportsCause) {
  rclcpp::init(0,nullptr);
  auto coordinator=std::make_shared<astribot_s1_autonomy::ExplorationCoordinatorNode>(
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("require_fixed_envelope",true),
      rclcpp::Parameter("control_period_sec",0.05)}));
  auto driver=std::make_shared<rclcpp::Node>("envelope_readiness_test");
  nlohmann::json status;
  auto sub=driver->create_subscription<std_msgs::msg::String>(
    "/exploration_coordinator_node/operator_status",rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr m){status=nlohmann::json::parse(m->data);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator);executor.add_node(driver);
  const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(400);
  while(std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  ASSERT_FALSE(status.empty());
  EXPECT_FALSE(status.at("ready").get<bool>());
  EXPECT_FALSE(status.at("goal_in_flight").get<bool>());
  EXPECT_EQ(status.at("readiness_detail"),"ENVELOPE_V2_SOURCE_NOT_UNIQUE");
  EXPECT_FALSE(coordinator->set_parameter(rclcpp::Parameter("require_fixed_envelope",false)).successful);
  executor.remove_node(coordinator);executor.remove_node(driver);
  coordinator.reset();driver.reset();rclcpp::shutdown();
}
TEST(ExplorationEnd, PauseDoesNotSaveCancelSavesOnceAndCannotResume) {
  rclcpp::init(0, nullptr);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto driver = std::make_shared<rclcpp::Node>("exploration_end_test");
  int saves = 0;
  auto saver = driver->create_service<std_srvs::srv::Trigger>("/mapping_session/finalize_canceled",
    [&](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
      ++saves; r->success = true;
    });
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("control_period_sec", 0.05),
    rclcpp::Parameter("nav_action_name", "/test_only/nav"),
    rclcpp::Parameter("plan_action_name", "/test_only/plan")});
  auto coordinator = std::make_shared<astribot_s1_autonomy::ExplorationCoordinatorNode>(options);
  executor.add_node(driver); executor.add_node(coordinator);
  auto spin = [&](int ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  };
  auto call = [&](const std::string & name) {
    auto client = driver->create_client<std_srvs::srv::Trigger>("/exploration_coordinator_node/" + name);
    if (!client->wait_for_service(std::chrono::seconds(2))) throw std::runtime_error("missing service");
    auto result = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    for(int i = 0; i < 200 && result.wait_for(std::chrono::seconds(0)) != std::future_status::ready; ++i) spin(10);
    if (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready) throw std::runtime_error("service timeout");
    return result.get()->success;
  };
  using Command=astribot_operator_msgs::srv::ExplorationCommand;
  nlohmann::json status;
  auto status_sub=driver->create_subscription<std_msgs::msg::String>("/exploration_coordinator_node/operator_status",rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr m){status=nlohmann::json::parse(m->data);});
  auto command=driver->create_client<Command>("/exploration_coordinator_node/command");
  auto submit=[&](Command::Request::SharedPtr request) {
    if(!command->wait_for_service(std::chrono::seconds(2)))throw std::runtime_error("missing command service");
    auto future=command->async_send_request(request);
    for(int i=0;i<200&&future.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)spin(10);
    if(future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)throw std::runtime_error("command timeout");
    return *future.get();
  };
  spin(200);ASSERT_FALSE(status.empty());EXPECT_EQ(status.at("reason_code"),"EXPLORATION.NOT_READY");
  auto request=std::make_shared<Command::Request>();request->command_id="pause-1";request->operation="pause";
  request->expected_boot_id="wrong-boot";request->expected_revision=status.at("revision");
  EXPECT_EQ(submit(request).reason_code,"REQUEST.BOOT_MISMATCH");
  request->expected_boot_id=status.at("boot_id");
  auto accepted=submit(request);EXPECT_TRUE(accepted.accepted);
  EXPECT_TRUE(submit(request).accepted); // Lost response retry does not perform another operation.
  request->operation="cancel_save";EXPECT_EQ(submit(request).reason_code,"REQUEST.CONFLICT");
  request->command_id="stale";EXPECT_EQ(submit(request).reason_code,"STATE.REVISION_MISMATCH");
  spin(100);request->command_id="resume-not-ready";request->expected_revision=status.at("revision");request->operation="resume";
  EXPECT_EQ(submit(request).reason_code,"EXPLORATION.OPERATION_BLOCKED");EXPECT_EQ(saves,0);
  spin(200); EXPECT_TRUE(call("pause")); spin(200); EXPECT_EQ(saves, 0);
  EXPECT_TRUE(call("resume")); EXPECT_TRUE(call("cancel")); spin(300);
  EXPECT_EQ(saves, 1); EXPECT_FALSE(call("resume")); EXPECT_FALSE(call("pause"));
  EXPECT_TRUE(call("cancel")); spin(200); EXPECT_EQ(saves, 1);
  executor.remove_node(coordinator); executor.remove_node(driver); coordinator.reset(); driver.reset();
  rclcpp::shutdown();
}
