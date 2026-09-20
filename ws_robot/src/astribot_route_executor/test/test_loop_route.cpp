#define ASTRIBOT_ROUTE_EXECUTOR_NO_MAIN
#include "../src/loop_route_executor.cpp"
#include <gtest/gtest.h>
using namespace astribot_route_executor;
class RouteTest : public ::testing::Test {
protected:
 std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
 std::shared_ptr<LoopRouteExecutor> runner;
 rclcpp::Node::SharedPtr fake;
 rclcpp_action::Server<Nav>::SharedPtr server;
 using GH=rclcpp_action::ServerGoalHandle<Nav>;
 std::shared_ptr<GH> goal;
 rclcpp::Client<Start>::SharedPtr start;
 rclcpp::Client<Cancel>::SharedPtr cancel;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
 Json status;
 std::vector<double> visited;
 int cancel_calls=0; bool finish_cancel=true; bool defer_accept=false, reject_goal=false;
 void SetUp() override {
  rclcpp::init(0,nullptr);executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  fake=std::make_shared<rclcpp::Node>("fake_route_navigation");
  server=rclcpp_action::create_server<Nav>(fake,"/test_route/nav",
   [&](const auto &,auto){return reject_goal?rclcpp_action::GoalResponse::REJECT:defer_accept?rclcpp_action::GoalResponse::ACCEPT_AND_DEFER:rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
   [&](auto){++cancel_calls;return rclcpp_action::CancelResponse::ACCEPT;},
   [&](auto h){goal=h;visited.push_back(h->get_goal()->pose.pose.position.x);});
  rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("navigation_action","/test_route/nav"),
   rclcpp::Parameter("goal_timeout_sec",1.2),rclcpp::Parameter("server_timeout_sec",.5),rclcpp::Parameter("cancel_timeout_sec",.15)});
  runner=std::make_shared<LoopRouteExecutor>(options);executor->add_node(fake);executor->add_node(runner);
  start=fake->create_client<Start>("/loop_route_executor/start");cancel=fake->create_client<Cancel>("/loop_route_executor/cancel");
  subscription=fake->create_subscription<std_msgs::msg::String>("/loop_route_executor/status",rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});
  spin(250);
 }
 void spin(int ms) {
  auto end=Clock::now()+std::chrono::milliseconds(ms);
  while(Clock::now()<end){executor->spin_some();
   if(finish_cancel && goal && goal->is_canceling()) {goal->canceled(std::make_shared<Nav::Result>());goal.reset();}
   std::this_thread::sleep_for(std::chrono::milliseconds(5));}
 }
 template<class Service> auto call(typename rclcpp::Client<Service>::SharedPtr client,typename Service::Request::SharedPtr request) {
  auto future=client->async_send_request(request);
  for(int i=0;i<200 && future.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)spin(5);
  if(future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)throw std::runtime_error("Test service timeout");
  return future.get();
 }
 Start::Request::SharedPtr route(const std::string & id="test") {
  auto req=std::make_shared<Start::Request>();req->request_id=id;req->expected_boot_id=status.at("boot_id");req->dwell_sec=.05;
  for(double x:{1.,2.}){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.orientation.w=1;req->waypoints.push_back(p);}return req;
 }
 auto stop(const std::string & id) {auto req=std::make_shared<Cancel::Request>();req->route_id=id;return call<Cancel>(cancel,req);}
 void TearDown() override {
  if(goal && goal->is_active()){if(goal->is_canceling())goal->canceled(std::make_shared<Nav::Result>());else goal->abort(std::make_shared<Nav::Result>());goal.reset();spin(50);}
  executor->remove_node(runner);executor->remove_node(fake);runner.reset();fake.reset();executor.reset();rclcpp::shutdown();
 }
};
TEST_F(RouteTest, SequentialWrapAndTargetedCancellation) {
 auto request=route();auto accepted=call<Start>(start,request);ASSERT_TRUE(accepted->accepted);spin(150);
 EXPECT_TRUE(call<Start>(start,request)->accepted);EXPECT_FALSE(call<Start>(start,route("other"))->accepted);
 for(int n=0;n<4;++n){ASSERT_TRUE(goal);goal->succeed(std::make_shared<Nav::Result>());goal.reset();spin(160);}
 ASSERT_EQ(visited.size(),5u);EXPECT_EQ(visited,(std::vector<double>{1,2,1,2,1}));
 EXPECT_EQ(status["completed_cycles"],2);
 EXPECT_FALSE(stop("stale-id")->accepted);EXPECT_EQ(cancel_calls,0);
 EXPECT_TRUE(stop(accepted->route_id)->accepted);spin(200);
 EXPECT_EQ(status["state"],"CANCELED");EXPECT_EQ(cancel_calls,1);
 EXPECT_TRUE(stop(accepted->route_id)->accepted);spin(100);EXPECT_EQ(visited.size(),5u);
 EXPECT_TRUE(call<Start>(start,request)->accepted);spin(100);EXPECT_EQ(visited.size(),5u);
}
TEST_F(RouteTest, CancellationWaitsForTerminalAndBlocksRestart) {
 auto accepted=call<Start>(start,route());spin(150);finish_cancel=false;
 ASSERT_TRUE(stop(accepted->route_id)->accepted);spin(300);
 EXPECT_EQ(status["state"],"CANCEL_UNCONFIRMED");EXPECT_FALSE(call<Start>(start,route("new"))->accepted);
 EXPECT_EQ(visited.size(),1u);finish_cancel=true;spin(150);EXPECT_EQ(status["state"],"CANCELED");
}
TEST_F(RouteTest, CancelDuringDwellNeverAdvances) {
 auto req=route();req->dwell_sec=1.;auto accepted=call<Start>(start,req);spin(150);
 ASSERT_TRUE(goal);goal->succeed(std::make_shared<Nav::Result>());goal.reset();spin(70);
 ASSERT_EQ(status["state"],"DWELL");stop(accepted->route_id);spin(200);
 EXPECT_EQ(status["state"],"CANCELED");EXPECT_EQ(visited.size(),1u);EXPECT_EQ(cancel_calls,0);
}
TEST_F(RouteTest, FailureOrPreemptionStopsInsteadOfSkipping) {
 ASSERT_TRUE(call<Start>(start,route())->accepted);spin(150);ASSERT_TRUE(goal);
 goal->abort(std::make_shared<Nav::Result>());goal.reset();spin(200);
 EXPECT_EQ(status["state"],"FAILED");EXPECT_EQ(visited.size(),1u);
}
TEST_F(RouteTest, WaypointTimeoutCancelsOwnedGoal) {
 ASSERT_TRUE(call<Start>(start,route())->accepted);spin(1550);
 EXPECT_EQ(status["state"],"FAILED");EXPECT_EQ(cancel_calls,1);EXPECT_EQ(visited.size(),1u);
}
TEST_F(RouteTest, ValidatesDraftAndIdempotencyPayload) {
 auto req=route();req->waypoints.resize(1);EXPECT_FALSE(call<Start>(start,req)->accepted);
 req=route();req->waypoints[1].header.frame_id="odom";EXPECT_FALSE(call<Start>(start,req)->accepted);
 req=route();req->waypoints[0].pose.orientation.w=0;EXPECT_FALSE(call<Start>(start,req)->accepted);
 req=route();req->waypoints[0].pose.position.x=NAN;EXPECT_FALSE(call<Start>(start,req)->accepted);
 req=route();EXPECT_TRUE(call<Start>(start,req)->accepted);
 req->waypoints[0].pose.position.x=3;EXPECT_FALSE(call<Start>(start,req)->accepted);
}
TEST(LateAcceptance, CancelDuringDispatchCancelsLateAcceptedGoal) {
 rclcpp::init(0,nullptr);
 auto fake=std::make_shared<rclcpp::Node>("late_route_server");
 using GH=rclcpp_action::ServerGoalHandle<Nav>;
 std::shared_ptr<GH> goal;std::atomic<int> accepted{0},cancels{0};
 auto server=rclcpp_action::create_server<Nav>(fake,"/test_route/late_nav",
  [&](const auto &,auto){std::this_thread::sleep_for(std::chrono::milliseconds(300));return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [&](auto){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
  [&](auto h){goal=h;++accepted;});
 auto timer=fake->create_wall_timer(std::chrono::milliseconds(10),[&]{if(goal && goal->is_canceling()){goal->canceled(std::make_shared<Nav::Result>());goal.reset();}});
 rclcpp::executors::SingleThreadedExecutor server_executor;server_executor.add_node(fake);
 std::thread thread([&]{server_executor.spin();});
 rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("navigation_action","/test_route/late_nav"),rclcpp::Parameter("cancel_timeout_sec",.1)});
 auto runner=std::make_shared<LoopRouteExecutor>(options);auto driver=std::make_shared<rclcpp::Node>("late_route_driver");
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(runner);executor.add_node(driver);
 Json status;auto sub=driver->create_subscription<std_msgs::msg::String>("/loop_route_executor/status",10,[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});
 auto spin=[&](int ms){auto end=Clock::now()+std::chrono::milliseconds(ms);while(Clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}};
 auto start=driver->create_client<Start>("/loop_route_executor/start");auto cancel=driver->create_client<Cancel>("/loop_route_executor/cancel");
 spin(250);
 auto req=std::make_shared<Start::Request>();req->request_id="late";req->expected_boot_id=status.at("boot_id");
 geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.orientation.w=1;req->waypoints={p,p};
 auto sent=start->async_send_request(req);spin(100);
 if(sent.wait_for(std::chrono::seconds(0))==std::future_status::ready){
  auto cancel_req=std::make_shared<Cancel::Request>();cancel_req->route_id=sent.get()->route_id;
  cancel->async_send_request(cancel_req);spin(170);EXPECT_EQ(status["state"],"CANCEL_UNCONFIRMED");
  spin(350);EXPECT_EQ(status["state"],"CANCELED");EXPECT_EQ(accepted,1);EXPECT_EQ(cancels,1);
 }else ADD_FAILURE()<<"Start response missing";
 server_executor.cancel();thread.join();
 executor.remove_node(runner);executor.remove_node(driver);server_executor.remove_node(fake);rclcpp::shutdown();
}

TEST_F(RouteTest, GoalRejectionStopsWithoutSkipping) {
 reject_goal=true;ASSERT_TRUE(call<Start>(start,route())->accepted);spin(200);
 EXPECT_EQ(status["state"],"FAILED");EXPECT_TRUE(visited.empty());
}
TEST_F(RouteTest, CancelWhileServerUnavailableDoesNotDispatch) {
 server.reset();spin(150);auto accepted=call<Start>(start,route());ASSERT_TRUE(accepted->accepted);
 EXPECT_TRUE(stop(accepted->route_id)->accepted);spin(200);
 EXPECT_EQ(status["state"],"CANCELED");EXPECT_TRUE(visited.empty());
}

TEST_F(RouteTest, OldAcceptedRequestCannotRestartAfterAnotherRoute) {
 auto first=call<Start>(start,route("first"));ASSERT_TRUE(first->accepted);stop(first->route_id);spin(150);
 auto second=call<Start>(start,route("second"));ASSERT_TRUE(second->accepted);stop(second->route_id);spin(150);
 EXPECT_FALSE(call<Start>(start,route("first"))->accepted);
}

TEST_F(RouteTest, StaleBootIsRejectedAndDuplicateTerminalIsNotActive) {
 auto req=route();const auto current=req->expected_boot_id;req->expected_boot_id="previous-boot";
 EXPECT_FALSE(call<Start>(start,req)->accepted);EXPECT_TRUE(visited.empty());
 req->expected_boot_id=current;auto first=call<Start>(start,req);ASSERT_TRUE(first->accepted);EXPECT_TRUE(first->active);
 stop(first->route_id);spin(150);
 auto duplicate=call<Start>(start,req);EXPECT_TRUE(duplicate->accepted);EXPECT_FALSE(duplicate->active);EXPECT_EQ(duplicate->state,"CANCELED");
 // A fresh executor also rejects the old request, even though it has no request history.
 executor->remove_node(runner);runner.reset();
 rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("navigation_action","/test_route/nav")});
 runner=std::make_shared<LoopRouteExecutor>(options);executor->add_node(runner);spin(250);
 EXPECT_FALSE(call<Start>(start,req)->accepted);
}
