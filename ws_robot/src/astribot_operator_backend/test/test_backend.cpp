#define ASTRIBOT_OPERATOR_BACKEND_NO_MAIN
#include "../src/operator_backend.cpp"
#include <gtest/gtest.h>
#include <thread>
TEST(Backend, LeaseIdempotencyExpiryAndTerminalBarrier) {
 rclcpp::init(0,nullptr);
 auto node=std::make_shared<rclcpp::Node>("fake_operator_dependencies");
 auto backend=std::make_shared<OperatorBackend>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("lease_ttl_sec",1.0)}));
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);executor.add_node(backend);
 using Command=astribot_operator_msgs::srv::OperatorCommand;using Nav=nav2_msgs::action::NavigateToPose;
 int marks=0,goals=0,cancels=0;
 auto mark=node->create_service<std_srvs::srv::Trigger>("/diagnostics_recorder/mark",[&](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){++marks;r->success=true;});
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal;
 auto nav=rclcpp_action::create_server<Nav>(node,"/navigate_to_pose",[&](const auto &,auto){++goals;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[&](auto){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},[&](auto h){goal=h;});
 Json status;
 auto sub=node->create_subscription<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});
 auto client=node->create_client<Command>("/operator_backend/command");
 auto spin=[&](int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<until){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}};
 auto call=[&](Command::Request::SharedPtr q){auto f=client->async_send_request(q);for(int i=0;i<500&&f.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)spin(2);if(f.wait_for(std::chrono::seconds(0))!=std::future_status::ready)throw std::runtime_error("timeout");return *f.get();};
 spin(300);ASSERT_FALSE(status.empty());
 auto q=std::make_shared<Command::Request>();q->robot_id="astribot";q->command_id="client";q->expected_boot_id=status.at("boot_id");q->operation="acquire";
 auto acquired=call(q);ASSERT_TRUE(acquired.accepted);EXPECT_EQ(Json::parse(call(q).result_json).at("lease_id"),Json::parse(acquired.result_json).at("lease_id"));q->lease_id=Json::parse(acquired.result_json).at("lease_id");
 q->operation="record_mark";q->command_id="mark";
 for(int i=0;i<100;++i)EXPECT_TRUE(call(q).accepted);
 spin(50);EXPECT_EQ(marks,1);
 q->payload_json="{}";EXPECT_EQ(call(q).reason_code,"REQUEST.CONFLICT");q->payload_json="";
 q->operation="arm_execute";q->command_id="arm-no-adapter";EXPECT_EQ(call(q).reason_code,"ARM.EXECUTION_ADAPTER_UNAVAILABLE");
 q->operation="record_mark";
 auto token=q->lease_id;q->lease_id="invalid";q->command_id="other";EXPECT_EQ(call(q).reason_code,"CONTROL.NOT_OWNER");q->lease_id=token;
 q->operation="renew";EXPECT_TRUE(call(q).accepted);
 q->operation="navigate";q->command_id="nav";q->payload_json=R"({"frame":"map","x":1,"y":0,"yaw":0})";
 EXPECT_TRUE(call(q).accepted);spin(100);EXPECT_EQ(goals,1);
 q->operation="cancel_navigation";q->command_id="stale-cancel";q->payload_json=R"({"operation_id":"old-navigation"})";
 EXPECT_EQ(call(q).reason_code,"NAV.NO_MATCHING_OWNED_GOAL");EXPECT_EQ(cancels,0);
 spin(1300);EXPECT_GE(cancels,1);EXPECT_EQ(status.at("control_state"),"STOP_UNCONFIRMED");
 q->operation="acquire";q->command_id="next";EXPECT_EQ(call(q).reason_code,"CONTROL.BUSY");
 ASSERT_TRUE(goal&&goal->is_canceling());goal->canceled(std::make_shared<Nav::Result>());spin(300);
 auto next=call(q);EXPECT_TRUE(next.accepted);q->lease_id=Json::parse(next.result_json).at("lease_id");
 auto transport_pub=node->create_publisher<std_msgs::msg::String>("/transport/status",10);
 spin(100);std_msgs::msg::String transport_message;transport_message.data=R"({"stage":"GRASP","object_state":"ATTACHED"})";transport_pub->publish(transport_message);spin(250);
 EXPECT_EQ(status.at("transport").at("quality"),"VALID");
 auto runtime_pub=node->create_publisher<std_msgs::msg::String>("/simulation_transport/status",rclcpp::QoS(1).transient_local());
 spin(50);std_msgs::msg::String runtime_message;runtime_message.data=R"({"state":"RUNNING","motion_blocked":true})";runtime_pub->publish(runtime_message);spin(50);
 q->operation="navigate";q->command_id="sim-busy";q->payload_json=R"({"frame":"map","x":1,"y":0,"yaw":0})";
 EXPECT_EQ(call(q).reason_code,"SIM.TRANSPORT_BUSY_OR_UNKNOWN");EXPECT_EQ(goals,1);
 q->operation="simulation_transport_start";q->command_id="sim-duplicate";
 EXPECT_EQ(call(q).reason_code,"SIM.TRANSPORT_BUSY_OR_UNKNOWN");
 runtime_message.data=R"({"state":"RECOVERY_REQUIRED","motion_blocked":true})";runtime_pub->publish(runtime_message);spin(50);
 q->command_id="sim-recovery";EXPECT_EQ(call(q).reason_code,"SIM.TRANSPORT_BUSY_OR_UNKNOWN");
 runtime_message.data=R"({"state":"IDLE","motion_blocked":false})";runtime_pub->publish(runtime_message);spin(50);
 q->operation="renew";EXPECT_TRUE(call(q).accepted);
 Json catalog={{"boot_id","mapboot"},{"revision",1},{"active_map",{{"version","v1"}}},{"motion_blocked",true}};
 auto catalog_pub=node->create_publisher<std_msgs::msg::String>("/map_manager/status",rclcpp::QoS(1).transient_local());
 auto update_catalog=[&]{std_msgs::msg::String m;m.data=catalog.dump();catalog_pub->publish(m);spin(100);};update_catalog();
 q->operation="navigate";q->command_id="blocked-map";q->payload_json=R"({"frame":"map","x":1,"y":0,"yaw":0,"map_version":"v1"})";
 EXPECT_EQ(call(q).reason_code,"MAP.CONTEXT_BLOCKED");EXPECT_EQ(goals,1);
 catalog["motion_blocked"]=false;catalog["active_map"]["version"]="v2";update_catalog();
 q->command_id="stale-map";EXPECT_EQ(call(q).reason_code,"MAP.VERSION_MISMATCH");EXPECT_EQ(goals,1);
 executor.remove_node(backend);executor.remove_node(node);backend.reset();node.reset();rclcpp::shutdown();
}
