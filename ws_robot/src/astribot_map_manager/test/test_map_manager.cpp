#define ASTRIBOT_MAP_MANAGER_NO_MAIN
#include "../src/map_manager.cpp"
#include "fixture.hpp"
#include <gtest/gtest.h>
#include <thread>
#include <unistd.h>
using namespace astribot_map_manager;
TEST(MapManager, TransferVerificationTimeoutRecoveryAndNoVelocity){
 rclcpp::init(0,nullptr);auto root=fs::temp_directory_path()/("map_manager_test_"+std::to_string(getpid()));fs::remove_all(root);makeMap(root/"imports/floor1");
 auto node=std::make_shared<rclcpp::Node>("fake_map_dependencies");auto manager=std::make_shared<MapManager>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("storage_root",(root/"store").string()),rclcpp::Parameter("import_root",(root/"imports").string()),rclcpp::Parameter("verification_timeout_sec",2.5)}));
 using Command=astribot_operator_msgs::srv::OperatorCommand;Json status,proof={{"boot_id","adapter1"},{"supports_voxel_sessions",true},{"handover_ready",true},{"cargo_known",true},{"transport_ready",true}};bool valid=false,control=true;int loads=0;
 std::map<std::string,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs;for(auto topic:{"/operator_backend/status","/loop_route_executor/status","/exploration_coordinator_node/operator_status","/map_session_adapter/status"})pubs[topic]=node->create_publisher<std_msgs::msg::String>(topic,rclcpp::QoS(1).transient_local());
 auto odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom",10);auto timer=node->create_wall_timer(std::chrono::milliseconds(40),[&]{
 auto send=[&](std::string topic,Json p){std_msgs::msg::String m;m.data=p.dump();pubs.at(topic)->publish(m);};
 send("/operator_backend/status",{{"control_state",control?"HELD":"OBSERVER"},{"control_owner","alice"},{"nav_outstanding",false}});send("/loop_route_executor/status",{{"active",false}});send("/exploration_coordinator_node/operator_status",{{"manual_pause",true},{"goal_in_flight",false}});
 for(auto key:{"localization_ready","tf_ready","map_ready","global_costmap_ready","local_costmap_ready"})proof[key]=valid;
 send("/map_session_adapter/status",proof);nav_msgs::msg::Odometry m;m.header.stamp=node->now();odom->publish(m);});
 auto adapter=node->create_service<Command>("/map_session_adapter/command",[&](Command::Request::SharedPtr q,Command::Response::SharedPtr r){++loads;auto p=Json::parse(q->payload_json);r->accepted=true;r->boot_id="adapter1";proof["attempt_id"]=p.at("attempt_id");proof["transaction_id"]=p.at("transaction_id");proof["map_id"]=p.at("target").at("map_id");proof["map_version"]=p.at("target").at("version");proof["state"]="READY";});
 auto sub=node->create_subscription<std_msgs::msg::String>("/map_manager/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});auto client=node->create_client<Command>("/map_manager/command");
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);exec.add_node(manager);
 auto spin=[&](int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<until){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 auto call=[&](std::string id,std::string op,Json p){p["expected_revision"]=status.at("revision");auto q=std::make_shared<Command::Request>();q->robot_id="astribot";q->expected_boot_id=status.at("boot_id");q->command_id=id;q->operation=op;q->payload_json=p.dump();auto f=client->async_send_request(q);for(int i=0;i<500&&f.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)spin(2);if(f.wait_for(std::chrono::seconds(0))!=std::future_status::ready)throw std::runtime_error("timeout");auto r=*f.get();spin(30);return r;};
 spin(1400);ASSERT_TRUE(status.at("idle_evidence"));auto imported=call("import","map_import",{{"map_id","floor1"},{"floor",1},{"directory",(root/"imports/floor1").string()}});ASSERT_TRUE(imported.accepted)<<imported.message;auto map=Json::parse(imported.result_json);
 EXPECT_TRUE(call("switch","map_switch_begin",{{"map_id","floor1"},{"map_version",map.at("version")},{"manual_transfer",true}}).accepted);spin(200);EXPECT_EQ(loads,0);EXPECT_EQ(status.at("state"),"WAIT_TRANSFER");
 EXPECT_FALSE(call("wrong","map_transfer_confirm",{{"transaction_id","other"}}).accepted);EXPECT_EQ(loads,0);
 EXPECT_TRUE(call("confirm","map_transfer_confirm",{{"transaction_id","switch"}}).accepted);spin(2800);EXPECT_EQ(loads,1);EXPECT_EQ(status.at("state"),"RECOVERY_REQUIRED");EXPECT_TRUE(status.at("active_map").is_null());
 valid=true;EXPECT_TRUE(call("recover","map_recover",{{"transaction_id","switch"}}).accepted);spin(1500);EXPECT_EQ(loads,2);EXPECT_EQ(status.at("transaction").at("state"),"COMMITTED");EXPECT_EQ(status.at("active_map").at("version"),map.at("version"));
 valid=false;spin(200);EXPECT_TRUE(status.at("motion_blocked"));EXPECT_EQ(status.at("active_map").at("version"),map.at("version"));valid=true;spin(200);
 EXPECT_TRUE(call("switch2","map_switch_begin",{{"map_id","floor1"},{"map_version",map.at("version")},{"manual_transfer",false}}).accepted);control=false;spin(300);EXPECT_EQ(status.at("state"),"RECOVERY_REQUIRED");EXPECT_TRUE(status.at("motion_blocked"));
 EXPECT_EQ(node->count_publishers("/cmd_vel"),0u);
 exec.remove_node(manager);exec.remove_node(node);manager.reset();node.reset();rclcpp::shutdown();fs::remove_all(root);
}
