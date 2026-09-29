#define ASTRIBOT_ZONE_SERVER_NO_MAIN
#include "../src/zone_server.cpp"
#include "astribot_navigation_zones/zone_layer.hpp"
#include <gtest/gtest.h>
#include <thread>
using namespace astribot_navigation_zones;
namespace {
void spin(rclcpp::executors::SingleThreadedExecutor &e,int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<end){e.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}}
Json wall(){return Json::array({{{"id","wall"},{"name","test"},{"type","wall"},{"enabled",true},{"points",{{.5,0},{.5,2}}},{"width_m",.05},{"margin_m",0.}}});}
}
TEST(ZoneLayer, ResetRemovalAndStalenessFailClosed){
 rclcpp::init(0,nullptr);auto node=std::make_shared<nav2_util::LifecycleNode>("zone_costmap_test","",rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("zone.consumer","global_costmap")}));
 auto driver=std::make_shared<rclcpp::Node>("zone_layer_driver");rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node->get_node_base_interface());executor.add_node(driver);
 nav2_costmap_2d::LayeredCostmap map("map",false,false);map.resizeMap(20,20,.1,0,0);tf2_ros::Buffer tf(node->get_clock());
 auto layer=std::make_shared<ZoneLayer>();map.addPlugin(layer);layer->initialize(&map,"zone",&tf,node,node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
 auto pub=driver->create_publisher<std_msgs::msg::String>("/navigation_zones/constraints",rclcpp::QoS(1).transient_local());
 Json w={{"schema_version",1},{"frame","map"},{"boot_id","b"},{"context_id","map:m:v"},{"revision",1},{"sequence",1},{"valid",true},{"regions",wall()}};
 auto publish=[&]{w["stamp"]=driver->now().seconds();std_msgs::msg::String m;m.data=w.dump();pub->publish(m);spin(executor,150);};publish();map.updateMap(1,1,0);
 EXPECT_EQ(map.getCostmap()->getCost(5,10),254);EXPECT_EQ(map.getCostmap()->getCost(15,10),0);EXPECT_FALSE(layer->isClearable());
 layer->reset();map.getCostmap()->resetMap(0,0,20,20);map.updateMap(1,1,0);EXPECT_EQ(map.getCostmap()->getCost(5,10),254);
 w["revision"]=2;w["sequence"]=2;w["regions"]=Json::array();publish();map.updateMap(1,1,0);EXPECT_EQ(map.getCostmap()->getCost(5,10),0);
 auto local_node=std::make_shared<nav2_util::LifecycleNode>("zone_local_test","",rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("zone.consumer","local_costmap")}));executor.add_node(local_node->get_node_base_interface());
 nav2_costmap_2d::LayeredCostmap local("odom",false,false);local.resizeMap(20,20,.1,0,0);auto local_layer=std::make_shared<ZoneLayer>();local.addPlugin(local_layer);local_layer->initialize(&local,"zone",&tf,local_node,local_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
 w["revision"]=3;w["sequence"]=3;w["regions"]=wall();publish();local.updateMap(1,1,0);EXPECT_EQ(local.getCostmap()->getCost(15,10),254);EXPECT_FALSE(local_layer->isCurrent());
 geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="map";transform.child_frame_id="odom";transform.transform.translation.x=-1;transform.transform.rotation.w=1;ASSERT_TRUE(tf.setTransform(transform,"test",true));local.updateMap(1,1,0);EXPECT_TRUE(local_layer->isCurrent());EXPECT_EQ(local.getCostmap()->getCost(15,10),254);EXPECT_EQ(local.getCostmap()->getCost(5,10),0);
 executor.remove_node(local_node->get_node_base_interface());
 spin(executor,2100);map.updateMap(1,1,0);EXPECT_EQ(map.getCostmap()->getCost(15,10),254);EXPECT_FALSE(layer->isCurrent());
 executor.remove_node(node->get_node_base_interface());executor.remove_node(driver);rclcpp::shutdown();
}
TEST(ZoneServer, MapSessionRevisionStandstillAndConsumerBarrier){
 rclcpp::init(0,nullptr);auto dir=std::filesystem::temp_directory_path()/("zones_ros_"+std::to_string(getpid()));std::filesystem::remove_all(dir);
 auto server=std::make_shared<ZoneServer>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("storage_root",dir.string())}));
 auto driver=std::make_shared<rclcpp::Node>("zone_server_driver");rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(server);executor.add_node(driver);
 Json status;auto sub=driver->create_subscription<std_msgs::msg::String>("/navigation_zones/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});
 auto map=driver->create_publisher<std_msgs::msg::String>("/map_manager/status",rclcpp::QoS(1).transient_local());auto mapping=driver->create_publisher<std_msgs::msg::String>("/mapping_session/status",rclcpp::QoS(1).transient_local());auto owner=driver->create_publisher<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local());auto odom=driver->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());auto ack=driver->create_publisher<std_msgs::msg::String>("/navigation_zones/applied",10);
 bool acknowledge=false,moving=false;std::string mapping_state="IDLE";Json catalog={{"active_map",nullptr},{"transaction",nullptr}};
 auto timer=driver->create_wall_timer(std::chrono::milliseconds(30),[&]{std_msgs::msg::String m;m.data=catalog.dump();map->publish(m);m.data=Json({{"session_id","session"},{"state",mapping_state},{"directory","/maps/session"}}).dump();mapping->publish(m);m.data=R"({"control_state":"HELD","control_owner":"tester","nav_outstanding":false,"scene_resource_busy":false,"route_id":""})";owner->publish(m);nav_msgs::msg::Odometry o;o.header.stamp=driver->now();o.twist.twist.linear.x=moving?1.:0.;odom->publish(o);
 if(acknowledge&&status.contains("token"))for(auto c:{"global_costmap","local_costmap"}){m.data=Json({{"token",status.at("token")},{"consumer",c},{"applied",true}}).dump();ack->publish(m);}});
 spin(executor,1400);ASSERT_FALSE(status.empty());EXPECT_FALSE(status.at("ready"));EXPECT_TRUE(status.at("can_edit"));EXPECT_EQ(status.at("context_id"),"mapping:session");
 acknowledge=true;spin(executor,450);EXPECT_TRUE(status.at("ready"));
 using Command=astribot_operator_msgs::srv::OperatorCommand;auto client=driver->create_client<Command>("/navigation_zones/command");auto q=std::make_shared<Command::Request>();q->schema_version=1;q->robot_id="astribot";q->expected_boot_id=status.at("boot_id");q->operation="zones_replace";q->command_id="edit";
 auto payload=Json({{"context_id","mapping:session"},{"expected_revision",0},{"regions",wall()},{"reviewed",true}});q->payload_json=payload.dump();
 auto call=[&]{auto f=client->async_send_request(q);for(int i=0;i<200&&f.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)spin(executor,5);return f.get();};
 auto response=call();ASSERT_TRUE(response->accepted)<<response->message;spin(executor,450);EXPECT_EQ(status.at("revision"),1);EXPECT_TRUE(status.at("ready"));
 q->command_id="stale";EXPECT_FALSE(call()->accepted);payload["expected_revision"]=1;q->payload_json=payload.dump();moving=true;spin(executor,100);q->command_id="moving";EXPECT_FALSE(call()->accepted);
 mapping_state="SAVED";moving=false;spin(executor,1300);EXPECT_FALSE(status.at("can_edit"));q->command_id="closed-session";EXPECT_FALSE(call()->accepted);
 catalog={{"active_map",{{"map_id","floor1"},{"version","v1"},{"source_directory","/maps/session"}}},{"transaction",nullptr},{"active_map_ready",true},{"motion_blocked",false}};spin(executor,450);EXPECT_EQ(status.at("context_id"),"map:floor1:v1");EXPECT_TRUE(status.at("review_required"));EXPECT_FALSE(status.at("ready"));EXPECT_EQ(status.at("regions").size(),1u);
 executor.remove_node(server);executor.remove_node(driver);server.reset();rclcpp::shutdown();std::filesystem::remove_all(dir);
}
