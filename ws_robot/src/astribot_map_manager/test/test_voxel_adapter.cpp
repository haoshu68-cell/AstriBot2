#define ASTRIBOT_VOXEL_ADAPTER_NO_MAIN
#include "../src/voxel_session_adapter.cpp"
#include <gtest/gtest.h>
#include <thread>
using namespace astribot_map_manager;
TEST(VoxelAdapter, DisabledDeploymentCannotStartOrResetAnything){
 rclcpp::init(0,nullptr);auto root=fs::temp_directory_path()/("voxel_adapter_test_"+std::to_string(getpid()));
 auto node=std::make_shared<rclcpp::Node>("adapter_test_client");auto adapter=std::make_shared<VoxelSessionAdapter>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("runtime_root",root.string())}));
 using Command=astribot_operator_msgs::srv::OperatorCommand;Json status;
 auto sub=node->create_subscription<std_msgs::msg::String>("/map_session_adapter/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});
 auto client=node->create_client<Command>("/map_session_adapter/command");
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);exec.add_node(adapter);
 auto spin=[&](int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<until){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 spin(400);ASSERT_FALSE(status.empty());EXPECT_FALSE(status.at("supports_voxel_sessions"));EXPECT_EQ(status.at("owned_pid"),-1);
 auto q=std::make_shared<Command::Request>();q->expected_boot_id=status.at("boot_id");q->robot_id="astribot";q->operation="load_session";q->command_id="test";q->payload_json="{}";
 auto f=client->async_send_request(q);spin(200);ASSERT_EQ(f.wait_for(std::chrono::seconds(0)),std::future_status::ready);
 EXPECT_EQ(f.get()->reason_code,"MAP.DEPLOYMENT_NOT_CONFIGURED");EXPECT_EQ(node->count_publishers("/cmd_vel"),0u);
 exec.remove_node(adapter);exec.remove_node(node);adapter.reset();node.reset();rclcpp::shutdown();fs::remove_all(root);
}

#include "fixture.hpp"
#include <tf2_ros/transform_broadcaster.h>
class TestAdapter:public VoxelSessionAdapter{
public:
 bool launched=false;pid_t test_pid=-1;
 explicit TestAdapter(const rclcpp::NodeOptions & o):VoxelSessionAdapter(o){}
protected:
 pid_t spawnOwned(std::vector<std::string> command)override{
  EXPECT_NE(std::find(command.begin(),command.end(),"mode:=localization"),command.end());
  EXPECT_NE(std::find(command.begin(),command.end(),"save_map:=0"),command.end());
  EXPECT_NE(std::find(command.begin(),command.end(),"previous_map:=floor1:0.5"),command.end());
  launched=true;test_pid=VoxelSessionAdapter::spawnOwned({"/bin/sleep","30"});return test_pid;
 }
};
TEST(VoxelAdapter, OwnedActivationRequiresResetAndNewEvidence){
 rclcpp::init(0,nullptr);auto root=fs::temp_directory_path()/("voxel_positive_"+std::to_string(getpid()));fs::remove_all(root);makeMap(root/"imports/floor1");
 Json target;{Catalog c(root/"catalog",root/"imports");target=c.command("import","map_import",{{"expected_revision",0},{"map_id","floor1"},{"floor",1},{"directory",(root/"imports/floor1").string()}},"test");}
 auto node=std::make_shared<rclcpp::Node>("activation_dependencies");
 auto global=std::make_shared<rclcpp::Node>("global_costmap","/global_costmap");global->declare_parameter("static_layer.map_topic","/map");global->declare_parameter("global_frame","map");global->declare_parameter("always_send_full_costmap",true);
 auto adapter=std::make_shared<TestAdapter>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("profile","sim"),rclcpp::Parameter("allow_navigation_reconfigure",true),rclcpp::Parameter("runtime_root",(root/"runtime").string()),rclcpp::Parameter("asset_root",(root/"catalog/assets").string())}));
 using Command=astribot_operator_msgs::srv::OperatorCommand;using Manage=nav2_msgs::srv::ManageLifecycleNodes;Json status;
 int resets=0,starts=0;bool active=false,streams=true;
 auto lifecycle=node->create_service<Manage>("/lifecycle_manager_navigation/manage_nodes",[&](Manage::Request::SharedPtr q,Manage::Response::SharedPtr r){if(q->command==Manage::Request::RESET){++resets;active=false;}else if(q->command==Manage::Request::STARTUP){EXPECT_EQ(resets,1);++starts;active=true;}else ADD_FAILURE()<<"Unexpected lifecycle command";r->success=true;});
 auto manager=node->create_publisher<std_msgs::msg::String>("/map_manager/status",rclcpp::QoS(1).transient_local());auto authority=node->create_publisher<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local());auto odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slam;rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map;rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr g,l;
 auto tf=std::make_shared<tf2_ros::TransformBroadcaster>(node);
 auto timer=node->create_wall_timer(std::chrono::milliseconds(40),[&]{
  std_msgs::msg::String m;m.data=Json({{"state","LOADING"},{"motion_blocked",true},{"transaction",{{"transaction_id","switch"},{"state","LOADING"},{"target",target}}}}).dump();manager->publish(m);
  m.data=R"({"control_state":"HELD"})";authority->publish(m);nav_msgs::msg::Odometry o;o.header.stamp=node->now();odom->publish(o);
  if(!adapter->launched)return;
  if(!slam){slam=node->create_publisher<std_msgs::msg::String>("/slam/status",rclcpp::QoS(1).transient_local());map=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());g=node->create_publisher<nav2_msgs::msg::Costmap>("/global_costmap/costmap_raw",rclcpp::QoS(1).transient_local());l=node->create_publisher<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",rclcpp::QoS(1).transient_local());m.data="TRACKING";slam->publish(m);}
  if(!streams)return;
  for(const auto & frames:std::vector<std::pair<std::string,std::string>>{{"map","astribot_torso_base"},{"camera_init","aft_mapped"}}){geometry_msgs::msg::TransformStamped t;t.header.stamp=node->now();t.header.frame_id=frames.first;t.child_frame_id=frames.second;t.transform.rotation.w=1;tf->sendTransform(t);}
  nav_msgs::msg::OccupancyGrid grid;grid.header.stamp=node->now();grid.header.frame_id="map";grid.info.resolution=.05;grid.info.width=grid.info.height=2;grid.data={0,0,0,0};map->publish(grid);
  if(active){nav2_msgs::msg::Costmap cost;cost.header.frame_id="map";cost.metadata.update_time=node->now();cost.metadata.size_x=cost.metadata.size_y=2;cost.data={0,0,0,0};g->publish(cost);cost.header.frame_id="odom";l->publish(cost);}
 });
 auto sub=node->create_subscription<std_msgs::msg::String>("/map_session_adapter/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});auto client=node->create_client<Command>("/map_session_adapter/command");
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);exec.add_node(global);exec.add_node(adapter);
 auto spin=[&](int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<until){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 spin(1500);ASSERT_TRUE(status.at("supports_voxel_sessions"));
 auto q=std::make_shared<Command::Request>();q->robot_id="astribot";q->expected_boot_id=status.at("boot_id");q->command_id="attempt1";q->operation="load_session";q->payload_json=Json({{"attempt_id","attempt1"},{"transaction_id","switch"},{"target",target}}).dump();
 auto external=std::make_shared<rclcpp::Node>("voxelslam");exec.add_node(external);spin(300);
 auto rejected=client->async_send_request(q);spin(100);ASSERT_EQ(rejected.wait_for(std::chrono::seconds(0)),std::future_status::ready);EXPECT_EQ(rejected.get()->reason_code,"MAP.EXTERNAL_SLAM_OWNED_BY_OTHER");EXPECT_EQ(resets,0);
 exec.remove_node(external);external.reset();spin(300);
 global->set_parameter(rclcpp::Parameter("static_layer.map_topic","/wrong_map"));
 auto mismatch=client->async_send_request(q);spin(1000);ASSERT_EQ(mismatch.wait_for(std::chrono::seconds(0)),std::future_status::ready);EXPECT_TRUE(mismatch.get()->accepted);EXPECT_EQ(status.at("state"),"FAILED");EXPECT_EQ(status.at("reason_code"),"MAP.NAV_CONFIG_MISMATCH");EXPECT_EQ(resets,0);
 global->set_parameter(rclcpp::Parameter("static_layer.map_topic","/map"));q->command_id="attempt2";q->payload_json=Json({{"attempt_id","attempt2"},{"transaction_id","switch"},{"target",target}}).dump();
 auto response=client->async_send_request(q);spin(300);ASSERT_EQ(response.wait_for(std::chrono::seconds(0)),std::future_status::ready);ASSERT_TRUE(response.get()->accepted);
 spin(2200);EXPECT_EQ(status.at("state"),"READY")<<status.dump();EXPECT_EQ(resets,1);EXPECT_EQ(starts,1);
 auto duplicate=client->async_send_request(q);spin(100);EXPECT_TRUE(duplicate.get()->accepted);EXPECT_EQ(resets,1);
 streams=false;spin(2400);EXPECT_NE(status.at("state"),"READY");EXPECT_FALSE(status.at("tf_ready"));EXPECT_EQ(node->count_publishers("/cmd_vel"),0u);
 pid_t child=adapter->test_pid;exec.remove_node(adapter);exec.remove_node(global);exec.remove_node(node);adapter.reset();if(child>0){int result=0;waitpid(child,&result,0);}global.reset();node.reset();rclcpp::shutdown();fs::remove_all(root);
}
