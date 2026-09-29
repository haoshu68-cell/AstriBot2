#define ASTRIBOT_MAP_MANAGER_NO_MAIN
#include "../src/map_manager.cpp"
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <thread>
#include <fstream>
#include <unistd.h>
using namespace astribot_map_manager;
TEST(StaticMapContext, RealGridRequiredMismatchAndPublisherLossBlock) {
 rclcpp::init(0,nullptr);
 auto root=fs::temp_directory_path()/("static_map_context_"+std::to_string(getpid()));fs::remove_all(root);fs::create_directories(root);
 {std::ofstream f(root/"map.pgm");f<<"P2\n2 2\n255\n254 254\n254 0\n";}
 {std::ofstream f(root/"map.yaml");f<<"image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";}
 auto driver=std::make_shared<rclcpp::Node>("static_context_driver");
 auto manager=std::make_shared<MapManager>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time",true),rclcpp::Parameter("storage_root",(root/"store").string()),rclcpp::Parameter("simulation_static_map_yaml",(root/"map.yaml").string())}));
 rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(driver);ex.add_node(manager);Json status;
 auto sub=driver->create_subscription<std_msgs::msg::String>("/map_manager/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=Json::parse(m->data);});
 auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<end){ex.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 spin(350);ASSERT_TRUE(status.contains("motion_blocked"));EXPECT_TRUE(status.at("motion_blocked"));EXPECT_TRUE(status.at("active_map").is_null());
 auto pub=driver->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());
 nav_msgs::msg::OccupancyGrid grid;grid.header.frame_id="map";grid.info.width=grid.info.height=2;grid.info.resolution=.05;grid.info.origin.orientation.w=1;grid.data={0,100,0,0};
 pub->publish(grid);spin(400);EXPECT_FALSE(status.at("motion_blocked"));ASSERT_FALSE(status.at("active_map").is_null());EXPECT_TRUE(status.at("active_map").at("read_only"));EXPECT_FALSE(status.at("switch_available"));
 grid.data[0]=100;pub->publish(grid);spin(250);EXPECT_TRUE(status.at("motion_blocked"));
 grid.data[0]=0;grid.header.frame_id="wrong";pub->publish(grid);spin(250);EXPECT_TRUE(status.at("motion_blocked"));
 grid.header.frame_id="map";grid.info.origin.position.x=1;pub->publish(grid);spin(250);EXPECT_TRUE(status.at("motion_blocked"));
 grid.info.origin.position.x=0;pub->publish(grid);spin(250);EXPECT_FALSE(status.at("motion_blocked"));
 auto duplicate=driver->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());duplicate->publish(grid);spin(350);EXPECT_TRUE(status.at("motion_blocked"));
 duplicate.reset();spin(250);pub->publish(grid);spin(250);EXPECT_FALSE(status.at("motion_blocked"));
 pub.reset();spin(400);EXPECT_TRUE(status.at("motion_blocked"));EXPECT_TRUE(status.at("active_map").is_null());
 ex.remove_node(manager);ex.remove_node(driver);manager.reset();driver.reset();rclcpp::shutdown();fs::remove_all(root);
}
TEST(StaticMapContext, RejectsHardwareAndMissingFiles) {
 rclcpp::init(0,nullptr);
 auto root=fs::temp_directory_path()/("static_map_invalid_"+std::to_string(getpid()));fs::remove_all(root);
 auto options=rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("storage_root",root.string()),rclcpp::Parameter("simulation_static_map_yaml","/missing/map.yaml")});
 EXPECT_THROW(std::make_shared<MapManager>(options),std::runtime_error);
 options.append_parameter_override("use_sim_time",true);
 EXPECT_THROW(std::make_shared<MapManager>(options),std::runtime_error);
 rclcpp::shutdown();fs::remove_all(root);
}
