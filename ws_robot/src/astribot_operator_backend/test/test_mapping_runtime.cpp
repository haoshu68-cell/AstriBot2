#define ASTRIBOT_MAPPING_RUNTIME_NO_MAIN
#include "../src/mapping_runtime.cpp"
#include <gtest/gtest.h>
#include <thread>
TEST(MappingRuntime, RefusesUnownedStackAndMissingStopEvidence) {
 rclcpp::init(0,nullptr);
 auto runtime=std::make_shared<MappingRuntime>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("profile","sim")}));
 auto driver=std::make_shared<rclcpp::Node>("voxelslam"); // External instance, never owned by the runtime.
 auto odom=driver->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
 auto timer=driver->create_wall_timer(std::chrono::milliseconds(20),[&]{odom->publish(nav_msgs::msg::Odometry());});
 auto client=driver->create_client<std_srvs::srv::Trigger>("/mapping_runtime/start");
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(runtime);executor.add_node(driver);
 auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 auto call=[&]{auto f=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());for(int i=0;i<100&&f.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)spin(10);if(f.wait_for(std::chrono::seconds(0))!=std::future_status::ready)throw std::runtime_error("timeout");return *f.get();};
 spin(200);auto no_stop=call();EXPECT_FALSE(no_stop.success);EXPECT_NE(no_stop.message.find("stop evidence"),std::string::npos);
 spin(2200);auto external=call();EXPECT_FALSE(external.success);EXPECT_NE(external.message.find("Existing SLAM"),std::string::npos);
 executor.remove_node(runtime);executor.remove_node(driver);runtime.reset();driver.reset();rclcpp::shutdown();
}
