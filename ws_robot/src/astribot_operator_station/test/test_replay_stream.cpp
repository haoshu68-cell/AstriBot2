#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <filesystem>
#include <thread>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
extern char ** environ;
TEST(ReplayStream, PublishesOnlyPrefixedDisplayData) {
 auto root=std::filesystem::path("/tmp")/("replay_contract_"+std::to_string(getpid()));
 std::filesystem::remove_all(root);
 {
 rosbag2_cpp::Writer writer;writer.open(root.string());
 auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();map->header.frame_id="map";map->info.resolution=.1;map->info.width=1;map->info.height=1;map->data={42};
 writer.write(*map,"/map",rclcpp::Time(1000000000LL));
 auto velocity=std::make_shared<geometry_msgs::msg::Twist>();velocity->linear.x=10;
 writer.write(*velocity,"/cmd_vel",rclcpp::Time(1100000000LL));
 }
 rclcpp::init(0,nullptr);auto node=std::make_shared<rclcpp::Node>("replay_contract_observer");int received=0;
 const std::string prefix="/astribot_replay_test_"+std::to_string(getpid());
 auto map_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>(prefix+"/map",rclcpp::QoS(10).transient_local(),[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){++received;EXPECT_EQ(m->data.at(0),42);});
 auto prohibited=node->create_subscription<geometry_msgs::msg::Twist>(prefix+"/cmd_vel",10,[](geometry_msgs::msg::Twist::ConstSharedPtr){FAIL()<<"Replayed velocity";});
 auto original=node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[](geometry_msgs::msg::Twist::ConstSharedPtr){FAIL()<<"Original velocity";});
 std::vector<std::string> arguments={ament_index_cpp::get_package_prefix("astribot_operator_station")+"/lib/astribot_operator_station/replay_stream",root.string(),prefix,"0","4"};
 std::vector<char *> argv;for(auto & arg:arguments)argv.push_back(arg.data());argv.push_back(nullptr);
 pid_t child=-1;ASSERT_EQ(posix_spawn(&child,argv[0],nullptr,nullptr,argv.data(),environ),0);
 const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(5);
 while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 EXPECT_GT(received,0);EXPECT_EQ(prohibited->get_publisher_count(),0u);EXPECT_EQ(original->get_publisher_count(),0u);
 kill(child,SIGTERM);int status=0;waitpid(child,&status,0);EXPECT_TRUE(WIFEXITED(status));
 node.reset();rclcpp::shutdown();std::filesystem::remove_all(root);
}
