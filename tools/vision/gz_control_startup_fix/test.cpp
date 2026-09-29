#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "bounded_urdf.hpp"
using namespace std::chrono_literals;
class Fetch : public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp() override {node=std::make_shared<rclcpp::Node>("fetch_test");server=std::make_shared<rclcpp::Node>("fetch_server");ex.add_node(node);ex.add_node(server);thread=std::thread([this]{ex.spin();});}
  void TearDown() override {ex.cancel();thread.join();}
  void service(bool delay=false,bool wrong_type=false){srv=server->create_service<rcl_interfaces::srv::GetParameters>("/fixture/get_parameters",[this,delay,wrong_type](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> req,std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> res){int n=++requests;if(delay&&n==1)std::this_thread::sleep_for(180ms);rcl_interfaces::msg::ParameterValue v;v.type=wrong_type?2:4;v.string_value=(delay&&n==1)?"<robot name='late_old_model'/>":"<robot name='fixture'/>";res->values.push_back(v);});}
  rclcpp::Node::SharedPtr node,server;rclcpp::executors::MultiThreadedExecutor ex;std::thread thread;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr srv;std::atomic<int> requests{0};
};
TEST_F(Fetch, ValidResponse){service();auto v=astribot::fetch_urdf(node,"/fixture","robot_description",1s,50ms);EXPECT_EQ(v,"<robot name='fixture'/>");}
TEST_F(Fetch, DelayedFirstReplyRetries){service(true);auto v=astribot::fetch_urdf(node,"/fixture","robot_description",1s,60ms);EXPECT_EQ(v,"<robot name='fixture'/>");EXPECT_GT(requests,1);}
TEST_F(Fetch, MissingServiceBounded){auto start=std::chrono::steady_clock::now();EXPECT_TRUE(astribot::fetch_urdf(node,"/missing","robot_description",200ms,50ms).empty());EXPECT_LT(std::chrono::steady_clock::now()-start,600ms);}
TEST_F(Fetch, WrongTypeRejected){service(false,true);EXPECT_TRUE(astribot::fetch_urdf(node,"/fixture","robot_description",200ms,50ms).empty());}
