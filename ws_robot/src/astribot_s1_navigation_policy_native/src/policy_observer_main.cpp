#include "astribot_s1_navigation_policy_native/policy_observer_node.hpp"
#include "astribot_s1_navigation_policy_native/policy_integer_json.hpp"
#include <iostream>
int main(int argc,char** argv) {
  int result=0;
  try {
    (void)astribot::navigation::policy::python_integer_digit_limit();
    rclcpp::init(argc,argv);
    auto node=std::make_shared<astribot::navigation::policy::PolicyObserverNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
    executor.add_node(node);executor.spin();executor.remove_node(node);
  } catch(const std::exception& error){std::cerr<<"policy observer: "<<error.what()<<'\n';result=1;}
  if(rclcpp::ok())rclcpp::shutdown();return result;
}
