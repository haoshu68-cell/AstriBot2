#include <rclcpp/time.hpp>
#include <iostream>
int main(){for(std::int64_t n:{2147483647999999999LL,2147483648499999999LL}){
  try {builtin_interfaces::msg::Time msg=rclcpp::Time(n);std::cout<<n<<" "<<msg.sec<<" "<<msg.nanosec<<"\n";}
  catch(const std::exception& e){std::cout<<n<<" error "<<e.what()<<"\n";}
}}
