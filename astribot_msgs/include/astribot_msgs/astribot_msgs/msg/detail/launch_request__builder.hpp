// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/LaunchRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/launch_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_LaunchRequest_launch_action
{
public:
  explicit Init_LaunchRequest_launch_action(::astribot_msgs::msg::LaunchRequest & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::LaunchRequest launch_action(::astribot_msgs::msg::LaunchRequest::_launch_action_type arg)
  {
    msg_.launch_action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::LaunchRequest msg_;
};

class Init_LaunchRequest_launch_target
{
public:
  Init_LaunchRequest_launch_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaunchRequest_launch_action launch_target(::astribot_msgs::msg::LaunchRequest::_launch_target_type arg)
  {
    msg_.launch_target = std::move(arg);
    return Init_LaunchRequest_launch_action(msg_);
  }

private:
  ::astribot_msgs::msg::LaunchRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::LaunchRequest>()
{
  return astribot_msgs::msg::builder::Init_LaunchRequest_launch_target();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__BUILDER_HPP_
