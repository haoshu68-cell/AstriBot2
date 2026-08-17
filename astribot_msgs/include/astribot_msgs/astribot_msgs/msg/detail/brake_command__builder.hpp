// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/BrakeCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/brake_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_BrakeCommand_brake
{
public:
  Init_BrakeCommand_brake()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::msg::BrakeCommand brake(::astribot_msgs::msg::BrakeCommand::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::BrakeCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::BrakeCommand>()
{
  return astribot_msgs::msg::builder::Init_BrakeCommand_brake();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__BUILDER_HPP_
