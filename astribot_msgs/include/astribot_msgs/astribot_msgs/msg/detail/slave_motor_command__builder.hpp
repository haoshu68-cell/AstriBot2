// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/SlaveMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/slave_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_SlaveMotorCommand_slave_motor_command
{
public:
  explicit Init_SlaveMotorCommand_slave_motor_command(::astribot_msgs::msg::SlaveMotorCommand & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::SlaveMotorCommand slave_motor_command(::astribot_msgs::msg::SlaveMotorCommand::_slave_motor_command_type arg)
  {
    msg_.slave_motor_command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::SlaveMotorCommand msg_;
};

class Init_SlaveMotorCommand_header
{
public:
  Init_SlaveMotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SlaveMotorCommand_slave_motor_command header(::astribot_msgs::msg::SlaveMotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SlaveMotorCommand_slave_motor_command(msg_);
  }

private:
  ::astribot_msgs::msg::SlaveMotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::SlaveMotorCommand>()
{
  return astribot_msgs::msg::builder::Init_SlaveMotorCommand_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__BUILDER_HPP_
