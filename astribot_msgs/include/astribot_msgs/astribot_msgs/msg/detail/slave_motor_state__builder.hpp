// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/SlaveMotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/slave_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_SlaveMotorState_slave_motor_state
{
public:
  explicit Init_SlaveMotorState_slave_motor_state(::astribot_msgs::msg::SlaveMotorState & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::SlaveMotorState slave_motor_state(::astribot_msgs::msg::SlaveMotorState::_slave_motor_state_type arg)
  {
    msg_.slave_motor_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::SlaveMotorState msg_;
};

class Init_SlaveMotorState_header
{
public:
  Init_SlaveMotorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SlaveMotorState_slave_motor_state header(::astribot_msgs::msg::SlaveMotorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SlaveMotorState_slave_motor_state(msg_);
  }

private:
  ::astribot_msgs::msg::SlaveMotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::SlaveMotorState>()
{
  return astribot_msgs::msg::builder::Init_SlaveMotorState_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__BUILDER_HPP_
