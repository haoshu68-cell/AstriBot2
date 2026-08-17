// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/RobotCartesianStates.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATES__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/robot_cartesian_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotCartesianStates_states
{
public:
  explicit Init_RobotCartesianStates_states(::astribot_msgs::msg::RobotCartesianStates & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::RobotCartesianStates states(::astribot_msgs::msg::RobotCartesianStates::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianStates msg_;
};

class Init_RobotCartesianStates_names
{
public:
  explicit Init_RobotCartesianStates_names(::astribot_msgs::msg::RobotCartesianStates & msg)
  : msg_(msg)
  {}
  Init_RobotCartesianStates_states names(::astribot_msgs::msg::RobotCartesianStates::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_RobotCartesianStates_states(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianStates msg_;
};

class Init_RobotCartesianStates_header
{
public:
  Init_RobotCartesianStates_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCartesianStates_names header(::astribot_msgs::msg::RobotCartesianStates::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotCartesianStates_names(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::RobotCartesianStates>()
{
  return astribot_msgs::msg::builder::Init_RobotCartesianStates_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATES__BUILDER_HPP_
