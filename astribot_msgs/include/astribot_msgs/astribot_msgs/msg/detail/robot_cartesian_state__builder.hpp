// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/RobotCartesianState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATE__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/robot_cartesian_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotCartesianState_wrench
{
public:
  explicit Init_RobotCartesianState_wrench(::astribot_msgs::msg::RobotCartesianState & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::RobotCartesianState wrench(::astribot_msgs::msg::RobotCartesianState::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianState msg_;
};

class Init_RobotCartesianState_twist
{
public:
  explicit Init_RobotCartesianState_twist(::astribot_msgs::msg::RobotCartesianState & msg)
  : msg_(msg)
  {}
  Init_RobotCartesianState_wrench twist(::astribot_msgs::msg::RobotCartesianState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_RobotCartesianState_wrench(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianState msg_;
};

class Init_RobotCartesianState_pose
{
public:
  explicit Init_RobotCartesianState_pose(::astribot_msgs::msg::RobotCartesianState & msg)
  : msg_(msg)
  {}
  Init_RobotCartesianState_twist pose(::astribot_msgs::msg::RobotCartesianState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_RobotCartesianState_twist(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianState msg_;
};

class Init_RobotCartesianState_header
{
public:
  Init_RobotCartesianState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCartesianState_pose header(::astribot_msgs::msg::RobotCartesianState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotCartesianState_pose(msg_);
  }

private:
  ::astribot_msgs::msg::RobotCartesianState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::RobotCartesianState>()
{
  return astribot_msgs::msg::builder::Init_RobotCartesianState_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATE__BUILDER_HPP_
