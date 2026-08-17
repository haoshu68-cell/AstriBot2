// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/RobotJointState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/robot_joint_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotJointState_torque
{
public:
  explicit Init_RobotJointState_torque(::astribot_msgs::msg::RobotJointState & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::RobotJointState torque(::astribot_msgs::msg::RobotJointState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

class Init_RobotJointState_acceleration
{
public:
  explicit Init_RobotJointState_acceleration(::astribot_msgs::msg::RobotJointState & msg)
  : msg_(msg)
  {}
  Init_RobotJointState_torque acceleration(::astribot_msgs::msg::RobotJointState::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_RobotJointState_torque(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

class Init_RobotJointState_velocity
{
public:
  explicit Init_RobotJointState_velocity(::astribot_msgs::msg::RobotJointState & msg)
  : msg_(msg)
  {}
  Init_RobotJointState_acceleration velocity(::astribot_msgs::msg::RobotJointState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_RobotJointState_acceleration(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

class Init_RobotJointState_position
{
public:
  explicit Init_RobotJointState_position(::astribot_msgs::msg::RobotJointState & msg)
  : msg_(msg)
  {}
  Init_RobotJointState_velocity position(::astribot_msgs::msg::RobotJointState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_RobotJointState_velocity(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

class Init_RobotJointState_name
{
public:
  explicit Init_RobotJointState_name(::astribot_msgs::msg::RobotJointState & msg)
  : msg_(msg)
  {}
  Init_RobotJointState_position name(::astribot_msgs::msg::RobotJointState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_RobotJointState_position(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

class Init_RobotJointState_mode
{
public:
  explicit Init_RobotJointState_mode(::astribot_msgs::msg::RobotJointState & msg)
  : msg_(msg)
  {}
  Init_RobotJointState_name mode(::astribot_msgs::msg::RobotJointState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_RobotJointState_name(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

class Init_RobotJointState_header
{
public:
  Init_RobotJointState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotJointState_mode header(::astribot_msgs::msg::RobotJointState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotJointState_mode(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::RobotJointState>()
{
  return astribot_msgs::msg::builder::Init_RobotJointState_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__BUILDER_HPP_
