// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/RobotJointController.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_CONTROLLER__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_CONTROLLER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/robot_joint_controller__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotJointController_command
{
public:
  explicit Init_RobotJointController_command(::astribot_msgs::msg::RobotJointController & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::RobotJointController command(::astribot_msgs::msg::RobotJointController::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointController msg_;
};

class Init_RobotJointController_name
{
public:
  explicit Init_RobotJointController_name(::astribot_msgs::msg::RobotJointController & msg)
  : msg_(msg)
  {}
  Init_RobotJointController_command name(::astribot_msgs::msg::RobotJointController::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_RobotJointController_command(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointController msg_;
};

class Init_RobotJointController_mode
{
public:
  explicit Init_RobotJointController_mode(::astribot_msgs::msg::RobotJointController & msg)
  : msg_(msg)
  {}
  Init_RobotJointController_name mode(::astribot_msgs::msg::RobotJointController::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_RobotJointController_name(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointController msg_;
};

class Init_RobotJointController_header
{
public:
  Init_RobotJointController_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotJointController_mode header(::astribot_msgs::msg::RobotJointController::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotJointController_mode(msg_);
  }

private:
  ::astribot_msgs::msg::RobotJointController msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::RobotJointController>()
{
  return astribot_msgs::msg::builder::Init_RobotJointController_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_CONTROLLER__BUILDER_HPP_
