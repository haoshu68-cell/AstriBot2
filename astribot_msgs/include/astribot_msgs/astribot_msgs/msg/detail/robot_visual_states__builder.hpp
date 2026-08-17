// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/RobotVisualStates.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/robot_visual_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotVisualStates_twist
{
public:
  explicit Init_RobotVisualStates_twist(::astribot_msgs::msg::RobotVisualStates & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::RobotVisualStates twist(::astribot_msgs::msg::RobotVisualStates::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::RobotVisualStates msg_;
};

class Init_RobotVisualStates_pose
{
public:
  explicit Init_RobotVisualStates_pose(::astribot_msgs::msg::RobotVisualStates & msg)
  : msg_(msg)
  {}
  Init_RobotVisualStates_twist pose(::astribot_msgs::msg::RobotVisualStates::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_RobotVisualStates_twist(msg_);
  }

private:
  ::astribot_msgs::msg::RobotVisualStates msg_;
};

class Init_RobotVisualStates_id
{
public:
  explicit Init_RobotVisualStates_id(::astribot_msgs::msg::RobotVisualStates & msg)
  : msg_(msg)
  {}
  Init_RobotVisualStates_pose id(::astribot_msgs::msg::RobotVisualStates::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_RobotVisualStates_pose(msg_);
  }

private:
  ::astribot_msgs::msg::RobotVisualStates msg_;
};

class Init_RobotVisualStates_header
{
public:
  Init_RobotVisualStates_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotVisualStates_id header(::astribot_msgs::msg::RobotVisualStates::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotVisualStates_id(msg_);
  }

private:
  ::astribot_msgs::msg::RobotVisualStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::RobotVisualStates>()
{
  return astribot_msgs::msg::builder::Init_RobotVisualStates_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__BUILDER_HPP_
