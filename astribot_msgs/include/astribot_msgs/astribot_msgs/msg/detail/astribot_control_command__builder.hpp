// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/astribot_control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_AstribotControlCommand_add_default_torso
{
public:
  explicit Init_AstribotControlCommand_add_default_torso(::astribot_msgs::msg::AstribotControlCommand & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::AstribotControlCommand add_default_torso(::astribot_msgs::msg::AstribotControlCommand::_add_default_torso_type arg)
  {
    msg_.add_default_torso = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

class Init_AstribotControlCommand_use_wbc
{
public:
  explicit Init_AstribotControlCommand_use_wbc(::astribot_msgs::msg::AstribotControlCommand & msg)
  : msg_(msg)
  {}
  Init_AstribotControlCommand_add_default_torso use_wbc(::astribot_msgs::msg::AstribotControlCommand::_use_wbc_type arg)
  {
    msg_.use_wbc = std::move(arg);
    return Init_AstribotControlCommand_add_default_torso(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

class Init_AstribotControlCommand_frame
{
public:
  explicit Init_AstribotControlCommand_frame(::astribot_msgs::msg::AstribotControlCommand & msg)
  : msg_(msg)
  {}
  Init_AstribotControlCommand_use_wbc frame(::astribot_msgs::msg::AstribotControlCommand::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_AstribotControlCommand_use_wbc(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

class Init_AstribotControlCommand_control_way
{
public:
  explicit Init_AstribotControlCommand_control_way(::astribot_msgs::msg::AstribotControlCommand & msg)
  : msg_(msg)
  {}
  Init_AstribotControlCommand_frame control_way(::astribot_msgs::msg::AstribotControlCommand::_control_way_type arg)
  {
    msg_.control_way = std::move(arg);
    return Init_AstribotControlCommand_frame(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

class Init_AstribotControlCommand_command_list
{
public:
  explicit Init_AstribotControlCommand_command_list(::astribot_msgs::msg::AstribotControlCommand & msg)
  : msg_(msg)
  {}
  Init_AstribotControlCommand_control_way command_list(::astribot_msgs::msg::AstribotControlCommand::_command_list_type arg)
  {
    msg_.command_list = std::move(arg);
    return Init_AstribotControlCommand_control_way(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

class Init_AstribotControlCommand_dofs_list
{
public:
  explicit Init_AstribotControlCommand_dofs_list(::astribot_msgs::msg::AstribotControlCommand & msg)
  : msg_(msg)
  {}
  Init_AstribotControlCommand_command_list dofs_list(::astribot_msgs::msg::AstribotControlCommand::_dofs_list_type arg)
  {
    msg_.dofs_list = std::move(arg);
    return Init_AstribotControlCommand_command_list(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

class Init_AstribotControlCommand_name_list
{
public:
  Init_AstribotControlCommand_name_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AstribotControlCommand_dofs_list name_list(::astribot_msgs::msg::AstribotControlCommand::_name_list_type arg)
  {
    msg_.name_list = std::move(arg);
    return Init_AstribotControlCommand_dofs_list(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotControlCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::AstribotControlCommand>()
{
  return astribot_msgs::msg::builder::Init_AstribotControlCommand_name_list();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__BUILDER_HPP_
