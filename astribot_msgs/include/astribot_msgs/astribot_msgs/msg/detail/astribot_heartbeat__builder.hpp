// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/AstribotHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/astribot_heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_AstribotHeartbeat_node_name
{
public:
  explicit Init_AstribotHeartbeat_node_name(::astribot_msgs::msg::AstribotHeartbeat & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::AstribotHeartbeat node_name(::astribot_msgs::msg::AstribotHeartbeat::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotHeartbeat msg_;
};

class Init_AstribotHeartbeat_header
{
public:
  Init_AstribotHeartbeat_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AstribotHeartbeat_node_name header(::astribot_msgs::msg::AstribotHeartbeat::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AstribotHeartbeat_node_name(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotHeartbeat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::AstribotHeartbeat>()
{
  return astribot_msgs::msg::builder::Init_AstribotHeartbeat_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__BUILDER_HPP_
