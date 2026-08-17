// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/AstribotSerializedProtoMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/astribot_serialized_proto_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_AstribotSerializedProtoMsg_data
{
public:
  Init_AstribotSerializedProtoMsg_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::msg::AstribotSerializedProtoMsg data(::astribot_msgs::msg::AstribotSerializedProtoMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotSerializedProtoMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::AstribotSerializedProtoMsg>()
{
  return astribot_msgs::msg::builder::Init_AstribotSerializedProtoMsg_data();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__BUILDER_HPP_
