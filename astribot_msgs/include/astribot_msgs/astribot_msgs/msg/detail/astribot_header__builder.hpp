// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/AstribotHeader.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/astribot_header__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_AstribotHeader_time_pub
{
public:
  explicit Init_AstribotHeader_time_pub(::astribot_msgs::msg::AstribotHeader & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::AstribotHeader time_pub(::astribot_msgs::msg::AstribotHeader::_time_pub_type arg)
  {
    msg_.time_pub = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotHeader msg_;
};

class Init_AstribotHeader_time_meas
{
public:
  explicit Init_AstribotHeader_time_meas(::astribot_msgs::msg::AstribotHeader & msg)
  : msg_(msg)
  {}
  Init_AstribotHeader_time_pub time_meas(::astribot_msgs::msg::AstribotHeader::_time_meas_type arg)
  {
    msg_.time_meas = std::move(arg);
    return Init_AstribotHeader_time_pub(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotHeader msg_;
};

class Init_AstribotHeader_seq
{
public:
  Init_AstribotHeader_seq()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AstribotHeader_time_meas seq(::astribot_msgs::msg::AstribotHeader::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return Init_AstribotHeader_time_meas(msg_);
  }

private:
  ::astribot_msgs::msg::AstribotHeader msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::AstribotHeader>()
{
  return astribot_msgs::msg::builder::Init_AstribotHeader_seq();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__BUILDER_HPP_
