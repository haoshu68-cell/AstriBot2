// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/DoubleArray.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/double_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_DoubleArray_data
{
public:
  explicit Init_DoubleArray_data(::astribot_msgs::msg::DoubleArray & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::DoubleArray data(::astribot_msgs::msg::DoubleArray::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::DoubleArray msg_;
};

class Init_DoubleArray_header
{
public:
  Init_DoubleArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DoubleArray_data header(::astribot_msgs::msg::DoubleArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DoubleArray_data(msg_);
  }

private:
  ::astribot_msgs::msg::DoubleArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::DoubleArray>()
{
  return astribot_msgs::msg::builder::Init_DoubleArray_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__BUILDER_HPP_
