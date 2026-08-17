// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__CUSTOM_POINT__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__CUSTOM_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/custom_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_CustomPoint_line
{
public:
  explicit Init_CustomPoint_line(::astribot_msgs::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::CustomPoint line(::astribot_msgs::msg::CustomPoint::_line_type arg)
  {
    msg_.line = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

class Init_CustomPoint_tag
{
public:
  explicit Init_CustomPoint_tag(::astribot_msgs::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_line tag(::astribot_msgs::msg::CustomPoint::_tag_type arg)
  {
    msg_.tag = std::move(arg);
    return Init_CustomPoint_line(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

class Init_CustomPoint_reflectivity
{
public:
  explicit Init_CustomPoint_reflectivity(::astribot_msgs::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_tag reflectivity(::astribot_msgs::msg::CustomPoint::_reflectivity_type arg)
  {
    msg_.reflectivity = std::move(arg);
    return Init_CustomPoint_tag(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

class Init_CustomPoint_z
{
public:
  explicit Init_CustomPoint_z(::astribot_msgs::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_reflectivity z(::astribot_msgs::msg::CustomPoint::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_CustomPoint_reflectivity(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

class Init_CustomPoint_y
{
public:
  explicit Init_CustomPoint_y(::astribot_msgs::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_z y(::astribot_msgs::msg::CustomPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CustomPoint_z(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

class Init_CustomPoint_x
{
public:
  explicit Init_CustomPoint_x(::astribot_msgs::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_y x(::astribot_msgs::msg::CustomPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CustomPoint_y(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

class Init_CustomPoint_offset_time
{
public:
  Init_CustomPoint_offset_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomPoint_x offset_time(::astribot_msgs::msg::CustomPoint::_offset_time_type arg)
  {
    msg_.offset_time = std::move(arg);
    return Init_CustomPoint_x(msg_);
  }

private:
  ::astribot_msgs::msg::CustomPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::CustomPoint>()
{
  return astribot_msgs::msg::builder::Init_CustomPoint_offset_time();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__CUSTOM_POINT__BUILDER_HPP_
