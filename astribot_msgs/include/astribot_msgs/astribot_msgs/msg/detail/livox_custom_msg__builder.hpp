// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/LivoxCustomMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/livox_custom_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_LivoxCustomMsg_points
{
public:
  explicit Init_LivoxCustomMsg_points(::astribot_msgs::msg::LivoxCustomMsg & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::LivoxCustomMsg points(::astribot_msgs::msg::LivoxCustomMsg::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::LivoxCustomMsg msg_;
};

class Init_LivoxCustomMsg_rsvd
{
public:
  explicit Init_LivoxCustomMsg_rsvd(::astribot_msgs::msg::LivoxCustomMsg & msg)
  : msg_(msg)
  {}
  Init_LivoxCustomMsg_points rsvd(::astribot_msgs::msg::LivoxCustomMsg::_rsvd_type arg)
  {
    msg_.rsvd = std::move(arg);
    return Init_LivoxCustomMsg_points(msg_);
  }

private:
  ::astribot_msgs::msg::LivoxCustomMsg msg_;
};

class Init_LivoxCustomMsg_lidar_id
{
public:
  explicit Init_LivoxCustomMsg_lidar_id(::astribot_msgs::msg::LivoxCustomMsg & msg)
  : msg_(msg)
  {}
  Init_LivoxCustomMsg_rsvd lidar_id(::astribot_msgs::msg::LivoxCustomMsg::_lidar_id_type arg)
  {
    msg_.lidar_id = std::move(arg);
    return Init_LivoxCustomMsg_rsvd(msg_);
  }

private:
  ::astribot_msgs::msg::LivoxCustomMsg msg_;
};

class Init_LivoxCustomMsg_point_num
{
public:
  explicit Init_LivoxCustomMsg_point_num(::astribot_msgs::msg::LivoxCustomMsg & msg)
  : msg_(msg)
  {}
  Init_LivoxCustomMsg_lidar_id point_num(::astribot_msgs::msg::LivoxCustomMsg::_point_num_type arg)
  {
    msg_.point_num = std::move(arg);
    return Init_LivoxCustomMsg_lidar_id(msg_);
  }

private:
  ::astribot_msgs::msg::LivoxCustomMsg msg_;
};

class Init_LivoxCustomMsg_timebase
{
public:
  explicit Init_LivoxCustomMsg_timebase(::astribot_msgs::msg::LivoxCustomMsg & msg)
  : msg_(msg)
  {}
  Init_LivoxCustomMsg_point_num timebase(::astribot_msgs::msg::LivoxCustomMsg::_timebase_type arg)
  {
    msg_.timebase = std::move(arg);
    return Init_LivoxCustomMsg_point_num(msg_);
  }

private:
  ::astribot_msgs::msg::LivoxCustomMsg msg_;
};

class Init_LivoxCustomMsg_header
{
public:
  Init_LivoxCustomMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LivoxCustomMsg_timebase header(::astribot_msgs::msg::LivoxCustomMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LivoxCustomMsg_timebase(msg_);
  }

private:
  ::astribot_msgs::msg::LivoxCustomMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::LivoxCustomMsg>()
{
  return astribot_msgs::msg::builder::Init_LivoxCustomMsg_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__BUILDER_HPP_
