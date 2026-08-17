// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:srv/AstribotHeaderSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__BUILDER_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/srv/detail/astribot_header_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_AstribotHeaderSrv_Request_header
{
public:
  Init_AstribotHeaderSrv_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::AstribotHeaderSrv_Request header(::astribot_msgs::srv::AstribotHeaderSrv_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::AstribotHeaderSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::AstribotHeaderSrv_Request>()
{
  return astribot_msgs::srv::builder::Init_AstribotHeaderSrv_Request_header();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_AstribotHeaderSrv_Response_header
{
public:
  Init_AstribotHeaderSrv_Response_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::AstribotHeaderSrv_Response header(::astribot_msgs::srv::AstribotHeaderSrv_Response::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::AstribotHeaderSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::AstribotHeaderSrv_Response>()
{
  return astribot_msgs::srv::builder::Init_AstribotHeaderSrv_Response_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__BUILDER_HPP_
