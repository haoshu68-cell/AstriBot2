// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:srv/LaunchRequestSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__BUILDER_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/srv/detail/launch_request_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_LaunchRequestSrv_Request_request
{
public:
  Init_LaunchRequestSrv_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::LaunchRequestSrv_Request request(::astribot_msgs::srv::LaunchRequestSrv_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::LaunchRequestSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::LaunchRequestSrv_Request>()
{
  return astribot_msgs::srv::builder::Init_LaunchRequestSrv_Request_request();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_LaunchRequestSrv_Response_message
{
public:
  explicit Init_LaunchRequestSrv_Response_message(::astribot_msgs::srv::LaunchRequestSrv_Response & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::srv::LaunchRequestSrv_Response message(::astribot_msgs::srv::LaunchRequestSrv_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::LaunchRequestSrv_Response msg_;
};

class Init_LaunchRequestSrv_Response_response
{
public:
  Init_LaunchRequestSrv_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaunchRequestSrv_Response_message response(::astribot_msgs::srv::LaunchRequestSrv_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_LaunchRequestSrv_Response_message(msg_);
  }

private:
  ::astribot_msgs::srv::LaunchRequestSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::LaunchRequestSrv_Response>()
{
  return astribot_msgs::srv::builder::Init_LaunchRequestSrv_Response_response();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__BUILDER_HPP_
