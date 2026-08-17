// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:srv/RawRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__BUILDER_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/srv/detail/raw_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_RawRequest_Request_request
{
public:
  Init_RawRequest_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::RawRequest_Request request(::astribot_msgs::srv::RawRequest_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::RawRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::RawRequest_Request>()
{
  return astribot_msgs::srv::builder::Init_RawRequest_Request_request();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_RawRequest_Response_response
{
public:
  Init_RawRequest_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::RawRequest_Response response(::astribot_msgs::srv::RawRequest_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::RawRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::RawRequest_Response>()
{
  return astribot_msgs::srv::builder::Init_RawRequest_Response_response();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__BUILDER_HPP_
