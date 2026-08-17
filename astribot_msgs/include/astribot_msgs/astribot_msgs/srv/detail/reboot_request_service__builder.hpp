// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:srv/RebootRequestService.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__REBOOT_REQUEST_SERVICE__BUILDER_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__REBOOT_REQUEST_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/srv/detail/reboot_request_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::RebootRequestService_Request>()
{
  return ::astribot_msgs::srv::RebootRequestService_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_RebootRequestService_Response_message
{
public:
  explicit Init_RebootRequestService_Response_message(::astribot_msgs::srv::RebootRequestService_Response & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::srv::RebootRequestService_Response message(::astribot_msgs::srv::RebootRequestService_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::RebootRequestService_Response msg_;
};

class Init_RebootRequestService_Response_response
{
public:
  Init_RebootRequestService_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RebootRequestService_Response_message response(::astribot_msgs::srv::RebootRequestService_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_RebootRequestService_Response_message(msg_);
  }

private:
  ::astribot_msgs::srv::RebootRequestService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::RebootRequestService_Response>()
{
  return astribot_msgs::srv::builder::Init_RebootRequestService_Response_response();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__REBOOT_REQUEST_SERVICE__BUILDER_HPP_
