// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:srv/StorageService.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__BUILDER_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/srv/detail/storage_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_StorageService_Request_request
{
public:
  Init_StorageService_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::StorageService_Request request(::astribot_msgs::srv::StorageService_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::StorageService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::StorageService_Request>()
{
  return astribot_msgs::srv::builder::Init_StorageService_Request_request();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_StorageService_Response_response
{
public:
  Init_StorageService_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astribot_msgs::srv::StorageService_Response response(::astribot_msgs::srv::StorageService_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::StorageService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::StorageService_Response>()
{
  return astribot_msgs::srv::builder::Init_StorageService_Response_response();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__BUILDER_HPP_
