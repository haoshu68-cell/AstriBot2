// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:srv/DoubleArrayRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__BUILDER_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/srv/detail/double_array_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_DoubleArrayRequest_Request_data
{
public:
  explicit Init_DoubleArrayRequest_Request_data(::astribot_msgs::srv::DoubleArrayRequest_Request & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::srv::DoubleArrayRequest_Request data(::astribot_msgs::srv::DoubleArrayRequest_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::DoubleArrayRequest_Request msg_;
};

class Init_DoubleArrayRequest_Request_header
{
public:
  Init_DoubleArrayRequest_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DoubleArrayRequest_Request_data header(::astribot_msgs::srv::DoubleArrayRequest_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DoubleArrayRequest_Request_data(msg_);
  }

private:
  ::astribot_msgs::srv::DoubleArrayRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::DoubleArrayRequest_Request>()
{
  return astribot_msgs::srv::builder::Init_DoubleArrayRequest_Request_header();
}

}  // namespace astribot_msgs


namespace astribot_msgs
{

namespace srv
{

namespace builder
{

class Init_DoubleArrayRequest_Response_data
{
public:
  explicit Init_DoubleArrayRequest_Response_data(::astribot_msgs::srv::DoubleArrayRequest_Response & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::srv::DoubleArrayRequest_Response data(::astribot_msgs::srv::DoubleArrayRequest_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::srv::DoubleArrayRequest_Response msg_;
};

class Init_DoubleArrayRequest_Response_header
{
public:
  Init_DoubleArrayRequest_Response_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DoubleArrayRequest_Response_data header(::astribot_msgs::srv::DoubleArrayRequest_Response::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DoubleArrayRequest_Response_data(msg_);
  }

private:
  ::astribot_msgs::srv::DoubleArrayRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::srv::DoubleArrayRequest_Response>()
{
  return astribot_msgs::srv::builder::Init_DoubleArrayRequest_Response_header();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__BUILDER_HPP_
