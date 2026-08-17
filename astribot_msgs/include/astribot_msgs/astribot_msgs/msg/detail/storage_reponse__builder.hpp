// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/StorageReponse.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REPONSE__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/storage_reponse__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_StorageReponse_response
{
public:
  explicit Init_StorageReponse_response(::astribot_msgs::msg::StorageReponse & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::StorageReponse response(::astribot_msgs::msg::StorageReponse::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::StorageReponse msg_;
};

class Init_StorageReponse_is_success
{
public:
  Init_StorageReponse_is_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StorageReponse_response is_success(::astribot_msgs::msg::StorageReponse::_is_success_type arg)
  {
    msg_.is_success = std::move(arg);
    return Init_StorageReponse_response(msg_);
  }

private:
  ::astribot_msgs::msg::StorageReponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::StorageReponse>()
{
  return astribot_msgs::msg::builder::Init_StorageReponse_is_success();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REPONSE__BUILDER_HPP_
