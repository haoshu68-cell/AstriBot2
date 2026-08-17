// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astribot_msgs:msg/StorageRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__BUILDER_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astribot_msgs/msg/detail/storage_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astribot_msgs
{

namespace msg
{

namespace builder
{

class Init_StorageRequest_topic_list
{
public:
  explicit Init_StorageRequest_topic_list(::astribot_msgs::msg::StorageRequest & msg)
  : msg_(msg)
  {}
  ::astribot_msgs::msg::StorageRequest topic_list(::astribot_msgs::msg::StorageRequest::_topic_list_type arg)
  {
    msg_.topic_list = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astribot_msgs::msg::StorageRequest msg_;
};

class Init_StorageRequest_tag_content
{
public:
  explicit Init_StorageRequest_tag_content(::astribot_msgs::msg::StorageRequest & msg)
  : msg_(msg)
  {}
  Init_StorageRequest_topic_list tag_content(::astribot_msgs::msg::StorageRequest::_tag_content_type arg)
  {
    msg_.tag_content = std::move(arg);
    return Init_StorageRequest_topic_list(msg_);
  }

private:
  ::astribot_msgs::msg::StorageRequest msg_;
};

class Init_StorageRequest_storage_type
{
public:
  explicit Init_StorageRequest_storage_type(::astribot_msgs::msg::StorageRequest & msg)
  : msg_(msg)
  {}
  Init_StorageRequest_tag_content storage_type(::astribot_msgs::msg::StorageRequest::_storage_type_type arg)
  {
    msg_.storage_type = std::move(arg);
    return Init_StorageRequest_tag_content(msg_);
  }

private:
  ::astribot_msgs::msg::StorageRequest msg_;
};

class Init_StorageRequest_uuid
{
public:
  Init_StorageRequest_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StorageRequest_storage_type uuid(::astribot_msgs::msg::StorageRequest::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_StorageRequest_storage_type(msg_);
  }

private:
  ::astribot_msgs::msg::StorageRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astribot_msgs::msg::StorageRequest>()
{
  return astribot_msgs::msg::builder::Init_StorageRequest_uuid();
}

}  // namespace astribot_msgs

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__BUILDER_HPP_
