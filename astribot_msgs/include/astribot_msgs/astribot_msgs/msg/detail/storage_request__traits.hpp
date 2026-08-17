// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/StorageRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/storage_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const StorageRequest & msg,
  std::ostream & out)
{
  out << "{";
  // member: uuid
  {
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << ", ";
  }

  // member: storage_type
  {
    out << "storage_type: ";
    rosidl_generator_traits::value_to_yaml(msg.storage_type, out);
    out << ", ";
  }

  // member: tag_content
  {
    out << "tag_content: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_content, out);
    out << ", ";
  }

  // member: topic_list
  {
    out << "topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_list, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StorageRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: uuid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << "\n";
  }

  // member: storage_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "storage_type: ";
    rosidl_generator_traits::value_to_yaml(msg.storage_type, out);
    out << "\n";
  }

  // member: tag_content
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag_content: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_content, out);
    out << "\n";
  }

  // member: topic_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_list, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StorageRequest & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::msg::StorageRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::StorageRequest & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::StorageRequest>()
{
  return "astribot_msgs::msg::StorageRequest";
}

template<>
inline const char * name<astribot_msgs::msg::StorageRequest>()
{
  return "astribot_msgs/msg/StorageRequest";
}

template<>
struct has_fixed_size<astribot_msgs::msg::StorageRequest>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::msg::StorageRequest>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::msg::StorageRequest>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__TRAITS_HPP_
