// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/AstribotHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/astribot_heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "astribot_msgs/msg/detail/astribot_header__traits.hpp"
// Member 'node_name'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AstribotHeartbeat & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: node_name
  {
    out << "node_name: ";
    to_flow_style_yaml(msg.node_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AstribotHeartbeat & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: node_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_name:\n";
    to_block_style_yaml(msg.node_name, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AstribotHeartbeat & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::AstribotHeartbeat & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::AstribotHeartbeat & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::AstribotHeartbeat>()
{
  return "astribot_msgs::msg::AstribotHeartbeat";
}

template<>
inline const char * name<astribot_msgs::msg::AstribotHeartbeat>()
{
  return "astribot_msgs/msg/AstribotHeartbeat";
}

template<>
struct has_fixed_size<astribot_msgs::msg::AstribotHeartbeat>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::msg::AstribotHeader>::value && has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<astribot_msgs::msg::AstribotHeartbeat>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::msg::AstribotHeader>::value && has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<astribot_msgs::msg::AstribotHeartbeat>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__TRAITS_HPP_
