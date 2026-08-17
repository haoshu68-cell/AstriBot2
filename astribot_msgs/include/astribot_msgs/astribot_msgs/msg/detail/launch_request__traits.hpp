// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/LaunchRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/launch_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LaunchRequest & msg,
  std::ostream & out)
{
  out << "{";
  // member: launch_target
  {
    out << "launch_target: ";
    rosidl_generator_traits::value_to_yaml(msg.launch_target, out);
    out << ", ";
  }

  // member: launch_action
  {
    out << "launch_action: ";
    rosidl_generator_traits::value_to_yaml(msg.launch_action, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: launch_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "launch_target: ";
    rosidl_generator_traits::value_to_yaml(msg.launch_target, out);
    out << "\n";
  }

  // member: launch_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "launch_action: ";
    rosidl_generator_traits::value_to_yaml(msg.launch_action, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchRequest & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::LaunchRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::LaunchRequest & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::LaunchRequest>()
{
  return "astribot_msgs::msg::LaunchRequest";
}

template<>
inline const char * name<astribot_msgs::msg::LaunchRequest>()
{
  return "astribot_msgs/msg/LaunchRequest";
}

template<>
struct has_fixed_size<astribot_msgs::msg::LaunchRequest>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astribot_msgs::msg::LaunchRequest>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astribot_msgs::msg::LaunchRequest>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__TRAITS_HPP_
