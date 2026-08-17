// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/AstribotHeader.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/astribot_header__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AstribotHeader & msg,
  std::ostream & out)
{
  out << "{";
  // member: seq
  {
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
    out << ", ";
  }

  // member: time_meas
  {
    out << "time_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.time_meas, out);
    out << ", ";
  }

  // member: time_pub
  {
    out << "time_pub: ";
    rosidl_generator_traits::value_to_yaml(msg.time_pub, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AstribotHeader & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
    out << "\n";
  }

  // member: time_meas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.time_meas, out);
    out << "\n";
  }

  // member: time_pub
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_pub: ";
    rosidl_generator_traits::value_to_yaml(msg.time_pub, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AstribotHeader & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::AstribotHeader & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::AstribotHeader & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::AstribotHeader>()
{
  return "astribot_msgs::msg::AstribotHeader";
}

template<>
inline const char * name<astribot_msgs::msg::AstribotHeader>()
{
  return "astribot_msgs/msg/AstribotHeader";
}

template<>
struct has_fixed_size<astribot_msgs::msg::AstribotHeader>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astribot_msgs::msg::AstribotHeader>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astribot_msgs::msg::AstribotHeader>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__TRAITS_HPP_
