// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__CUSTOM_POINT__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__CUSTOM_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/custom_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CustomPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: offset_time
  {
    out << "offset_time: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_time, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: reflectivity
  {
    out << "reflectivity: ";
    rosidl_generator_traits::value_to_yaml(msg.reflectivity, out);
    out << ", ";
  }

  // member: tag
  {
    out << "tag: ";
    rosidl_generator_traits::value_to_yaml(msg.tag, out);
    out << ", ";
  }

  // member: line
  {
    out << "line: ";
    rosidl_generator_traits::value_to_yaml(msg.line, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CustomPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: offset_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_time: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_time, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: reflectivity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reflectivity: ";
    rosidl_generator_traits::value_to_yaml(msg.reflectivity, out);
    out << "\n";
  }

  // member: tag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag: ";
    rosidl_generator_traits::value_to_yaml(msg.tag, out);
    out << "\n";
  }

  // member: line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "line: ";
    rosidl_generator_traits::value_to_yaml(msg.line, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CustomPoint & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::CustomPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::CustomPoint & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::CustomPoint>()
{
  return "astribot_msgs::msg::CustomPoint";
}

template<>
inline const char * name<astribot_msgs::msg::CustomPoint>()
{
  return "astribot_msgs/msg/CustomPoint";
}

template<>
struct has_fixed_size<astribot_msgs::msg::CustomPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astribot_msgs::msg::CustomPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astribot_msgs::msg::CustomPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__CUSTOM_POINT__TRAITS_HPP_
