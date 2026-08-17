// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/encoder_collector_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderCollectorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: position_rad
  {
    out << "position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.position_rad, out);
    out << ", ";
  }

  // member: velocity_rps
  {
    out << "velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_rps, out);
    out << ", ";
  }

  // member: acceleration_rpss
  {
    out << "acceleration_rpss: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_rpss, out);
    out << ", ";
  }

  // member: rx_sequence_count
  {
    out << "rx_sequence_count: ";
    rosidl_generator_traits::value_to_yaml(msg.rx_sequence_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderCollectorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.position_rad, out);
    out << "\n";
  }

  // member: velocity_rps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_rps, out);
    out << "\n";
  }

  // member: acceleration_rpss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_rpss: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_rpss, out);
    out << "\n";
  }

  // member: rx_sequence_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rx_sequence_count: ";
    rosidl_generator_traits::value_to_yaml(msg.rx_sequence_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderCollectorState & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::EncoderCollectorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::EncoderCollectorState & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::EncoderCollectorState>()
{
  return "astribot_msgs::msg::EncoderCollectorState";
}

template<>
inline const char * name<astribot_msgs::msg::EncoderCollectorState>()
{
  return "astribot_msgs/msg/EncoderCollectorState";
}

template<>
struct has_fixed_size<astribot_msgs::msg::EncoderCollectorState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astribot_msgs::msg::EncoderCollectorState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astribot_msgs::msg::EncoderCollectorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__TRAITS_HPP_
