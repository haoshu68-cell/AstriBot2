// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/SlaveMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/slave_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "astribot_msgs/msg/detail/astribot_header__traits.hpp"
// Member 'slave_motor_command'
#include "astribot_msgs/msg/detail/motor_command__traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SlaveMotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: slave_motor_command
  {
    if (msg.slave_motor_command.size() == 0) {
      out << "slave_motor_command: []";
    } else {
      out << "slave_motor_command: [";
      size_t pending_items = msg.slave_motor_command.size();
      for (auto item : msg.slave_motor_command) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SlaveMotorCommand & msg,
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

  // member: slave_motor_command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.slave_motor_command.size() == 0) {
      out << "slave_motor_command: []\n";
    } else {
      out << "slave_motor_command:\n";
      for (auto item : msg.slave_motor_command) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SlaveMotorCommand & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::SlaveMotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::SlaveMotorCommand & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::SlaveMotorCommand>()
{
  return "astribot_msgs::msg::SlaveMotorCommand";
}

template<>
inline const char * name<astribot_msgs::msg::SlaveMotorCommand>()
{
  return "astribot_msgs/msg/SlaveMotorCommand";
}

template<>
struct has_fixed_size<astribot_msgs::msg::SlaveMotorCommand>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::msg::AstribotHeader>::value && has_fixed_size<astribot_msgs::msg::MotorCommand>::value> {};

template<>
struct has_bounded_size<astribot_msgs::msg::SlaveMotorCommand>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::msg::AstribotHeader>::value && has_bounded_size<astribot_msgs::msg::MotorCommand>::value> {};

template<>
struct is_message<astribot_msgs::msg::SlaveMotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__TRAITS_HPP_
