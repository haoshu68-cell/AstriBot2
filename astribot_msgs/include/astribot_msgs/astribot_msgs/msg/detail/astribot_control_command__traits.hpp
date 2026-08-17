// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/astribot_control_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AstribotControlCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: name_list
  {
    if (msg.name_list.size() == 0) {
      out << "name_list: []";
    } else {
      out << "name_list: [";
      size_t pending_items = msg.name_list.size();
      for (auto item : msg.name_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: dofs_list
  {
    if (msg.dofs_list.size() == 0) {
      out << "dofs_list: []";
    } else {
      out << "dofs_list: [";
      size_t pending_items = msg.dofs_list.size();
      for (auto item : msg.dofs_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: command_list
  {
    if (msg.command_list.size() == 0) {
      out << "command_list: []";
    } else {
      out << "command_list: [";
      size_t pending_items = msg.command_list.size();
      for (auto item : msg.command_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: control_way
  {
    out << "control_way: ";
    rosidl_generator_traits::value_to_yaml(msg.control_way, out);
    out << ", ";
  }

  // member: frame
  {
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << ", ";
  }

  // member: use_wbc
  {
    out << "use_wbc: ";
    rosidl_generator_traits::value_to_yaml(msg.use_wbc, out);
    out << ", ";
  }

  // member: add_default_torso
  {
    out << "add_default_torso: ";
    rosidl_generator_traits::value_to_yaml(msg.add_default_torso, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AstribotControlCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.name_list.size() == 0) {
      out << "name_list: []\n";
    } else {
      out << "name_list:\n";
      for (auto item : msg.name_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: dofs_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dofs_list.size() == 0) {
      out << "dofs_list: []\n";
    } else {
      out << "dofs_list:\n";
      for (auto item : msg.dofs_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: command_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.command_list.size() == 0) {
      out << "command_list: []\n";
    } else {
      out << "command_list:\n";
      for (auto item : msg.command_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: control_way
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_way: ";
    rosidl_generator_traits::value_to_yaml(msg.control_way, out);
    out << "\n";
  }

  // member: frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << "\n";
  }

  // member: use_wbc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_wbc: ";
    rosidl_generator_traits::value_to_yaml(msg.use_wbc, out);
    out << "\n";
  }

  // member: add_default_torso
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "add_default_torso: ";
    rosidl_generator_traits::value_to_yaml(msg.add_default_torso, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AstribotControlCommand & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::AstribotControlCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::AstribotControlCommand & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::AstribotControlCommand>()
{
  return "astribot_msgs::msg::AstribotControlCommand";
}

template<>
inline const char * name<astribot_msgs::msg::AstribotControlCommand>()
{
  return "astribot_msgs/msg/AstribotControlCommand";
}

template<>
struct has_fixed_size<astribot_msgs::msg::AstribotControlCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::msg::AstribotControlCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::msg::AstribotControlCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__TRAITS_HPP_
