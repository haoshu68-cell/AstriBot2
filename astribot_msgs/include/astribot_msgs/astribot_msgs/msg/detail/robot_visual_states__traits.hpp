// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/RobotVisualStates.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/robot_visual_states__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotVisualStates & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    if (msg.id.size() == 0) {
      out << "id: []";
    } else {
      out << "id: [";
      size_t pending_items = msg.id.size();
      for (auto item : msg.id) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pose
  {
    if (msg.pose.size() == 0) {
      out << "pose: []";
    } else {
      out << "pose: [";
      size_t pending_items = msg.pose.size();
      for (auto item : msg.pose) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: twist
  {
    if (msg.twist.size() == 0) {
      out << "twist: []";
    } else {
      out << "twist: [";
      size_t pending_items = msg.twist.size();
      for (auto item : msg.twist) {
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
  const RobotVisualStates & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id.size() == 0) {
      out << "id: []\n";
    } else {
      out << "id:\n";
      for (auto item : msg.id) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose.size() == 0) {
      out << "pose: []\n";
    } else {
      out << "pose:\n";
      for (auto item : msg.pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.twist.size() == 0) {
      out << "twist: []\n";
    } else {
      out << "twist:\n";
      for (auto item : msg.twist) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotVisualStates & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::RobotVisualStates & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::RobotVisualStates & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::RobotVisualStates>()
{
  return "astribot_msgs::msg::RobotVisualStates";
}

template<>
inline const char * name<astribot_msgs::msg::RobotVisualStates>()
{
  return "astribot_msgs/msg/RobotVisualStates";
}

template<>
struct has_fixed_size<astribot_msgs::msg::RobotVisualStates>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::msg::RobotVisualStates>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::msg::RobotVisualStates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__TRAITS_HPP_
