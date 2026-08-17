// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/LivoxCustomMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/livox_custom_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'points'
#include "astribot_msgs/msg/detail/custom_point__traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LivoxCustomMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: timebase
  {
    out << "timebase: ";
    rosidl_generator_traits::value_to_yaml(msg.timebase, out);
    out << ", ";
  }

  // member: point_num
  {
    out << "point_num: ";
    rosidl_generator_traits::value_to_yaml(msg.point_num, out);
    out << ", ";
  }

  // member: lidar_id
  {
    out << "lidar_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar_id, out);
    out << ", ";
  }

  // member: rsvd
  {
    if (msg.rsvd.size() == 0) {
      out << "rsvd: []";
    } else {
      out << "rsvd: [";
      size_t pending_items = msg.rsvd.size();
      for (auto item : msg.rsvd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
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
  const LivoxCustomMsg & msg,
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

  // member: timebase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timebase: ";
    rosidl_generator_traits::value_to_yaml(msg.timebase, out);
    out << "\n";
  }

  // member: point_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_num: ";
    rosidl_generator_traits::value_to_yaml(msg.point_num, out);
    out << "\n";
  }

  // member: lidar_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lidar_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar_id, out);
    out << "\n";
  }

  // member: rsvd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rsvd.size() == 0) {
      out << "rsvd: []\n";
    } else {
      out << "rsvd:\n";
      for (auto item : msg.rsvd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LivoxCustomMsg & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::LivoxCustomMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::LivoxCustomMsg & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::LivoxCustomMsg>()
{
  return "astribot_msgs::msg::LivoxCustomMsg";
}

template<>
inline const char * name<astribot_msgs::msg::LivoxCustomMsg>()
{
  return "astribot_msgs/msg/LivoxCustomMsg";
}

template<>
struct has_fixed_size<astribot_msgs::msg::LivoxCustomMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::msg::LivoxCustomMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::msg::LivoxCustomMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__TRAITS_HPP_
