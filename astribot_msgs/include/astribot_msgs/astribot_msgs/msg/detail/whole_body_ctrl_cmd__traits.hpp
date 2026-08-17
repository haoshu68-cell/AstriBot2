// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "astribot_msgs/msg/detail/astribot_header__traits.hpp"
// Member 'left_arm_twist'
// Member 'right_arm_twist'
// Member 'torso_twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WholeBodyCtrlCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: left_arm_twist
  {
    out << "left_arm_twist: ";
    to_flow_style_yaml(msg.left_arm_twist, out);
    out << ", ";
  }

  // member: right_arm_twist
  {
    out << "right_arm_twist: ";
    to_flow_style_yaml(msg.right_arm_twist, out);
    out << ", ";
  }

  // member: torso_twist
  {
    out << "torso_twist: ";
    to_flow_style_yaml(msg.torso_twist, out);
    out << ", ";
  }

  // member: pose_world_to_torso_desired
  {
    if (msg.pose_world_to_torso_desired.size() == 0) {
      out << "pose_world_to_torso_desired: []";
    } else {
      out << "pose_world_to_torso_desired: [";
      size_t pending_items = msg.pose_world_to_torso_desired.size();
      for (auto item : msg.pose_world_to_torso_desired) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pose_world_to_left_arm_desired
  {
    if (msg.pose_world_to_left_arm_desired.size() == 0) {
      out << "pose_world_to_left_arm_desired: []";
    } else {
      out << "pose_world_to_left_arm_desired: [";
      size_t pending_items = msg.pose_world_to_left_arm_desired.size();
      for (auto item : msg.pose_world_to_left_arm_desired) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pose_world_to_right_arm_desired
  {
    if (msg.pose_world_to_right_arm_desired.size() == 0) {
      out << "pose_world_to_right_arm_desired: []";
    } else {
      out << "pose_world_to_right_arm_desired: [";
      size_t pending_items = msg.pose_world_to_right_arm_desired.size();
      for (auto item : msg.pose_world_to_right_arm_desired) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pose_world_to_chassis_current
  {
    if (msg.pose_world_to_chassis_current.size() == 0) {
      out << "pose_world_to_chassis_current: []";
    } else {
      out << "pose_world_to_chassis_current: [";
      size_t pending_items = msg.pose_world_to_chassis_current.size();
      for (auto item : msg.pose_world_to_chassis_current) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: torso_open_loop
  {
    out << "torso_open_loop: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_open_loop, out);
    out << ", ";
  }

  // member: enable_collision_avoidance
  {
    out << "enable_collision_avoidance: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_collision_avoidance, out);
    out << ", ";
  }

  // member: smooth_t
  {
    out << "smooth_t: ";
    rosidl_generator_traits::value_to_yaml(msg.smooth_t, out);
    out << ", ";
  }

  // member: smooth_duration
  {
    out << "smooth_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.smooth_duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WholeBodyCtrlCmd & msg,
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

  // member: left_arm_twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_arm_twist:\n";
    to_block_style_yaml(msg.left_arm_twist, out, indentation + 2);
  }

  // member: right_arm_twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_arm_twist:\n";
    to_block_style_yaml(msg.right_arm_twist, out, indentation + 2);
  }

  // member: torso_twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torso_twist:\n";
    to_block_style_yaml(msg.torso_twist, out, indentation + 2);
  }

  // member: pose_world_to_torso_desired
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose_world_to_torso_desired.size() == 0) {
      out << "pose_world_to_torso_desired: []\n";
    } else {
      out << "pose_world_to_torso_desired:\n";
      for (auto item : msg.pose_world_to_torso_desired) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pose_world_to_left_arm_desired
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose_world_to_left_arm_desired.size() == 0) {
      out << "pose_world_to_left_arm_desired: []\n";
    } else {
      out << "pose_world_to_left_arm_desired:\n";
      for (auto item : msg.pose_world_to_left_arm_desired) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pose_world_to_right_arm_desired
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose_world_to_right_arm_desired.size() == 0) {
      out << "pose_world_to_right_arm_desired: []\n";
    } else {
      out << "pose_world_to_right_arm_desired:\n";
      for (auto item : msg.pose_world_to_right_arm_desired) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pose_world_to_chassis_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose_world_to_chassis_current.size() == 0) {
      out << "pose_world_to_chassis_current: []\n";
    } else {
      out << "pose_world_to_chassis_current:\n";
      for (auto item : msg.pose_world_to_chassis_current) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: torso_open_loop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torso_open_loop: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_open_loop, out);
    out << "\n";
  }

  // member: enable_collision_avoidance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_collision_avoidance: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_collision_avoidance, out);
    out << "\n";
  }

  // member: smooth_t
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "smooth_t: ";
    rosidl_generator_traits::value_to_yaml(msg.smooth_t, out);
    out << "\n";
  }

  // member: smooth_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "smooth_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.smooth_duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WholeBodyCtrlCmd & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::WholeBodyCtrlCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::WholeBodyCtrlCmd & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::WholeBodyCtrlCmd>()
{
  return "astribot_msgs::msg::WholeBodyCtrlCmd";
}

template<>
inline const char * name<astribot_msgs::msg::WholeBodyCtrlCmd>()
{
  return "astribot_msgs/msg/WholeBodyCtrlCmd";
}

template<>
struct has_fixed_size<astribot_msgs::msg::WholeBodyCtrlCmd>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::msg::WholeBodyCtrlCmd>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::msg::WholeBodyCtrlCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__TRAITS_HPP_
