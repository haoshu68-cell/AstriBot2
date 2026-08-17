// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: kp
  {
    if (msg.kp.size() == 0) {
      out << "kp: []";
    } else {
      out << "kp: [";
      size_t pending_items = msg.kp.size();
      for (auto item : msg.kp) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: kd
  {
    if (msg.kd.size() == 0) {
      out << "kd: []";
    } else {
      out << "kd: [";
      size_t pending_items = msg.kd.size();
      for (auto item : msg.kd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: p
  {
    if (msg.p.size() == 0) {
      out << "p: []";
    } else {
      out << "p: [";
      size_t pending_items = msg.p.size();
      for (auto item : msg.p) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: v
  {
    if (msg.v.size() == 0) {
      out << "v: []";
    } else {
      out << "v: [";
      size_t pending_items = msg.v.size();
      for (auto item : msg.v) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: t_ff
  {
    if (msg.t_ff.size() == 0) {
      out << "t_ff: []";
    } else {
      out << "t_ff: [";
      size_t pending_items = msg.t_ff.size();
      for (auto item : msg.t_ff) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: t_limit
  {
    if (msg.t_limit.size() == 0) {
      out << "t_limit: []";
    } else {
      out << "t_limit: [";
      size_t pending_items = msg.t_limit.size();
      for (auto item : msg.t_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: break_relase
  {
    if (msg.break_relase.size() == 0) {
      out << "break_relase: []";
    } else {
      out << "break_relase: [";
      size_t pending_items = msg.break_relase.size();
      for (auto item : msg.break_relase) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: kd_slave
  {
    if (msg.kd_slave.size() == 0) {
      out << "kd_slave: []";
    } else {
      out << "kd_slave: [";
      size_t pending_items = msg.kd_slave.size();
      for (auto item : msg.kd_slave) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel_slave
  {
    if (msg.vel_slave.size() == 0) {
      out << "vel_slave: []";
    } else {
      out << "vel_slave: [";
      size_t pending_items = msg.vel_slave.size();
      for (auto item : msg.vel_slave) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: motor_id_list
  {
    if (msg.motor_id_list.size() == 0) {
      out << "motor_id_list: []";
    } else {
      out << "motor_id_list: [";
      size_t pending_items = msg.motor_id_list.size();
      for (auto item : msg.motor_id_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.kp.size() == 0) {
      out << "kp: []\n";
    } else {
      out << "kp:\n";
      for (auto item : msg.kp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.kd.size() == 0) {
      out << "kd: []\n";
    } else {
      out << "kd:\n";
      for (auto item : msg.kd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.p.size() == 0) {
      out << "p: []\n";
    } else {
      out << "p:\n";
      for (auto item : msg.p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.v.size() == 0) {
      out << "v: []\n";
    } else {
      out << "v:\n";
      for (auto item : msg.v) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: t_ff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.t_ff.size() == 0) {
      out << "t_ff: []\n";
    } else {
      out << "t_ff:\n";
      for (auto item : msg.t_ff) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: t_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.t_limit.size() == 0) {
      out << "t_limit: []\n";
    } else {
      out << "t_limit:\n";
      for (auto item : msg.t_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: break_relase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.break_relase.size() == 0) {
      out << "break_relase: []\n";
    } else {
      out << "break_relase:\n";
      for (auto item : msg.break_relase) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: kd_slave
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.kd_slave.size() == 0) {
      out << "kd_slave: []\n";
    } else {
      out << "kd_slave:\n";
      for (auto item : msg.kd_slave) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_slave
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_slave.size() == 0) {
      out << "vel_slave: []\n";
    } else {
      out << "vel_slave:\n";
      for (auto item : msg.vel_slave) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: motor_id_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_id_list.size() == 0) {
      out << "motor_id_list: []\n";
    } else {
      out << "motor_id_list:\n";
      for (auto item : msg.motor_id_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorCommand & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::MotorCommand & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::MotorCommand>()
{
  return "astribot_msgs::msg::MotorCommand";
}

template<>
inline const char * name<astribot_msgs::msg::MotorCommand>()
{
  return "astribot_msgs/msg/MotorCommand";
}

template<>
struct has_fixed_size<astribot_msgs::msg::MotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::msg::MotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::msg::MotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
