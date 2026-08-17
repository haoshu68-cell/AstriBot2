// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorState & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: c
  {
    if (msg.c.size() == 0) {
      out << "c: []";
    } else {
      out << "c: [";
      size_t pending_items = msg.c.size();
      for (auto item : msg.c) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vol
  {
    if (msg.vol.size() == 0) {
      out << "vol: []";
    } else {
      out << "vol: [";
      size_t pending_items = msg.vol.size();
      for (auto item : msg.vol) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: acc
  {
    if (msg.acc.size() == 0) {
      out << "acc: []";
    } else {
      out << "acc: [";
      size_t pending_items = msg.acc.size();
      for (auto item : msg.acc) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: can_count
  {
    if (msg.can_count.size() == 0) {
      out << "can_count: []";
    } else {
      out << "can_count: [";
      size_t pending_items = msg.can_count.size();
      for (auto item : msg.can_count) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: can_count_last
  {
    if (msg.can_count_last.size() == 0) {
      out << "can_count_last: []";
    } else {
      out << "can_count_last: [";
      size_t pending_items = msg.can_count_last.size();
      for (auto item : msg.can_count_last) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: can_error
  {
    if (msg.can_error.size() == 0) {
      out << "can_error: []";
    } else {
      out << "can_error: [";
      size_t pending_items = msg.can_error.size();
      for (auto item : msg.can_error) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: j_s
  {
    if (msg.j_s.size() == 0) {
      out << "j_s: []";
    } else {
      out << "j_s: [";
      size_t pending_items = msg.j_s.size();
      for (auto item : msg.j_s) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: j_p
  {
    if (msg.j_p.size() == 0) {
      out << "j_p: []";
    } else {
      out << "j_p: [";
      size_t pending_items = msg.j_p.size();
      for (auto item : msg.j_p) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: j_v
  {
    if (msg.j_v.size() == 0) {
      out << "j_v: []";
    } else {
      out << "j_v: [";
      size_t pending_items = msg.j_v.size();
      for (auto item : msg.j_v) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: j_a
  {
    if (msg.j_a.size() == 0) {
      out << "j_a: []";
    } else {
      out << "j_a: [";
      size_t pending_items = msg.j_a.size();
      for (auto item : msg.j_a) {
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
  const MotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.c.size() == 0) {
      out << "c: []\n";
    } else {
      out << "c:\n";
      for (auto item : msg.c) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vol
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vol.size() == 0) {
      out << "vol: []\n";
    } else {
      out << "vol:\n";
      for (auto item : msg.vol) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.acc.size() == 0) {
      out << "acc: []\n";
    } else {
      out << "acc:\n";
      for (auto item : msg.acc) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: can_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.can_count.size() == 0) {
      out << "can_count: []\n";
    } else {
      out << "can_count:\n";
      for (auto item : msg.can_count) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: can_count_last
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.can_count_last.size() == 0) {
      out << "can_count_last: []\n";
    } else {
      out << "can_count_last:\n";
      for (auto item : msg.can_count_last) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: can_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.can_error.size() == 0) {
      out << "can_error: []\n";
    } else {
      out << "can_error:\n";
      for (auto item : msg.can_error) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: j_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.j_s.size() == 0) {
      out << "j_s: []\n";
    } else {
      out << "j_s:\n";
      for (auto item : msg.j_s) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: j_p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.j_p.size() == 0) {
      out << "j_p: []\n";
    } else {
      out << "j_p:\n";
      for (auto item : msg.j_p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: j_v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.j_v.size() == 0) {
      out << "j_v: []\n";
    } else {
      out << "j_v:\n";
      for (auto item : msg.j_v) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: j_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.j_a.size() == 0) {
      out << "j_a: []\n";
    } else {
      out << "j_a:\n";
      for (auto item : msg.j_a) {
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

inline std::string to_yaml(const MotorState & msg, bool use_flow_style = false)
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
  const astribot_msgs::msg::MotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::msg::MotorState & msg)
{
  return astribot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::msg::MotorState>()
{
  return "astribot_msgs::msg::MotorState";
}

template<>
inline const char * name<astribot_msgs::msg::MotorState>()
{
  return "astribot_msgs/msg/MotorState";
}

template<>
struct has_fixed_size<astribot_msgs::msg::MotorState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astribot_msgs::msg::MotorState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astribot_msgs::msg::MotorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_
