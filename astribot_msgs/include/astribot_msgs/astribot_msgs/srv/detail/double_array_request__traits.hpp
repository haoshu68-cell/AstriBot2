// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:srv/DoubleArrayRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__TRAITS_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/srv/detail/double_array_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace astribot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DoubleArrayRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
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
  const DoubleArrayRequest_Request & msg,
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

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
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

inline std::string to_yaml(const DoubleArrayRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::srv::DoubleArrayRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::srv::DoubleArrayRequest_Request & msg)
{
  return astribot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::srv::DoubleArrayRequest_Request>()
{
  return "astribot_msgs::srv::DoubleArrayRequest_Request";
}

template<>
inline const char * name<astribot_msgs::srv::DoubleArrayRequest_Request>()
{
  return "astribot_msgs/srv/DoubleArrayRequest_Request";
}

template<>
struct has_fixed_size<astribot_msgs::srv::DoubleArrayRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::srv::DoubleArrayRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::srv::DoubleArrayRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'header'
// already included above
// #include "std_msgs/msg/detail/header__traits.hpp"

namespace astribot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DoubleArrayRequest_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
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
  const DoubleArrayRequest_Response & msg,
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

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
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

inline std::string to_yaml(const DoubleArrayRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::srv::DoubleArrayRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::srv::DoubleArrayRequest_Response & msg)
{
  return astribot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::srv::DoubleArrayRequest_Response>()
{
  return "astribot_msgs::srv::DoubleArrayRequest_Response";
}

template<>
inline const char * name<astribot_msgs::srv::DoubleArrayRequest_Response>()
{
  return "astribot_msgs/srv/DoubleArrayRequest_Response";
}

template<>
struct has_fixed_size<astribot_msgs::srv::DoubleArrayRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::srv::DoubleArrayRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::srv::DoubleArrayRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astribot_msgs::srv::DoubleArrayRequest>()
{
  return "astribot_msgs::srv::DoubleArrayRequest";
}

template<>
inline const char * name<astribot_msgs::srv::DoubleArrayRequest>()
{
  return "astribot_msgs/srv/DoubleArrayRequest";
}

template<>
struct has_fixed_size<astribot_msgs::srv::DoubleArrayRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<astribot_msgs::srv::DoubleArrayRequest_Request>::value &&
    has_fixed_size<astribot_msgs::srv::DoubleArrayRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<astribot_msgs::srv::DoubleArrayRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<astribot_msgs::srv::DoubleArrayRequest_Request>::value &&
    has_bounded_size<astribot_msgs::srv::DoubleArrayRequest_Response>::value
  >
{
};

template<>
struct is_service<astribot_msgs::srv::DoubleArrayRequest>
  : std::true_type
{
};

template<>
struct is_service_request<astribot_msgs::srv::DoubleArrayRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astribot_msgs::srv::DoubleArrayRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__TRAITS_HPP_
