// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:srv/LaunchRequestSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__TRAITS_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/srv/detail/launch_request_srv__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'request'
#include "astribot_msgs/msg/detail/launch_request__traits.hpp"

namespace astribot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const LaunchRequestSrv_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    to_flow_style_yaml(msg.request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchRequestSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request:\n";
    to_block_style_yaml(msg.request, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchRequestSrv_Request & msg, bool use_flow_style = false)
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
  const astribot_msgs::srv::LaunchRequestSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::srv::LaunchRequestSrv_Request & msg)
{
  return astribot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::srv::LaunchRequestSrv_Request>()
{
  return "astribot_msgs::srv::LaunchRequestSrv_Request";
}

template<>
inline const char * name<astribot_msgs::srv::LaunchRequestSrv_Request>()
{
  return "astribot_msgs/srv/LaunchRequestSrv_Request";
}

template<>
struct has_fixed_size<astribot_msgs::srv::LaunchRequestSrv_Request>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::msg::LaunchRequest>::value> {};

template<>
struct has_bounded_size<astribot_msgs::srv::LaunchRequestSrv_Request>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::msg::LaunchRequest>::value> {};

template<>
struct is_message<astribot_msgs::srv::LaunchRequestSrv_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace astribot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const LaunchRequestSrv_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchRequestSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchRequestSrv_Response & msg, bool use_flow_style = false)
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
  const astribot_msgs::srv::LaunchRequestSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::srv::LaunchRequestSrv_Response & msg)
{
  return astribot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::srv::LaunchRequestSrv_Response>()
{
  return "astribot_msgs::srv::LaunchRequestSrv_Response";
}

template<>
inline const char * name<astribot_msgs::srv::LaunchRequestSrv_Response>()
{
  return "astribot_msgs/srv/LaunchRequestSrv_Response";
}

template<>
struct has_fixed_size<astribot_msgs::srv::LaunchRequestSrv_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::srv::LaunchRequestSrv_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::srv::LaunchRequestSrv_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astribot_msgs::srv::LaunchRequestSrv>()
{
  return "astribot_msgs::srv::LaunchRequestSrv";
}

template<>
inline const char * name<astribot_msgs::srv::LaunchRequestSrv>()
{
  return "astribot_msgs/srv/LaunchRequestSrv";
}

template<>
struct has_fixed_size<astribot_msgs::srv::LaunchRequestSrv>
  : std::integral_constant<
    bool,
    has_fixed_size<astribot_msgs::srv::LaunchRequestSrv_Request>::value &&
    has_fixed_size<astribot_msgs::srv::LaunchRequestSrv_Response>::value
  >
{
};

template<>
struct has_bounded_size<astribot_msgs::srv::LaunchRequestSrv>
  : std::integral_constant<
    bool,
    has_bounded_size<astribot_msgs::srv::LaunchRequestSrv_Request>::value &&
    has_bounded_size<astribot_msgs::srv::LaunchRequestSrv_Response>::value
  >
{
};

template<>
struct is_service<astribot_msgs::srv::LaunchRequestSrv>
  : std::true_type
{
};

template<>
struct is_service_request<astribot_msgs::srv::LaunchRequestSrv_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astribot_msgs::srv::LaunchRequestSrv_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__TRAITS_HPP_
