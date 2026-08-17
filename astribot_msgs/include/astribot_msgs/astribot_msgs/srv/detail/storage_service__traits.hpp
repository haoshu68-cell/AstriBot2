// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:srv/StorageService.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__TRAITS_HPP_
#define ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/srv/detail/storage_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'request'
#include "astribot_msgs/msg/detail/storage_request__traits.hpp"

namespace astribot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const StorageService_Request & msg,
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
  const StorageService_Request & msg,
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

inline std::string to_yaml(const StorageService_Request & msg, bool use_flow_style = false)
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
  const astribot_msgs::srv::StorageService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::srv::StorageService_Request & msg)
{
  return astribot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::srv::StorageService_Request>()
{
  return "astribot_msgs::srv::StorageService_Request";
}

template<>
inline const char * name<astribot_msgs::srv::StorageService_Request>()
{
  return "astribot_msgs/srv/StorageService_Request";
}

template<>
struct has_fixed_size<astribot_msgs::srv::StorageService_Request>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::msg::StorageRequest>::value> {};

template<>
struct has_bounded_size<astribot_msgs::srv::StorageService_Request>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::msg::StorageRequest>::value> {};

template<>
struct is_message<astribot_msgs::srv::StorageService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'response'
#include "astribot_msgs/msg/detail/storage_reponse__traits.hpp"

namespace astribot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const StorageService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    to_flow_style_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StorageService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response:\n";
    to_block_style_yaml(msg.response, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StorageService_Response & msg, bool use_flow_style = false)
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
  const astribot_msgs::srv::StorageService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::srv::StorageService_Response & msg)
{
  return astribot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::srv::StorageService_Response>()
{
  return "astribot_msgs::srv::StorageService_Response";
}

template<>
inline const char * name<astribot_msgs::srv::StorageService_Response>()
{
  return "astribot_msgs/srv/StorageService_Response";
}

template<>
struct has_fixed_size<astribot_msgs::srv::StorageService_Response>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::msg::StorageReponse>::value> {};

template<>
struct has_bounded_size<astribot_msgs::srv::StorageService_Response>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::msg::StorageReponse>::value> {};

template<>
struct is_message<astribot_msgs::srv::StorageService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astribot_msgs::srv::StorageService>()
{
  return "astribot_msgs::srv::StorageService";
}

template<>
inline const char * name<astribot_msgs::srv::StorageService>()
{
  return "astribot_msgs/srv/StorageService";
}

template<>
struct has_fixed_size<astribot_msgs::srv::StorageService>
  : std::integral_constant<
    bool,
    has_fixed_size<astribot_msgs::srv::StorageService_Request>::value &&
    has_fixed_size<astribot_msgs::srv::StorageService_Response>::value
  >
{
};

template<>
struct has_bounded_size<astribot_msgs::srv::StorageService>
  : std::integral_constant<
    bool,
    has_bounded_size<astribot_msgs::srv::StorageService_Request>::value &&
    has_bounded_size<astribot_msgs::srv::StorageService_Response>::value
  >
{
};

template<>
struct is_service<astribot_msgs::srv::StorageService>
  : std::true_type
{
};

template<>
struct is_service_request<astribot_msgs::srv::StorageService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astribot_msgs::srv::StorageService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__TRAITS_HPP_
