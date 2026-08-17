// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astribot_msgs:action/Storage.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__TRAITS_HPP_
#define ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astribot_msgs/action/detail/storage__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_list
  {
    out << "topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_list, out);
    out << ", ";
  }

  // member: storage_type
  {
    out << "storage_type: ";
    rosidl_generator_traits::value_to_yaml(msg.storage_type, out);
    out << ", ";
  }

  // member: trigger_time
  {
    out << "trigger_time: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_time, out);
    out << ", ";
  }

  // member: archive_path
  {
    out << "archive_path: ";
    rosidl_generator_traits::value_to_yaml(msg.archive_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_list, out);
    out << "\n";
  }

  // member: storage_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "storage_type: ";
    rosidl_generator_traits::value_to_yaml(msg.storage_type, out);
    out << "\n";
  }

  // member: trigger_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_time: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_time, out);
    out << "\n";
  }

  // member: archive_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "archive_path: ";
    rosidl_generator_traits::value_to_yaml(msg.archive_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_Goal & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_Goal>()
{
  return "astribot_msgs::action::Storage_Goal";
}

template<>
inline const char * name<astribot_msgs::action::Storage_Goal>()
{
  return "astribot_msgs/action/Storage_Goal";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::action::Storage_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: dir_path
  {
    out << "dir_path: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_path, out);
    out << ", ";
  }

  // member: miss_topic_list
  {
    out << "miss_topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.miss_topic_list, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dir_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dir_path: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_path, out);
    out << "\n";
  }

  // member: miss_topic_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "miss_topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.miss_topic_list, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_Result & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_Result>()
{
  return "astribot_msgs::action::Storage_Result";
}

template<>
inline const char * name<astribot_msgs::action::Storage_Result>()
{
  return "astribot_msgs/action/Storage_Result";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::action::Storage_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: seconds
  {
    out << "seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.seconds, out);
    out << ", ";
  }

  // member: disk_space
  {
    out << "disk_space: ";
    rosidl_generator_traits::value_to_yaml(msg.disk_space, out);
    out << ", ";
  }

  // member: miss_topic_list
  {
    out << "miss_topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.miss_topic_list, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.seconds, out);
    out << "\n";
  }

  // member: disk_space
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "disk_space: ";
    rosidl_generator_traits::value_to_yaml(msg.disk_space, out);
    out << "\n";
  }

  // member: miss_topic_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "miss_topic_list: ";
    rosidl_generator_traits::value_to_yaml(msg.miss_topic_list, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_Feedback & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_Feedback>()
{
  return "astribot_msgs::action::Storage_Feedback";
}

template<>
inline const char * name<astribot_msgs::action::Storage_Feedback>()
{
  return "astribot_msgs/action/Storage_Feedback";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astribot_msgs::action::Storage_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "astribot_msgs/action/detail/storage__traits.hpp"

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_SendGoal_Request & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_SendGoal_Request>()
{
  return "astribot_msgs::action::Storage_SendGoal_Request";
}

template<>
inline const char * name<astribot_msgs::action::Storage_SendGoal_Request>()
{
  return "astribot_msgs/action/Storage_SendGoal_Request";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::action::Storage_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::action::Storage_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<astribot_msgs::action::Storage_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_SendGoal_Response & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_SendGoal_Response>()
{
  return "astribot_msgs::action::Storage_SendGoal_Response";
}

template<>
inline const char * name<astribot_msgs::action::Storage_SendGoal_Response>()
{
  return "astribot_msgs/action/Storage_SendGoal_Response";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<astribot_msgs::action::Storage_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astribot_msgs::action::Storage_SendGoal>()
{
  return "astribot_msgs::action::Storage_SendGoal";
}

template<>
inline const char * name<astribot_msgs::action::Storage_SendGoal>()
{
  return "astribot_msgs/action/Storage_SendGoal";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<astribot_msgs::action::Storage_SendGoal_Request>::value &&
    has_fixed_size<astribot_msgs::action::Storage_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<astribot_msgs::action::Storage_SendGoal_Request>::value &&
    has_bounded_size<astribot_msgs::action::Storage_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<astribot_msgs::action::Storage_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<astribot_msgs::action::Storage_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astribot_msgs::action::Storage_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_GetResult_Request & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_GetResult_Request>()
{
  return "astribot_msgs::action::Storage_GetResult_Request";
}

template<>
inline const char * name<astribot_msgs::action::Storage_GetResult_Request>()
{
  return "astribot_msgs/action/Storage_GetResult_Request";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<astribot_msgs::action::Storage_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "astribot_msgs/action/detail/storage__traits.hpp"

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_GetResult_Response & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_GetResult_Response>()
{
  return "astribot_msgs::action::Storage_GetResult_Response";
}

template<>
inline const char * name<astribot_msgs::action::Storage_GetResult_Response>()
{
  return "astribot_msgs/action/Storage_GetResult_Response";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::action::Storage_Result>::value> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::action::Storage_Result>::value> {};

template<>
struct is_message<astribot_msgs::action::Storage_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astribot_msgs::action::Storage_GetResult>()
{
  return "astribot_msgs::action::Storage_GetResult";
}

template<>
inline const char * name<astribot_msgs::action::Storage_GetResult>()
{
  return "astribot_msgs/action/Storage_GetResult";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<astribot_msgs::action::Storage_GetResult_Request>::value &&
    has_fixed_size<astribot_msgs::action::Storage_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<astribot_msgs::action::Storage_GetResult_Request>::value &&
    has_bounded_size<astribot_msgs::action::Storage_GetResult_Response>::value
  >
{
};

template<>
struct is_service<astribot_msgs::action::Storage_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<astribot_msgs::action::Storage_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astribot_msgs::action::Storage_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "astribot_msgs/action/detail/storage__traits.hpp"

namespace astribot_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Storage_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Storage_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Storage_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace astribot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use astribot_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astribot_msgs::action::Storage_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  astribot_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astribot_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const astribot_msgs::action::Storage_FeedbackMessage & msg)
{
  return astribot_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<astribot_msgs::action::Storage_FeedbackMessage>()
{
  return "astribot_msgs::action::Storage_FeedbackMessage";
}

template<>
inline const char * name<astribot_msgs::action::Storage_FeedbackMessage>()
{
  return "astribot_msgs/action/Storage_FeedbackMessage";
}

template<>
struct has_fixed_size<astribot_msgs::action::Storage_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<astribot_msgs::action::Storage_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<astribot_msgs::action::Storage_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<astribot_msgs::action::Storage_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<astribot_msgs::action::Storage_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<astribot_msgs::action::Storage>
  : std::true_type
{
};

template<>
struct is_action_goal<astribot_msgs::action::Storage_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<astribot_msgs::action::Storage_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<astribot_msgs::action::Storage_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__TRAITS_HPP_
