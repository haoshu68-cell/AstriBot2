// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/encoder_collector_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace astribot_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EncoderCollectorState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::EncoderCollectorState(_init);
}

void EncoderCollectorState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::EncoderCollectorState *>(message_memory);
  typed_message->~EncoderCollectorState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EncoderCollectorState_message_member_array[4] = {
  {
    "position_rad",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::EncoderCollectorState, position_rad),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity_rps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::EncoderCollectorState, velocity_rps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acceleration_rpss",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::EncoderCollectorState, acceleration_rpss),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rx_sequence_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::EncoderCollectorState, rx_sequence_count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EncoderCollectorState_message_members = {
  "astribot_msgs::msg",  // message namespace
  "EncoderCollectorState",  // message name
  4,  // number of fields
  sizeof(astribot_msgs::msg::EncoderCollectorState),
  EncoderCollectorState_message_member_array,  // message members
  EncoderCollectorState_init_function,  // function to initialize message memory (memory has to be allocated)
  EncoderCollectorState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EncoderCollectorState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EncoderCollectorState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace astribot_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<astribot_msgs::msg::EncoderCollectorState>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::EncoderCollectorState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, EncoderCollectorState)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::EncoderCollectorState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
