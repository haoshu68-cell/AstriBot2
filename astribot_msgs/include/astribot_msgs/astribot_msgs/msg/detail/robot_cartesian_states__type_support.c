// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/RobotCartesianStates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/robot_cartesian_states__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/robot_cartesian_states__functions.h"
#include "astribot_msgs/msg/detail/robot_cartesian_states__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `names`
#include "rosidl_runtime_c/string_functions.h"
// Member `states`
#include "astribot_msgs/msg/robot_cartesian_state.h"
// Member `states`
#include "astribot_msgs/msg/detail/robot_cartesian_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__RobotCartesianStates__init(message_memory);
}

void astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_fini_function(void * message_memory)
{
  astribot_msgs__msg__RobotCartesianStates__fini(message_memory);
}

size_t astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__size_function__RobotCartesianStates__names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_const_function__RobotCartesianStates__names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_function__RobotCartesianStates__names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__fetch_function__RobotCartesianStates__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_const_function__RobotCartesianStates__names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__assign_function__RobotCartesianStates__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_function__RobotCartesianStates__names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__resize_function__RobotCartesianStates__names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__size_function__RobotCartesianStates__states(
  const void * untyped_member)
{
  const astribot_msgs__msg__RobotCartesianState__Sequence * member =
    (const astribot_msgs__msg__RobotCartesianState__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_const_function__RobotCartesianStates__states(
  const void * untyped_member, size_t index)
{
  const astribot_msgs__msg__RobotCartesianState__Sequence * member =
    (const astribot_msgs__msg__RobotCartesianState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_function__RobotCartesianStates__states(
  void * untyped_member, size_t index)
{
  astribot_msgs__msg__RobotCartesianState__Sequence * member =
    (astribot_msgs__msg__RobotCartesianState__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__fetch_function__RobotCartesianStates__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const astribot_msgs__msg__RobotCartesianState * item =
    ((const astribot_msgs__msg__RobotCartesianState *)
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_const_function__RobotCartesianStates__states(untyped_member, index));
  astribot_msgs__msg__RobotCartesianState * value =
    (astribot_msgs__msg__RobotCartesianState *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__assign_function__RobotCartesianStates__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  astribot_msgs__msg__RobotCartesianState * item =
    ((astribot_msgs__msg__RobotCartesianState *)
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_function__RobotCartesianStates__states(untyped_member, index));
  const astribot_msgs__msg__RobotCartesianState * value =
    (const astribot_msgs__msg__RobotCartesianState *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__resize_function__RobotCartesianStates__states(
  void * untyped_member, size_t size)
{
  astribot_msgs__msg__RobotCartesianState__Sequence * member =
    (astribot_msgs__msg__RobotCartesianState__Sequence *)(untyped_member);
  astribot_msgs__msg__RobotCartesianState__Sequence__fini(member);
  return astribot_msgs__msg__RobotCartesianState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianStates, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianStates, names),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__size_function__RobotCartesianStates__names,  // size() function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_const_function__RobotCartesianStates__names,  // get_const(index) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_function__RobotCartesianStates__names,  // get(index) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__fetch_function__RobotCartesianStates__names,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__assign_function__RobotCartesianStates__names,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__resize_function__RobotCartesianStates__names  // resize(index) function pointer
  },
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianStates, states),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__size_function__RobotCartesianStates__states,  // size() function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_const_function__RobotCartesianStates__states,  // get_const(index) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__get_function__RobotCartesianStates__states,  // get(index) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__fetch_function__RobotCartesianStates__states,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__assign_function__RobotCartesianStates__states,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__resize_function__RobotCartesianStates__states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_members = {
  "astribot_msgs__msg",  // message namespace
  "RobotCartesianStates",  // message name
  3,  // number of fields
  sizeof(astribot_msgs__msg__RobotCartesianStates),
  astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_member_array,  // message members
  astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_type_support_handle = {
  0,
  &astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, RobotCartesianStates)() {
  astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, RobotCartesianState)();
  if (!astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__RobotCartesianStates__rosidl_typesupport_introspection_c__RobotCartesianStates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
