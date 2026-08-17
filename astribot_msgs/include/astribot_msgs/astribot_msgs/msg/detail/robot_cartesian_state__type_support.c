// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/RobotCartesianState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/robot_cartesian_state__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/robot_cartesian_state__functions.h"
#include "astribot_msgs/msg/detail/robot_cartesian_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `wrench`
#include "geometry_msgs/msg/wrench.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__RobotCartesianState__init(message_memory);
}

void astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_fini_function(void * message_memory)
{
  astribot_msgs__msg__RobotCartesianState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianState, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianState, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotCartesianState, wrench),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_members = {
  "astribot_msgs__msg",  // message namespace
  "RobotCartesianState",  // message name
  4,  // number of fields
  sizeof(astribot_msgs__msg__RobotCartesianState),
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_member_array,  // message members
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_type_support_handle = {
  0,
  &astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, RobotCartesianState)() {
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Wrench)();
  if (!astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__RobotCartesianState__rosidl_typesupport_introspection_c__RobotCartesianState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
