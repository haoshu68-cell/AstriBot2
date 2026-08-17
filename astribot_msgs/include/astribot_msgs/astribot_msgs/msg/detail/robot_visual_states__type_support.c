// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/RobotVisualStates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/robot_visual_states__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/robot_visual_states__functions.h"
#include "astribot_msgs/msg/detail/robot_visual_states__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `id`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__RobotVisualStates__init(message_memory);
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_fini_function(void * message_memory)
{
  astribot_msgs__msg__RobotVisualStates__fini(message_memory);
}

size_t astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__size_function__RobotVisualStates__id(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__fetch_function__RobotVisualStates__id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__id(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__assign_function__RobotVisualStates__id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__id(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__resize_function__RobotVisualStates__id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__size_function__RobotVisualStates__pose(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__pose(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__pose(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__fetch_function__RobotVisualStates__pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__pose(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__assign_function__RobotVisualStates__pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__pose(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__resize_function__RobotVisualStates__pose(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__size_function__RobotVisualStates__twist(
  const void * untyped_member)
{
  const geometry_msgs__msg__Twist__Sequence * member =
    (const geometry_msgs__msg__Twist__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__twist(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Twist__Sequence * member =
    (const geometry_msgs__msg__Twist__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__twist(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Twist__Sequence * member =
    (geometry_msgs__msg__Twist__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__fetch_function__RobotVisualStates__twist(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Twist * item =
    ((const geometry_msgs__msg__Twist *)
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__twist(untyped_member, index));
  geometry_msgs__msg__Twist * value =
    (geometry_msgs__msg__Twist *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__assign_function__RobotVisualStates__twist(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Twist * item =
    ((geometry_msgs__msg__Twist *)
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__twist(untyped_member, index));
  const geometry_msgs__msg__Twist * value =
    (const geometry_msgs__msg__Twist *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__resize_function__RobotVisualStates__twist(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Twist__Sequence * member =
    (geometry_msgs__msg__Twist__Sequence *)(untyped_member);
  geometry_msgs__msg__Twist__Sequence__fini(member);
  return geometry_msgs__msg__Twist__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotVisualStates, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotVisualStates, id),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__size_function__RobotVisualStates__id,  // size() function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__id,  // get_const(index) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__id,  // get(index) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__fetch_function__RobotVisualStates__id,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__assign_function__RobotVisualStates__id,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__resize_function__RobotVisualStates__id  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotVisualStates, pose),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__size_function__RobotVisualStates__pose,  // size() function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__pose,  // get_const(index) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__pose,  // get(index) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__fetch_function__RobotVisualStates__pose,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__assign_function__RobotVisualStates__pose,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__resize_function__RobotVisualStates__pose  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotVisualStates, twist),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__size_function__RobotVisualStates__twist,  // size() function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_const_function__RobotVisualStates__twist,  // get_const(index) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__get_function__RobotVisualStates__twist,  // get(index) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__fetch_function__RobotVisualStates__twist,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__assign_function__RobotVisualStates__twist,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__resize_function__RobotVisualStates__twist  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_members = {
  "astribot_msgs__msg",  // message namespace
  "RobotVisualStates",  // message name
  4,  // number of fields
  sizeof(astribot_msgs__msg__RobotVisualStates),
  astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_member_array,  // message members
  astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_type_support_handle = {
  0,
  &astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, RobotVisualStates)() {
  astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__RobotVisualStates__rosidl_typesupport_introspection_c__RobotVisualStates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
