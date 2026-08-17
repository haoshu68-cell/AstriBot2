// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/LivoxCustomMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/livox_custom_msg__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/livox_custom_msg__functions.h"
#include "astribot_msgs/msg/detail/livox_custom_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "astribot_msgs/msg/custom_point.h"
// Member `points`
#include "astribot_msgs/msg/detail/custom_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__LivoxCustomMsg__init(message_memory);
}

void astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_fini_function(void * message_memory)
{
  astribot_msgs__msg__LivoxCustomMsg__fini(message_memory);
}

size_t astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__size_function__LivoxCustomMsg__rsvd(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_const_function__LivoxCustomMsg__rsvd(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_function__LivoxCustomMsg__rsvd(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__fetch_function__LivoxCustomMsg__rsvd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_const_function__LivoxCustomMsg__rsvd(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__assign_function__LivoxCustomMsg__rsvd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_function__LivoxCustomMsg__rsvd(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__size_function__LivoxCustomMsg__points(
  const void * untyped_member)
{
  const astribot_msgs__msg__CustomPoint__Sequence * member =
    (const astribot_msgs__msg__CustomPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_const_function__LivoxCustomMsg__points(
  const void * untyped_member, size_t index)
{
  const astribot_msgs__msg__CustomPoint__Sequence * member =
    (const astribot_msgs__msg__CustomPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_function__LivoxCustomMsg__points(
  void * untyped_member, size_t index)
{
  astribot_msgs__msg__CustomPoint__Sequence * member =
    (astribot_msgs__msg__CustomPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__fetch_function__LivoxCustomMsg__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const astribot_msgs__msg__CustomPoint * item =
    ((const astribot_msgs__msg__CustomPoint *)
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_const_function__LivoxCustomMsg__points(untyped_member, index));
  astribot_msgs__msg__CustomPoint * value =
    (astribot_msgs__msg__CustomPoint *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__assign_function__LivoxCustomMsg__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  astribot_msgs__msg__CustomPoint * item =
    ((astribot_msgs__msg__CustomPoint *)
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_function__LivoxCustomMsg__points(untyped_member, index));
  const astribot_msgs__msg__CustomPoint * value =
    (const astribot_msgs__msg__CustomPoint *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__resize_function__LivoxCustomMsg__points(
  void * untyped_member, size_t size)
{
  astribot_msgs__msg__CustomPoint__Sequence * member =
    (astribot_msgs__msg__CustomPoint__Sequence *)(untyped_member);
  astribot_msgs__msg__CustomPoint__Sequence__fini(member);
  return astribot_msgs__msg__CustomPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__LivoxCustomMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timebase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__LivoxCustomMsg, timebase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__LivoxCustomMsg, point_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lidar_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__LivoxCustomMsg, lidar_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rsvd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__LivoxCustomMsg, rsvd),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__size_function__LivoxCustomMsg__rsvd,  // size() function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_const_function__LivoxCustomMsg__rsvd,  // get_const(index) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_function__LivoxCustomMsg__rsvd,  // get(index) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__fetch_function__LivoxCustomMsg__rsvd,  // fetch(index, &value) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__assign_function__LivoxCustomMsg__rsvd,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__LivoxCustomMsg, points),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__size_function__LivoxCustomMsg__points,  // size() function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_const_function__LivoxCustomMsg__points,  // get_const(index) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__get_function__LivoxCustomMsg__points,  // get(index) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__fetch_function__LivoxCustomMsg__points,  // fetch(index, &value) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__assign_function__LivoxCustomMsg__points,  // assign(index, value) function pointer
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__resize_function__LivoxCustomMsg__points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_members = {
  "astribot_msgs__msg",  // message namespace
  "LivoxCustomMsg",  // message name
  6,  // number of fields
  sizeof(astribot_msgs__msg__LivoxCustomMsg),
  astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_member_array,  // message members
  astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_type_support_handle = {
  0,
  &astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, LivoxCustomMsg)() {
  astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, CustomPoint)();
  if (!astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__LivoxCustomMsg__rosidl_typesupport_introspection_c__LivoxCustomMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
