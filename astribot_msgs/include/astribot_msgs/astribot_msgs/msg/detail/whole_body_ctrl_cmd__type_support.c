// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__functions.h"
#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__struct.h"


// Include directives for member types
// Member `header`
#include "astribot_msgs/msg/astribot_header.h"
// Member `header`
#include "astribot_msgs/msg/detail/astribot_header__rosidl_typesupport_introspection_c.h"
// Member `left_arm_twist`
// Member `right_arm_twist`
// Member `torso_twist`
#include "geometry_msgs/msg/twist.h"
// Member `left_arm_twist`
// Member `right_arm_twist`
// Member `torso_twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `pose_world_to_torso_desired`
// Member `pose_world_to_left_arm_desired`
// Member `pose_world_to_right_arm_desired`
// Member `pose_world_to_chassis_current`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__WholeBodyCtrlCmd__init(message_memory);
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_fini_function(void * message_memory)
{
  astribot_msgs__msg__WholeBodyCtrlCmd__fini(message_memory);
}

size_t astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_member_array[12] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_arm_twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, left_arm_twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_arm_twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, right_arm_twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torso_twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, torso_twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose_world_to_torso_desired",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, pose_world_to_torso_desired),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // size() function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // get_const(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // get(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // fetch(index, &value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // assign(index, value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_torso_desired  // resize(index) function pointer
  },
  {
    "pose_world_to_left_arm_desired",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, pose_world_to_left_arm_desired),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // size() function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // get_const(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // get(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // fetch(index, &value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // assign(index, value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired  // resize(index) function pointer
  },
  {
    "pose_world_to_right_arm_desired",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, pose_world_to_right_arm_desired),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // size() function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // get_const(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // get(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // fetch(index, &value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // assign(index, value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired  // resize(index) function pointer
  },
  {
    "pose_world_to_chassis_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, pose_world_to_chassis_current),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__size_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // size() function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // get_const(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__get_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // get(index) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // fetch(index, &value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__assign_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // assign(index, value) function pointer
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__resize_function__WholeBodyCtrlCmd__pose_world_to_chassis_current  // resize(index) function pointer
  },
  {
    "torso_open_loop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, torso_open_loop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enable_collision_avoidance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, enable_collision_avoidance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "smooth_t",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, smooth_t),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "smooth_duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__WholeBodyCtrlCmd, smooth_duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_members = {
  "astribot_msgs__msg",  // message namespace
  "WholeBodyCtrlCmd",  // message name
  12,  // number of fields
  sizeof(astribot_msgs__msg__WholeBodyCtrlCmd),
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_member_array,  // message members
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_type_support_handle = {
  0,
  &astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, WholeBodyCtrlCmd)() {
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, AstribotHeader)();
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__WholeBodyCtrlCmd__rosidl_typesupport_introspection_c__WholeBodyCtrlCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
