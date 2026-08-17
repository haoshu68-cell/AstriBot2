// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/RobotJointState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'position'
// Member 'velocity'
// Member 'acceleration'
// Member 'torque'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RobotJointState in the package astribot_msgs.
typedef struct astribot_msgs__msg__RobotJointState
{
  std_msgs__msg__Header header;
  int8_t mode;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  rosidl_runtime_c__double__Sequence acceleration;
  rosidl_runtime_c__double__Sequence torque;
} astribot_msgs__msg__RobotJointState;

// Struct for a sequence of astribot_msgs__msg__RobotJointState.
typedef struct astribot_msgs__msg__RobotJointState__Sequence
{
  astribot_msgs__msg__RobotJointState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__RobotJointState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_STATE__STRUCT_H_
