// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/RobotVisualStates.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__STRUCT_H_

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
// Member 'id'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/RobotVisualStates in the package astribot_msgs.
typedef struct astribot_msgs__msg__RobotVisualStates
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence id;
  geometry_msgs__msg__Pose__Sequence pose;
  geometry_msgs__msg__Twist__Sequence twist;
} astribot_msgs__msg__RobotVisualStates;

// Struct for a sequence of astribot_msgs__msg__RobotVisualStates.
typedef struct astribot_msgs__msg__RobotVisualStates__Sequence
{
  astribot_msgs__msg__RobotVisualStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__RobotVisualStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_VISUAL_STATES__STRUCT_H_
