// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/RobotCartesianState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATE__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.h"

/// Struct defined in msg/RobotCartesianState in the package astribot_msgs.
typedef struct astribot_msgs__msg__RobotCartesianState
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist twist;
  geometry_msgs__msg__Wrench wrench;
} astribot_msgs__msg__RobotCartesianState;

// Struct for a sequence of astribot_msgs__msg__RobotCartesianState.
typedef struct astribot_msgs__msg__RobotCartesianState__Sequence
{
  astribot_msgs__msg__RobotCartesianState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__RobotCartesianState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_CARTESIAN_STATE__STRUCT_H_
