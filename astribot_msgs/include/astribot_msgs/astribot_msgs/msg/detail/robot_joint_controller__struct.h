// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/RobotJointController.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_CONTROLLER__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_CONTROLLER__STRUCT_H_

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
// Member 'command'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RobotJointController in the package astribot_msgs.
typedef struct astribot_msgs__msg__RobotJointController
{
  std_msgs__msg__Header header;
  int8_t mode;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence command;
} astribot_msgs__msg__RobotJointController;

// Struct for a sequence of astribot_msgs__msg__RobotJointController.
typedef struct astribot_msgs__msg__RobotJointController__Sequence
{
  astribot_msgs__msg__RobotJointController * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__RobotJointController__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ROBOT_JOINT_CONTROLLER__STRUCT_H_
