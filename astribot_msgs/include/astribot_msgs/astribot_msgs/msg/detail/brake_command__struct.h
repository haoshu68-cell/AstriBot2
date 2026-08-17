// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/BrakeCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OPEN_BREAK_CMD'.
enum
{
  astribot_msgs__msg__BrakeCommand__OPEN_BREAK_CMD = 1l
};

/// Constant 'CLOSE_BREAK_CMD'.
enum
{
  astribot_msgs__msg__BrakeCommand__CLOSE_BREAK_CMD = 0l
};

/// Struct defined in msg/BrakeCommand in the package astribot_msgs.
/**
  * 1: open brake
  * 0: close brake
 */
typedef struct astribot_msgs__msg__BrakeCommand
{
  int32_t brake;
} astribot_msgs__msg__BrakeCommand;

// Struct for a sequence of astribot_msgs__msg__BrakeCommand.
typedef struct astribot_msgs__msg__BrakeCommand__Sequence
{
  astribot_msgs__msg__BrakeCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__BrakeCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__BRAKE_COMMAND__STRUCT_H_
