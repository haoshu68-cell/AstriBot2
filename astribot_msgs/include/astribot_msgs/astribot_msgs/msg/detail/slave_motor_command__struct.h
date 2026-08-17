// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/SlaveMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAX_SLAVE_MOTOR'.
enum
{
  astribot_msgs__msg__SlaveMotorCommand__MAX_SLAVE_MOTOR = 3
};

// Include directives for member types
// Member 'header'
#include "astribot_msgs/msg/detail/astribot_header__struct.h"
// Member 'slave_motor_command'
#include "astribot_msgs/msg/detail/motor_command__struct.h"

/// Struct defined in msg/SlaveMotorCommand in the package astribot_msgs.
typedef struct astribot_msgs__msg__SlaveMotorCommand
{
  astribot_msgs__msg__AstribotHeader header;
  astribot_msgs__msg__MotorCommand slave_motor_command[3];
} astribot_msgs__msg__SlaveMotorCommand;

// Struct for a sequence of astribot_msgs__msg__SlaveMotorCommand.
typedef struct astribot_msgs__msg__SlaveMotorCommand__Sequence
{
  astribot_msgs__msg__SlaveMotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__SlaveMotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__STRUCT_H_
