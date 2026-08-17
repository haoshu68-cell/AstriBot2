// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/SlaveMotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAX_SLAVE_MOTOR'.
/**
  * Kept for documentation purposes
 */
enum
{
  astribot_msgs__msg__SlaveMotorState__MAX_SLAVE_MOTOR = 3
};

// Include directives for member types
// Member 'header'
#include "astribot_msgs/msg/detail/astribot_header__struct.h"
// Member 'slave_motor_state'
#include "astribot_msgs/msg/detail/motor_state__struct.h"

/// Struct defined in msg/SlaveMotorState in the package astribot_msgs.
typedef struct astribot_msgs__msg__SlaveMotorState
{
  astribot_msgs__msg__AstribotHeader header;
  astribot_msgs__msg__MotorState slave_motor_state[3];
} astribot_msgs__msg__SlaveMotorState;

// Struct for a sequence of astribot_msgs__msg__SlaveMotorState.
typedef struct astribot_msgs__msg__SlaveMotorState__Sequence
{
  astribot_msgs__msg__SlaveMotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__SlaveMotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_STATE__STRUCT_H_
