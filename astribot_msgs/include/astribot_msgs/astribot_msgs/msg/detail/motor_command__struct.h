// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAX_MOTOR_NUM'.
/**
  * Matches the existing constant
 */
enum
{
  astribot_msgs__msg__MotorCommand__MAX_MOTOR_NUM = 9
};

// Include directives for member types
// Member 'motor_id_list'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MotorCommand in the package astribot_msgs.
typedef struct astribot_msgs__msg__MotorCommand
{
  /// Control parameters
  /// Proportional gain
  double kp[9];
  /// Derivative gain
  double kd[9];
  /// Position setpoint
  double p[9];
  /// Velocity setpoint
  double v[9];
  /// Feedforward torque
  double t_ff[9];
  /// Torque limit
  double t_limit[9];
  /// Brake release status
  int8_t break_relase[9];
  /// Slave derivative gain
  double kd_slave[9];
  /// Velocity setpoint for slave motors
  double vel_slave[9];
  /// Motor IDs (variable length)
  /// List of motor IDs to control
  rosidl_runtime_c__int32__Sequence motor_id_list;
} astribot_msgs__msg__MotorCommand;

// Struct for a sequence of astribot_msgs__msg__MotorCommand.
typedef struct astribot_msgs__msg__MotorCommand__Sequence
{
  astribot_msgs__msg__MotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__MotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
