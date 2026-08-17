// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

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
  * Kept for documentation purposes
 */
enum
{
  astribot_msgs__msg__MotorState__MAX_MOTOR_NUM = 9
};

/// Struct defined in msg/MotorState in the package astribot_msgs.
typedef struct astribot_msgs__msg__MotorState
{
  /// Position, velocity, and dynamics arrays
  /// Position
  double p[9];
  /// Velocity
  double v[9];
  /// Current
  double c[9];
  /// Voltage
  double vol[9];
  /// Acceleration
  double acc[9];
  /// CAN communication status
  uint8_t can_count[9];
  uint8_t can_count_last[9];
  /// For CanErrorStatus, you might need a separate message or define it here
  /// If using a simple approach for now:
  /// Error status for each motor
  uint8_t can_error[9];
  /// Joint sensor data
  /// Joint encoder status
  bool j_s[9];
  /// Joint position (rad)
  double j_p[9];
  /// Joint velocity (rad/s)
  double j_v[9];
  /// Joint acceleration (rad/s^2)
  double j_a[9];
} astribot_msgs__msg__MotorState;

// Struct for a sequence of astribot_msgs__msg__MotorState.
typedef struct astribot_msgs__msg__MotorState__Sequence
{
  astribot_msgs__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
