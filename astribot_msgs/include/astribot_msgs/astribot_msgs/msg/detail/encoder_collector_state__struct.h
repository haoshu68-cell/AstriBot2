// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EncoderCollectorState in the package astribot_msgs.
/**
  * Message representing the state of an encoder collector
 */
typedef struct astribot_msgs__msg__EncoderCollectorState
{
  /// Position in radians
  float position_rad;
  /// Velocity in radians per second
  float velocity_rps;
  /// Acceleration in radians per second squared
  float acceleration_rpss;
  /// Receive sequence count
  uint8_t rx_sequence_count;
} astribot_msgs__msg__EncoderCollectorState;

// Struct for a sequence of astribot_msgs__msg__EncoderCollectorState.
typedef struct astribot_msgs__msg__EncoderCollectorState__Sequence
{
  astribot_msgs__msg__EncoderCollectorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__EncoderCollectorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__STRUCT_H_
