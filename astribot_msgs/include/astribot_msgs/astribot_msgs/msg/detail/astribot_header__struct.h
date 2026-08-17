// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/AstribotHeader.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/AstribotHeader in the package astribot_msgs.
typedef struct astribot_msgs__msg__AstribotHeader
{
  uint64_t seq;
  uint64_t time_meas;
  uint64_t time_pub;
} astribot_msgs__msg__AstribotHeader;

// Struct for a sequence of astribot_msgs__msg__AstribotHeader.
typedef struct astribot_msgs__msg__AstribotHeader__Sequence
{
  astribot_msgs__msg__AstribotHeader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__AstribotHeader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEADER__STRUCT_H_
