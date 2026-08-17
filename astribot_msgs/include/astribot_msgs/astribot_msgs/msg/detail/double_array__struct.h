// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/DoubleArray.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/DoubleArray in the package astribot_msgs.
typedef struct astribot_msgs__msg__DoubleArray
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence data;
} astribot_msgs__msg__DoubleArray;

// Struct for a sequence of astribot_msgs__msg__DoubleArray.
typedef struct astribot_msgs__msg__DoubleArray__Sequence
{
  astribot_msgs__msg__DoubleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__DoubleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__DOUBLE_ARRAY__STRUCT_H_
