// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/AstribotSerializedProtoMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AstribotSerializedProtoMsg in the package astribot_msgs.
typedef struct astribot_msgs__msg__AstribotSerializedProtoMsg
{
  rosidl_runtime_c__uint8__Sequence data;
} astribot_msgs__msg__AstribotSerializedProtoMsg;

// Struct for a sequence of astribot_msgs__msg__AstribotSerializedProtoMsg.
typedef struct astribot_msgs__msg__AstribotSerializedProtoMsg__Sequence
{
  astribot_msgs__msg__AstribotSerializedProtoMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__AstribotSerializedProtoMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_SERIALIZED_PROTO_MSG__STRUCT_H_
