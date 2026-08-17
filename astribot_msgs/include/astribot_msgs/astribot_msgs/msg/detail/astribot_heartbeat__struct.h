// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/AstribotHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__STRUCT_H_

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
#include "astribot_msgs/msg/detail/astribot_header__struct.h"
// Member 'node_name'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in msg/AstribotHeartbeat in the package astribot_msgs.
typedef struct astribot_msgs__msg__AstribotHeartbeat
{
  astribot_msgs__msg__AstribotHeader header;
  std_msgs__msg__String node_name;
} astribot_msgs__msg__AstribotHeartbeat;

// Struct for a sequence of astribot_msgs__msg__AstribotHeartbeat.
typedef struct astribot_msgs__msg__AstribotHeartbeat__Sequence
{
  astribot_msgs__msg__AstribotHeartbeat * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__AstribotHeartbeat__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_HEARTBEAT__STRUCT_H_
