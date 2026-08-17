// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name_list'
// Member 'control_way'
// Member 'frame'
#include "rosidl_runtime_c/string.h"
// Member 'dofs_list'
// Member 'command_list'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AstribotControlCommand in the package astribot_msgs.
typedef struct astribot_msgs__msg__AstribotControlCommand
{
  rosidl_runtime_c__String__Sequence name_list;
  rosidl_runtime_c__float__Sequence dofs_list;
  rosidl_runtime_c__float__Sequence command_list;
  rosidl_runtime_c__String control_way;
  rosidl_runtime_c__String frame;
  bool use_wbc;
  bool add_default_torso;
} astribot_msgs__msg__AstribotControlCommand;

// Struct for a sequence of astribot_msgs__msg__AstribotControlCommand.
typedef struct astribot_msgs__msg__AstribotControlCommand__Sequence
{
  astribot_msgs__msg__AstribotControlCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__AstribotControlCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ASTRIBOT_CONTROL_COMMAND__STRUCT_H_
