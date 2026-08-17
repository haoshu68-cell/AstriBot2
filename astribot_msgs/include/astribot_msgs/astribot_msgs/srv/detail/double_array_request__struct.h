// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:srv/DoubleArrayRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__STRUCT_H_
#define ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__STRUCT_H_

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

/// Struct defined in srv/DoubleArrayRequest in the package astribot_msgs.
typedef struct astribot_msgs__srv__DoubleArrayRequest_Request
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence data;
} astribot_msgs__srv__DoubleArrayRequest_Request;

// Struct for a sequence of astribot_msgs__srv__DoubleArrayRequest_Request.
typedef struct astribot_msgs__srv__DoubleArrayRequest_Request__Sequence
{
  astribot_msgs__srv__DoubleArrayRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__DoubleArrayRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'header'
// already included above
// #include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/DoubleArrayRequest in the package astribot_msgs.
typedef struct astribot_msgs__srv__DoubleArrayRequest_Response
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence data;
} astribot_msgs__srv__DoubleArrayRequest_Response;

// Struct for a sequence of astribot_msgs__srv__DoubleArrayRequest_Response.
typedef struct astribot_msgs__srv__DoubleArrayRequest_Response__Sequence
{
  astribot_msgs__srv__DoubleArrayRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__DoubleArrayRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__DOUBLE_ARRAY_REQUEST__STRUCT_H_
