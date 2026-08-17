// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:srv/RawRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__STRUCT_H_
#define ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RawRequest in the package astribot_msgs.
typedef struct astribot_msgs__srv__RawRequest_Request
{
  rosidl_runtime_c__String request;
} astribot_msgs__srv__RawRequest_Request;

// Struct for a sequence of astribot_msgs__srv__RawRequest_Request.
typedef struct astribot_msgs__srv__RawRequest_Request__Sequence
{
  astribot_msgs__srv__RawRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__RawRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RawRequest in the package astribot_msgs.
typedef struct astribot_msgs__srv__RawRequest_Response
{
  rosidl_runtime_c__String response;
} astribot_msgs__srv__RawRequest_Response;

// Struct for a sequence of astribot_msgs__srv__RawRequest_Response.
typedef struct astribot_msgs__srv__RawRequest_Response__Sequence
{
  astribot_msgs__srv__RawRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__RawRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__RAW_REQUEST__STRUCT_H_
