// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:srv/LaunchRequestSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__STRUCT_H_
#define ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__STRUCT_H_

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
#include "astribot_msgs/msg/detail/launch_request__struct.h"

/// Struct defined in srv/LaunchRequestSrv in the package astribot_msgs.
typedef struct astribot_msgs__srv__LaunchRequestSrv_Request
{
  astribot_msgs__msg__LaunchRequest request;
} astribot_msgs__srv__LaunchRequestSrv_Request;

// Struct for a sequence of astribot_msgs__srv__LaunchRequestSrv_Request.
typedef struct astribot_msgs__srv__LaunchRequestSrv_Request__Sequence
{
  astribot_msgs__srv__LaunchRequestSrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__LaunchRequestSrv_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LaunchRequestSrv in the package astribot_msgs.
typedef struct astribot_msgs__srv__LaunchRequestSrv_Response
{
  bool response;
  rosidl_runtime_c__String message;
} astribot_msgs__srv__LaunchRequestSrv_Response;

// Struct for a sequence of astribot_msgs__srv__LaunchRequestSrv_Response.
typedef struct astribot_msgs__srv__LaunchRequestSrv_Response__Sequence
{
  astribot_msgs__srv__LaunchRequestSrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__LaunchRequestSrv_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__LAUNCH_REQUEST_SRV__STRUCT_H_
