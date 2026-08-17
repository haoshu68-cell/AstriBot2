// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:srv/AstribotHeaderSrv.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__STRUCT_H_
#define ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__STRUCT_H_

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

/// Struct defined in srv/AstribotHeaderSrv in the package astribot_msgs.
typedef struct astribot_msgs__srv__AstribotHeaderSrv_Request
{
  astribot_msgs__msg__AstribotHeader header;
} astribot_msgs__srv__AstribotHeaderSrv_Request;

// Struct for a sequence of astribot_msgs__srv__AstribotHeaderSrv_Request.
typedef struct astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence
{
  astribot_msgs__srv__AstribotHeaderSrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'header'
// already included above
// #include "astribot_msgs/msg/detail/astribot_header__struct.h"

/// Struct defined in srv/AstribotHeaderSrv in the package astribot_msgs.
typedef struct astribot_msgs__srv__AstribotHeaderSrv_Response
{
  astribot_msgs__msg__AstribotHeader header;
} astribot_msgs__srv__AstribotHeaderSrv_Response;

// Struct for a sequence of astribot_msgs__srv__AstribotHeaderSrv_Response.
typedef struct astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence
{
  astribot_msgs__srv__AstribotHeaderSrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__ASTRIBOT_HEADER_SRV__STRUCT_H_
