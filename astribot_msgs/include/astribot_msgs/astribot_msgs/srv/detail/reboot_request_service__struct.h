// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:srv/RebootRequestService.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__REBOOT_REQUEST_SERVICE__STRUCT_H_
#define ASTRIBOT_MSGS__SRV__DETAIL__REBOOT_REQUEST_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RebootRequestService in the package astribot_msgs.
typedef struct astribot_msgs__srv__RebootRequestService_Request
{
  uint8_t structure_needs_at_least_one_member;
} astribot_msgs__srv__RebootRequestService_Request;

// Struct for a sequence of astribot_msgs__srv__RebootRequestService_Request.
typedef struct astribot_msgs__srv__RebootRequestService_Request__Sequence
{
  astribot_msgs__srv__RebootRequestService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__RebootRequestService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RebootRequestService in the package astribot_msgs.
typedef struct astribot_msgs__srv__RebootRequestService_Response
{
  bool response;
  rosidl_runtime_c__String message;
} astribot_msgs__srv__RebootRequestService_Response;

// Struct for a sequence of astribot_msgs__srv__RebootRequestService_Response.
typedef struct astribot_msgs__srv__RebootRequestService_Response__Sequence
{
  astribot_msgs__srv__RebootRequestService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__RebootRequestService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__REBOOT_REQUEST_SERVICE__STRUCT_H_
