// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:srv/StorageService.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__STRUCT_H_
#define ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__STRUCT_H_

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
#include "astribot_msgs/msg/detail/storage_request__struct.h"

/// Struct defined in srv/StorageService in the package astribot_msgs.
typedef struct astribot_msgs__srv__StorageService_Request
{
  astribot_msgs__msg__StorageRequest request;
} astribot_msgs__srv__StorageService_Request;

// Struct for a sequence of astribot_msgs__srv__StorageService_Request.
typedef struct astribot_msgs__srv__StorageService_Request__Sequence
{
  astribot_msgs__srv__StorageService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__StorageService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "astribot_msgs/msg/detail/storage_reponse__struct.h"

/// Struct defined in srv/StorageService in the package astribot_msgs.
typedef struct astribot_msgs__srv__StorageService_Response
{
  astribot_msgs__msg__StorageReponse response;
} astribot_msgs__srv__StorageService_Response;

// Struct for a sequence of astribot_msgs__srv__StorageService_Response.
typedef struct astribot_msgs__srv__StorageService_Response__Sequence
{
  astribot_msgs__srv__StorageService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__srv__StorageService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__SRV__DETAIL__STORAGE_SERVICE__STRUCT_H_
