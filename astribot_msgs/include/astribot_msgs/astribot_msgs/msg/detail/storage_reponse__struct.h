// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/StorageReponse.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REPONSE__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StorageReponse in the package astribot_msgs.
typedef struct astribot_msgs__msg__StorageReponse
{
  bool is_success;
  rosidl_runtime_c__String response;
} astribot_msgs__msg__StorageReponse;

// Struct for a sequence of astribot_msgs__msg__StorageReponse.
typedef struct astribot_msgs__msg__StorageReponse__Sequence
{
  astribot_msgs__msg__StorageReponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__StorageReponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REPONSE__STRUCT_H_
