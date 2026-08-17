// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/StorageRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_RECORD_MODE'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_RECORD_MODE = 0
};

/// Constant 'TYPE_START_RECORD'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_START_RECORD = 1
};

/// Constant 'TYPE_STOP_RECORD'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_STOP_RECORD = 2
};

/// Constant 'TYPE_SNAPSHOT_MODE'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_SNAPSHOT_MODE = 3
};

/// Constant 'TYPE_TRIGGER_SNAPSHOT'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_TRIGGER_SNAPSHOT = 4
};

/// Constant 'TYPE_INACTIVE_MODE'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_INACTIVE_MODE = 5
};

/// Constant 'TYPE_SAVE_DATA'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_SAVE_DATA = 6
};

/// Constant 'TYPE_DELETE_DATA'.
enum
{
  astribot_msgs__msg__StorageRequest__TYPE_DELETE_DATA = 7
};

// Include directives for member types
// Member 'uuid'
// Member 'tag_content'
// Member 'topic_list'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StorageRequest in the package astribot_msgs.
typedef struct astribot_msgs__msg__StorageRequest
{
  rosidl_runtime_c__String uuid;
  uint8_t storage_type;
  rosidl_runtime_c__String tag_content;
  rosidl_runtime_c__String topic_list;
} astribot_msgs__msg__StorageRequest;

// Struct for a sequence of astribot_msgs__msg__StorageRequest.
typedef struct astribot_msgs__msg__StorageRequest__Sequence
{
  astribot_msgs__msg__StorageRequest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__StorageRequest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__STORAGE_REQUEST__STRUCT_H_
