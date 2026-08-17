// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/LaunchRequest.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ENUM_UNKNOWN'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_UNKNOWN = 0
};

/// Constant 'ENUM_ASTRIBOT_CAMERA'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ASTRIBOT_CAMERA = 1
};

/// Constant 'ENUM_ASTRIBOT_JOY'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ASTRIBOT_JOY = 2
};

/// Constant 'ENUM_ASTRIBOT_VR'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ASTRIBOT_VR = 3
};

/// Constant 'ENUM_ASTRIBOT_MIC'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ASTRIBOT_MIC = 4
};

/// Constant 'ENUM_ASTRIBOT_LIDAR'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ASTRIBOT_LIDAR = 5
};

/// Constant 'ENUM_ASTRIBOT_TELEOP'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ASTRIBOT_TELEOP = 6
};

/// Constant 'ENUM_DEACTIVE'.
/**
  * 启动或停止类型
 */
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_DEACTIVE = 1
};

/// Constant 'ENUM_ACTIVE'.
enum
{
  astribot_msgs__msg__LaunchRequest__ENUM_ACTIVE = 2
};

/// Struct defined in msg/LaunchRequest in the package astribot_msgs.
/**
  * 启动/停止的目标类型
 */
typedef struct astribot_msgs__msg__LaunchRequest
{
  uint8_t launch_target;
  uint8_t launch_action;
} astribot_msgs__msg__LaunchRequest;

// Struct for a sequence of astribot_msgs__msg__LaunchRequest.
typedef struct astribot_msgs__msg__LaunchRequest__Sequence
{
  astribot_msgs__msg__LaunchRequest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__LaunchRequest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LAUNCH_REQUEST__STRUCT_H_
