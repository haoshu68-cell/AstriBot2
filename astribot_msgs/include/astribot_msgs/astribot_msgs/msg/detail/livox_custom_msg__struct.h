// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/LivoxCustomMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__STRUCT_H_

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
// Member 'points'
#include "astribot_msgs/msg/detail/custom_point__struct.h"

/// Struct defined in msg/LivoxCustomMsg in the package astribot_msgs.
/**
  * Livox publish pointcloud msg format.
 */
typedef struct astribot_msgs__msg__LivoxCustomMsg
{
  /// ROS standard message header
  std_msgs__msg__Header header;
  /// The time of first point
  uint64_t timebase;
  /// Total number of pointclouds
  uint32_t point_num;
  /// Lidar device id number
  uint8_t lidar_id;
  /// Reserved use
  uint8_t rsvd[3];
  /// Pointcloud data
  astribot_msgs__msg__CustomPoint__Sequence points;
} astribot_msgs__msg__LivoxCustomMsg;

// Struct for a sequence of astribot_msgs__msg__LivoxCustomMsg.
typedef struct astribot_msgs__msg__LivoxCustomMsg__Sequence
{
  astribot_msgs__msg__LivoxCustomMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__LivoxCustomMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__STRUCT_H_
