// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__STRUCT_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__STRUCT_H_

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
// Member 'left_arm_twist'
// Member 'right_arm_twist'
// Member 'torso_twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'pose_world_to_torso_desired'
// Member 'pose_world_to_left_arm_desired'
// Member 'pose_world_to_right_arm_desired'
// Member 'pose_world_to_chassis_current'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/WholeBodyCtrlCmd in the package astribot_msgs.
/**
  * Request
 */
typedef struct astribot_msgs__msg__WholeBodyCtrlCmd
{
  astribot_msgs__msg__AstribotHeader header;
  /// Twist for left and right arms (linear and angular velocity)
  geometry_msgs__msg__Twist left_arm_twist;
  geometry_msgs__msg__Twist right_arm_twist;
  geometry_msgs__msg__Twist torso_twist;
  /// [x,y,z,qx,qy,qz,qw] desired poses in world frame for cartesian commands
  rosidl_runtime_c__double__Sequence pose_world_to_torso_desired;
  rosidl_runtime_c__double__Sequence pose_world_to_left_arm_desired;
  rosidl_runtime_c__double__Sequence pose_world_to_right_arm_desired;
  rosidl_runtime_c__double__Sequence pose_world_to_chassis_current;
  bool torso_open_loop;
  bool enable_collision_avoidance;
  /// # !!!It will not work!!! Reset WBC first loop flag if true
  /// bool first_flag
  /// Current smooth time (0.0 to smooth_duration)
  double smooth_t;
  /// Total duration for smooth transition (default: 1.0)
  double smooth_duration;
} astribot_msgs__msg__WholeBodyCtrlCmd;

// Struct for a sequence of astribot_msgs__msg__WholeBodyCtrlCmd.
typedef struct astribot_msgs__msg__WholeBodyCtrlCmd__Sequence
{
  astribot_msgs__msg__WholeBodyCtrlCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__msg__WholeBodyCtrlCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__STRUCT_H_
