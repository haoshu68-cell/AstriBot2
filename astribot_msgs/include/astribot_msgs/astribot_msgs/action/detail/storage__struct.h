// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astribot_msgs:action/Storage.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__STRUCT_H_
#define ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic_list'
// Member 'archive_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_Goal
{
  /// list of topics need to record
  rosidl_runtime_c__String topic_list;
  /// 0 表示采用数采模式并发送需要录制的话题
  /// 1 表示触发数采
  /// 2 表示采用快照模式并发送需要录制的话题
  /// 3 表示触发快照
  int32_t storage_type;
  /// 触发时间
  int64_t trigger_time;
  /// 最终数据归档的目标文件夹
  rosidl_runtime_c__String archive_path;
} astribot_msgs__action__Storage_Goal;

// Struct for a sequence of astribot_msgs__action__Storage_Goal.
typedef struct astribot_msgs__action__Storage_Goal__Sequence
{
  astribot_msgs__action__Storage_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'dir_path'
// Member 'miss_topic_list'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_Result
{
  rosidl_runtime_c__String dir_path;
  rosidl_runtime_c__String miss_topic_list;
} astribot_msgs__action__Storage_Result;

// Struct for a sequence of astribot_msgs__action__Storage_Result.
typedef struct astribot_msgs__action__Storage_Result__Sequence
{
  astribot_msgs__action__Storage_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_Result__Sequence;


// Constants defined in the message

/// Constant 'STATUS_RECORDING'.
enum
{
  astribot_msgs__action__Storage_Feedback__STATUS_RECORDING = 3ul
};

/// Constant 'STATUS_UPDATE'.
enum
{
  astribot_msgs__action__Storage_Feedback__STATUS_UPDATE = 4ul
};

/// Constant 'STATUS_STOP'.
enum
{
  astribot_msgs__action__Storage_Feedback__STATUS_STOP = 5ul
};

// Include directives for member types
// Member 'miss_topic_list'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_Feedback
{
  int32_t seconds;
  int32_t disk_space;
  rosidl_runtime_c__String miss_topic_list;
  uint32_t status;
} astribot_msgs__action__Storage_Feedback;

// Struct for a sequence of astribot_msgs__action__Storage_Feedback.
typedef struct astribot_msgs__action__Storage_Feedback__Sequence
{
  astribot_msgs__action__Storage_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "astribot_msgs/action/detail/storage__struct.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  astribot_msgs__action__Storage_Goal goal;
} astribot_msgs__action__Storage_SendGoal_Request;

// Struct for a sequence of astribot_msgs__action__Storage_SendGoal_Request.
typedef struct astribot_msgs__action__Storage_SendGoal_Request__Sequence
{
  astribot_msgs__action__Storage_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} astribot_msgs__action__Storage_SendGoal_Response;

// Struct for a sequence of astribot_msgs__action__Storage_SendGoal_Response.
typedef struct astribot_msgs__action__Storage_SendGoal_Response__Sequence
{
  astribot_msgs__action__Storage_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} astribot_msgs__action__Storage_GetResult_Request;

// Struct for a sequence of astribot_msgs__action__Storage_GetResult_Request.
typedef struct astribot_msgs__action__Storage_GetResult_Request__Sequence
{
  astribot_msgs__action__Storage_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_GetResult_Response
{
  int8_t status;
  astribot_msgs__action__Storage_Result result;
} astribot_msgs__action__Storage_GetResult_Response;

// Struct for a sequence of astribot_msgs__action__Storage_GetResult_Response.
typedef struct astribot_msgs__action__Storage_GetResult_Response__Sequence
{
  astribot_msgs__action__Storage_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"

/// Struct defined in action/Storage in the package astribot_msgs.
typedef struct astribot_msgs__action__Storage_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  astribot_msgs__action__Storage_Feedback feedback;
} astribot_msgs__action__Storage_FeedbackMessage;

// Struct for a sequence of astribot_msgs__action__Storage_FeedbackMessage.
typedef struct astribot_msgs__action__Storage_FeedbackMessage__Sequence
{
  astribot_msgs__action__Storage_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astribot_msgs__action__Storage_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__STRUCT_H_
