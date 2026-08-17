// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:action/Storage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/action/detail/storage__functions.h"
#include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `topic_list`
// Member `archive_path`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_Goal__init(message_memory);
}

void astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_member_array[4] = {
  {
    "topic_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Goal, topic_list),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "storage_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Goal, storage_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trigger_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Goal, trigger_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "archive_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Goal, archive_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_Goal",  // message name
  4,  // number of fields
  sizeof(astribot_msgs__action__Storage_Goal),
  astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_member_array,  // message members
  astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_Goal)() {
  if (!astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_Goal__rosidl_typesupport_introspection_c__Storage_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `dir_path`
// Member `miss_topic_list`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_Result__init(message_memory);
}

void astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_member_array[2] = {
  {
    "dir_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Result, dir_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "miss_topic_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Result, miss_topic_list),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_Result",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__action__Storage_Result),
  astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_member_array,  // message members
  astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_Result)() {
  if (!astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_Result__rosidl_typesupport_introspection_c__Storage_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `miss_topic_list`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_Feedback__init(message_memory);
}

void astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_member_array[4] = {
  {
    "seconds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Feedback, seconds),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "disk_space",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Feedback, disk_space),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "miss_topic_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Feedback, miss_topic_list),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_Feedback, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_Feedback",  // message name
  4,  // number of fields
  sizeof(astribot_msgs__action__Storage_Feedback),
  astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_member_array,  // message members
  astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_Feedback)() {
  if (!astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_Feedback__rosidl_typesupport_introspection_c__Storage_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "astribot_msgs/action/storage.h"
// Member `goal`
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_SendGoal_Request__init(message_memory);
}

void astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__action__Storage_SendGoal_Request),
  astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_member_array,  // message members
  astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal_Request)() {
  astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_Goal)();
  if (!astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_SendGoal_Request__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_SendGoal_Response__init(message_memory);
}

void astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__action__Storage_SendGoal_Response),
  astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_member_array,  // message members
  astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal_Response)() {
  astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_SendGoal_Response__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_members = {
  "astribot_msgs__action",  // service namespace
  "Storage_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_type_support_handle = {
  0,
  &astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal)() {
  if (!astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_SendGoal_Response)()->data;
  }

  return &astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_GetResult_Request__init(message_memory);
}

void astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(astribot_msgs__action__Storage_GetResult_Request),
  astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_member_array,  // message members
  astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult_Request)() {
  astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_GetResult_Request__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "astribot_msgs/action/storage.h"
// Member `result`
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_GetResult_Response__init(message_memory);
}

void astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__action__Storage_GetResult_Response),
  astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_member_array,  // message members
  astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult_Response)() {
  astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_Result)();
  if (!astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_GetResult_Response__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_members = {
  "astribot_msgs__action",  // service namespace
  "Storage_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_type_support_handle = {
  0,
  &astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult)() {
  if (!astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_GetResult_Response)()->data;
  }

  return &astribot_msgs__action__detail__storage__rosidl_typesupport_introspection_c__Storage_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/action/detail/storage__functions.h"
// already included above
// #include "astribot_msgs/action/detail/storage__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "astribot_msgs/action/storage.h"
// Member `feedback`
// already included above
// #include "astribot_msgs/action/detail/storage__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__action__Storage_FeedbackMessage__init(message_memory);
}

void astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_fini_function(void * message_memory)
{
  astribot_msgs__action__Storage_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__action__Storage_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_members = {
  "astribot_msgs__action",  // message namespace
  "Storage_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__action__Storage_FeedbackMessage),
  astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_member_array,  // message members
  astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_type_support_handle = {
  0,
  &astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_FeedbackMessage)() {
  astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, action, Storage_Feedback)();
  if (!astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__action__Storage_FeedbackMessage__rosidl_typesupport_introspection_c__Storage_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
