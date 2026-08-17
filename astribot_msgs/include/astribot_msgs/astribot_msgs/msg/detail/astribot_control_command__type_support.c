// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/astribot_control_command__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/astribot_control_command__functions.h"
#include "astribot_msgs/msg/detail/astribot_control_command__struct.h"


// Include directives for member types
// Member `name_list`
// Member `control_way`
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"
// Member `dofs_list`
// Member `command_list`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__AstribotControlCommand__init(message_memory);
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_fini_function(void * message_memory)
{
  astribot_msgs__msg__AstribotControlCommand__fini(message_memory);
}

size_t astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__size_function__AstribotControlCommand__name_list(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__name_list(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__name_list(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__fetch_function__AstribotControlCommand__name_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__name_list(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__assign_function__AstribotControlCommand__name_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__name_list(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__resize_function__AstribotControlCommand__name_list(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__size_function__AstribotControlCommand__dofs_list(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__dofs_list(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__dofs_list(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__fetch_function__AstribotControlCommand__dofs_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__dofs_list(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__assign_function__AstribotControlCommand__dofs_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__dofs_list(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__resize_function__AstribotControlCommand__dofs_list(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__size_function__AstribotControlCommand__command_list(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__command_list(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__command_list(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__fetch_function__AstribotControlCommand__command_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__command_list(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__assign_function__AstribotControlCommand__command_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__command_list(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__resize_function__AstribotControlCommand__command_list(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_member_array[7] = {
  {
    "name_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, name_list),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__size_function__AstribotControlCommand__name_list,  // size() function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__name_list,  // get_const(index) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__name_list,  // get(index) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__fetch_function__AstribotControlCommand__name_list,  // fetch(index, &value) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__assign_function__AstribotControlCommand__name_list,  // assign(index, value) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__resize_function__AstribotControlCommand__name_list  // resize(index) function pointer
  },
  {
    "dofs_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, dofs_list),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__size_function__AstribotControlCommand__dofs_list,  // size() function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__dofs_list,  // get_const(index) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__dofs_list,  // get(index) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__fetch_function__AstribotControlCommand__dofs_list,  // fetch(index, &value) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__assign_function__AstribotControlCommand__dofs_list,  // assign(index, value) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__resize_function__AstribotControlCommand__dofs_list  // resize(index) function pointer
  },
  {
    "command_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, command_list),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__size_function__AstribotControlCommand__command_list,  // size() function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_const_function__AstribotControlCommand__command_list,  // get_const(index) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__get_function__AstribotControlCommand__command_list,  // get(index) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__fetch_function__AstribotControlCommand__command_list,  // fetch(index, &value) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__assign_function__AstribotControlCommand__command_list,  // assign(index, value) function pointer
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__resize_function__AstribotControlCommand__command_list  // resize(index) function pointer
  },
  {
    "control_way",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, control_way),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "use_wbc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, use_wbc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "add_default_torso",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__AstribotControlCommand, add_default_torso),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_members = {
  "astribot_msgs__msg",  // message namespace
  "AstribotControlCommand",  // message name
  7,  // number of fields
  sizeof(astribot_msgs__msg__AstribotControlCommand),
  astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_member_array,  // message members
  astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_type_support_handle = {
  0,
  &astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, AstribotControlCommand)() {
  if (!astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__AstribotControlCommand__rosidl_typesupport_introspection_c__AstribotControlCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
