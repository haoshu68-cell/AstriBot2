// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/SlaveMotorCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/slave_motor_command__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/slave_motor_command__functions.h"
#include "astribot_msgs/msg/detail/slave_motor_command__struct.h"


// Include directives for member types
// Member `header`
#include "astribot_msgs/msg/astribot_header.h"
// Member `header`
#include "astribot_msgs/msg/detail/astribot_header__rosidl_typesupport_introspection_c.h"
// Member `slave_motor_command`
#include "astribot_msgs/msg/motor_command.h"
// Member `slave_motor_command`
#include "astribot_msgs/msg/detail/motor_command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__SlaveMotorCommand__init(message_memory);
}

void astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_fini_function(void * message_memory)
{
  astribot_msgs__msg__SlaveMotorCommand__fini(message_memory);
}

size_t astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__size_function__SlaveMotorCommand__slave_motor_command(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__get_const_function__SlaveMotorCommand__slave_motor_command(
  const void * untyped_member, size_t index)
{
  const astribot_msgs__msg__MotorCommand * member =
    (const astribot_msgs__msg__MotorCommand *)(untyped_member);
  return &member[index];
}

void * astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__get_function__SlaveMotorCommand__slave_motor_command(
  void * untyped_member, size_t index)
{
  astribot_msgs__msg__MotorCommand * member =
    (astribot_msgs__msg__MotorCommand *)(untyped_member);
  return &member[index];
}

void astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__fetch_function__SlaveMotorCommand__slave_motor_command(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const astribot_msgs__msg__MotorCommand * item =
    ((const astribot_msgs__msg__MotorCommand *)
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__get_const_function__SlaveMotorCommand__slave_motor_command(untyped_member, index));
  astribot_msgs__msg__MotorCommand * value =
    (astribot_msgs__msg__MotorCommand *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__assign_function__SlaveMotorCommand__slave_motor_command(
  void * untyped_member, size_t index, const void * untyped_value)
{
  astribot_msgs__msg__MotorCommand * item =
    ((astribot_msgs__msg__MotorCommand *)
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__get_function__SlaveMotorCommand__slave_motor_command(untyped_member, index));
  const astribot_msgs__msg__MotorCommand * value =
    (const astribot_msgs__msg__MotorCommand *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__SlaveMotorCommand, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "slave_motor_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__SlaveMotorCommand, slave_motor_command),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__size_function__SlaveMotorCommand__slave_motor_command,  // size() function pointer
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__get_const_function__SlaveMotorCommand__slave_motor_command,  // get_const(index) function pointer
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__get_function__SlaveMotorCommand__slave_motor_command,  // get(index) function pointer
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__fetch_function__SlaveMotorCommand__slave_motor_command,  // fetch(index, &value) function pointer
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__assign_function__SlaveMotorCommand__slave_motor_command,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_members = {
  "astribot_msgs__msg",  // message namespace
  "SlaveMotorCommand",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__msg__SlaveMotorCommand),
  astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_member_array,  // message members
  astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_type_support_handle = {
  0,
  &astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, SlaveMotorCommand)() {
  astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, AstribotHeader)();
  astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, MotorCommand)();
  if (!astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__SlaveMotorCommand__rosidl_typesupport_introspection_c__SlaveMotorCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
