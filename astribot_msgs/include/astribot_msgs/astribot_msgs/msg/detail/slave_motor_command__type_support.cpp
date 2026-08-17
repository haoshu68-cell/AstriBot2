// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/SlaveMotorCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/slave_motor_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace astribot_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SlaveMotorCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::SlaveMotorCommand(_init);
}

void SlaveMotorCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::SlaveMotorCommand *>(message_memory);
  typed_message->~SlaveMotorCommand();
}

size_t size_function__SlaveMotorCommand__slave_motor_command(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__SlaveMotorCommand__slave_motor_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<astribot_msgs::msg::MotorCommand, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__SlaveMotorCommand__slave_motor_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<astribot_msgs::msg::MotorCommand, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__SlaveMotorCommand__slave_motor_command(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const astribot_msgs::msg::MotorCommand *>(
    get_const_function__SlaveMotorCommand__slave_motor_command(untyped_member, index));
  auto & value = *reinterpret_cast<astribot_msgs::msg::MotorCommand *>(untyped_value);
  value = item;
}

void assign_function__SlaveMotorCommand__slave_motor_command(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<astribot_msgs::msg::MotorCommand *>(
    get_function__SlaveMotorCommand__slave_motor_command(untyped_member, index));
  const auto & value = *reinterpret_cast<const astribot_msgs::msg::MotorCommand *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SlaveMotorCommand_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<astribot_msgs::msg::AstribotHeader>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::SlaveMotorCommand, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "slave_motor_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<astribot_msgs::msg::MotorCommand>(),  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::SlaveMotorCommand, slave_motor_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__SlaveMotorCommand__slave_motor_command,  // size() function pointer
    get_const_function__SlaveMotorCommand__slave_motor_command,  // get_const(index) function pointer
    get_function__SlaveMotorCommand__slave_motor_command,  // get(index) function pointer
    fetch_function__SlaveMotorCommand__slave_motor_command,  // fetch(index, &value) function pointer
    assign_function__SlaveMotorCommand__slave_motor_command,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SlaveMotorCommand_message_members = {
  "astribot_msgs::msg",  // message namespace
  "SlaveMotorCommand",  // message name
  2,  // number of fields
  sizeof(astribot_msgs::msg::SlaveMotorCommand),
  SlaveMotorCommand_message_member_array,  // message members
  SlaveMotorCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  SlaveMotorCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SlaveMotorCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SlaveMotorCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace astribot_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<astribot_msgs::msg::SlaveMotorCommand>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::SlaveMotorCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, SlaveMotorCommand)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::SlaveMotorCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
