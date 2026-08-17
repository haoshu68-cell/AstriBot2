// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/astribot_control_command__struct.hpp"
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

void AstribotControlCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::AstribotControlCommand(_init);
}

void AstribotControlCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::AstribotControlCommand *>(message_memory);
  typed_message->~AstribotControlCommand();
}

size_t size_function__AstribotControlCommand__name_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AstribotControlCommand__name_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__AstribotControlCommand__name_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__AstribotControlCommand__name_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__AstribotControlCommand__name_list(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__AstribotControlCommand__name_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__AstribotControlCommand__name_list(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__AstribotControlCommand__name_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AstribotControlCommand__dofs_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AstribotControlCommand__dofs_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__AstribotControlCommand__dofs_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__AstribotControlCommand__dofs_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__AstribotControlCommand__dofs_list(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__AstribotControlCommand__dofs_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__AstribotControlCommand__dofs_list(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__AstribotControlCommand__dofs_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AstribotControlCommand__command_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AstribotControlCommand__command_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__AstribotControlCommand__command_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__AstribotControlCommand__command_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__AstribotControlCommand__command_list(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__AstribotControlCommand__command_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__AstribotControlCommand__command_list(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__AstribotControlCommand__command_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AstribotControlCommand_message_member_array[7] = {
  {
    "name_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, name_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__AstribotControlCommand__name_list,  // size() function pointer
    get_const_function__AstribotControlCommand__name_list,  // get_const(index) function pointer
    get_function__AstribotControlCommand__name_list,  // get(index) function pointer
    fetch_function__AstribotControlCommand__name_list,  // fetch(index, &value) function pointer
    assign_function__AstribotControlCommand__name_list,  // assign(index, value) function pointer
    resize_function__AstribotControlCommand__name_list  // resize(index) function pointer
  },
  {
    "dofs_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, dofs_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__AstribotControlCommand__dofs_list,  // size() function pointer
    get_const_function__AstribotControlCommand__dofs_list,  // get_const(index) function pointer
    get_function__AstribotControlCommand__dofs_list,  // get(index) function pointer
    fetch_function__AstribotControlCommand__dofs_list,  // fetch(index, &value) function pointer
    assign_function__AstribotControlCommand__dofs_list,  // assign(index, value) function pointer
    resize_function__AstribotControlCommand__dofs_list  // resize(index) function pointer
  },
  {
    "command_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, command_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__AstribotControlCommand__command_list,  // size() function pointer
    get_const_function__AstribotControlCommand__command_list,  // get_const(index) function pointer
    get_function__AstribotControlCommand__command_list,  // get(index) function pointer
    fetch_function__AstribotControlCommand__command_list,  // fetch(index, &value) function pointer
    assign_function__AstribotControlCommand__command_list,  // assign(index, value) function pointer
    resize_function__AstribotControlCommand__command_list  // resize(index) function pointer
  },
  {
    "control_way",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, control_way),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frame",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, frame),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "use_wbc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, use_wbc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "add_default_torso",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::AstribotControlCommand, add_default_torso),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AstribotControlCommand_message_members = {
  "astribot_msgs::msg",  // message namespace
  "AstribotControlCommand",  // message name
  7,  // number of fields
  sizeof(astribot_msgs::msg::AstribotControlCommand),
  AstribotControlCommand_message_member_array,  // message members
  AstribotControlCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  AstribotControlCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AstribotControlCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AstribotControlCommand_message_members,
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
get_message_type_support_handle<astribot_msgs::msg::AstribotControlCommand>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::AstribotControlCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, AstribotControlCommand)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::AstribotControlCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
