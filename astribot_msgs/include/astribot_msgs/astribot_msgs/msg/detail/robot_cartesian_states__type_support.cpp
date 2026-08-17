// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/RobotCartesianStates.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/robot_cartesian_states__struct.hpp"
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

void RobotCartesianStates_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::RobotCartesianStates(_init);
}

void RobotCartesianStates_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::RobotCartesianStates *>(message_memory);
  typed_message->~RobotCartesianStates();
}

size_t size_function__RobotCartesianStates__names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotCartesianStates__names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotCartesianStates__names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotCartesianStates__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RobotCartesianStates__names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RobotCartesianStates__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RobotCartesianStates__names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RobotCartesianStates__names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RobotCartesianStates__states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<astribot_msgs::msg::RobotCartesianState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotCartesianStates__states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<astribot_msgs::msg::RobotCartesianState> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotCartesianStates__states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<astribot_msgs::msg::RobotCartesianState> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotCartesianStates__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const astribot_msgs::msg::RobotCartesianState *>(
    get_const_function__RobotCartesianStates__states(untyped_member, index));
  auto & value = *reinterpret_cast<astribot_msgs::msg::RobotCartesianState *>(untyped_value);
  value = item;
}

void assign_function__RobotCartesianStates__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<astribot_msgs::msg::RobotCartesianState *>(
    get_function__RobotCartesianStates__states(untyped_member, index));
  const auto & value = *reinterpret_cast<const astribot_msgs::msg::RobotCartesianState *>(untyped_value);
  item = value;
}

void resize_function__RobotCartesianStates__states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<astribot_msgs::msg::RobotCartesianState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotCartesianStates_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::RobotCartesianStates, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::RobotCartesianStates, names),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotCartesianStates__names,  // size() function pointer
    get_const_function__RobotCartesianStates__names,  // get_const(index) function pointer
    get_function__RobotCartesianStates__names,  // get(index) function pointer
    fetch_function__RobotCartesianStates__names,  // fetch(index, &value) function pointer
    assign_function__RobotCartesianStates__names,  // assign(index, value) function pointer
    resize_function__RobotCartesianStates__names  // resize(index) function pointer
  },
  {
    "states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<astribot_msgs::msg::RobotCartesianState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::RobotCartesianStates, states),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotCartesianStates__states,  // size() function pointer
    get_const_function__RobotCartesianStates__states,  // get_const(index) function pointer
    get_function__RobotCartesianStates__states,  // get(index) function pointer
    fetch_function__RobotCartesianStates__states,  // fetch(index, &value) function pointer
    assign_function__RobotCartesianStates__states,  // assign(index, value) function pointer
    resize_function__RobotCartesianStates__states  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotCartesianStates_message_members = {
  "astribot_msgs::msg",  // message namespace
  "RobotCartesianStates",  // message name
  3,  // number of fields
  sizeof(astribot_msgs::msg::RobotCartesianStates),
  RobotCartesianStates_message_member_array,  // message members
  RobotCartesianStates_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotCartesianStates_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotCartesianStates_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotCartesianStates_message_members,
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
get_message_type_support_handle<astribot_msgs::msg::RobotCartesianStates>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::RobotCartesianStates_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, RobotCartesianStates)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::RobotCartesianStates_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
