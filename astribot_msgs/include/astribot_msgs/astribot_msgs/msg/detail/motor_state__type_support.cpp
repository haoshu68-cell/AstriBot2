// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/motor_state__struct.hpp"
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

void MotorState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::MotorState(_init);
}

void MotorState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::MotorState *>(message_memory);
  typed_message->~MotorState();
}

size_t size_function__MotorState__p(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__p(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__p(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__p(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__p(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__p(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__p(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__v(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__v(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__v(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__v(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__v(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__v(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__v(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__c(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__c(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__c(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__c(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__c(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__c(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__c(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__vol(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__vol(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__vol(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__vol(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__vol(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__vol(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__vol(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__acc(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__acc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__acc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__acc(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__acc(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__can_count(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__can_count(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__can_count(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__can_count(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__MotorState__can_count(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__MotorState__can_count(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__MotorState__can_count(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__can_count_last(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__can_count_last(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__can_count_last(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__can_count_last(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__MotorState__can_count_last(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__MotorState__can_count_last(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__MotorState__can_count_last(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__can_error(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__can_error(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__can_error(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__can_error(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__MotorState__can_error(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__MotorState__can_error(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__MotorState__can_error(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__j_s(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__j_s(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__j_s(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__j_s(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__MotorState__j_s(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__MotorState__j_s(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__MotorState__j_s(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__j_p(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__j_p(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__j_p(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__j_p(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__j_p(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__j_p(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__j_p(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__j_v(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__j_v(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__j_v(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__j_v(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__j_v(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__j_v(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__j_v(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorState__j_a(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorState__j_a(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorState__j_a(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorState__j_a(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorState__j_a(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorState__j_a(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorState__j_a(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorState_message_member_array[12] = {
  {
    "p",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, p),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__p,  // size() function pointer
    get_const_function__MotorState__p,  // get_const(index) function pointer
    get_function__MotorState__p,  // get(index) function pointer
    fetch_function__MotorState__p,  // fetch(index, &value) function pointer
    assign_function__MotorState__p,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "v",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, v),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__v,  // size() function pointer
    get_const_function__MotorState__v,  // get_const(index) function pointer
    get_function__MotorState__v,  // get(index) function pointer
    fetch_function__MotorState__v,  // fetch(index, &value) function pointer
    assign_function__MotorState__v,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "c",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, c),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__c,  // size() function pointer
    get_const_function__MotorState__c,  // get_const(index) function pointer
    get_function__MotorState__c,  // get(index) function pointer
    fetch_function__MotorState__c,  // fetch(index, &value) function pointer
    assign_function__MotorState__c,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vol",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, vol),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__vol,  // size() function pointer
    get_const_function__MotorState__vol,  // get_const(index) function pointer
    get_function__MotorState__vol,  // get(index) function pointer
    fetch_function__MotorState__vol,  // fetch(index, &value) function pointer
    assign_function__MotorState__vol,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, acc),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__acc,  // size() function pointer
    get_const_function__MotorState__acc,  // get_const(index) function pointer
    get_function__MotorState__acc,  // get(index) function pointer
    fetch_function__MotorState__acc,  // fetch(index, &value) function pointer
    assign_function__MotorState__acc,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "can_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, can_count),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__can_count,  // size() function pointer
    get_const_function__MotorState__can_count,  // get_const(index) function pointer
    get_function__MotorState__can_count,  // get(index) function pointer
    fetch_function__MotorState__can_count,  // fetch(index, &value) function pointer
    assign_function__MotorState__can_count,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "can_count_last",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, can_count_last),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__can_count_last,  // size() function pointer
    get_const_function__MotorState__can_count_last,  // get_const(index) function pointer
    get_function__MotorState__can_count_last,  // get(index) function pointer
    fetch_function__MotorState__can_count_last,  // fetch(index, &value) function pointer
    assign_function__MotorState__can_count_last,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "can_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, can_error),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__can_error,  // size() function pointer
    get_const_function__MotorState__can_error,  // get_const(index) function pointer
    get_function__MotorState__can_error,  // get(index) function pointer
    fetch_function__MotorState__can_error,  // fetch(index, &value) function pointer
    assign_function__MotorState__can_error,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "j_s",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, j_s),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__j_s,  // size() function pointer
    get_const_function__MotorState__j_s,  // get_const(index) function pointer
    get_function__MotorState__j_s,  // get(index) function pointer
    fetch_function__MotorState__j_s,  // fetch(index, &value) function pointer
    assign_function__MotorState__j_s,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "j_p",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, j_p),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__j_p,  // size() function pointer
    get_const_function__MotorState__j_p,  // get_const(index) function pointer
    get_function__MotorState__j_p,  // get(index) function pointer
    fetch_function__MotorState__j_p,  // fetch(index, &value) function pointer
    assign_function__MotorState__j_p,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "j_v",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, j_v),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__j_v,  // size() function pointer
    get_const_function__MotorState__j_v,  // get_const(index) function pointer
    get_function__MotorState__j_v,  // get(index) function pointer
    fetch_function__MotorState__j_v,  // fetch(index, &value) function pointer
    assign_function__MotorState__j_v,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "j_a",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorState, j_a),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorState__j_a,  // size() function pointer
    get_const_function__MotorState__j_a,  // get_const(index) function pointer
    get_function__MotorState__j_a,  // get(index) function pointer
    fetch_function__MotorState__j_a,  // fetch(index, &value) function pointer
    assign_function__MotorState__j_a,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorState_message_members = {
  "astribot_msgs::msg",  // message namespace
  "MotorState",  // message name
  12,  // number of fields
  sizeof(astribot_msgs::msg::MotorState),
  MotorState_message_member_array,  // message members
  MotorState_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorState_message_members,
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
get_message_type_support_handle<astribot_msgs::msg::MotorState>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::MotorState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, MotorState)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::MotorState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
