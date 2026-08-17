// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/motor_command__struct.hpp"
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

void MotorCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::MotorCommand(_init);
}

void MotorCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::MotorCommand *>(message_memory);
  typed_message->~MotorCommand();
}

size_t size_function__MotorCommand__kp(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__kp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__kp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__kp(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__kp(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__kd(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__kd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__kd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__kd(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__kd(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__p(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__p(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__p(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__p(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__p(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__p(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__p(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__v(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__v(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__v(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__v(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__v(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__v(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__v(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__t_ff(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__t_ff(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__t_ff(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__t_ff(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__t_ff(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__t_ff(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__t_ff(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__t_limit(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__t_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__t_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__t_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__t_limit(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__t_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__t_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__break_relase(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__break_relase(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int8_t, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__break_relase(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int8_t, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__break_relase(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__MotorCommand__break_relase(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__break_relase(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__MotorCommand__break_relase(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__kd_slave(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__kd_slave(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__kd_slave(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__kd_slave(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__kd_slave(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__kd_slave(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__kd_slave(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__vel_slave(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__MotorCommand__vel_slave(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__vel_slave(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__vel_slave(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorCommand__vel_slave(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__vel_slave(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorCommand__vel_slave(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__MotorCommand__motor_id_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorCommand__motor_id_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorCommand__motor_id_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorCommand__motor_id_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__MotorCommand__motor_id_list(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__MotorCommand__motor_id_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__MotorCommand__motor_id_list(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__MotorCommand__motor_id_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorCommand_message_member_array[10] = {
  {
    "kp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, kp),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__kp,  // size() function pointer
    get_const_function__MotorCommand__kp,  // get_const(index) function pointer
    get_function__MotorCommand__kp,  // get(index) function pointer
    fetch_function__MotorCommand__kp,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__kp,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, kd),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__kd,  // size() function pointer
    get_const_function__MotorCommand__kd,  // get_const(index) function pointer
    get_function__MotorCommand__kd,  // get(index) function pointer
    fetch_function__MotorCommand__kd,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__kd,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "p",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, p),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__p,  // size() function pointer
    get_const_function__MotorCommand__p,  // get_const(index) function pointer
    get_function__MotorCommand__p,  // get(index) function pointer
    fetch_function__MotorCommand__p,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__p,  // assign(index, value) function pointer
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
    offsetof(astribot_msgs::msg::MotorCommand, v),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__v,  // size() function pointer
    get_const_function__MotorCommand__v,  // get_const(index) function pointer
    get_function__MotorCommand__v,  // get(index) function pointer
    fetch_function__MotorCommand__v,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__v,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_ff",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, t_ff),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__t_ff,  // size() function pointer
    get_const_function__MotorCommand__t_ff,  // get_const(index) function pointer
    get_function__MotorCommand__t_ff,  // get(index) function pointer
    fetch_function__MotorCommand__t_ff,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__t_ff,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, t_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__t_limit,  // size() function pointer
    get_const_function__MotorCommand__t_limit,  // get_const(index) function pointer
    get_function__MotorCommand__t_limit,  // get(index) function pointer
    fetch_function__MotorCommand__t_limit,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__t_limit,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "break_relase",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, break_relase),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__break_relase,  // size() function pointer
    get_const_function__MotorCommand__break_relase,  // get_const(index) function pointer
    get_function__MotorCommand__break_relase,  // get(index) function pointer
    fetch_function__MotorCommand__break_relase,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__break_relase,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kd_slave",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, kd_slave),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__kd_slave,  // size() function pointer
    get_const_function__MotorCommand__kd_slave,  // get_const(index) function pointer
    get_function__MotorCommand__kd_slave,  // get(index) function pointer
    fetch_function__MotorCommand__kd_slave,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__kd_slave,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel_slave",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, vel_slave),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__vel_slave,  // size() function pointer
    get_const_function__MotorCommand__vel_slave,  // get_const(index) function pointer
    get_function__MotorCommand__vel_slave,  // get(index) function pointer
    fetch_function__MotorCommand__vel_slave,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__vel_slave,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_id_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::MotorCommand, motor_id_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorCommand__motor_id_list,  // size() function pointer
    get_const_function__MotorCommand__motor_id_list,  // get_const(index) function pointer
    get_function__MotorCommand__motor_id_list,  // get(index) function pointer
    fetch_function__MotorCommand__motor_id_list,  // fetch(index, &value) function pointer
    assign_function__MotorCommand__motor_id_list,  // assign(index, value) function pointer
    resize_function__MotorCommand__motor_id_list  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorCommand_message_members = {
  "astribot_msgs::msg",  // message namespace
  "MotorCommand",  // message name
  10,  // number of fields
  sizeof(astribot_msgs::msg::MotorCommand),
  MotorCommand_message_member_array,  // message members
  MotorCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorCommand_message_members,
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
get_message_type_support_handle<astribot_msgs::msg::MotorCommand>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::MotorCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, MotorCommand)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::MotorCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
