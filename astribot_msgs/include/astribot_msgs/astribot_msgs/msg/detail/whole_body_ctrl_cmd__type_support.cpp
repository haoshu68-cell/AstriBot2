// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__struct.hpp"
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

void WholeBodyCtrlCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astribot_msgs::msg::WholeBodyCtrlCmd(_init);
}

void WholeBodyCtrlCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astribot_msgs::msg::WholeBodyCtrlCmd *>(message_memory);
  typed_message->~WholeBodyCtrlCmd();
}

size_t size_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__WholeBodyCtrlCmd__pose_world_to_torso_desired(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__WholeBodyCtrlCmd__pose_world_to_chassis_current(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember WholeBodyCtrlCmd_message_member_array[12] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<astribot_msgs::msg::AstribotHeader>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_arm_twist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Twist>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, left_arm_twist),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_arm_twist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Twist>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, right_arm_twist),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "torso_twist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Twist>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, torso_twist),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose_world_to_torso_desired",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, pose_world_to_torso_desired),  // bytes offset in struct
    nullptr,  // default value
    size_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // size() function pointer
    get_const_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // get_const(index) function pointer
    get_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // get(index) function pointer
    fetch_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // fetch(index, &value) function pointer
    assign_function__WholeBodyCtrlCmd__pose_world_to_torso_desired,  // assign(index, value) function pointer
    resize_function__WholeBodyCtrlCmd__pose_world_to_torso_desired  // resize(index) function pointer
  },
  {
    "pose_world_to_left_arm_desired",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, pose_world_to_left_arm_desired),  // bytes offset in struct
    nullptr,  // default value
    size_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // size() function pointer
    get_const_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // get_const(index) function pointer
    get_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // get(index) function pointer
    fetch_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // fetch(index, &value) function pointer
    assign_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired,  // assign(index, value) function pointer
    resize_function__WholeBodyCtrlCmd__pose_world_to_left_arm_desired  // resize(index) function pointer
  },
  {
    "pose_world_to_right_arm_desired",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, pose_world_to_right_arm_desired),  // bytes offset in struct
    nullptr,  // default value
    size_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // size() function pointer
    get_const_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // get_const(index) function pointer
    get_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // get(index) function pointer
    fetch_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // fetch(index, &value) function pointer
    assign_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired,  // assign(index, value) function pointer
    resize_function__WholeBodyCtrlCmd__pose_world_to_right_arm_desired  // resize(index) function pointer
  },
  {
    "pose_world_to_chassis_current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, pose_world_to_chassis_current),  // bytes offset in struct
    nullptr,  // default value
    size_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // size() function pointer
    get_const_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // get_const(index) function pointer
    get_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // get(index) function pointer
    fetch_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // fetch(index, &value) function pointer
    assign_function__WholeBodyCtrlCmd__pose_world_to_chassis_current,  // assign(index, value) function pointer
    resize_function__WholeBodyCtrlCmd__pose_world_to_chassis_current  // resize(index) function pointer
  },
  {
    "torso_open_loop",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, torso_open_loop),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "enable_collision_avoidance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, enable_collision_avoidance),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "smooth_t",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, smooth_t),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "smooth_duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs::msg::WholeBodyCtrlCmd, smooth_duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers WholeBodyCtrlCmd_message_members = {
  "astribot_msgs::msg",  // message namespace
  "WholeBodyCtrlCmd",  // message name
  12,  // number of fields
  sizeof(astribot_msgs::msg::WholeBodyCtrlCmd),
  WholeBodyCtrlCmd_message_member_array,  // message members
  WholeBodyCtrlCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  WholeBodyCtrlCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t WholeBodyCtrlCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &WholeBodyCtrlCmd_message_members,
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
get_message_type_support_handle<astribot_msgs::msg::WholeBodyCtrlCmd>()
{
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::WholeBodyCtrlCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astribot_msgs, msg, WholeBodyCtrlCmd)() {
  return &::astribot_msgs::msg::rosidl_typesupport_introspection_cpp::WholeBodyCtrlCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
