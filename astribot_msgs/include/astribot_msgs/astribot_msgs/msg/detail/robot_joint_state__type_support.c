// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:msg/RobotJointState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/msg/detail/robot_joint_state__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/msg/detail/robot_joint_state__functions.h"
#include "astribot_msgs/msg/detail/robot_joint_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
// Member `torque`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__msg__RobotJointState__init(message_memory);
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_fini_function(void * message_memory)
{
  astribot_msgs__msg__RobotJointState__fini(message_memory);
}

size_t astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__acceleration(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__acceleration(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__acceleration(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__acceleration(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, name),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__name,  // size() function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__name,  // get_const(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__name,  // get(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__name,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__name,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__name  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, position),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__position,  // size() function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__position,  // get_const(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__position,  // get(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__position,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__position,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, velocity),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__velocity,  // size() function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__velocity,  // get_const(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__velocity,  // get(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__velocity,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__velocity,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__velocity  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, acceleration),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__acceleration,  // size() function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__acceleration,  // get_const(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__acceleration,  // get(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__acceleration,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__acceleration,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__acceleration  // resize(index) function pointer
  },
  {
    "torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__msg__RobotJointState, torque),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__size_function__RobotJointState__torque,  // size() function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_const_function__RobotJointState__torque,  // get_const(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__get_function__RobotJointState__torque,  // get(index) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__fetch_function__RobotJointState__torque,  // fetch(index, &value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__assign_function__RobotJointState__torque,  // assign(index, value) function pointer
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__resize_function__RobotJointState__torque  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_members = {
  "astribot_msgs__msg",  // message namespace
  "RobotJointState",  // message name
  7,  // number of fields
  sizeof(astribot_msgs__msg__RobotJointState),
  astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_member_array,  // message members
  astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_type_support_handle = {
  0,
  &astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, msg, RobotJointState)() {
  astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__msg__RobotJointState__rosidl_typesupport_introspection_c__RobotJointState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
