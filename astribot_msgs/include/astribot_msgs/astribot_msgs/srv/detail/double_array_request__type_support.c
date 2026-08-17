// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astribot_msgs:srv/DoubleArrayRequest.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astribot_msgs/srv/detail/double_array_request__rosidl_typesupport_introspection_c.h"
#include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astribot_msgs/srv/detail/double_array_request__functions.h"
#include "astribot_msgs/srv/detail/double_array_request__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__srv__DoubleArrayRequest_Request__init(message_memory);
}

void astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_fini_function(void * message_memory)
{
  astribot_msgs__srv__DoubleArrayRequest_Request__fini(message_memory);
}

size_t astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__size_function__DoubleArrayRequest_Request__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__get_const_function__DoubleArrayRequest_Request__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__get_function__DoubleArrayRequest_Request__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__fetch_function__DoubleArrayRequest_Request__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__get_const_function__DoubleArrayRequest_Request__data(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__assign_function__DoubleArrayRequest_Request__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__get_function__DoubleArrayRequest_Request__data(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__resize_function__DoubleArrayRequest_Request__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__srv__DoubleArrayRequest_Request, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__srv__DoubleArrayRequest_Request, data),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__size_function__DoubleArrayRequest_Request__data,  // size() function pointer
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__get_const_function__DoubleArrayRequest_Request__data,  // get_const(index) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__get_function__DoubleArrayRequest_Request__data,  // get(index) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__fetch_function__DoubleArrayRequest_Request__data,  // fetch(index, &value) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__assign_function__DoubleArrayRequest_Request__data,  // assign(index, value) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__resize_function__DoubleArrayRequest_Request__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_members = {
  "astribot_msgs__srv",  // message namespace
  "DoubleArrayRequest_Request",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__srv__DoubleArrayRequest_Request),
  astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_member_array,  // message members
  astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_type_support_handle = {
  0,
  &astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest_Request)() {
  astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__srv__DoubleArrayRequest_Request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "astribot_msgs/srv/detail/double_array_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "astribot_msgs/srv/detail/double_array_request__functions.h"
// already included above
// #include "astribot_msgs/srv/detail/double_array_request__struct.h"


// Include directives for member types
// Member `header`
// already included above
// #include "std_msgs/msg/header.h"
// Member `header`
// already included above
// #include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astribot_msgs__srv__DoubleArrayRequest_Response__init(message_memory);
}

void astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_fini_function(void * message_memory)
{
  astribot_msgs__srv__DoubleArrayRequest_Response__fini(message_memory);
}

size_t astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__size_function__DoubleArrayRequest_Response__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__get_const_function__DoubleArrayRequest_Response__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__get_function__DoubleArrayRequest_Response__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__fetch_function__DoubleArrayRequest_Response__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__get_const_function__DoubleArrayRequest_Response__data(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__assign_function__DoubleArrayRequest_Response__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__get_function__DoubleArrayRequest_Response__data(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__resize_function__DoubleArrayRequest_Response__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__srv__DoubleArrayRequest_Response, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astribot_msgs__srv__DoubleArrayRequest_Response, data),  // bytes offset in struct
    NULL,  // default value
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__size_function__DoubleArrayRequest_Response__data,  // size() function pointer
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__get_const_function__DoubleArrayRequest_Response__data,  // get_const(index) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__get_function__DoubleArrayRequest_Response__data,  // get(index) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__fetch_function__DoubleArrayRequest_Response__data,  // fetch(index, &value) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__assign_function__DoubleArrayRequest_Response__data,  // assign(index, value) function pointer
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__resize_function__DoubleArrayRequest_Response__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_members = {
  "astribot_msgs__srv",  // message namespace
  "DoubleArrayRequest_Response",  // message name
  2,  // number of fields
  sizeof(astribot_msgs__srv__DoubleArrayRequest_Response),
  astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_member_array,  // message members
  astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_type_support_handle = {
  0,
  &astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest_Response)() {
  astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_type_support_handle.typesupport_identifier) {
    astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astribot_msgs__srv__DoubleArrayRequest_Response__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "astribot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "astribot_msgs/srv/detail/double_array_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_members = {
  "astribot_msgs__srv",  // service namespace
  "DoubleArrayRequest",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Request_message_type_support_handle,
  NULL  // response message
  // astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_Response_message_type_support_handle
};

static rosidl_service_type_support_t astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_type_support_handle = {
  0,
  &astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astribot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest)() {
  if (!astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_type_support_handle.typesupport_identifier) {
    astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astribot_msgs, srv, DoubleArrayRequest_Response)()->data;
  }

  return &astribot_msgs__srv__detail__double_array_request__rosidl_typesupport_introspection_c__DoubleArrayRequest_service_type_support_handle;
}
