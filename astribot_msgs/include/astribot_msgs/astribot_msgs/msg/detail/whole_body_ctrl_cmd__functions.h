// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__FUNCTIONS_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "astribot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__struct.h"

/// Initialize msg/WholeBodyCtrlCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__msg__WholeBodyCtrlCmd
 * )) before or use
 * astribot_msgs__msg__WholeBodyCtrlCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__WholeBodyCtrlCmd__init(astribot_msgs__msg__WholeBodyCtrlCmd * msg);

/// Finalize msg/WholeBodyCtrlCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__WholeBodyCtrlCmd__fini(astribot_msgs__msg__WholeBodyCtrlCmd * msg);

/// Create msg/WholeBodyCtrlCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__msg__WholeBodyCtrlCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__WholeBodyCtrlCmd *
astribot_msgs__msg__WholeBodyCtrlCmd__create();

/// Destroy msg/WholeBodyCtrlCmd message.
/**
 * It calls
 * astribot_msgs__msg__WholeBodyCtrlCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__WholeBodyCtrlCmd__destroy(astribot_msgs__msg__WholeBodyCtrlCmd * msg);

/// Check for msg/WholeBodyCtrlCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__WholeBodyCtrlCmd__are_equal(const astribot_msgs__msg__WholeBodyCtrlCmd * lhs, const astribot_msgs__msg__WholeBodyCtrlCmd * rhs);

/// Copy a msg/WholeBodyCtrlCmd message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__WholeBodyCtrlCmd__copy(
  const astribot_msgs__msg__WholeBodyCtrlCmd * input,
  astribot_msgs__msg__WholeBodyCtrlCmd * output);

/// Initialize array of msg/WholeBodyCtrlCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__msg__WholeBodyCtrlCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__init(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array, size_t size);

/// Finalize array of msg/WholeBodyCtrlCmd messages.
/**
 * It calls
 * astribot_msgs__msg__WholeBodyCtrlCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__fini(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array);

/// Create array of msg/WholeBodyCtrlCmd messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence *
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__create(size_t size);

/// Destroy array of msg/WholeBodyCtrlCmd messages.
/**
 * It calls
 * astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__destroy(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array);

/// Check for msg/WholeBodyCtrlCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__are_equal(const astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * lhs, const astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * rhs);

/// Copy an array of msg/WholeBodyCtrlCmd messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__copy(
  const astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * input,
  astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__WHOLE_BODY_CTRL_CMD__FUNCTIONS_H_
