// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from astribot_msgs:msg/LivoxCustomMsg.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__FUNCTIONS_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "astribot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "astribot_msgs/msg/detail/livox_custom_msg__struct.h"

/// Initialize msg/LivoxCustomMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__msg__LivoxCustomMsg
 * )) before or use
 * astribot_msgs__msg__LivoxCustomMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__LivoxCustomMsg__init(astribot_msgs__msg__LivoxCustomMsg * msg);

/// Finalize msg/LivoxCustomMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__LivoxCustomMsg__fini(astribot_msgs__msg__LivoxCustomMsg * msg);

/// Create msg/LivoxCustomMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__msg__LivoxCustomMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__LivoxCustomMsg *
astribot_msgs__msg__LivoxCustomMsg__create();

/// Destroy msg/LivoxCustomMsg message.
/**
 * It calls
 * astribot_msgs__msg__LivoxCustomMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__LivoxCustomMsg__destroy(astribot_msgs__msg__LivoxCustomMsg * msg);

/// Check for msg/LivoxCustomMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__LivoxCustomMsg__are_equal(const astribot_msgs__msg__LivoxCustomMsg * lhs, const astribot_msgs__msg__LivoxCustomMsg * rhs);

/// Copy a msg/LivoxCustomMsg message.
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
astribot_msgs__msg__LivoxCustomMsg__copy(
  const astribot_msgs__msg__LivoxCustomMsg * input,
  astribot_msgs__msg__LivoxCustomMsg * output);

/// Initialize array of msg/LivoxCustomMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__msg__LivoxCustomMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__LivoxCustomMsg__Sequence__init(astribot_msgs__msg__LivoxCustomMsg__Sequence * array, size_t size);

/// Finalize array of msg/LivoxCustomMsg messages.
/**
 * It calls
 * astribot_msgs__msg__LivoxCustomMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__LivoxCustomMsg__Sequence__fini(astribot_msgs__msg__LivoxCustomMsg__Sequence * array);

/// Create array of msg/LivoxCustomMsg messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__msg__LivoxCustomMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__LivoxCustomMsg__Sequence *
astribot_msgs__msg__LivoxCustomMsg__Sequence__create(size_t size);

/// Destroy array of msg/LivoxCustomMsg messages.
/**
 * It calls
 * astribot_msgs__msg__LivoxCustomMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__LivoxCustomMsg__Sequence__destroy(astribot_msgs__msg__LivoxCustomMsg__Sequence * array);

/// Check for msg/LivoxCustomMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__LivoxCustomMsg__Sequence__are_equal(const astribot_msgs__msg__LivoxCustomMsg__Sequence * lhs, const astribot_msgs__msg__LivoxCustomMsg__Sequence * rhs);

/// Copy an array of msg/LivoxCustomMsg messages.
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
astribot_msgs__msg__LivoxCustomMsg__Sequence__copy(
  const astribot_msgs__msg__LivoxCustomMsg__Sequence * input,
  astribot_msgs__msg__LivoxCustomMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__LIVOX_CUSTOM_MSG__FUNCTIONS_H_
