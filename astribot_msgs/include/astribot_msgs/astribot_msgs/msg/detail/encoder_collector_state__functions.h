// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__FUNCTIONS_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "astribot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "astribot_msgs/msg/detail/encoder_collector_state__struct.h"

/// Initialize msg/EncoderCollectorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__msg__EncoderCollectorState
 * )) before or use
 * astribot_msgs__msg__EncoderCollectorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__EncoderCollectorState__init(astribot_msgs__msg__EncoderCollectorState * msg);

/// Finalize msg/EncoderCollectorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__EncoderCollectorState__fini(astribot_msgs__msg__EncoderCollectorState * msg);

/// Create msg/EncoderCollectorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__msg__EncoderCollectorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__EncoderCollectorState *
astribot_msgs__msg__EncoderCollectorState__create();

/// Destroy msg/EncoderCollectorState message.
/**
 * It calls
 * astribot_msgs__msg__EncoderCollectorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__EncoderCollectorState__destroy(astribot_msgs__msg__EncoderCollectorState * msg);

/// Check for msg/EncoderCollectorState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__EncoderCollectorState__are_equal(const astribot_msgs__msg__EncoderCollectorState * lhs, const astribot_msgs__msg__EncoderCollectorState * rhs);

/// Copy a msg/EncoderCollectorState message.
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
astribot_msgs__msg__EncoderCollectorState__copy(
  const astribot_msgs__msg__EncoderCollectorState * input,
  astribot_msgs__msg__EncoderCollectorState * output);

/// Initialize array of msg/EncoderCollectorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__msg__EncoderCollectorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__EncoderCollectorState__Sequence__init(astribot_msgs__msg__EncoderCollectorState__Sequence * array, size_t size);

/// Finalize array of msg/EncoderCollectorState messages.
/**
 * It calls
 * astribot_msgs__msg__EncoderCollectorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__EncoderCollectorState__Sequence__fini(astribot_msgs__msg__EncoderCollectorState__Sequence * array);

/// Create array of msg/EncoderCollectorState messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__msg__EncoderCollectorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__EncoderCollectorState__Sequence *
astribot_msgs__msg__EncoderCollectorState__Sequence__create(size_t size);

/// Destroy array of msg/EncoderCollectorState messages.
/**
 * It calls
 * astribot_msgs__msg__EncoderCollectorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__EncoderCollectorState__Sequence__destroy(astribot_msgs__msg__EncoderCollectorState__Sequence * array);

/// Check for msg/EncoderCollectorState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__EncoderCollectorState__Sequence__are_equal(const astribot_msgs__msg__EncoderCollectorState__Sequence * lhs, const astribot_msgs__msg__EncoderCollectorState__Sequence * rhs);

/// Copy an array of msg/EncoderCollectorState messages.
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
astribot_msgs__msg__EncoderCollectorState__Sequence__copy(
  const astribot_msgs__msg__EncoderCollectorState__Sequence * input,
  astribot_msgs__msg__EncoderCollectorState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__ENCODER_COLLECTOR_STATE__FUNCTIONS_H_
