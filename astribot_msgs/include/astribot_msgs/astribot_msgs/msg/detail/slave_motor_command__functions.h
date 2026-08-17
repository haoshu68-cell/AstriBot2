// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from astribot_msgs:msg/SlaveMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__FUNCTIONS_H_
#define ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "astribot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "astribot_msgs/msg/detail/slave_motor_command__struct.h"

/// Initialize msg/SlaveMotorCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__msg__SlaveMotorCommand
 * )) before or use
 * astribot_msgs__msg__SlaveMotorCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__SlaveMotorCommand__init(astribot_msgs__msg__SlaveMotorCommand * msg);

/// Finalize msg/SlaveMotorCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__SlaveMotorCommand__fini(astribot_msgs__msg__SlaveMotorCommand * msg);

/// Create msg/SlaveMotorCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__msg__SlaveMotorCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__SlaveMotorCommand *
astribot_msgs__msg__SlaveMotorCommand__create();

/// Destroy msg/SlaveMotorCommand message.
/**
 * It calls
 * astribot_msgs__msg__SlaveMotorCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__SlaveMotorCommand__destroy(astribot_msgs__msg__SlaveMotorCommand * msg);

/// Check for msg/SlaveMotorCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__SlaveMotorCommand__are_equal(const astribot_msgs__msg__SlaveMotorCommand * lhs, const astribot_msgs__msg__SlaveMotorCommand * rhs);

/// Copy a msg/SlaveMotorCommand message.
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
astribot_msgs__msg__SlaveMotorCommand__copy(
  const astribot_msgs__msg__SlaveMotorCommand * input,
  astribot_msgs__msg__SlaveMotorCommand * output);

/// Initialize array of msg/SlaveMotorCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__msg__SlaveMotorCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__SlaveMotorCommand__Sequence__init(astribot_msgs__msg__SlaveMotorCommand__Sequence * array, size_t size);

/// Finalize array of msg/SlaveMotorCommand messages.
/**
 * It calls
 * astribot_msgs__msg__SlaveMotorCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__SlaveMotorCommand__Sequence__fini(astribot_msgs__msg__SlaveMotorCommand__Sequence * array);

/// Create array of msg/SlaveMotorCommand messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__msg__SlaveMotorCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__msg__SlaveMotorCommand__Sequence *
astribot_msgs__msg__SlaveMotorCommand__Sequence__create(size_t size);

/// Destroy array of msg/SlaveMotorCommand messages.
/**
 * It calls
 * astribot_msgs__msg__SlaveMotorCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__msg__SlaveMotorCommand__Sequence__destroy(astribot_msgs__msg__SlaveMotorCommand__Sequence * array);

/// Check for msg/SlaveMotorCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__msg__SlaveMotorCommand__Sequence__are_equal(const astribot_msgs__msg__SlaveMotorCommand__Sequence * lhs, const astribot_msgs__msg__SlaveMotorCommand__Sequence * rhs);

/// Copy an array of msg/SlaveMotorCommand messages.
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
astribot_msgs__msg__SlaveMotorCommand__Sequence__copy(
  const astribot_msgs__msg__SlaveMotorCommand__Sequence * input,
  astribot_msgs__msg__SlaveMotorCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__MSG__DETAIL__SLAVE_MOTOR_COMMAND__FUNCTIONS_H_
