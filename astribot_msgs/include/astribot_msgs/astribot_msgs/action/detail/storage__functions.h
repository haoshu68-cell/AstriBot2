// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from astribot_msgs:action/Storage.idl
// generated code does not contain a copyright notice

#ifndef ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__FUNCTIONS_H_
#define ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "astribot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "astribot_msgs/action/detail/storage__struct.h"

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_Goal
 * )) before or use
 * astribot_msgs__action__Storage_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Goal__init(astribot_msgs__action__Storage_Goal * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Goal__fini(astribot_msgs__action__Storage_Goal * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_Goal *
astribot_msgs__action__Storage_Goal__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Goal__destroy(astribot_msgs__action__Storage_Goal * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Goal__are_equal(const astribot_msgs__action__Storage_Goal * lhs, const astribot_msgs__action__Storage_Goal * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_Goal__copy(
  const astribot_msgs__action__Storage_Goal * input,
  astribot_msgs__action__Storage_Goal * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Goal__Sequence__init(astribot_msgs__action__Storage_Goal__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Goal__Sequence__fini(astribot_msgs__action__Storage_Goal__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_Goal__Sequence *
astribot_msgs__action__Storage_Goal__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Goal__Sequence__destroy(astribot_msgs__action__Storage_Goal__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Goal__Sequence__are_equal(const astribot_msgs__action__Storage_Goal__Sequence * lhs, const astribot_msgs__action__Storage_Goal__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_Goal__Sequence__copy(
  const astribot_msgs__action__Storage_Goal__Sequence * input,
  astribot_msgs__action__Storage_Goal__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_Result
 * )) before or use
 * astribot_msgs__action__Storage_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Result__init(astribot_msgs__action__Storage_Result * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Result__fini(astribot_msgs__action__Storage_Result * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_Result *
astribot_msgs__action__Storage_Result__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Result__destroy(astribot_msgs__action__Storage_Result * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Result__are_equal(const astribot_msgs__action__Storage_Result * lhs, const astribot_msgs__action__Storage_Result * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_Result__copy(
  const astribot_msgs__action__Storage_Result * input,
  astribot_msgs__action__Storage_Result * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Result__Sequence__init(astribot_msgs__action__Storage_Result__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Result__Sequence__fini(astribot_msgs__action__Storage_Result__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_Result__Sequence *
astribot_msgs__action__Storage_Result__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Result__Sequence__destroy(astribot_msgs__action__Storage_Result__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Result__Sequence__are_equal(const astribot_msgs__action__Storage_Result__Sequence * lhs, const astribot_msgs__action__Storage_Result__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_Result__Sequence__copy(
  const astribot_msgs__action__Storage_Result__Sequence * input,
  astribot_msgs__action__Storage_Result__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_Feedback
 * )) before or use
 * astribot_msgs__action__Storage_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Feedback__init(astribot_msgs__action__Storage_Feedback * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Feedback__fini(astribot_msgs__action__Storage_Feedback * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_Feedback *
astribot_msgs__action__Storage_Feedback__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Feedback__destroy(astribot_msgs__action__Storage_Feedback * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Feedback__are_equal(const astribot_msgs__action__Storage_Feedback * lhs, const astribot_msgs__action__Storage_Feedback * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_Feedback__copy(
  const astribot_msgs__action__Storage_Feedback * input,
  astribot_msgs__action__Storage_Feedback * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Feedback__Sequence__init(astribot_msgs__action__Storage_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Feedback__Sequence__fini(astribot_msgs__action__Storage_Feedback__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_Feedback__Sequence *
astribot_msgs__action__Storage_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_Feedback__Sequence__destroy(astribot_msgs__action__Storage_Feedback__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_Feedback__Sequence__are_equal(const astribot_msgs__action__Storage_Feedback__Sequence * lhs, const astribot_msgs__action__Storage_Feedback__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_Feedback__Sequence__copy(
  const astribot_msgs__action__Storage_Feedback__Sequence * input,
  astribot_msgs__action__Storage_Feedback__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_SendGoal_Request
 * )) before or use
 * astribot_msgs__action__Storage_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Request__init(astribot_msgs__action__Storage_SendGoal_Request * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Request__fini(astribot_msgs__action__Storage_SendGoal_Request * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_SendGoal_Request *
astribot_msgs__action__Storage_SendGoal_Request__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Request__destroy(astribot_msgs__action__Storage_SendGoal_Request * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Request__are_equal(const astribot_msgs__action__Storage_SendGoal_Request * lhs, const astribot_msgs__action__Storage_SendGoal_Request * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_SendGoal_Request__copy(
  const astribot_msgs__action__Storage_SendGoal_Request * input,
  astribot_msgs__action__Storage_SendGoal_Request * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Request__Sequence__init(astribot_msgs__action__Storage_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Request__Sequence__fini(astribot_msgs__action__Storage_SendGoal_Request__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_SendGoal_Request__Sequence *
astribot_msgs__action__Storage_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Request__Sequence__destroy(astribot_msgs__action__Storage_SendGoal_Request__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Request__Sequence__are_equal(const astribot_msgs__action__Storage_SendGoal_Request__Sequence * lhs, const astribot_msgs__action__Storage_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_SendGoal_Request__Sequence__copy(
  const astribot_msgs__action__Storage_SendGoal_Request__Sequence * input,
  astribot_msgs__action__Storage_SendGoal_Request__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_SendGoal_Response
 * )) before or use
 * astribot_msgs__action__Storage_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Response__init(astribot_msgs__action__Storage_SendGoal_Response * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Response__fini(astribot_msgs__action__Storage_SendGoal_Response * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_SendGoal_Response *
astribot_msgs__action__Storage_SendGoal_Response__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Response__destroy(astribot_msgs__action__Storage_SendGoal_Response * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Response__are_equal(const astribot_msgs__action__Storage_SendGoal_Response * lhs, const astribot_msgs__action__Storage_SendGoal_Response * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_SendGoal_Response__copy(
  const astribot_msgs__action__Storage_SendGoal_Response * input,
  astribot_msgs__action__Storage_SendGoal_Response * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Response__Sequence__init(astribot_msgs__action__Storage_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Response__Sequence__fini(astribot_msgs__action__Storage_SendGoal_Response__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_SendGoal_Response__Sequence *
astribot_msgs__action__Storage_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_SendGoal_Response__Sequence__destroy(astribot_msgs__action__Storage_SendGoal_Response__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_SendGoal_Response__Sequence__are_equal(const astribot_msgs__action__Storage_SendGoal_Response__Sequence * lhs, const astribot_msgs__action__Storage_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_SendGoal_Response__Sequence__copy(
  const astribot_msgs__action__Storage_SendGoal_Response__Sequence * input,
  astribot_msgs__action__Storage_SendGoal_Response__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_GetResult_Request
 * )) before or use
 * astribot_msgs__action__Storage_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Request__init(astribot_msgs__action__Storage_GetResult_Request * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Request__fini(astribot_msgs__action__Storage_GetResult_Request * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_GetResult_Request *
astribot_msgs__action__Storage_GetResult_Request__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Request__destroy(astribot_msgs__action__Storage_GetResult_Request * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Request__are_equal(const astribot_msgs__action__Storage_GetResult_Request * lhs, const astribot_msgs__action__Storage_GetResult_Request * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_GetResult_Request__copy(
  const astribot_msgs__action__Storage_GetResult_Request * input,
  astribot_msgs__action__Storage_GetResult_Request * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Request__Sequence__init(astribot_msgs__action__Storage_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Request__Sequence__fini(astribot_msgs__action__Storage_GetResult_Request__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_GetResult_Request__Sequence *
astribot_msgs__action__Storage_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Request__Sequence__destroy(astribot_msgs__action__Storage_GetResult_Request__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Request__Sequence__are_equal(const astribot_msgs__action__Storage_GetResult_Request__Sequence * lhs, const astribot_msgs__action__Storage_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_GetResult_Request__Sequence__copy(
  const astribot_msgs__action__Storage_GetResult_Request__Sequence * input,
  astribot_msgs__action__Storage_GetResult_Request__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_GetResult_Response
 * )) before or use
 * astribot_msgs__action__Storage_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Response__init(astribot_msgs__action__Storage_GetResult_Response * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Response__fini(astribot_msgs__action__Storage_GetResult_Response * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_GetResult_Response *
astribot_msgs__action__Storage_GetResult_Response__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Response__destroy(astribot_msgs__action__Storage_GetResult_Response * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Response__are_equal(const astribot_msgs__action__Storage_GetResult_Response * lhs, const astribot_msgs__action__Storage_GetResult_Response * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_GetResult_Response__copy(
  const astribot_msgs__action__Storage_GetResult_Response * input,
  astribot_msgs__action__Storage_GetResult_Response * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Response__Sequence__init(astribot_msgs__action__Storage_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Response__Sequence__fini(astribot_msgs__action__Storage_GetResult_Response__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_GetResult_Response__Sequence *
astribot_msgs__action__Storage_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_GetResult_Response__Sequence__destroy(astribot_msgs__action__Storage_GetResult_Response__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_GetResult_Response__Sequence__are_equal(const astribot_msgs__action__Storage_GetResult_Response__Sequence * lhs, const astribot_msgs__action__Storage_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_GetResult_Response__Sequence__copy(
  const astribot_msgs__action__Storage_GetResult_Response__Sequence * input,
  astribot_msgs__action__Storage_GetResult_Response__Sequence * output);

/// Initialize action/Storage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astribot_msgs__action__Storage_FeedbackMessage
 * )) before or use
 * astribot_msgs__action__Storage_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_FeedbackMessage__init(astribot_msgs__action__Storage_FeedbackMessage * msg);

/// Finalize action/Storage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_FeedbackMessage__fini(astribot_msgs__action__Storage_FeedbackMessage * msg);

/// Create action/Storage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astribot_msgs__action__Storage_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_FeedbackMessage *
astribot_msgs__action__Storage_FeedbackMessage__create();

/// Destroy action/Storage message.
/**
 * It calls
 * astribot_msgs__action__Storage_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_FeedbackMessage__destroy(astribot_msgs__action__Storage_FeedbackMessage * msg);

/// Check for action/Storage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_FeedbackMessage__are_equal(const astribot_msgs__action__Storage_FeedbackMessage * lhs, const astribot_msgs__action__Storage_FeedbackMessage * rhs);

/// Copy a action/Storage message.
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
astribot_msgs__action__Storage_FeedbackMessage__copy(
  const astribot_msgs__action__Storage_FeedbackMessage * input,
  astribot_msgs__action__Storage_FeedbackMessage * output);

/// Initialize array of action/Storage messages.
/**
 * It allocates the memory for the number of elements and calls
 * astribot_msgs__action__Storage_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_FeedbackMessage__Sequence__init(astribot_msgs__action__Storage_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_FeedbackMessage__Sequence__fini(astribot_msgs__action__Storage_FeedbackMessage__Sequence * array);

/// Create array of action/Storage messages.
/**
 * It allocates the memory for the array and calls
 * astribot_msgs__action__Storage_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
astribot_msgs__action__Storage_FeedbackMessage__Sequence *
astribot_msgs__action__Storage_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Storage messages.
/**
 * It calls
 * astribot_msgs__action__Storage_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
void
astribot_msgs__action__Storage_FeedbackMessage__Sequence__destroy(astribot_msgs__action__Storage_FeedbackMessage__Sequence * array);

/// Check for action/Storage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astribot_msgs
bool
astribot_msgs__action__Storage_FeedbackMessage__Sequence__are_equal(const astribot_msgs__action__Storage_FeedbackMessage__Sequence * lhs, const astribot_msgs__action__Storage_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Storage messages.
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
astribot_msgs__action__Storage_FeedbackMessage__Sequence__copy(
  const astribot_msgs__action__Storage_FeedbackMessage__Sequence * input,
  astribot_msgs__action__Storage_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASTRIBOT_MSGS__ACTION__DETAIL__STORAGE__FUNCTIONS_H_
