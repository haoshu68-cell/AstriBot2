// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/RobotVisualStates.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/robot_visual_states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `id`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
astribot_msgs__msg__RobotVisualStates__init(astribot_msgs__msg__RobotVisualStates * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    astribot_msgs__msg__RobotVisualStates__fini(msg);
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__Sequence__init(&msg->id, 0)) {
    astribot_msgs__msg__RobotVisualStates__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->pose, 0)) {
    astribot_msgs__msg__RobotVisualStates__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__Sequence__init(&msg->twist, 0)) {
    astribot_msgs__msg__RobotVisualStates__fini(msg);
    return false;
  }
  return true;
}

void
astribot_msgs__msg__RobotVisualStates__fini(astribot_msgs__msg__RobotVisualStates * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  rosidl_runtime_c__String__Sequence__fini(&msg->id);
  // pose
  geometry_msgs__msg__Pose__Sequence__fini(&msg->pose);
  // twist
  geometry_msgs__msg__Twist__Sequence__fini(&msg->twist);
}

bool
astribot_msgs__msg__RobotVisualStates__are_equal(const astribot_msgs__msg__RobotVisualStates * lhs, const astribot_msgs__msg__RobotVisualStates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__Sequence__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__RobotVisualStates__copy(
  const astribot_msgs__msg__RobotVisualStates * input,
  astribot_msgs__msg__RobotVisualStates * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__Sequence__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  return true;
}

astribot_msgs__msg__RobotVisualStates *
astribot_msgs__msg__RobotVisualStates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__RobotVisualStates * msg = (astribot_msgs__msg__RobotVisualStates *)allocator.allocate(sizeof(astribot_msgs__msg__RobotVisualStates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__RobotVisualStates));
  bool success = astribot_msgs__msg__RobotVisualStates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__RobotVisualStates__destroy(astribot_msgs__msg__RobotVisualStates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__RobotVisualStates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__RobotVisualStates__Sequence__init(astribot_msgs__msg__RobotVisualStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__RobotVisualStates * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__RobotVisualStates *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__RobotVisualStates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__RobotVisualStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__RobotVisualStates__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
astribot_msgs__msg__RobotVisualStates__Sequence__fini(astribot_msgs__msg__RobotVisualStates__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      astribot_msgs__msg__RobotVisualStates__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

astribot_msgs__msg__RobotVisualStates__Sequence *
astribot_msgs__msg__RobotVisualStates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__RobotVisualStates__Sequence * array = (astribot_msgs__msg__RobotVisualStates__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__RobotVisualStates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__RobotVisualStates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__RobotVisualStates__Sequence__destroy(astribot_msgs__msg__RobotVisualStates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__RobotVisualStates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__RobotVisualStates__Sequence__are_equal(const astribot_msgs__msg__RobotVisualStates__Sequence * lhs, const astribot_msgs__msg__RobotVisualStates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__RobotVisualStates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__RobotVisualStates__Sequence__copy(
  const astribot_msgs__msg__RobotVisualStates__Sequence * input,
  astribot_msgs__msg__RobotVisualStates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__RobotVisualStates);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__RobotVisualStates * data =
      (astribot_msgs__msg__RobotVisualStates *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__RobotVisualStates__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__RobotVisualStates__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__RobotVisualStates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
