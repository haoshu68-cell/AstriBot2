// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/LaunchRequest.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/launch_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
astribot_msgs__msg__LaunchRequest__init(astribot_msgs__msg__LaunchRequest * msg)
{
  if (!msg) {
    return false;
  }
  // launch_target
  // launch_action
  return true;
}

void
astribot_msgs__msg__LaunchRequest__fini(astribot_msgs__msg__LaunchRequest * msg)
{
  if (!msg) {
    return;
  }
  // launch_target
  // launch_action
}

bool
astribot_msgs__msg__LaunchRequest__are_equal(const astribot_msgs__msg__LaunchRequest * lhs, const astribot_msgs__msg__LaunchRequest * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // launch_target
  if (lhs->launch_target != rhs->launch_target) {
    return false;
  }
  // launch_action
  if (lhs->launch_action != rhs->launch_action) {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__LaunchRequest__copy(
  const astribot_msgs__msg__LaunchRequest * input,
  astribot_msgs__msg__LaunchRequest * output)
{
  if (!input || !output) {
    return false;
  }
  // launch_target
  output->launch_target = input->launch_target;
  // launch_action
  output->launch_action = input->launch_action;
  return true;
}

astribot_msgs__msg__LaunchRequest *
astribot_msgs__msg__LaunchRequest__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__LaunchRequest * msg = (astribot_msgs__msg__LaunchRequest *)allocator.allocate(sizeof(astribot_msgs__msg__LaunchRequest), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__LaunchRequest));
  bool success = astribot_msgs__msg__LaunchRequest__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__LaunchRequest__destroy(astribot_msgs__msg__LaunchRequest * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__LaunchRequest__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__LaunchRequest__Sequence__init(astribot_msgs__msg__LaunchRequest__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__LaunchRequest * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__LaunchRequest *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__LaunchRequest), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__LaunchRequest__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__LaunchRequest__fini(&data[i - 1]);
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
astribot_msgs__msg__LaunchRequest__Sequence__fini(astribot_msgs__msg__LaunchRequest__Sequence * array)
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
      astribot_msgs__msg__LaunchRequest__fini(&array->data[i]);
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

astribot_msgs__msg__LaunchRequest__Sequence *
astribot_msgs__msg__LaunchRequest__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__LaunchRequest__Sequence * array = (astribot_msgs__msg__LaunchRequest__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__LaunchRequest__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__LaunchRequest__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__LaunchRequest__Sequence__destroy(astribot_msgs__msg__LaunchRequest__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__LaunchRequest__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__LaunchRequest__Sequence__are_equal(const astribot_msgs__msg__LaunchRequest__Sequence * lhs, const astribot_msgs__msg__LaunchRequest__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__LaunchRequest__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__LaunchRequest__Sequence__copy(
  const astribot_msgs__msg__LaunchRequest__Sequence * input,
  astribot_msgs__msg__LaunchRequest__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__LaunchRequest);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__LaunchRequest * data =
      (astribot_msgs__msg__LaunchRequest *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__LaunchRequest__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__LaunchRequest__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__LaunchRequest__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
