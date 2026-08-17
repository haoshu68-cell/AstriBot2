// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/AstribotHeader.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/astribot_header__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
astribot_msgs__msg__AstribotHeader__init(astribot_msgs__msg__AstribotHeader * msg)
{
  if (!msg) {
    return false;
  }
  // seq
  // time_meas
  // time_pub
  return true;
}

void
astribot_msgs__msg__AstribotHeader__fini(astribot_msgs__msg__AstribotHeader * msg)
{
  if (!msg) {
    return;
  }
  // seq
  // time_meas
  // time_pub
}

bool
astribot_msgs__msg__AstribotHeader__are_equal(const astribot_msgs__msg__AstribotHeader * lhs, const astribot_msgs__msg__AstribotHeader * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // seq
  if (lhs->seq != rhs->seq) {
    return false;
  }
  // time_meas
  if (lhs->time_meas != rhs->time_meas) {
    return false;
  }
  // time_pub
  if (lhs->time_pub != rhs->time_pub) {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__AstribotHeader__copy(
  const astribot_msgs__msg__AstribotHeader * input,
  astribot_msgs__msg__AstribotHeader * output)
{
  if (!input || !output) {
    return false;
  }
  // seq
  output->seq = input->seq;
  // time_meas
  output->time_meas = input->time_meas;
  // time_pub
  output->time_pub = input->time_pub;
  return true;
}

astribot_msgs__msg__AstribotHeader *
astribot_msgs__msg__AstribotHeader__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__AstribotHeader * msg = (astribot_msgs__msg__AstribotHeader *)allocator.allocate(sizeof(astribot_msgs__msg__AstribotHeader), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__AstribotHeader));
  bool success = astribot_msgs__msg__AstribotHeader__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__AstribotHeader__destroy(astribot_msgs__msg__AstribotHeader * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__AstribotHeader__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__AstribotHeader__Sequence__init(astribot_msgs__msg__AstribotHeader__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__AstribotHeader * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__AstribotHeader *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__AstribotHeader), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__AstribotHeader__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__AstribotHeader__fini(&data[i - 1]);
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
astribot_msgs__msg__AstribotHeader__Sequence__fini(astribot_msgs__msg__AstribotHeader__Sequence * array)
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
      astribot_msgs__msg__AstribotHeader__fini(&array->data[i]);
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

astribot_msgs__msg__AstribotHeader__Sequence *
astribot_msgs__msg__AstribotHeader__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__AstribotHeader__Sequence * array = (astribot_msgs__msg__AstribotHeader__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__AstribotHeader__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__AstribotHeader__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__AstribotHeader__Sequence__destroy(astribot_msgs__msg__AstribotHeader__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__AstribotHeader__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__AstribotHeader__Sequence__are_equal(const astribot_msgs__msg__AstribotHeader__Sequence * lhs, const astribot_msgs__msg__AstribotHeader__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__AstribotHeader__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__AstribotHeader__Sequence__copy(
  const astribot_msgs__msg__AstribotHeader__Sequence * input,
  astribot_msgs__msg__AstribotHeader__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__AstribotHeader);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__AstribotHeader * data =
      (astribot_msgs__msg__AstribotHeader *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__AstribotHeader__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__AstribotHeader__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__AstribotHeader__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
