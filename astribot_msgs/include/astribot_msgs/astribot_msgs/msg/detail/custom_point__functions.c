// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/CustomPoint.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/custom_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
astribot_msgs__msg__CustomPoint__init(astribot_msgs__msg__CustomPoint * msg)
{
  if (!msg) {
    return false;
  }
  // offset_time
  // x
  // y
  // z
  // reflectivity
  // tag
  // line
  return true;
}

void
astribot_msgs__msg__CustomPoint__fini(astribot_msgs__msg__CustomPoint * msg)
{
  if (!msg) {
    return;
  }
  // offset_time
  // x
  // y
  // z
  // reflectivity
  // tag
  // line
}

bool
astribot_msgs__msg__CustomPoint__are_equal(const astribot_msgs__msg__CustomPoint * lhs, const astribot_msgs__msg__CustomPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // offset_time
  if (lhs->offset_time != rhs->offset_time) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // reflectivity
  if (lhs->reflectivity != rhs->reflectivity) {
    return false;
  }
  // tag
  if (lhs->tag != rhs->tag) {
    return false;
  }
  // line
  if (lhs->line != rhs->line) {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__CustomPoint__copy(
  const astribot_msgs__msg__CustomPoint * input,
  astribot_msgs__msg__CustomPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // offset_time
  output->offset_time = input->offset_time;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // reflectivity
  output->reflectivity = input->reflectivity;
  // tag
  output->tag = input->tag;
  // line
  output->line = input->line;
  return true;
}

astribot_msgs__msg__CustomPoint *
astribot_msgs__msg__CustomPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__CustomPoint * msg = (astribot_msgs__msg__CustomPoint *)allocator.allocate(sizeof(astribot_msgs__msg__CustomPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__CustomPoint));
  bool success = astribot_msgs__msg__CustomPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__CustomPoint__destroy(astribot_msgs__msg__CustomPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__CustomPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__CustomPoint__Sequence__init(astribot_msgs__msg__CustomPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__CustomPoint * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__CustomPoint *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__CustomPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__CustomPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__CustomPoint__fini(&data[i - 1]);
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
astribot_msgs__msg__CustomPoint__Sequence__fini(astribot_msgs__msg__CustomPoint__Sequence * array)
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
      astribot_msgs__msg__CustomPoint__fini(&array->data[i]);
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

astribot_msgs__msg__CustomPoint__Sequence *
astribot_msgs__msg__CustomPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__CustomPoint__Sequence * array = (astribot_msgs__msg__CustomPoint__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__CustomPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__CustomPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__CustomPoint__Sequence__destroy(astribot_msgs__msg__CustomPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__CustomPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__CustomPoint__Sequence__are_equal(const astribot_msgs__msg__CustomPoint__Sequence * lhs, const astribot_msgs__msg__CustomPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__CustomPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__CustomPoint__Sequence__copy(
  const astribot_msgs__msg__CustomPoint__Sequence * input,
  astribot_msgs__msg__CustomPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__CustomPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__CustomPoint * data =
      (astribot_msgs__msg__CustomPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__CustomPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__CustomPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__CustomPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
