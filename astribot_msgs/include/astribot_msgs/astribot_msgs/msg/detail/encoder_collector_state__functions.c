// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/encoder_collector_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
astribot_msgs__msg__EncoderCollectorState__init(astribot_msgs__msg__EncoderCollectorState * msg)
{
  if (!msg) {
    return false;
  }
  // position_rad
  // velocity_rps
  // acceleration_rpss
  // rx_sequence_count
  return true;
}

void
astribot_msgs__msg__EncoderCollectorState__fini(astribot_msgs__msg__EncoderCollectorState * msg)
{
  if (!msg) {
    return;
  }
  // position_rad
  // velocity_rps
  // acceleration_rpss
  // rx_sequence_count
}

bool
astribot_msgs__msg__EncoderCollectorState__are_equal(const astribot_msgs__msg__EncoderCollectorState * lhs, const astribot_msgs__msg__EncoderCollectorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position_rad
  if (lhs->position_rad != rhs->position_rad) {
    return false;
  }
  // velocity_rps
  if (lhs->velocity_rps != rhs->velocity_rps) {
    return false;
  }
  // acceleration_rpss
  if (lhs->acceleration_rpss != rhs->acceleration_rpss) {
    return false;
  }
  // rx_sequence_count
  if (lhs->rx_sequence_count != rhs->rx_sequence_count) {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__EncoderCollectorState__copy(
  const astribot_msgs__msg__EncoderCollectorState * input,
  astribot_msgs__msg__EncoderCollectorState * output)
{
  if (!input || !output) {
    return false;
  }
  // position_rad
  output->position_rad = input->position_rad;
  // velocity_rps
  output->velocity_rps = input->velocity_rps;
  // acceleration_rpss
  output->acceleration_rpss = input->acceleration_rpss;
  // rx_sequence_count
  output->rx_sequence_count = input->rx_sequence_count;
  return true;
}

astribot_msgs__msg__EncoderCollectorState *
astribot_msgs__msg__EncoderCollectorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__EncoderCollectorState * msg = (astribot_msgs__msg__EncoderCollectorState *)allocator.allocate(sizeof(astribot_msgs__msg__EncoderCollectorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__EncoderCollectorState));
  bool success = astribot_msgs__msg__EncoderCollectorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__EncoderCollectorState__destroy(astribot_msgs__msg__EncoderCollectorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__EncoderCollectorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__EncoderCollectorState__Sequence__init(astribot_msgs__msg__EncoderCollectorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__EncoderCollectorState * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__EncoderCollectorState *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__EncoderCollectorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__EncoderCollectorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__EncoderCollectorState__fini(&data[i - 1]);
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
astribot_msgs__msg__EncoderCollectorState__Sequence__fini(astribot_msgs__msg__EncoderCollectorState__Sequence * array)
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
      astribot_msgs__msg__EncoderCollectorState__fini(&array->data[i]);
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

astribot_msgs__msg__EncoderCollectorState__Sequence *
astribot_msgs__msg__EncoderCollectorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__EncoderCollectorState__Sequence * array = (astribot_msgs__msg__EncoderCollectorState__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__EncoderCollectorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__EncoderCollectorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__EncoderCollectorState__Sequence__destroy(astribot_msgs__msg__EncoderCollectorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__EncoderCollectorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__EncoderCollectorState__Sequence__are_equal(const astribot_msgs__msg__EncoderCollectorState__Sequence * lhs, const astribot_msgs__msg__EncoderCollectorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__EncoderCollectorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__EncoderCollectorState__Sequence__copy(
  const astribot_msgs__msg__EncoderCollectorState__Sequence * input,
  astribot_msgs__msg__EncoderCollectorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__EncoderCollectorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__EncoderCollectorState * data =
      (astribot_msgs__msg__EncoderCollectorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__EncoderCollectorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__EncoderCollectorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__EncoderCollectorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
