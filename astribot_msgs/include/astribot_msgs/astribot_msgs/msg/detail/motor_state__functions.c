// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/motor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
astribot_msgs__msg__MotorState__init(astribot_msgs__msg__MotorState * msg)
{
  if (!msg) {
    return false;
  }
  // p
  // v
  // c
  // vol
  // acc
  // can_count
  // can_count_last
  // can_error
  // j_s
  // j_p
  // j_v
  // j_a
  return true;
}

void
astribot_msgs__msg__MotorState__fini(astribot_msgs__msg__MotorState * msg)
{
  if (!msg) {
    return;
  }
  // p
  // v
  // c
  // vol
  // acc
  // can_count
  // can_count_last
  // can_error
  // j_s
  // j_p
  // j_v
  // j_a
}

bool
astribot_msgs__msg__MotorState__are_equal(const astribot_msgs__msg__MotorState * lhs, const astribot_msgs__msg__MotorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // p
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->p[i] != rhs->p[i]) {
      return false;
    }
  }
  // v
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->v[i] != rhs->v[i]) {
      return false;
    }
  }
  // c
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->c[i] != rhs->c[i]) {
      return false;
    }
  }
  // vol
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->vol[i] != rhs->vol[i]) {
      return false;
    }
  }
  // acc
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->acc[i] != rhs->acc[i]) {
      return false;
    }
  }
  // can_count
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->can_count[i] != rhs->can_count[i]) {
      return false;
    }
  }
  // can_count_last
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->can_count_last[i] != rhs->can_count_last[i]) {
      return false;
    }
  }
  // can_error
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->can_error[i] != rhs->can_error[i]) {
      return false;
    }
  }
  // j_s
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->j_s[i] != rhs->j_s[i]) {
      return false;
    }
  }
  // j_p
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->j_p[i] != rhs->j_p[i]) {
      return false;
    }
  }
  // j_v
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->j_v[i] != rhs->j_v[i]) {
      return false;
    }
  }
  // j_a
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->j_a[i] != rhs->j_a[i]) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__MotorState__copy(
  const astribot_msgs__msg__MotorState * input,
  astribot_msgs__msg__MotorState * output)
{
  if (!input || !output) {
    return false;
  }
  // p
  for (size_t i = 0; i < 9; ++i) {
    output->p[i] = input->p[i];
  }
  // v
  for (size_t i = 0; i < 9; ++i) {
    output->v[i] = input->v[i];
  }
  // c
  for (size_t i = 0; i < 9; ++i) {
    output->c[i] = input->c[i];
  }
  // vol
  for (size_t i = 0; i < 9; ++i) {
    output->vol[i] = input->vol[i];
  }
  // acc
  for (size_t i = 0; i < 9; ++i) {
    output->acc[i] = input->acc[i];
  }
  // can_count
  for (size_t i = 0; i < 9; ++i) {
    output->can_count[i] = input->can_count[i];
  }
  // can_count_last
  for (size_t i = 0; i < 9; ++i) {
    output->can_count_last[i] = input->can_count_last[i];
  }
  // can_error
  for (size_t i = 0; i < 9; ++i) {
    output->can_error[i] = input->can_error[i];
  }
  // j_s
  for (size_t i = 0; i < 9; ++i) {
    output->j_s[i] = input->j_s[i];
  }
  // j_p
  for (size_t i = 0; i < 9; ++i) {
    output->j_p[i] = input->j_p[i];
  }
  // j_v
  for (size_t i = 0; i < 9; ++i) {
    output->j_v[i] = input->j_v[i];
  }
  // j_a
  for (size_t i = 0; i < 9; ++i) {
    output->j_a[i] = input->j_a[i];
  }
  return true;
}

astribot_msgs__msg__MotorState *
astribot_msgs__msg__MotorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__MotorState * msg = (astribot_msgs__msg__MotorState *)allocator.allocate(sizeof(astribot_msgs__msg__MotorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__MotorState));
  bool success = astribot_msgs__msg__MotorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__MotorState__destroy(astribot_msgs__msg__MotorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__MotorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__MotorState__Sequence__init(astribot_msgs__msg__MotorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__MotorState * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__MotorState *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__MotorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__MotorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__MotorState__fini(&data[i - 1]);
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
astribot_msgs__msg__MotorState__Sequence__fini(astribot_msgs__msg__MotorState__Sequence * array)
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
      astribot_msgs__msg__MotorState__fini(&array->data[i]);
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

astribot_msgs__msg__MotorState__Sequence *
astribot_msgs__msg__MotorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__MotorState__Sequence * array = (astribot_msgs__msg__MotorState__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__MotorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__MotorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__MotorState__Sequence__destroy(astribot_msgs__msg__MotorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__MotorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__MotorState__Sequence__are_equal(const astribot_msgs__msg__MotorState__Sequence * lhs, const astribot_msgs__msg__MotorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__MotorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__MotorState__Sequence__copy(
  const astribot_msgs__msg__MotorState__Sequence * input,
  astribot_msgs__msg__MotorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__MotorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__MotorState * data =
      (astribot_msgs__msg__MotorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__MotorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__MotorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__MotorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
