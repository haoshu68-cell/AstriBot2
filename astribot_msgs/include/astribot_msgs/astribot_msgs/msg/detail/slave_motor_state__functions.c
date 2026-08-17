// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/SlaveMotorState.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/slave_motor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "astribot_msgs/msg/detail/astribot_header__functions.h"
// Member `slave_motor_state`
#include "astribot_msgs/msg/detail/motor_state__functions.h"

bool
astribot_msgs__msg__SlaveMotorState__init(astribot_msgs__msg__SlaveMotorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!astribot_msgs__msg__AstribotHeader__init(&msg->header)) {
    astribot_msgs__msg__SlaveMotorState__fini(msg);
    return false;
  }
  // slave_motor_state
  for (size_t i = 0; i < 3; ++i) {
    if (!astribot_msgs__msg__MotorState__init(&msg->slave_motor_state[i])) {
      astribot_msgs__msg__SlaveMotorState__fini(msg);
      return false;
    }
  }
  return true;
}

void
astribot_msgs__msg__SlaveMotorState__fini(astribot_msgs__msg__SlaveMotorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  astribot_msgs__msg__AstribotHeader__fini(&msg->header);
  // slave_motor_state
  for (size_t i = 0; i < 3; ++i) {
    astribot_msgs__msg__MotorState__fini(&msg->slave_motor_state[i]);
  }
}

bool
astribot_msgs__msg__SlaveMotorState__are_equal(const astribot_msgs__msg__SlaveMotorState * lhs, const astribot_msgs__msg__SlaveMotorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!astribot_msgs__msg__AstribotHeader__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // slave_motor_state
  for (size_t i = 0; i < 3; ++i) {
    if (!astribot_msgs__msg__MotorState__are_equal(
        &(lhs->slave_motor_state[i]), &(rhs->slave_motor_state[i])))
    {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__SlaveMotorState__copy(
  const astribot_msgs__msg__SlaveMotorState * input,
  astribot_msgs__msg__SlaveMotorState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!astribot_msgs__msg__AstribotHeader__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // slave_motor_state
  for (size_t i = 0; i < 3; ++i) {
    if (!astribot_msgs__msg__MotorState__copy(
        &(input->slave_motor_state[i]), &(output->slave_motor_state[i])))
    {
      return false;
    }
  }
  return true;
}

astribot_msgs__msg__SlaveMotorState *
astribot_msgs__msg__SlaveMotorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__SlaveMotorState * msg = (astribot_msgs__msg__SlaveMotorState *)allocator.allocate(sizeof(astribot_msgs__msg__SlaveMotorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__SlaveMotorState));
  bool success = astribot_msgs__msg__SlaveMotorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__SlaveMotorState__destroy(astribot_msgs__msg__SlaveMotorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__SlaveMotorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__SlaveMotorState__Sequence__init(astribot_msgs__msg__SlaveMotorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__SlaveMotorState * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__SlaveMotorState *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__SlaveMotorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__SlaveMotorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__SlaveMotorState__fini(&data[i - 1]);
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
astribot_msgs__msg__SlaveMotorState__Sequence__fini(astribot_msgs__msg__SlaveMotorState__Sequence * array)
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
      astribot_msgs__msg__SlaveMotorState__fini(&array->data[i]);
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

astribot_msgs__msg__SlaveMotorState__Sequence *
astribot_msgs__msg__SlaveMotorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__SlaveMotorState__Sequence * array = (astribot_msgs__msg__SlaveMotorState__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__SlaveMotorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__SlaveMotorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__SlaveMotorState__Sequence__destroy(astribot_msgs__msg__SlaveMotorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__SlaveMotorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__SlaveMotorState__Sequence__are_equal(const astribot_msgs__msg__SlaveMotorState__Sequence * lhs, const astribot_msgs__msg__SlaveMotorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__SlaveMotorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__SlaveMotorState__Sequence__copy(
  const astribot_msgs__msg__SlaveMotorState__Sequence * input,
  astribot_msgs__msg__SlaveMotorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__SlaveMotorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__SlaveMotorState * data =
      (astribot_msgs__msg__SlaveMotorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__SlaveMotorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__SlaveMotorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__SlaveMotorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
