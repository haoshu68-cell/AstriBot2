// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/motor_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motor_id_list`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
astribot_msgs__msg__MotorCommand__init(astribot_msgs__msg__MotorCommand * msg)
{
  if (!msg) {
    return false;
  }
  // kp
  // kd
  // p
  // v
  // t_ff
  // t_limit
  // break_relase
  // kd_slave
  // vel_slave
  // motor_id_list
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->motor_id_list, 0)) {
    astribot_msgs__msg__MotorCommand__fini(msg);
    return false;
  }
  return true;
}

void
astribot_msgs__msg__MotorCommand__fini(astribot_msgs__msg__MotorCommand * msg)
{
  if (!msg) {
    return;
  }
  // kp
  // kd
  // p
  // v
  // t_ff
  // t_limit
  // break_relase
  // kd_slave
  // vel_slave
  // motor_id_list
  rosidl_runtime_c__int32__Sequence__fini(&msg->motor_id_list);
}

bool
astribot_msgs__msg__MotorCommand__are_equal(const astribot_msgs__msg__MotorCommand * lhs, const astribot_msgs__msg__MotorCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // kp
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->kp[i] != rhs->kp[i]) {
      return false;
    }
  }
  // kd
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->kd[i] != rhs->kd[i]) {
      return false;
    }
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
  // t_ff
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->t_ff[i] != rhs->t_ff[i]) {
      return false;
    }
  }
  // t_limit
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->t_limit[i] != rhs->t_limit[i]) {
      return false;
    }
  }
  // break_relase
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->break_relase[i] != rhs->break_relase[i]) {
      return false;
    }
  }
  // kd_slave
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->kd_slave[i] != rhs->kd_slave[i]) {
      return false;
    }
  }
  // vel_slave
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->vel_slave[i] != rhs->vel_slave[i]) {
      return false;
    }
  }
  // motor_id_list
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->motor_id_list), &(rhs->motor_id_list)))
  {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__MotorCommand__copy(
  const astribot_msgs__msg__MotorCommand * input,
  astribot_msgs__msg__MotorCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // kp
  for (size_t i = 0; i < 9; ++i) {
    output->kp[i] = input->kp[i];
  }
  // kd
  for (size_t i = 0; i < 9; ++i) {
    output->kd[i] = input->kd[i];
  }
  // p
  for (size_t i = 0; i < 9; ++i) {
    output->p[i] = input->p[i];
  }
  // v
  for (size_t i = 0; i < 9; ++i) {
    output->v[i] = input->v[i];
  }
  // t_ff
  for (size_t i = 0; i < 9; ++i) {
    output->t_ff[i] = input->t_ff[i];
  }
  // t_limit
  for (size_t i = 0; i < 9; ++i) {
    output->t_limit[i] = input->t_limit[i];
  }
  // break_relase
  for (size_t i = 0; i < 9; ++i) {
    output->break_relase[i] = input->break_relase[i];
  }
  // kd_slave
  for (size_t i = 0; i < 9; ++i) {
    output->kd_slave[i] = input->kd_slave[i];
  }
  // vel_slave
  for (size_t i = 0; i < 9; ++i) {
    output->vel_slave[i] = input->vel_slave[i];
  }
  // motor_id_list
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->motor_id_list), &(output->motor_id_list)))
  {
    return false;
  }
  return true;
}

astribot_msgs__msg__MotorCommand *
astribot_msgs__msg__MotorCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__MotorCommand * msg = (astribot_msgs__msg__MotorCommand *)allocator.allocate(sizeof(astribot_msgs__msg__MotorCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__MotorCommand));
  bool success = astribot_msgs__msg__MotorCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__MotorCommand__destroy(astribot_msgs__msg__MotorCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__MotorCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__MotorCommand__Sequence__init(astribot_msgs__msg__MotorCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__MotorCommand * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__MotorCommand *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__MotorCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__MotorCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__MotorCommand__fini(&data[i - 1]);
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
astribot_msgs__msg__MotorCommand__Sequence__fini(astribot_msgs__msg__MotorCommand__Sequence * array)
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
      astribot_msgs__msg__MotorCommand__fini(&array->data[i]);
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

astribot_msgs__msg__MotorCommand__Sequence *
astribot_msgs__msg__MotorCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__MotorCommand__Sequence * array = (astribot_msgs__msg__MotorCommand__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__MotorCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__MotorCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__MotorCommand__Sequence__destroy(astribot_msgs__msg__MotorCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__MotorCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__MotorCommand__Sequence__are_equal(const astribot_msgs__msg__MotorCommand__Sequence * lhs, const astribot_msgs__msg__MotorCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__MotorCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__MotorCommand__Sequence__copy(
  const astribot_msgs__msg__MotorCommand__Sequence * input,
  astribot_msgs__msg__MotorCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__MotorCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__MotorCommand * data =
      (astribot_msgs__msg__MotorCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__MotorCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__MotorCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__MotorCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
