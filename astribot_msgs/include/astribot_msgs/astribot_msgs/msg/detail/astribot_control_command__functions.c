// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/AstribotControlCommand.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/astribot_control_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name_list`
// Member `control_way`
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"
// Member `dofs_list`
// Member `command_list`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
astribot_msgs__msg__AstribotControlCommand__init(astribot_msgs__msg__AstribotControlCommand * msg)
{
  if (!msg) {
    return false;
  }
  // name_list
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name_list, 0)) {
    astribot_msgs__msg__AstribotControlCommand__fini(msg);
    return false;
  }
  // dofs_list
  if (!rosidl_runtime_c__float__Sequence__init(&msg->dofs_list, 0)) {
    astribot_msgs__msg__AstribotControlCommand__fini(msg);
    return false;
  }
  // command_list
  if (!rosidl_runtime_c__float__Sequence__init(&msg->command_list, 0)) {
    astribot_msgs__msg__AstribotControlCommand__fini(msg);
    return false;
  }
  // control_way
  if (!rosidl_runtime_c__String__init(&msg->control_way)) {
    astribot_msgs__msg__AstribotControlCommand__fini(msg);
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__init(&msg->frame)) {
    astribot_msgs__msg__AstribotControlCommand__fini(msg);
    return false;
  }
  // use_wbc
  // add_default_torso
  return true;
}

void
astribot_msgs__msg__AstribotControlCommand__fini(astribot_msgs__msg__AstribotControlCommand * msg)
{
  if (!msg) {
    return;
  }
  // name_list
  rosidl_runtime_c__String__Sequence__fini(&msg->name_list);
  // dofs_list
  rosidl_runtime_c__float__Sequence__fini(&msg->dofs_list);
  // command_list
  rosidl_runtime_c__float__Sequence__fini(&msg->command_list);
  // control_way
  rosidl_runtime_c__String__fini(&msg->control_way);
  // frame
  rosidl_runtime_c__String__fini(&msg->frame);
  // use_wbc
  // add_default_torso
}

bool
astribot_msgs__msg__AstribotControlCommand__are_equal(const astribot_msgs__msg__AstribotControlCommand * lhs, const astribot_msgs__msg__AstribotControlCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name_list
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->name_list), &(rhs->name_list)))
  {
    return false;
  }
  // dofs_list
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->dofs_list), &(rhs->dofs_list)))
  {
    return false;
  }
  // command_list
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->command_list), &(rhs->command_list)))
  {
    return false;
  }
  // control_way
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->control_way), &(rhs->control_way)))
  {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame), &(rhs->frame)))
  {
    return false;
  }
  // use_wbc
  if (lhs->use_wbc != rhs->use_wbc) {
    return false;
  }
  // add_default_torso
  if (lhs->add_default_torso != rhs->add_default_torso) {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__AstribotControlCommand__copy(
  const astribot_msgs__msg__AstribotControlCommand * input,
  astribot_msgs__msg__AstribotControlCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // name_list
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->name_list), &(output->name_list)))
  {
    return false;
  }
  // dofs_list
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->dofs_list), &(output->dofs_list)))
  {
    return false;
  }
  // command_list
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->command_list), &(output->command_list)))
  {
    return false;
  }
  // control_way
  if (!rosidl_runtime_c__String__copy(
      &(input->control_way), &(output->control_way)))
  {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__copy(
      &(input->frame), &(output->frame)))
  {
    return false;
  }
  // use_wbc
  output->use_wbc = input->use_wbc;
  // add_default_torso
  output->add_default_torso = input->add_default_torso;
  return true;
}

astribot_msgs__msg__AstribotControlCommand *
astribot_msgs__msg__AstribotControlCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__AstribotControlCommand * msg = (astribot_msgs__msg__AstribotControlCommand *)allocator.allocate(sizeof(astribot_msgs__msg__AstribotControlCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__AstribotControlCommand));
  bool success = astribot_msgs__msg__AstribotControlCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__AstribotControlCommand__destroy(astribot_msgs__msg__AstribotControlCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__AstribotControlCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__AstribotControlCommand__Sequence__init(astribot_msgs__msg__AstribotControlCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__AstribotControlCommand * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__AstribotControlCommand *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__AstribotControlCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__AstribotControlCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__AstribotControlCommand__fini(&data[i - 1]);
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
astribot_msgs__msg__AstribotControlCommand__Sequence__fini(astribot_msgs__msg__AstribotControlCommand__Sequence * array)
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
      astribot_msgs__msg__AstribotControlCommand__fini(&array->data[i]);
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

astribot_msgs__msg__AstribotControlCommand__Sequence *
astribot_msgs__msg__AstribotControlCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__AstribotControlCommand__Sequence * array = (astribot_msgs__msg__AstribotControlCommand__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__AstribotControlCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__AstribotControlCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__AstribotControlCommand__Sequence__destroy(astribot_msgs__msg__AstribotControlCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__AstribotControlCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__AstribotControlCommand__Sequence__are_equal(const astribot_msgs__msg__AstribotControlCommand__Sequence * lhs, const astribot_msgs__msg__AstribotControlCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__AstribotControlCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__AstribotControlCommand__Sequence__copy(
  const astribot_msgs__msg__AstribotControlCommand__Sequence * input,
  astribot_msgs__msg__AstribotControlCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__AstribotControlCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__AstribotControlCommand * data =
      (astribot_msgs__msg__AstribotControlCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__AstribotControlCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__AstribotControlCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__AstribotControlCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
