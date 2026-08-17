// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:srv/AstribotHeaderSrv.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/srv/detail/astribot_header_srv__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `header`
#include "astribot_msgs/msg/detail/astribot_header__functions.h"

bool
astribot_msgs__srv__AstribotHeaderSrv_Request__init(astribot_msgs__srv__AstribotHeaderSrv_Request * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!astribot_msgs__msg__AstribotHeader__init(&msg->header)) {
    astribot_msgs__srv__AstribotHeaderSrv_Request__fini(msg);
    return false;
  }
  return true;
}

void
astribot_msgs__srv__AstribotHeaderSrv_Request__fini(astribot_msgs__srv__AstribotHeaderSrv_Request * msg)
{
  if (!msg) {
    return;
  }
  // header
  astribot_msgs__msg__AstribotHeader__fini(&msg->header);
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Request__are_equal(const astribot_msgs__srv__AstribotHeaderSrv_Request * lhs, const astribot_msgs__srv__AstribotHeaderSrv_Request * rhs)
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
  return true;
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Request__copy(
  const astribot_msgs__srv__AstribotHeaderSrv_Request * input,
  astribot_msgs__srv__AstribotHeaderSrv_Request * output)
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
  return true;
}

astribot_msgs__srv__AstribotHeaderSrv_Request *
astribot_msgs__srv__AstribotHeaderSrv_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__srv__AstribotHeaderSrv_Request * msg = (astribot_msgs__srv__AstribotHeaderSrv_Request *)allocator.allocate(sizeof(astribot_msgs__srv__AstribotHeaderSrv_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__srv__AstribotHeaderSrv_Request));
  bool success = astribot_msgs__srv__AstribotHeaderSrv_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__srv__AstribotHeaderSrv_Request__destroy(astribot_msgs__srv__AstribotHeaderSrv_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__srv__AstribotHeaderSrv_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__init(astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__srv__AstribotHeaderSrv_Request * data = NULL;

  if (size) {
    data = (astribot_msgs__srv__AstribotHeaderSrv_Request *)allocator.zero_allocate(size, sizeof(astribot_msgs__srv__AstribotHeaderSrv_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__srv__AstribotHeaderSrv_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__srv__AstribotHeaderSrv_Request__fini(&data[i - 1]);
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
astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__fini(astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * array)
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
      astribot_msgs__srv__AstribotHeaderSrv_Request__fini(&array->data[i]);
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

astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence *
astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * array = (astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence *)allocator.allocate(sizeof(astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__destroy(astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__are_equal(const astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * lhs, const astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__srv__AstribotHeaderSrv_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence__copy(
  const astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * input,
  astribot_msgs__srv__AstribotHeaderSrv_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__srv__AstribotHeaderSrv_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__srv__AstribotHeaderSrv_Request * data =
      (astribot_msgs__srv__AstribotHeaderSrv_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__srv__AstribotHeaderSrv_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__srv__AstribotHeaderSrv_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__srv__AstribotHeaderSrv_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `header`
// already included above
// #include "astribot_msgs/msg/detail/astribot_header__functions.h"

bool
astribot_msgs__srv__AstribotHeaderSrv_Response__init(astribot_msgs__srv__AstribotHeaderSrv_Response * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!astribot_msgs__msg__AstribotHeader__init(&msg->header)) {
    astribot_msgs__srv__AstribotHeaderSrv_Response__fini(msg);
    return false;
  }
  return true;
}

void
astribot_msgs__srv__AstribotHeaderSrv_Response__fini(astribot_msgs__srv__AstribotHeaderSrv_Response * msg)
{
  if (!msg) {
    return;
  }
  // header
  astribot_msgs__msg__AstribotHeader__fini(&msg->header);
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Response__are_equal(const astribot_msgs__srv__AstribotHeaderSrv_Response * lhs, const astribot_msgs__srv__AstribotHeaderSrv_Response * rhs)
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
  return true;
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Response__copy(
  const astribot_msgs__srv__AstribotHeaderSrv_Response * input,
  astribot_msgs__srv__AstribotHeaderSrv_Response * output)
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
  return true;
}

astribot_msgs__srv__AstribotHeaderSrv_Response *
astribot_msgs__srv__AstribotHeaderSrv_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__srv__AstribotHeaderSrv_Response * msg = (astribot_msgs__srv__AstribotHeaderSrv_Response *)allocator.allocate(sizeof(astribot_msgs__srv__AstribotHeaderSrv_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__srv__AstribotHeaderSrv_Response));
  bool success = astribot_msgs__srv__AstribotHeaderSrv_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__srv__AstribotHeaderSrv_Response__destroy(astribot_msgs__srv__AstribotHeaderSrv_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__srv__AstribotHeaderSrv_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__init(astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__srv__AstribotHeaderSrv_Response * data = NULL;

  if (size) {
    data = (astribot_msgs__srv__AstribotHeaderSrv_Response *)allocator.zero_allocate(size, sizeof(astribot_msgs__srv__AstribotHeaderSrv_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__srv__AstribotHeaderSrv_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__srv__AstribotHeaderSrv_Response__fini(&data[i - 1]);
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
astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__fini(astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * array)
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
      astribot_msgs__srv__AstribotHeaderSrv_Response__fini(&array->data[i]);
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

astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence *
astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * array = (astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence *)allocator.allocate(sizeof(astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__destroy(astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__are_equal(const astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * lhs, const astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__srv__AstribotHeaderSrv_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence__copy(
  const astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * input,
  astribot_msgs__srv__AstribotHeaderSrv_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__srv__AstribotHeaderSrv_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__srv__AstribotHeaderSrv_Response * data =
      (astribot_msgs__srv__AstribotHeaderSrv_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__srv__AstribotHeaderSrv_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__srv__AstribotHeaderSrv_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__srv__AstribotHeaderSrv_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
