// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/StorageRequest.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/storage_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `uuid`
// Member `tag_content`
// Member `topic_list`
#include "rosidl_runtime_c/string_functions.h"

bool
astribot_msgs__msg__StorageRequest__init(astribot_msgs__msg__StorageRequest * msg)
{
  if (!msg) {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__init(&msg->uuid)) {
    astribot_msgs__msg__StorageRequest__fini(msg);
    return false;
  }
  // storage_type
  // tag_content
  if (!rosidl_runtime_c__String__init(&msg->tag_content)) {
    astribot_msgs__msg__StorageRequest__fini(msg);
    return false;
  }
  // topic_list
  if (!rosidl_runtime_c__String__init(&msg->topic_list)) {
    astribot_msgs__msg__StorageRequest__fini(msg);
    return false;
  }
  return true;
}

void
astribot_msgs__msg__StorageRequest__fini(astribot_msgs__msg__StorageRequest * msg)
{
  if (!msg) {
    return;
  }
  // uuid
  rosidl_runtime_c__String__fini(&msg->uuid);
  // storage_type
  // tag_content
  rosidl_runtime_c__String__fini(&msg->tag_content);
  // topic_list
  rosidl_runtime_c__String__fini(&msg->topic_list);
}

bool
astribot_msgs__msg__StorageRequest__are_equal(const astribot_msgs__msg__StorageRequest * lhs, const astribot_msgs__msg__StorageRequest * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uuid), &(rhs->uuid)))
  {
    return false;
  }
  // storage_type
  if (lhs->storage_type != rhs->storage_type) {
    return false;
  }
  // tag_content
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tag_content), &(rhs->tag_content)))
  {
    return false;
  }
  // topic_list
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic_list), &(rhs->topic_list)))
  {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__StorageRequest__copy(
  const astribot_msgs__msg__StorageRequest * input,
  astribot_msgs__msg__StorageRequest * output)
{
  if (!input || !output) {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__copy(
      &(input->uuid), &(output->uuid)))
  {
    return false;
  }
  // storage_type
  output->storage_type = input->storage_type;
  // tag_content
  if (!rosidl_runtime_c__String__copy(
      &(input->tag_content), &(output->tag_content)))
  {
    return false;
  }
  // topic_list
  if (!rosidl_runtime_c__String__copy(
      &(input->topic_list), &(output->topic_list)))
  {
    return false;
  }
  return true;
}

astribot_msgs__msg__StorageRequest *
astribot_msgs__msg__StorageRequest__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__StorageRequest * msg = (astribot_msgs__msg__StorageRequest *)allocator.allocate(sizeof(astribot_msgs__msg__StorageRequest), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__StorageRequest));
  bool success = astribot_msgs__msg__StorageRequest__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__StorageRequest__destroy(astribot_msgs__msg__StorageRequest * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__StorageRequest__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__StorageRequest__Sequence__init(astribot_msgs__msg__StorageRequest__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__StorageRequest * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__StorageRequest *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__StorageRequest), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__StorageRequest__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__StorageRequest__fini(&data[i - 1]);
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
astribot_msgs__msg__StorageRequest__Sequence__fini(astribot_msgs__msg__StorageRequest__Sequence * array)
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
      astribot_msgs__msg__StorageRequest__fini(&array->data[i]);
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

astribot_msgs__msg__StorageRequest__Sequence *
astribot_msgs__msg__StorageRequest__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__StorageRequest__Sequence * array = (astribot_msgs__msg__StorageRequest__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__StorageRequest__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__StorageRequest__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__StorageRequest__Sequence__destroy(astribot_msgs__msg__StorageRequest__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__StorageRequest__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__StorageRequest__Sequence__are_equal(const astribot_msgs__msg__StorageRequest__Sequence * lhs, const astribot_msgs__msg__StorageRequest__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__StorageRequest__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__StorageRequest__Sequence__copy(
  const astribot_msgs__msg__StorageRequest__Sequence * input,
  astribot_msgs__msg__StorageRequest__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__StorageRequest);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__StorageRequest * data =
      (astribot_msgs__msg__StorageRequest *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__StorageRequest__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__StorageRequest__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__StorageRequest__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
