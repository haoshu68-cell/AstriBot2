// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
// generated code does not contain a copyright notice
#include "astribot_msgs/msg/detail/whole_body_ctrl_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "astribot_msgs/msg/detail/astribot_header__functions.h"
// Member `left_arm_twist`
// Member `right_arm_twist`
// Member `torso_twist`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `pose_world_to_torso_desired`
// Member `pose_world_to_left_arm_desired`
// Member `pose_world_to_right_arm_desired`
// Member `pose_world_to_chassis_current`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
astribot_msgs__msg__WholeBodyCtrlCmd__init(astribot_msgs__msg__WholeBodyCtrlCmd * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!astribot_msgs__msg__AstribotHeader__init(&msg->header)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // left_arm_twist
  if (!geometry_msgs__msg__Twist__init(&msg->left_arm_twist)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // right_arm_twist
  if (!geometry_msgs__msg__Twist__init(&msg->right_arm_twist)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // torso_twist
  if (!geometry_msgs__msg__Twist__init(&msg->torso_twist)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // pose_world_to_torso_desired
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pose_world_to_torso_desired, 0)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // pose_world_to_left_arm_desired
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pose_world_to_left_arm_desired, 0)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // pose_world_to_right_arm_desired
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pose_world_to_right_arm_desired, 0)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // pose_world_to_chassis_current
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pose_world_to_chassis_current, 0)) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
    return false;
  }
  // torso_open_loop
  // enable_collision_avoidance
  // smooth_t
  // smooth_duration
  return true;
}

void
astribot_msgs__msg__WholeBodyCtrlCmd__fini(astribot_msgs__msg__WholeBodyCtrlCmd * msg)
{
  if (!msg) {
    return;
  }
  // header
  astribot_msgs__msg__AstribotHeader__fini(&msg->header);
  // left_arm_twist
  geometry_msgs__msg__Twist__fini(&msg->left_arm_twist);
  // right_arm_twist
  geometry_msgs__msg__Twist__fini(&msg->right_arm_twist);
  // torso_twist
  geometry_msgs__msg__Twist__fini(&msg->torso_twist);
  // pose_world_to_torso_desired
  rosidl_runtime_c__double__Sequence__fini(&msg->pose_world_to_torso_desired);
  // pose_world_to_left_arm_desired
  rosidl_runtime_c__double__Sequence__fini(&msg->pose_world_to_left_arm_desired);
  // pose_world_to_right_arm_desired
  rosidl_runtime_c__double__Sequence__fini(&msg->pose_world_to_right_arm_desired);
  // pose_world_to_chassis_current
  rosidl_runtime_c__double__Sequence__fini(&msg->pose_world_to_chassis_current);
  // torso_open_loop
  // enable_collision_avoidance
  // smooth_t
  // smooth_duration
}

bool
astribot_msgs__msg__WholeBodyCtrlCmd__are_equal(const astribot_msgs__msg__WholeBodyCtrlCmd * lhs, const astribot_msgs__msg__WholeBodyCtrlCmd * rhs)
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
  // left_arm_twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->left_arm_twist), &(rhs->left_arm_twist)))
  {
    return false;
  }
  // right_arm_twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->right_arm_twist), &(rhs->right_arm_twist)))
  {
    return false;
  }
  // torso_twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->torso_twist), &(rhs->torso_twist)))
  {
    return false;
  }
  // pose_world_to_torso_desired
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->pose_world_to_torso_desired), &(rhs->pose_world_to_torso_desired)))
  {
    return false;
  }
  // pose_world_to_left_arm_desired
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->pose_world_to_left_arm_desired), &(rhs->pose_world_to_left_arm_desired)))
  {
    return false;
  }
  // pose_world_to_right_arm_desired
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->pose_world_to_right_arm_desired), &(rhs->pose_world_to_right_arm_desired)))
  {
    return false;
  }
  // pose_world_to_chassis_current
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->pose_world_to_chassis_current), &(rhs->pose_world_to_chassis_current)))
  {
    return false;
  }
  // torso_open_loop
  if (lhs->torso_open_loop != rhs->torso_open_loop) {
    return false;
  }
  // enable_collision_avoidance
  if (lhs->enable_collision_avoidance != rhs->enable_collision_avoidance) {
    return false;
  }
  // smooth_t
  if (lhs->smooth_t != rhs->smooth_t) {
    return false;
  }
  // smooth_duration
  if (lhs->smooth_duration != rhs->smooth_duration) {
    return false;
  }
  return true;
}

bool
astribot_msgs__msg__WholeBodyCtrlCmd__copy(
  const astribot_msgs__msg__WholeBodyCtrlCmd * input,
  astribot_msgs__msg__WholeBodyCtrlCmd * output)
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
  // left_arm_twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->left_arm_twist), &(output->left_arm_twist)))
  {
    return false;
  }
  // right_arm_twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->right_arm_twist), &(output->right_arm_twist)))
  {
    return false;
  }
  // torso_twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->torso_twist), &(output->torso_twist)))
  {
    return false;
  }
  // pose_world_to_torso_desired
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->pose_world_to_torso_desired), &(output->pose_world_to_torso_desired)))
  {
    return false;
  }
  // pose_world_to_left_arm_desired
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->pose_world_to_left_arm_desired), &(output->pose_world_to_left_arm_desired)))
  {
    return false;
  }
  // pose_world_to_right_arm_desired
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->pose_world_to_right_arm_desired), &(output->pose_world_to_right_arm_desired)))
  {
    return false;
  }
  // pose_world_to_chassis_current
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->pose_world_to_chassis_current), &(output->pose_world_to_chassis_current)))
  {
    return false;
  }
  // torso_open_loop
  output->torso_open_loop = input->torso_open_loop;
  // enable_collision_avoidance
  output->enable_collision_avoidance = input->enable_collision_avoidance;
  // smooth_t
  output->smooth_t = input->smooth_t;
  // smooth_duration
  output->smooth_duration = input->smooth_duration;
  return true;
}

astribot_msgs__msg__WholeBodyCtrlCmd *
astribot_msgs__msg__WholeBodyCtrlCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__WholeBodyCtrlCmd * msg = (astribot_msgs__msg__WholeBodyCtrlCmd *)allocator.allocate(sizeof(astribot_msgs__msg__WholeBodyCtrlCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astribot_msgs__msg__WholeBodyCtrlCmd));
  bool success = astribot_msgs__msg__WholeBodyCtrlCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astribot_msgs__msg__WholeBodyCtrlCmd__destroy(astribot_msgs__msg__WholeBodyCtrlCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astribot_msgs__msg__WholeBodyCtrlCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__init(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__WholeBodyCtrlCmd * data = NULL;

  if (size) {
    data = (astribot_msgs__msg__WholeBodyCtrlCmd *)allocator.zero_allocate(size, sizeof(astribot_msgs__msg__WholeBodyCtrlCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astribot_msgs__msg__WholeBodyCtrlCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astribot_msgs__msg__WholeBodyCtrlCmd__fini(&data[i - 1]);
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
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__fini(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array)
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
      astribot_msgs__msg__WholeBodyCtrlCmd__fini(&array->data[i]);
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

astribot_msgs__msg__WholeBodyCtrlCmd__Sequence *
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array = (astribot_msgs__msg__WholeBodyCtrlCmd__Sequence *)allocator.allocate(sizeof(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__destroy(astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__are_equal(const astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * lhs, const astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astribot_msgs__msg__WholeBodyCtrlCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astribot_msgs__msg__WholeBodyCtrlCmd__Sequence__copy(
  const astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * input,
  astribot_msgs__msg__WholeBodyCtrlCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astribot_msgs__msg__WholeBodyCtrlCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astribot_msgs__msg__WholeBodyCtrlCmd * data =
      (astribot_msgs__msg__WholeBodyCtrlCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astribot_msgs__msg__WholeBodyCtrlCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astribot_msgs__msg__WholeBodyCtrlCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astribot_msgs__msg__WholeBodyCtrlCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
