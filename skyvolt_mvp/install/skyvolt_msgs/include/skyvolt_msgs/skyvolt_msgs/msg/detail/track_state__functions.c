// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from skyvolt_msgs:msg/TrackState.idl
// generated code does not contain a copyright notice
#include "skyvolt_msgs/msg/detail/track_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"

bool
skyvolt_msgs__msg__TrackState__init(skyvolt_msgs__msg__TrackState * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    skyvolt_msgs__msg__TrackState__fini(msg);
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    skyvolt_msgs__msg__TrackState__fini(msg);
    return false;
  }
  // branch_id
  // arclength_m
  // speed_mps
  // arclength_var
  return true;
}

void
skyvolt_msgs__msg__TrackState__fini(skyvolt_msgs__msg__TrackState * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // branch_id
  // arclength_m
  // speed_mps
  // arclength_var
}

bool
skyvolt_msgs__msg__TrackState__are_equal(const skyvolt_msgs__msg__TrackState * lhs, const skyvolt_msgs__msg__TrackState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // branch_id
  if (lhs->branch_id != rhs->branch_id) {
    return false;
  }
  // arclength_m
  if (lhs->arclength_m != rhs->arclength_m) {
    return false;
  }
  // speed_mps
  if (lhs->speed_mps != rhs->speed_mps) {
    return false;
  }
  // arclength_var
  if (lhs->arclength_var != rhs->arclength_var) {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__msg__TrackState__copy(
  const skyvolt_msgs__msg__TrackState * input,
  skyvolt_msgs__msg__TrackState * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // branch_id
  output->branch_id = input->branch_id;
  // arclength_m
  output->arclength_m = input->arclength_m;
  // speed_mps
  output->speed_mps = input->speed_mps;
  // arclength_var
  output->arclength_var = input->arclength_var;
  return true;
}

skyvolt_msgs__msg__TrackState *
skyvolt_msgs__msg__TrackState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__TrackState * msg = (skyvolt_msgs__msg__TrackState *)allocator.allocate(sizeof(skyvolt_msgs__msg__TrackState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__msg__TrackState));
  bool success = skyvolt_msgs__msg__TrackState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__msg__TrackState__destroy(skyvolt_msgs__msg__TrackState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__msg__TrackState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__msg__TrackState__Sequence__init(skyvolt_msgs__msg__TrackState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__TrackState * data = NULL;

  if (size) {
    data = (skyvolt_msgs__msg__TrackState *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__msg__TrackState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__msg__TrackState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__msg__TrackState__fini(&data[i - 1]);
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
skyvolt_msgs__msg__TrackState__Sequence__fini(skyvolt_msgs__msg__TrackState__Sequence * array)
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
      skyvolt_msgs__msg__TrackState__fini(&array->data[i]);
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

skyvolt_msgs__msg__TrackState__Sequence *
skyvolt_msgs__msg__TrackState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__TrackState__Sequence * array = (skyvolt_msgs__msg__TrackState__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__msg__TrackState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__msg__TrackState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__msg__TrackState__Sequence__destroy(skyvolt_msgs__msg__TrackState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__msg__TrackState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__msg__TrackState__Sequence__are_equal(const skyvolt_msgs__msg__TrackState__Sequence * lhs, const skyvolt_msgs__msg__TrackState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__msg__TrackState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__msg__TrackState__Sequence__copy(
  const skyvolt_msgs__msg__TrackState__Sequence * input,
  skyvolt_msgs__msg__TrackState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__msg__TrackState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__msg__TrackState * data =
      (skyvolt_msgs__msg__TrackState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__msg__TrackState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__msg__TrackState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__msg__TrackState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
