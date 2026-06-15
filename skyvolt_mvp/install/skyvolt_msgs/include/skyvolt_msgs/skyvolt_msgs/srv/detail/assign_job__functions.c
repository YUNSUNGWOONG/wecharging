// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from skyvolt_msgs:srv/AssignJob.idl
// generated code does not contain a copyright notice
#include "skyvolt_msgs/srv/detail/assign_job__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `job`
#include "skyvolt_msgs/msg/detail/charging_job__functions.h"

bool
skyvolt_msgs__srv__AssignJob_Request__init(skyvolt_msgs__srv__AssignJob_Request * msg)
{
  if (!msg) {
    return false;
  }
  // job
  if (!skyvolt_msgs__msg__ChargingJob__init(&msg->job)) {
    skyvolt_msgs__srv__AssignJob_Request__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__srv__AssignJob_Request__fini(skyvolt_msgs__srv__AssignJob_Request * msg)
{
  if (!msg) {
    return;
  }
  // job
  skyvolt_msgs__msg__ChargingJob__fini(&msg->job);
}

bool
skyvolt_msgs__srv__AssignJob_Request__are_equal(const skyvolt_msgs__srv__AssignJob_Request * lhs, const skyvolt_msgs__srv__AssignJob_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // job
  if (!skyvolt_msgs__msg__ChargingJob__are_equal(
      &(lhs->job), &(rhs->job)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__srv__AssignJob_Request__copy(
  const skyvolt_msgs__srv__AssignJob_Request * input,
  skyvolt_msgs__srv__AssignJob_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // job
  if (!skyvolt_msgs__msg__ChargingJob__copy(
      &(input->job), &(output->job)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__srv__AssignJob_Request *
skyvolt_msgs__srv__AssignJob_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__srv__AssignJob_Request * msg = (skyvolt_msgs__srv__AssignJob_Request *)allocator.allocate(sizeof(skyvolt_msgs__srv__AssignJob_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__srv__AssignJob_Request));
  bool success = skyvolt_msgs__srv__AssignJob_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__srv__AssignJob_Request__destroy(skyvolt_msgs__srv__AssignJob_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__srv__AssignJob_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__srv__AssignJob_Request__Sequence__init(skyvolt_msgs__srv__AssignJob_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__srv__AssignJob_Request * data = NULL;

  if (size) {
    data = (skyvolt_msgs__srv__AssignJob_Request *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__srv__AssignJob_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__srv__AssignJob_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__srv__AssignJob_Request__fini(&data[i - 1]);
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
skyvolt_msgs__srv__AssignJob_Request__Sequence__fini(skyvolt_msgs__srv__AssignJob_Request__Sequence * array)
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
      skyvolt_msgs__srv__AssignJob_Request__fini(&array->data[i]);
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

skyvolt_msgs__srv__AssignJob_Request__Sequence *
skyvolt_msgs__srv__AssignJob_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__srv__AssignJob_Request__Sequence * array = (skyvolt_msgs__srv__AssignJob_Request__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__srv__AssignJob_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__srv__AssignJob_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__srv__AssignJob_Request__Sequence__destroy(skyvolt_msgs__srv__AssignJob_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__srv__AssignJob_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__srv__AssignJob_Request__Sequence__are_equal(const skyvolt_msgs__srv__AssignJob_Request__Sequence * lhs, const skyvolt_msgs__srv__AssignJob_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__srv__AssignJob_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__srv__AssignJob_Request__Sequence__copy(
  const skyvolt_msgs__srv__AssignJob_Request__Sequence * input,
  skyvolt_msgs__srv__AssignJob_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__srv__AssignJob_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__srv__AssignJob_Request * data =
      (skyvolt_msgs__srv__AssignJob_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__srv__AssignJob_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__srv__AssignJob_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__srv__AssignJob_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `assigned_robot_id`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
skyvolt_msgs__srv__AssignJob_Response__init(skyvolt_msgs__srv__AssignJob_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // assigned_robot_id
  if (!rosidl_runtime_c__String__init(&msg->assigned_robot_id)) {
    skyvolt_msgs__srv__AssignJob_Response__fini(msg);
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    skyvolt_msgs__srv__AssignJob_Response__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__srv__AssignJob_Response__fini(skyvolt_msgs__srv__AssignJob_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // assigned_robot_id
  rosidl_runtime_c__String__fini(&msg->assigned_robot_id);
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
skyvolt_msgs__srv__AssignJob_Response__are_equal(const skyvolt_msgs__srv__AssignJob_Response * lhs, const skyvolt_msgs__srv__AssignJob_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // assigned_robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->assigned_robot_id), &(rhs->assigned_robot_id)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__srv__AssignJob_Response__copy(
  const skyvolt_msgs__srv__AssignJob_Response * input,
  skyvolt_msgs__srv__AssignJob_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // assigned_robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->assigned_robot_id), &(output->assigned_robot_id)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__srv__AssignJob_Response *
skyvolt_msgs__srv__AssignJob_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__srv__AssignJob_Response * msg = (skyvolt_msgs__srv__AssignJob_Response *)allocator.allocate(sizeof(skyvolt_msgs__srv__AssignJob_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__srv__AssignJob_Response));
  bool success = skyvolt_msgs__srv__AssignJob_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__srv__AssignJob_Response__destroy(skyvolt_msgs__srv__AssignJob_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__srv__AssignJob_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__srv__AssignJob_Response__Sequence__init(skyvolt_msgs__srv__AssignJob_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__srv__AssignJob_Response * data = NULL;

  if (size) {
    data = (skyvolt_msgs__srv__AssignJob_Response *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__srv__AssignJob_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__srv__AssignJob_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__srv__AssignJob_Response__fini(&data[i - 1]);
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
skyvolt_msgs__srv__AssignJob_Response__Sequence__fini(skyvolt_msgs__srv__AssignJob_Response__Sequence * array)
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
      skyvolt_msgs__srv__AssignJob_Response__fini(&array->data[i]);
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

skyvolt_msgs__srv__AssignJob_Response__Sequence *
skyvolt_msgs__srv__AssignJob_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__srv__AssignJob_Response__Sequence * array = (skyvolt_msgs__srv__AssignJob_Response__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__srv__AssignJob_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__srv__AssignJob_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__srv__AssignJob_Response__Sequence__destroy(skyvolt_msgs__srv__AssignJob_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__srv__AssignJob_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__srv__AssignJob_Response__Sequence__are_equal(const skyvolt_msgs__srv__AssignJob_Response__Sequence * lhs, const skyvolt_msgs__srv__AssignJob_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__srv__AssignJob_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__srv__AssignJob_Response__Sequence__copy(
  const skyvolt_msgs__srv__AssignJob_Response__Sequence * input,
  skyvolt_msgs__srv__AssignJob_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__srv__AssignJob_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__srv__AssignJob_Response * data =
      (skyvolt_msgs__srv__AssignJob_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__srv__AssignJob_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__srv__AssignJob_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__srv__AssignJob_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
