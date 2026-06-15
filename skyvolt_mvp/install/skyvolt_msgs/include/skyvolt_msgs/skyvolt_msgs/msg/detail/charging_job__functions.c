// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from skyvolt_msgs:msg/ChargingJob.idl
// generated code does not contain a copyright notice
#include "skyvolt_msgs/msg/detail/charging_job__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `job_id`
// Member `requester_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `submitted_at`
// Member `deadline`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
skyvolt_msgs__msg__ChargingJob__init(skyvolt_msgs__msg__ChargingJob * msg)
{
  if (!msg) {
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__init(&msg->job_id)) {
    skyvolt_msgs__msg__ChargingJob__fini(msg);
    return false;
  }
  // requester_id
  if (!rosidl_runtime_c__String__init(&msg->requester_id)) {
    skyvolt_msgs__msg__ChargingJob__fini(msg);
    return false;
  }
  // target_branch_id
  // target_arclength_m
  // submitted_at
  if (!builtin_interfaces__msg__Time__init(&msg->submitted_at)) {
    skyvolt_msgs__msg__ChargingJob__fini(msg);
    return false;
  }
  // deadline
  if (!builtin_interfaces__msg__Time__init(&msg->deadline)) {
    skyvolt_msgs__msg__ChargingJob__fini(msg);
    return false;
  }
  // priority
  return true;
}

void
skyvolt_msgs__msg__ChargingJob__fini(skyvolt_msgs__msg__ChargingJob * msg)
{
  if (!msg) {
    return;
  }
  // job_id
  rosidl_runtime_c__String__fini(&msg->job_id);
  // requester_id
  rosidl_runtime_c__String__fini(&msg->requester_id);
  // target_branch_id
  // target_arclength_m
  // submitted_at
  builtin_interfaces__msg__Time__fini(&msg->submitted_at);
  // deadline
  builtin_interfaces__msg__Time__fini(&msg->deadline);
  // priority
}

bool
skyvolt_msgs__msg__ChargingJob__are_equal(const skyvolt_msgs__msg__ChargingJob * lhs, const skyvolt_msgs__msg__ChargingJob * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->job_id), &(rhs->job_id)))
  {
    return false;
  }
  // requester_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->requester_id), &(rhs->requester_id)))
  {
    return false;
  }
  // target_branch_id
  if (lhs->target_branch_id != rhs->target_branch_id) {
    return false;
  }
  // target_arclength_m
  if (lhs->target_arclength_m != rhs->target_arclength_m) {
    return false;
  }
  // submitted_at
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->submitted_at), &(rhs->submitted_at)))
  {
    return false;
  }
  // deadline
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->deadline), &(rhs->deadline)))
  {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__msg__ChargingJob__copy(
  const skyvolt_msgs__msg__ChargingJob * input,
  skyvolt_msgs__msg__ChargingJob * output)
{
  if (!input || !output) {
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__copy(
      &(input->job_id), &(output->job_id)))
  {
    return false;
  }
  // requester_id
  if (!rosidl_runtime_c__String__copy(
      &(input->requester_id), &(output->requester_id)))
  {
    return false;
  }
  // target_branch_id
  output->target_branch_id = input->target_branch_id;
  // target_arclength_m
  output->target_arclength_m = input->target_arclength_m;
  // submitted_at
  if (!builtin_interfaces__msg__Time__copy(
      &(input->submitted_at), &(output->submitted_at)))
  {
    return false;
  }
  // deadline
  if (!builtin_interfaces__msg__Time__copy(
      &(input->deadline), &(output->deadline)))
  {
    return false;
  }
  // priority
  output->priority = input->priority;
  return true;
}

skyvolt_msgs__msg__ChargingJob *
skyvolt_msgs__msg__ChargingJob__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__ChargingJob * msg = (skyvolt_msgs__msg__ChargingJob *)allocator.allocate(sizeof(skyvolt_msgs__msg__ChargingJob), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__msg__ChargingJob));
  bool success = skyvolt_msgs__msg__ChargingJob__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__msg__ChargingJob__destroy(skyvolt_msgs__msg__ChargingJob * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__msg__ChargingJob__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__msg__ChargingJob__Sequence__init(skyvolt_msgs__msg__ChargingJob__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__ChargingJob * data = NULL;

  if (size) {
    data = (skyvolt_msgs__msg__ChargingJob *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__msg__ChargingJob), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__msg__ChargingJob__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__msg__ChargingJob__fini(&data[i - 1]);
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
skyvolt_msgs__msg__ChargingJob__Sequence__fini(skyvolt_msgs__msg__ChargingJob__Sequence * array)
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
      skyvolt_msgs__msg__ChargingJob__fini(&array->data[i]);
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

skyvolt_msgs__msg__ChargingJob__Sequence *
skyvolt_msgs__msg__ChargingJob__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__ChargingJob__Sequence * array = (skyvolt_msgs__msg__ChargingJob__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__msg__ChargingJob__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__msg__ChargingJob__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__msg__ChargingJob__Sequence__destroy(skyvolt_msgs__msg__ChargingJob__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__msg__ChargingJob__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__msg__ChargingJob__Sequence__are_equal(const skyvolt_msgs__msg__ChargingJob__Sequence * lhs, const skyvolt_msgs__msg__ChargingJob__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__msg__ChargingJob__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__msg__ChargingJob__Sequence__copy(
  const skyvolt_msgs__msg__ChargingJob__Sequence * input,
  skyvolt_msgs__msg__ChargingJob__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__msg__ChargingJob);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__msg__ChargingJob * data =
      (skyvolt_msgs__msg__ChargingJob *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__msg__ChargingJob__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__msg__ChargingJob__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__msg__ChargingJob__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
