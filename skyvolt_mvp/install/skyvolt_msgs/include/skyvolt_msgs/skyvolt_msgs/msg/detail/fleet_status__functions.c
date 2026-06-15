// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from skyvolt_msgs:msg/FleetStatus.idl
// generated code does not contain a copyright notice
#include "skyvolt_msgs/msg/detail/fleet_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
skyvolt_msgs__msg__FleetStatus__init(skyvolt_msgs__msg__FleetStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    skyvolt_msgs__msg__FleetStatus__fini(msg);
    return false;
  }
  // num_robots
  // num_pending_jobs
  // num_active_jobs
  // num_reservations
  // avg_job_age_s
  return true;
}

void
skyvolt_msgs__msg__FleetStatus__fini(skyvolt_msgs__msg__FleetStatus * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // num_robots
  // num_pending_jobs
  // num_active_jobs
  // num_reservations
  // avg_job_age_s
}

bool
skyvolt_msgs__msg__FleetStatus__are_equal(const skyvolt_msgs__msg__FleetStatus * lhs, const skyvolt_msgs__msg__FleetStatus * rhs)
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
  // num_robots
  if (lhs->num_robots != rhs->num_robots) {
    return false;
  }
  // num_pending_jobs
  if (lhs->num_pending_jobs != rhs->num_pending_jobs) {
    return false;
  }
  // num_active_jobs
  if (lhs->num_active_jobs != rhs->num_active_jobs) {
    return false;
  }
  // num_reservations
  if (lhs->num_reservations != rhs->num_reservations) {
    return false;
  }
  // avg_job_age_s
  if (lhs->avg_job_age_s != rhs->avg_job_age_s) {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__msg__FleetStatus__copy(
  const skyvolt_msgs__msg__FleetStatus * input,
  skyvolt_msgs__msg__FleetStatus * output)
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
  // num_robots
  output->num_robots = input->num_robots;
  // num_pending_jobs
  output->num_pending_jobs = input->num_pending_jobs;
  // num_active_jobs
  output->num_active_jobs = input->num_active_jobs;
  // num_reservations
  output->num_reservations = input->num_reservations;
  // avg_job_age_s
  output->avg_job_age_s = input->avg_job_age_s;
  return true;
}

skyvolt_msgs__msg__FleetStatus *
skyvolt_msgs__msg__FleetStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__FleetStatus * msg = (skyvolt_msgs__msg__FleetStatus *)allocator.allocate(sizeof(skyvolt_msgs__msg__FleetStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__msg__FleetStatus));
  bool success = skyvolt_msgs__msg__FleetStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__msg__FleetStatus__destroy(skyvolt_msgs__msg__FleetStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__msg__FleetStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__msg__FleetStatus__Sequence__init(skyvolt_msgs__msg__FleetStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__FleetStatus * data = NULL;

  if (size) {
    data = (skyvolt_msgs__msg__FleetStatus *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__msg__FleetStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__msg__FleetStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__msg__FleetStatus__fini(&data[i - 1]);
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
skyvolt_msgs__msg__FleetStatus__Sequence__fini(skyvolt_msgs__msg__FleetStatus__Sequence * array)
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
      skyvolt_msgs__msg__FleetStatus__fini(&array->data[i]);
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

skyvolt_msgs__msg__FleetStatus__Sequence *
skyvolt_msgs__msg__FleetStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__FleetStatus__Sequence * array = (skyvolt_msgs__msg__FleetStatus__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__msg__FleetStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__msg__FleetStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__msg__FleetStatus__Sequence__destroy(skyvolt_msgs__msg__FleetStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__msg__FleetStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__msg__FleetStatus__Sequence__are_equal(const skyvolt_msgs__msg__FleetStatus__Sequence * lhs, const skyvolt_msgs__msg__FleetStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__msg__FleetStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__msg__FleetStatus__Sequence__copy(
  const skyvolt_msgs__msg__FleetStatus__Sequence * input,
  skyvolt_msgs__msg__FleetStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__msg__FleetStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__msg__FleetStatus * data =
      (skyvolt_msgs__msg__FleetStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__msg__FleetStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__msg__FleetStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__msg__FleetStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
