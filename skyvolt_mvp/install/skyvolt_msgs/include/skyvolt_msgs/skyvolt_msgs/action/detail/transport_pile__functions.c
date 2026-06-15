// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from skyvolt_msgs:action/TransportPile.idl
// generated code does not contain a copyright notice
#include "skyvolt_msgs/action/detail/transport_pile__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `job_id`
#include "rosidl_runtime_c/string_functions.h"

bool
skyvolt_msgs__action__TransportPile_Goal__init(skyvolt_msgs__action__TransportPile_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__init(&msg->job_id)) {
    skyvolt_msgs__action__TransportPile_Goal__fini(msg);
    return false;
  }
  // source_branch_id
  // source_arclength_m
  // target_branch_id
  // target_arclength_m
  return true;
}

void
skyvolt_msgs__action__TransportPile_Goal__fini(skyvolt_msgs__action__TransportPile_Goal * msg)
{
  if (!msg) {
    return;
  }
  // job_id
  rosidl_runtime_c__String__fini(&msg->job_id);
  // source_branch_id
  // source_arclength_m
  // target_branch_id
  // target_arclength_m
}

bool
skyvolt_msgs__action__TransportPile_Goal__are_equal(const skyvolt_msgs__action__TransportPile_Goal * lhs, const skyvolt_msgs__action__TransportPile_Goal * rhs)
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
  // source_branch_id
  if (lhs->source_branch_id != rhs->source_branch_id) {
    return false;
  }
  // source_arclength_m
  if (lhs->source_arclength_m != rhs->source_arclength_m) {
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
  return true;
}

bool
skyvolt_msgs__action__TransportPile_Goal__copy(
  const skyvolt_msgs__action__TransportPile_Goal * input,
  skyvolt_msgs__action__TransportPile_Goal * output)
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
  // source_branch_id
  output->source_branch_id = input->source_branch_id;
  // source_arclength_m
  output->source_arclength_m = input->source_arclength_m;
  // target_branch_id
  output->target_branch_id = input->target_branch_id;
  // target_arclength_m
  output->target_arclength_m = input->target_arclength_m;
  return true;
}

skyvolt_msgs__action__TransportPile_Goal *
skyvolt_msgs__action__TransportPile_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Goal * msg = (skyvolt_msgs__action__TransportPile_Goal *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_Goal));
  bool success = skyvolt_msgs__action__TransportPile_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_Goal__destroy(skyvolt_msgs__action__TransportPile_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_Goal__Sequence__init(skyvolt_msgs__action__TransportPile_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Goal * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_Goal *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_Goal__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_Goal__Sequence__fini(skyvolt_msgs__action__TransportPile_Goal__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_Goal__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_Goal__Sequence *
skyvolt_msgs__action__TransportPile_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Goal__Sequence * array = (skyvolt_msgs__action__TransportPile_Goal__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_Goal__Sequence__destroy(skyvolt_msgs__action__TransportPile_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_Goal__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_Goal__Sequence * lhs, const skyvolt_msgs__action__TransportPile_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_Goal__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_Goal__Sequence * input,
  skyvolt_msgs__action__TransportPile_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_Goal * data =
      (skyvolt_msgs__action__TransportPile_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `failure_reason`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
skyvolt_msgs__action__TransportPile_Result__init(skyvolt_msgs__action__TransportPile_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // failure_reason
  if (!rosidl_runtime_c__String__init(&msg->failure_reason)) {
    skyvolt_msgs__action__TransportPile_Result__fini(msg);
    return false;
  }
  // final_lateral_error_mm
  // final_longitudinal_error_mm
  // total_time_s
  return true;
}

void
skyvolt_msgs__action__TransportPile_Result__fini(skyvolt_msgs__action__TransportPile_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // failure_reason
  rosidl_runtime_c__String__fini(&msg->failure_reason);
  // final_lateral_error_mm
  // final_longitudinal_error_mm
  // total_time_s
}

bool
skyvolt_msgs__action__TransportPile_Result__are_equal(const skyvolt_msgs__action__TransportPile_Result * lhs, const skyvolt_msgs__action__TransportPile_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->failure_reason), &(rhs->failure_reason)))
  {
    return false;
  }
  // final_lateral_error_mm
  if (lhs->final_lateral_error_mm != rhs->final_lateral_error_mm) {
    return false;
  }
  // final_longitudinal_error_mm
  if (lhs->final_longitudinal_error_mm != rhs->final_longitudinal_error_mm) {
    return false;
  }
  // total_time_s
  if (lhs->total_time_s != rhs->total_time_s) {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_Result__copy(
  const skyvolt_msgs__action__TransportPile_Result * input,
  skyvolt_msgs__action__TransportPile_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // failure_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->failure_reason), &(output->failure_reason)))
  {
    return false;
  }
  // final_lateral_error_mm
  output->final_lateral_error_mm = input->final_lateral_error_mm;
  // final_longitudinal_error_mm
  output->final_longitudinal_error_mm = input->final_longitudinal_error_mm;
  // total_time_s
  output->total_time_s = input->total_time_s;
  return true;
}

skyvolt_msgs__action__TransportPile_Result *
skyvolt_msgs__action__TransportPile_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Result * msg = (skyvolt_msgs__action__TransportPile_Result *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_Result));
  bool success = skyvolt_msgs__action__TransportPile_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_Result__destroy(skyvolt_msgs__action__TransportPile_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_Result__Sequence__init(skyvolt_msgs__action__TransportPile_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Result * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_Result *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_Result__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_Result__Sequence__fini(skyvolt_msgs__action__TransportPile_Result__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_Result__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_Result__Sequence *
skyvolt_msgs__action__TransportPile_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Result__Sequence * array = (skyvolt_msgs__action__TransportPile_Result__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_Result__Sequence__destroy(skyvolt_msgs__action__TransportPile_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_Result__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_Result__Sequence * lhs, const skyvolt_msgs__action__TransportPile_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_Result__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_Result__Sequence * input,
  skyvolt_msgs__action__TransportPile_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_Result * data =
      (skyvolt_msgs__action__TransportPile_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
skyvolt_msgs__action__TransportPile_Feedback__init(skyvolt_msgs__action__TransportPile_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // phase
  // progress_arclength_m
  return true;
}

void
skyvolt_msgs__action__TransportPile_Feedback__fini(skyvolt_msgs__action__TransportPile_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // phase
  // progress_arclength_m
}

bool
skyvolt_msgs__action__TransportPile_Feedback__are_equal(const skyvolt_msgs__action__TransportPile_Feedback * lhs, const skyvolt_msgs__action__TransportPile_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // phase
  if (lhs->phase != rhs->phase) {
    return false;
  }
  // progress_arclength_m
  if (lhs->progress_arclength_m != rhs->progress_arclength_m) {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_Feedback__copy(
  const skyvolt_msgs__action__TransportPile_Feedback * input,
  skyvolt_msgs__action__TransportPile_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // phase
  output->phase = input->phase;
  // progress_arclength_m
  output->progress_arclength_m = input->progress_arclength_m;
  return true;
}

skyvolt_msgs__action__TransportPile_Feedback *
skyvolt_msgs__action__TransportPile_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Feedback * msg = (skyvolt_msgs__action__TransportPile_Feedback *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_Feedback));
  bool success = skyvolt_msgs__action__TransportPile_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_Feedback__destroy(skyvolt_msgs__action__TransportPile_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_Feedback__Sequence__init(skyvolt_msgs__action__TransportPile_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Feedback * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_Feedback *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_Feedback__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_Feedback__Sequence__fini(skyvolt_msgs__action__TransportPile_Feedback__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_Feedback__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_Feedback__Sequence *
skyvolt_msgs__action__TransportPile_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_Feedback__Sequence * array = (skyvolt_msgs__action__TransportPile_Feedback__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_Feedback__Sequence__destroy(skyvolt_msgs__action__TransportPile_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_Feedback__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_Feedback__Sequence * lhs, const skyvolt_msgs__action__TransportPile_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_Feedback__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_Feedback__Sequence * input,
  skyvolt_msgs__action__TransportPile_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_Feedback * data =
      (skyvolt_msgs__action__TransportPile_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__functions.h"

bool
skyvolt_msgs__action__TransportPile_SendGoal_Request__init(skyvolt_msgs__action__TransportPile_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!skyvolt_msgs__action__TransportPile_Goal__init(&msg->goal)) {
    skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(skyvolt_msgs__action__TransportPile_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  skyvolt_msgs__action__TransportPile_Goal__fini(&msg->goal);
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Request__are_equal(const skyvolt_msgs__action__TransportPile_SendGoal_Request * lhs, const skyvolt_msgs__action__TransportPile_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!skyvolt_msgs__action__TransportPile_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Request__copy(
  const skyvolt_msgs__action__TransportPile_SendGoal_Request * input,
  skyvolt_msgs__action__TransportPile_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!skyvolt_msgs__action__TransportPile_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__action__TransportPile_SendGoal_Request *
skyvolt_msgs__action__TransportPile_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_SendGoal_Request * msg = (skyvolt_msgs__action__TransportPile_SendGoal_Request *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Request));
  bool success = skyvolt_msgs__action__TransportPile_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_SendGoal_Request__destroy(skyvolt_msgs__action__TransportPile_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__init(skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_SendGoal_Request * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_SendGoal_Request *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__fini(skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence *
skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * array = (skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__destroy(skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * lhs, const skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * input,
  skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_SendGoal_Request * data =
      (skyvolt_msgs__action__TransportPile_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
skyvolt_msgs__action__TransportPile_SendGoal_Response__init(skyvolt_msgs__action__TransportPile_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    skyvolt_msgs__action__TransportPile_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__action__TransportPile_SendGoal_Response__fini(skyvolt_msgs__action__TransportPile_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Response__are_equal(const skyvolt_msgs__action__TransportPile_SendGoal_Response * lhs, const skyvolt_msgs__action__TransportPile_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Response__copy(
  const skyvolt_msgs__action__TransportPile_SendGoal_Response * input,
  skyvolt_msgs__action__TransportPile_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__action__TransportPile_SendGoal_Response *
skyvolt_msgs__action__TransportPile_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_SendGoal_Response * msg = (skyvolt_msgs__action__TransportPile_SendGoal_Response *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Response));
  bool success = skyvolt_msgs__action__TransportPile_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_SendGoal_Response__destroy(skyvolt_msgs__action__TransportPile_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__init(skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_SendGoal_Response * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_SendGoal_Response *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_SendGoal_Response__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__fini(skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_SendGoal_Response__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence *
skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * array = (skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__destroy(skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * lhs, const skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * input,
  skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_SendGoal_Response * data =
      (skyvolt_msgs__action__TransportPile_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
skyvolt_msgs__action__TransportPile_GetResult_Request__init(skyvolt_msgs__action__TransportPile_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    skyvolt_msgs__action__TransportPile_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__action__TransportPile_GetResult_Request__fini(skyvolt_msgs__action__TransportPile_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Request__are_equal(const skyvolt_msgs__action__TransportPile_GetResult_Request * lhs, const skyvolt_msgs__action__TransportPile_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Request__copy(
  const skyvolt_msgs__action__TransportPile_GetResult_Request * input,
  skyvolt_msgs__action__TransportPile_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__action__TransportPile_GetResult_Request *
skyvolt_msgs__action__TransportPile_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_GetResult_Request * msg = (skyvolt_msgs__action__TransportPile_GetResult_Request *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_GetResult_Request));
  bool success = skyvolt_msgs__action__TransportPile_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_GetResult_Request__destroy(skyvolt_msgs__action__TransportPile_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__init(skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_GetResult_Request * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_GetResult_Request *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_GetResult_Request__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__fini(skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_GetResult_Request__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence *
skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * array = (skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__destroy(skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * lhs, const skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * input,
  skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_GetResult_Request * data =
      (skyvolt_msgs__action__TransportPile_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__functions.h"

bool
skyvolt_msgs__action__TransportPile_GetResult_Response__init(skyvolt_msgs__action__TransportPile_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!skyvolt_msgs__action__TransportPile_Result__init(&msg->result)) {
    skyvolt_msgs__action__TransportPile_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__action__TransportPile_GetResult_Response__fini(skyvolt_msgs__action__TransportPile_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  skyvolt_msgs__action__TransportPile_Result__fini(&msg->result);
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Response__are_equal(const skyvolt_msgs__action__TransportPile_GetResult_Response * lhs, const skyvolt_msgs__action__TransportPile_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!skyvolt_msgs__action__TransportPile_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Response__copy(
  const skyvolt_msgs__action__TransportPile_GetResult_Response * input,
  skyvolt_msgs__action__TransportPile_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!skyvolt_msgs__action__TransportPile_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__action__TransportPile_GetResult_Response *
skyvolt_msgs__action__TransportPile_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_GetResult_Response * msg = (skyvolt_msgs__action__TransportPile_GetResult_Response *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_GetResult_Response));
  bool success = skyvolt_msgs__action__TransportPile_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_GetResult_Response__destroy(skyvolt_msgs__action__TransportPile_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__init(skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_GetResult_Response * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_GetResult_Response *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_GetResult_Response__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__fini(skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_GetResult_Response__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence *
skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * array = (skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__destroy(skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * lhs, const skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * input,
  skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_GetResult_Response * data =
      (skyvolt_msgs__action__TransportPile_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__functions.h"

bool
skyvolt_msgs__action__TransportPile_FeedbackMessage__init(skyvolt_msgs__action__TransportPile_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!skyvolt_msgs__action__TransportPile_Feedback__init(&msg->feedback)) {
    skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(skyvolt_msgs__action__TransportPile_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  skyvolt_msgs__action__TransportPile_Feedback__fini(&msg->feedback);
}

bool
skyvolt_msgs__action__TransportPile_FeedbackMessage__are_equal(const skyvolt_msgs__action__TransportPile_FeedbackMessage * lhs, const skyvolt_msgs__action__TransportPile_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!skyvolt_msgs__action__TransportPile_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_FeedbackMessage__copy(
  const skyvolt_msgs__action__TransportPile_FeedbackMessage * input,
  skyvolt_msgs__action__TransportPile_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!skyvolt_msgs__action__TransportPile_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

skyvolt_msgs__action__TransportPile_FeedbackMessage *
skyvolt_msgs__action__TransportPile_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_FeedbackMessage * msg = (skyvolt_msgs__action__TransportPile_FeedbackMessage *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__action__TransportPile_FeedbackMessage));
  bool success = skyvolt_msgs__action__TransportPile_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__action__TransportPile_FeedbackMessage__destroy(skyvolt_msgs__action__TransportPile_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__init(skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_FeedbackMessage * data = NULL;

  if (size) {
    data = (skyvolt_msgs__action__TransportPile_FeedbackMessage *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__action__TransportPile_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__action__TransportPile_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(&data[i - 1]);
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
skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__fini(skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * array)
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
      skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(&array->data[i]);
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

skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence *
skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * array = (skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__destroy(skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__are_equal(const skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * lhs, const skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__copy(
  const skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * input,
  skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__action__TransportPile_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__action__TransportPile_FeedbackMessage * data =
      (skyvolt_msgs__action__TransportPile_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__action__TransportPile_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__action__TransportPile_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__action__TransportPile_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
