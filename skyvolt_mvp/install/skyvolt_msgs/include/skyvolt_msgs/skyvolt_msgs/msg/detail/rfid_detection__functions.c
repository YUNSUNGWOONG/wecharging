// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from skyvolt_msgs:msg/RfidDetection.idl
// generated code does not contain a copyright notice
#include "skyvolt_msgs/msg/detail/rfid_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `robot_id`
// Member `tag_id`
#include "rosidl_runtime_c/string_functions.h"

bool
skyvolt_msgs__msg__RfidDetection__init(skyvolt_msgs__msg__RfidDetection * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    skyvolt_msgs__msg__RfidDetection__fini(msg);
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    skyvolt_msgs__msg__RfidDetection__fini(msg);
    return false;
  }
  // kind
  // tag_id
  if (!rosidl_runtime_c__String__init(&msg->tag_id)) {
    skyvolt_msgs__msg__RfidDetection__fini(msg);
    return false;
  }
  // expected_arclength_m
  // rssi
  return true;
}

void
skyvolt_msgs__msg__RfidDetection__fini(skyvolt_msgs__msg__RfidDetection * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // kind
  // tag_id
  rosidl_runtime_c__String__fini(&msg->tag_id);
  // expected_arclength_m
  // rssi
}

bool
skyvolt_msgs__msg__RfidDetection__are_equal(const skyvolt_msgs__msg__RfidDetection * lhs, const skyvolt_msgs__msg__RfidDetection * rhs)
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
  // kind
  if (lhs->kind != rhs->kind) {
    return false;
  }
  // tag_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tag_id), &(rhs->tag_id)))
  {
    return false;
  }
  // expected_arclength_m
  if (lhs->expected_arclength_m != rhs->expected_arclength_m) {
    return false;
  }
  // rssi
  if (lhs->rssi != rhs->rssi) {
    return false;
  }
  return true;
}

bool
skyvolt_msgs__msg__RfidDetection__copy(
  const skyvolt_msgs__msg__RfidDetection * input,
  skyvolt_msgs__msg__RfidDetection * output)
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
  // kind
  output->kind = input->kind;
  // tag_id
  if (!rosidl_runtime_c__String__copy(
      &(input->tag_id), &(output->tag_id)))
  {
    return false;
  }
  // expected_arclength_m
  output->expected_arclength_m = input->expected_arclength_m;
  // rssi
  output->rssi = input->rssi;
  return true;
}

skyvolt_msgs__msg__RfidDetection *
skyvolt_msgs__msg__RfidDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__RfidDetection * msg = (skyvolt_msgs__msg__RfidDetection *)allocator.allocate(sizeof(skyvolt_msgs__msg__RfidDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(skyvolt_msgs__msg__RfidDetection));
  bool success = skyvolt_msgs__msg__RfidDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
skyvolt_msgs__msg__RfidDetection__destroy(skyvolt_msgs__msg__RfidDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    skyvolt_msgs__msg__RfidDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
skyvolt_msgs__msg__RfidDetection__Sequence__init(skyvolt_msgs__msg__RfidDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__RfidDetection * data = NULL;

  if (size) {
    data = (skyvolt_msgs__msg__RfidDetection *)allocator.zero_allocate(size, sizeof(skyvolt_msgs__msg__RfidDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = skyvolt_msgs__msg__RfidDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        skyvolt_msgs__msg__RfidDetection__fini(&data[i - 1]);
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
skyvolt_msgs__msg__RfidDetection__Sequence__fini(skyvolt_msgs__msg__RfidDetection__Sequence * array)
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
      skyvolt_msgs__msg__RfidDetection__fini(&array->data[i]);
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

skyvolt_msgs__msg__RfidDetection__Sequence *
skyvolt_msgs__msg__RfidDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  skyvolt_msgs__msg__RfidDetection__Sequence * array = (skyvolt_msgs__msg__RfidDetection__Sequence *)allocator.allocate(sizeof(skyvolt_msgs__msg__RfidDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = skyvolt_msgs__msg__RfidDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
skyvolt_msgs__msg__RfidDetection__Sequence__destroy(skyvolt_msgs__msg__RfidDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    skyvolt_msgs__msg__RfidDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
skyvolt_msgs__msg__RfidDetection__Sequence__are_equal(const skyvolt_msgs__msg__RfidDetection__Sequence * lhs, const skyvolt_msgs__msg__RfidDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!skyvolt_msgs__msg__RfidDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
skyvolt_msgs__msg__RfidDetection__Sequence__copy(
  const skyvolt_msgs__msg__RfidDetection__Sequence * input,
  skyvolt_msgs__msg__RfidDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(skyvolt_msgs__msg__RfidDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    skyvolt_msgs__msg__RfidDetection * data =
      (skyvolt_msgs__msg__RfidDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!skyvolt_msgs__msg__RfidDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          skyvolt_msgs__msg__RfidDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!skyvolt_msgs__msg__RfidDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
