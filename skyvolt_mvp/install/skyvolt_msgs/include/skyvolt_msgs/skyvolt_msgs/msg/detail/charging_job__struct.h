// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:msg/ChargingJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__STRUCT_H_
#define SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'job_id'
// Member 'requester_id'
#include "rosidl_runtime_c/string.h"
// Member 'submitted_at'
// Member 'deadline'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ChargingJob in the package skyvolt_msgs.
/**
  * A request to deliver a charging pile to a target support point.
 */
typedef struct skyvolt_msgs__msg__ChargingJob
{
  rosidl_runtime_c__String job_id;
  /// vehicle / parking-spot identifier
  rosidl_runtime_c__String requester_id;
  uint32_t target_branch_id;
  double target_arclength_m;
  builtin_interfaces__msg__Time submitted_at;
  /// 0 = no deadline
  builtin_interfaces__msg__Time deadline;
  /// 0=low, 255=urgent
  uint8_t priority;
} skyvolt_msgs__msg__ChargingJob;

// Struct for a sequence of skyvolt_msgs__msg__ChargingJob.
typedef struct skyvolt_msgs__msg__ChargingJob__Sequence
{
  skyvolt_msgs__msg__ChargingJob * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__msg__ChargingJob__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__STRUCT_H_
