// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:msg/FleetStatus.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__STRUCT_H_
#define SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/FleetStatus in the package skyvolt_msgs.
/**
  * Aggregate fleet status snapshot.
 */
typedef struct skyvolt_msgs__msg__FleetStatus
{
  builtin_interfaces__msg__Time stamp;
  uint32_t num_robots;
  uint32_t num_pending_jobs;
  uint32_t num_active_jobs;
  uint32_t num_reservations;
  double avg_job_age_s;
} skyvolt_msgs__msg__FleetStatus;

// Struct for a sequence of skyvolt_msgs__msg__FleetStatus.
typedef struct skyvolt_msgs__msg__FleetStatus__Sequence
{
  skyvolt_msgs__msg__FleetStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__msg__FleetStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__STRUCT_H_
