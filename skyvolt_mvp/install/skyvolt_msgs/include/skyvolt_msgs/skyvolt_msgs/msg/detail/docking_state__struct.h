// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:msg/DockingState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__STRUCT_H_
#define SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_IDLE'.
enum
{
  skyvolt_msgs__msg__DockingState__STATE_IDLE = 0
};

/// Constant 'STATE_CRUISE'.
enum
{
  skyvolt_msgs__msg__DockingState__STATE_CRUISE = 1
};

/// Constant 'STATE_APPROACH_1'.
enum
{
  skyvolt_msgs__msg__DockingState__STATE_APPROACH_1 = 2
};

/// Constant 'STATE_APPROACH_2'.
enum
{
  skyvolt_msgs__msg__DockingState__STATE_APPROACH_2 = 3
};

/// Constant 'STATE_DOCKED'.
enum
{
  skyvolt_msgs__msg__DockingState__STATE_DOCKED = 4
};

/// Constant 'STATE_FAULT'.
enum
{
  skyvolt_msgs__msg__DockingState__STATE_FAULT = 255
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DockingState in the package skyvolt_msgs.
/**
  * Docking FSM state, exposed for telemetry/dashboards.
 */
typedef struct skyvolt_msgs__msg__DockingState
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String robot_id;
  uint8_t state;
  /// last measured (-1 if unknown)
  double lateral_error_mm;
  /// last measured (-1 if unknown)
  double longitudinal_error_mm;
} skyvolt_msgs__msg__DockingState;

// Struct for a sequence of skyvolt_msgs__msg__DockingState.
typedef struct skyvolt_msgs__msg__DockingState__Sequence
{
  skyvolt_msgs__msg__DockingState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__msg__DockingState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__STRUCT_H_
