// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:msg/RfidDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__STRUCT_H_
#define SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'KIND_CUE'.
enum
{
  skyvolt_msgs__msg__RfidDetection__KIND_CUE = 1
};

/// Constant 'KIND_POSITIONING'.
enum
{
  skyvolt_msgs__msg__RfidDetection__KIND_POSITIONING = 2
};

/// Constant 'KIND_DOCKING'.
enum
{
  skyvolt_msgs__msg__RfidDetection__KIND_DOCKING = 3
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'robot_id'
// Member 'tag_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RfidDetection in the package skyvolt_msgs.
/**
  * RFID tag detection on the track.
  *
  * Tag kinds (paper section 3):
  *   1 = CUE         -> first deceleration trigger
  *   2 = POSITIONING -> second deceleration trigger
  *   3 = DOCKING     -> served by photoeye, included for completeness
 */
typedef struct skyvolt_msgs__msg__RfidDetection
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String robot_id;
  uint8_t kind;
  /// globally unique tag identifier
  rosidl_runtime_c__String tag_id;
  /// pre-mapped tag arclength along the track
  double expected_arclength_m;
  /// for future range estimation
  double rssi;
} skyvolt_msgs__msg__RfidDetection;

// Struct for a sequence of skyvolt_msgs__msg__RfidDetection.
typedef struct skyvolt_msgs__msg__RfidDetection__Sequence
{
  skyvolt_msgs__msg__RfidDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__msg__RfidDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__STRUCT_H_
