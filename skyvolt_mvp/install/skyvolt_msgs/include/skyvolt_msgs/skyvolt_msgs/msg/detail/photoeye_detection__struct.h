// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:msg/PhotoeyeDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__STRUCT_H_
#define SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__STRUCT_H_

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
// Member 'robot_id'
// Member 'docking_card_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PhotoeyeDetection in the package skyvolt_msgs.
/**
  * Photoelectric sensor binary state at the docking card.
  * When triggered, the robot must stop immediately (paper Fig.3d).
 */
typedef struct skyvolt_msgs__msg__PhotoeyeDetection
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String robot_id;
  bool triggered;
  /// if known
  rosidl_runtime_c__String docking_card_id;
} skyvolt_msgs__msg__PhotoeyeDetection;

// Struct for a sequence of skyvolt_msgs__msg__PhotoeyeDetection.
typedef struct skyvolt_msgs__msg__PhotoeyeDetection__Sequence
{
  skyvolt_msgs__msg__PhotoeyeDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__msg__PhotoeyeDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__STRUCT_H_
