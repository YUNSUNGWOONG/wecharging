// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:msg/TrackState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__STRUCT_H_
#define SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrackState in the package skyvolt_msgs.
/**
  * Localizer output: pose along the track.
 */
typedef struct skyvolt_msgs__msg__TrackState
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String robot_id;
  /// 0 = main loop; reserved for switches
  uint32_t branch_id;
  /// position along branch centerline
  double arclength_m;
  /// signed; positive = forward
  double speed_mps;
  /// KF variance estimate (m^2)
  double arclength_var;
} skyvolt_msgs__msg__TrackState;

// Struct for a sequence of skyvolt_msgs__msg__TrackState.
typedef struct skyvolt_msgs__msg__TrackState__Sequence
{
  skyvolt_msgs__msg__TrackState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__msg__TrackState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__STRUCT_H_
