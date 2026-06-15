// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:srv/AssignJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__STRUCT_H_
#define SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'job'
#include "skyvolt_msgs/msg/detail/charging_job__struct.h"

/// Struct defined in srv/AssignJob in the package skyvolt_msgs.
typedef struct skyvolt_msgs__srv__AssignJob_Request
{
  skyvolt_msgs__msg__ChargingJob job;
} skyvolt_msgs__srv__AssignJob_Request;

// Struct for a sequence of skyvolt_msgs__srv__AssignJob_Request.
typedef struct skyvolt_msgs__srv__AssignJob_Request__Sequence
{
  skyvolt_msgs__srv__AssignJob_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__srv__AssignJob_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'assigned_robot_id'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/AssignJob in the package skyvolt_msgs.
typedef struct skyvolt_msgs__srv__AssignJob_Response
{
  bool accepted;
  rosidl_runtime_c__String assigned_robot_id;
  /// populated on rejection
  rosidl_runtime_c__String reason;
} skyvolt_msgs__srv__AssignJob_Response;

// Struct for a sequence of skyvolt_msgs__srv__AssignJob_Response.
typedef struct skyvolt_msgs__srv__AssignJob_Response__Sequence
{
  skyvolt_msgs__srv__AssignJob_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__srv__AssignJob_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__STRUCT_H_
