// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from skyvolt_msgs:action/TransportPile.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__STRUCT_H_
#define SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_Goal
{
  rosidl_runtime_c__String job_id;
  uint32_t source_branch_id;
  double source_arclength_m;
  uint32_t target_branch_id;
  double target_arclength_m;
} skyvolt_msgs__action__TransportPile_Goal;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_Goal.
typedef struct skyvolt_msgs__action__TransportPile_Goal__Sequence
{
  skyvolt_msgs__action__TransportPile_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'failure_reason'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_Result
{
  bool success;
  rosidl_runtime_c__String failure_reason;
  double final_lateral_error_mm;
  double final_longitudinal_error_mm;
  double total_time_s;
} skyvolt_msgs__action__TransportPile_Result;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_Result.
typedef struct skyvolt_msgs__action__TransportPile_Result__Sequence
{
  skyvolt_msgs__action__TransportPile_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_Feedback
{
  /// mirrors DockingState.STATE_*
  uint8_t phase;
  double progress_arclength_m;
} skyvolt_msgs__action__TransportPile_Feedback;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_Feedback.
typedef struct skyvolt_msgs__action__TransportPile_Feedback__Sequence
{
  skyvolt_msgs__action__TransportPile_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "skyvolt_msgs/action/detail/transport_pile__struct.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  skyvolt_msgs__action__TransportPile_Goal goal;
} skyvolt_msgs__action__TransportPile_SendGoal_Request;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_SendGoal_Request.
typedef struct skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence
{
  skyvolt_msgs__action__TransportPile_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} skyvolt_msgs__action__TransportPile_SendGoal_Response;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_SendGoal_Response.
typedef struct skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence
{
  skyvolt_msgs__action__TransportPile_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} skyvolt_msgs__action__TransportPile_GetResult_Request;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_GetResult_Request.
typedef struct skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence
{
  skyvolt_msgs__action__TransportPile_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__struct.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_GetResult_Response
{
  int8_t status;
  skyvolt_msgs__action__TransportPile_Result result;
} skyvolt_msgs__action__TransportPile_GetResult_Response;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_GetResult_Response.
typedef struct skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence
{
  skyvolt_msgs__action__TransportPile_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__struct.h"

/// Struct defined in action/TransportPile in the package skyvolt_msgs.
typedef struct skyvolt_msgs__action__TransportPile_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  skyvolt_msgs__action__TransportPile_Feedback feedback;
} skyvolt_msgs__action__TransportPile_FeedbackMessage;

// Struct for a sequence of skyvolt_msgs__action__TransportPile_FeedbackMessage.
typedef struct skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence
{
  skyvolt_msgs__action__TransportPile_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__STRUCT_H_
