// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from skyvolt_msgs:msg/ChargingJob.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "skyvolt_msgs/msg/detail/charging_job__rosidl_typesupport_introspection_c.h"
#include "skyvolt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "skyvolt_msgs/msg/detail/charging_job__functions.h"
#include "skyvolt_msgs/msg/detail/charging_job__struct.h"


// Include directives for member types
// Member `job_id`
// Member `requester_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `submitted_at`
// Member `deadline`
#include "builtin_interfaces/msg/time.h"
// Member `submitted_at`
// Member `deadline`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  skyvolt_msgs__msg__ChargingJob__init(message_memory);
}

void skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_fini_function(void * message_memory)
{
  skyvolt_msgs__msg__ChargingJob__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_member_array[7] = {
  {
    "job_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, job_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "requester_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, requester_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_branch_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, target_branch_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_arclength_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, target_arclength_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "submitted_at",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, submitted_at),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "deadline",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, deadline),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "priority",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__ChargingJob, priority),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_members = {
  "skyvolt_msgs__msg",  // message namespace
  "ChargingJob",  // message name
  7,  // number of fields
  sizeof(skyvolt_msgs__msg__ChargingJob),
  skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_member_array,  // message members
  skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_init_function,  // function to initialize message memory (memory has to be allocated)
  skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_type_support_handle = {
  0,
  &skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_skyvolt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, msg, ChargingJob)() {
  skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_type_support_handle.typesupport_identifier) {
    skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &skyvolt_msgs__msg__ChargingJob__rosidl_typesupport_introspection_c__ChargingJob_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
