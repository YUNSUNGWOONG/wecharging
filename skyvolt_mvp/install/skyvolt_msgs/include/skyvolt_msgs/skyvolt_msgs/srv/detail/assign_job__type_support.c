// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from skyvolt_msgs:srv/AssignJob.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "skyvolt_msgs/srv/detail/assign_job__rosidl_typesupport_introspection_c.h"
#include "skyvolt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "skyvolt_msgs/srv/detail/assign_job__functions.h"
#include "skyvolt_msgs/srv/detail/assign_job__struct.h"


// Include directives for member types
// Member `job`
#include "skyvolt_msgs/msg/charging_job.h"
// Member `job`
#include "skyvolt_msgs/msg/detail/charging_job__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  skyvolt_msgs__srv__AssignJob_Request__init(message_memory);
}

void skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_fini_function(void * message_memory)
{
  skyvolt_msgs__srv__AssignJob_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_member_array[1] = {
  {
    "job",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__srv__AssignJob_Request, job),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_members = {
  "skyvolt_msgs__srv",  // message namespace
  "AssignJob_Request",  // message name
  1,  // number of fields
  sizeof(skyvolt_msgs__srv__AssignJob_Request),
  skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_member_array,  // message members
  skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_type_support_handle = {
  0,
  &skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_skyvolt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob_Request)() {
  skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, msg, ChargingJob)();
  if (!skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_type_support_handle.typesupport_identifier) {
    skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &skyvolt_msgs__srv__AssignJob_Request__rosidl_typesupport_introspection_c__AssignJob_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "skyvolt_msgs/srv/detail/assign_job__rosidl_typesupport_introspection_c.h"
// already included above
// #include "skyvolt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "skyvolt_msgs/srv/detail/assign_job__functions.h"
// already included above
// #include "skyvolt_msgs/srv/detail/assign_job__struct.h"


// Include directives for member types
// Member `assigned_robot_id`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  skyvolt_msgs__srv__AssignJob_Response__init(message_memory);
}

void skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_fini_function(void * message_memory)
{
  skyvolt_msgs__srv__AssignJob_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_member_array[3] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__srv__AssignJob_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "assigned_robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__srv__AssignJob_Response, assigned_robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__srv__AssignJob_Response, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_members = {
  "skyvolt_msgs__srv",  // message namespace
  "AssignJob_Response",  // message name
  3,  // number of fields
  sizeof(skyvolt_msgs__srv__AssignJob_Response),
  skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_member_array,  // message members
  skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_type_support_handle = {
  0,
  &skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_skyvolt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob_Response)() {
  if (!skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_type_support_handle.typesupport_identifier) {
    skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &skyvolt_msgs__srv__AssignJob_Response__rosidl_typesupport_introspection_c__AssignJob_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "skyvolt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "skyvolt_msgs/srv/detail/assign_job__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_members = {
  "skyvolt_msgs__srv",  // service namespace
  "AssignJob",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_Request_message_type_support_handle,
  NULL  // response message
  // skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_Response_message_type_support_handle
};

static rosidl_service_type_support_t skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_type_support_handle = {
  0,
  &skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_skyvolt_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob)() {
  if (!skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_type_support_handle.typesupport_identifier) {
    skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, srv, AssignJob_Response)()->data;
  }

  return &skyvolt_msgs__srv__detail__assign_job__rosidl_typesupport_introspection_c__AssignJob_service_type_support_handle;
}
