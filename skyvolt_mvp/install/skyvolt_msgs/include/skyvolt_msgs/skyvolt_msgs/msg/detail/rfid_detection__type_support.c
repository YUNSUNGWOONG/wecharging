// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from skyvolt_msgs:msg/RfidDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "skyvolt_msgs/msg/detail/rfid_detection__rosidl_typesupport_introspection_c.h"
#include "skyvolt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "skyvolt_msgs/msg/detail/rfid_detection__functions.h"
#include "skyvolt_msgs/msg/detail/rfid_detection__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `robot_id`
// Member `tag_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  skyvolt_msgs__msg__RfidDetection__init(message_memory);
}

void skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_fini_function(void * message_memory)
{
  skyvolt_msgs__msg__RfidDetection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_member_array[6] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__RfidDetection, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__RfidDetection, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__RfidDetection, kind),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tag_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__RfidDetection, tag_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "expected_arclength_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__RfidDetection, expected_arclength_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs__msg__RfidDetection, rssi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_members = {
  "skyvolt_msgs__msg",  // message namespace
  "RfidDetection",  // message name
  6,  // number of fields
  sizeof(skyvolt_msgs__msg__RfidDetection),
  skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_member_array,  // message members
  skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_type_support_handle = {
  0,
  &skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_skyvolt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, skyvolt_msgs, msg, RfidDetection)() {
  skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_type_support_handle.typesupport_identifier) {
    skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &skyvolt_msgs__msg__RfidDetection__rosidl_typesupport_introspection_c__RfidDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
