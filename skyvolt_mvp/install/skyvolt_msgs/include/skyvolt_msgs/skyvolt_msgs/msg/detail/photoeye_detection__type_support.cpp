// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from skyvolt_msgs:msg/PhotoeyeDetection.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "skyvolt_msgs/msg/detail/photoeye_detection__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace skyvolt_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PhotoeyeDetection_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) skyvolt_msgs::msg::PhotoeyeDetection(_init);
}

void PhotoeyeDetection_fini_function(void * message_memory)
{
  auto typed_message = static_cast<skyvolt_msgs::msg::PhotoeyeDetection *>(message_memory);
  typed_message->~PhotoeyeDetection();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PhotoeyeDetection_message_member_array[4] = {
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs::msg::PhotoeyeDetection, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs::msg::PhotoeyeDetection, robot_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "triggered",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs::msg::PhotoeyeDetection, triggered),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "docking_card_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(skyvolt_msgs::msg::PhotoeyeDetection, docking_card_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PhotoeyeDetection_message_members = {
  "skyvolt_msgs::msg",  // message namespace
  "PhotoeyeDetection",  // message name
  4,  // number of fields
  sizeof(skyvolt_msgs::msg::PhotoeyeDetection),
  PhotoeyeDetection_message_member_array,  // message members
  PhotoeyeDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  PhotoeyeDetection_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PhotoeyeDetection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PhotoeyeDetection_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace skyvolt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<skyvolt_msgs::msg::PhotoeyeDetection>()
{
  return &::skyvolt_msgs::msg::rosidl_typesupport_introspection_cpp::PhotoeyeDetection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, skyvolt_msgs, msg, PhotoeyeDetection)() {
  return &::skyvolt_msgs::msg::rosidl_typesupport_introspection_cpp::PhotoeyeDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
