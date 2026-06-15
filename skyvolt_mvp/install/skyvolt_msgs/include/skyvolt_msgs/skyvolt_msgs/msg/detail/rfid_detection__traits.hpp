// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from skyvolt_msgs:msg/RfidDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__TRAITS_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "skyvolt_msgs/msg/detail/rfid_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace skyvolt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RfidDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: kind
  {
    out << "kind: ";
    rosidl_generator_traits::value_to_yaml(msg.kind, out);
    out << ", ";
  }

  // member: tag_id
  {
    out << "tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_id, out);
    out << ", ";
  }

  // member: expected_arclength_m
  {
    out << "expected_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.expected_arclength_m, out);
    out << ", ";
  }

  // member: rssi
  {
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RfidDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: kind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kind: ";
    rosidl_generator_traits::value_to_yaml(msg.kind, out);
    out << "\n";
  }

  // member: tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_id, out);
    out << "\n";
  }

  // member: expected_arclength_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expected_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.expected_arclength_m, out);
    out << "\n";
  }

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RfidDetection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::msg::RfidDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::msg::RfidDetection & msg)
{
  return skyvolt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::msg::RfidDetection>()
{
  return "skyvolt_msgs::msg::RfidDetection";
}

template<>
inline const char * name<skyvolt_msgs::msg::RfidDetection>()
{
  return "skyvolt_msgs/msg/RfidDetection";
}

template<>
struct has_fixed_size<skyvolt_msgs::msg::RfidDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<skyvolt_msgs::msg::RfidDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<skyvolt_msgs::msg::RfidDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__TRAITS_HPP_
