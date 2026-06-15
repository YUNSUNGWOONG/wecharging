// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from skyvolt_msgs:msg/DockingState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__TRAITS_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "skyvolt_msgs/msg/detail/docking_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace skyvolt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DockingState & msg,
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

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: lateral_error_mm
  {
    out << "lateral_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.lateral_error_mm, out);
    out << ", ";
  }

  // member: longitudinal_error_mm
  {
    out << "longitudinal_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.longitudinal_error_mm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DockingState & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: lateral_error_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lateral_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.lateral_error_mm, out);
    out << "\n";
  }

  // member: longitudinal_error_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitudinal_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.longitudinal_error_mm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DockingState & msg, bool use_flow_style = false)
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
  const skyvolt_msgs::msg::DockingState & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::msg::DockingState & msg)
{
  return skyvolt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::msg::DockingState>()
{
  return "skyvolt_msgs::msg::DockingState";
}

template<>
inline const char * name<skyvolt_msgs::msg::DockingState>()
{
  return "skyvolt_msgs/msg/DockingState";
}

template<>
struct has_fixed_size<skyvolt_msgs::msg::DockingState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<skyvolt_msgs::msg::DockingState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<skyvolt_msgs::msg::DockingState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__TRAITS_HPP_
