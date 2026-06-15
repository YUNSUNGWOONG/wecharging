// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from skyvolt_msgs:msg/FleetStatus.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__TRAITS_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "skyvolt_msgs/msg/detail/fleet_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace skyvolt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FleetStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: num_robots
  {
    out << "num_robots: ";
    rosidl_generator_traits::value_to_yaml(msg.num_robots, out);
    out << ", ";
  }

  // member: num_pending_jobs
  {
    out << "num_pending_jobs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_pending_jobs, out);
    out << ", ";
  }

  // member: num_active_jobs
  {
    out << "num_active_jobs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_active_jobs, out);
    out << ", ";
  }

  // member: num_reservations
  {
    out << "num_reservations: ";
    rosidl_generator_traits::value_to_yaml(msg.num_reservations, out);
    out << ", ";
  }

  // member: avg_job_age_s
  {
    out << "avg_job_age_s: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_job_age_s, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FleetStatus & msg,
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

  // member: num_robots
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_robots: ";
    rosidl_generator_traits::value_to_yaml(msg.num_robots, out);
    out << "\n";
  }

  // member: num_pending_jobs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_pending_jobs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_pending_jobs, out);
    out << "\n";
  }

  // member: num_active_jobs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_active_jobs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_active_jobs, out);
    out << "\n";
  }

  // member: num_reservations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_reservations: ";
    rosidl_generator_traits::value_to_yaml(msg.num_reservations, out);
    out << "\n";
  }

  // member: avg_job_age_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_job_age_s: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_job_age_s, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FleetStatus & msg, bool use_flow_style = false)
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
  const skyvolt_msgs::msg::FleetStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::msg::FleetStatus & msg)
{
  return skyvolt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::msg::FleetStatus>()
{
  return "skyvolt_msgs::msg::FleetStatus";
}

template<>
inline const char * name<skyvolt_msgs::msg::FleetStatus>()
{
  return "skyvolt_msgs/msg/FleetStatus";
}

template<>
struct has_fixed_size<skyvolt_msgs::msg::FleetStatus>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::msg::FleetStatus>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<skyvolt_msgs::msg::FleetStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__TRAITS_HPP_
