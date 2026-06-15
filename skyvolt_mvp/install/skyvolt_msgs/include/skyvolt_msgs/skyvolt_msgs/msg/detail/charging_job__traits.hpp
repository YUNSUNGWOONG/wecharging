// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from skyvolt_msgs:msg/ChargingJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__TRAITS_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "skyvolt_msgs/msg/detail/charging_job__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'submitted_at'
// Member 'deadline'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace skyvolt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChargingJob & msg,
  std::ostream & out)
{
  out << "{";
  // member: job_id
  {
    out << "job_id: ";
    rosidl_generator_traits::value_to_yaml(msg.job_id, out);
    out << ", ";
  }

  // member: requester_id
  {
    out << "requester_id: ";
    rosidl_generator_traits::value_to_yaml(msg.requester_id, out);
    out << ", ";
  }

  // member: target_branch_id
  {
    out << "target_branch_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_branch_id, out);
    out << ", ";
  }

  // member: target_arclength_m
  {
    out << "target_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.target_arclength_m, out);
    out << ", ";
  }

  // member: submitted_at
  {
    out << "submitted_at: ";
    to_flow_style_yaml(msg.submitted_at, out);
    out << ", ";
  }

  // member: deadline
  {
    out << "deadline: ";
    to_flow_style_yaml(msg.deadline, out);
    out << ", ";
  }

  // member: priority
  {
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChargingJob & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: job_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "job_id: ";
    rosidl_generator_traits::value_to_yaml(msg.job_id, out);
    out << "\n";
  }

  // member: requester_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requester_id: ";
    rosidl_generator_traits::value_to_yaml(msg.requester_id, out);
    out << "\n";
  }

  // member: target_branch_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_branch_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_branch_id, out);
    out << "\n";
  }

  // member: target_arclength_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.target_arclength_m, out);
    out << "\n";
  }

  // member: submitted_at
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submitted_at:\n";
    to_block_style_yaml(msg.submitted_at, out, indentation + 2);
  }

  // member: deadline
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deadline:\n";
    to_block_style_yaml(msg.deadline, out, indentation + 2);
  }

  // member: priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChargingJob & msg, bool use_flow_style = false)
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
  const skyvolt_msgs::msg::ChargingJob & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::msg::ChargingJob & msg)
{
  return skyvolt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::msg::ChargingJob>()
{
  return "skyvolt_msgs::msg::ChargingJob";
}

template<>
inline const char * name<skyvolt_msgs::msg::ChargingJob>()
{
  return "skyvolt_msgs/msg/ChargingJob";
}

template<>
struct has_fixed_size<skyvolt_msgs::msg::ChargingJob>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<skyvolt_msgs::msg::ChargingJob>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<skyvolt_msgs::msg::ChargingJob>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__TRAITS_HPP_
