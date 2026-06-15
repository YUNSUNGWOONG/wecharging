// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from skyvolt_msgs:srv/AssignJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__TRAITS_HPP_
#define SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "skyvolt_msgs/srv/detail/assign_job__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'job'
#include "skyvolt_msgs/msg/detail/charging_job__traits.hpp"

namespace skyvolt_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AssignJob_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: job
  {
    out << "job: ";
    to_flow_style_yaml(msg.job, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AssignJob_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: job
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "job:\n";
    to_block_style_yaml(msg.job, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AssignJob_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::srv::AssignJob_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::srv::AssignJob_Request & msg)
{
  return skyvolt_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::srv::AssignJob_Request>()
{
  return "skyvolt_msgs::srv::AssignJob_Request";
}

template<>
inline const char * name<skyvolt_msgs::srv::AssignJob_Request>()
{
  return "skyvolt_msgs/srv/AssignJob_Request";
}

template<>
struct has_fixed_size<skyvolt_msgs::srv::AssignJob_Request>
  : std::integral_constant<bool, has_fixed_size<skyvolt_msgs::msg::ChargingJob>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::srv::AssignJob_Request>
  : std::integral_constant<bool, has_bounded_size<skyvolt_msgs::msg::ChargingJob>::value> {};

template<>
struct is_message<skyvolt_msgs::srv::AssignJob_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace skyvolt_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AssignJob_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: assigned_robot_id
  {
    out << "assigned_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.assigned_robot_id, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AssignJob_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: assigned_robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "assigned_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.assigned_robot_id, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AssignJob_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::srv::AssignJob_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::srv::AssignJob_Response & msg)
{
  return skyvolt_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::srv::AssignJob_Response>()
{
  return "skyvolt_msgs::srv::AssignJob_Response";
}

template<>
inline const char * name<skyvolt_msgs::srv::AssignJob_Response>()
{
  return "skyvolt_msgs/srv/AssignJob_Response";
}

template<>
struct has_fixed_size<skyvolt_msgs::srv::AssignJob_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<skyvolt_msgs::srv::AssignJob_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<skyvolt_msgs::srv::AssignJob_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<skyvolt_msgs::srv::AssignJob>()
{
  return "skyvolt_msgs::srv::AssignJob";
}

template<>
inline const char * name<skyvolt_msgs::srv::AssignJob>()
{
  return "skyvolt_msgs/srv/AssignJob";
}

template<>
struct has_fixed_size<skyvolt_msgs::srv::AssignJob>
  : std::integral_constant<
    bool,
    has_fixed_size<skyvolt_msgs::srv::AssignJob_Request>::value &&
    has_fixed_size<skyvolt_msgs::srv::AssignJob_Response>::value
  >
{
};

template<>
struct has_bounded_size<skyvolt_msgs::srv::AssignJob>
  : std::integral_constant<
    bool,
    has_bounded_size<skyvolt_msgs::srv::AssignJob_Request>::value &&
    has_bounded_size<skyvolt_msgs::srv::AssignJob_Response>::value
  >
{
};

template<>
struct is_service<skyvolt_msgs::srv::AssignJob>
  : std::true_type
{
};

template<>
struct is_service_request<skyvolt_msgs::srv::AssignJob_Request>
  : std::true_type
{
};

template<>
struct is_service_response<skyvolt_msgs::srv::AssignJob_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__TRAITS_HPP_
