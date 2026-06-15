// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from skyvolt_msgs:action/TransportPile.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__TRAITS_HPP_
#define SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "skyvolt_msgs/action/detail/transport_pile__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: job_id
  {
    out << "job_id: ";
    rosidl_generator_traits::value_to_yaml(msg.job_id, out);
    out << ", ";
  }

  // member: source_branch_id
  {
    out << "source_branch_id: ";
    rosidl_generator_traits::value_to_yaml(msg.source_branch_id, out);
    out << ", ";
  }

  // member: source_arclength_m
  {
    out << "source_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.source_arclength_m, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_Goal & msg,
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

  // member: source_branch_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_branch_id: ";
    rosidl_generator_traits::value_to_yaml(msg.source_branch_id, out);
    out << "\n";
  }

  // member: source_arclength_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.source_arclength_m, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_Goal & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_Goal>()
{
  return "skyvolt_msgs::action::TransportPile_Goal";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_Goal>()
{
  return "skyvolt_msgs/action/TransportPile_Goal";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: failure_reason
  {
    out << "failure_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_reason, out);
    out << ", ";
  }

  // member: final_lateral_error_mm
  {
    out << "final_lateral_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.final_lateral_error_mm, out);
    out << ", ";
  }

  // member: final_longitudinal_error_mm
  {
    out << "final_longitudinal_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.final_longitudinal_error_mm, out);
    out << ", ";
  }

  // member: total_time_s
  {
    out << "total_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time_s, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: failure_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failure_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_reason, out);
    out << "\n";
  }

  // member: final_lateral_error_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_lateral_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.final_lateral_error_mm, out);
    out << "\n";
  }

  // member: final_longitudinal_error_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_longitudinal_error_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.final_longitudinal_error_mm, out);
    out << "\n";
  }

  // member: total_time_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time_s, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_Result & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_Result>()
{
  return "skyvolt_msgs::action::TransportPile_Result";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_Result>()
{
  return "skyvolt_msgs/action/TransportPile_Result";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: progress_arclength_m
  {
    out << "progress_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_arclength_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: progress_arclength_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_arclength_m: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_arclength_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_Feedback & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_Feedback>()
{
  return "skyvolt_msgs::action::TransportPile_Feedback";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_Feedback>()
{
  return "skyvolt_msgs/action/TransportPile_Feedback";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "skyvolt_msgs/action/detail/transport_pile__traits.hpp"

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_SendGoal_Request & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_SendGoal_Request>()
{
  return "skyvolt_msgs::action::TransportPile_SendGoal_Request";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_SendGoal_Request>()
{
  return "skyvolt_msgs/action/TransportPile_SendGoal_Request";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<skyvolt_msgs::action::TransportPile_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<skyvolt_msgs::action::TransportPile_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_SendGoal_Response & msg,
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

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_SendGoal_Response & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_SendGoal_Response>()
{
  return "skyvolt_msgs::action::TransportPile_SendGoal_Response";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_SendGoal_Response>()
{
  return "skyvolt_msgs/action/TransportPile_SendGoal_Response";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_SendGoal>()
{
  return "skyvolt_msgs::action::TransportPile_SendGoal";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_SendGoal>()
{
  return "skyvolt_msgs/action/TransportPile_SendGoal";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<skyvolt_msgs::action::TransportPile_SendGoal_Request>::value &&
    has_fixed_size<skyvolt_msgs::action::TransportPile_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<skyvolt_msgs::action::TransportPile_SendGoal_Request>::value &&
    has_bounded_size<skyvolt_msgs::action::TransportPile_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<skyvolt_msgs::action::TransportPile_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<skyvolt_msgs::action::TransportPile_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<skyvolt_msgs::action::TransportPile_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_GetResult_Request & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_GetResult_Request>()
{
  return "skyvolt_msgs::action::TransportPile_GetResult_Request";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_GetResult_Request>()
{
  return "skyvolt_msgs/action/TransportPile_GetResult_Request";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__traits.hpp"

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_GetResult_Response & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_GetResult_Response>()
{
  return "skyvolt_msgs::action::TransportPile_GetResult_Response";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_GetResult_Response>()
{
  return "skyvolt_msgs/action/TransportPile_GetResult_Response";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<skyvolt_msgs::action::TransportPile_Result>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<skyvolt_msgs::action::TransportPile_Result>::value> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_GetResult>()
{
  return "skyvolt_msgs::action::TransportPile_GetResult";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_GetResult>()
{
  return "skyvolt_msgs/action/TransportPile_GetResult";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<skyvolt_msgs::action::TransportPile_GetResult_Request>::value &&
    has_fixed_size<skyvolt_msgs::action::TransportPile_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<skyvolt_msgs::action::TransportPile_GetResult_Request>::value &&
    has_bounded_size<skyvolt_msgs::action::TransportPile_GetResult_Response>::value
  >
{
};

template<>
struct is_service<skyvolt_msgs::action::TransportPile_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<skyvolt_msgs::action::TransportPile_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<skyvolt_msgs::action::TransportPile_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "skyvolt_msgs/action/detail/transport_pile__traits.hpp"

namespace skyvolt_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const TransportPile_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TransportPile_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TransportPile_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace skyvolt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use skyvolt_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const skyvolt_msgs::action::TransportPile_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  skyvolt_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use skyvolt_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const skyvolt_msgs::action::TransportPile_FeedbackMessage & msg)
{
  return skyvolt_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<skyvolt_msgs::action::TransportPile_FeedbackMessage>()
{
  return "skyvolt_msgs::action::TransportPile_FeedbackMessage";
}

template<>
inline const char * name<skyvolt_msgs::action::TransportPile_FeedbackMessage>()
{
  return "skyvolt_msgs/action/TransportPile_FeedbackMessage";
}

template<>
struct has_fixed_size<skyvolt_msgs::action::TransportPile_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<skyvolt_msgs::action::TransportPile_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<skyvolt_msgs::action::TransportPile_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<skyvolt_msgs::action::TransportPile_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<skyvolt_msgs::action::TransportPile_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<skyvolt_msgs::action::TransportPile>
  : std::true_type
{
};

template<>
struct is_action_goal<skyvolt_msgs::action::TransportPile_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<skyvolt_msgs::action::TransportPile_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<skyvolt_msgs::action::TransportPile_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__TRAITS_HPP_
