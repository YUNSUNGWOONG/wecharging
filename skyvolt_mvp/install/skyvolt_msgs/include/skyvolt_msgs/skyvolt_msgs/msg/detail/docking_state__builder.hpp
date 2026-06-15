// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:msg/DockingState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__BUILDER_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/msg/detail/docking_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace msg
{

namespace builder
{

class Init_DockingState_longitudinal_error_mm
{
public:
  explicit Init_DockingState_longitudinal_error_mm(::skyvolt_msgs::msg::DockingState & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::msg::DockingState longitudinal_error_mm(::skyvolt_msgs::msg::DockingState::_longitudinal_error_mm_type arg)
  {
    msg_.longitudinal_error_mm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::msg::DockingState msg_;
};

class Init_DockingState_lateral_error_mm
{
public:
  explicit Init_DockingState_lateral_error_mm(::skyvolt_msgs::msg::DockingState & msg)
  : msg_(msg)
  {}
  Init_DockingState_longitudinal_error_mm lateral_error_mm(::skyvolt_msgs::msg::DockingState::_lateral_error_mm_type arg)
  {
    msg_.lateral_error_mm = std::move(arg);
    return Init_DockingState_longitudinal_error_mm(msg_);
  }

private:
  ::skyvolt_msgs::msg::DockingState msg_;
};

class Init_DockingState_state
{
public:
  explicit Init_DockingState_state(::skyvolt_msgs::msg::DockingState & msg)
  : msg_(msg)
  {}
  Init_DockingState_lateral_error_mm state(::skyvolt_msgs::msg::DockingState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_DockingState_lateral_error_mm(msg_);
  }

private:
  ::skyvolt_msgs::msg::DockingState msg_;
};

class Init_DockingState_robot_id
{
public:
  explicit Init_DockingState_robot_id(::skyvolt_msgs::msg::DockingState & msg)
  : msg_(msg)
  {}
  Init_DockingState_state robot_id(::skyvolt_msgs::msg::DockingState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_DockingState_state(msg_);
  }

private:
  ::skyvolt_msgs::msg::DockingState msg_;
};

class Init_DockingState_stamp
{
public:
  Init_DockingState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DockingState_robot_id stamp(::skyvolt_msgs::msg::DockingState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_DockingState_robot_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::DockingState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::msg::DockingState>()
{
  return skyvolt_msgs::msg::builder::Init_DockingState_stamp();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__BUILDER_HPP_
