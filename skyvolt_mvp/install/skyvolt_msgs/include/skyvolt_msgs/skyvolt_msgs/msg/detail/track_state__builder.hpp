// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:msg/TrackState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__BUILDER_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/msg/detail/track_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace msg
{

namespace builder
{

class Init_TrackState_arclength_var
{
public:
  explicit Init_TrackState_arclength_var(::skyvolt_msgs::msg::TrackState & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::msg::TrackState arclength_var(::skyvolt_msgs::msg::TrackState::_arclength_var_type arg)
  {
    msg_.arclength_var = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::msg::TrackState msg_;
};

class Init_TrackState_speed_mps
{
public:
  explicit Init_TrackState_speed_mps(::skyvolt_msgs::msg::TrackState & msg)
  : msg_(msg)
  {}
  Init_TrackState_arclength_var speed_mps(::skyvolt_msgs::msg::TrackState::_speed_mps_type arg)
  {
    msg_.speed_mps = std::move(arg);
    return Init_TrackState_arclength_var(msg_);
  }

private:
  ::skyvolt_msgs::msg::TrackState msg_;
};

class Init_TrackState_arclength_m
{
public:
  explicit Init_TrackState_arclength_m(::skyvolt_msgs::msg::TrackState & msg)
  : msg_(msg)
  {}
  Init_TrackState_speed_mps arclength_m(::skyvolt_msgs::msg::TrackState::_arclength_m_type arg)
  {
    msg_.arclength_m = std::move(arg);
    return Init_TrackState_speed_mps(msg_);
  }

private:
  ::skyvolt_msgs::msg::TrackState msg_;
};

class Init_TrackState_branch_id
{
public:
  explicit Init_TrackState_branch_id(::skyvolt_msgs::msg::TrackState & msg)
  : msg_(msg)
  {}
  Init_TrackState_arclength_m branch_id(::skyvolt_msgs::msg::TrackState::_branch_id_type arg)
  {
    msg_.branch_id = std::move(arg);
    return Init_TrackState_arclength_m(msg_);
  }

private:
  ::skyvolt_msgs::msg::TrackState msg_;
};

class Init_TrackState_robot_id
{
public:
  explicit Init_TrackState_robot_id(::skyvolt_msgs::msg::TrackState & msg)
  : msg_(msg)
  {}
  Init_TrackState_branch_id robot_id(::skyvolt_msgs::msg::TrackState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_TrackState_branch_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::TrackState msg_;
};

class Init_TrackState_stamp
{
public:
  Init_TrackState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackState_robot_id stamp(::skyvolt_msgs::msg::TrackState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_TrackState_robot_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::TrackState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::msg::TrackState>()
{
  return skyvolt_msgs::msg::builder::Init_TrackState_stamp();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__BUILDER_HPP_
