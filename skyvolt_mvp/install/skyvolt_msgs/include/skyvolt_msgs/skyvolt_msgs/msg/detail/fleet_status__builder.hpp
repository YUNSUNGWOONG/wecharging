// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:msg/FleetStatus.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__BUILDER_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/msg/detail/fleet_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace msg
{

namespace builder
{

class Init_FleetStatus_avg_job_age_s
{
public:
  explicit Init_FleetStatus_avg_job_age_s(::skyvolt_msgs::msg::FleetStatus & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::msg::FleetStatus avg_job_age_s(::skyvolt_msgs::msg::FleetStatus::_avg_job_age_s_type arg)
  {
    msg_.avg_job_age_s = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::msg::FleetStatus msg_;
};

class Init_FleetStatus_num_reservations
{
public:
  explicit Init_FleetStatus_num_reservations(::skyvolt_msgs::msg::FleetStatus & msg)
  : msg_(msg)
  {}
  Init_FleetStatus_avg_job_age_s num_reservations(::skyvolt_msgs::msg::FleetStatus::_num_reservations_type arg)
  {
    msg_.num_reservations = std::move(arg);
    return Init_FleetStatus_avg_job_age_s(msg_);
  }

private:
  ::skyvolt_msgs::msg::FleetStatus msg_;
};

class Init_FleetStatus_num_active_jobs
{
public:
  explicit Init_FleetStatus_num_active_jobs(::skyvolt_msgs::msg::FleetStatus & msg)
  : msg_(msg)
  {}
  Init_FleetStatus_num_reservations num_active_jobs(::skyvolt_msgs::msg::FleetStatus::_num_active_jobs_type arg)
  {
    msg_.num_active_jobs = std::move(arg);
    return Init_FleetStatus_num_reservations(msg_);
  }

private:
  ::skyvolt_msgs::msg::FleetStatus msg_;
};

class Init_FleetStatus_num_pending_jobs
{
public:
  explicit Init_FleetStatus_num_pending_jobs(::skyvolt_msgs::msg::FleetStatus & msg)
  : msg_(msg)
  {}
  Init_FleetStatus_num_active_jobs num_pending_jobs(::skyvolt_msgs::msg::FleetStatus::_num_pending_jobs_type arg)
  {
    msg_.num_pending_jobs = std::move(arg);
    return Init_FleetStatus_num_active_jobs(msg_);
  }

private:
  ::skyvolt_msgs::msg::FleetStatus msg_;
};

class Init_FleetStatus_num_robots
{
public:
  explicit Init_FleetStatus_num_robots(::skyvolt_msgs::msg::FleetStatus & msg)
  : msg_(msg)
  {}
  Init_FleetStatus_num_pending_jobs num_robots(::skyvolt_msgs::msg::FleetStatus::_num_robots_type arg)
  {
    msg_.num_robots = std::move(arg);
    return Init_FleetStatus_num_pending_jobs(msg_);
  }

private:
  ::skyvolt_msgs::msg::FleetStatus msg_;
};

class Init_FleetStatus_stamp
{
public:
  Init_FleetStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FleetStatus_num_robots stamp(::skyvolt_msgs::msg::FleetStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_FleetStatus_num_robots(msg_);
  }

private:
  ::skyvolt_msgs::msg::FleetStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::msg::FleetStatus>()
{
  return skyvolt_msgs::msg::builder::Init_FleetStatus_stamp();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__BUILDER_HPP_
