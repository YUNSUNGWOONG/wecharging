// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:msg/ChargingJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__BUILDER_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/msg/detail/charging_job__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace msg
{

namespace builder
{

class Init_ChargingJob_priority
{
public:
  explicit Init_ChargingJob_priority(::skyvolt_msgs::msg::ChargingJob & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::msg::ChargingJob priority(::skyvolt_msgs::msg::ChargingJob::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

class Init_ChargingJob_deadline
{
public:
  explicit Init_ChargingJob_deadline(::skyvolt_msgs::msg::ChargingJob & msg)
  : msg_(msg)
  {}
  Init_ChargingJob_priority deadline(::skyvolt_msgs::msg::ChargingJob::_deadline_type arg)
  {
    msg_.deadline = std::move(arg);
    return Init_ChargingJob_priority(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

class Init_ChargingJob_submitted_at
{
public:
  explicit Init_ChargingJob_submitted_at(::skyvolt_msgs::msg::ChargingJob & msg)
  : msg_(msg)
  {}
  Init_ChargingJob_deadline submitted_at(::skyvolt_msgs::msg::ChargingJob::_submitted_at_type arg)
  {
    msg_.submitted_at = std::move(arg);
    return Init_ChargingJob_deadline(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

class Init_ChargingJob_target_arclength_m
{
public:
  explicit Init_ChargingJob_target_arclength_m(::skyvolt_msgs::msg::ChargingJob & msg)
  : msg_(msg)
  {}
  Init_ChargingJob_submitted_at target_arclength_m(::skyvolt_msgs::msg::ChargingJob::_target_arclength_m_type arg)
  {
    msg_.target_arclength_m = std::move(arg);
    return Init_ChargingJob_submitted_at(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

class Init_ChargingJob_target_branch_id
{
public:
  explicit Init_ChargingJob_target_branch_id(::skyvolt_msgs::msg::ChargingJob & msg)
  : msg_(msg)
  {}
  Init_ChargingJob_target_arclength_m target_branch_id(::skyvolt_msgs::msg::ChargingJob::_target_branch_id_type arg)
  {
    msg_.target_branch_id = std::move(arg);
    return Init_ChargingJob_target_arclength_m(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

class Init_ChargingJob_requester_id
{
public:
  explicit Init_ChargingJob_requester_id(::skyvolt_msgs::msg::ChargingJob & msg)
  : msg_(msg)
  {}
  Init_ChargingJob_target_branch_id requester_id(::skyvolt_msgs::msg::ChargingJob::_requester_id_type arg)
  {
    msg_.requester_id = std::move(arg);
    return Init_ChargingJob_target_branch_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

class Init_ChargingJob_job_id
{
public:
  Init_ChargingJob_job_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChargingJob_requester_id job_id(::skyvolt_msgs::msg::ChargingJob::_job_id_type arg)
  {
    msg_.job_id = std::move(arg);
    return Init_ChargingJob_requester_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::ChargingJob msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::msg::ChargingJob>()
{
  return skyvolt_msgs::msg::builder::Init_ChargingJob_job_id();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__BUILDER_HPP_
