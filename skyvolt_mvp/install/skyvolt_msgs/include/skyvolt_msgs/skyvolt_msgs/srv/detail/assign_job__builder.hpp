// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:srv/AssignJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__BUILDER_HPP_
#define SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/srv/detail/assign_job__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace srv
{

namespace builder
{

class Init_AssignJob_Request_job
{
public:
  Init_AssignJob_Request_job()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::skyvolt_msgs::srv::AssignJob_Request job(::skyvolt_msgs::srv::AssignJob_Request::_job_type arg)
  {
    msg_.job = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::srv::AssignJob_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::srv::AssignJob_Request>()
{
  return skyvolt_msgs::srv::builder::Init_AssignJob_Request_job();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace srv
{

namespace builder
{

class Init_AssignJob_Response_reason
{
public:
  explicit Init_AssignJob_Response_reason(::skyvolt_msgs::srv::AssignJob_Response & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::srv::AssignJob_Response reason(::skyvolt_msgs::srv::AssignJob_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::srv::AssignJob_Response msg_;
};

class Init_AssignJob_Response_assigned_robot_id
{
public:
  explicit Init_AssignJob_Response_assigned_robot_id(::skyvolt_msgs::srv::AssignJob_Response & msg)
  : msg_(msg)
  {}
  Init_AssignJob_Response_reason assigned_robot_id(::skyvolt_msgs::srv::AssignJob_Response::_assigned_robot_id_type arg)
  {
    msg_.assigned_robot_id = std::move(arg);
    return Init_AssignJob_Response_reason(msg_);
  }

private:
  ::skyvolt_msgs::srv::AssignJob_Response msg_;
};

class Init_AssignJob_Response_accepted
{
public:
  Init_AssignJob_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AssignJob_Response_assigned_robot_id accepted(::skyvolt_msgs::srv::AssignJob_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_AssignJob_Response_assigned_robot_id(msg_);
  }

private:
  ::skyvolt_msgs::srv::AssignJob_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::srv::AssignJob_Response>()
{
  return skyvolt_msgs::srv::builder::Init_AssignJob_Response_accepted();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__BUILDER_HPP_
