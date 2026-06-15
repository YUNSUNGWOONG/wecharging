// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:action/TransportPile.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__BUILDER_HPP_
#define SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/action/detail/transport_pile__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_Goal_target_arclength_m
{
public:
  explicit Init_TransportPile_Goal_target_arclength_m(::skyvolt_msgs::action::TransportPile_Goal & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_Goal target_arclength_m(::skyvolt_msgs::action::TransportPile_Goal::_target_arclength_m_type arg)
  {
    msg_.target_arclength_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Goal msg_;
};

class Init_TransportPile_Goal_target_branch_id
{
public:
  explicit Init_TransportPile_Goal_target_branch_id(::skyvolt_msgs::action::TransportPile_Goal & msg)
  : msg_(msg)
  {}
  Init_TransportPile_Goal_target_arclength_m target_branch_id(::skyvolt_msgs::action::TransportPile_Goal::_target_branch_id_type arg)
  {
    msg_.target_branch_id = std::move(arg);
    return Init_TransportPile_Goal_target_arclength_m(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Goal msg_;
};

class Init_TransportPile_Goal_source_arclength_m
{
public:
  explicit Init_TransportPile_Goal_source_arclength_m(::skyvolt_msgs::action::TransportPile_Goal & msg)
  : msg_(msg)
  {}
  Init_TransportPile_Goal_target_branch_id source_arclength_m(::skyvolt_msgs::action::TransportPile_Goal::_source_arclength_m_type arg)
  {
    msg_.source_arclength_m = std::move(arg);
    return Init_TransportPile_Goal_target_branch_id(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Goal msg_;
};

class Init_TransportPile_Goal_source_branch_id
{
public:
  explicit Init_TransportPile_Goal_source_branch_id(::skyvolt_msgs::action::TransportPile_Goal & msg)
  : msg_(msg)
  {}
  Init_TransportPile_Goal_source_arclength_m source_branch_id(::skyvolt_msgs::action::TransportPile_Goal::_source_branch_id_type arg)
  {
    msg_.source_branch_id = std::move(arg);
    return Init_TransportPile_Goal_source_arclength_m(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Goal msg_;
};

class Init_TransportPile_Goal_job_id
{
public:
  Init_TransportPile_Goal_job_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_Goal_source_branch_id job_id(::skyvolt_msgs::action::TransportPile_Goal::_job_id_type arg)
  {
    msg_.job_id = std::move(arg);
    return Init_TransportPile_Goal_source_branch_id(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_Goal>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_Goal_job_id();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_Result_total_time_s
{
public:
  explicit Init_TransportPile_Result_total_time_s(::skyvolt_msgs::action::TransportPile_Result & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_Result total_time_s(::skyvolt_msgs::action::TransportPile_Result::_total_time_s_type arg)
  {
    msg_.total_time_s = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Result msg_;
};

class Init_TransportPile_Result_final_longitudinal_error_mm
{
public:
  explicit Init_TransportPile_Result_final_longitudinal_error_mm(::skyvolt_msgs::action::TransportPile_Result & msg)
  : msg_(msg)
  {}
  Init_TransportPile_Result_total_time_s final_longitudinal_error_mm(::skyvolt_msgs::action::TransportPile_Result::_final_longitudinal_error_mm_type arg)
  {
    msg_.final_longitudinal_error_mm = std::move(arg);
    return Init_TransportPile_Result_total_time_s(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Result msg_;
};

class Init_TransportPile_Result_final_lateral_error_mm
{
public:
  explicit Init_TransportPile_Result_final_lateral_error_mm(::skyvolt_msgs::action::TransportPile_Result & msg)
  : msg_(msg)
  {}
  Init_TransportPile_Result_final_longitudinal_error_mm final_lateral_error_mm(::skyvolt_msgs::action::TransportPile_Result::_final_lateral_error_mm_type arg)
  {
    msg_.final_lateral_error_mm = std::move(arg);
    return Init_TransportPile_Result_final_longitudinal_error_mm(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Result msg_;
};

class Init_TransportPile_Result_failure_reason
{
public:
  explicit Init_TransportPile_Result_failure_reason(::skyvolt_msgs::action::TransportPile_Result & msg)
  : msg_(msg)
  {}
  Init_TransportPile_Result_final_lateral_error_mm failure_reason(::skyvolt_msgs::action::TransportPile_Result::_failure_reason_type arg)
  {
    msg_.failure_reason = std::move(arg);
    return Init_TransportPile_Result_final_lateral_error_mm(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Result msg_;
};

class Init_TransportPile_Result_success
{
public:
  Init_TransportPile_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_Result_failure_reason success(::skyvolt_msgs::action::TransportPile_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_TransportPile_Result_failure_reason(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_Result>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_Result_success();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_Feedback_progress_arclength_m
{
public:
  explicit Init_TransportPile_Feedback_progress_arclength_m(::skyvolt_msgs::action::TransportPile_Feedback & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_Feedback progress_arclength_m(::skyvolt_msgs::action::TransportPile_Feedback::_progress_arclength_m_type arg)
  {
    msg_.progress_arclength_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Feedback msg_;
};

class Init_TransportPile_Feedback_phase
{
public:
  Init_TransportPile_Feedback_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_Feedback_progress_arclength_m phase(::skyvolt_msgs::action::TransportPile_Feedback::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_TransportPile_Feedback_progress_arclength_m(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_Feedback>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_Feedback_phase();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_SendGoal_Request_goal
{
public:
  explicit Init_TransportPile_SendGoal_Request_goal(::skyvolt_msgs::action::TransportPile_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_SendGoal_Request goal(::skyvolt_msgs::action::TransportPile_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_SendGoal_Request msg_;
};

class Init_TransportPile_SendGoal_Request_goal_id
{
public:
  Init_TransportPile_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_SendGoal_Request_goal goal_id(::skyvolt_msgs::action::TransportPile_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TransportPile_SendGoal_Request_goal(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_SendGoal_Request>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_SendGoal_Request_goal_id();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_SendGoal_Response_stamp
{
public:
  explicit Init_TransportPile_SendGoal_Response_stamp(::skyvolt_msgs::action::TransportPile_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_SendGoal_Response stamp(::skyvolt_msgs::action::TransportPile_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_SendGoal_Response msg_;
};

class Init_TransportPile_SendGoal_Response_accepted
{
public:
  Init_TransportPile_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_SendGoal_Response_stamp accepted(::skyvolt_msgs::action::TransportPile_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_TransportPile_SendGoal_Response_stamp(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_SendGoal_Response>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_SendGoal_Response_accepted();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_GetResult_Request_goal_id
{
public:
  Init_TransportPile_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::skyvolt_msgs::action::TransportPile_GetResult_Request goal_id(::skyvolt_msgs::action::TransportPile_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_GetResult_Request>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_GetResult_Request_goal_id();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_GetResult_Response_result
{
public:
  explicit Init_TransportPile_GetResult_Response_result(::skyvolt_msgs::action::TransportPile_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_GetResult_Response result(::skyvolt_msgs::action::TransportPile_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_GetResult_Response msg_;
};

class Init_TransportPile_GetResult_Response_status
{
public:
  Init_TransportPile_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_GetResult_Response_result status(::skyvolt_msgs::action::TransportPile_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TransportPile_GetResult_Response_result(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_GetResult_Response>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_GetResult_Response_status();
}

}  // namespace skyvolt_msgs


namespace skyvolt_msgs
{

namespace action
{

namespace builder
{

class Init_TransportPile_FeedbackMessage_feedback
{
public:
  explicit Init_TransportPile_FeedbackMessage_feedback(::skyvolt_msgs::action::TransportPile_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::action::TransportPile_FeedbackMessage feedback(::skyvolt_msgs::action::TransportPile_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_FeedbackMessage msg_;
};

class Init_TransportPile_FeedbackMessage_goal_id
{
public:
  Init_TransportPile_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TransportPile_FeedbackMessage_feedback goal_id(::skyvolt_msgs::action::TransportPile_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TransportPile_FeedbackMessage_feedback(msg_);
  }

private:
  ::skyvolt_msgs::action::TransportPile_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::action::TransportPile_FeedbackMessage>()
{
  return skyvolt_msgs::action::builder::Init_TransportPile_FeedbackMessage_goal_id();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__ACTION__DETAIL__TRANSPORT_PILE__BUILDER_HPP_
