
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to skyvolt_msgs__action__TransportPile_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub job_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_branch_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_arclength_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_branch_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_arclength_m: f64,

}



impl Default for TransportPile_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_Goal {
  type RmwMsg = super::action::rmw::TransportPile_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        job_id: msg.job_id.as_str().into(),
        source_branch_id: msg.source_branch_id,
        source_arclength_m: msg.source_arclength_m,
        target_branch_id: msg.target_branch_id,
        target_arclength_m: msg.target_arclength_m,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        job_id: msg.job_id.as_str().into(),
      source_branch_id: msg.source_branch_id,
      source_arclength_m: msg.source_arclength_m,
      target_branch_id: msg.target_branch_id,
      target_arclength_m: msg.target_arclength_m,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      job_id: msg.job_id.to_string(),
      source_branch_id: msg.source_branch_id,
      source_arclength_m: msg.source_arclength_m,
      target_branch_id: msg.target_branch_id,
      target_arclength_m: msg.target_arclength_m,
    }
  }
}


// Corresponds to skyvolt_msgs__action__TransportPile_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub failure_reason: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_lateral_error_mm: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_longitudinal_error_mm: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub total_time_s: f64,

}



impl Default for TransportPile_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_Result::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_Result {
  type RmwMsg = super::action::rmw::TransportPile_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        failure_reason: msg.failure_reason.as_str().into(),
        final_lateral_error_mm: msg.final_lateral_error_mm,
        final_longitudinal_error_mm: msg.final_longitudinal_error_mm,
        total_time_s: msg.total_time_s,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        failure_reason: msg.failure_reason.as_str().into(),
      final_lateral_error_mm: msg.final_lateral_error_mm,
      final_longitudinal_error_mm: msg.final_longitudinal_error_mm,
      total_time_s: msg.total_time_s,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      failure_reason: msg.failure_reason.to_string(),
      final_lateral_error_mm: msg.final_lateral_error_mm,
      final_longitudinal_error_mm: msg.final_longitudinal_error_mm,
      total_time_s: msg.total_time_s,
    }
  }
}


// Corresponds to skyvolt_msgs__action__TransportPile_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_Feedback {
    /// mirrors DockingState.STATE_*
    pub phase: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub progress_arclength_m: f64,

}



impl Default for TransportPile_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_Feedback {
  type RmwMsg = super::action::rmw::TransportPile_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        phase: msg.phase,
        progress_arclength_m: msg.progress_arclength_m,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      phase: msg.phase,
      progress_arclength_m: msg.progress_arclength_m,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      phase: msg.phase,
      progress_arclength_m: msg.progress_arclength_m,
    }
  }
}


// Corresponds to skyvolt_msgs__action__TransportPile_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::TransportPile_Feedback,

}



impl Default for TransportPile_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_FeedbackMessage {
  type RmwMsg = super::action::rmw::TransportPile_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::TransportPile_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::TransportPile_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::TransportPile_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to skyvolt_msgs__action__TransportPile_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::TransportPile_Goal,

}



impl Default for TransportPile_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_SendGoal_Request {
  type RmwMsg = super::action::rmw::TransportPile_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::TransportPile_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::TransportPile_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::TransportPile_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to skyvolt_msgs__action__TransportPile_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for TransportPile_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_SendGoal_Response {
  type RmwMsg = super::action::rmw::TransportPile_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to skyvolt_msgs__action__TransportPile_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for TransportPile_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_GetResult_Request {
  type RmwMsg = super::action::rmw::TransportPile_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to skyvolt_msgs__action__TransportPile_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::TransportPile_Result,

}



impl Default for TransportPile_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::TransportPile_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for TransportPile_GetResult_Response {
  type RmwMsg = super::action::rmw::TransportPile_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::TransportPile_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::TransportPile_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::TransportPile_Result::from_rmw_message(msg.result),
    }
  }
}






#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__skyvolt_msgs__action__TransportPile_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to skyvolt_msgs__action__TransportPile_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct TransportPile_SendGoal;

impl rosidl_runtime_rs::Service for TransportPile_SendGoal {
    type Request = TransportPile_SendGoal_Request;
    type Response = TransportPile_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__skyvolt_msgs__action__TransportPile_SendGoal() }
    }
}




#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__skyvolt_msgs__action__TransportPile_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to skyvolt_msgs__action__TransportPile_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct TransportPile_GetResult;

impl rosidl_runtime_rs::Service for TransportPile_GetResult {
    type Request = TransportPile_GetResult_Request;
    type Response = TransportPile_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__skyvolt_msgs__action__TransportPile_GetResult() }
    }
}






#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__skyvolt_msgs__action__TransportPile() -> *const std::ffi::c_void;
}

// Corresponds to skyvolt_msgs__action__TransportPile
#[allow(missing_docs, non_camel_case_types)]
pub struct TransportPile;

impl rosidl_runtime_rs::Action for TransportPile {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = TransportPile_Goal;

  /// The result message defined in the action definition.
  type Result = TransportPile_Result;

  /// The feedback message defined in the action definition.
  type Feedback = TransportPile_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::TransportPile_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::TransportPile_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::TransportPile_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__skyvolt_msgs__action__TransportPile() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::TransportPile_Goal,
  ) -> super::action::rmw::TransportPile_SendGoal_Request {
   super::action::rmw::TransportPile_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::TransportPile_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::TransportPile_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::TransportPile_SendGoal_Response {
   super::action::rmw::TransportPile_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::TransportPile_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::TransportPile_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::TransportPile_Feedback,
  ) -> super::action::rmw::TransportPile_FeedbackMessage {
    let mut message = super::action::rmw::TransportPile_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::TransportPile_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::TransportPile_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::TransportPile_GetResult_Request {
   super::action::rmw::TransportPile_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::TransportPile_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::TransportPile_Result,
  ) -> super::action::rmw::TransportPile_GetResult_Response {
   super::action::rmw::TransportPile_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::TransportPile_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::TransportPile_Result,
  ) {
    (response.status, response.result)
  }
}


