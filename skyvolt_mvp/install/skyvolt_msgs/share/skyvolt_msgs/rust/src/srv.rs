#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to skyvolt_msgs__srv__AssignJob_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssignJob_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub job: super::msg::ChargingJob,

}



impl Default for AssignJob_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::AssignJob_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AssignJob_Request {
  type RmwMsg = super::srv::rmw::AssignJob_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        job: super::msg::ChargingJob::into_rmw_message(std::borrow::Cow::Owned(msg.job)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        job: super::msg::ChargingJob::into_rmw_message(std::borrow::Cow::Borrowed(&msg.job)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      job: super::msg::ChargingJob::from_rmw_message(msg.job),
    }
  }
}


// Corresponds to skyvolt_msgs__srv__AssignJob_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssignJob_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub assigned_robot_id: std::string::String,

    /// populated on rejection
    pub reason: std::string::String,

}



impl Default for AssignJob_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::AssignJob_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AssignJob_Response {
  type RmwMsg = super::srv::rmw::AssignJob_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        assigned_robot_id: msg.assigned_robot_id.as_str().into(),
        reason: msg.reason.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        assigned_robot_id: msg.assigned_robot_id.as_str().into(),
        reason: msg.reason.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      assigned_robot_id: msg.assigned_robot_id.to_string(),
      reason: msg.reason.to_string(),
    }
  }
}






#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__skyvolt_msgs__srv__AssignJob() -> *const std::ffi::c_void;
}

// Corresponds to skyvolt_msgs__srv__AssignJob
#[allow(missing_docs, non_camel_case_types)]
pub struct AssignJob;

impl rosidl_runtime_rs::Service for AssignJob {
    type Request = AssignJob_Request;
    type Response = AssignJob_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__skyvolt_msgs__srv__AssignJob() }
    }
}


