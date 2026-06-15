#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__srv__AssignJob_Request() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__srv__AssignJob_Request__init(msg: *mut AssignJob_Request) -> bool;
    fn skyvolt_msgs__srv__AssignJob_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssignJob_Request>, size: usize) -> bool;
    fn skyvolt_msgs__srv__AssignJob_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssignJob_Request>);
    fn skyvolt_msgs__srv__AssignJob_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssignJob_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AssignJob_Request>) -> bool;
}

// Corresponds to skyvolt_msgs__srv__AssignJob_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssignJob_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub job: super::super::msg::rmw::ChargingJob,

}



impl Default for AssignJob_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__srv__AssignJob_Request__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__srv__AssignJob_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssignJob_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__srv__AssignJob_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__srv__AssignJob_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__srv__AssignJob_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssignJob_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssignJob_Request where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/srv/AssignJob_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__srv__AssignJob_Request() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__srv__AssignJob_Response() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__srv__AssignJob_Response__init(msg: *mut AssignJob_Response) -> bool;
    fn skyvolt_msgs__srv__AssignJob_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssignJob_Response>, size: usize) -> bool;
    fn skyvolt_msgs__srv__AssignJob_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssignJob_Response>);
    fn skyvolt_msgs__srv__AssignJob_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssignJob_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AssignJob_Response>) -> bool;
}

// Corresponds to skyvolt_msgs__srv__AssignJob_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssignJob_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub assigned_robot_id: rosidl_runtime_rs::String,

    /// populated on rejection
    pub reason: rosidl_runtime_rs::String,

}



impl Default for AssignJob_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__srv__AssignJob_Response__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__srv__AssignJob_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssignJob_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__srv__AssignJob_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__srv__AssignJob_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__srv__AssignJob_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssignJob_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssignJob_Response where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/srv/AssignJob_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__srv__AssignJob_Response() }
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


