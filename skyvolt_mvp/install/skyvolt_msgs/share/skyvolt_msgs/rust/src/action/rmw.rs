
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_Goal() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_Goal__init(msg: *mut TransportPile_Goal) -> bool;
    fn skyvolt_msgs__action__TransportPile_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Goal>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Goal>);
    fn skyvolt_msgs__action__TransportPile_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Goal>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub job_id: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_Goal__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_Goal() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_Result() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_Result__init(msg: *mut TransportPile_Result) -> bool;
    fn skyvolt_msgs__action__TransportPile_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Result>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Result>);
    fn skyvolt_msgs__action__TransportPile_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Result>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub failure_reason: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_Result__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_Result where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_Result() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_Feedback__init(msg: *mut TransportPile_Feedback) -> bool;
    fn skyvolt_msgs__action__TransportPile_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Feedback>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Feedback>);
    fn skyvolt_msgs__action__TransportPile_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_Feedback>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_Feedback__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_Feedback() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_FeedbackMessage__init(msg: *mut TransportPile_FeedbackMessage) -> bool;
    fn skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_FeedbackMessage>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_FeedbackMessage>);
    fn skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_FeedbackMessage>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::TransportPile_Feedback,

}



impl Default for TransportPile_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_FeedbackMessage() }
  }
}




#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_SendGoal_Request__init(msg: *mut TransportPile_SendGoal_Request) -> bool;
    fn skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Request>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Request>);
    fn skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Request>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::TransportPile_Goal,

}



impl Default for TransportPile_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_SendGoal_Request() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_SendGoal_Response__init(msg: *mut TransportPile_SendGoal_Response) -> bool;
    fn skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Response>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Response>);
    fn skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_SendGoal_Response>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for TransportPile_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_SendGoal_Response() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_GetResult_Request__init(msg: *mut TransportPile_GetResult_Request) -> bool;
    fn skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_GetResult_Request>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_GetResult_Request>);
    fn skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_GetResult_Request>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for TransportPile_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_GetResult_Request() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__action__TransportPile_GetResult_Response__init(msg: *mut TransportPile_GetResult_Response) -> bool;
    fn skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_GetResult_Response>, size: usize) -> bool;
    fn skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TransportPile_GetResult_Response>);
    fn skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TransportPile_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<TransportPile_GetResult_Response>) -> bool;
}

// Corresponds to skyvolt_msgs__action__TransportPile_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransportPile_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::TransportPile_Result,

}



impl Default for TransportPile_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__action__TransportPile_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__action__TransportPile_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TransportPile_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__action__TransportPile_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TransportPile_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TransportPile_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/action/TransportPile_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__action__TransportPile_GetResult_Response() }
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


