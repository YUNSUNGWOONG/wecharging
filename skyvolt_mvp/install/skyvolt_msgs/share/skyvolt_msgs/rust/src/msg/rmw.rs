#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__RfidDetection() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__msg__RfidDetection__init(msg: *mut RfidDetection) -> bool;
    fn skyvolt_msgs__msg__RfidDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RfidDetection>, size: usize) -> bool;
    fn skyvolt_msgs__msg__RfidDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RfidDetection>);
    fn skyvolt_msgs__msg__RfidDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RfidDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<RfidDetection>) -> bool;
}

// Corresponds to skyvolt_msgs__msg__RfidDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RFID tag detection on the track.
///
/// Tag kinds (paper section 3):
///   1 = CUE         -> first deceleration trigger
///   2 = POSITIONING -> second deceleration trigger
///   3 = DOCKING     -> served by photoeye, included for completeness

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RfidDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kind: u8,

    /// globally unique tag identifier
    pub tag_id: rosidl_runtime_rs::String,

    /// pre-mapped tag arclength along the track
    pub expected_arclength_m: f64,

    /// for future range estimation
    pub rssi: f64,

}

impl RfidDetection {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const KIND_CUE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const KIND_POSITIONING: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const KIND_DOCKING: u8 = 3;

}


impl Default for RfidDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__msg__RfidDetection__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__msg__RfidDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RfidDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__RfidDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__RfidDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__RfidDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RfidDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RfidDetection where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/msg/RfidDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__RfidDetection() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__PhotoeyeDetection() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__msg__PhotoeyeDetection__init(msg: *mut PhotoeyeDetection) -> bool;
    fn skyvolt_msgs__msg__PhotoeyeDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PhotoeyeDetection>, size: usize) -> bool;
    fn skyvolt_msgs__msg__PhotoeyeDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PhotoeyeDetection>);
    fn skyvolt_msgs__msg__PhotoeyeDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PhotoeyeDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<PhotoeyeDetection>) -> bool;
}

// Corresponds to skyvolt_msgs__msg__PhotoeyeDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Photoelectric sensor binary state at the docking card.
/// When triggered, the robot must stop immediately (paper Fig.3d).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PhotoeyeDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub triggered: bool,

    /// if known
    pub docking_card_id: rosidl_runtime_rs::String,

}



impl Default for PhotoeyeDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__msg__PhotoeyeDetection__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__msg__PhotoeyeDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PhotoeyeDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__PhotoeyeDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__PhotoeyeDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__PhotoeyeDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PhotoeyeDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PhotoeyeDetection where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/msg/PhotoeyeDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__PhotoeyeDetection() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__TrackState() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__msg__TrackState__init(msg: *mut TrackState) -> bool;
    fn skyvolt_msgs__msg__TrackState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrackState>, size: usize) -> bool;
    fn skyvolt_msgs__msg__TrackState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrackState>);
    fn skyvolt_msgs__msg__TrackState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrackState>, out_seq: *mut rosidl_runtime_rs::Sequence<TrackState>) -> bool;
}

// Corresponds to skyvolt_msgs__msg__TrackState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Localizer output: pose along the track.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,

    /// 0 = main loop; reserved for switches
    pub branch_id: u32,

    /// position along branch centerline
    pub arclength_m: f64,

    /// signed; positive = forward
    pub speed_mps: f64,

    /// KF variance estimate (m^2)
    pub arclength_var: f64,

}



impl Default for TrackState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__msg__TrackState__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__msg__TrackState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrackState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__TrackState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__TrackState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__TrackState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrackState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrackState where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/msg/TrackState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__TrackState() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__DockingState() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__msg__DockingState__init(msg: *mut DockingState) -> bool;
    fn skyvolt_msgs__msg__DockingState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DockingState>, size: usize) -> bool;
    fn skyvolt_msgs__msg__DockingState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DockingState>);
    fn skyvolt_msgs__msg__DockingState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DockingState>, out_seq: *mut rosidl_runtime_rs::Sequence<DockingState>) -> bool;
}

// Corresponds to skyvolt_msgs__msg__DockingState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Docking FSM state, exposed for telemetry/dashboards.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockingState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: u8,

    /// last measured (-1 if unknown)
    pub lateral_error_mm: f64,

    /// last measured (-1 if unknown)
    pub longitudinal_error_mm: f64,

}

impl DockingState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATE_IDLE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATE_CRUISE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATE_APPROACH_1: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATE_APPROACH_2: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATE_DOCKED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATE_FAULT: u8 = 255;

}


impl Default for DockingState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__msg__DockingState__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__msg__DockingState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DockingState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__DockingState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__DockingState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__DockingState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DockingState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DockingState where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/msg/DockingState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__DockingState() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__ChargingJob() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__msg__ChargingJob__init(msg: *mut ChargingJob) -> bool;
    fn skyvolt_msgs__msg__ChargingJob__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ChargingJob>, size: usize) -> bool;
    fn skyvolt_msgs__msg__ChargingJob__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ChargingJob>);
    fn skyvolt_msgs__msg__ChargingJob__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ChargingJob>, out_seq: *mut rosidl_runtime_rs::Sequence<ChargingJob>) -> bool;
}

// Corresponds to skyvolt_msgs__msg__ChargingJob
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// A request to deliver a charging pile to a target support point.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChargingJob {

    // This member is not documented.
    #[allow(missing_docs)]
    pub job_id: rosidl_runtime_rs::String,

    /// vehicle / parking-spot identifier
    pub requester_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_branch_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_arclength_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub submitted_at: builtin_interfaces::msg::rmw::Time,

    /// 0 = no deadline
    pub deadline: builtin_interfaces::msg::rmw::Time,

    /// 0=low, 255=urgent
    pub priority: u8,

}



impl Default for ChargingJob {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__msg__ChargingJob__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__msg__ChargingJob__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ChargingJob {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__ChargingJob__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__ChargingJob__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__ChargingJob__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ChargingJob {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ChargingJob where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/msg/ChargingJob";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__ChargingJob() }
  }
}


#[link(name = "skyvolt_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__FleetStatus() -> *const std::ffi::c_void;
}

#[link(name = "skyvolt_msgs__rosidl_generator_c")]
extern "C" {
    fn skyvolt_msgs__msg__FleetStatus__init(msg: *mut FleetStatus) -> bool;
    fn skyvolt_msgs__msg__FleetStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FleetStatus>, size: usize) -> bool;
    fn skyvolt_msgs__msg__FleetStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FleetStatus>);
    fn skyvolt_msgs__msg__FleetStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FleetStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<FleetStatus>) -> bool;
}

// Corresponds to skyvolt_msgs__msg__FleetStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Aggregate fleet status snapshot.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FleetStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub num_robots: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub num_pending_jobs: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub num_active_jobs: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub num_reservations: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub avg_job_age_s: f64,

}



impl Default for FleetStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !skyvolt_msgs__msg__FleetStatus__init(&mut msg as *mut _) {
        panic!("Call to skyvolt_msgs__msg__FleetStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FleetStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__FleetStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__FleetStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { skyvolt_msgs__msg__FleetStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FleetStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FleetStatus where Self: Sized {
  const TYPE_NAME: &'static str = "skyvolt_msgs/msg/FleetStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__skyvolt_msgs__msg__FleetStatus() }
  }
}


