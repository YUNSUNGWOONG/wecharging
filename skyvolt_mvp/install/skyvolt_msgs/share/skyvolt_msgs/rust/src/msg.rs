#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to skyvolt_msgs__msg__RfidDetection
/// RFID tag detection on the track.
///
/// Tag kinds (paper section 3):
///   1 = CUE         -> first deceleration trigger
///   2 = POSITIONING -> second deceleration trigger
///   3 = DOCKING     -> served by photoeye, included for completeness

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RfidDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kind: u8,

    /// globally unique tag identifier
    pub tag_id: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RfidDetection::default())
  }
}

impl rosidl_runtime_rs::Message for RfidDetection {
  type RmwMsg = super::msg::rmw::RfidDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        kind: msg.kind,
        tag_id: msg.tag_id.as_str().into(),
        expected_arclength_m: msg.expected_arclength_m,
        rssi: msg.rssi,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
      kind: msg.kind,
        tag_id: msg.tag_id.as_str().into(),
      expected_arclength_m: msg.expected_arclength_m,
      rssi: msg.rssi,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      robot_id: msg.robot_id.to_string(),
      kind: msg.kind,
      tag_id: msg.tag_id.to_string(),
      expected_arclength_m: msg.expected_arclength_m,
      rssi: msg.rssi,
    }
  }
}


// Corresponds to skyvolt_msgs__msg__PhotoeyeDetection
/// Photoelectric sensor binary state at the docking card.
/// When triggered, the robot must stop immediately (paper Fig.3d).

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PhotoeyeDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub triggered: bool,

    /// if known
    pub docking_card_id: std::string::String,

}



impl Default for PhotoeyeDetection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PhotoeyeDetection::default())
  }
}

impl rosidl_runtime_rs::Message for PhotoeyeDetection {
  type RmwMsg = super::msg::rmw::PhotoeyeDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        triggered: msg.triggered,
        docking_card_id: msg.docking_card_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
      triggered: msg.triggered,
        docking_card_id: msg.docking_card_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      robot_id: msg.robot_id.to_string(),
      triggered: msg.triggered,
      docking_card_id: msg.docking_card_id.to_string(),
    }
  }
}


// Corresponds to skyvolt_msgs__msg__TrackState
/// Localizer output: pose along the track.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrackState::default())
  }
}

impl rosidl_runtime_rs::Message for TrackState {
  type RmwMsg = super::msg::rmw::TrackState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        branch_id: msg.branch_id,
        arclength_m: msg.arclength_m,
        speed_mps: msg.speed_mps,
        arclength_var: msg.arclength_var,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
      branch_id: msg.branch_id,
      arclength_m: msg.arclength_m,
      speed_mps: msg.speed_mps,
      arclength_var: msg.arclength_var,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      robot_id: msg.robot_id.to_string(),
      branch_id: msg.branch_id,
      arclength_m: msg.arclength_m,
      speed_mps: msg.speed_mps,
      arclength_var: msg.arclength_var,
    }
  }
}


// Corresponds to skyvolt_msgs__msg__DockingState
/// Docking FSM state, exposed for telemetry/dashboards.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DockingState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DockingState::default())
  }
}

impl rosidl_runtime_rs::Message for DockingState {
  type RmwMsg = super::msg::rmw::DockingState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        state: msg.state,
        lateral_error_mm: msg.lateral_error_mm,
        longitudinal_error_mm: msg.longitudinal_error_mm,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
      state: msg.state,
      lateral_error_mm: msg.lateral_error_mm,
      longitudinal_error_mm: msg.longitudinal_error_mm,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      robot_id: msg.robot_id.to_string(),
      state: msg.state,
      lateral_error_mm: msg.lateral_error_mm,
      longitudinal_error_mm: msg.longitudinal_error_mm,
    }
  }
}


// Corresponds to skyvolt_msgs__msg__ChargingJob
/// A request to deliver a charging pile to a target support point.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChargingJob {

    // This member is not documented.
    #[allow(missing_docs)]
    pub job_id: std::string::String,

    /// vehicle / parking-spot identifier
    pub requester_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_branch_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_arclength_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub submitted_at: builtin_interfaces::msg::Time,

    /// 0 = no deadline
    pub deadline: builtin_interfaces::msg::Time,

    /// 0=low, 255=urgent
    pub priority: u8,

}



impl Default for ChargingJob {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ChargingJob::default())
  }
}

impl rosidl_runtime_rs::Message for ChargingJob {
  type RmwMsg = super::msg::rmw::ChargingJob;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        job_id: msg.job_id.as_str().into(),
        requester_id: msg.requester_id.as_str().into(),
        target_branch_id: msg.target_branch_id,
        target_arclength_m: msg.target_arclength_m,
        submitted_at: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.submitted_at)).into_owned(),
        deadline: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.deadline)).into_owned(),
        priority: msg.priority,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        job_id: msg.job_id.as_str().into(),
        requester_id: msg.requester_id.as_str().into(),
      target_branch_id: msg.target_branch_id,
      target_arclength_m: msg.target_arclength_m,
        submitted_at: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.submitted_at)).into_owned(),
        deadline: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.deadline)).into_owned(),
      priority: msg.priority,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      job_id: msg.job_id.to_string(),
      requester_id: msg.requester_id.to_string(),
      target_branch_id: msg.target_branch_id,
      target_arclength_m: msg.target_arclength_m,
      submitted_at: builtin_interfaces::msg::Time::from_rmw_message(msg.submitted_at),
      deadline: builtin_interfaces::msg::Time::from_rmw_message(msg.deadline),
      priority: msg.priority,
    }
  }
}


// Corresponds to skyvolt_msgs__msg__FleetStatus
/// Aggregate fleet status snapshot.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FleetStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FleetStatus::default())
  }
}

impl rosidl_runtime_rs::Message for FleetStatus {
  type RmwMsg = super::msg::rmw::FleetStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        num_robots: msg.num_robots,
        num_pending_jobs: msg.num_pending_jobs,
        num_active_jobs: msg.num_active_jobs,
        num_reservations: msg.num_reservations,
        avg_job_age_s: msg.avg_job_age_s,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      num_robots: msg.num_robots,
      num_pending_jobs: msg.num_pending_jobs,
      num_active_jobs: msg.num_active_jobs,
      num_reservations: msg.num_reservations,
      avg_job_age_s: msg.avg_job_age_s,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      num_robots: msg.num_robots,
      num_pending_jobs: msg.num_pending_jobs,
      num_active_jobs: msg.num_active_jobs,
      num_reservations: msg.num_reservations,
      avg_job_age_s: msg.avg_job_age_s,
    }
  }
}


