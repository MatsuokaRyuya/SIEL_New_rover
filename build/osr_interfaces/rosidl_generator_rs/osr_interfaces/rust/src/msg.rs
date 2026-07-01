#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to osr_interfaces__msg__Status
/// voltage level being read into the RoboClaws, in Volts

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Status {

    // This member is not documented.
    #[allow(missing_docs)]
    pub battery: f32,

    /// status of an error being present on each RoboClaw. Stringified hex
    pub error_status: [std::string::String; 3],

    /// Temperature of the motor controllers expressed in Celsius
    pub temp: [f32; 3],

    /// Sensed current from the motor controllers in Ampere
    pub current: [f32; 6],

}



impl Default for Status {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Status::default())
  }
}

impl rosidl_runtime_rs::Message for Status {
  type RmwMsg = super::msg::rmw::Status;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        battery: msg.battery,
        error_status: msg.error_status
          .map(|elem| elem.as_str().into()),
        temp: msg.temp,
        current: msg.current,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      battery: msg.battery,
        error_status: msg.error_status
          .iter()
          .map(|elem| elem.as_str().into())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
        temp: msg.temp,
        current: msg.current,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      battery: msg.battery,
      error_status: msg.error_status
        .map(|elem| elem.to_string()),
      temp: msg.temp,
      current: msg.current,
    }
  }
}


// Corresponds to osr_interfaces__msg__CommandDrive
/// drive motors, velocity in rad/s

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CommandDrive {

    // This member is not documented.
    #[allow(missing_docs)]
    pub left_front_vel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_middle_vel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_back_vel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_front_vel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_middle_vel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_back_vel: f64,

}



impl Default for CommandDrive {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CommandDrive::default())
  }
}

impl rosidl_runtime_rs::Message for CommandDrive {
  type RmwMsg = super::msg::rmw::CommandDrive;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        left_front_vel: msg.left_front_vel,
        left_middle_vel: msg.left_middle_vel,
        left_back_vel: msg.left_back_vel,
        right_front_vel: msg.right_front_vel,
        right_middle_vel: msg.right_middle_vel,
        right_back_vel: msg.right_back_vel,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      left_front_vel: msg.left_front_vel,
      left_middle_vel: msg.left_middle_vel,
      left_back_vel: msg.left_back_vel,
      right_front_vel: msg.right_front_vel,
      right_middle_vel: msg.right_middle_vel,
      right_back_vel: msg.right_back_vel,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      left_front_vel: msg.left_front_vel,
      left_middle_vel: msg.left_middle_vel,
      left_back_vel: msg.left_back_vel,
      right_front_vel: msg.right_front_vel,
      right_middle_vel: msg.right_middle_vel,
      right_back_vel: msg.right_back_vel,
    }
  }
}


// Corresponds to osr_interfaces__msg__CommandCorner
/// corner motors, position in rad around the vertical z-axis, with zero indicating the position where the wheels are straight

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CommandCorner {

    // This member is not documented.
    #[allow(missing_docs)]
    pub left_front_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_back_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_front_pos: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_back_pos: f64,

}



impl Default for CommandCorner {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CommandCorner::default())
  }
}

impl rosidl_runtime_rs::Message for CommandCorner {
  type RmwMsg = super::msg::rmw::CommandCorner;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        left_front_pos: msg.left_front_pos,
        left_back_pos: msg.left_back_pos,
        right_front_pos: msg.right_front_pos,
        right_back_pos: msg.right_back_pos,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      left_front_pos: msg.left_front_pos,
      left_back_pos: msg.left_back_pos,
      right_front_pos: msg.right_front_pos,
      right_back_pos: msg.right_back_pos,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      left_front_pos: msg.left_front_pos,
      left_back_pos: msg.left_back_pos,
      right_front_pos: msg.right_front_pos,
      right_back_pos: msg.right_back_pos,
    }
  }
}


