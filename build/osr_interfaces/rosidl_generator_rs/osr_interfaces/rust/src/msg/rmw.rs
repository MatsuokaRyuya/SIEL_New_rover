#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "osr_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__osr_interfaces__msg__Status() -> *const std::ffi::c_void;
}

#[link(name = "osr_interfaces__rosidl_generator_c")]
extern "C" {
    fn osr_interfaces__msg__Status__init(msg: *mut Status) -> bool;
    fn osr_interfaces__msg__Status__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Status>, size: usize) -> bool;
    fn osr_interfaces__msg__Status__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Status>);
    fn osr_interfaces__msg__Status__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Status>, out_seq: *mut rosidl_runtime_rs::Sequence<Status>) -> bool;
}

// Corresponds to osr_interfaces__msg__Status
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// voltage level being read into the RoboClaws, in Volts

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Status {

    // This member is not documented.
    #[allow(missing_docs)]
    pub battery: f32,

    /// status of an error being present on each RoboClaw. Stringified hex
    pub error_status: [rosidl_runtime_rs::String; 3],

    /// Temperature of the motor controllers expressed in Celsius
    pub temp: [f32; 3],

    /// Sensed current from the motor controllers in Ampere
    pub current: [f32; 6],

}



impl Default for Status {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !osr_interfaces__msg__Status__init(&mut msg as *mut _) {
        panic!("Call to osr_interfaces__msg__Status__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Status {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__Status__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__Status__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__Status__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Status {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Status where Self: Sized {
  const TYPE_NAME: &'static str = "osr_interfaces/msg/Status";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__osr_interfaces__msg__Status() }
  }
}


#[link(name = "osr_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__osr_interfaces__msg__CommandDrive() -> *const std::ffi::c_void;
}

#[link(name = "osr_interfaces__rosidl_generator_c")]
extern "C" {
    fn osr_interfaces__msg__CommandDrive__init(msg: *mut CommandDrive) -> bool;
    fn osr_interfaces__msg__CommandDrive__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CommandDrive>, size: usize) -> bool;
    fn osr_interfaces__msg__CommandDrive__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CommandDrive>);
    fn osr_interfaces__msg__CommandDrive__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CommandDrive>, out_seq: *mut rosidl_runtime_rs::Sequence<CommandDrive>) -> bool;
}

// Corresponds to osr_interfaces__msg__CommandDrive
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// drive motors, velocity in rad/s

#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !osr_interfaces__msg__CommandDrive__init(&mut msg as *mut _) {
        panic!("Call to osr_interfaces__msg__CommandDrive__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CommandDrive {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__CommandDrive__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__CommandDrive__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__CommandDrive__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CommandDrive {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CommandDrive where Self: Sized {
  const TYPE_NAME: &'static str = "osr_interfaces/msg/CommandDrive";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__osr_interfaces__msg__CommandDrive() }
  }
}


#[link(name = "osr_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__osr_interfaces__msg__CommandCorner() -> *const std::ffi::c_void;
}

#[link(name = "osr_interfaces__rosidl_generator_c")]
extern "C" {
    fn osr_interfaces__msg__CommandCorner__init(msg: *mut CommandCorner) -> bool;
    fn osr_interfaces__msg__CommandCorner__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CommandCorner>, size: usize) -> bool;
    fn osr_interfaces__msg__CommandCorner__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CommandCorner>);
    fn osr_interfaces__msg__CommandCorner__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CommandCorner>, out_seq: *mut rosidl_runtime_rs::Sequence<CommandCorner>) -> bool;
}

// Corresponds to osr_interfaces__msg__CommandCorner
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// corner motors, position in rad around the vertical z-axis, with zero indicating the position where the wheels are straight

#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !osr_interfaces__msg__CommandCorner__init(&mut msg as *mut _) {
        panic!("Call to osr_interfaces__msg__CommandCorner__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CommandCorner {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__CommandCorner__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__CommandCorner__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { osr_interfaces__msg__CommandCorner__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CommandCorner {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CommandCorner where Self: Sized {
  const TYPE_NAME: &'static str = "osr_interfaces/msg/CommandCorner";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__osr_interfaces__msg__CommandCorner() }
  }
}


