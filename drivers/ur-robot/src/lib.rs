//! Universal Robots driver — a Rust port of epics-modules `urRobot`.
//!
//! `urRobot` is an asyn module that drives a UR arm through four TCP interfaces,
//! all of which are re-implemented here from the C++ they were built on
//! (`ur_rtde`, SDU Robotics, pin 68ac4e18, vendored in
//! `urRobotSupport/rtdeSrc/ur_rtde`):
//!
//! | interface | port | module |
//! |---|---|---|
//! | RTDE (binary, big-endian) | 30004 | [`rtde`], [`session`], [`stream`], [`receive`], [`control`], [`io`] |
//! | script server (URScript text) | 30003 | [`script`] |
//! | dashboard server (line text) | 29999 | [`dashboard`] |
//! | Robotiq gripper URCap (text) | 63352 | [`gripper`] |
//!
//! The five asyn port drivers in [`drivers`] mirror urRobot's five
//! `asynPortDriver` subclasses and keep its PV surface.

pub mod control;
pub mod dashboard;
pub mod drivers;
pub mod error;
pub mod gripper;
pub mod io;
pub mod receive;
pub mod registry;
pub mod rtde;
pub mod script;
pub mod session;
pub mod state;
pub mod stream;

pub use error::{UrError, UrResult};
