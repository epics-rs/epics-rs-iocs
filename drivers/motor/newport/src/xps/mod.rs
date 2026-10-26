//! Newport XPS-C8 motion controller driver (TCP, full-parity port of
//! `epics-modules/motor` `motorNewport` `XPSController`/`XPSAxis`).
//!
//! Unlike the serial Newport controllers, the XPS speaks an ASCII RPC protocol
//! over TCP: the vendor `XPS_C8_drivers` library is a pure command marshaller
//! (`FuncName (args)` out, `errorCode,values,EndOfAPI` back), so this port
//! reimplements the ~60 RPC functions the motor driver uses directly on the
//! asyn TCP transport — no vendor binary is required. See [`rpc`] for the wire
//! layer.

pub mod axis;
pub mod commands;
pub mod controller;
pub mod corrector;
#[cfg(test)]
pub(crate) mod fake;
pub mod ftp;
pub mod gathering;
pub mod gpio;
pub mod pco;
pub mod profile;
pub mod profile_move;
pub mod rpc;
pub mod sftp;

pub use axis::XpsAxis;
pub use controller::{SetPositionFailure, XpsController, XpsParams};
pub use corrector::{CorrectorType, PidFailure, XpsCorrectorInfo};
pub use gathering::{GatheringReadback, parse_gathering_buffer};
pub use pco::PcoParams;
pub use profile::{Profile, ProfileAxis, ProfileError, TrajectoryFile, pulse_output_window};
pub use rpc::{SocketMode, XPS_TERMINATOR, XpsError, XpsReply, XpsResult, XpsSocket};
