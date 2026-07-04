//! Pro-Dex OMS MAXnet / MXA asyn motor controller driver.
//!
//! See [`oms`] for the protocol and porting notes, and [`ioc`] for the iocsh
//! configuration commands.

pub mod ioc;
pub mod oms;

pub use oms::{OmsAxis, OmsController};
