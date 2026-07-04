//! Micronix MMC-100/103/110/200 motor controller driver.
//!
//! Ported from `epics-modules/motor` motorMicronix (`MMC200Driver.cpp`, a
//! model-3 `asynMotorController`/`asynMotorAxis` driver). One
//! [`Mmc200Controller`] is shared (`Arc<Mutex<_>>`) by its per-axis
//! [`Mmc200Axis`]es; the [`ioc`] module provides the `MMC200CreateController`
//! iocsh command.

pub mod ioc;
pub mod mmc200;

pub use mmc200::{Mmc200Axis, Mmc200Controller};
