//! Aerotech Ensemble asyn motor controller driver.
//!
//! See [`ensemble`] for the protocol and porting notes, and [`ioc`] for the
//! iocsh configuration command.

pub mod ensemble;
pub mod ioc;

pub use ensemble::{ENSEMBLE_MAX_AXES, EnsembleAxis, EnsembleController};
