//! Aerotech asyn motor controller drivers (Ensemble and A3200).
//!
//! See [`ensemble`] and [`a3200`] for the per-controller protocols and porting
//! notes, and [`ioc`] for the iocsh configuration commands.

pub mod a3200;
pub mod ensemble;
pub mod ioc;

pub use a3200::{A3200_MAX_AXES, A3200Axis, A3200Controller};
pub use ensemble::{ENSEMBLE_MAX_AXES, EnsembleAxis, EnsembleController};
