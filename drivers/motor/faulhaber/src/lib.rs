//! Faulhaber MCDC2805 DC servo motor controller driver.
//!
//! Ported from `epics-modules/motor` motorFaulhaber (`drvMCDC2805.cc` +
//! `devMCDC2805.cc`, the model-1 dev/drv pair). Several MCDC2805 modules can
//! share one serial line, each addressed by a node number; one
//! [`FaulhaberController`] owns the shared serial handle and its per-axis
//! [`FaulhaberAxis`]es. The [`ioc`] module provides the `MCDC2805Setup` /
//! `MCDC2805Config` iocsh commands.

pub mod faulhaber;
pub mod ioc;

pub use faulhaber::{FaulhaberAxis, FaulhaberController};
