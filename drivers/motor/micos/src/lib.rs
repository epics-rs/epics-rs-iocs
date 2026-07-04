//! Micos motor controller drivers.
//!
//! The Micos `motorMicos` module ships several controllers; the **SMC corvus**
//! and **SMC hydra** (ASCII over an asyn octet port) are ported here. The
//! SMCTaurus controller is a planned addition to this crate.

pub mod corvus;
pub mod hydra;
pub mod ioc;

pub use corvus::{CorvusAxis, CorvusController};
pub use hydra::{HydraAxis, HydraController};
