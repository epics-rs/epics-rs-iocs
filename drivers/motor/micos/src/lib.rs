//! Micos motor controller drivers.
//!
//! The Micos `motorMicos` module ships several controllers; the **SMC corvus**,
//! **SMC hydra** and **SMC Taurus** (ASCII over an asyn octet port) are ported
//! here, with the model-1 **MoCo dc** in [`drv_micos`].

pub mod corvus;
pub mod drv_micos;
pub mod hydra;
pub mod ioc;
pub mod smc;
pub mod taurus;

pub use corvus::{CorvusAxis, CorvusController};
pub use hydra::{HydraAxis, HydraController};
pub use taurus::{TaurusAxis, TaurusController};
