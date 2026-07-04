//! Micos motor controller drivers.
//!
//! The Micos `motorMicos` module ships several controllers; the **SMC corvus**
//! (ASCII over an asyn octet port) is ported here. The SMChydra and SMCTaurus
//! controllers are planned additions to this crate.

pub mod corvus;
pub mod ioc;

pub use corvus::{CorvusAxis, CorvusController};
