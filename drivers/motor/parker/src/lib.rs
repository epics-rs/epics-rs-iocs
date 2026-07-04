//! Parker motor controller drivers.
//!
//! The Parker `motorParker` module ships several controllers; the **OEM750**
//! and **ACR** (including the Aries) series, both ASCII over an asyn octet port,
//! are ported here. The PC6K controller is a planned addition to this crate.

pub mod acr;
pub mod ioc;
pub mod oem;

pub use acr::{AcrAxis, AcrController};
pub use oem::{OemAxis, OemController};
