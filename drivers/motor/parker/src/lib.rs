//! Parker motor controller drivers.
//!
//! The Parker `motorParker` module ships several controllers; the **OEM750**
//! and **ACR** (including the Aries) series, both ASCII over an asyn octet port,
//! are ported here, and the model-1 PC6K (6K series) beside them.

pub mod acr;
pub mod drv_pc6k;
pub mod ioc;
pub mod oem;

pub use acr::{AcrAxis, AcrController, AcrParams};
pub use oem::{OemAxis, OemController};
