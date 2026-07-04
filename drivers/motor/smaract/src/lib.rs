//! SmarAct MCS2-series piezo-stepper controller driver.
//!
//! The SmarAct `motorSmarAct` module ships three controllers: the **MCS2**
//! (ASCII SCPI, ported here), the **SCU** and the **MCS** (planned additions to
//! this crate).

pub mod ioc;
pub mod mcs2;

pub use mcs2::{Mcs2Axis, Mcs2Controller};
