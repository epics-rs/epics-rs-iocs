//! SmarAct piezo-stepper controller drivers.
//!
//! The SmarAct `motorSmarAct` module ships three controllers: the **MCS2**
//! (ASCII SCPI) and the **SCU** (ASCII serial), both ported here, and the
//! **MCS** (a planned addition to this crate).

pub mod ioc;
pub mod mcs2;
pub mod scu;

pub use mcs2::{Mcs2Axis, Mcs2Controller};
pub use scu::{ScuAxis, ScuController};
