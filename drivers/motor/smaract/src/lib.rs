//! SmarAct piezo-stepper controller drivers.
//!
//! The SmarAct `motorSmarAct` module ships three controllers, all ported here:
//! the **MCS2** (ASCII SCPI), the **SCU** (ASCII serial) and the **MCS**
//! (ASCII RS-232).

pub mod ioc;
pub mod mcs;
pub mod mcs2;
pub mod scu;

pub use mcs::{McsAxis, McsController};
pub use mcs2::{Mcs2Axis, Mcs2Controller};
pub use scu::{ScuAxis, ScuController};
