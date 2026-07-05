//! AMCI ANF2 / ANG1 stepper motor controller drivers (Modbus/TCP register
//! access), ported from `epics-modules/motor` `motorAMCI`.

pub mod anf2;
pub mod ang1;
pub mod ioc;
pub mod regs;

pub use anf2::{Anf2Axis, Anf2Controller};
pub use ang1::{Ang1Axis, Ang1Controller};
