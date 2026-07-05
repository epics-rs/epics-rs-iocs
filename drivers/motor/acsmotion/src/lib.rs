//! ACS SPiiPlus motion controller driver (motorAcsMotion).

pub mod ioc;
pub mod spiiplus;

pub use spiiplus::{SpiiPlusAxis, SpiiPlusController};
