//! ACS SPiiPlus motion controller driver (motorAcsMotion).

pub mod bincomm;
pub mod comm;
pub mod ioc;
pub mod ioc_comm;
pub mod spiiplus;

pub use spiiplus::{AcsMotionParams, SpiiPlusAxis, SpiiPlusController};
