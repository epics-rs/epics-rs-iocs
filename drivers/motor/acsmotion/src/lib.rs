//! ACS SPiiPlus motion controller driver (motorAcsMotion).

pub mod aux_io;
pub mod bincomm;
pub mod comm;
#[cfg(test)]
mod fake;
pub mod ioc;
pub mod ioc_aux;
pub mod ioc_comm;
pub mod spiiplus;

pub use spiiplus::{AcsMotionParams, SpiiPlusAxis, SpiiPlusController};
