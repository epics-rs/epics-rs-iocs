//! ACS Tech80 SPiiPlus motion controller driver.

pub mod ioc;
pub mod spiiplus;

pub use spiiplus::{CommandMode, SpiiPlusAxis, SpiiPlusController};
