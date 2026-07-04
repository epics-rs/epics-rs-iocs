//! Oriel Encoder Mike 18011 (EMC18011) motor controller driver.

pub mod emc18011;
pub mod ioc;

pub use emc18011::{Emc18011Axis, Emc18011Controller};
