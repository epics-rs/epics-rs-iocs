//! ACS MCB-4B stepper motor controller driver.

pub mod drv_mcb4b;
pub mod ioc;
pub mod mcb4b;

pub use mcb4b::{Mcb4bAxis, Mcb4bController};
