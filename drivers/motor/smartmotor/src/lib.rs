//! Animatics SmartMotor integrated servo controller driver.

pub mod ioc;
pub mod smartmotor;

pub use smartmotor::{SmartMotorAxis, SmartMotorController};
