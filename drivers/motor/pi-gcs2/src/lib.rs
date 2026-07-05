//! PI (Physik Instrumente) GCS2 stage-controller driver (motorPIGCS2).

pub mod gcs2;
pub mod ioc;

pub use gcs2::{PIGCS2Axis, PIGCS2Controller};
