//! nPoint C300-series piezo controller driver.
//!
//! The nPoint `motorNPoint` module ships two controllers: the **C300** (ASCII
//! SCPI-style protocol, ported here) and the **LC400** (a binary struct
//! protocol over TCP — a planned addition to this crate).

pub mod c300;
pub mod ioc;

pub use c300::{C300Axis, C300Controller};
