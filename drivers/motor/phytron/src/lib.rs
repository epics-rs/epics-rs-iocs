//! Phytron motor controller drivers.
//!
//! The Phytron `motorPhytron` module drives the **phyMOTION** (MCM) and
//! **MCC-1/MCC-2** stepper controllers over an asyn octet port with STX/ETX
//! framing. Both are ported here onto the asyn-rs `AsynMotor` boundary.

pub mod ioc;
pub mod phytron;

pub use phytron::{CtrlType, PhytronAxis, PhytronController};
