//! Allen-Bradley ControlLogix / PLC-5 EtherNet/IP + CIP driver.
//!
//! A port of `epics-modules/ether_ip` (`ether_ip.c`, `drvEtherIP.c`,
//! `devEtherIP.c`). The layers, bottom up:
//!
//! - [`cip`] -- the CIP message-router PDU: tag-path encoding, ReadData /
//!   WriteData / MultiRequest / CM_Unconnected_Send, and the typed value
//!   accessors. Pure functions, unit-tested against byte fixtures produced by
//!   the C encoders themselves.
//! - [`encap`] -- the EtherNet/IP encapsulation header and its commands
//!   (`ListServices`, `RegisterSession`, `SendRRData`). Also pure.
//! - [`connection`] -- one TCP session with one PLC. The only socket code.
//! - [`driver`] -- PLCs, scan lists, tags, and the per-PLC scan thread that
//!   packs tags into MultiRequests.
//! - [`device`] -- the EPICS device support and the `@PLC tag flags` link.

pub mod cip;
pub mod connection;
pub mod device;
pub mod driver;
pub mod encap;
