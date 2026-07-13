//! The ADS/AMS wire subset the twincat-ads driver needs, in pure Rust.
//!
//! Reverse-engineered from Beckhoff's MIT-licensed reference implementation
//! (<https://github.com/Beckhoff/ADS>); no C/C++ is linked. The protocol is
//! plain TCP on port 48898 with a 6-byte AMS/TCP header, a 32-byte AoE header,
//! and a little-endian payload.

pub mod client;
pub mod defs;
pub mod error;
pub mod frame;
pub mod notification;
pub mod symbol;

pub use client::AdsClient;
pub use defs::{AdsState, AdsType, AdsVersion, AmsAddr, AmsNetId};
pub use error::AdsError;
pub use notification::NotificationSample;
pub use symbol::SymbolEntry;
