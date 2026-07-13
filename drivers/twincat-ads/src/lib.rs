//! EPICS driver for Beckhoff TwinCAT PLCs over the ADS protocol.
//!
//! Port of epics-modules `twincat-ads` (`adsAsynPortDriver`). The C driver links
//! Beckhoff's `AdsLib`; this crate speaks the ADS/AMS wire protocol directly
//! (see [`ads`]), so nothing C or C++ is built or linked.
//!
//! Layers, bottom up:
//!
//! * [`ads`] — AMS/TCP framing, the nine ADS commands, notifications, symbol
//!   lookup. Knows nothing about EPICS.
//! * [`time`] — Windows FILETIME → EPICS timestamps.
//! * [`drvinfo`] — the `ADSPORT=851/…/Main.fTest?` link grammar.
//! * [`convert`] — the PLC-type × asyn-type matrix.
//! * [`octet`] — the asynOctet ASCII command protocol.

pub mod ads;
pub mod convert;
pub mod drvinfo;
pub mod octet;
pub mod time;

pub use ads::{AdsClient, AdsError, AdsState, AdsType, AmsAddr, AmsNetId};
pub use drvinfo::{DrvInfo, DrvInfoDefaults, TimeBase};
