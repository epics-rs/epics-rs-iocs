//! areaDetector driver for the Quantum Detectors Merlin (Medipix) family.
//!
//! Port of `areaDetector/ADMerlin/merlinApp/src`. The detector is driven by a
//! Labview server that speaks MPX over two TCP sockets: a request/response
//! command channel and a push-only data channel. Both are ordinary asyn octet
//! ports, created by `drvAsynIPPortConfigure` in the startup script.
//!
//! - [`protocol`] — MPX framing, the command codec, and the data-frame header
//!   parser. Pure functions; this is where the unit tests live.
//! - [`image`] — pixel payload decoding (byte order and the Y flip).
//! - [`connection`] — MPX framing over one asyn octet port.
//! - [`driver`] — the asyn port driver and its parameters.
//! - [`task`] — the data and status background tasks.

pub mod connection;
pub mod driver;
pub mod image;
pub mod params;
pub mod protocol;
pub mod task;
pub mod types;

pub use driver::{MerlinDetector, MerlinRuntime, create_merlin_detector};
pub use types::DetectorType;
