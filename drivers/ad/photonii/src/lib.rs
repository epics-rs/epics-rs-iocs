//! Bruker PhotonII areaDetector driver, ported from `areaDetector/ADPhotonII`
//! (`PhotonIIApp/src/PhotonII.cpp`).
//!
//! The detector is driven by Bruker's `p2util` program, which the IOC reaches
//! over a TCP socket (a procServ port). The driver sends `p2util` command lines
//! (`set --exposure-time 1.0`, `grab --dstdir ... --count 5`, `abort`), and
//! `p2util` answers each command and, during an acquisition, announces every
//! frame it writes with a line naming the `.raw` file. The driver then reads
//! that file off the file system and publishes it as an NDArray.
//!
//! Layout:
//! - [`protocol`] — the command language and the frame-message parse, as pure
//!   functions.
//! - [`raw`] — the `.raw` frame file: readiness test and decode.
//! - [`connection`] — the p2util socket.
//! - [`driver`] — the asyn port, the parameters, the record writes.
//! - [`task`] — the acquisition task.

pub mod connection;
pub mod driver;
pub mod params;
pub mod protocol;
pub mod raw;
pub mod task;
pub mod types;

pub use driver::{PhotonIIDetector, PhotonIIRuntime, create_photonii_detector};
pub use types::{FrameType, PII_UTIL};
