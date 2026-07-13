//! areaDetector driver for a Bruker detector run by the BIS server
//! (a Rust port of areaDetector `ADBruker`, `BISDetector.cpp`).
//!
//! BIS is a server, not a detector: the driver sends it bracketed ASCII
//! commands on one TCP socket, listens to what it broadcasts on another, and
//! reads the frames BIS writes as SFRM files on a shared filesystem. No vendor
//! library is involved.

pub mod connection;
pub mod driver;
pub mod filename;
pub mod params;
pub mod protocol;
pub mod sfrm;
pub mod task;
pub mod types;

pub use driver::{BrukerDetector, BrukerRuntime, create_bruker_detector};
pub use params::BrukerParams;
pub use types::FrameType;
