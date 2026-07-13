//! areaDetector driver for Photonic Sciences Ltd. CCD detectors, ported from
//! `ADPSL/pslApp/src/PSL.cpp`.
//!
//! The detector is driven by the PSLViewer program on the camera PC, which
//! listens on a TCP socket and speaks a `Command;argument` language. This
//! driver talks to it through a `drvAsynIPPort`: it reads the configuration
//! back after every write, polls `HasNewData` while an acquisition runs and
//! pulls each frame off the same socket with `GetImage`. The image *files*
//! PSLViewer writes are its own business — the driver only tells it where to
//! put them (`SetRecordPath` / `SetRecordName` / `SetAutoSave`).

pub mod connection;
pub mod driver;
pub mod image;
pub mod params;
pub mod protocol;
pub mod task;
pub mod types;

pub use driver::{PslDetector, PslRuntime, create_psl_detector};
pub use params::PslParams;
