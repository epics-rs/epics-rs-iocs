//! Constants of the PSL driver (C `PSL.cpp`).

use std::time::Duration;

/// Read buffer, big enough for the binary image chunks the server sends
/// (C `MAX_MESSAGE_SIZE`).
pub const MAX_MESSAGE_SIZE: usize = 4096;
/// Timeout on an ordinary command (C `PSL_SERVER_TIMEOUT`).
pub const SERVER_TIMEOUT: Duration = Duration::from_secs(2);
/// Timeout on `GetImage`, which waits for the exposure (C
/// `PSL_GET_IMAGE_TIMEOUT`).
pub const GET_IMAGE_TIMEOUT: Duration = Duration::from_secs(20);
/// Oldest PSLViewer this driver speaks to (C `getVersion`).
pub const MIN_SERVER_VERSION: f64 = 4.3;
/// The prefix of the `GetVersion` reply (C `expectedResponse`).
pub const VERSION_PREFIX: &str = "PSLViewer-";
/// How long the acquisition task waits between `HasNewData` polls
/// (C `epicsThreadSleepQuantum()`).
pub const POLL_INTERVAL: Duration = Duration::from_millis(20);

pub const PSL_CAMERA_NAME: &str = "PSL_CAMERA_NAME";
pub const PSL_TIFF_COMMENT: &str = "PSL_TIFF_COMMENT";
