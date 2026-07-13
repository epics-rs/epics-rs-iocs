//! Constants and the enumerations the BIS driver adds (C `BISDetector.cpp`).

use std::time::Duration;

/// The longest message BIS sends or takes (C `MAX_MESSAGE_SIZE`).
pub const MAX_MESSAGE_SIZE: usize = 512;

/// The longest frame file name (C `MAX_FILENAME_LEN`).
pub const MAX_FILENAME_LEN: usize = 256;

/// How long the status socket is read for before the read is started again.
///
/// C read with a timeout of -1 — forever. A finite wait that goes straight back
/// to reading is the same thing to BIS, and it does not tie the thread to a
/// socket that may never be connected.
pub const STATUS_READ_TIMEOUT: Duration = Duration::from_secs(10);

/// The timeout of an ordinary command (C `BIS_DEFAULT_TIMEOUT`).
pub const BIS_DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);

/// The timeout of a scan or shutter command (C's `writeBIS(2.0)` calls).
pub const BIS_COMMAND_TIMEOUT: Duration = Duration::from_secs(2);

/// How often the exposure countdown is published (C `BIS_POLL_DELAY`).
pub const BIS_POLL_DELAY: Duration = Duration::from_millis(10);

/// How often the frame file is looked for (C `FILE_READ_DELAY`).
pub const FILE_READ_DELAY: Duration = Duration::from_millis(10);

/// How long BIS is given to say it has finished processing the frame
/// (C's `epicsEventWaitWithTimeout(readoutEventId, 5.0)`).
pub const READOUT_TIMEOUT: Duration = Duration::from_secs(5);

/// How far the file server's clock may be behind ours before a frame file that
/// is older than the exposure is taken for a leftover (C's `> -10` on
/// `difftime`).
pub const CLOCK_SKEW_ALLOWANCE: Duration = Duration::from_secs(10);

/// The detector geometry BIS reports before it has said anything
/// (C's `setIntegerParam(ADMaxSizeX, 4096)`).
pub const MAX_SIZE: i32 = 4096;

// The parameters this driver adds to the areaDetector base set.
pub const BIS_SFRM_TIMEOUT: &str = "SFRM_TIMEOUT";
pub const BIS_NUM_DARKS: &str = "NUM_DARKS";
pub const BIS_STATUS: &str = "BIS_STATUS";

// Two parameters that have no record: they are how the acquisition task asks
// the port actor — the only owner of both the parameter library and the command
// socket — to do something on its behalf. C did both from the task thread, under
// the port lock.
/// Name the next frame's file and send BIS the scan command for it
/// (C's `createFileName` plus the `switch (frameType)` that followed it).
pub const BIS_START_SCAN: &str = "BIS_START_SCAN";
/// Drive the EPICS shutter (C's `ADDriver::setShutter` calls in `BISTask`).
pub const BIS_EPICS_SHUTTER: &str = "BIS_EPICS_SHUTTER";

/// What BIS is asked to collect (C `BISFrameType_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    Normal = 0,
    Dark = 1,
    Raw = 2,
    DoubleCorrelation = 3,
}

impl FrameType {
    pub fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Normal),
            1 => Some(Self::Dark),
            2 => Some(Self::Raw),
            3 => Some(Self::DoubleCorrelation),
            _ => None,
        }
    }
}
