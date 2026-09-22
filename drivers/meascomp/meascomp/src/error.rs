use std::ffi::{CStr, c_char};
use std::fmt;

use uldaq_sys::{self, ERR_MSG_LEN, ERR_NO_ERROR};

/// Error type wrapping a uldaq `UlError` code with a human-readable message.
#[derive(Debug, Clone)]
pub struct MeasCompError {
    pub code: uldaq_sys::UlError,
    pub message: String,
}

impl MeasCompError {
    pub fn from_code(code: uldaq_sys::UlError) -> Self {
        let mut buf = [0 as c_char; ERR_MSG_LEN];
        unsafe {
            uldaq_sys::ulGetErrMsg(code, buf.as_mut_ptr());
        }
        let message = unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        Self { code, message }
    }
}

impl fmt::Display for MeasCompError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "uldaq error {}: {}", self.code, self.message)
    }
}

impl std::error::Error for MeasCompError {}

pub type Result<T> = std::result::Result<T, MeasCompError>;

/// Check a uldaq return code; return Ok(()) on success or Err on failure.
#[inline]
pub fn check(code: uldaq_sys::UlError) -> Result<()> {
    if code == ERR_NO_ERROR {
        Ok(())
    } else {
        Err(MeasCompError::from_code(code))
    }
}

/// What a `ul*ScanStatus` call reported. libuldaq fills the scan state and
/// transfer position even when it also returns an error -- a scan that ended
/// on a transfer error is reported `SS_IDLE` together with that error -- so
/// the error travels beside them instead of replacing them.
#[derive(Debug, Clone)]
pub struct ScanStatusReport {
    pub status: i32,
    pub xfer: uldaq_sys::TransferStatus,
    pub error: Option<MeasCompError>,
}

impl ScanStatusReport {
    pub(crate) fn new(
        code: uldaq_sys::UlError,
        status: i32,
        xfer: uldaq_sys::TransferStatus,
    ) -> Self {
        Self {
            status,
            xfer,
            error: check(code).err(),
        }
    }

    /// The four numbers C's pollers `asynPrint` after this call.
    pub fn position(&self) -> ScanPosition {
        ScanPosition {
            code: self.error.as_ref().map_or(ERR_NO_ERROR, |e| e.code),
            status: self.status,
            total_count: self.xfer.current_total_count,
            index: self.xfer.current_index,
        }
    }
}

/// A scan-status call as C traces it: the libuldaq return code, the scan
/// state, and the transfer position (`currentTotalCount`, `currentIndex`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ScanPosition {
    pub code: uldaq_sys::UlError,
    pub status: i32,
    pub total_count: u64,
    pub index: i64,
}
