use std::ffi::CStr;
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
        let mut buf = [0i8; ERR_MSG_LEN];
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
