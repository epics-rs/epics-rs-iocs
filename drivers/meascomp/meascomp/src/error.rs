use std::ffi::{CStr, c_char};
use std::fmt;

use uldaq_sys::{self, ERR_MSG_LEN, ERR_NO_ERROR};

/// A failed uldaq call: its `UlError` code alone. The library's text is
/// read only where a caller prints it, as C calls `ulGetErrMsg` there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeasCompError {
    pub code: uldaq_sys::UlError,
}

impl MeasCompError {
    /// The library's text for the code, read by `ulGetErrMsg` each time it
    /// is formatted: inside a trace print, only when the mask lets the line
    /// out, as C `asynPrint` evaluates its arguments.
    pub fn message(&self) -> ErrorMessage {
        ErrorMessage(self.code)
    }
}

/// See [`MeasCompError::message`].
#[derive(Debug, Clone, Copy)]
pub struct ErrorMessage(pub uldaq_sys::UlError);

impl fmt::Display for ErrorMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut buf = [0 as c_char; ERR_MSG_LEN];
        unsafe {
            uldaq_sys::ulGetErrMsg(self.0, buf.as_mut_ptr());
        }
        f.write_str(&unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy())
    }
}

impl fmt::Display for MeasCompError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "uldaq error {}: {}", self.code, self.message())
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
        Err(MeasCompError { code })
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;

    use super::*;

    thread_local! {
        static ERR_MSG_CALLS: Cell<usize> = const { Cell::new(0) };
    }

    /// Counts the library calls; the test binary's definition wins over
    /// libuldaq's.
    #[unsafe(no_mangle)]
    extern "C" fn ulGetErrMsg(
        _code: uldaq_sys::UlError,
        msg: *mut std::ffi::c_char,
    ) -> uldaq_sys::UlError {
        ERR_MSG_CALLS.set(ERR_MSG_CALLS.get() + 1);
        unsafe { msg.write(0) };
        ERR_NO_ERROR
    }

    #[test]
    fn the_library_message_is_read_once_per_format() {
        let err = check(uldaq_sys::ERR_TIMEDOUT).unwrap_err();
        let message = err.message();
        assert_eq!(ERR_MSG_CALLS.get(), 0);
        let _ = format!("{message}");
        assert_eq!(ERR_MSG_CALLS.get(), 1);
    }
}
