//! Errors raised by the ADS client.

use std::fmt;

use super::defs::error_to_string;

/// Anything that can go wrong talking ADS to a PLC.
#[derive(Debug)]
pub enum AdsError {
    /// The PLC (or the AMS router) answered with a non-zero ADS error code.
    ///
    /// This covers both the AoE header's `errorCode` and the response payload's
    /// leading `result` word; the C AdsLib merges them the same way.
    Ads(u32),
    /// A frame was shorter than the layout requires.
    ShortFrame { need: usize, got: usize },
    /// The PLC returned fewer bytes than the request asked for.
    ShortRead { need: usize, got: usize },
    /// Socket-level failure.
    Io(std::io::Error),
    /// No response arrived within the configured ADS command timeout.
    Timeout,
    /// The client is not connected to the PLC.
    NotConnected,
}

impl AdsError {
    /// The ADS error code, for callers that mirror the C driver's
    /// `if (status == ADSERR_...)` checks.
    pub fn code(&self) -> Option<u32> {
        match self {
            Self::Ads(c) => Some(*c),
            _ => None,
        }
    }
}

impl fmt::Display for AdsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ads(c) => write!(f, "ADS error 0x{:x} ({})", c, error_to_string(*c)),
            Self::ShortFrame { need, got } => {
                write!(f, "short frame: need {need} bytes, got {got}")
            }
            Self::ShortRead { need, got } => {
                write!(f, "short read: expected {need} bytes, PLC returned {got}")
            }
            Self::Io(e) => write!(f, "I/O error: {e}"),
            Self::Timeout => write!(f, "ADS command timed out"),
            Self::NotConnected => write!(f, "not connected to PLC"),
        }
    }
}

impl std::error::Error for AdsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for AdsError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Map a non-zero ADS status to an error; zero is success.
pub fn check(status: u32) -> Result<(), AdsError> {
    if status == 0 {
        Ok(())
    } else {
        Err(AdsError::Ads(status))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_maps_zero_to_ok_and_nonzero_to_error() {
        assert!(check(0).is_ok());
        let e = check(0x0710).unwrap_err();
        assert_eq!(e.code(), Some(0x0710));
        assert!(e.to_string().contains("ADSERR_DEVICE_SYMBOLNOTFOUND"));
    }
}
