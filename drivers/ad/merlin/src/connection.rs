//! MPX framing over an asyn octet port (C `mpxConnection`).
//!
//! One `MpxConnection` owns one socket: the command channel and the data
//! channel each get their own. C shared a single `mpxConnection` object
//! between the two — `merlinTask` called `cmdConnection->mpxRead()` with the
//! *data* channel's asynUser, so a data frame overwrote the command channel's
//! response buffers and error code while the command thread could be mid
//! transaction.

use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;

use crate::protocol::{self, MpxError};
use crate::types::*;

/// Anything that can go wrong on an MPX channel.
#[derive(Debug)]
pub enum ChannelError {
    /// The socket read timed out — expected on an idle data channel.
    Timeout,
    /// Transport failure (socket closed, port disconnected, ...).
    Transport(AsynError),
    /// The peer spoke MPX badly, or reported an error code.
    Protocol(MpxError),
}

impl std::fmt::Display for ChannelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timeout"),
            Self::Transport(e) => write!(f, "{e}"),
            Self::Protocol(e) => write!(f, "{e}"),
        }
    }
}

impl From<MpxError> for ChannelError {
    fn from(e: MpxError) -> Self {
        Self::Protocol(e)
    }
}

impl From<AsynError> for ChannelError {
    fn from(e: AsynError) -> Self {
        // `status()` reads *through* a `PartialRead` carrier, so a timeout that
        // returned partial bytes (EOS interpose installed) is still recognized.
        if e.status() == AsynStatus::Timeout {
            Self::Timeout
        } else {
            Self::Transport(e)
        }
    }
}

/// A framed MPX channel on top of one asyn octet port.
pub struct MpxConnection {
    handle: PortHandle,
    max_frame: usize,
}

impl MpxConnection {
    pub fn new(handle: PortHandle, max_frame: usize) -> Self {
        Self { handle, max_frame }
    }

    fn user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(0).with_addr(0).with_timeout(timeout)
    }

    /// Read exactly `count` bytes, or fail. A zero-length read means the peer
    /// closed the socket.
    fn read_exact(&self, count: usize, timeout: Duration) -> Result<Vec<u8>, ChannelError> {
        let mut out = Vec::with_capacity(count);
        while out.len() < count {
            // OctetReadBinary brackets the read with the input EOS cleared, so
            // a 0x0A byte inside a pixel payload cannot terminate it early —
            // MPX frames are length-delimited, never EOS-delimited.
            let result = self.handle.submit_blocking(
                RequestOp::OctetReadBinary {
                    buf_size: count - out.len(),
                },
                self.user(timeout),
            )?;
            let chunk = result.data.unwrap_or_default();
            if chunk.is_empty() {
                return Err(ChannelError::Transport(AsynError::Status {
                    status: AsynStatus::Disconnected,
                    message: "peer closed the MPX socket".into(),
                }));
            }
            out.extend_from_slice(&chunk);
        }
        Ok(out)
    }

    /// Read one MPX frame, discarding any bytes that precede the `MPX` magic
    /// so the channel re-syncs after an error or a server restart.
    ///
    /// C matched the magic with a counter it reset to zero on every mismatch,
    /// so junk ending in a prefix of the magic (`"...MMPX"`) desynchronised it
    /// permanently. This slides the window by one byte instead.
    pub fn read_frame(&self, timeout: Duration) -> Result<Vec<u8>, ChannelError> {
        let mut window = self.read_exact(MPX_HEADER.len(), timeout)?;
        let mut junk = 0usize;
        while window != MPX_HEADER {
            window.remove(0);
            window.extend_from_slice(&self.read_exact(1, timeout)?);
            junk += 1;
        }
        if junk > 0 {
            log::error!("merlin: discarded {junk} bytes of junk before an MPX header");
        }

        let rest = self.read_exact(MPX_FRAME_HEADER_LEN - MPX_HEADER.len(), timeout)?;
        window.extend_from_slice(&rest);
        let body_len = protocol::parse_frame_header(&window, self.max_frame)?;

        // C returned asynSuccess on a short body read (`status = asynSuccess ?
        // asynError : status` always took the else branch), handing the caller
        // a truncated frame it believed was complete. read_exact cannot.
        self.read_exact(body_len, timeout)
    }

    fn write(&self, msg: &str, timeout: Duration) -> Result<(), ChannelError> {
        self.handle.submit_blocking(
            RequestOp::OctetWrite {
                data: msg.as_bytes().to_vec(),
            },
            self.user(timeout),
        )?;
        Ok(())
    }

    /// Send one command and read frames until the reply that echoes it comes
    /// back (C `mpxWriteRead`). Anything else on the wire is logged and
    /// dropped, which is how the channel recovers from a desynchronised
    /// server.
    fn transact(
        &self,
        kind: MpxKind,
        name: &str,
        value: Option<&str>,
        timeout: Duration,
    ) -> Result<Vec<u8>, ChannelError> {
        let msg = protocol::encode(kind, name, value)?;
        log::debug!("merlin -> {msg}");
        self.write(&msg, timeout)?;

        loop {
            let body = self.read_frame(timeout)?;
            if protocol::response_echoes(&body, kind, name) {
                log::debug!("merlin <- {}", String::from_utf8_lossy(&body));
                return Ok(body);
            }
            log::error!(
                "merlin: unexpected response to {} {name}: '{}'",
                kind.as_str(),
                String::from_utf8_lossy(&body)
            );
        }
    }

    /// `SET <name> <value>`.
    pub fn set(&self, name: &str, value: &str, timeout: Duration) -> Result<(), ChannelError> {
        let body = self.transact(MpxKind::Set, name, Some(value), timeout)?;
        protocol::decode_ack(&body)?;
        Ok(())
    }

    /// `GET <name>`, returning the device's value field.
    pub fn get(&self, name: &str, timeout: Duration) -> Result<String, ChannelError> {
        let body = self.transact(MpxKind::Get, name, None, timeout)?;
        Ok(protocol::decode_get(&body)?)
    }

    /// `CMD <name>`.
    pub fn command(&self, name: &str, timeout: Duration) -> Result<(), ChannelError> {
        let body = self.transact(MpxKind::Cmd, name, None, timeout)?;
        protocol::decode_ack(&body)?;
        Ok(())
    }

    /// `GET <name>` parsed as a float, or `None` if the device refused.
    pub fn get_f64(&self, name: &str, timeout: Duration) -> Option<f64> {
        match self.get(name, timeout) {
            Ok(v) => v.trim().parse().ok(),
            Err(e) => {
                log::error!("merlin: GET {name} failed: {e}");
                None
            }
        }
    }

    /// `GET <name>` parsed as an integer, or `None` if the device refused.
    pub fn get_i32(&self, name: &str, timeout: Duration) -> Option<i32> {
        match self.get(name, timeout) {
            Ok(v) => v.trim().parse().ok(),
            Err(e) => {
                log::error!("merlin: GET {name} failed: {e}");
                None
            }
        }
    }
}

/// The command timeout every parameter write uses (C
/// `Labview_DEFAULT_TIMEOUT`).
pub fn default_timeout() -> Duration {
    Duration::from_secs_f64(LABVIEW_DEFAULT_TIMEOUT_SEC)
}

#[cfg(test)]
mod is_timeout_tests {
    use super::*;
    use epics_rs::asyn::interpose::{EomReason, PartialOctetRead};

    /// A read that times out *after* transferring bytes arrives as
    /// `AsynError::PartialRead` when the EOS interpose is installed (this
    /// port's st.cmd uses `noProcessEos=0`). A bare `AsynError::Status`
    /// match missed it and misclassified the timeout as a transport fault.
    #[test]
    fn a_partial_read_wrapped_timeout_maps_to_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
        .with_partial_read(PartialOctetRead {
            data: b"partial".to_vec(),
            eom_reason: EomReason::empty(),
        });
        assert!(matches!(ChannelError::from(e), ChannelError::Timeout));
    }

    #[test]
    fn a_real_error_maps_to_transport() {
        let e = AsynError::Status {
            status: AsynStatus::Error,
            message: "boom".into(),
        };
        assert!(matches!(ChannelError::from(e), ChannelError::Transport(_)));
    }
}
