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
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
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

impl ChannelError {
    /// The `asynStatus` C prints for this failure.
    pub(crate) fn code(&self) -> i32 {
        match self {
            Self::Timeout => AsynStatus::Timeout as i32,
            Self::Transport(e) => e.status() as i32,
            Self::Protocol(_) => AsynStatus::Error as i32,
        }
    }
}

/// What C's `mpxSet`/`mpxCommand` return to `writeInt32`/`writeFloat64`.
impl From<ChannelError> for AsynError {
    fn from(e: ChannelError) -> Self {
        match e {
            ChannelError::Transport(e) => e,
            ChannelError::Timeout => AsynError::Status {
                status: AsynStatus::Timeout,
                message: "timeout".into(),
            },
            ChannelError::Protocol(e) => AsynError::Status {
                status: AsynStatus::Error,
                message: e.to_string(),
            },
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
    io: SyncIOHandle,
    max_frame: usize,
}

impl MpxConnection {
    pub fn new(io: SyncIOHandle, max_frame: usize) -> Self {
        Self { io, max_frame }
    }

    fn user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(0).with_addr(0).with_timeout(timeout)
    }

    /// C prints these on the asynUser of the socket being read or written.
    pub(crate) fn trace(&self, msg: &str) {
        self.io.print(0, TraceMask::ERROR, format_args!("{msg}"));
    }

    /// Read exactly `count` more bytes into `out`. `Err(None)` is C's read
    /// that succeeded with zero bytes: the peer closed the socket.
    fn read_exact(
        &self,
        out: &mut Vec<u8>,
        count: usize,
        timeout: Duration,
    ) -> Result<(), Option<ChannelError>> {
        let want = out.len() + count;
        while out.len() < want {
            // OctetReadBinary brackets the read with the input EOS cleared, so
            // a 0x0A byte inside a pixel payload cannot terminate it early —
            // MPX frames are length-delimited, never EOS-delimited.
            let result = self
                .io
                .port()
                .and_then(|handle| {
                    handle.submit_blocking(
                        RequestOp::OctetReadBinary {
                            buf_size: want - out.len(),
                        },
                        self.user(timeout),
                    )
                })
                .map_err(|e| Some(e.into()))?;
            let chunk = result.data.unwrap_or_default();
            if chunk.is_empty() {
                return Err(None);
            }
            out.extend_from_slice(&chunk);
        }
        Ok(())
    }

    /// Read one MPX frame, discarding any bytes that precede the `MPX` magic
    /// so the channel re-syncs after an error or a server restart. `header`
    /// receives the frame header once it is complete (C `fromLabviewHeader`).
    ///
    /// C matched the magic with a counter it reset to zero on every mismatch,
    /// so junk ending in a prefix of the magic (`"...MMPX"`) desynchronised it
    /// permanently. This slides the window by one byte instead.
    fn read_frame_into(
        &self,
        header: &mut String,
        timeout: Duration,
    ) -> Result<Vec<u8>, ChannelError> {
        let timeout_s = timeout.as_secs_f64();
        let closed = || {
            ChannelError::Transport(AsynError::Status {
                status: AsynStatus::Disconnected,
                message: "peer closed the MPX socket".into(),
            })
        };
        // C prints `fromLabview`, which `mpxReadCmd` clears before every read
        // and which this connection never fills in between.
        let mut window = Vec::with_capacity(MPX_FRAME_HEADER_LEN);
        self.read_exact(&mut window, MPX_HEADER.len(), timeout)
            .map_err(|e| e.unwrap_or_else(closed))?;
        let mut junk = 0usize;
        while window != MPX_HEADER {
            window.remove(0);
            self.read_exact(&mut window, 1, timeout)
                .map_err(|e| e.unwrap_or_else(closed))?;
            junk += 1;
        }
        if junk > 0 {
            self.trace(&format!(
                "merlinDetector:mpxRead, status=0 {junk} bytes of leading garbage discarded before header  "
            ));
        }

        let rest = MPX_FRAME_HEADER_LEN - MPX_HEADER.len();
        match self.read_exact(&mut window, rest, timeout) {
            Ok(()) => {}
            Err(None) => {
                self.trace("merlinDetector:mpxRead, Header too short");
                return Err(closed());
            }
            Err(Some(e)) => {
                self.trace(&format!(
                    "merlinDetector:mpxRead, timeout={timeout_s:.6}, status={} received {} bytes",
                    e.code(),
                    window.len() - MPX_HEADER.len()
                ));
                return Err(e);
            }
        }
        *header = String::from_utf8_lossy(&window).into_owned();
        let body_len = match protocol::parse_frame_header(&window, self.max_frame) {
            Ok(n) => n,
            Err(e) => {
                if let MpxError::BadBodySize(n) = e {
                    self.trace(&format!(
                        "merlinDetector:mpxRead, frame size {n} not supported"
                    ));
                }
                return Err(e.into());
            }
        };

        // C returned asynSuccess on a short body read (`status = asynSuccess ?
        // asynError : status` always took the else branch), handing the caller
        // a truncated frame it believed was complete. This fails it instead.
        let mut body = Vec::with_capacity(body_len);
        if let Err(e) = self.read_exact(&mut body, body_len, timeout) {
            self.trace(&format!(
                "merlinDetector:mpxRead, timeout={timeout_s:.6}, status={} received {} bytes in MPX command body, expected {body_len}",
                e.as_ref().map_or(0, ChannelError::code),
                body.len()
            ));
            return Err(e.unwrap_or_else(closed));
        }
        Ok(body)
    }

    pub fn read_frame(&self, timeout: Duration) -> Result<Vec<u8>, ChannelError> {
        self.read_frame_into(&mut String::new(), timeout)
    }

    /// Send one command and read frames until the reply that echoes it comes
    /// back (C `mpxWriteRead`). Anything else on the wire is reported and
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
        // C `mpxWrite` reports a failed write and still reads the reply.
        if let Err(e) = self.io.port().and_then(|handle| {
            handle.submit_blocking(
                RequestOp::OctetWrite {
                    data: msg.as_bytes().to_vec(),
                },
                self.user(timeout),
            )
        }) {
            self.trace(&format!(
                "merlinDetector:mpxWrite, status={}, sent\n{msg}",
                ChannelError::from(e).code()
            ));
        }

        loop {
            let mut header = String::new();
            let read = self.read_frame_into(&mut header, timeout);
            let (code, body) = match &read {
                Ok(body) if protocol::response_echoes(body, kind, name) => break read,
                Ok(body) => (0, String::from_utf8_lossy(body).into_owned()),
                Err(e) => (e.code(), String::new()),
            };
            self.trace(&format!(
                "merlinDetector:mpxReadCmd error, status={code} unexpected response from labview: '{header}{body}'"
            ));
            read?;
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
            Err(_) => None,
        }
    }

    /// `GET <name>` parsed as an integer, or `None` if the device refused.
    pub fn get_i32(&self, name: &str, timeout: Duration) -> Option<i32> {
        match self.get(name, timeout) {
            Ok(v) => v.trim().parse().ok(),
            Err(_) => None,
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
