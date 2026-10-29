//! The BIS command socket (C `writeBIS`).
//!
//! # Ownership
//!
//! The port actor is the only thing that sends a command. C sent the scan
//! command from the acquisition thread instead, and had to take the port lock
//! to do it; here the acquisition task asks the actor for a scan through a
//! parameter, so the socket has one owner and needs no lock at all.

use std::time::Duration;

use epics_rs::asyn::error::{AsynError, OctetStep};
use epics_rs::asyn::sync_io::SyncIOHandle;

use crate::types::MAX_MESSAGE_SIZE;

/// How C's `pasynOctetSyncIO->writeRead` in `writeBIS` left `fromBIS`.
pub enum Exchange {
    /// It failed before the read — not connected, flush or write — so
    /// `fromBIS` keeps the previous reply.
    NotSent(AsynError),
    /// The read ran and `fromBIS` holds what it received: the reply, or the
    /// bytes before a timeout or error (normally none).
    Read {
        reply: String,
        status: Result<(), AsynError>,
    },
}

/// C posts `fromBIS` up to its first NUL, whitespace included
/// (BISDetector.cpp:325).
pub(crate) fn reply_text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

/// One `pasynOctetSyncIO->writeRead` of `command` (BISDetector.cpp:316-319).
pub(crate) fn exchange(io: &SyncIOHandle, command: &str, timeout: Duration) -> Exchange {
    let reply = io
        .with_timeout(timeout)
        .write_read(0, command.as_bytes(), MAX_MESSAGE_SIZE);
    match reply.into_parts() {
        (reply, Ok(())) => Exchange::Read {
            reply: reply_text(&reply),
            status: Ok(()),
        },
        (reply, Err(e)) if e.octet_step() == Some(OctetStep::Read) => Exchange::Read {
            reply: reply_text(&reply),
            status: Err(e),
        },
        (_, Err(e)) => Exchange::NotSent(e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_reply_is_cut_at_its_first_nul_and_keeps_its_whitespace() {
        assert_eq!(reply_text(b"[ACK] \r\n\0old"), "[ACK] \r\n");
    }
}
