//! The blocking command channel to the detector's asyn IP port
//! (C `writeReadMeter`, mythen.cpp:239).
//!
//! The IP port itself is created by `drvAsynIPPortConfigure` in st.cmd, exactly
//! as in C, and the output EOS the detector needs is set there with
//! `asynOctetSetOutputEos` — the driver sends the bare command text.

use std::num::NonZeroUsize;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;

use crate::protocol::{INT_REPLY_LEN, VERSION_REPLY_LEN, decode_f32, decode_i32, decode_version};

/// The command timeout (C `M1K_TIMEOUT`).
pub const M1K_TIMEOUT: Duration = Duration::from_secs(5);

/// asyn subaddress of the octet port (single device).
const ADDR: i32 = 0;

/// One command/reply transaction with the detector.
///
/// Every method takes `&self`; the driver owns exactly one `Transport` behind a
/// mutex, which is what makes a write/read pair atomic against the acquisition
/// task issuing a readout at the same time (C gets this from asyn's per-port
/// request queue).
pub struct Transport {
    handle: PortHandle,
}

impl Transport {
    pub fn new(handle: PortHandle) -> Self {
        Self { handle }
    }

    /// Send `command` and read up to `expect` bytes back.
    ///
    /// Returns the bytes that arrived, which may be fewer than `expect` if the
    /// detector went quiet — the caller decides whether a short reply is fatal
    /// (C compares `nread` with `nread_expect`). A reply is accumulated across
    /// socket reads: TCP may split a 5 KB readout over several segments, and a
    /// binary protocol with no input EOS has no other way to know it is done.
    pub fn write_read(
        &self,
        command: &[u8],
        expect: usize,
        timeout: Duration,
    ) -> AsynResult<Vec<u8>> {
        let io = SyncIOHandle::from_handle(self.handle.clone(), ADDR, timeout);
        io.write_octet(0, command)?;

        let deadline = Instant::now() + timeout;
        let mut reply = Vec::with_capacity(expect);
        while reply.len() < expect {
            let remaining = expect - reply.len();
            match io.read_octet(0, remaining) {
                Ok(bytes) if bytes.is_empty() => break,
                Ok(bytes) => reply.extend_from_slice(&bytes),
                // A timeout with something already in hand is a short reply,
                // which the caller has to see; with nothing in hand it is the
                // detector not answering at all.
                Err(e) if is_timeout(&e) => {
                    if reply.is_empty() {
                        return Err(e);
                    }
                    break;
                }
                Err(e) => return Err(e),
            }
            if Instant::now() >= deadline {
                break;
            }
        }
        Ok(reply)
    }

    /// Send a command whose reply is a status integer, and check it
    /// (C `sendCommand`, mythen.cpp:221).
    pub fn send_command(&self, command: &str) -> AsynResult<i32> {
        let reply = self.write_read(command.as_bytes(), INT_REPLY_LEN, M1K_TIMEOUT)?;
        let value = decode_i32(&reply).ok_or_else(|| short_reply(command, reply.len()))?;
        if value < 0 {
            log::error!("mythen: [{command}] error, expected 0, received {value}");
        }
        Ok(value)
    }

    /// `-get <name>` returning a 4-byte integer.
    pub fn get_int(&self, command: &str) -> AsynResult<i32> {
        let reply = self.write_read(command.as_bytes(), INT_REPLY_LEN, M1K_TIMEOUT)?;
        decode_i32(&reply).ok_or_else(|| short_reply(command, reply.len()))
    }

    /// `-get tau` returning a 4-byte float.
    pub fn get_float(&self, command: &str) -> AsynResult<f32> {
        let reply = self.write_read(command.as_bytes(), INT_REPLY_LEN, M1K_TIMEOUT)?;
        decode_f32(&reply).ok_or_else(|| short_reply(command, reply.len()))
    }

    /// `-get version` returning the 7-byte firmware string.
    pub fn get_version(&self) -> AsynResult<String> {
        let reply = self.write_read(b"-get version", VERSION_REPLY_LEN, M1K_TIMEOUT)?;
        if reply.is_empty() {
            return Err(short_reply("-get version", 0));
        }
        Ok(decode_version(&reply))
    }

    /// A readout command, whose reply is `expect` bytes of binary data.
    ///
    /// The only length on this port that is computed rather than a protocol
    /// constant, so it is the only one that can come out zero — and
    /// [`Transport::write_read`] writes before it reads, so a zero would leave
    /// the detector's reply in the socket for the next command to misread.
    /// [`NonZeroUsize`] is what makes that unrepresentable.
    pub fn readout(
        &self,
        command: &str,
        expect: NonZeroUsize,
        timeout: Duration,
    ) -> AsynResult<Vec<u8>> {
        self.write_read(command.as_bytes(), expect.get(), timeout)
    }
}

fn is_timeout(e: &AsynError) -> bool {
    matches!(
        e,
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        }
    )
}

fn short_reply(command: &str, got: usize) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("mythen: [{command}] short reply: {got} bytes"),
    }
}
