//! The blocking command channel to the detector's asyn IP port
//! (C `writeReadMeter`, mythen.cpp:239).
//!
//! The IP port itself is created by `drvAsynIPPortConfigure` in st.cmd, exactly
//! as in C, and the output EOS the detector needs is set there with
//! `asynOctetSetOutputEos` — the driver sends the bare command text.

use std::num::NonZeroUsize;
use std::sync::Arc;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceManager;

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

    pub fn trace(&self) -> Arc<TraceManager> {
        self.handle.trace().clone()
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
        let deadline = Instant::now() + timeout;
        let mut reply = Vec::with_capacity(expect);
        // C's writeRead flushes, writes and reads once; the later reads gather
        // the rest of a split reply.
        let mut read = io.write_read(0, command, expect);
        loop {
            match read.into_parts() {
                (bytes, Ok(())) if bytes.is_empty() => break,
                (bytes, Ok(())) => reply.extend_from_slice(&bytes),
                // A timeout with something already in hand is a short reply,
                // which the caller has to see; with nothing in hand it is the
                // detector not answering at all.
                (bytes, Err(e)) if e.status() == AsynStatus::Timeout => {
                    reply.extend_from_slice(&bytes);
                    if reply.is_empty() {
                        return Err(e);
                    }
                    break;
                }
                (_, Err(e)) => return Err(e),
            }
            if reply.len() >= expect || Instant::now() >= deadline {
                break;
            }
            read = io.read_octet(0, expect - reply.len());
        }
        Ok(reply)
    }

    /// Send a command whose reply is a status integer
    /// (C `sendCommand`, mythen.cpp:221).
    pub fn send_command(&self, command: &str) -> AsynResult<i32> {
        let reply = self.write_read(command.as_bytes(), INT_REPLY_LEN, M1K_TIMEOUT)?;
        decode_i32(&reply).ok_or_else(|| short_reply(command, reply.len()))
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

fn short_reply(command: &str, got: usize) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("mythen: [{command}] short reply: {got} bytes"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;

    /// An octet port whose every read times out after delivering `abc`.
    struct PartialThenTimeout(PortDriverBase);

    impl PortDriver for PartialThenTimeout {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            buf[..3].copy_from_slice(b"abc");
            OctetReadResult {
                status: Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "read timeout".into(),
                }),
                ..OctetReadResult::ok(3, EomReason::empty())
            }
        }
    }

    fn partial_then_timeout_port(name: &str) -> PortRuntimeHandle {
        let claim = claim_port(name).unwrap();
        let (runtime, _actor) = create_port_runtime(
            &claim,
            PartialThenTimeout(PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            )),
            RuntimeConfig::default(),
        )
        .unwrap();
        runtime
    }

    /// A timeout after part of a reply is a short reply the caller sees
    /// (mythen.cpp compares `nread` with `nread_expect`), not a failure.
    #[test]
    fn a_timeout_after_part_of_a_reply_returns_the_part() {
        let runtime = partial_then_timeout_port("MYTHEN_PARTIAL_TIMEOUT");
        let transport = Transport::new(runtime.port_handle().clone());
        let reply = transport
            .write_read(b"-get version", 7, Duration::from_millis(100))
            .unwrap();
        assert_eq!(reply, b"abc");
    }

    /// An octet port logging each flush, write and read; reads answer `1234`.
    struct Logged(PortDriverBase, Arc<parking_lot::Mutex<Vec<&'static str>>>);

    impl PortDriver for Logged {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_flush(&mut self, _user: &mut AsynUser) -> AsynResult<()> {
            self.1.lock().push("flush");
            Ok(())
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.1.lock().push("write");
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            self.1.lock().push("read");
            let n = buf.len().min(4);
            buf[..n].copy_from_slice(&b"1234"[..n]);
            OctetReadResult::ok(n, EomReason::empty())
        }
    }

    /// `writeReadMeter` is `pasynOctetSyncIO->writeRead` (mythen.cpp:248-259),
    /// which flushes the stale input before writing (asynOctetSyncIO.c:250).
    #[test]
    fn a_command_flushes_before_it_writes() {
        let name = "MYTHEN_FLUSH";
        let log = Arc::new(parking_lot::Mutex::new(Vec::new()));
        let (runtime, _actor) = create_port_runtime(
            &claim_port(name).unwrap(),
            Logged(
                PortDriverBase::new(
                    name,
                    1,
                    PortFlags::default(),
                    InterfaceMask::UNMIGRATED_ALL,
                    InterfaceMask::UNMIGRATED_INTERRUPTS,
                ),
                log.clone(),
            ),
            RuntimeConfig::default(),
        )
        .unwrap();
        let transport = Transport::new(runtime.port_handle().clone());
        assert_eq!(transport.send_command("-start").unwrap(), 0x3433_3231);
        assert_eq!(*log.lock(), ["flush", "write", "read"]);
    }
}
