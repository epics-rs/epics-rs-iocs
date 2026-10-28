//! Blocking octet client, the `asynOctetSyncIO` surface the quadEM device
//! drivers use.
//!
//! # Input EOS
//!
//! C++ sets the port's input EOS to `"\r\n"` at the top of every setAcquire
//! (`drvTetrAMM.cpp:465`) and to `""` before a binary data read (:523). That
//! makes the port's EOS mean two different things depending on which thread
//! last wrote it, and the read thread and the command path race for it.
//!
//! This port only ever writes `"\r\n"` to that cell — at construction and at
//! the top of every setAcquire, as C does — and selects the framing per
//! operation: [`OctetIo::write_read`] and [`OctetIo::read_line`] are
//! EOS-terminated, while the read thread's [`LockedOctet::read_binary`]
//! suppresses the EOS for the duration of the read
//! (`RequestOp::OctetReadBinary`). The bytes on the wire and the bytes
//! delivered to the parser are identical; only the ownership of the EOS state
//! changes, from shared mutable to per-call.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::request::{OctetRead, OctetReadOp, PortCalls, RequestOp};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;

/// asyn subaddress of a single-device octet port.
const OCTET_ADDR: i32 = 0;
/// asyn `reason` for raw octet I/O.
const OCTET_REASON: usize = 0;

/// Outcome of a read: the bytes plus why the read ended.
#[derive(Debug, Clone)]
pub struct ReadOutcome {
    pub data: Vec<u8>,
    pub eom: EomReason,
}

/// A read's bytes and end reason beside its status, which C reads together
/// (`nRead`, `eomReason`, the returned `asynStatus`).
pub fn outcome(read: OctetRead) -> (ReadOutcome, AsynResult<()>) {
    let eom = read.eom();
    let (data, status) = read.into_parts();
    (ReadOutcome { data, eom }, status)
}

impl ReadOutcome {
    pub fn as_str(&self) -> String {
        reply_text(&self.data)
    }
}

/// Blocking octet handle at one timeout: the `asynUser` that
/// `pasynOctetSyncIO->connect` returns.
pub struct OctetIo {
    /// A user a failed connect still hands back is bound to no port: C drivers
    /// keep it, and every later call on it fails in asynManager.
    io: SyncIOHandle,
}

impl Clone for OctetIo {
    fn clone(&self) -> Self {
        let io = match self.io.timeout() {
            Some(timeout) => self.io.with_timeout(timeout),
            None => self.io.waiting_forever(),
        };
        Self { io }
    }
}

impl OctetIo {
    pub fn new(handle: PortHandle, timeout: Duration) -> Self {
        Self {
            io: SyncIOHandle::from_handle(handle, OCTET_ADDR, timeout),
        }
    }

    /// The user a failed connect leaves: `pasynOctetSyncIO->connect` to no
    /// port.
    pub fn unbound(timeout: Duration) -> Self {
        let (io, _) = SyncIOHandle::connect_port("", OCTET_ADDR, timeout, InterfaceType::Octet);
        Self { io }
    }

    /// The bound port, for the asynCommon operations (`connect`,
    /// `disconnect`) that have no `asynOctetSyncIO` wrapper.
    pub fn handle(&self) -> Option<&PortHandle> {
        self.io.port().ok()
    }

    /// The same port at another timeout — `drvNSLS_EM` reads the discovery
    /// datagrams with a shorter one than it uses for commands.
    pub fn with_timeout(&self, timeout: Duration) -> Self {
        Self {
            io: self.io.with_timeout(timeout),
        }
    }

    /// `asynOctetSyncIO->write` of raw bytes.
    pub fn write_bytes(&self, out: &[u8]) -> AsynResult<usize> {
        self.io.write_octet(OCTET_REASON, out)
    }

    /// `pasynManager->lockPort` … `unlockPort` around direct `pasynOctet`
    /// calls, as the C++ read threads do: no connect gate, so a down link
    /// answers with the port's own error, and the calls in `f` run back to back.
    pub fn locked<R: Send + 'static>(
        &self,
        f: impl FnOnce(&mut LockedOctet<'_>) -> R + Send + 'static,
    ) -> AsynResult<R> {
        let timeout = self.io.timeout();
        // asynManager.c:1783-1786, the unbound user's refusal.
        let port = self.handle().ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "asynManager::lockPort not connected\n".into(),
        })?;
        port.lock_port_blocking(octet_user(timeout), move |calls| {
            f(&mut LockedOctet { calls, timeout })
        })
    }

    /// `asynOctetSyncIO->writeRead`: flush stale input, write, read one
    /// EOS-terminated response.
    pub fn write_read(&self, out: &str, buf_size: usize) -> OctetRead {
        self.io.write_read(OCTET_REASON, out.as_bytes(), buf_size)
    }

    /// `asynOctetSyncIO->write`.
    pub fn write(&self, out: &str) -> AsynResult<usize> {
        self.write_bytes(out.as_bytes())
    }

    /// `asynOctetSyncIO->read` with the port's input EOS active.
    pub fn read_line(&self, buf_size: usize) -> OctetRead {
        self.io.read_octet(OCTET_REASON, buf_size)
    }

    /// `asynOctetSyncIO->read` of a binary frame or datagram. C takes the
    /// port's input EOS here too, which these ports leave empty.
    pub fn read_binary(&self, buf_size: usize) -> OctetRead {
        self.read_line(buf_size)
    }

    /// One `pasynOctet->read` under [`Self::locked`], EOS-terminated.
    pub fn locked_read_line(&self, buf_size: usize) -> OctetRead {
        self.locked(move |port| port.read_line(buf_size))
            .unwrap_or_else(OctetRead::failed)
    }

    /// One `pasynOctet->read` under [`Self::locked`], EOS suppressed.
    pub fn locked_read_binary(&self, buf_size: usize) -> OctetRead {
        self.locked(move |port| port.read_binary(buf_size))
            .unwrap_or_else(OctetRead::failed)
    }

    /// `pasynOctetSyncIO->setInputEos`. C's SyncIO EOS setters take
    /// `lockPort`, not `queueLockPort`, so they succeed on a port whose device
    /// is down — the constructor runs before the meter is reachable.
    pub fn set_input_eos(&self, eos: &[u8]) -> AsynResult<()> {
        self.io.set_input_eos(OCTET_REASON, eos)
    }

    /// `pasynOctetSyncIO->setOutputEos`.
    pub fn set_output_eos(&self, eos: &[u8]) -> AsynResult<()> {
        self.io.set_output_eos(OCTET_REASON, eos)
    }

    /// `asynOctetSyncIO->flush`.
    pub fn flush(&self) -> AsynResult<()> {
        self.io.flush(OCTET_REASON)
    }
}

/// A reply as C++ keeps it in `inString_`: whatever the read transferred,
/// trailing whitespace dropped.
pub fn reply_text(data: &[u8]) -> String {
    String::from_utf8_lossy(data).trim_end().to_string()
}

/// `timeout` is the bound user's own, `None` being C's negative "wait
/// forever".
fn octet_user(timeout: Option<Duration>) -> AsynUser {
    AsynUser::new(OCTET_REASON)
        .with_addr(OCTET_ADDR)
        .with_timeout_opt(timeout)
}

/// The octet interface under [`OctetIo::locked`].
pub struct LockedOctet<'a> {
    calls: &'a mut dyn PortCalls,
    timeout: Option<Duration>,
}

impl LockedOctet<'_> {
    /// `pasynOctet->read` with the port's input EOS active.
    pub fn read_line(&mut self, buf_size: usize) -> OctetRead {
        self.calls
            .octet_read(OctetReadOp::Read { buf_size }, octet_user(self.timeout))
    }

    /// `pasynOctet->read` with the input EOS suppressed: C++'s
    /// `setInputEos("", 0)` followed by `pasynOctet->read(..., nRequested,
    /// ...)`, with the suppression scoped to this call.
    pub fn read_binary(&mut self, buf_size: usize) -> OctetRead {
        self.calls.octet_read(
            OctetReadOp::ReadBinary { buf_size },
            octet_user(self.timeout),
        )
    }

    /// `pasynOctet->flush`.
    pub fn flush(&mut self) -> AsynResult<()> {
        self.calls
            .call(&RequestOp::Flush, octet_user(self.timeout))
            .map(drop)
    }
}

/// Build an IP port the way `drvAsynIPPortConfigure` does and wrap it for
/// blocking use.
///
/// `drvNSLS_EM`, `drvT4U_EM` and `drvT4UDirect_EM` all build their own
/// transports in their constructors instead of taking a pre-configured port
/// name, so each of them calls this rather than [`connect_octet`].
///
/// `process_eos` installs the EOS interpose; leave it off for raw datagram or
/// binary-framed reads, where an EOS scan would read past the frame.
///
/// The port is registered under `port_name` before its thread starts, as
/// `drvAsynIPPortConfigure` does (drvAsynIPPort.c:1084-1089).
pub fn create_ip_port(
    port_name: &str,
    host_info: &str,
    timeout: Duration,
    no_auto_connect: bool,
    process_eos: bool,
    owner: &Arc<InterruptGate>,
) -> AsynResult<(OctetIo, PortRuntimeHandle)> {
    let claim = claim_port(port_name).inspect_err(|_| {
        println!("drvAsynIPPortConfigure: Can't register myself.");
    })?;
    let mut driver = epics_rs::asyn::drivers::ip_port::DrvAsynIPPort::new(port_name, host_info)?;
    if no_auto_connect {
        driver.base_mut().init_auto_connect(false);
    }
    if process_eos {
        driver.install_interpose(Box::new(
            epics_rs::asyn::interpose::eos::EosInterpose::default(),
        ));
    }
    let (runtime, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default())?;
    claim.publish(runtime.port_handle().clone(), owner)?;
    let io = OctetIo::new(runtime.port_handle().clone(), timeout);
    Ok((io, runtime))
}

/// `pasynOctetSyncIO->connect` to a pre-configured octet port (created by
/// `drvAsynIPPortConfigure` or `drvAsynSerialPortConfigure`). Like C it always
/// hands back a user — unbound on a missing port, beside `connectDevice`'s
/// refusal (asynManager.c:1339-1342).
pub fn connect_octet(port_name: &str, timeout: Duration) -> (OctetIo, AsynResult<()>) {
    let (io, status) =
        SyncIOHandle::connect_port(port_name, OCTET_ADDR, timeout, InterfaceType::Octet);
    (OctetIo { io }, status)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriverBase, PortFlags};

    /// A registered octet port that refuses every input EOS.
    struct NoInputEos(PortDriverBase);

    impl PortDriver for NoInputEos {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn set_input_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "no input EOS".into(),
            })
        }
    }

    pub(crate) fn eos_refusing_port(name: &str) -> (OctetIo, PortRuntimeHandle) {
        let driver = NoInputEos(PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::OCTET_TRANSPORT,
            InterfaceMask::OCTET,
        ));
        published_port(name, driver)
    }

    /// `driver` registered as port `name`, and a 100 ms user on it.
    pub(crate) fn published_port(
        name: &str,
        driver: impl PortDriver + 'static,
    ) -> (OctetIo, PortRuntimeHandle) {
        let claim = claim_port(name).unwrap();
        let (runtime, _actor) =
            create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(runtime.port_handle().clone(), &InterruptGate::opened())
            .unwrap();
        let io = OctetIo::new(runtime.port_handle().clone(), Duration::from_millis(100));
        (io, runtime)
    }

    /// An octet port that answers every read `pong`.
    struct Pong(PortDriverBase);

    impl PortDriver for Pong {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            buf[..4].copy_from_slice(b"pong");
            epics_rs::asyn::interpose::OctetReadResult::ok(4, EomReason::empty())
        }
        fn set_input_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Ok(())
        }
        fn set_output_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Ok(())
        }
    }

    /// Every `OctetIo` call is its `pasynOctetSyncIO` namesake, which prints
    /// its TRACEIO_DEVICE line (asynOctetSyncIO.c:194-200, :219-223,
    /// :259-266, :291-293, :312, :356).
    #[test]
    fn every_call_prints_its_sync_io_line() {
        use epics_rs::asyn::trace::{TraceFile, TraceIoMask, TraceMask};
        let name = "QE_SYNCIO_LINES";
        let (io, runtime) = published_port(
            name,
            Pong(PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            )),
        );
        let trace = runtime.port_handle().trace();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        trace.set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&path).unwrap(),
            ))),
        );
        trace.set_trace_mask(Some(name), TraceMask::IO_DEVICE);
        trace.set_trace_io_mask(Some(name), TraceIoMask::ASCII);
        io.set_input_eos(b"\r\n").unwrap();
        io.set_output_eos(b"\r").unwrap();
        io.flush().unwrap();
        io.write("w1").unwrap();
        io.write_bytes(b"w2").unwrap();
        assert!(io.write_read("w3", 64).into_parts().1.is_ok());
        assert!(io.read_line(64).into_parts().1.is_ok());
        assert!(io.read_binary(64).into_parts().1.is_ok());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for line in [
            "asynOctetSyncIO setInputEos eoslen 2\n",
            "asynOctetSyncIO setOutputEos eoslen 1\n",
            "asynOctetSyncIO flush\n",
            "asynOctetSyncIO wrote:\nw1\n",
            "asynOctetSyncIO wrote:\nw2\n",
            "asynOctetSyncIO wrote:\nw3\n",
        ] {
            assert!(text.contains(line), "{line:?} not in\n{text}");
        }
        assert_eq!(
            text.matches("asynOctetSyncIO read:\npong\n").count(),
            3,
            "{text}"
        );
    }

    /// The driver constructors set the EOS before the meter is reachable.
    #[test]
    fn eos_setters_succeed_on_a_port_that_is_not_connected() {
        let (io, _runtime) = create_ip_port(
            "QE_EOS_DOWN",
            "127.0.0.1:1",
            Duration::from_millis(200),
            true,
            true,
            &InterruptGate::opened(),
        )
        .unwrap();
        io.set_input_eos(b"\r\n").unwrap();
        io.set_output_eos(b"\r").unwrap();
    }

    /// A read-thread read on a down data link is the IP port's own error, as
    /// C's `pasynOctet->read` under `lockPort` reports it — not the queue's
    /// asynDisconnected.
    #[test]
    fn a_locked_read_on_a_down_link_is_the_ports_own_error() {
        let (io, _runtime) = create_ip_port(
            "QE_LOCKED_DOWN",
            "127.0.0.1:1",
            Duration::from_millis(200),
            true,
            true,
            &InterruptGate::opened(),
        )
        .unwrap();
        let (_, status) = io.locked_read_line(64).into_parts();
        let err = status.unwrap_err();
        assert_eq!(err.status(), AsynStatus::Error, "{err:?}");
        assert!(
            err.to_string().contains("127.0.0.1:1 disconnected:"),
            "{err}"
        );
    }
}
