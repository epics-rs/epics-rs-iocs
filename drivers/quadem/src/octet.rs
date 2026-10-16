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
//! EOS-terminated, while [`OctetIo::read_binary`] suppresses the EOS for the
//! duration of the read (`RequestOp::OctetReadBinary`). The bytes on the wire and the bytes
//! delivered to the parser are identical; only the ownership of the EOS state
//! changes, from shared mutable to per-call.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::request::{PortCalls, RequestOp};
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

impl ReadOutcome {
    pub fn as_str(&self) -> String {
        String::from_utf8_lossy(&self.data).trim_end().to_string()
    }
}

/// Blocking octet handle at one timeout: the `asynUser` that
/// `pasynOctetSyncIO->connect` returns.
#[derive(Clone)]
pub struct OctetIo {
    /// `None` is the user a failed connect still hands back, bound to no port:
    /// C drivers keep it, and every later call on it fails in asynManager.
    handle: Option<PortHandle>,
    timeout: Duration,
}

impl OctetIo {
    pub fn new(handle: PortHandle, timeout: Duration) -> Self {
        Self {
            handle: Some(handle),
            timeout,
        }
    }

    /// The user [`connect_octet`] leaves when the port does not exist.
    pub fn unbound(timeout: Duration) -> Self {
        Self {
            handle: None,
            timeout,
        }
    }

    /// The port, or `queueLockPort`'s refusal of an unbound user
    /// (asynManager.c:1843-1846), which every SyncIO call meets first.
    fn port(&self) -> AsynResult<&PortHandle> {
        self.handle.as_ref().ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "asynManager::queueLockPort not connected".into(),
        })
    }

    /// The bound port, for the asynCommon operations (`connect`,
    /// `disconnect`) that have no `asynOctetSyncIO` wrapper.
    pub fn handle(&self) -> Option<&PortHandle> {
        self.handle.as_ref()
    }

    /// The same port at another timeout — `drvNSLS_EM` reads the discovery
    /// datagrams with a shorter one than it uses for commands.
    pub fn with_timeout(&self, timeout: Duration) -> Self {
        Self {
            handle: self.handle.clone(),
            timeout,
        }
    }

    /// `asynOctetSyncIO->write` of raw bytes.
    pub fn write_bytes(&self, out: &[u8]) -> AsynResult<usize> {
        let result = self
            .port()?
            .submit_blocking(RequestOp::OctetWrite { data: out.to_vec() }, self.user())?;
        Ok(result.nbytes)
    }

    fn user(&self) -> AsynUser {
        octet_user(self.timeout)
    }

    /// `pasynManager->lockPort` … `unlockPort` around direct `pasynOctet`
    /// calls, as the C++ read threads do: no connect gate, so a down link
    /// answers with the port's own error, and the calls in `f` run back to back.
    pub fn locked<R: Send + 'static>(
        &self,
        f: impl FnOnce(&mut LockedOctet<'_>) -> R + Send + 'static,
    ) -> AsynResult<R> {
        let timeout = self.timeout;
        // asynManager.c:1783-1786, the unbound user's refusal.
        let port = self.handle.as_ref().ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "asynManager::lockPort not connected\n".into(),
        })?;
        port.lock_port_blocking(move |calls| f(&mut LockedOctet { calls, timeout }))
    }

    /// `asynOctetSyncIO->writeRead`: flush stale input, write, read one
    /// EOS-terminated response.
    pub fn write_read(&self, out: &str, buf_size: usize) -> AsynResult<String> {
        let result = self.port()?.submit_blocking(
            RequestOp::OctetWriteRead {
                data: out.as_bytes().to_vec(),
                buf_size,
                flush: true,
            },
            self.user(),
        )?;
        let data = result.data.unwrap_or_default();
        Ok(String::from_utf8_lossy(&data).trim_end().to_string())
    }

    /// `asynOctetSyncIO->write`.
    pub fn write(&self, out: &str) -> AsynResult<usize> {
        let result = self.port()?.submit_blocking(
            RequestOp::OctetWrite {
                data: out.as_bytes().to_vec(),
            },
            self.user(),
        )?;
        Ok(result.nbytes)
    }

    /// `asynOctetSyncIO->read` with the port's input EOS active.
    pub fn read_line(&self, buf_size: usize) -> AsynResult<ReadOutcome> {
        let result = self
            .port()?
            .submit_blocking(RequestOp::OctetRead { buf_size }, self.user())?;
        Ok(ReadOutcome {
            data: result.data.unwrap_or_default(),
            eom: EomReason::from_bits_truncate(result.eom_reason),
        })
    }

    /// Count-terminated read with the input EOS suppressed — the binary data
    /// path. Equivalent to C++'s `setInputEos("", 0)` followed by
    /// `pasynOctet->read(..., nRequested, ...)`, but the suppression is scoped
    /// to this call.
    pub fn read_binary(&self, buf_size: usize) -> AsynResult<ReadOutcome> {
        let result = self
            .port()?
            .submit_blocking(RequestOp::OctetReadBinary { buf_size }, self.user())?;
        Ok(ReadOutcome {
            data: result.data.unwrap_or_default(),
            eom: EomReason::from_bits_truncate(result.eom_reason),
        })
    }

    /// One `pasynOctet->read` under [`Self::locked`], EOS-terminated.
    pub fn locked_read_line(&self, buf_size: usize) -> AsynResult<ReadOutcome> {
        self.locked(move |port| port.read_line(buf_size))?
    }

    /// One `pasynOctet->read` under [`Self::locked`], EOS suppressed.
    pub fn locked_read_binary(&self, buf_size: usize) -> AsynResult<ReadOutcome> {
        self.locked(move |port| port.read_binary(buf_size))?
    }

    /// `pasynOctetSyncIO->setInputEos`. C's SyncIO EOS setters take
    /// `lockPort`, not `queueLockPort`, so they succeed on a port whose device
    /// is down — the constructor runs before the meter is reachable.
    pub fn set_input_eos(&self, eos: &[u8]) -> AsynResult<()> {
        let eos = eos.to_vec();
        self.locked(move |port| port.set_eos(RequestOp::SetInputEos { eos }))?
    }

    /// `pasynOctetSyncIO->setOutputEos`.
    pub fn set_output_eos(&self, eos: &[u8]) -> AsynResult<()> {
        let eos = eos.to_vec();
        self.locked(move |port| port.set_eos(RequestOp::SetOutputEos { eos }))?
    }

    /// `asynOctetSyncIO->flush`.
    pub fn flush(&self) -> AsynResult<()> {
        self.port()?
            .submit_blocking(RequestOp::Flush, self.user())?;
        Ok(())
    }
}

/// What C++ prints as `inString_` after a `writeReadMeter`: the reply, or on a
/// failure whatever the read did transfer.
pub fn reply_text(reply: AsynResult<String>) -> String {
    match reply {
        Ok(text) => text,
        Err(e) => e
            .partial_read()
            .map(|p| String::from_utf8_lossy(&p.data).trim_end().to_string())
            .unwrap_or_default(),
    }
}

fn octet_user(timeout: Duration) -> AsynUser {
    AsynUser::new(OCTET_REASON)
        .with_addr(OCTET_ADDR)
        .with_timeout(timeout)
}

/// The octet interface under [`OctetIo::locked`].
pub struct LockedOctet<'a> {
    calls: &'a mut dyn PortCalls,
    timeout: Duration,
}

impl LockedOctet<'_> {
    fn read(&mut self, op: RequestOp) -> AsynResult<ReadOutcome> {
        let result = self.calls.call(&op, octet_user(self.timeout))?;
        Ok(ReadOutcome {
            data: result.data.unwrap_or_default(),
            eom: EomReason::from_bits_truncate(result.eom_reason),
        })
    }

    /// `pasynOctet->read` with the port's input EOS active.
    pub fn read_line(&mut self, buf_size: usize) -> AsynResult<ReadOutcome> {
        self.read(RequestOp::OctetRead { buf_size })
    }

    /// `pasynOctet->read` with the input EOS suppressed; see
    /// [`OctetIo::read_binary`].
    pub fn read_binary(&mut self, buf_size: usize) -> AsynResult<ReadOutcome> {
        self.read(RequestOp::OctetReadBinary { buf_size })
    }

    /// `pasynOctet->flush`.
    pub fn flush(&mut self) -> AsynResult<()> {
        self.calls
            .call(&RequestOp::Flush, octet_user(self.timeout))
            .map(drop)
    }

    fn set_eos(&mut self, op: RequestOp) -> AsynResult<()> {
        self.calls.call(&op, octet_user(self.timeout)).map(drop)
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
    let (user, status) = SyncIOHandle::connect_port(port_name, OCTET_ADDR, timeout);
    let io = match user.port() {
        Ok(port) => OctetIo::new(port.clone(), timeout),
        Err(_) => OctetIo::unbound(timeout),
    };
    (io, status)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
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
        let driver = NoInputEos(PortDriverBase::new(name, 1, PortFlags::default()));
        let claim = claim_port(name).unwrap();
        let (runtime, _actor) =
            create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(runtime.port_handle().clone(), &InterruptGate::opened())
            .unwrap();
        let io = OctetIo::new(runtime.port_handle().clone(), Duration::from_millis(100));
        (io, runtime)
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
        let err = io.locked_read_line(64).unwrap_err();
        assert_eq!(err.status(), AsynStatus::Error, "{err:?}");
        assert!(
            err.to_string().contains("127.0.0.1:1 disconnected:"),
            "{err}"
        );
    }
}
