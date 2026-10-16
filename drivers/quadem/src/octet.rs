//! Blocking octet client, the `asynOctetSyncIO` surface the quadEM device
//! drivers use.
//!
//! # Input EOS
//!
//! C++ sets the port's input EOS once in the driver constructor
//! (`drvTetrAMM.cpp:465`) and then toggles it at runtime: `""` before a binary
//! data read, back to `"\r\n"` on stop. That makes the port's EOS mean two
//! different things depending on which thread last wrote it, and the read
//! thread and the command path race for it.
//!
//! This port keeps the constructor-time [`OctetIo::set_input_eos`] — the only
//! write to that cell — and selects the framing per operation:
//! [`OctetIo::write_read`] and [`OctetIo::read_line`] are EOS-terminated,
//! while [`OctetIo::read_binary`] suppresses the EOS for the duration of the
//! read (`RequestOp::OctetReadBinary`). The bytes on the wire and the bytes
//! delivered to the parser are identical; only the ownership of the EOS state
//! changes, from shared mutable to per-call.

use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{PortCalls, RequestOp};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

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

/// Blocking octet handle bound to one asyn port at one timeout.
#[derive(Clone)]
pub struct OctetIo {
    handle: PortHandle,
    timeout: Duration,
}

impl OctetIo {
    pub fn new(handle: PortHandle, timeout: Duration) -> Self {
        Self { handle, timeout }
    }

    pub fn port_name(&self) -> &str {
        self.handle.port_name()
    }

    /// The underlying port, for the asynCommon operations (`connect`,
    /// `disconnect`, `setOutputEos`) that have no `asynOctetSyncIO` wrapper.
    pub fn handle(&self) -> &PortHandle {
        &self.handle
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
            .handle
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
        self.handle
            .lock_port_blocking(move |calls| f(&mut LockedOctet { calls, timeout }))
    }

    /// `asynOctetSyncIO->writeRead`: flush stale input, write, read one
    /// EOS-terminated response.
    pub fn write_read(&self, out: &str, buf_size: usize) -> AsynResult<String> {
        let result = self.handle.submit_blocking(
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
        let result = self.handle.submit_blocking(
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
            .handle
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
            .handle
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

    /// `pasynOctetSyncIO->setInputEos`. Called once at driver construction;
    /// nothing writes this cell afterwards. C's SyncIO EOS setters take
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
        self.handle.submit_blocking(RequestOp::Flush, self.user())?;
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
        let result = self.calls.call(op, octet_user(self.timeout))?;
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
            .call(RequestOp::Flush, octet_user(self.timeout))
            .map(drop)
    }

    fn set_eos(&mut self, op: RequestOp) -> AsynResult<()> {
        self.calls.call(op, octet_user(self.timeout)).map(drop)
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
pub fn create_ip_port(
    port_name: &str,
    host_info: &str,
    timeout: Duration,
    no_auto_connect: bool,
    process_eos: bool,
) -> AsynResult<(OctetIo, PortRuntimeHandle)> {
    let mut driver = epics_rs::asyn::drivers::ip_port::DrvAsynIPPort::new(port_name, host_info)?;
    if no_auto_connect {
        driver.base_mut().init_auto_connect(false);
    }
    if process_eos {
        driver.install_interpose(Box::new(
            epics_rs::asyn::interpose::eos::EosInterpose::default(),
        ));
    }
    let (runtime, _actor) = create_port_runtime(driver, RuntimeConfig::default())?;
    let io = OctetIo::new(runtime.port_handle().clone(), timeout);
    Ok((io, runtime))
}

/// Look up a pre-configured octet port (created by `drvAsynIPPortConfigure` or
/// `drvAsynSerialPortConfigure`) and wrap it for blocking use.
pub fn connect_octet(port_name: &str, timeout: Duration) -> AsynResult<OctetIo> {
    let port =
        epics_rs::asyn::asyn_record::get_port(port_name).ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: format!(
                "octet port '{port_name}' not found \
             (call drvAsynIPPortConfigure/drvAsynSerialPortConfigure first)"
            ),
        })?;
    Ok(OctetIo::new(port.handle.clone(), timeout))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The driver constructors set the EOS before the meter is reachable.
    #[test]
    fn eos_setters_succeed_on_a_port_that_is_not_connected() {
        let (io, _runtime) = create_ip_port(
            "QE_EOS_DOWN",
            "127.0.0.1:1",
            Duration::from_millis(200),
            true,
            true,
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
