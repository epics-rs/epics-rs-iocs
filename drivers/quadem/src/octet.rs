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
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
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
        AsynUser::new(OCTET_REASON)
            .with_addr(OCTET_ADDR)
            .with_timeout(self.timeout)
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

    /// `pasynOctetSyncIO->setInputEos`. Called once at driver construction;
    /// nothing writes this cell afterwards.
    pub fn set_input_eos(&self, eos: &[u8]) -> AsynResult<()> {
        self.handle.set_input_eos_blocking(eos)
    }

    /// `asynOctetSyncIO->flush`.
    pub fn flush(&self) -> AsynResult<()> {
        self.handle.submit_blocking(RequestOp::Flush, self.user())?;
        Ok(())
    }
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
