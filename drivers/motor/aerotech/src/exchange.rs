//! C `sendAndReceive`, which drvA3200Asyn.cc and drvEnsembleAsyn.cc share
//! line for line apart from the driver name and Ensemble's fault-reply line.

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus, OctetStep};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

fn trim_line(raw: &[u8]) -> String {
    let s = String::from_utf8_lossy(raw);
    s.trim_end_matches(['\r', '\n', '\0']).to_string()
}

/// One controller's command/reply line.
pub(crate) struct Exchange {
    handle: SyncIOHandle,
    /// C `DRIVER_NAME`, the prefix of every line.
    driver: &'static str,
    /// C `inputSize`.
    read_buf: usize,
    /// The reply character C reports as "Error returned" (Ensemble's
    /// `ASCII_FAULT_CHAR`; the A3200 reports none).
    fault: Option<u8>,
    /// The octet port `pController->pasynUser` prints on.
    serial_port: String,
    /// The motor port whose axis-0 user C's `PRINT(pAxis[0].logParam)`
    /// lines go to.
    motor_port: String,
}

impl Exchange {
    pub(crate) fn new(
        handle: SyncIOHandle,
        driver: &'static str,
        read_buf: usize,
        fault: Option<u8>,
        serial_port: &str,
        motor_port: &str,
    ) -> Self {
        Self {
            handle,
            driver,
            read_buf,
            fault,
            serial_port: serial_port.to_owned(),
            motor_port: motor_port.to_owned(),
        }
    }

    /// Write a command and read one reply line.
    pub(crate) fn write_read(&self, cmd: &str) -> AsynResult<String> {
        let driver = self.driver;
        let mut result = self.handle.write_read(0, cmd.as_bytes(), self.read_buf);
        match &result {
            Err(e) => {
                let written = match e.octet_step() {
                    Some(OctetStep::Read) => cmd.len(),
                    _ => e.partial_write().unwrap_or(0),
                };
                if written != cmd.len() {
                    result = Err(AsynError::Status {
                        status: AsynStatus::Error,
                        message: e.message(),
                    });
                } else if e.status() == AsynStatus::Timeout {
                    let mut retry = 1;
                    while retry <= 3
                        && matches!(&result, Err(e) if e.status() == AsynStatus::Timeout)
                    {
                        self.print_axis0(format_args!(
                            "{driver}:sendAndReceive: Retrying read, retry# = {retry}.\n"
                        ));
                        result = self.handle.read_octet(0, self.read_buf);
                        retry += 1;
                    }
                    // C counts a success on the third retry as exhausted.
                    if retry > 3 {
                        self.print_axis0(format_args!(
                            "{driver}:sendAndReceive: Retries exhausted on response to command = {cmd}.\n"
                        ));
                    } else {
                        let reply = result.as_deref().map(trim_line).unwrap_or_default();
                        self.print_axis0(format_args!(
                            "{driver}:sendAndReceive: Retry succeeded for command = {cmd} with response = {reply}\n"
                        ));
                    }
                }
            }
            Ok(raw) => {
                if self.fault.is_some() && raw.first() == self.fault.as_ref() {
                    self.print_axis0(format_args!(
                        "{driver}:sendAndReceive: Error returned for command = {cmd} with response = {}\n",
                        trim_line(raw)
                    ));
                }
            }
        }
        match result {
            Ok(raw) => Ok(trim_line(&raw)),
            Err(e) => {
                PortManager::global().trace_manager().print(
                    Some(&self.serial_port),
                    Some(0),
                    0,
                    TraceMask::ERROR,
                    format_args!(
                        "{driver}:sendAndReceive writeRead error, output={cmd} status={}, error={}\n",
                        e.status() as i32,
                        e.message()
                    ),
                );
                Err(e)
            }
        }
    }

    /// C `PRINT(pController->pAxis[0].logParam, TERROR, ...)`.
    fn print_axis0(&self, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.motor_port),
            Some(0),
            0,
            TraceMask::ERROR,
            args,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    /// Times out the first `timeouts` reads, then answers `%1`.
    struct Line {
        base: PortDriverBase,
        timeouts: usize,
        reads: Arc<AtomicUsize>,
    }

    impl PortDriver for Line {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            if self.reads.fetch_add(1, Ordering::SeqCst) < self.timeouts {
                return Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "timeout".into(),
                });
            }
            buf[..2].copy_from_slice(b"%1");
            Ok(2)
        }
    }

    fn exchange(name: &str, timeouts: usize) -> (Exchange, Arc<AtomicUsize>) {
        let reads = Arc::new(AtomicUsize::new(0));
        let line = Line {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            timeouts,
            reads: reads.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_millis(100));
        (Exchange::new(io, "drvTest", 64, None, name, name), reads)
    }

    /// drvA3200Asyn.cc:1024-1040: a timed-out reply is re-read up to three
    /// times.
    #[test]
    fn a_timed_out_reply_is_read_again_three_times() {
        let (io, reads) = exchange("aerotech_retry", 2);
        assert_eq!(io.write_read("CMD").unwrap(), "%1");
        assert_eq!(reads.load(Ordering::SeqCst), 3);

        let (io, reads) = exchange("aerotech_dead", usize::MAX);
        assert_eq!(
            io.write_read("CMD").unwrap_err().status(),
            AsynStatus::Timeout
        );
        assert_eq!(reads.load(Ordering::SeqCst), 4);
    }
}
