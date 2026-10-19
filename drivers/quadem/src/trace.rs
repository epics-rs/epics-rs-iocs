//! C quadEM drivers report every fault they cannot return — a read thread has
//! no caller — with `asynPrint(pasynUserSelf, ...)` on the driver's own port.
//! [`asyn_print!`](crate::asyn_print) is that call: the line goes through the port's asyn trace,
//! so `ASYN_TRACE_ERROR` prints by default and `asynSetTraceMask` governs the
//! rest, exactly as in C.

use std::fmt;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
pub use epics_rs::asyn::trace::TraceMask;

use crate::octet::{OctetIo, ReadOutcome, reply_text};

/// A holder of the driver port's trace: the driver itself inside a port
/// method, its [`PortHandle`] on a driver thread.
pub trait AsynPrint {
    fn asyn_print(&self, mask: TraceMask, args: fmt::Arguments<'_>);
}

impl AsynPrint for PortDriverBase {
    fn asyn_print(&self, mask: TraceMask, args: fmt::Arguments<'_>) {
        self.trace_print(mask, args);
    }
}

impl AsynPrint for PortHandle {
    fn asyn_print(&self, mask: TraceMask, args: fmt::Arguments<'_>) {
        self.trace()
            .print(Some(self.port_name()), Some(0), 0, mask, args);
    }
}

/// `asynPrint(pasynUserSelf, ASYN_TRACE_<MASK>, ...)`, printed as given.
#[macro_export]
macro_rules! asyn_print {
    ($target:expr, $mask:ident, $($arg:tt)*) => {
        $crate::trace::AsynPrint::asyn_print(
            &$target,
            $crate::trace::TraceMask::$mask,
            format_args!($($arg)*),
        )
    };
}

/// The `status`, `nRead` and `eomReason` C prints for a read it rejected.
pub fn read_result(read: &ReadOutcome, status: &AsynResult<()>) -> (AsynStatus, usize, u32) {
    let status = status
        .as_ref()
        .map_or_else(|e| e.status(), |()| AsynStatus::Success);
    (status, read.data.len(), read.eom.bits())
}

/// `asynPrintIO(pasynUserSelf, mask, data, len, ...)` from a driver thread.
pub fn print_io(handle: &PortHandle, mask: TraceMask, data: &[u8], args: fmt::Arguments<'_>) {
    let trace = handle.trace();
    if trace.is_enabled_device(handle.port_name(), 0, mask) {
        trace.output_device_io(
            handle.port_name(),
            Some(0),
            0,
            mask,
            data,
            &args.to_string(),
            "",
            0,
        );
    }
}

/// The line every quadEM read thread prints for a read it rejected that was
/// not a timeout; `origin` is C's `"%s::%s"` (or `"%s:%s"`) prefix.
pub fn print_read_error(
    handle: &PortHandle,
    origin: &str,
    read: &ReadOutcome,
    status: &AsynResult<()>,
) {
    let (status, n_read, eom) = read_result(read, status);
    asyn_print!(
        *handle,
        ERROR,
        "{origin}: unexpected error reading meter status={}, nRead={n_read}, eomReason={eom}\n",
        status as i32
    );
}

/// `setAcquire`'s "Make sure the input EOS is set" (drvAHxxx.cpp:465-472,
/// drvTetrAMM.cpp:464-471, drvPCR4.cpp:356-363): a refusal is printed and
/// fails the acquire. `origin` is C's `"%s::%s"` (or `"%s:%s"`) prefix.
pub fn ensure_input_eos(io: &OctetIo, port: &PortDriverBase, origin: &str) -> AsynResult<()> {
    io.set_input_eos(b"\r\n").map_err(|e| {
        asyn_print!(
            *port,
            ERROR,
            "{origin}: error calling pasynOctetSyncIO->setInputEos, status={}\n",
            e.c_status_code()
        );
        AsynError::Status {
            status: AsynStatus::Error,
            message: e.message(),
        }
    })
}

/// C's `outString_` / `inString_` as a `getStatus` leaves them, for its
/// `"error, outString=%s, inString=%s"` line.
#[derive(Default)]
pub struct StatusExchange {
    pub out: String,
    pub inp: String,
}

impl StatusExchange {
    /// One query: the command is recorded, and the reply when there is one;
    /// a failed read leaves C's buffer, and this, holding the previous reply.
    pub fn query(&mut self, io: &OctetIo, out: &str, buf_size: usize) -> AsynResult<String> {
        self.out.clear();
        self.out.push_str(out);
        let (raw, status) = io.write_read(out, buf_size).into_parts();
        status?;
        let resp = reply_text(&raw);
        self.inp.clone_from(&resp);
        Ok(resp)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    use super::*;

    struct Meter(PortDriverBase);

    impl PortDriver for Meter {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    /// C's `pasynUserSelf` is connected to device 0, so on a multi-device
    /// port a driver thread's line follows `asynSetTraceFile P 0`, not the
    /// port-level file.
    #[test]
    fn a_driver_thread_line_goes_through_device_0() {
        let name = "QE_SELF_DEV0";
        let flags = PortFlags {
            multi_device: true,
            ..PortFlags::default()
        };
        let claim = claim_port(name).unwrap();
        let (runtime, _actor) = create_port_runtime(
            &claim,
            Meter(PortDriverBase::new(
                name,
                2,
                flags,
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            )),
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        claim
            .publish(handle.clone(), &InterruptGate::opened())
            .unwrap();

        let path = std::env::temp_dir().join(format!("qe-self-dev0-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        handle.trace().set_device_trace_file(
            name,
            0,
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );

        crate::asyn_print!(handle, ERROR, "drvQuadEM::readThread: line\n");

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.ends_with("drvQuadEM::readThread: line\n"), "{text:?}");
    }
}
