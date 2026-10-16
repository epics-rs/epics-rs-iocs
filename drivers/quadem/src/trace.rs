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

use crate::octet::{OctetIo, ReadOutcome};

/// A holder of the driver port's trace: the driver itself inside a port
/// method, its [`PortHandle`] on a driver thread.
pub trait AsynPrint {
    fn asyn_print(&self, mask: TraceMask, args: fmt::Arguments<'_>);
}

impl AsynPrint for PortDriverBase {
    fn asyn_print(&self, mask: TraceMask, args: fmt::Arguments<'_>) {
        self.trace_print(mask, &args.to_string());
    }
}

impl AsynPrint for PortHandle {
    fn asyn_print(&self, mask: TraceMask, args: fmt::Arguments<'_>) {
        self.trace()
            .print(Some(self.port_name()), None, 0, mask, args);
    }
}

/// `asynPrint(pasynUserSelf, ASYN_TRACE_<MASK>, ...)`; the newline is added.
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

/// The `status=%d` C prints: the `asynStatus` value.
pub fn status_code(e: &AsynError) -> i32 {
    e.status() as i32
}

/// The `status`, `nRead` and `eomReason` C prints for a read it rejected.
pub fn read_result(r: &AsynResult<ReadOutcome>) -> (AsynStatus, usize, u32) {
    match r {
        Ok(o) => (AsynStatus::Success, o.data.len(), o.eom.bits()),
        Err(e) => {
            let (n, eom) = e
                .partial_read()
                .map_or((0, 0), |p| (p.data.len(), p.eom_reason.bits()));
            (e.status(), n, eom)
        }
    }
}

/// `asynPrintIO(pasynUserSelf, mask, data, len, ...)` from a driver thread.
pub fn print_io(handle: &PortHandle, mask: TraceMask, data: &[u8], args: fmt::Arguments<'_>) {
    let trace = handle.trace();
    if trace.is_enabled(handle.port_name(), mask) {
        trace.output_io(handle.port_name(), mask, data, &args.to_string(), "", 0);
    }
}

/// The line every quadEM read thread prints for a read it rejected that was
/// not a timeout; `origin` is C's `"%s::%s"` (or `"%s:%s"`) prefix.
pub fn print_read_error(handle: &PortHandle, origin: &str, read: &AsynResult<ReadOutcome>) {
    let (status, n_read, eom) = read_result(read);
    asyn_print!(
        *handle,
        ERROR,
        "{origin}: unexpected error reading meter status={}, nRead={n_read}, eomReason={eom}",
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
            "{origin}: error calling pasynOctetSyncIO->setInputEos, status={}",
            status_code(&e)
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
        let resp = io.write_read(out, buf_size)?;
        self.inp.clone_from(&resp);
        Ok(resp)
    }
}
