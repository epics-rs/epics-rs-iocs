//! asyn-octet device support for the two record families.
//!
//! `devVacSen` and `devDigitelPump` are C dsets whose `readWrite` runs in two
//! passes: `pact == 0` builds one command and queues an asyn request, and the
//! request's callback performs the whole write/read exchange on the port
//! thread, packs the replies into a fixed buffer, and reprocesses the record
//! so `pact == 1` decodes it. Here the first pass is
//! [`DeviceSupport::read_begin`](epics_rs::base::server::device_support::DeviceSupport::read_begin),
//! the callback a [`CellBody`](epics_rs::asyn::request::CellBody) on a
//! preallocated [`RequestCell`](epics_rs::asyn::request::RequestCell), and the
//! second pass `read`.
//!
//! EOS is owned entirely by the startup script (neither C dset sets it), so the
//! exchange helpers here emit bare command bytes and rely on the port's
//! configured terminators.

pub mod digitel_pump;
pub mod vac_sen;

use std::fmt;
use std::sync::Mutex;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::QueuePriority;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{PortCalls, RequestOp};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::task::CallbackPriority;
use epics_rs::base::server::database::{DeviceDone, DeviceDoneSource};

/// `asynPrint(pasynUser, mask, ...)` on the record's port, printed as given.
fn asyn_print(handle: &PortHandle, mask: TraceMask, args: fmt::Arguments<'_>) {
    handle
        .trace()
        .print(Some(handle.port_name()), None, 0, mask, args);
}

/// The record's `asynUser` as C's callback sets it: subaddress `addr`, the
/// dset's fixed timeout, queued at `asynQueuePriorityLow`.
fn user(addr: i32, timeout: Duration) -> AsynUser {
    AsynUser::new(0)
        .with_addr(addr)
        .with_timeout(timeout)
        .with_priority(QueuePriority::Low)
}

/// One `pasynOctet->read`: the bytes it left in the buffer (a failed read's
/// partial bytes included), its `eomReason`, and the failure C sees in its
/// status.
struct OctetRead {
    data: Vec<u8>,
    eom: u32,
    err: Option<AsynError>,
}

fn octet_read(port: &mut dyn PortCalls, user: AsynUser, size: usize) -> OctetRead {
    match port.call(&RequestOp::OctetRead { buf_size: size }, user) {
        Ok(r) => OctetRead {
            data: r.data.unwrap_or_default(),
            eom: r.eom_reason,
            err: None,
        },
        Err(e) => {
            let (data, eom) = e
                .partial_read()
                .map_or((Vec::new(), 0), |p| (p.data.clone(), p.eom_reason.bits()));
            OctetRead {
                data,
                eom,
                err: Some(e),
            }
        }
    }
}

/// `pasynOctet->write`: the bytes written, or the failure.
fn octet_write(port: &mut dyn PortCalls, user: AsynUser, data: &[u8]) -> AsynResult<usize> {
    port.call(
        &RequestOp::OctetWrite {
            data: data.to_vec(),
        },
        user,
    )
    .map(|r| r.nbytes)
}

/// C's `%s` of a reply buffer: up to its NUL.
fn c_str(b: &[u8]) -> std::borrow::Cow<'_, str> {
    String::from_utf8_lossy(crate::protocol::cstr(b))
}

/// The pass a queued request's callback re-enters — C's `dbScanLock(pr);
/// prset->process(pr)` at the callback's `finish`.
#[derive(Default)]
struct Reentry(Mutex<Option<(DeviceDone, CallbackPriority)>>);

impl Reentry {
    fn arm(&self, done: &DeviceDoneSource<'_>) {
        let mut slot = self.0.lock().unwrap();
        match &mut *slot {
            Some((_, prio)) => *prio = done.priority(),
            None => *slot = Some((done.device_done(), done.priority())),
        }
    }

    fn reprocess(&self) {
        let armed = self.0.lock().unwrap().clone();
        if let Some((done, prio)) = armed {
            let _ = done.reprocess(prio);
        }
    }
}
