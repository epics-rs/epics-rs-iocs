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
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::QueuePriority;
use epics_rs::asyn::port_handle::{PortHandle, QueueOutcome};
use epics_rs::asyn::request::{OctetReadOp, PortCalls, RequestCell, RequestOp};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
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
    let read = port.octet_read(OctetReadOp::Read { buf_size: size }, user);
    let eom = read.eom().bits();
    let (data, status) = read.into_parts();
    OctetRead {
        data,
        eom,
        err: status.err(),
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
/// prset->process(pr)` at the callback's `finish`: armed by
/// [`Reentry::queue`] for a port that can block, ended once by that request's
/// `process` or `unrun`.
#[derive(Default)]
struct Reentry(Mutex<Option<DeviceDone>>);

impl Reentry {
    /// `queueRequest` of `cell`, whose body ends the pass armed here. A port
    /// without `ASYN_CANBLOCK` runs the body inside this call, under the
    /// record's lock set, so nothing is armed and [`QueueOutcome::Ran`] tells
    /// the caller to finish in this pass; C processes the record a second
    /// time from inside the first instead.
    fn queue(
        &self,
        handle: &PortHandle,
        cell: &Arc<RequestCell>,
        user: &AsynUser,
        done: &DeviceDoneSource<'_>,
    ) -> AsynResult<QueueOutcome> {
        if handle.can_block() {
            *self.0.lock().unwrap() = Some(done.device_done());
        }
        let outcome = handle.queue_request(cell, user);
        if outcome.is_err() {
            self.0.lock().unwrap().take();
        }
        outcome
    }

    /// From the request's `process` or `unrun`. On a port that can block
    /// both run on its thread, which holds no lock set, so the pass runs
    /// there, as C's does, and no full callback band can refuse it.
    fn end(&self) {
        if let Some(done) = self.0.lock().unwrap().take() {
            done.reprocess_now();
        }
    }
}
