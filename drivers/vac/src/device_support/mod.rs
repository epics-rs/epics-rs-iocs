//! asyn-octet device support for the two record families.
//!
//! `devVacSen` and `devDigitelPump` are C dsets whose `readWrite` runs in two
//! passes: `pact == 0` builds one command and queues an asyn request, and the
//! request's callback performs the whole write/read exchange, packs the replies
//! into a fixed buffer, and reprocesses the record so `pact == 1` decodes it.
//! There is no `pasynManager->queueRequest` primitive exposed by the published
//! framework, so this port collapses both passes into a synchronous
//! [`DeviceSupport::read`](epics_rs::base::server::record::Record) that talks to
//! the octet port with blocking I/O and writes straight into the record. The
//! consequence — no `COMM_ALARM` on a queue-full condition, because there is no
//! queue — is recorded in the crate docs.
//!
//! EOS is owned entirely by the startup script (neither C dset sets it), so the
//! exchange helpers here emit bare command bytes and rely on the port's
//! configured terminators.

pub mod digitel_pump;
pub mod vac_sen;

use std::time::Duration;

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;

/// A connected octet port plus the per-request `asynUser` parameters. The C
/// dsets talk to a single-device serial port at subaddress 0.
struct PortIo {
    handle: PortHandle,
    addr: i32,
    timeout: Duration,
}

impl PortIo {
    fn user(&self) -> AsynUser {
        AsynUser::new(0)
            .with_addr(self.addr)
            .with_timeout(self.timeout)
    }

    /// C `pasynOctet->write`. A failed write is logged and ignored in C; here it
    /// simply yields no effect and the following read returns what it can.
    fn write(&self, data: &[u8]) {
        let _ = self.handle.submit_blocking(
            RequestOp::OctetWrite {
                data: data.to_vec(),
            },
            self.user(),
        );
    }

    /// C `pasynOctet->read` into a `size`-byte buffer, returning the bytes read.
    /// An asyn error becomes an empty reply, matching C where `*nread` is the
    /// (zero) byte count after a failed read.
    fn read(&self, size: usize) -> Vec<u8> {
        match self
            .handle
            .submit_blocking(RequestOp::OctetRead { buf_size: size }, self.user())
        {
            Ok(res) => res.data.unwrap_or_default(),
            Err(_) => Vec::new(),
        }
    }

    /// C `pasynOctet->flush`.
    fn flush(&self) {
        let _ = self.handle.submit_blocking(RequestOp::Flush, self.user());
    }
}
