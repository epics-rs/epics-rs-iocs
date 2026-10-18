//! Octet-port connect helper shared by the delaygen `*Config` iocsh
//! commands. The underlying asyn port is created first by
//! `drvAsynSerialPortConfigure` or `drvAsynIPPortConfigure`; this looks it up
//! by name and wraps it in a blocking [`SyncIOHandle`] with the given
//! command timeout.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::{TraceMask, TraceOut};

/// A driver's C `pasynOctetSyncIO` user on its io port: the blocking handle
/// plus the port and address its `asynPrint` lines resolve against.
pub struct OctetLink {
    pub(crate) sync: SyncIOHandle,
    port: PortHandle,
    addr: i32,
}

impl OctetLink {
    /// C `asynPrint(pasynUser, mask, ...)` on this io-port user.
    pub fn print(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        let addr = (self.addr >= 0).then_some(self.addr);
        self.port
            .trace()
            .print(Some(self.port.port_name()), addr, 0, mask, args);
    }

    /// [`Self::print`] for a line that carries device bytes, written by
    /// `body` with [`TraceOut::write_bytes`] as C's `%s` writes them.
    pub fn write_trace(
        &self,
        mask: TraceMask,
        body: impl FnOnce(&mut TraceOut<'_>) -> std::fmt::Result,
    ) {
        let addr = (self.addr >= 0).then_some(self.addr);
        self.port
            .trace()
            .print_with(Some(self.port.port_name()), addr, 0, mask, ("", 0), body);
    }

    /// C's recovery after a failed transaction: `lockPort`, then
    /// `pasynCommon->disconnect` and `->connect` on this user, results
    /// ignored.
    pub fn reset(&self) {
        if self.addr < 0 {
            let _ = self.port.disconnect_blocking();
            let _ = self.port.connect_blocking();
        } else {
            let _ = self.port.disconnect_addr_blocking(self.addr);
            let _ = self.port.connect_addr_blocking(self.addr);
        }
    }
}

/// Connect an [`OctetLink`] to a pre-configured octet port (serial or IP)
/// by name and address. `addr` is the C driver's `ioaddr`/`addr` iocsh
/// argument (`pasynOctetSyncIO->connect(ioport, ioaddr, ...)`); each of the
/// three C drivers passes it straight through from the user, so it is not
/// hardcoded here even though a plain point-to-point serial/IP line
/// typically uses 0.
pub fn connect_octet(port_name: &str, addr: i32, timeout: Duration) -> Result<OctetLink, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!(
            "octet port '{port_name}' not found (call drvAsynSerialPortConfigure/drvAsynIPPortConfigure first)"
        )
    })?;
    Ok(OctetLink {
        sync: SyncIOHandle::from_handle(port.handle.clone(), addr, timeout),
        port: port.handle.clone(),
        addr,
    })
}

/// An [`OctetLink`] onto a published port `io` whose every octet transfer
/// fails: a device that never answers.
#[cfg(test)]
pub(crate) fn mute_link(io: &str) -> OctetLink {
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};

    struct Mute(PortDriverBase);

    impl PortDriver for Mute {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    link_onto(
        io,
        Mute(PortDriverBase::new(
            io,
            1,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_INTERRUPTS,
        )),
    )
}

/// An [`OctetLink`] onto a published port `io` that accepts every write and
/// answers every read with `reply`.
#[cfg(test)]
pub(crate) fn replying_link(io: &str, reply: &'static [u8]) -> OctetLink {
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::user::AsynUser;

    struct Replying(PortDriverBase, &'static [u8]);

    impl PortDriver for Replying {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let n = self.1.len().min(buf.len());
            buf[..n].copy_from_slice(&self.1[..n]);
            Ok(n)
        }
    }

    link_onto(
        io,
        Replying(
            PortDriverBase::new(
                io,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            reply,
        ),
    )
}

#[cfg(test)]
fn link_onto(io: &str, driver: impl epics_rs::asyn::port::PortDriver + 'static) -> OctetLink {
    use epics_rs::asyn::asyn_record::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    let claim = claim_port(io).unwrap();
    let (runtime, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
    claim
        .publish(runtime.port_handle().clone(), &InterruptGate::opened())
        .unwrap();
    connect_octet(io, 0, Duration::from_millis(100)).unwrap()
}
