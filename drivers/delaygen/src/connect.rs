//! Octet-port connect helper shared by the delaygen `*Config` iocsh
//! commands. The underlying asyn port is created first by
//! `drvAsynSerialPortConfigure` or `drvAsynIPPortConfigure`; this looks it up
//! by name and wraps it in a blocking [`SyncIOHandle`] with the given
//! command timeout.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

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
    use epics_rs::asyn::asyn_record::claim_port;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    struct Mute(PortDriverBase);

    impl PortDriver for Mute {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    let claim = claim_port(io).unwrap();
    let (runtime, _actor) = create_port_runtime(
        &claim,
        Mute(PortDriverBase::new(io, 1, PortFlags::default())),
        RuntimeConfig::default(),
    )
    .unwrap();
    claim
        .publish(runtime.port_handle().clone(), &InterruptGate::opened())
        .unwrap();
    connect_octet(io, 0, Duration::from_millis(100)).unwrap()
}
