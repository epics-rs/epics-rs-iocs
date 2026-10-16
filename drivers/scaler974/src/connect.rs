//! Octet-port connect helper for the `initScaler974` iocsh command. Mirrors
//! `pasynOctetSyncIO->connect(serialPort, serialAddr, &this->pasynUserScaler,
//! NULL)` (`drvScaler974.cpp:72`) — the C driver never calls
//! `pasynOctetSetInputEos`/`pasynOctetSetOutputEos` itself, so this doesn't
//! either. EOS is expected to already be configured on the underlying
//! octet port (e.g. via `asynOctetSetInputEos`/`OutputEos` in `st.cmd`,
//! consulting the Ortec 974 hardware manual for the actual terminator)
//! before `initScaler974` runs.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::{TraceManager, TraceMask};

/// C `pasynUserScaler`, plus the trace manager `Scaler974`'s own
/// `pasynUserSelf` prints through.
pub struct OctetLink {
    pub(crate) sync: SyncIOHandle,
    trace: Arc<TraceManager>,
}

impl OctetLink {
    /// C `asynPrint(this->pasynUserSelf, mask, ...)`. The scaler port is no
    /// asyn port here, so the line takes the global trace settings.
    pub fn print_self(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        self.trace.print(None, None, 0, mask, args);
    }
}

/// Connect an [`OctetLink`] to a pre-configured octet port (serial or
/// GPIB) by name and address. `addr` is C `drvScaler974`'s `serialAddr`
/// argument, passed straight through. Call this twice per `initScaler974`
/// invocation (see `driver::Scaler974Driver::new`) — once for the
/// synchronous command path (reset/arm/write_preset), once for the
/// background `SHOW_COUNTS` poll thread — since `SyncIOHandle` is not
/// `Clone`; both handles wrap the same underlying `PortHandle`, whose
/// requests are serialized by the port's actor regardless of which handle
/// instance issues them.
pub fn connect_octet(port_name: &str, addr: i32, timeout: Duration) -> Result<OctetLink, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!(
            "octet port '{port_name}' not found (call drvAsynSerialPortConfigure/drvAsynIPPortConfigure first)"
        )
    })?;
    Ok(OctetLink {
        sync: SyncIOHandle::from_handle(port.handle.clone(), addr, timeout),
        trace: port.handle.trace().clone(),
    })
}
