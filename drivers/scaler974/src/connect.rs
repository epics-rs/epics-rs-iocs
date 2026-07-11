//! Octet-port connect helper for the `initScaler974` iocsh command. Mirrors
//! `pasynOctetSyncIO->connect(serialPort, serialAddr, &this->pasynUserScaler,
//! NULL)` (`drvScaler974.cpp:72`) — the C driver never calls
//! `pasynOctetSetInputEos`/`pasynOctetSetOutputEos` itself, so this doesn't
//! either. EOS is expected to already be configured on the underlying
//! octet port (e.g. via `asynOctetSetInputEos`/`OutputEos` in `st.cmd`,
//! consulting the Ortec 974 hardware manual for the actual terminator)
//! before `initScaler974` runs.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// Connect a [`SyncIOHandle`] to a pre-configured octet port (serial or
/// GPIB) by name and address. `addr` is C `drvScaler974`'s `serialAddr`
/// argument, passed straight through. Call this twice per `initScaler974`
/// invocation (see `driver::Scaler974Driver::new`) — once for the
/// synchronous command path (reset/arm/write_preset), once for the
/// background `SHOW_COUNTS` poll thread — since `SyncIOHandle` is not
/// `Clone`; both handles wrap the same underlying `PortHandle`, whose
/// requests are serialized by the port's actor regardless of which handle
/// instance issues them.
pub fn connect_octet(
    port_name: &str,
    addr: i32,
    timeout: Duration,
) -> Result<SyncIOHandle, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!(
            "octet port '{port_name}' not found (call drvAsynSerialPortConfigure/drvAsynIPPortConfigure first)"
        )
    })?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}
