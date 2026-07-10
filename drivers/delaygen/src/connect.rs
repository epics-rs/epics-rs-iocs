//! Octet-port connect helper shared by the delaygen `*Config` iocsh
//! commands. The underlying asyn port is created first by
//! `drvAsynSerialPortConfigure` or `drvAsynIPPortConfigure`; this looks it up
//! by name and wraps it in a blocking [`SyncIOHandle`] with the given
//! command timeout.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// Connect a [`SyncIOHandle`] to a pre-configured octet port (serial or IP)
/// by name and address. `addr` is the C driver's `ioaddr`/`addr` iocsh
/// argument (`pasynOctetSyncIO->connect(ioport, ioaddr, ...)`); each of the
/// three C drivers passes it straight through from the user, so it is not
/// hardcoded here even though a plain point-to-point serial/IP line
/// typically uses 0.
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
