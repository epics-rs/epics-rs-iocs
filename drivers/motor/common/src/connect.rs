//! Octet-port connect helpers shared by the vendor create commands. A
//! controller's asyn port is created first by `drvAsynSerialPortConfigure` or
//! `drvAsynIPPortConfigure`; these look it up by name and wrap it in a
//! blocking [`SyncIOHandle`] with the given command timeout.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

/// asyn subaddress for a single-device octet port (0; GPIB would differ).
const SERIAL_ADDR: i32 = 0;

/// Connect a [`SyncIOHandle`] to a pre-configured serial octet port by name
/// (created by `drvAsynSerialPortConfigure`).
pub fn connect_serial(serial_port: &str, timeout: Duration) -> Result<SyncIOHandle, String> {
    let port = get_port(serial_port).ok_or_else(|| {
        format!("serial port '{serial_port}' not found (call drvAsynSerialPortConfigure first)")
    })?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        SERIAL_ADDR,
        timeout,
    ))
}

/// Connect a [`SyncIOHandle`] to a pre-configured TCP octet port by name
/// (created by `drvAsynIPPortConfigure`). Same lookup as [`connect_serial`] —
/// separate only for a TCP-appropriate error message.
pub fn connect_ip(ip_port: &str, timeout: Duration) -> Result<SyncIOHandle, String> {
    let port = get_port(ip_port).ok_or_else(|| {
        format!("IP port '{ip_port}' not found (call drvAsynIPPortConfigure first)")
    })?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        SERIAL_ADDR,
        timeout,
    ))
}

/// Connect to a pre-configured octet port by name and set the input and
/// output EOS the C driver sets on its `pasynUser` right after
/// `pasynOctetSyncIO->connect`, so the wire framing does not depend on
/// st.cmd. EOS lives on the port (per address), so it must be set on the
/// `PortHandle` before wrapping.
pub fn connect_with_eos(
    port_name: &str,
    timeout: Duration,
    input_eos: &[u8],
    output_eos: &[u8],
) -> Result<SyncIOHandle, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!("octet port '{port_name}' not found (call drvAsynSerialPortConfigure or drvAsynIPPortConfigure first)")
    })?;
    let user = || AsynUser::new(0).with_addr(SERIAL_ADDR);
    port.handle
        .set_input_eos_blocking(user(), input_eos)
        .map_err(|e| format!("failed to set input EOS on '{port_name}': {e}"))?;
    port.handle
        .set_output_eos_blocking(user(), output_eos)
        .map_err(|e| format!("failed to set output EOS on '{port_name}': {e}"))?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        SERIAL_ADDR,
        timeout,
    ))
}
