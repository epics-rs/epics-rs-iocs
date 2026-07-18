//! Octet-port connect helper for the `LoveInit` iocsh command. The
//! underlying asyn port is created first by `drvAsynSerialPortConfigure` or
//! `drvAsynIPPortConfigure`; this looks it up by name, applies the fixed
//! input/output EOS (C `setDefaultEos`, `drvLove.c:640-660`, called
//! unconditionally from `drvLoveInit` itself — unlike the `delaygen`
//! drivers, no `.cmd` startup fragment sets EOS for Love), and wraps the
//! port in a blocking [`SyncIOHandle`] with the given command timeout.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// Connect a [`SyncIOHandle`] to a pre-configured octet port (serial or IP)
/// by name and address, after applying `input_eos`/`output_eos` to it. `addr`
/// is C `drvLoveInit`'s `serAddr` argument
/// (`pasynManager->connectDevice(pasynUser,serPort,serAddr)`), passed
/// straight through from the user. `SyncIOHandle` has no accessor back to
/// its underlying `PortHandle`, so the EOS calls (which need the
/// `PortHandle` itself, per `PortHandle::set_input_eos_blocking`/
/// `set_output_eos_blocking`) must happen here, before wrapping.
pub fn connect_octet(
    port_name: &str,
    addr: i32,
    timeout: Duration,
    input_eos: &[u8],
    output_eos: &[u8],
) -> Result<SyncIOHandle, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!(
            "octet port '{port_name}' not found (call drvAsynSerialPortConfigure/drvAsynIPPortConfigure first)"
        )
    })?;
    port.handle
        .set_input_eos_blocking(epics_rs::asyn::user::AsynUser::default(), input_eos)
        .map_err(|e| format!("failed to set input EOS on '{port_name}': {e}"))?;
    port.handle
        .set_output_eos_blocking(epics_rs::asyn::user::AsynUser::default(), output_eos)
        .map_err(|e| format!("failed to set output EOS on '{port_name}': {e}"))?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}
