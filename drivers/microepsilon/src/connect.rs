//! Octet-port connect helper for the L0 config port's iocsh init command.
//! The underlying asyn port (serial or IP) is created first by
//! `drvAsynSerialPortConfigure`/`drvAsynIPPortConfigure`; this looks it up by
//! name, applies `capaNCDT6200.proto`'s fixed EOS (`InTerminator = CR LF;
//! OutTerminator = CR;`), and wraps the port in a blocking [`SyncIOHandle`].
//! Mirrors `drivers/love::connect::connect_octet`'s established pattern in
//! this workspace.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// `capaNCDT6200.proto`: `InTerminator = CR LF;`.
pub const INPUT_EOS: &[u8] = b"\r\n";
/// `capaNCDT6200.proto`: `OutTerminator = CR;`.
pub const OUTPUT_EOS: &[u8] = b"\r";

/// Connect a [`SyncIOHandle`] to a pre-configured octet port (serial or IP)
/// by name and address, after applying the `.proto`'s fixed input/output EOS
/// to it. `SyncIOHandle` has no accessor back to its underlying
/// `PortHandle`, so the EOS calls (which need the `PortHandle` itself, per
/// `PortHandle::set_input_eos_blocking`/`set_output_eos_blocking`) must
/// happen here, before wrapping.
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
    port.handle
        .set_input_eos_blocking(epics_rs::asyn::user::AsynUser::default(), INPUT_EOS)
        .map_err(|e| format!("failed to set input EOS on '{port_name}': {e}"))?;
    port.handle
        .set_output_eos_blocking(epics_rs::asyn::user::AsynUser::default(), OUTPUT_EOS)
        .map_err(|e| format!("failed to set output EOS on '{port_name}': {e}"))?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}
