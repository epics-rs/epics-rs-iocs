//! Octet-port connect helper for the `TeledyneDInit`/`TeledyneHInit` iocsh
//! commands.
//!
//! # EOS ownership
//! Neither `teled_d.proto` nor `teled_h.proto` is StreamDevice-engine
//! executed in this port (per the task's "no StreamDevice engine" decision:
//! the `.proto` files are the wire-format *specification* being translated,
//! not a runtime dependency) -- but each still opens with `Terminator =
//! CR;`, StreamDevice's directive for "use this byte as both the input and
//! output end-of-string." That makes the `.proto` file itself the framing
//! spec's owner, the same role a hardcoded C `setDefaultEos` call plays for
//! `drivers/love` (see `love::connect`'s module doc) -- so, like Love, this
//! driver applies EOS itself at connect time rather than relying on a
//! Teledyne `st.cmd` fragment to set it (no such upstream `st.cmd`/iocBoot
//! directory ships for either Teledyne family to begin with -- confirmed
//! absent from `epics-modules/SyringePump/iocBoot`).
//!
//! `teled_h.proto` additionally declares `ExtraInput = Ignore;`; see
//! `wire_h`'s module doc for why that needs no separate handling here.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// `Terminator = CR;` in both `teled_d.proto` and `teled_h.proto` -- a bare
/// CR (`\r`), applied to both input and output EOS.
pub const EOS: &[u8] = b"\r";

/// Connect a [`SyncIOHandle`] to a pre-configured octet port (serial or IP)
/// by name and address, after applying [`EOS`] to both directions.
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
        .set_input_eos_blocking(epics_rs::asyn::user::AsynUser::default(), EOS)
        .map_err(|e| format!("failed to set input EOS on '{port_name}': {e}"))?;
    port.handle
        .set_output_eos_blocking(epics_rs::asyn::user::AsynUser::default(), EOS)
        .map_err(|e| format!("failed to set output EOS on '{port_name}': {e}"))?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}
