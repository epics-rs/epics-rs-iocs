//! Octet-port connect helper for the L0 config port's iocsh init command.
//! The underlying asyn port (serial or IP) is created first by
//! `drvAsynSerialPortConfigure`/`drvAsynIPPortConfigure`; this looks it up by
//! name, applies `capaNCDT6200.proto`'s `InTerminator = CR LF;` as
//! StreamDevice does, and wraps the port in a blocking [`SyncIOHandle`].
//! Mirrors `drivers/love::connect::connect_octet`'s established pattern in
//! this workspace.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// `capaNCDT6200.proto`: `InTerminator = CR LF;`.
pub const INPUT_EOS: &[u8] = b"\r\n";
/// `capaNCDT6200.proto`: `OutTerminator = CR;`.
pub const OUTPUT_EOS: &[u8] = b"\r";

/// C sets the EOS on a line that may still be down (`lockPort` in
/// `asynOctetSyncIO.c:306`, `asynSetEos`'s QUEUE_EVEN_IF_NOT_CONNECTED).
fn eos_user(addr: i32) -> epics_rs::asyn::user::AsynUser {
    epics_rs::asyn::user::AsynUser::default()
        .with_addr(addr)
        .queue_even_if_not_connected()
}

/// StreamDevice `readHandler`'s EOS setup: a port with no input EOS support
/// is left alone, and a refused terminator is retried without its leading
/// byte (a GPIB line may take LF but not CR LF), then warned about. Never a
/// configure failure.
fn set_input_eos(port: &PortHandle, port_name: &str, addr: i32) {
    let Ok(old) = port.get_input_eos_blocking(eos_user(addr)) else {
        return;
    };
    let mut eos = INPUT_EOS;
    while !eos.is_empty() && eos != old.as_slice() {
        match port.set_input_eos_blocking(eos_user(addr), eos) {
            Ok(()) => return,
            Err(e) if eos.len() == 1 => {
                eprintln!("{port_name}: warning: pasynOctet->setInputEos() failed: {e}")
            }
            Err(_) => {}
        }
        eos = &eos[1..];
    }
}

/// Connect a [`SyncIOHandle`] to a pre-configured octet port (serial or IP)
/// by name and address, after applying the `.proto`'s input EOS to it.
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
    set_input_eos(&port.handle, port_name, addr);
    // StreamDevice appends the OutTerminator itself and clears asyn's,
    // ignoring the result.
    let _ = port.handle.set_output_eos_blocking(eos_user(addr), b"");
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}
