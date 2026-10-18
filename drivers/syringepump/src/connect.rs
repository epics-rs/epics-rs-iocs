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
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

/// `Terminator = CR;` in both `teled_d.proto` and `teled_h.proto` -- a bare
/// CR (`\r`). The writer appends it; asyn applies it only on input.
pub const EOS: &[u8] = b"\r";

/// C sets the EOS on a line that may still be down (`asynSetEos`'s
/// QUEUE_EVEN_IF_NOT_CONNECTED).
fn eos_user(addr: i32) -> AsynUser {
    AsynUser::default()
        .with_addr(addr)
        .queue_even_if_not_connected()
}

/// StreamDevice `readHandler`'s EOS setup (AsynDriverInterface.cc:828-857):
/// a port with no input EOS support is left alone, and a refused terminator
/// is retried without its leading byte, then warned about. Never a failure.
fn set_input_eos(port: &PortHandle, port_name: &str, addr: i32) {
    let Ok(old) = port.get_input_eos_blocking(eos_user(addr)) else {
        return;
    };
    let mut eos = EOS;
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
/// by name and address, after applying [`EOS`] to its input.
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
    // ignoring the result (AsynDriverInterface.cc:650-663).
    let _ = port.handle.set_output_eos_blocking(eos_user(addr), b"");
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use std::sync::{Arc, Mutex};

    /// A port that refuses every EOS and keeps what is written to it.
    struct NoEos(PortDriverBase, Arc<Mutex<Vec<u8>>>);

    impl PortDriver for NoEos {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.1.lock().unwrap().extend_from_slice(data);
            Ok(data.len())
        }
        fn set_input_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "no EOS".into(),
            })
        }
        fn set_output_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "no EOS".into(),
            })
        }
    }

    /// A published [`NoEos`] port and the bytes written to it.
    pub(crate) fn eos_refusing_port(name: &str) -> Arc<Mutex<Vec<u8>>> {
        let written = Arc::new(Mutex::new(Vec::new()));
        let driver = NoEos(
            PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written.clone(),
        );
        let claim = epics_rs::asyn::registry::claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(
                rt.port_handle().clone(),
                &epics_rs::base::runtime::interrupt_accept::InterruptGate::opened(),
            )
            .unwrap();
        std::mem::forget(rt);
        written
    }

    #[test]
    fn a_port_refusing_both_eos_still_connects() {
        eos_refusing_port("SP_NOEOS");
        assert!(connect_octet("SP_NOEOS", 0, Duration::from_secs(1)).is_ok());
    }
}
