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
    // StreamDevice never fails on a refused EOS: it warns and reads on
    // (AsynDriverInterface.cc:450-456, :842-857) and ignores the output
    // status (:663, :689).
    let _ = port
        .handle
        .set_input_eos_blocking(epics_rs::asyn::user::AsynUser::default(), EOS);
    let _ = port
        .handle
        .set_output_eos_blocking(epics_rs::asyn::user::AsynUser::default(), EOS);
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        addr,
        timeout,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;

    /// A port that refuses every EOS.
    struct NoEos(PortDriverBase);

    impl PortDriver for NoEos {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
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

    #[test]
    fn a_port_refusing_both_eos_still_connects() {
        let name = "SP_NOEOS";
        let driver = NoEos(PortDriverBase::new(name, 1, PortFlags::default()));
        let (rt, _actor) = create_port_runtime(driver, RuntimeConfig::default()).unwrap();
        epics_rs::asyn::asyn_record::register_port(
            name,
            rt.port_handle().clone(),
            &epics_rs::base::runtime::interrupt_accept::InterruptGate::opened(),
        )
        .unwrap();
        assert!(connect_octet(name, 0, Duration::from_secs(1)).is_ok());
    }
}
