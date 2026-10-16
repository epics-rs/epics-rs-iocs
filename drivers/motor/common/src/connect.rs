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

/// What a refused EOS set does to a controller's create, and the order the
/// two are set in — each C driver decides both for itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EosRefusal {
    /// Both statuses ignored (drvPIC862.cc:530-533, drvEMC18011.cc:668-671,
    /// OEMMotorDriver.cpp:54-55 and their siblings).
    Ignored,
    /// Output then input; only an input refusal fails the create, the output
    /// status is ignored "in case this is a GPIB port" (drvESP300.cc:679-689).
    InputFatal,
    /// Input then output; either refusal fails the create
    /// (PIasynController.cpp:133-146).
    BothFatal,
}

/// Connect to a pre-configured octet port by name and set the EOS the C
/// driver sets on its `pasynUser` right after `pasynOctetSyncIO->connect`, so
/// the wire framing does not depend on st.cmd. EOS lives on the port (per
/// address), so it must be set on the `PortHandle` before wrapping.
pub fn connect_with_eos(
    port_name: &str,
    timeout: Duration,
    input_eos: &[u8],
    output_eos: &[u8],
    refusal: EosRefusal,
) -> Result<SyncIOHandle, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!("octet port '{port_name}' not found (call drvAsynSerialPortConfigure or drvAsynIPPortConfigure first)")
    })?;
    let user = || AsynUser::new(0).with_addr(SERIAL_ADDR);
    let set_input = || {
        port.handle
            .set_input_eos_blocking(user(), input_eos)
            .map_err(|e| format!("unable to set input EOS on {port_name}: {e}"))
    };
    let set_output = || {
        port.handle
            .set_output_eos_blocking(user(), output_eos)
            .map_err(|e| format!("unable to set output EOS on {port_name}: {e}"))
    };
    match refusal {
        EosRefusal::Ignored => {
            let _ = set_output();
            let _ = set_input();
        }
        EosRefusal::InputFatal => {
            let _ = set_output();
            set_input()?;
        }
        EosRefusal::BothFatal => {
            set_input()?;
            set_output()?;
        }
    }
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        SERIAL_ADDR,
        timeout,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{open_owner, register_port};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

    /// A port with no EOS support of its own: both sets are refused.
    struct Bare(PortDriverBase);

    impl PortDriver for Bare {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    /// GPIB-like: it takes an input EOS and refuses an output one.
    struct Gpib(PortDriverBase);

    impl PortDriver for Gpib {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn set_input_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Ok(())
        }
    }

    fn publish<D: PortDriver>(name: &str, driver: impl FnOnce(PortDriverBase) -> D) {
        let base = PortDriverBase::new(name, 1, PortFlags::default());
        let (rt, _actor) = create_port_runtime(driver(base), RuntimeConfig::default()).unwrap();
        register_port(name, rt.port_handle().clone(), &open_owner()).unwrap();
        std::mem::forget(rt);
    }

    /// The create's verdict: `Ok(())` when it connected.
    fn connect(port: &str, refusal: EosRefusal) -> Result<(), String> {
        connect_with_eos(port, Duration::from_secs(1), b"\n", b"\r", refusal).map(drop)
    }

    #[test]
    fn a_gpib_port_refusing_output_eos_still_connects_an_input_fatal_driver() {
        publish("eosGpibInputFatal", Gpib);
        assert!(connect("eosGpibInputFatal", EosRefusal::InputFatal).is_ok());
    }

    #[test]
    fn an_input_eos_refusal_fails_an_input_fatal_driver() {
        publish("eosBareInputFatal", Bare);
        let e = connect("eosBareInputFatal", EosRefusal::InputFatal).unwrap_err();
        assert!(
            e.starts_with("unable to set input EOS on eosBareInputFatal"),
            "{e}"
        );
    }

    #[test]
    fn refusals_are_ignored_where_the_c_driver_ignores_them() {
        publish("eosBareIgnored", Bare);
        assert!(connect("eosBareIgnored", EosRefusal::Ignored).is_ok());
    }

    #[test]
    fn an_output_eos_refusal_fails_a_both_fatal_driver() {
        publish("eosGpibBothFatal", Gpib);
        let e = connect("eosGpibBothFatal", EosRefusal::BothFatal).unwrap_err();
        assert!(
            e.starts_with("unable to set output EOS on eosGpibBothFatal"),
            "{e}"
        );
    }
}
