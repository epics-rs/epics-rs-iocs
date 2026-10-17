//! Octet-port connect helpers shared by the vendor create commands. A
//! controller's asyn port is created first by `drvAsynSerialPortConfigure` or
//! `drvAsynIPPortConfigure`; these look it up by name and wrap it in a
//! blocking [`SyncIOHandle`] with the given command timeout.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

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

/// A model-3 `asynMotorController` constructor's
/// `pasynOctetSyncIO->connect(octetPort, 0, &pasynUserController_, NULL)`:
/// on a missing port it prints `report` on `pasynUserSelf` (the controller
/// port `self_port`, address 0) and the controller keeps the user anyway,
/// so every later transfer on it is refused rather than the create failing.
pub fn connect_controller(
    octet_port: &str,
    timeout: Duration,
    self_port: &str,
    report: std::fmt::Arguments<'_>,
) -> (SyncIOHandle, AsynResult<()>) {
    let (handle, status) =
        SyncIOHandle::connect_port(octet_port, SERIAL_ADDR, timeout, InterfaceType::Octet);
    if status.is_err() {
        MotorTrace::new(self_port, 0).print_self(TraceMask::ERROR, report);
    }
    (handle, status)
}

/// `pasynOctetSyncIO->setInputEos` then `setOutputEos` on a
/// [`connect_controller`] user, both statuses ignored as the model-3
/// constructors that call them right after the connect ignore them
/// (OEMMotorDriver.cpp:54-55, smarActMCS2MotorDriver.cpp:64-65). On a user
/// of a missing port both are refused at `lockPort`.
pub fn set_eos_ignored(handle: &SyncIOHandle, input_eos: &[u8], output_eos: &[u8]) {
    let Ok(port) = handle.port() else {
        return;
    };
    let user = || AsynUser::new(0).with_addr(SERIAL_ADDR);
    let _ = port.set_input_eos_blocking(user(), input_eos);
    let _ = port.set_output_eos_blocking(user(), output_eos);
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

/// Why [`connect_with_eos`] failed, step by step, so a driver whose C prints
/// its own message for a refused EOS (drvESP300.cc:687) can print it.
#[derive(Debug)]
pub enum EosConnectError {
    PortNotFound(String),
    InputEos(String, AsynError),
    OutputEos(String, AsynError),
}

impl std::fmt::Display for EosConnectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PortNotFound(port) => write!(
                f,
                "octet port '{port}' not found (call drvAsynSerialPortConfigure or drvAsynIPPortConfigure first)"
            ),
            Self::InputEos(port, e) => write!(f, "unable to set input EOS on {port}: {e}"),
            Self::OutputEos(port, e) => write!(f, "unable to set output EOS on {port}: {e}"),
        }
    }
}

impl From<EosConnectError> for String {
    fn from(e: EosConnectError) -> Self {
        e.to_string()
    }
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
) -> Result<SyncIOHandle, EosConnectError> {
    let port =
        get_port(port_name).ok_or_else(|| EosConnectError::PortNotFound(port_name.to_string()))?;
    let user = || AsynUser::new(0).with_addr(SERIAL_ADDR);
    let set_input = || {
        port.handle
            .set_input_eos_blocking(user(), input_eos)
            .map_err(|e| EosConnectError::InputEos(port_name.to_string(), e))
    };
    let set_output = || {
        port.handle
            .set_output_eos_blocking(user(), output_eos)
            .map_err(|e| EosConnectError::OutputEos(port_name.to_string(), e))
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
    use epics_rs::asyn::registry::{claim_port, open_owner};
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
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, driver(base), RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        std::mem::forget(rt);
    }

    /// The create's verdict: `Ok(())` when it connected.
    fn connect(port: &str, refusal: EosRefusal) -> Result<(), String> {
        connect_with_eos(port, Duration::from_secs(1), b"\n", b"\r", refusal)
            .map(drop)
            .map_err(String::from)
    }

    #[test]
    fn a_controller_keeps_a_user_of_a_missing_port() {
        let (handle, status) = connect_controller(
            "motorNoSuchPort",
            Duration::from_secs(1),
            "motorNoSuchCtl",
            format_args!("cannot connect"),
        );
        assert!(status.is_err());
        let e = handle.write_read(0, b"?", 16).unwrap_err();
        assert_eq!(e.message(), "asynManager::queueLockPort not connected");
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
