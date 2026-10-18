//! Rust port of the serial (asyn octet) device support in the EPICS
//! `ip` module (`ipApp/src`).
//!
//! Each C device-support file stays per-record device support, bound by DTYP
//! and driving its device over a pre-configured octet port
//! (`drvAsynSerialPortConfigure` / `drvAsynIPPortConfigure`), as in C:
//!
//! | C file | module | device |
//! |---|---|---|
//! | `devMPC.c` | [`mpc`] | MPC / Digitel ion-pump controller |
//! | `devTPG261.c` | [`tpg261`] | Pfeiffer TPG261 / TPG262 gauge controller |
//! | `devTelevac.c` | [`televac`] | Televac vacuum gauge controller |
//! | `devAiMKS.c` | [`mks`] | MKS / HPS SensaVac 937 gauge controller |
//! | `devAiHeidND261.c` | [`nd261`] | Heidenhain ND261 display unit |
//! | `devXxEurotherm.c` | [`eurotherm`] | Eurotherm 800/2000 temperature controller |
//! | `devXxStrParm.c` | [`strparm`] | generic string-parameter serial I/O |

pub mod eurotherm;
pub mod fmt;
pub mod mks;
pub mod mpc;
pub mod nd261;
pub mod scan;
pub mod strparm;
pub mod televac;
pub mod tpg261;
pub mod user_param;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::record::Record;
use epics_rs::base::types::EpicsValue;

/// The body `MPCConvert`, `TPG261Convert`, `TelevacConvert` and
/// `strParmConvert` share: `eslo = 1.0; roff = 0` on an ai. The device writes
/// VAL itself, so the slope only has to stay out of the way.
pub(crate) fn unit_slope(ai: &mut dyn Record) -> CaResult<()> {
    ai.put_field("ESLO", EpicsValue::Double(1.0))?;
    ai.put_field("ROFF", EpicsValue::ULong(0))
}

use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::manager::DeviceUser;
use epics_rs::asyn::port_handle::PortHandle;

/// Why `initCommon` took its `bad:` arm at `connectDevice`/`findInterface`.
pub(crate) enum Unconnected {
    /// `connectDevice` failed, leaving this `errorMessage`.
    Device(String),
    /// The port has no `asynOctet`; C's status is still `asynSuccess`.
    NoOctet,
}

/// C `pasynManager->connectDevice(pasynUser, port, addr)`, then
/// `findInterface(pasynUser, asynOctetType, 1)`.
pub(crate) fn connect_octet(port: &str, addr: i32) -> Result<PortHandle, Unconnected> {
    let mut user = DeviceUser::default();
    let entry = user
        .connect_device(port, addr)
        .map_err(|_| Unconnected::Device(user.error_message().to_owned()))?;
    if !entry.handle.has_interface(InterfaceType::Octet) {
        return Err(Unconnected::NoOctet);
    }
    Ok(entry.handle.clone())
}

/// A registered port without `asynOctet`, as `findInterface` sees it.
#[cfg(test)]
pub(crate) fn no_octet_port(name: &str) {
    use epics_rs::asyn::interfaces::Capability;
    use epics_rs::asyn::manager::{PortManager, claim_port_driver};
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::open_owner;

    struct Int32Only(PortDriverBase);
    impl PortDriver for Int32Only {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn capabilities(&self) -> Vec<Capability> {
            vec![Capability::Int32Read, Capability::Int32Write]
        }
    }
    let driver = Int32Only(PortDriverBase::new(
        name,
        1,
        PortFlags::default(),
        InterfaceMask::UNMIGRATED_ALL,
        InterfaceMask::UNMIGRATED_INTERRUPTS,
    ));
    let runtime = PortManager::new()
        .register_port(claim_port_driver(name).unwrap(), driver, &open_owner())
        .unwrap();
    std::mem::forget(runtime);
}
