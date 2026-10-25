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
//!
//! `devXxStrParm.c` lives in ip-rs, beside the ip SNL programs whose
//! templates use it; [`strparm`] and the helpers it shares with the devices
//! here ([`fmt`], [`user_param`], `connect_octet`, `unit_slope`) are
//! re-exported from there.

pub mod eurotherm;
pub mod mks;
pub mod mpc;
pub mod nd261;
pub mod scan;
pub mod televac;
pub mod tpg261;

pub(crate) use ip_rs::{Unconnected, connect_octet, unit_slope};
pub use ip_rs::{fmt, strparm, user_param};

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
