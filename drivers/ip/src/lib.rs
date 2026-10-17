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

use epics_rs::asyn::manager::DeviceUser;
use epics_rs::asyn::port_handle::PortHandle;

/// C `pasynManager->connectDevice(pasynUser, port, addr)`: the port, or the
/// `errorMessage` the failed call left.
pub(crate) fn connect_device(port: &str, addr: i32) -> Result<PortHandle, String> {
    let mut user = DeviceUser::default();
    match user.connect_device(port, addr) {
        Ok(entry) => Ok(entry.handle.clone()),
        Err(_) => Err(user.error_message().to_owned()),
    }
}
