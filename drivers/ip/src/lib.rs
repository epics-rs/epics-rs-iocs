//! Rust port of the serial (asyn octet) device support in the EPICS
//! `ip` module (`ipApp/src`).
//!
//! Each C device-support file becomes an asyn **port driver** here: the record
//! link no longer carries the command name (`@asyn(port addr)GET_PRESSURE 1`),
//! it binds to a parameter of the port instead. The port drives the device over
//! a pre-configured octet port (`drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`), exactly as the C device support did.
//!
//! Ported so far:
//!
//! | C file | module | device |
//! |---|---|---|
//! | `devMPC.c` | [`mpc`] | MPC / Digitel ion-pump controller |
//! | `devTPG261.c` | [`tpg261`] | Pfeiffer TPG261 / TPG262 gauge controller |
//! | `devTelevac.c` | [`televac`] | Televac vacuum gauge controller |
//! | `devAiMKS.c` | [`mks`] | MKS / HPS SensaVac 937 gauge controller |
//! | `devAiHeidND261.c` | [`nd261`] | Heidenhain ND261 display unit |
//!
//! All device I/O runs on a plain `std::thread` worker owned by the port (see
//! [`worker`]); the asyn write handlers only enqueue onto it. That keeps
//! `SyncIOHandle` calls off the port-actor thread, where the blocking submit
//! path is not usable.

pub mod connect;
pub mod fmt;
pub mod mks;
pub mod mpc;
pub mod nd261;
pub mod runtime;
pub mod televac;
pub mod tpg261;
pub mod worker;

use epics_rs::asyn::error::{AsynError, AsynStatus};

/// Build the asyn error the port drivers return for a device/protocol failure.
pub(crate) fn asyn_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}
