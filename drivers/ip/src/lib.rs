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
//! `devXxStrParm.c` sits beside the ip SNL programs whose templates use it
//! ([`snl`], compiled by epics-seq-rs from `src/snl` and run on
//! epics-seq-rs; templates in `db/`). [`strparm`] and the helpers it shares
//! with the devices here ([`fmt`], [`user_param`], `connect_octet`,
//! `unit_slope`) are defined in this crate.
//!
//! # C reference pins
//!
//! Every `file:NNN` citation in the SNL programs and `strparm` resolves in
//! ip at `d00f687`.

pub mod eurotherm;
pub mod fmt;
pub mod mks;
pub mod mpc;
pub mod nd261;
pub mod scan;
pub mod snl;
pub mod strparm;
pub mod televac;
pub mod tpg261;
pub mod user_param;

use std::sync::{Arc, Mutex};

use asyn_rs::error::AsynResult;
use asyn_rs::interfaces::InterfaceType;
use asyn_rs::manager::DeviceUser;
use asyn_rs::port_handle::{PortHandle, QueueOutcome};
use asyn_rs::request::RequestCell;
use asyn_rs::user::AsynUser;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::database::{DeviceDone, DeviceDoneSource};
use epics_rs::base::server::record::Record;
use epics_rs::base::types::EpicsValue;

/// The module root, for an `st.cmd` that names `$(IP)/db/...`.
pub const IP_DIR: &str = env!("CARGO_MANIFEST_DIR");

/// Path to the bundled database template directory.
pub const IP_DB_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/db");

/// The `registrar()` lines of `ipSupport.dbd`: each program becomes
/// startable by the shell's `seq` command, which the first registration
/// adds. Idempotent.
pub fn register_ip_dbd() {
    // C defect fixed: ipSupport.dbd has no registrar(FederalRegistrar), so a
    // C IOC built on it answers `seq &Federal` with "Can't find sequencer".
    snl::federal::FederalRegistrar();
    snl::keithley2k_dmm::Keithley2kDMMRegistrar();
    snl::keithley2k_dmm_mf40::Keithley2kDMM_mf40Registrar();
    snl::keithley65xx_em::Keithley65xxEMRegistrar();
    snl::oxford_cs800::Oxford_CS800Registrar();
}

/// The body `MPCConvert`, `TPG261Convert`, `TelevacConvert` and
/// `strParmConvert` share: `eslo = 1.0; roff = 0` on an ai. The device writes
/// VAL itself, so the slope only has to stay out of the way.
pub fn unit_slope(ai: &mut dyn Record) -> CaResult<()> {
    ai.put_field("ESLO", EpicsValue::Double(1.0))?;
    ai.put_field("ROFF", EpicsValue::ULong(0))
}

/// Why `initCommon` took its `bad:` arm at `connectDevice`/`findInterface`.
pub enum Unconnected {
    /// `connectDevice` failed, leaving this `errorMessage`.
    Device(String),
    /// The port has no `asynOctet`; C's status is still `asynSuccess`.
    NoOctet,
}

/// C `pasynManager->connectDevice(pasynUser, port, addr)`, then
/// `findInterface(pasynUser, asynOctetType, 1)`. `port` is never C's NULL:
/// `parseLink` mallocs it on every success (asynEpicsUtils.c:80, :101).
pub fn connect_octet(port: &str, addr: i32) -> Result<PortHandle, Unconnected> {
    let mut user = DeviceUser::default();
    let entry = user
        .connect_device(Some(port), addr)
        .map_err(|_| Unconnected::Device(user.error_message().to_owned()))?;
    if !entry.handle.has_interface(InterfaceType::Octet) {
        return Err(Unconnected::NoOctet);
    }
    Ok(entry.handle.clone())
}

/// The pass each callback here ends with, C's `dbScanLock(pr);
/// (*prset->process)(pr); dbScanUnlock(pr)`: armed by [`Reentry::queue`] for
/// a port that can block, ended once by that request's `process` or `unrun`.
#[derive(Default)]
pub(crate) struct Reentry(Mutex<Option<Armed>>);

struct Armed {
    done: DeviceDone,
    /// The pass completes a write rather than re-entering a read.
    write: bool,
}

impl Reentry {
    /// `queueRequest` of `cell`, whose body ends the pass armed here. A port
    /// without `ASYN_CANBLOCK` runs the body inside this call, under the
    /// record's lock set, so nothing is armed and [`QueueOutcome::Ran`] tells
    /// the caller to finish in this pass; C processes the record a second
    /// time from inside the first instead.
    pub(crate) fn queue(
        &self,
        handle: &PortHandle,
        cell: &Arc<RequestCell>,
        user: &AsynUser,
        done: &DeviceDoneSource<'_>,
        write: bool,
    ) -> AsynResult<QueueOutcome> {
        if handle.can_block() {
            *self.0.lock().unwrap() = Some(Armed {
                done: done.device_done(),
                write,
            });
        }
        let outcome = handle.queue_request(cell, user);
        if outcome.is_err() {
            self.0.lock().unwrap().take();
        }
        outcome
    }

    /// From the request's `process` or `unrun`. On a port that can block
    /// both run on its thread, which holds no lock set, so the pass runs
    /// there, as C's does, and no full callback band can refuse it.
    pub(crate) fn end(&self) {
        let Some(Armed { done, write }) = self.0.lock().unwrap().take() else {
            return;
        };
        if write {
            done.complete_now();
        } else {
            done.reprocess_now();
        }
    }
}

/// A registered port without `asynOctet`, as `findInterface` sees it.
#[cfg(test)]
pub(crate) fn no_octet_port(name: &str) {
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
    }
    let driver = Int32Only(PortDriverBase::new(
        name,
        1,
        PortFlags::default(),
        InterfaceMask::INT32,
        InterfaceMask::NONE,
    ));
    let runtime = PortManager::new()
        .register_port(claim_port_driver(name).unwrap(), driver, &open_owner())
        .unwrap();
    std::mem::forget(runtime);
}
