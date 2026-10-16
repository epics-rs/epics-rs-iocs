//! `devMcaAsyn` — the mca record's asyn device support, ported from
//! `mcaApp/mcaSrc/devMcaAsyn.c` (417 lines). Binds `mca-rs`'s `McaRecord` to
//! any driver that implements [`crate::interface`]'s asyn MCA contract (an
//! `asynInt32`/`asynFloat64`/`asynInt32Array` port whose `drvInfo` strings
//! resolve via [`crate::interface::McaReason::drv_info`]) — [`crate::fastsweep`]
//! is the first such driver, and round-2 vendor crates (mca-rontec,
//! mca-amptek, ...) are expected to implement the same contract and bind
//! through this same device support unmodified.
//!
//! # Restructuring vs. C
//! C's `send_msg`/`asynCallback` is a two-phase, `pact`/`rdng`/`rdns`-flagged
//! dance: `mcaData`/`mcaReadStatus` requests are queued
//! (`pact=1; rdng|rdns=1`) and only completed (`process()`'d again) once
//! asyn's callback fires. `mca-rs`'s `cycle` module already collapsed that
//! into a synchronous contract for device support authors: every
//! [`crate::interface::McaReason`] other than `Data`/`ReadStatus` is a plain
//! command dispatched via `McaRecord::take_device_requests()`; the status
//! read (`ReadStatus`) is unconditional every cycle (C's `send_msg`
//! effectively always queues `mcaReadStatus` after any of the other
//! commands complete); and whether the spectrum (`Data`) must also be read
//! is decided by `McaRecord::apply_status`'s return value. `Data` and
//! `ReadStatus` therefore never appear in `take_device_requests()`'s output
//! -- seeing either there would be a logic error in `mca-rs` itself, not
//! something this device support needs to guard against.
//!
//! This device support uses blocking asyn calls (`PortHandle::*_blocking`),
//! matching every other synchronous-poll device support in this workspace
//! (e.g. `microepsilon`'s config port) -- record processing already runs on
//! its own thread, and 0.24.0's `PortHandle` blocking path no longer risks
//! the `block_in_place` panic older versions had (see
//! `drivers/microepsilon/src/data_driver.rs`'s `park_on` discussion).
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::device_support::{DeviceInitOutcome, DeviceReadOutcome, DeviceSupport};
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset, Record, register_device_row};
use epics_rs::base::types::{EpicsValue, SharedArray};

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use mca_rs::record::{McaCommand, McaRecord, McaStatus, commands};

use crate::interface::{ASYN_MCA_DTYP, McaReason};

/// `mcaAsynPvt` (`devMcaAsyn.c:38-56`).
pub struct DevMcaAsyn {
    handle: PortHandle,
    /// `pasynUser->addr`, fixed for this record's lifetime by its own INP
    /// link (`connectDevice(pasynUser, port, signal)`, `devMcaAsyn.c:170`).
    /// Every call this device support makes to the driver uses this same
    /// addr -- it is the record's own asyn address, not a per-command
    /// choice.
    addr: i32,
    /// `driverReasons[MAX_MCA_COMMANDS]` (`devMcaAsyn.c:56`), resolved once
    /// in [`DeviceSupport::init`] via 21 `asynDrvUser->create()` calls
    /// (`devMcaAsyn.c:184-204`) in [`McaReason::ALL`] order. Indexed by
    /// `McaReason as usize`.
    reasons: [usize; McaReason::COUNT],
    /// C's `acquiring`/`elapsedLive`/`elapsedReal`/`totalCounts`/`dwellTime`:
    /// C discards every driver status, so a failed read keeps the last value.
    status: McaStatus,
    /// C's `data`/`nread`, kept across reads for the same reason.
    data: SharedArray<i32>,
}

impl DevMcaAsyn {
    fn mca(record: &mut dyn Record) -> CaResult<&mut McaRecord> {
        record
            .as_any_mut()
            .and_then(|any| any.downcast_mut::<McaRecord>())
            .ok_or_else(|| {
                CaError::TypeMismatch(format!(
                    "DTYP \"{ASYN_MCA_DTYP}\" (devMcaAsyn) supports the mca record only"
                ))
            })
    }

    fn write_i32(&self, reason: McaReason, value: i32) -> AsynResult<()> {
        self.handle
            .write_int32_blocking(self.reasons[reason as usize], self.addr, value)
    }

    fn write_f64(&self, reason: McaReason, value: f64) -> AsynResult<()> {
        self.handle
            .write_float64_blocking(self.reasons[reason as usize], self.addr, value)
    }

    /// C `paramList::getInteger` (`asynPortDriver.cpp:301-321`) writes the
    /// type default (`0`) *unconditionally* before attempting the real
    /// read, then separately returns `asynParamUndefined` if the value was
    /// never set -- a status [`Self::read_status`]'s C counterpart
    /// (`devMcaAsyn.c:364-380`, the `mcaReadStatus` branch) never inspects:
    /// every `pasynInt32->read`/`pasynFloat64->read` call there is bare,
    /// its return value discarded. A never-yet-acquired status field (e.g.
    /// `Acquiring` before the first status sample) is therefore a normal
    /// "0" read in C, not an error, so [`AsynError::ParamUndefined`] stores
    /// `0`; any other error leaves `slot` as it was, as a C read that fails
    /// before writing its output does.
    fn read_i32(&self, reason: McaReason, slot: &mut i32) {
        match self
            .handle
            .read_int32_blocking(self.reasons[reason as usize], self.addr)
        {
            Ok(v) => *slot = v,
            Err(AsynError::ParamUndefined(_)) => *slot = 0,
            Err(_) => {}
        }
    }

    /// See [`Self::read_i32`].
    fn read_f64(&self, reason: McaReason, slot: &mut f64) {
        match self
            .handle
            .read_float64_blocking(self.reasons[reason as usize], self.addr)
        {
            Ok(v) => *slot = v,
            Err(AsynError::ParamUndefined(_)) => *slot = 0.0,
            Err(_) => {}
        }
    }

    /// C `send_msg`'s switch (`devMcaAsyn.c:262-330`), restructured: every
    /// variant here is one of C's plain (non-`mcaData`/`mcaReadStatus`)
    /// commands, dispatched as a single blocking write. `StartAcquire`/
    /// `StopAcquire`/`Erase` write `0`, matching C's `parg=NULL -> ivalue=0`
    /// convention for the argument-less commands (`devMcaAsyn.c:264-282`).
    ///
    /// `Data`/`ReadStatus` are structurally unreachable --
    /// `McaRecord::take_device_requests()` never yields them (see module
    /// doc) -- but the match must stay exhaustive against
    /// [`McaCommand`], so they are explicit no-op arms rather than a
    /// wildcard that would silently swallow a real future variant.
    fn send_command(&self, command: McaCommand) -> AsynResult<()> {
        match command {
            McaCommand::Data | McaCommand::ReadStatus => Ok(()),
            McaCommand::StartAcquire => self.write_i32(McaReason::StartAcquire, 0),
            McaCommand::StopAcquire => self.write_i32(McaReason::StopAcquire, 0),
            McaCommand::Erase => self.write_i32(McaReason::Erase, 0),
            McaCommand::ChannelAdvanceSource(v) => {
                self.write_i32(McaReason::ChannelAdvanceSource, v)
            }
            McaCommand::NumChannels(v) => self.write_i32(McaReason::NumChannels, v),
            McaCommand::AcquireMode(v) => self.write_i32(McaReason::AcquireMode, v),
            McaCommand::Sequence(v) => self.write_i32(McaReason::Sequence, v),
            McaCommand::Prescale(v) => self.write_i32(McaReason::Prescale, v),
            McaCommand::PresetSweeps(v) => self.write_i32(McaReason::PresetSweeps, v),
            McaCommand::PresetLowChannel(v) => self.write_i32(McaReason::PresetLowChannel, v),
            McaCommand::PresetHighChannel(v) => self.write_i32(McaReason::PresetHighChannel, v),
            McaCommand::DwellTime(v) => self.write_f64(McaReason::DwellTime, v),
            McaCommand::PresetLiveTime(v) => self.write_f64(McaReason::PresetLiveTime, v),
            McaCommand::PresetRealTime(v) => self.write_f64(McaReason::PresetRealTime, v),
            McaCommand::PresetCounts(v) => self.write_f64(McaReason::PresetCounts, v),
        }
    }

    /// C `asynCallback`'s `mcaReadStatus` branch (`devMcaAsyn.c:349-381`):
    /// trigger a status sample (C writes `0` to the `mcaReadStatus` reason
    /// itself first), then read the five status scalars. Always run, once
    /// per device-support cycle -- C's `mcaReadStatus` is not conditionally
    /// queued (see module doc). Every status is discarded, as in C.
    fn read_status(&mut self) {
        let _ = self.write_i32(McaReason::ReadStatus, 0);
        let mut status = self.status;
        let mut acquiring = i32::from(status.acquiring);
        self.read_i32(McaReason::Acquiring, &mut acquiring);
        status.acquiring = acquiring != 0;
        self.read_f64(McaReason::ElapsedLiveTime, &mut status.elapsed_live);
        self.read_f64(McaReason::ElapsedRealTime, &mut status.elapsed_real);
        self.read_f64(McaReason::ElapsedCounts, &mut status.total_counts);
        self.read_f64(McaReason::DwellTime, &mut status.dwell_time);
        self.status = status;
    }

    /// C `asynCallback`'s `mcaData` branch (`devMcaAsyn.c:355-358`):
    /// `pasynInt32Array->read(..., pPvt->data, pmca->nuse, &pPvt->nread)`,
    /// status discarded.
    fn read_spectrum(&mut self, max_elements: usize) {
        let user = AsynUser::new(self.reasons[McaReason::Data as usize]).with_addr(self.addr);
        if let Ok(result) = self
            .handle
            .submit_blocking(RequestOp::Int32ArrayRead { max_elements }, user)
        {
            self.data = result.int32_array.unwrap_or_default();
        }
    }
}

impl DeviceSupport for DevMcaAsyn {
    fn dtyp(&self) -> &str {
        ASYN_MCA_DTYP
    }

    /// C `init_record` (`devMcaAsyn.c:100-230`): resolve all 21 drvInfo
    /// strings via `asynDrvUser->create()`, in `McaReason::ALL` order
    /// (`findDrvInfo`, `devMcaAsyn.c:184-204`). The link parse and port
    /// lookup that precede this in C (`parseLink`/`connectDevice`,
    /// `devMcaAsyn.c:141-168`) already happened in
    /// [`connect`], which built `self.handle`/
    /// `self.addr` before this device support was ever registered -- unlike
    /// C, a bad link or unknown port cannot reach `init` at all (see
    /// `connect`'s doc).
    ///
    /// Also reproduces the record-level `init_record`'s OWN second half
    /// (`mcaRecord.c:463-487`, "Initialize hardware to agree with the
    /// record"): every setup field's db-loaded value is sent to the driver
    /// once, here, unconditionally -- independent of the runtime
    /// NEWV/`special()` write path ([`McaRecord::take_device_requests`]),
    /// which only yields a field the first time a *later* `caput` changes
    /// it. Without this, a driver never learns a setup field's db-loaded
    /// value (e.g. `NUSE`) until a client explicitly rewrites it after boot.
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        for reason in McaReason::ALL {
            let req = DrvUserRequest::new(reason.drv_info(), self.addr);
            // C `findDrvInfo` failure is a `goto bad` — pact=1, no alarm
            // (`devMcaAsyn.c:184-209`).
            let info: DrvUserInfo = match self.handle.drv_user_create_blocking(&req) {
                Ok(info) => info,
                Err(e) => {
                    eprintln!(
                        "devMcaAsyn::init_record drvInfo '{}': {e}",
                        reason.drv_info()
                    );
                    return Ok(DeviceInitOutcome::dead());
                }
            };
            self.reasons[reason as usize] = info.reason;
        }

        let mca = Self::mca(record)?;
        for command in [
            McaCommand::ChannelAdvanceSource(mca.chas as i32),
            McaCommand::NumChannels(mca.nuse),
            McaCommand::Sequence(mca.seq),
            McaCommand::DwellTime(mca.dwel),
            McaCommand::Prescale(mca.pscl),
            McaCommand::PresetRealTime(mca.prtm),
            McaCommand::PresetLiveTime(mca.pltm),
            McaCommand::PresetCounts(mca.pct),
            McaCommand::PresetLowChannel(mca.pctl),
            McaCommand::PresetHighChannel(mca.pcth),
            McaCommand::PresetSweeps(mca.pswp),
            McaCommand::AcquireMode(mca.mode as i32),
        ] {
            // C discards every one of these send_msg statuses
            // (`mcaRecord.c:464-487`).
            let _ = self.send_command(command);
        }
        Ok(DeviceInitOutcome::ok())
    }

    /// One process cycle, restructured from C's `send_msg` + `asynCallback`
    /// (`devMcaAsyn.c:262-416`) per the module doc: dispatch every pending
    /// command, unconditionally resample status, and read the spectrum only
    /// if the status update says a client asked for it or acquisition just
    /// finished (`McaRecord::apply_status`'s return value -- C's
    /// `rdng`/`pact` two-phase gate collapsed to one bool).
    ///
    /// C's `asynCallback` ignores every driver status (`devMcaAsyn.c:355-391`),
    /// so a failing driver neither alarms the record nor cuts the cycle short.
    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let mca = Self::mca(record)?;

        for command in mca.take_device_requests() {
            let _ = self.send_command(command);
        }

        self.read_status();
        let must_read_spectrum = mca.apply_status(self.status);

        if must_read_spectrum {
            self.read_spectrum(mca.nuse.max(0) as usize);
            mca.land_spectrum_read(EpicsValue::LongArray(self.data.clone()))?;
        }

        Ok(DeviceReadOutcome::ok())
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }

    /// The commands the record sends from its own `process()`: C's
    /// `send_msg(pmca, mcaStopAcquire, NULL)` on a reached ROI preset
    /// (`mcaRecord.c:797-800`).
    fn handle_command(
        &mut self,
        _record: &mut dyn Record,
        command: &str,
        _args: &[EpicsValue],
    ) -> CaResult<Vec<&'static str>> {
        if command == commands::STOP_ACQUIRE {
            let _ = self.send_command(McaCommand::StopAcquire);
        }
        Ok(Vec::new())
    }
}

/// C `init_record`'s link/port half (`devMcaAsyn.c:141-181`):
/// `parseLink` -> `pasynManager->connectDevice` -> `findInterface` (Int32,
/// Float64, Int32Array, DrvUser -- all required). Returns `None` on any
/// failure, which the `register_dynamic_device_support` factory calling
/// this turns into "no device support for this record" -- the closest
/// analogue available to C's `pmca->pact=1` permanent-disable, since a
/// dynamic-device-support factory has no record-alarm side channel of its
/// own. The 21 `findDrvInfo` calls that follow in C are deferred to
/// [`DeviceSupport::init`], which can return a proper `CaResult` `Err`
/// instead of silently disabling the record.
pub fn connect(inp: &str) -> Option<DevMcaAsyn> {
    let link = parse_asyn_link(inp).ok()?;
    let entry = get_port(&link.port_name)?;
    Some(DevMcaAsyn {
        handle: entry.handle,
        addr: link.addr,
        reasons: [0; McaReason::COUNT],
        status: McaStatus::default(),
        data: SharedArray::default(),
    })
}

/// `mcaSupport.dbd`'s `device(mca,INST_IO,devMcaAsyn,"asynMCA")`; the dset
/// (`devMcaAsyn.c:89-97`) fills init_record, send_msg and read_array.
pub static DEV_MCA_ASYN: DevSup = DevSup {
    name: "devMcaAsyn",
    choice: ASYN_MCA_DTYP,
    link_type: DbLinkType::InstIo,
    dset: Dset::new(6, &[2, 4, 5], false),
};

/// Bind every `DTYP "asynMCA"` mca record to the port its INP names, and
/// put `asynMCA` in the mca DTYP menu: a dynamic support adds no menu row,
/// and mca has none of its own, so without the row `DTYP` reads empty.
pub fn register_dev_mca_asyn(app: IocApplication) -> IocApplication {
    register_device_row("mca", &DEV_MCA_ASYN);
    app.register_dynamic_device_support(|ctx: &DeviceSupportContext| {
        if ctx.dtyp != ASYN_MCA_DTYP {
            return None;
        }
        let dev = connect(ctx.inp)?;
        Some(Box::new(dev) as Box<dyn DeviceSupport>)
    })
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
    use epics_rs::base::server::record::Record;

    use super::*;

    /// An asyn MCA driver that logs every int32 write's reason and rejects
    /// every `NumChannels` write.
    struct LoggingDriver {
        base: PortDriverBase,
        writes: Arc<Mutex<Vec<usize>>>,
        num_channels: usize,
    }

    impl PortDriver for LoggingDriver {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
            self.writes.lock().unwrap().push(user.reason);
            if user.reason == self.num_channels {
                return Err(AsynError::Status {
                    status: epics_rs::asyn::error::AsynStatus::Error,
                    message: "rejected".into(),
                });
            }
            self.base.params.set_int32(user.reason, user.addr, value)
        }
    }

    struct Fixture {
        _runtime: PortRuntimeHandle,
        dev: DevMcaAsyn,
        rec: McaRecord,
        writes: Arc<Mutex<Vec<usize>>>,
        reasons: [usize; McaReason::COUNT],
    }

    fn fixture(name: &str) -> Fixture {
        let mut base = PortDriverBase::new(
            name,
            1,
            PortFlags {
                multi_device: false,
                can_block: false,
                destructible: false,
            },
        );
        let reasons = McaReason::create_params(&mut base).unwrap();
        let writes = Arc::new(Mutex::new(Vec::new()));
        let driver = LoggingDriver {
            base,
            writes: Arc::clone(&writes),
            num_channels: reasons[McaReason::NumChannels as usize],
        };
        let (runtime, _actor) = create_port_runtime(driver, RuntimeConfig::default()).unwrap();
        let mut dev = DevMcaAsyn {
            handle: runtime.port_handle().clone(),
            addr: 0,
            reasons: [0; McaReason::COUNT],
            status: McaStatus::default(),
            data: SharedArray::default(),
        };
        let mut rec = McaRecord {
            nmax: 8,
            nuse: 8,
            ..Default::default()
        };
        rec.init_record(0).unwrap();
        rec.init_record(1).unwrap();
        dev.init(&mut rec).unwrap();
        writes.lock().unwrap().clear();
        Fixture {
            _runtime: runtime,
            dev,
            rec,
            writes,
            reasons,
        }
    }

    #[test]
    fn the_record_stop_command_reaches_the_driver() {
        let mut f = fixture("MCATESTSTOP");
        f.dev
            .handle_command(&mut f.rec, commands::STOP_ACQUIRE, &[])
            .unwrap();
        assert_eq!(
            *f.writes.lock().unwrap(),
            [f.reasons[McaReason::StopAcquire as usize]]
        );
    }

    /// C ignores the driver's write status (`devMcaAsyn.c:385-391`): the
    /// rejected write neither fails the cycle nor skips the status read.
    #[test]
    fn a_rejected_driver_write_does_not_cut_the_cycle_short() {
        let mut f = fixture("MCATESTREJECT");
        f.rec.put_field("NUSE", EpicsValue::Long(4)).unwrap();
        f.rec.special("NUSE", true).unwrap();
        f.dev.read(&mut f.rec).unwrap();
        assert_eq!(
            *f.writes.lock().unwrap(),
            [
                f.reasons[McaReason::NumChannels as usize],
                f.reasons[McaReason::ReadStatus as usize],
            ]
        );
    }
}
