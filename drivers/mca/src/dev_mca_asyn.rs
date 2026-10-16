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
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::device_support::{DeviceInitOutcome, DeviceReadOutcome, DeviceSupport};
use epics_rs::base::server::record::{
    DbLinkType, DevSup, Dset, Record, ScanType, register_dbd_devices,
};
use epics_rs::base::types::{EpicsValue, SharedArray};

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::services::PortServices;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use mca_rs::record::{McaCommand, McaRecord, McaStatus, commands};

use crate::interface::{ASYN_MCA_DTYP, McaReason};

/// One mca record's devMcaAsyn state.
pub struct DevMcaAsyn {
    /// The record's INP, parsed in [`DeviceSupport::init`] as C
    /// `init_record` does.
    inp: String,
    /// `pmca->name`, for `init_record`'s messages.
    name: String,
    /// `None` until `init` connects, and for good after it fails: C leaves
    /// that record in PACT (`devMcaAsyn.c:208-210`), so it never processes.
    port: Option<McaPort>,
}

/// `mcaAsynPvt` (`devMcaAsyn.c:38-56`) of a connected record.
struct McaPort {
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

/// C `asynPrint(pasynUser, ASYN_TRACE_ERROR, ...)` for a user on `port`
/// (`None`: a user no port accepted).
fn trace_error(port: Option<&str>, addr: i32, reason: usize, args: std::fmt::Arguments<'_>) {
    PortServices::global().trace().print(
        port,
        port.map(|_| addr),
        reason as i32,
        TraceMask::ERROR,
        args,
    );
}

impl DevMcaAsyn {
    /// The device support for a record whose INP is `inp`; nothing is
    /// connected before [`DeviceSupport::init`].
    pub fn new(inp: &str) -> Self {
        DevMcaAsyn {
            inp: inp.to_owned(),
            name: String::new(),
            port: None,
        }
    }

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

    /// C `init_record`'s `parseLink`, `connectDevice` and the 21
    /// `findDrvInfo` calls (`devMcaAsyn.c:121-204`), with C's messages. Any
    /// failure is C's `goto bad`.
    fn connect(&self) -> Connect {
        let link = match parse_asyn_link(&self.inp) {
            Ok(link) => link,
            Err(e) => {
                errlog_vprintf(format_args!(
                    "devMcaAsyn::init_record {} bad link {e}\n",
                    self.name
                ));
                return Connect::NoPort;
            }
        };
        let Some(entry) = get_port(&link.port_name) else {
            trace_error(
                None,
                link.addr,
                0,
                format_args!(
                    "devMcaAsyn::init_record, {} connectDevice failed to {}",
                    self.name, link.port_name
                ),
            );
            return Connect::NoPort;
        };
        let mut port = McaPort {
            handle: entry.handle,
            addr: link.addr,
            reasons: [0; McaReason::COUNT],
            status: McaStatus::default(),
            data: SharedArray::default(),
        };
        // `pasynUser->reason` as the previous create left it.
        let mut reason = 0;
        for command in McaReason::ALL {
            let req = DrvUserRequest::new(command.drv_info(), port.addr);
            match port.handle.drv_user_create_blocking(&req) {
                Ok(info) => {
                    reason = info.reason;
                    port.reasons[command as usize] = info.reason;
                }
                Err(_) => {
                    trace_error(
                        Some(&link.port_name),
                        port.addr,
                        reason,
                        format_args!(
                            "devMcaAsyn::findDrvInfo, {} drvUserCreate failed for {}",
                            self.name,
                            command.drv_info()
                        ),
                    );
                    return Connect::Unresolved(port);
                }
            }
        }
        Connect::Resolved(port)
    }
}

/// Where C `init_record` (`devMcaAsyn.c:100-212`) stopped.
enum Connect {
    Resolved(McaPort),
    /// A `findDrvInfo` failed: the reasons after it stay 0, as `calloc`
    /// left them.
    Unresolved(McaPort),
    /// The user never reached a port.
    NoPort,
}

impl McaPort {
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

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_owned();
    }

    /// C `init_record` (`devMcaAsyn.c:100-230`): `DevMcaAsyn::connect`;
    /// on failure the record is dead, no alarm (C `goto bad`, `pact=1`).
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
        let connect = self.connect();
        let mca = Self::mca(record)?;
        // C's `goto bad` returns 0, so the record still sends every setup
        // command (`mcaRecord.c:463-487`) and only then sits in PACT.
        let port = match &connect {
            Connect::Resolved(port) | Connect::Unresolved(port) => Some(port),
            Connect::NoPort => None,
        };
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
            match port {
                Some(port) => {
                    let _ = port.send_command(command);
                }
                None => trace_error(
                    None,
                    0,
                    0,
                    format_args!(
                        "devMcaAsyn::send_msg: {} error calling queueRequest, \
                         asynManager::queueRequest asynUser not associated with a port",
                        self.name
                    ),
                ),
            }
        }
        let Connect::Resolved(port) = connect else {
            return Ok(DeviceInitOutcome::dead());
        };
        self.port = Some(port);
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
        let Some(port) = self.port.as_mut() else {
            return Ok(DeviceReadOutcome::ok());
        };
        let mca = Self::mca(record)?;

        for command in mca.take_device_requests() {
            let _ = port.send_command(command);
        }

        port.read_status();
        let must_read_spectrum = mca.apply_status(port.status);

        if must_read_spectrum {
            port.read_spectrum(mca.nuse.max(0) as usize);
            mca.land_spectrum_read(EpicsValue::LongArray(port.data.clone()))?;
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
        if let Some(port) = &self.port
            && command == commands::STOP_ACQUIRE
        {
            let _ = port.send_command(McaCommand::StopAcquire);
        }
        Ok(Vec::new())
    }
}

/// `mcaSupport.dbd`'s `device(mca,CONSTANT,devMCA_soft,"Soft Channel")`.
pub static DEV_MCA_SOFT: DevSup = DevSup {
    name: "devMCA_soft",
    choice: SOFT_MCA_DTYP,
    link_type: DbLinkType::Constant,
    dset: Dset::new(6, &[2, 4, 5], false),
};

/// `mcaSupport.dbd`'s `device(mca,INST_IO,devMcaAsyn,"asynMCA")`; the dset
/// (`devMcaAsyn.c:89-97`) fills init_record, send_msg and read_array.
pub static DEV_MCA_ASYN: DevSup = DevSup {
    name: "devMcaAsyn",
    choice: ASYN_MCA_DTYP,
    link_type: DbLinkType::InstIo,
    dset: Dset::new(6, &[2, 4, 5], false),
};

const SOFT_MCA_DTYP: &str = "Soft Channel";

/// mca's whole devList in `mcaSupport.dbd:10-11` order.
static MCA_DEVICES: [DevSup; 2] = [DEV_MCA_SOFT, DEV_MCA_ASYN];

/// Put mca's `mcaSupport.dbd` devList in the DTYP menu and bind it.
pub fn register_dev_mca_asyn(app: IocApplication) -> IocApplication {
    register_dbd_devices("mca", &MCA_DEVICES);
    app.register_dynamic_device_support(mca_device_support)
}

/// `"Soft Channel"` is every record type's choice, and a dynamic factory
/// outranks the built-in soft supports, so only an mca record is served.
fn mca_device_support(ctx: &DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> {
    if ctx.record_type != "mca" {
        return None;
    }
    match ctx.dtyp {
        SOFT_MCA_DTYP => Some(Box::new(mca_rs::SoftMca)),
        ASYN_MCA_DTYP => Some(Box::new(DevMcaAsyn::new(ctx.inp))),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use epics_rs::asyn::asyn_record::register_port;
    use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;
    use epics_rs::base::server::record::Record;

    use super::*;

    /// An asyn MCA driver that logs every int32 write's reason, rejects
    /// every `NumChannels` write and refuses the `refuse` drvInfo string.
    struct LoggingDriver {
        base: PortDriverBase,
        writes: Arc<Mutex<Vec<usize>>>,
        num_channels: usize,
        refuse: Option<&'static str>,
    }

    impl PortDriver for LoggingDriver {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
            if self.refuse == Some(req.drv_info.as_str()) {
                return Err(AsynError::ParamNotFound(req.drv_info.clone()));
            }
            let reason = self.base.params.find_param(&req.drv_info).unwrap();
            Ok(DrvUserInfo::from_reason(reason))
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

    fn init_dev(inp: &str) -> (DevMcaAsyn, McaRecord, DeviceInitOutcome) {
        let mut dev = DevMcaAsyn::new(inp);
        dev.set_record_info("mcaTest:mca1", ScanType::Passive);
        let mut rec = McaRecord {
            nmax: 8,
            nuse: 8,
            ..Default::default()
        };
        rec.init_record(0).unwrap();
        rec.init_record(1).unwrap();
        let outcome = dev.init(&mut rec).unwrap();
        (dev, rec, outcome)
    }

    /// C `init_record`'s `goto bad` on a bad link or an unknown port
    /// (`devMcaAsyn.c:123-136`): the record is dead, with no alarm.
    #[test]
    fn a_bad_link_or_unknown_port_leaves_the_record_dead() {
        for inp in ["", "@asyn(MCATESTNOSUCHPORT)"] {
            let (dev, _, outcome) = init_dev(inp);
            assert!(matches!(outcome, DeviceInitOutcome::Dead { alarm: None }));
            assert!(dev.port.is_none());
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
        let (f, outcome) = fixture_refusing(name, None);
        assert!(matches!(outcome, DeviceInitOutcome::Ok(_)));
        f.writes.lock().unwrap().clear();
        f
    }

    /// A record initialised on a port whose driver refuses `refuse`'s
    /// drvInfo; the writes the init made stay logged.
    fn fixture_refusing(name: &str, refuse: Option<McaReason>) -> (Fixture, DeviceInitOutcome) {
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
            refuse: refuse.map(McaReason::drv_info),
        };
        let (runtime, _actor) = create_port_runtime(driver, RuntimeConfig::default()).unwrap();
        register_port(
            name,
            runtime.port_handle().clone(),
            &InterruptGate::opened(),
        )
        .unwrap();
        let (dev, rec, outcome) = init_dev(&format!("@asyn({name})"));
        let f = Fixture {
            _runtime: runtime,
            dev,
            rec,
            writes,
            reasons,
        };
        (f, outcome)
    }

    /// The int32 setup commands `mcaRecord.c:463-487` sends, in its order.
    const INT32_SETUP: [McaReason; 8] = [
        McaReason::ChannelAdvanceSource,
        McaReason::NumChannels,
        McaReason::Sequence,
        McaReason::Prescale,
        McaReason::PresetLowChannel,
        McaReason::PresetHighChannel,
        McaReason::PresetSweeps,
        McaReason::AcquireMode,
    ];

    /// C's `goto bad` returns 0 (`devMcaAsyn.c:208-210`), so mca
    /// `init_record` still sends every setup command: with the reasons it
    /// resolved, and reason 0 for the ones after the failing `findDrvInfo`.
    #[test]
    fn a_failed_drv_info_still_sends_the_setup_commands() {
        let (f, outcome) = fixture_refusing("MCATESTLASTINFO", Some(McaReason::ElapsedCounts));
        assert!(matches!(outcome, DeviceInitOutcome::Dead { alarm: None }));
        assert!(f.dev.port.is_none());
        let resolved = INT32_SETUP.map(|reason| f.reasons[reason as usize]);
        assert_eq!(*f.writes.lock().unwrap(), resolved);

        let (f, outcome) = fixture_refusing("MCATESTNUSEINFO", Some(McaReason::NumChannels));
        assert!(matches!(outcome, DeviceInitOutcome::Dead { alarm: None }));
        let mut partial = [0; 8];
        partial[0] = f.reasons[McaReason::ChannelAdvanceSource as usize];
        assert_eq!(*f.writes.lock().unwrap(), partial);
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

    /// `mcaSupport.dbd:10-11`: "Soft Channel" is mca DTYP 0 and binds
    /// devMCA_soft, "asynMCA" is 1, and another record type's "Soft Channel"
    /// is left to its own support.
    #[test]
    fn soft_channel_is_the_first_mca_dtyp_and_binds_the_soft_device() {
        use std::collections::HashMap;

        use epics_rs::base::runtime::task::test_block_on;
        use epics_rs::base::server::database::ProcStack;
        use epics_rs::base::server::ioc_builder::IocBuilder;

        let _ = register_dev_mca_asyn(IocApplication::new());
        let db = r#"
record(waveform, "MCASOFTWF") { field(NELM, "4") field(FTVL, "LONG") }
record(mca, "MCASOFT") {
    field(DTYP, "Soft Channel")
    field(INP, "MCASOFTWF")
    field(NMAX, "4")
}
"#;
        test_block_on(async {
            let (name, factory) = mca_rs::mca_record_factory();
            let (db, _) = IocBuilder::new()
                .register_record_type(name, factory)
                .register_dynamic_device_support(mca_device_support)
                .db_string(db, &HashMap::new())
                .unwrap()
                .build()
                .await
                .unwrap();
            let get = |pv: &str| db.get_pv(pv).unwrap();
            assert_eq!(get("MCASOFT.DTYP").to_f64(), Some(0.0));
            db.put_record_field_from_ca_no_notify(
                "MCASOFT",
                "DTYP",
                EpicsValue::String("asynMCA".into()),
            )
            .await
            .unwrap();
            assert_eq!(get("MCASOFT.DTYP").to_f64(), Some(1.0));
            db.put_record_field_from_ca_no_notify(
                "MCASOFT",
                "DTYP",
                EpicsValue::String("Soft Channel".into()),
            )
            .await
            .unwrap();

            db.put_pv("MCASOFTWF", EpicsValue::LongArray(vec![1, 2, 3, 4].into()))
                .await
                .unwrap();
            db.process_record_with_links("MCASOFT", &mut ProcStack::new())
                .await
                .unwrap();
            assert_eq!(
                get("MCASOFT.VAL"),
                EpicsValue::LongArray(vec![1, 2, 3, 4].into())
            );
            let epid = DeviceSupportContext {
                record_type: "epid",
                dtyp: SOFT_MCA_DTYP,
                inp: "",
                out: "",
            };
            assert!(mca_device_support(&epid).is_none());
        });
    }
}
