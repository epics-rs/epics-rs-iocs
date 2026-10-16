//! `devMcaAsyn` — the mca record's asyn device support, ported from
//! `mcaApp/mcaSrc/devMcaAsyn.c` (417 lines). Binds `mca-rs`'s `McaRecord` to
//! any driver that implements [`crate::interface`]'s asyn MCA contract (an
//! `asynInt32`/`asynFloat64`/`asynInt32Array` port whose `drvInfo` strings
//! resolve via [`crate::interface::McaReason::drv_info`]) — [`crate::fastsweep`]
//! is the first such driver, and round-2 vendor crates (mca-rontec,
//! mca-amptek, ...) are expected to implement the same contract and bind
//! through this same device support unmodified.
//!
//! The record drives the device half of its cycle through [`McaDset`]:
//! `read_begin` runs [`McaRecord::begin_device_cycle`] on every ask, the
//! ask again after a nested pass included, and `read()` has nothing left.
//! As in C, every message is queued to the port (`devMcaAsyn.c:262-337`).
//! Each read is its own request, reused from a free list, whose body is C's
//! `asynCallback` (`:342-391`): on an `ASYN_CANBLOCK` port the pass ends PACT
//! and the body re-enters the record, and on a port without it the body runs
//! inside the send and the re-entered pass runs nested.
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};

use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::CallbackPriority;
use epics_rs::base::server::database::{DeviceDone, DeviceDoneSource};
use epics_rs::base::server::device_support::{
    DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, ReadStart,
};
use epics_rs::base::server::record::{
    DbLinkType, DevSup, Dset, ProcessContext, Record, ScanType, register_dbd_devices,
};
use epics_rs::base::types::{EpicsValue, SharedArray};

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::{PortHandle, QueueOutcome};
use epics_rs::asyn::request::{CellBody, PortCalls, RequestCell, RequestOp, Unrun};
use epics_rs::asyn::services::PortServices;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use mca_rs::record::{
    McaCommand, McaDset, McaRead, McaRecord, McaStart, McaStatus, Nack, Sent, StampSource, commands,
};

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
    /// This cycle's `TSE`/`TIME`, for the record's read-start stamp.
    clock: StampSource,
}

/// The port side of `mcaAsynPvt` (`devMcaAsyn.c:38-56`): where C's
/// `send_msg` queues.
struct McaLink {
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
}

/// A connected record's `mcaAsynPvt`.
struct McaPort {
    link: McaLink,
    reads: Arc<Reads>,
    /// C `send_msg` duplicates its asynUser for every read it queues
    /// (`devMcaAsyn.c:263-265`), so any number may be out at once. These are
    /// those duplicates, reused: a request is taken from `idle` when sent and
    /// put back by its own `done`. One grows the set only when all are out.
    requests: Vec<(Arc<RequestCell>, Arc<Trip>)>,
    /// Indices into `requests`; its capacity covers every request, so the
    /// port thread's push never allocates.
    idle: Arc<Mutex<Vec<usize>>>,
    /// This pass's completion cell, on a port that can block: the read
    /// queued from it re-enters the record through it. Only `&mut self`
    /// touches it (`get_mut`); the `Mutex` makes the dset `Sync`.
    reentry: Mutex<Option<(DeviceDone, CallbackPriority)>>,
}

/// What C's `asynCallback` reads into `mcaAsynPvt` and the record copies
/// out on the pass the callback re-enters.
struct Reads {
    name: String,
    addr: i32,
    reasons: [usize; McaReason::COUNT],
    /// C's `acquiring`/`elapsedLive`/`elapsedReal`/`totalCounts`/`dwellTime`:
    /// C discards every driver status, so a failed read keeps the last value.
    status: Mutex<McaStatus>,
    /// C's `data`/`nread`, kept across reads for the same reason.
    data: Mutex<SharedArray<i32>>,
    /// `pmca->nuse`, the channel count C's `mcaData` callback reads
    /// (`devMcaAsyn.c:357`).
    nuse: AtomicI32,
}

/// C `asynCallback` (`devMcaAsyn.c:342-391`) for one queued read.
struct Trip {
    reads: Arc<Reads>,
    /// C's `pasynUser->userData`: what this request reads and the pass it
    /// re-enters, its own and no other request's.
    message: Mutex<Message>,
    index: usize,
    idle: Arc<Mutex<Vec<usize>>>,
}

struct Message {
    read: McaRead,
    reentry: Option<(DeviceDone, CallbackPriority)>,
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
            clock: StampSource::default(),
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
        let mut port = McaLink {
            handle: entry.handle,
            addr: link.addr,
            reasons: [0; McaReason::COUNT],
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
    Resolved(McaLink),
    /// A `findDrvInfo` failed: the reasons after it stay 0, as `calloc`
    /// left them.
    Unresolved(McaLink),
    /// The user never reached a port.
    NoPort,
}

impl McaLink {
    /// C `send_msg`'s `queueRequest` of a plain command, whose callback
    /// makes the one write (`devMcaAsyn.c:383-390`).
    fn send(&self, reason: McaReason, op: RequestOp) -> AsynResult<()> {
        let user = AsynUser::new(self.reasons[reason as usize]).with_addr(self.addr);
        if self.handle.can_block() {
            self.handle.try_submit(op, user).map(drop)
        } else {
            self.handle.submit_blocking(op, user).map(drop)
        }
    }

    fn write_i32(&self, reason: McaReason, value: i32) -> AsynResult<()> {
        self.send(reason, RequestOp::Int32Write { value })
    }

    fn write_f64(&self, reason: McaReason, value: f64) -> AsynResult<()> {
        self.send(reason, RequestOp::Float64Write { value })
    }

    /// C `send_msg`'s switch (`devMcaAsyn.c:262-330`) for the plain
    /// commands. `StartAcquire`/`StopAcquire`/`Erase` write `0`, matching
    /// C's `parg=NULL -> ivalue=0` convention for the argument-less
    /// commands (`devMcaAsyn.c:264-282`).
    ///
    /// `Data`/`ReadStatus` are reads, which [`McaDset::send_read`] queues as
    /// cells; they stay explicit no-op arms so a new [`McaCommand`] variant
    /// fails to compile instead of being swallowed.
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
}

impl McaPort {
    fn new(link: McaLink, name: &str) -> Self {
        let reads = Arc::new(Reads {
            name: name.to_owned(),
            addr: link.addr,
            reasons: link.reasons,
            status: Mutex::new(McaStatus::default()),
            data: Mutex::new(SharedArray::default()),
            nuse: AtomicI32::new(0),
        });
        let mut port = McaPort {
            link,
            reads,
            requests: Vec::new(),
            idle: Arc::new(Mutex::new(Vec::new())),
            reentry: Mutex::new(None),
        };
        // A cycle has at most a status and a data read out at once.
        for _ in 0..2 {
            let index = port.add_request();
            port.idle.lock().unwrap().push(index);
        }
        port
    }

    fn add_request(&mut self) -> usize {
        let index = self.requests.len();
        let trip = Arc::new(Trip {
            reads: Arc::clone(&self.reads),
            message: Mutex::new(Message {
                read: McaRead::Status,
                reentry: None,
            }),
            index,
            idle: Arc::clone(&self.idle),
        });
        let cell = self.link.handle.new_cell(trip.clone());
        self.requests.push((cell, trip));
        let mut idle = self.idle.lock().unwrap();
        let missing = self.requests.len() - idle.len();
        idle.reserve(missing);
        index
    }

    /// C `send_msg`'s `duplicateAsynUser` and `queueRequest` of a read
    /// (`devMcaAsyn.c:263-337`), re-entering the record through `reentry`
    /// when it is queued.
    fn queue(
        &mut self,
        read: McaRead,
        reentry: Option<(DeviceDone, CallbackPriority)>,
    ) -> Result<QueueOutcome, Nack> {
        let free = self.idle.lock().unwrap().pop();
        let index = free.unwrap_or_else(|| self.add_request());
        let (cell, trip) = &self.requests[index];
        *trip.message.lock().unwrap() = Message { read, reentry };
        let user = AsynUser::new(0).with_addr(self.link.addr);
        self.link.handle.queue_request(cell, &user).map_err(|_| {
            // Never queued, so no `done` returns it.
            trip.message.lock().unwrap().reentry = None;
            self.idle.lock().unwrap().push(index);
            Nack
        })
    }
}

impl Reads {
    /// C `paramList::getInteger` (`asynPortDriver.cpp:301-321`) writes the
    /// type default (`0`) *unconditionally* before attempting the real
    /// read, then separately returns `asynParamUndefined` if the value was
    /// never set -- a status C's `mcaReadStatus` callback
    /// (`devMcaAsyn.c:364-380`) never inspects: every
    /// `pasynInt32->read`/`pasynFloat64->read` call there is bare, its
    /// return value discarded. A never-yet-acquired status field (e.g.
    /// `Acquiring` before the first status sample) is therefore a normal
    /// "0" read in C, not an error, so [`AsynError::ParamUndefined`] stores
    /// `0`; any other error leaves `slot` as it was, as a C read that fails
    /// before writing its output does.
    fn read_i32(&self, port: &mut dyn PortCalls, reason: McaReason, slot: &mut i32) {
        match port.call(&RequestOp::Int32Read, self.user(reason)) {
            Ok(got) => *slot = got.int_val.unwrap_or(*slot),
            Err(AsynError::ParamUndefined(_)) => *slot = 0,
            Err(_) => {}
        }
    }

    /// See [`Self::read_i32`].
    fn read_f64(&self, port: &mut dyn PortCalls, reason: McaReason, slot: &mut f64) {
        match port.call(&RequestOp::Float64Read, self.user(reason)) {
            Ok(got) => *slot = got.float_val.unwrap_or(*slot),
            Err(AsynError::ParamUndefined(_)) => *slot = 0.0,
            Err(_) => {}
        }
    }

    fn user(&self, reason: McaReason) -> AsynUser {
        AsynUser::new(self.reasons[reason as usize]).with_addr(self.addr)
    }

    /// The `mcaReadStatus` branch (`devMcaAsyn.c:359-381`): trigger a
    /// status sample (C writes `0` to the `mcaReadStatus` reason itself
    /// first), then read the five status scalars.
    fn read_status(&self, port: &mut dyn PortCalls) {
        let write = RequestOp::Int32Write { value: 0 };
        let _ = port.call(&write, self.user(McaReason::ReadStatus));
        let mut status = *self.status.lock().unwrap();
        let mut acquiring = i32::from(status.acquiring);
        self.read_i32(port, McaReason::Acquiring, &mut acquiring);
        status.acquiring = acquiring != 0;
        self.read_f64(port, McaReason::ElapsedLiveTime, &mut status.elapsed_live);
        self.read_f64(port, McaReason::ElapsedRealTime, &mut status.elapsed_real);
        self.read_f64(port, McaReason::ElapsedCounts, &mut status.total_counts);
        self.read_f64(port, McaReason::DwellTime, &mut status.dwell_time);
        *self.status.lock().unwrap() = status;
    }

    /// The `mcaData` branch (`devMcaAsyn.c:355-358`):
    /// `pasynInt32Array->read(..., pPvt->data, pmca->nuse, &pPvt->nread)`.
    fn read_spectrum(&self, port: &mut dyn PortCalls) {
        let max_elements = self.nuse.load(Ordering::Relaxed).max(0) as usize;
        let op = RequestOp::Int32ArrayRead { max_elements };
        if let Ok(got) = port.call(&op, self.user(McaReason::Data)) {
            *self.data.lock().unwrap() = got.int32_array.unwrap_or_default();
        }
    }
}

impl CellBody for Trip {
    fn process(&self, port: &mut dyn PortCalls) {
        let read = self.message.lock().unwrap().read;
        match read {
            McaRead::Status => self.reads.read_status(port),
            McaRead::Data => self.reads.read_spectrum(port),
        }
        // C's `dbScanLock; prset->process` (`:358-360`, `:378-380`).
        let reentry = self.message.lock().unwrap().reentry.take();
        if let Some((done, band)) = reentry
            && done.reprocess(band).is_err()
        {
            errlog_vprintf(format_args!(
                "devMcaAsyn::asynCallback: {} callback queue full\n",
                self.reads.name
            ));
        }
    }

    /// C's duplicate has no `timeoutUser` (`devMcaAsyn.c:263`), so a read on
    /// a disconnected device still runs its callback (asynManager.c:886).
    fn timeout_user(&self) -> bool {
        false
    }

    /// C arms no queue timeout and keeps a request queued while its port is
    /// disabled, so nothing here re-enters the record.
    fn unrun(&self, _why: Unrun) {
        self.message.lock().unwrap().reentry = None;
    }

    /// C's `freeAsynUser` (`:391`): the request is free for the next send.
    fn done(self: Arc<Self>) {
        self.idle.lock().unwrap().push(self.index);
    }
}

/// C `send_msg`'s status (`devMcaAsyn.c:331-337`): -1 only when
/// `queueRequest` refuses; `asynCallback` discards the driver's own status.
fn queued(result: AsynResult<()>) -> Result<(), Nack> {
    match result {
        Err(e) if e.is_queue_refused() => Err(Nack),
        _ => Ok(()),
    }
}

/// C `send_msg` (`devMcaAsyn.c:229-338`) and `read_array` (`:395-407`).
impl McaDset for McaPort {
    /// The record sends its reads, a MODE put's `mcaData` included, through
    /// [`McaDset::send_read`].
    fn send_msg(&mut self, command: McaCommand) -> Result<(), Nack> {
        queued(self.link.send_command(command))
    }

    /// Every read a pass sends re-enters the record: C duplicates the
    /// asynUser per `send_msg` (`devMcaAsyn.c:263`), so a MODE put's queued
    /// `mcaData` and the status read after it each process it.
    fn send_read(&mut self, read: McaRead) -> Result<Sent, Nack> {
        let reentry = self.reentry.get_mut().unwrap().clone();
        Ok(match self.queue(read, reentry)? {
            QueueOutcome::Queued => Sent::Queued,
            QueueOutcome::Ran => Sent::Ran,
        })
    }

    /// C's second `send_msg(mcaReadStatus)` copies what the callback read
    /// (`devMcaAsyn.c:245-260`).
    fn read_status(&mut self, _mca: &McaRecord) -> Result<McaStatus, Nack> {
        Ok(*self.reads.status.lock().unwrap())
    }

    /// C copies whatever the last `mcaData` callback left in its buffer.
    fn read_array(&mut self, mca: &mut McaRecord) -> Result<(), Nack> {
        let data = self.reads.data.lock().unwrap().clone();
        mca.land_spectrum_read(EpicsValue::LongArray(data))
            .map_err(|_| Nack)
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
    /// once, here, unconditionally -- the record's NEWV path sends a field
    /// only after a *later* put changes it, so without this a driver never
    /// learns a setup field's db-loaded value (e.g. `NUSE`).
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        let connect = self.connect();
        let mca = Self::mca(record)?;
        // C's `goto bad` returns 0, so the record still sends every setup
        // command (`mcaRecord.c:463-487`) and only then sits in PACT.
        let link = match &connect {
            Connect::Resolved(link) | Connect::Unresolved(link) => Some(link),
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
            match link {
                Some(link) => {
                    let _ = link.send_command(command);
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
        let Connect::Resolved(link) = connect else {
            return Ok(DeviceInitOutcome::dead());
        };
        self.port = Some(McaPort::new(link, &self.name));
        Ok(DeviceInitOutcome::ok())
    }

    /// C `mcaRecord.c:507-790` up to the first read still out, on every
    /// pass, the re-entered ones included.
    fn read_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        let clock = self.clock;
        let Some(port) = self.port.as_mut() else {
            return Ok(ReadStart::Synchronous);
        };
        let mca = Self::mca(record)?;
        port.reads.nuse.store(mca.nuse, Ordering::Relaxed);
        *port.reentry.get_mut().unwrap() = port
            .link
            .handle
            .can_block()
            .then(|| (done.device_done(), done.priority()));
        Ok(match mca.begin_device_cycle(port, clock, done.again()) {
            McaStart::Synchronous => ReadStart::Synchronous,
            McaStart::Pending => ReadStart::Pending,
            McaStart::Reentered => ReadStart::Reentered,
        })
    }

    /// `read_begin` ran the whole device half.
    fn read(&mut self, _record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        Ok(DeviceReadOutcome::ok())
    }

    fn set_process_context(&mut self, ctx: &ProcessContext) {
        self.clock = ctx.into();
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
            let _ = port.link.send_command(McaCommand::StopAcquire);
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
        SOFT_MCA_DTYP => Some(Box::new(mca_rs::SoftMca::default())),
        ASYN_MCA_DTYP => Some(Box::new(DevMcaAsyn::new(ctx.inp))),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicUsize;

    use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
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
        /// Spectrum reads, each answering `1, 2, ...`.
        array_reads: Arc<AtomicUsize>,
        read_status: usize,
        /// Holds each read's first driver call until the test lets it run.
        gate: Option<Arc<Gate>>,
    }

    /// Permits for the port thread, one per read.
    #[derive(Default)]
    struct Gate {
        permits: Mutex<usize>,
        opened: std::sync::Condvar,
    }

    impl Gate {
        fn open(&self, n: usize) {
            *self.permits.lock().unwrap() += n;
            self.opened.notify_all();
        }

        fn pass(&self) {
            let mut permits = self.permits.lock().unwrap();
            while *permits == 0 {
                permits = self.opened.wait(permits).unwrap();
            }
            *permits -= 1;
        }
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
            if user.reason == self.read_status
                && let Some(gate) = &self.gate
            {
                gate.pass();
            }
            self.writes.lock().unwrap().push(user.reason);
            if user.reason == self.num_channels {
                return Err(AsynError::Status {
                    status: epics_rs::asyn::error::AsynStatus::Error,
                    message: "rejected".into(),
                });
            }
            self.base.set_int32_param(user.reason, user.addr, value)
        }

        fn read_int32_array(&mut self, _user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
            if let Some(gate) = &self.gate {
                gate.pass();
            }
            self.array_reads.fetch_add(1, Ordering::SeqCst);
            for (i, v) in buf.iter_mut().enumerate() {
                *v = i as i32 + 1;
            }
            Ok(buf.len())
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
        array_reads: Arc<AtomicUsize>,
        reasons: [usize; McaReason::COUNT],
    }

    fn fixture(name: &str) -> Fixture {
        let (f, outcome) = fixture_refusing(name, None);
        assert!(matches!(outcome, DeviceInitOutcome::Ok(_)));
        f.writes.lock().unwrap().clear();
        f
    }

    /// A registered port: its runtime, the driver's write and spectrum-read
    /// logs, and its reasons.
    type DriverPort = (
        PortRuntimeHandle,
        Arc<Mutex<Vec<usize>>>,
        Arc<AtomicUsize>,
        [usize; McaReason::COUNT],
    );

    /// The driver on a registered port `name`; with `down`, a multi-device
    /// port whose device `down` is disconnected and does not autoconnect.
    fn driver_port(
        name: &str,
        can_block: bool,
        refuse: Option<McaReason>,
        gate: Option<Arc<Gate>>,
        down: Option<i32>,
    ) -> DriverPort {
        let mut base = PortDriverBase::new(
            name,
            2,
            PortFlags {
                multi_device: down.is_some(),
                can_block,
                destructible: false,
            },
        );
        if let Some(addr) = down {
            base.set_auto_connect_addr(addr, false);
            base.set_addr_connected(addr, false);
        }
        let reasons = McaReason::create_params(&mut base).unwrap();
        let writes = Arc::new(Mutex::new(Vec::new()));
        let array_reads = Arc::new(AtomicUsize::new(0));
        let driver = LoggingDriver {
            base,
            writes: Arc::clone(&writes),
            num_channels: reasons[McaReason::NumChannels as usize],
            refuse: refuse.map(McaReason::drv_info),
            array_reads: Arc::clone(&array_reads),
            read_status: reasons[McaReason::ReadStatus as usize],
            gate,
        };
        let claim = claim_port(name).unwrap();
        let (runtime, _actor) =
            create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(runtime.port_handle().clone(), &InterruptGate::opened())
            .unwrap();
        (runtime, writes, array_reads, reasons)
    }

    /// A record initialised on a port without `ASYN_CANBLOCK` whose driver
    /// refuses `refuse`'s drvInfo; the writes the init made stay logged.
    fn fixture_refusing(name: &str, refuse: Option<McaReason>) -> (Fixture, DeviceInitOutcome) {
        let (runtime, writes, array_reads, reasons) = driver_port(name, false, refuse, None, None);
        let (dev, rec, outcome) = init_dev(&format!("@asyn({name})"));
        let f = Fixture {
            _runtime: runtime,
            dev,
            rec,
            writes,
            array_reads,
            reasons,
        };
        (f, outcome)
    }

    impl Fixture {
        /// One cycle's device half as the framework runs it on this port,
        /// whose reads all run inline: each `Reentered` runs the nested pass
        /// and asks again.
        fn cycle(&mut self) {
            fn pass(rec: &mut McaRecord, port: &mut McaPort) {
                port.reads.nuse.store(rec.nuse, Ordering::Relaxed);
                let clock = StampSource::default();
                let mut start = rec.begin_device_cycle(port, clock, false);
                while start == McaStart::Reentered {
                    pass(rec, port);
                    start = rec.begin_device_cycle(port, clock, true);
                }
            }
            pass(&mut self.rec, self.dev.port.as_mut().unwrap());
        }
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
        f.cycle();
        assert_eq!(
            *f.writes.lock().unwrap(),
            [
                f.reasons[McaReason::NumChannels as usize],
                f.reasons[McaReason::ReadStatus as usize],
            ]
        );
        assert_eq!(f.rec.nack, 0, "send_msg queued it, so C acknowledged");
        assert_eq!(f.array_reads.load(Ordering::SeqCst), 1, "the NUSE read");
    }

    /// C `send_msg` returns -1 when `queueRequest` refuses
    /// (`devMcaAsyn.c:331-337`), so the record raises NACK and, for the
    /// status read, drops ACQG (`mcaRecord.c:670-674`).
    #[test]
    fn a_refused_queue_is_a_nack() {
        let mut f = fixture("MCATESTREFUSE");
        f.dev
            .port
            .as_ref()
            .unwrap()
            .link
            .handle
            .set_enable(-1, false)
            .unwrap();
        f.rec.put_field("NUSE", EpicsValue::Long(4)).unwrap();
        f.rec.special("NUSE", true).unwrap();
        f.rec.acqg = 1;
        f.cycle();
        assert!(f.writes.lock().unwrap().is_empty());
        assert_eq!(f.rec.nack, 1);
        assert_eq!(f.rec.acqg, 0);
    }

    /// A put to `field` through an IOC on both port kinds, once it settles:
    /// the record's VAL and its forward links, and the driver's array reads.
    fn put_through_ioc(
        prefix: &str,
        field: &str,
        value: EpicsValue,
    ) -> [(bool, EpicsValue, Option<f64>, usize); 2] {
        use std::collections::HashMap;
        use std::time::Duration;

        use epics_rs::base::runtime::task::{sleep, test_block_on};
        use epics_rs::base::server::ioc_builder::IocBuilder;

        [true, false].map(|can_block| {
            let port = format!("{prefix}{}", if can_block { "BLK" } else { "SYNC" });
            let (runtime, _, array_reads, _) = driver_port(&port, can_block, None, None, None);
            let db = format!(
                r#"
record(mca, "{prefix}") {{
    field(DTYP, "asynMCA")
    field(INP, "@asyn({port})")
    field(NMAX, "4")
    field(NUSE, "4")
    field(FLNK, "{prefix}N")
}}
record(calc, "{prefix}N") {{ field(CALC, "A+1") field(INPA, "{prefix}N") }}
"#
            );
            let (val, flnks) = test_block_on(async {
                let (name, factory) = mca_rs::mca_record_factory();
                let (db, _) = IocBuilder::new()
                    .register_record_type(name, factory)
                    .register_dynamic_device_support(mca_device_support)
                    .db_string(&db, &HashMap::new())
                    .unwrap()
                    .build()
                    .await
                    .unwrap();
                db.put_record_field_from_ca(prefix, field, value.clone())
                    .await
                    .unwrap();
                for _ in 0..400 {
                    if !db.get_record(prefix).unwrap().read().is_processing() {
                        break;
                    }
                    sleep(Duration::from_millis(5)).await;
                }
                // A further pass would land within this window.
                sleep(Duration::from_millis(100)).await;
                let get = |pv: &str| db.get_pv(pv).unwrap();
                assert!(!db.get_record(prefix).unwrap().read().is_processing());
                assert_eq!(get(&format!("{prefix}.RDNG")).to_f64(), Some(0.0));
                (
                    get(&format!("{prefix}.VAL")),
                    get(&format!("{prefix}N")).to_f64(),
                )
            });
            let reads = array_reads.load(Ordering::SeqCst);
            drop(runtime);
            (can_block, val, flnks, reads)
        })
    }

    /// A spectrum read through an IOC: the status trip, then the data trip
    /// (`mcaRecord.c:670-681`, `:765-781`). On an `ASYN_CANBLOCK` port each
    /// ends its pass PACT and the port thread re-enters the record, which
    /// forward-links once; on a port without it C's three `process` calls
    /// nest and each forward-links (`:824-826`). The driver is read once.
    #[test]
    fn a_read_takes_two_trips_on_both_port_kinds() {
        let spectrum = EpicsValue::LongArray(vec![1, 2, 3, 4].into());
        assert_eq!(
            put_through_ioc("MCATRIP", "READ", EpicsValue::Enum(1)),
            [
                (true, spectrum.clone(), Some(1.0), 1),
                (false, spectrum, Some(3.0), 1),
            ]
        );
    }

    /// A MODE put's PHA is `mcaData` (`mcaRecord.c:630`), which sets RDNG
    /// (`devMcaAsyn.c:287`) and re-enters the record with the spectrum. On
    /// a port without `ASYN_CANBLOCK` the data pass and the status pass nest
    /// in the put's: three links. On a CANBLOCK port the data and status
    /// reads queue together; the data callback's pass links, the status
    /// callback's queues a fresh status read, and that one's pass links.
    #[test]
    fn a_pha_mode_put_reads_the_spectrum_on_both_port_kinds() {
        let spectrum = EpicsValue::LongArray(vec![1, 2, 3, 4].into());
        assert_eq!(
            put_through_ioc("MCAPHA", "MODE", EpicsValue::Enum(0)),
            [
                (true, spectrum.clone(), Some(2.0), 1),
                (false, spectrum, Some(3.0), 1),
            ]
        );
    }

    /// On an `ASYN_CANBLOCK` port every request a cycle makes is queued, as
    /// C's `send_msg` queues them (`devMcaAsyn.c:331`): PINI, a setup-field
    /// put, STOP and a READ never wait on the port thread under the hold.
    #[test]
    fn a_cycle_on_a_blocking_port_never_waits_under_the_hold() {
        use std::collections::HashMap;
        use std::time::Duration;

        use epics_rs::base::runtime::task::{sleep, test_block_on};
        use epics_rs::base::server::ioc_builder::IocBuilder;

        let (runtime, writes, array_reads, reasons) =
            driver_port("MCAHOLD", true, None, None, None);
        let db = r#"
record(mca, "MCAHOLD") {
    field(DTYP, "asynMCA")
    field(INP, "@asyn(MCAHOLD)")
    field(NMAX, "4")
    field(NUSE, "4")
    field(PINI, "YES")
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
            db.ioc_init().await;
            let idle = async || {
                for _ in 0..400 {
                    if !db.get_record("MCAHOLD").unwrap().read().is_processing() {
                        break;
                    }
                    sleep(Duration::from_millis(5)).await;
                }
            };
            idle().await;
            for (field, value) in [
                ("NUSE", EpicsValue::Long(3)),
                ("STOP", EpicsValue::Enum(1)),
                ("READ", EpicsValue::Enum(1)),
            ] {
                db.put_record_field_from_ca("MCAHOLD", field, value)
                    .await
                    .unwrap();
                idle().await;
            }
            assert_eq!(
                db.get_pv("MCAHOLD.VAL").unwrap(),
                EpicsValue::LongArray(vec![1, 2, 3].into())
            );
        });
        let writes = writes.lock().unwrap();
        for reason in [McaReason::NumChannels, McaReason::StopAcquire] {
            assert!(writes.contains(&reasons[reason as usize]), "{reason:?}");
        }
        assert!(array_reads.load(Ordering::SeqCst) >= 1);
        drop(runtime);
    }

    /// C duplicates the asynUser for every read it queues
    /// (`devMcaAsyn.c:263-265`) and each callback re-enters the record only
    /// for its own message: a data read queued ahead of the status read does
    /// not re-enter the status read's pass, a data read after it does not
    /// cancel that re-entry, and a read sent while another is out is queued,
    /// not refused.
    #[test]
    fn each_queued_read_re_enters_only_its_own_pass() {
        use std::collections::HashMap;
        use std::time::Duration;

        use epics_rs::base::runtime::task::{sleep, test_block_on};
        use epics_rs::base::server::ioc_builder::IocBuilder;

        let gate = Arc::new(Gate::default());
        let (runtime, _, array_reads, _) =
            driver_port("MCAOWN", true, None, Some(gate.clone()), None);
        let (mut dev, _, outcome) = init_dev("@asyn(MCAOWN)");
        assert!(matches!(outcome, DeviceInitOutcome::Ok(_)));
        let port = dev.port.as_mut().unwrap();
        test_block_on(async {
            let db = r#"record(calc, "MCAOWNN") { field(CALC, "A+1") field(INPA, "MCAOWNN") }"#;
            let (db, _) = IocBuilder::new()
                .db_string(db, &HashMap::new())
                .unwrap()
                .build()
                .await
                .unwrap();
            let done = DeviceDone::new(&db, &db.get_record("MCAOWNN").unwrap().read());
            let reentry = || Some((done.clone(), done.priority()));
            let reentered = || db.get_pv("MCAOWNN").unwrap().to_f64().unwrap();
            let run = async |n| {
                gate.open(n);
                sleep(Duration::from_millis(100)).await;
            };

            port.queue(McaRead::Data, None).unwrap();
            port.queue(McaRead::Status, reentry()).unwrap();
            run(1).await;
            assert_eq!(array_reads.load(Ordering::SeqCst), 1);
            assert_eq!(reentered(), 0.0, "the data read re-entered the status pass");
            run(1).await;
            assert_eq!(reentered(), 1.0, "the status read re-enters its pass");

            port.queue(McaRead::Status, reentry()).unwrap();
            port.queue(McaRead::Data, None).unwrap();
            run(1).await;
            assert_eq!(
                reentered(),
                2.0,
                "the data read behind it kept its re-entry"
            );
            run(1).await;
            assert_eq!(array_reads.load(Ordering::SeqCst), 2);
            assert_eq!(reentered(), 2.0);
        });
        drop(runtime);
    }

    /// C's read request has no `timeoutUser`, so on a disconnected device
    /// the port thread runs its callback anyway (asynManager.c:886): the
    /// driver is called and the record is re-entered.
    #[test]
    fn a_read_on_a_disconnected_device_still_runs_its_callback() {
        use std::collections::HashMap;
        use std::time::Duration;

        use epics_rs::base::runtime::task::{sleep, test_block_on};
        use epics_rs::base::server::ioc_builder::IocBuilder;

        let (runtime, writes, array_reads, reasons) =
            driver_port("MCADOWN", true, None, None, Some(1));
        let (mut dev, _, outcome) = init_dev("@asyn(MCADOWN,1)");
        assert!(matches!(outcome, DeviceInitOutcome::Ok(_)));
        let port = dev.port.as_mut().unwrap();
        test_block_on(async {
            let db = r#"record(calc, "MCADOWNN") { field(CALC, "A+1") field(INPA, "MCADOWNN") }"#;
            let (db, _) = IocBuilder::new()
                .db_string(db, &HashMap::new())
                .unwrap()
                .build()
                .await
                .unwrap();
            let done = DeviceDone::new(&db, &db.get_record("MCADOWNN").unwrap().read());
            port.queue(McaRead::Data, None).unwrap();
            port.queue(McaRead::Status, Some((done.clone(), done.priority())))
                .unwrap();
            sleep(Duration::from_millis(200)).await;
            assert_eq!(array_reads.load(Ordering::SeqCst), 1);
            assert!(
                writes
                    .lock()
                    .unwrap()
                    .contains(&reasons[McaReason::ReadStatus as usize])
            );
            assert_eq!(db.get_pv("MCADOWNN").unwrap().to_f64(), Some(1.0));
        });
        drop(runtime);
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
