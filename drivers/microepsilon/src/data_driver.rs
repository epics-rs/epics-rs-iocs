//! L1 native data-port `PortDriver`, ported from `capaNCDT6200Sup.c`'s
//! `readerThread`/`int32Write`/`int32Read`/`capaNCDT6200Configure`/`setLink`.
//! The C driver has no StreamDevice and no `asynDrvUser` interface at all --
//! every one of the 18 data-db records binds via `@asyn($(PORT) addr 0)` (no
//! drvInfo string), so [`DataDriver::drv_user_create`] dispatches purely by
//! `addr`, unlike [`crate::config_driver`]'s name-keyed reason table.
//!
//! # Addressing (`capaNCDT6200Sup.c:34-51`)
//!
//! Float64, push-only via I/O Intr (no read/write asynFloat64 methods at all
//! -- `float64Methods = {}`, `capaNCDT6200Sup.c:566`):
//! `A_DISP_CHAN{1-4}` = addr 1-4, all sharing [`DataDriver::disp_reason`]
//! (one asynFloat64 interrupt source in C, addr-filtered per client --
//! mirrored here as one asyn-rs param, addr-filtered by the framework's own
//! `InterruptFilter`).
//!
//! Int32 write-only (no `int32Read` case for these addresses; a read attempt
//! falls through to `int32Read`'s default "Bad int32 read address" branch):
//! `A_MEAS_RANGE_CHAN{1-4}` = addr 5-8, `A_PV_THROTTLE` = addr 9,
//! `A_NUM_MEAS_CHANNELS` = addr 40 (write ALSO lazily spawns the reader
//! thread, once, matching `readerTID == 0` gating in `int32Write`).
//!
//! Int32 read-only (no `int32Write` case for these addresses):
//! `A_DATA_PACKET_GOOD_COUNT`=20, `_BAD_READ_COUNT`=21, `_BAD_COUNT`=22 (bad
//! *preamble* count, despite the name), `_TIMEOUT_COUNT`=23,
//! `_OUT_OF_SEQUENCE_COUNT`=24, `A_DUPLICATE_DATA_PACKET_COUNT`=25,
//! `A_MISSED_DATA_PACKET_COUNT`=26, `A_MEASURED_COUNT`=27 (the raw
//! last-decoded-packet counter -- not one of [`packet::LinkStats`]'s fields).
//!
//! # Preserved quirks (continuing packet.rs's 1-4: channel masks, raw
//! truncation, stale buffer, f32-precision scaling)
//!
//! 5. **`int32Write`/`int32Read` default-case messages**
//!    (`capaNCDT6200Sup.c:613-616,626-630`): an unrecognized write address
//!    gets a bare `"Bad int32 write address"` (no address embedded); an
//!    unrecognized read address gets `"Bad int32 read address {addr}"` (WITH
//!    the address). Reproduced verbatim, not "fixed" to be consistent.
//! 6. **`int32Read`'s dead `!isCommunicating` gate**
//!    (`capaNCDT6200Sup.c:645-649`): guards `address <= 0`, but every real
//!    read address in this db is positive, so the gate can never actually
//!    fire for a db-driven read. Reproduced verbatim as inert dead code
//!    (matching the C source), not removed as "unreachable cleanup".
//! 7. **Uninitialized reader-thread priority** (`capaNCDT6200Sup.c:577-600`):
//!    `int32Write`'s `A_NUM_MEAS_CHANNELS` handler passes a never-assigned
//!    local `priority` to `epicsThreadLowestPriorityLevelAbove` -- undefined
//!    behavior upstream. `std::thread::spawn` has no priority-relative-to
//!    parameter to replicate this onto in the first place, so the reader
//!    thread is simply spawned at the default OS thread priority; this is a
//!    platform-level divergence, not a silently dropped feature.
//! 8. **Per-iteration reconnect bookkeeping collapses to "once per outcome"
//!    here**: C re-runs the `badPacketCount`/`isCommunicating`/maybe-disconnect
//!    block (`capaNCDT6200Sup.c:434-459`) on *every* inner-loop pass,
//!    including a partial read that completed no packet. The only case where
//!    that matters is a `disconnectDevice()` that keeps *failing* while
//!    `badPacketCount >= 2` -- each such pass re-invokes
//!    `processDataPacket(pdpvt, 0)`. `DrvAsynIPPort::disconnect` (asyn-rs
//!    0.22.1) is unconditionally infallible (`ip_port.rs:997-1007`), so that
//!    branch can never loop in this port; running the classify/communicate/
//!    disconnect sequence once per *actual* read outcome (this module's
//!    [`reader_thread`], matching [`packet::apply_read_outcome`]'s calling
//!    convention) is therefore behaviorally equivalent here, not an
//!    observable simplification.
//!
//! # Fixed upstream defects (doc/upstream-c-defects.md)
//!
//! - **#43 -- Inert `IPport` configure argument** (`capaNCDT6200Sup.c:698-707`):
//!   upstream validated the argument non-empty but never read it -- the TCP
//!   port was always hardcoded to [`PROTOCOL_TCP_PORT`] regardless of what
//!   string was passed. [`configure`] now honors it: parsed as the TCP port
//!   number, defaulting to [`PROTOCOL_TCP_PORT`] (10001) when omitted (`""`)
//!   or `"0"`.
//!
//! # Framework gaps (documented, not silently worked around)
//!
//! - **No `interruptAccept` equivalent**: C's `processDataPacket` gates the
//!   averaging/push tail on the global `extern volatile int interruptAccept`
//!   (set once iocInit's scan pass completes). epics-base-rs/asyn-rs 0.22.1
//!   expose no equivalent flag (grepped exhaustively for `interrupt_accept`/
//!   `InterruptAccept` across both crates: zero matches). [`reader_thread`]
//!   always passes `true` to [`packet::process_data_packet`].
//! - **No process-exit hook for driver crates**: C registers
//!   `capaNCDT6200Shutdown` via `epicsAtExit`, itself installed from an
//!   `initHookRegister` callback firing at `initHookAfterIocRunning`
//!   (`capaNCDT6200Sup.c:498-518`), so the reader thread's `for(;;)` checks a
//!   shutdown flag at each blocking point and exits cleanly on IOC shutdown.
//!   epics-base-rs 0.22.1 has a real `init_hook_register`/`InitHookState`
//!   (`epics_base_rs::server::ioc_app::init_hooks`) but no atExit/process-exit
//!   hook exposed to driver crates -- grepped exhaustively (`atexit`/`AtExit`/
//!   `ctrlc`/`signal::`) across epics-base-rs and asyn-rs; the only hits are
//!   `ioc_app.rs`'s own internal Ctrl-C/SIGTERM handling, not registrable by a
//!   driver crate. [`reader_thread`] therefore has no graceful-shutdown gate
//!   and runs until process exit (matching this workspace's
//!   `scaler974::poll_loop` precedent, which also has no shutdown flag) --
//!   but unlike C, the socket is never explicitly closed on IOC exit.
//! - **`PortRuntimeHandle` must outlive `configure()`**: dropping the last
//!   [`PortRuntimeHandle`] closes its internal shutdown channel and the
//!   actor thread exits (empirically confirmed: a `PortHandle` clone
//!   registered via [`asyn_record::register_port`] survives, but every
//!   request through it then fails with "actor channel closed"). This module
//!   retains both runtimes it creates (the outer port and the `_RBK` port)
//!   in [`KEPT_PORT_RUNTIMES`] for the process lifetime via
//!   [`keep_port_runtime_alive`]. **This is a real defect, confirmed at every
//!   hand-rolled `*Init`/`*Config` iocsh command in this workspace that
//!   calls `create_port_runtime` directly**: `iocs/syringepump-ioc/src/main.rs`'s
//!   `TeledyneDInit`/`TeledyneHInit`, `iocs/love-ioc/src/main.rs`'s
//!   `LoveInit`, and `iocs/delaygen-ioc/src/main.rs`'s `DG645Config`/
//!   `ColbyConfig`/`CoherentSdgConfig` (6 call sites total, `rg -n
//!   "create_port_runtime" iocs/` grepped exhaustively) all drop their local
//!   `runtime_handle` at the end of the closure without retaining it
//!   anywhere. Distinct from (not exhibiting this defect): asyn-rs 0.22.1's
//!   own `drvAsynIPPortConfigure`/`drvAsynSerialPortConfigure`/
//!   `prologixGPIBConfigure` (`asyn-rs::iocsh`), which call
//!   `keep_port_runtime` internally and are therefore safe. Out of scope to
//!   fix here (other drivers' IOC crates), flagged for the user; this
//!   module's own [`configure`] and `iocs/microepsilon-ioc`'s
//!   `CapaNCDT6200ConfigInit` both retain their runtimes correctly.

use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::asyn_record;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::AlarmSeverity;

use crate::packet::{self, MAX_CHANNELS, PushUpdate, ReadOutcome, ReaderState};

/// `capaNCDT6200Sup.c:34-37`.
const A_DISP_CHAN1: i32 = 1;
const A_DISP_CHAN4: i32 = 4;
/// `capaNCDT6200Sup.c:38-41`.
const A_MEAS_RANGE_CHAN1: i32 = 5;
const A_MEAS_RANGE_CHAN2: i32 = 6;
const A_MEAS_RANGE_CHAN3: i32 = 7;
const A_MEAS_RANGE_CHAN4: i32 = 8;
/// `capaNCDT6200Sup.c:42`.
const A_PV_THROTTLE: i32 = 9;
/// `capaNCDT6200Sup.c:43-50`.
const A_DATA_PACKET_GOOD_COUNT: i32 = 20;
const A_DATA_PACKET_BAD_READ_COUNT: i32 = 21;
const A_DATA_PACKET_BAD_COUNT: i32 = 22;
const A_DATA_PACKET_TIMEOUT_COUNT: i32 = 23;
const A_DATA_PACKET_OUT_OF_SEQUENCE_COUNT: i32 = 24;
const A_DUPLICATE_DATA_PACKET_COUNT: i32 = 25;
const A_MISSED_DATA_PACKET_COUNT: i32 = 26;
const A_MEASURED_COUNT: i32 = 27;
/// `capaNCDT6200Sup.c:51`.
const A_NUM_MEAS_CHANNELS: i32 = 40;

/// asyn reason used for octet transactions against the `_RBK` transport port
/// (mirrors `config_driver::TRANSPORT_REASON`).
const TRANSPORT_REASON: usize = 0;

/// `pdpvt->cbuf[200]` (`capaNCDT6200Sup.c:93`).
const READ_CHUNK_BUF_LEN: usize = 200;

/// `capaNCDT6200Protocol.h`: `capaNCDT6200_PROTOCOL_TCP_PORT (10001)`. Used
/// by [`configure`] as the default when its `ip_port` argument is omitted or
/// `"0"` (doc/upstream-c-defects.md #43 -- upstream hardcoded this
/// unconditionally instead).
const PROTOCOL_TCP_PORT: u16 = 10001;

/// Internal-only asyn reason: never resolved by [`DataDriver::drv_user_create`]
/// for any real db record. The reader thread (a background OS thread with no
/// `&mut PortDriverBase` access) stages a [`PushUpdate`] into
/// [`SharedState::pending_push`] and signals it by writing this reason back
/// through its own port's [`PortHandle`] -- routing the actual
/// `set_float64_param`/`set_param_status`/`call_param_callbacks` sequence
/// through [`DataDriver::write_int32`] on the actor thread, which alone has
/// the `&mut self` access those calls require.
const PUSH_DISP_REASON: usize = 1000;
/// Placeholder reason for every addr [`DataDriver::drv_user_create`] does not
/// recognize as a disp channel -- dispatch is by `addr` in [`DataDriver::write_int32`]/
/// [`DataDriver::read_int32`], so the exact value is never consulted, only its
/// distinctness from [`PUSH_DISP_REASON`].
const ADDR_DISPATCH_PLACEHOLDER_REASON: usize = 1001;

fn bad_int32_write_address() -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: "Bad int32 write address".into(),
    }
}

fn bad_int32_read_address(addr: i32) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("Bad int32 read address {addr}"),
    }
}

fn readbacks_not_available() -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: "Readbacks not available".into(),
    }
}

fn is_asyn_timeout(e: &AsynError) -> bool {
    matches!(
        e,
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        }
    )
}

/// `addr` resolution for [`DataDriver::drv_user_create`], factored out for
/// unit testing without a live port. `capaNCDT6200Sup.c` has no
/// `asynDrvUser` interface at all, so every addr resolves to *some* reason;
/// only addr 1-4 (the disp channels) resolve to a reason the framework's
/// interrupt-delivery machinery actually filters on.
fn resolve_reason(addr: i32, disp_reason: usize) -> usize {
    match addr {
        A_DISP_CHAN1..=A_DISP_CHAN4 => disp_reason,
        _ => ADDR_DISPATCH_PLACEHOLDER_REASON,
    }
}

/// One process-lifetime home for every [`PortRuntimeHandle`] this module
/// creates (see the module doc's `PortRuntimeHandle` gap note). Retained,
/// never drained -- an IOC's ports live for the process lifetime by design.
static KEPT_PORT_RUNTIMES: OnceLock<Mutex<Vec<PortRuntimeHandle>>> = OnceLock::new();

/// Keep a [`PortRuntimeHandle`] alive for the process lifetime. Dropping the
/// last handle to a port runtime closes its shutdown channel and the actor
/// thread exits -- see the module doc.
pub fn keep_port_runtime_alive(handle: PortRuntimeHandle) {
    KEPT_PORT_RUNTIMES
        .get_or_init(|| Mutex::new(Vec::new()))
        .lock()
        .unwrap()
        .push(handle);
}

/// Cross-thread state, mirroring the scattered fields on C's single unlocked
/// `drvPvt` struct that both `int32Write`/`int32Read` (actor thread) and
/// `readerThread` (background thread) touch.
#[derive(Debug, Clone)]
struct SharedState {
    /// `chan{1-4}MeasRange` (`capaNCDT6200Sup.c:98-101`).
    chan_meas_range: [i32; MAX_CHANNELS],
    /// `pvThrottle` (`capaNCDT6200Sup.c:106`).
    pv_throttle: i32,
    /// `numMeasChansAvail` (`capaNCDT6200Sup.c:129`).
    num_meas_chans_avail: u32,
    /// `rbkLink.isCommunicating` (`capaNCDT6200Sup.c:62`), mirrored here so
    /// `int32Read`'s gate (quirk 6) can see it from the actor thread.
    is_communicating: bool,
    /// `dataPacket{Good,Bad,Timeout,BadRead}Count` +
    /// `{duplicate,missed,dataPacketOutOfSequence}...Count`.
    stats: packet::LinkStats,
    /// `capaNCDT6200Data.measValueCounter`, read directly by `A_MEASURED_COUNT`
    /// (`capaNCDT6200Sup.c:672-674`) -- NOT `pdpvt->measuredCount`, which
    /// Configure zeroes but nothing else in the C source ever touches again.
    measured_count: u32,
    /// Staged by the reader thread, applied by [`DataDriver::write_int32`]'s
    /// [`PUSH_DISP_REASON`] handler on the actor thread.
    pending_push: Option<PushUpdate>,
}

impl SharedState {
    /// `pdpvt->pvThrottle = 5;` (`capaNCDT6200Sup.c:714`); everything else
    /// zero/false/default, matching `callocMustSucceed`'s zeroed `drvPvt`.
    fn new() -> Self {
        SharedState {
            chan_meas_range: [0; MAX_CHANNELS],
            pv_throttle: 5,
            num_meas_chans_avail: 0,
            is_communicating: false,
            stats: packet::LinkStats::default(),
            measured_count: 0,
            pending_push: None,
        }
    }
}

/// microEpsilon capaNCDT6200 L1 data-port driver state.
pub struct DataDriver {
    base: PortDriverBase,
    shared: Arc<Mutex<SharedState>>,
    disp_reason: usize,
    /// Filled in by [`configure`] after this driver's own `create_port_runtime`
    /// call returns -- the reader thread (spawned later, from a write) needs
    /// its own port's [`PortHandle`] to self-signal [`PUSH_DISP_REASON`].
    self_handle: Arc<Mutex<Option<PortHandle>>>,
    /// `readerTID == 0` gate (`capaNCDT6200Sup.c:596`).
    reader_started: bool,
    /// Taken by [`DataDriver::maybe_start_reader`] on first spawn.
    rbk_port_handle: Option<PortHandle>,
}

impl DataDriver {
    pub fn new(
        port_name: &str,
        rbk_port_handle: PortHandle,
        self_handle: Arc<Mutex<Option<PortHandle>>>,
    ) -> AsynResult<Self> {
        // max_addr = 5 covers addr 0..=4: the disp channels (1-4) are the
        // only addresses ever run through the ParamList (`set_float64_param`/
        // `call_param_callbacks`); every other C "subaddress" (5-9, 20-27,
        // 40) bypasses ParamList entirely via direct `SharedState` access, so
        // it needs no ParamList address slot.
        let mut base = PortDriverBase::new(
            port_name,
            5,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let disp_reason = base.create_param("DispChan", ParamType::Float64)?;
        Ok(Self {
            base,
            shared: Arc::new(Mutex::new(SharedState::new())),
            disp_reason,
            self_handle,
            reader_started: false,
            rbk_port_handle: Some(rbk_port_handle),
        })
    }

    /// `int32Write`'s `A_NUM_MEAS_CHANNELS` case (`capaNCDT6200Sup.c:595-611`):
    /// spawn the reader thread on first write only.
    fn maybe_start_reader(&mut self) {
        if self.reader_started {
            return;
        }
        self.reader_started = true;
        let Some(rbk_port_handle) = self.rbk_port_handle.take() else {
            return;
        };
        let shared = self.shared.clone();
        let self_handle = self.self_handle.clone();
        thread::spawn(move || reader_thread(rbk_port_handle, shared, self_handle));
    }

    /// [`PUSH_DISP_REASON`] handler: apply a reader-thread-staged
    /// [`PushUpdate`] to all 4 disp channels in one atomic actor-thread
    /// operation, mirroring C's single `interruptStart`/iterate/`interruptEnd`
    /// pass (`capaNCDT6200Sup.c:203-232`) -- including setting `auxStatus` on
    /// every push, valid or invalid, so a resumed-valid push clears the
    /// alarm the same way an invalid one raised it.
    fn apply_pending_push(&mut self) -> AsynResult<()> {
        let update = self.shared.lock().unwrap().pending_push.take();
        let Some(update) = update else {
            return Ok(());
        };
        let (status, alarm_status_code, alarm_severity) = if update.valid {
            (
                AsynStatus::Success,
                alarm_status::NO_ALARM,
                AlarmSeverity::NoAlarm as u16,
            )
        } else {
            (
                AsynStatus::Error,
                alarm_status::COMM_ALARM,
                AlarmSeverity::Invalid as u16,
            )
        };
        for (i, &value) in update.values.iter().enumerate() {
            let addr = (i + 1) as i32;
            self.base.set_float64_param(self.disp_reason, addr, value)?;
            self.base.set_param_status(
                self.disp_reason,
                addr,
                status,
                alarm_status_code,
                alarm_severity,
            )?;
            self.base.call_param_callbacks(addr)?;
        }
        Ok(())
    }
}

impl PortDriver for DataDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// No `asynDrvUser` interface in C at all -- every addr resolves to
    /// *some* reason (see [`resolve_reason`]), never rejected at bind time.
    fn drv_user_create(&mut self, _drv_info: &str, addr: i32) -> AsynResult<DrvUserInfo> {
        Ok(DrvUserInfo::from_reason(resolve_reason(
            addr,
            self.disp_reason,
        )))
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        if user.reason == PUSH_DISP_REASON {
            return self.apply_pending_push();
        }
        match user.addr {
            A_MEAS_RANGE_CHAN1 => {
                self.shared.lock().unwrap().chan_meas_range[0] = value;
                Ok(())
            }
            A_MEAS_RANGE_CHAN2 => {
                self.shared.lock().unwrap().chan_meas_range[1] = value;
                Ok(())
            }
            A_MEAS_RANGE_CHAN3 => {
                self.shared.lock().unwrap().chan_meas_range[2] = value;
                Ok(())
            }
            A_MEAS_RANGE_CHAN4 => {
                self.shared.lock().unwrap().chan_meas_range[3] = value;
                Ok(())
            }
            A_PV_THROTTLE => {
                self.shared.lock().unwrap().pv_throttle = value;
                Ok(())
            }
            A_NUM_MEAS_CHANNELS => {
                self.shared.lock().unwrap().num_meas_chans_avail = value as u32;
                self.maybe_start_reader();
                Ok(())
            }
            _ => Err(bad_int32_write_address()),
        }
    }

    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let is_communicating = self.shared.lock().unwrap().is_communicating;
        // Quirk 6 (see module doc): dead for every real read address in this
        // db (all positive), reproduced verbatim rather than removed.
        if !is_communicating && user.addr <= 0 {
            return Err(readbacks_not_available());
        }
        let shared = self.shared.lock().unwrap();
        match user.addr {
            A_DATA_PACKET_GOOD_COUNT => Ok(shared.stats.good_count as i32),
            A_DATA_PACKET_BAD_READ_COUNT => Ok(shared.stats.bad_read_count as i32),
            A_DATA_PACKET_BAD_COUNT => Ok(shared.stats.bad_preamble_count as i32),
            A_DATA_PACKET_TIMEOUT_COUNT => Ok(shared.stats.timeout_count as i32),
            A_DATA_PACKET_OUT_OF_SEQUENCE_COUNT => Ok(shared.stats.out_of_sequence_count as i32),
            A_DUPLICATE_DATA_PACKET_COUNT => Ok(shared.stats.duplicate_count as i32),
            A_MISSED_DATA_PACKET_COUNT => Ok(shared.stats.missed_count as i32),
            A_MEASURED_COUNT => Ok(shared.measured_count as i32),
            _ => Err(bad_int32_read_address(user.addr)),
        }
    }
}

/// `capaNCDT6200LongWait` (`capaNCDT6200Sup.c:238-249`): C sleeps in ten
/// 3-second increments (30s total), checking a shutdown flag each time. This
/// port has no shutdown flag to check (see the module doc's process-exit-hook
/// gap), so the ten increments collapse to one sleep of the same total
/// duration.
fn long_wait() {
    thread::sleep(Duration::from_secs(30));
}

/// `readerThread` (`capaNCDT6200Sup.c:254-462`). Runs until process exit (see
/// the module doc's process-exit-hook gap).
fn reader_thread(
    rbk_port_handle: PortHandle,
    shared: Arc<Mutex<SharedState>>,
    self_handle: Arc<Mutex<Option<PortHandle>>>,
) {
    let initial_throttle = shared.lock().unwrap().pv_throttle;
    let mut reader_state = ReaderState::new(initial_throttle);
    let mut buf = [0u8; READ_CHUNK_BUF_LEN];
    // `capaNCDT6200Data.measValueCounter`/`chanNMeasValue` (`capaNCDT6200Sup.c:86`):
    // persistent, only-overwritten-on-a-complete-packet fields `processDataPacket`
    // reads regardless of which iteration's outcome triggered it (quirk 3, see
    // `packet.rs`'s module doc, extended here to the cross-iteration case).
    let mut last_meas_value_counter: u32 = 0;
    let mut last_chan_values = [0.0f64; MAX_CHANNELS];

    loop {
        if !reader_state.is_communicating {
            let connected = rbk_port_handle
                .submit_blocking(RequestOp::Connect, AsynUser::default())
                .is_ok();
            if !connected {
                long_wait();
            }
        }

        let mut tread = 0usize;
        let mut eop_reached = false;
        while !eop_reached {
            // `pdpvt->pvThrottle` is read live every cycle in C (no separate
            // copy); `ReaderState::pv_throttle` is a plain field the reader
            // must keep resynced itself, or a live `A_PV_THROTTLE` write
            // would never take effect.
            reader_state.pv_throttle = shared.lock().unwrap().pv_throttle;

            let want = READ_CHUNK_BUF_LEN - tread;
            let sync_handle = SyncIOHandle::from_handle(
                rbk_port_handle.clone(),
                0,
                Duration::from_secs_f64(reader_state.read_timeout_secs),
            );
            let read_result = sync_handle.read_octet(TRANSPORT_REASON, want);

            let outcome = match read_result {
                Ok(bytes) => {
                    let n = bytes.len();
                    buf[tread..tread + n].copy_from_slice(&bytes);
                    tread += n;
                    let (num_meas_chans_avail, chan_meas_range) = {
                        let s = shared.lock().unwrap();
                        (s.num_meas_chans_avail, s.chan_meas_range)
                    };
                    if tread >= packet::packet_len(num_meas_chans_avail) {
                        eop_reached = true;
                        let decoded = packet::decode_packet(&buf, chan_meas_range);
                        last_meas_value_counter = decoded.header.meas_value_counter;
                        last_chan_values = decoded.chan_values;
                        if packet::preamble_ok(&buf) {
                            Some(ReadOutcome::GoodPacket)
                        } else {
                            Some(ReadOutcome::BadPreamble)
                        }
                    } else {
                        None
                    }
                }
                Err(e) => Some(if is_asyn_timeout(&e) {
                    ReadOutcome::Timeout
                } else {
                    ReadOutcome::ReadError
                }),
            };

            // Quirk 9 (see module doc): only run the classify/communicate/
            // disconnect sequence when this attempt actually produced an
            // outcome -- a partial successful read (no outcome) just keeps
            // accumulating.
            let Some(outcome) = outcome else {
                continue;
            };

            let should_process = packet::apply_read_outcome(&mut reader_state, outcome);
            shared.lock().unwrap().is_communicating = reader_state.is_communicating;

            if let Some(is_valid) = should_process {
                let push = packet::process_data_packet(
                    &mut reader_state,
                    last_meas_value_counter,
                    last_chan_values,
                    is_valid,
                    true, // interruptAccept gap, see module doc
                );
                {
                    let mut s = shared.lock().unwrap();
                    s.stats = reader_state.stats;
                    s.measured_count = last_meas_value_counter;
                }
                if let Some(update) = push {
                    shared.lock().unwrap().pending_push = Some(update);
                    if let Some(h) = self_handle.lock().unwrap().as_ref() {
                        let _ = h.write_int32_blocking(PUSH_DISP_REASON, 0, 0);
                    }
                }
            }

            if !reader_state.is_communicating {
                let disconnected = rbk_port_handle
                    .submit_blocking(RequestOp::Disconnect, AsynUser::default())
                    .is_ok();
                if disconnected {
                    eop_reached = true;
                    long_wait();
                }
            }
        }
    }
}

/// Fixed upstream defect (doc/upstream-c-defects.md #43): upstream validated
/// `IPport` non-empty but never read it, always connecting to hardcoded
/// [`PROTOCOL_TCP_PORT`]. Honor the argument: `""` or `"0"` defaults to
/// [`PROTOCOL_TCP_PORT`], anything else must parse as a `u16` TCP port.
fn resolve_tcp_port(ip_port: &str) -> AsynResult<u16> {
    match ip_port {
        "" | "0" => Ok(PROTOCOL_TCP_PORT),
        other => other.parse().map_err(|_| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("Invalid IPport '{other}'"),
        }),
    }
}

/// `capaNCDT6200Configure` + `setLink` (`capaNCDT6200Sup.c:467-496,682-791`):
/// build the internal `_RBK` transport port (`noAutoConnect=1,
/// noProcessEos=1`) and this driver's own outer port, wire the late-bound
/// self-handle the reader thread needs (see [`DataDriver::self_handle`]), and
/// register only the outer port -- the `_RBK` port stays unregistered/
/// invisible, matching C.
pub fn configure(
    port_name: &str,
    ip_address: &str,
    ip_port: &str,
    trace: Arc<TraceManager>,
) -> AsynResult<()> {
    if port_name.is_empty() || ip_address.is_empty() {
        return Err(AsynError::Status {
            status: AsynStatus::Error,
            message: "Required argument not present".into(),
        });
    }
    let tcp_port = resolve_tcp_port(ip_port)?;

    let host = format!("{ip_address}:{tcp_port} TCP");
    let rbk_port_name = format!("{port_name}_RBK");
    let mut rbk_driver = DrvAsynIPPort::new(&rbk_port_name, &host)?;
    // `drvAsynIPPortConfigure(link->portName, link->hostInfo, priority, 1, 1)`
    // (`capaNCDT6200Sup.c:476`): noAutoConnect=1.
    rbk_driver.base_mut().set_auto_connect(false);
    // noProcessEos=1: no `push_interpose` call -- no EOS interpose installed.
    let (rbk_runtime, _rbk_actor_jh) = create_port_runtime(rbk_driver, RuntimeConfig::default());
    let rbk_port_handle = rbk_runtime.port_handle().clone();

    let self_handle: Arc<Mutex<Option<PortHandle>>> = Arc::new(Mutex::new(None));
    let driver = DataDriver::new(port_name, rbk_port_handle, self_handle.clone())?;
    let (runtime_handle, _actor_jh) = create_port_runtime(driver, RuntimeConfig::default());
    *self_handle.lock().unwrap() = Some(runtime_handle.port_handle().clone());

    asyn_record::register_port(port_name, runtime_handle.port_handle().clone(), trace)?;

    // See the module doc's `PortRuntimeHandle` gap note: both runtimes must
    // outlive this function.
    keep_port_runtime_alive(rbk_runtime);
    keep_port_runtime_alive(runtime_handle);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addresses_match_c_defines() {
        assert_eq!(A_DISP_CHAN1, 1);
        assert_eq!(A_DISP_CHAN4, 4);
        assert_eq!(A_MEAS_RANGE_CHAN1, 5);
        assert_eq!(A_MEAS_RANGE_CHAN2, 6);
        assert_eq!(A_MEAS_RANGE_CHAN3, 7);
        assert_eq!(A_MEAS_RANGE_CHAN4, 8);
        assert_eq!(A_PV_THROTTLE, 9);
        assert_eq!(A_DATA_PACKET_GOOD_COUNT, 20);
        assert_eq!(A_DATA_PACKET_BAD_READ_COUNT, 21);
        assert_eq!(A_DATA_PACKET_BAD_COUNT, 22);
        assert_eq!(A_DATA_PACKET_TIMEOUT_COUNT, 23);
        assert_eq!(A_DATA_PACKET_OUT_OF_SEQUENCE_COUNT, 24);
        assert_eq!(A_DUPLICATE_DATA_PACKET_COUNT, 25);
        assert_eq!(A_MISSED_DATA_PACKET_COUNT, 26);
        assert_eq!(A_MEASURED_COUNT, 27);
        assert_eq!(A_NUM_MEAS_CHANNELS, 40);
    }

    #[test]
    fn resolve_tcp_port_defaults_on_empty_or_zero() {
        assert_eq!(resolve_tcp_port("").unwrap(), PROTOCOL_TCP_PORT);
        assert_eq!(resolve_tcp_port("0").unwrap(), PROTOCOL_TCP_PORT);
    }

    #[test]
    fn resolve_tcp_port_honors_explicit_value() {
        assert_eq!(resolve_tcp_port("10002").unwrap(), 10002);
        assert_eq!(resolve_tcp_port("1").unwrap(), 1);
        assert_eq!(resolve_tcp_port("65535").unwrap(), 65535);
    }

    #[test]
    fn resolve_tcp_port_rejects_unparsable_value() {
        assert!(resolve_tcp_port("not-a-port").is_err());
        assert!(resolve_tcp_port("70000").is_err());
        assert!(resolve_tcp_port("-1").is_err());
    }

    #[test]
    fn shared_state_defaults_match_c_constructor() {
        let s = SharedState::new();
        assert_eq!(s.pv_throttle, 5);
        assert_eq!(s.chan_meas_range, [0; MAX_CHANNELS]);
        assert_eq!(s.num_meas_chans_avail, 0);
        assert!(!s.is_communicating);
        assert_eq!(s.measured_count, 0);
        assert!(s.pending_push.is_none());
    }

    #[test]
    fn resolve_reason_maps_disp_addresses_to_the_shared_disp_reason() {
        for addr in A_DISP_CHAN1..=A_DISP_CHAN4 {
            assert_eq!(resolve_reason(addr, 7), 7);
        }
    }

    #[test]
    fn resolve_reason_maps_every_other_address_to_the_placeholder() {
        for addr in [
            0,
            A_MEAS_RANGE_CHAN1,
            A_PV_THROTTLE,
            A_DATA_PACKET_GOOD_COUNT,
            A_NUM_MEAS_CHANNELS,
            99,
        ] {
            assert_eq!(resolve_reason(addr, 7), ADDR_DISPATCH_PLACEHOLDER_REASON);
        }
    }

    #[test]
    fn push_disp_reason_and_placeholder_reason_are_distinct() {
        assert_ne!(PUSH_DISP_REASON, ADDR_DISPATCH_PLACEHOLDER_REASON);
    }

    fn make_test_driver() -> (DataDriver, PortRuntimeHandle) {
        struct DummyPort {
            base: PortDriverBase,
        }
        impl PortDriver for DummyPort {
            fn base(&self) -> &PortDriverBase {
                &self.base
            }
            fn base_mut(&mut self) -> &mut PortDriverBase {
                &mut self.base
            }
        }
        let dummy = DummyPort {
            base: PortDriverBase::new("dummy_rbk_for_test", 1, PortFlags::default()),
        };
        let (rbk_runtime, _jh) = create_port_runtime(dummy, RuntimeConfig::default());
        let rbk_handle = rbk_runtime.port_handle().clone();
        let driver =
            DataDriver::new("test_data_port", rbk_handle, Arc::new(Mutex::new(None))).unwrap();
        (driver, rbk_runtime)
    }

    #[test]
    fn write_int32_meas_range_addresses_write_shared_state_only() {
        let (mut driver, _keep_alive) = make_test_driver();
        let mut user = AsynUser::new(0).with_addr(A_MEAS_RANGE_CHAN2);
        driver.write_int32(&mut user, 137).unwrap();
        assert_eq!(driver.shared.lock().unwrap().chan_meas_range[1], 137);
    }

    #[test]
    fn write_int32_pv_throttle_writes_shared_state() {
        let (mut driver, _keep_alive) = make_test_driver();
        let mut user = AsynUser::new(0).with_addr(A_PV_THROTTLE);
        driver.write_int32(&mut user, 12).unwrap();
        assert_eq!(driver.shared.lock().unwrap().pv_throttle, 12);
    }

    #[test]
    fn write_int32_num_meas_channels_starts_the_reader_exactly_once() {
        let (mut driver, _keep_alive) = make_test_driver();
        let mut user = AsynUser::new(0).with_addr(A_NUM_MEAS_CHANNELS);
        driver.write_int32(&mut user, 4).unwrap();
        assert!(driver.reader_started);
        assert!(driver.rbk_port_handle.is_none());
        // Second write must not attempt to re-take the (already-taken) handle.
        driver.write_int32(&mut user, 4).unwrap();
        assert_eq!(driver.shared.lock().unwrap().num_meas_chans_avail, 4);
    }

    #[test]
    fn write_int32_unrecognized_address_matches_c_message_verbatim() {
        let (mut driver, _keep_alive) = make_test_driver();
        let mut user = AsynUser::new(0).with_addr(A_DISP_CHAN1);
        let err = driver.write_int32(&mut user, 0).unwrap_err();
        match err {
            AsynError::Status { message, .. } => assert_eq!(message, "Bad int32 write address"),
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn read_int32_returns_stats_counters() {
        let (mut driver, _keep_alive) = make_test_driver();
        driver.shared.lock().unwrap().stats.duplicate_count = 3;
        let user = AsynUser::new(0).with_addr(A_DUPLICATE_DATA_PACKET_COUNT);
        assert_eq!(driver.read_int32(&user).unwrap(), 3);
    }

    #[test]
    fn read_int32_measured_count_reads_last_packet_counter_not_link_stats() {
        let (mut driver, _keep_alive) = make_test_driver();
        driver.shared.lock().unwrap().measured_count = 55;
        let user = AsynUser::new(0).with_addr(A_MEASURED_COUNT);
        assert_eq!(driver.read_int32(&user).unwrap(), 55);
    }

    #[test]
    fn read_int32_unrecognized_address_embeds_the_address_verbatim() {
        let (mut driver, _keep_alive) = make_test_driver();
        let user = AsynUser::new(0).with_addr(A_PV_THROTTLE);
        let err = driver.read_int32(&user).unwrap_err();
        match err {
            AsynError::Status { message, .. } => assert_eq!(message, "Bad int32 read address 9"),
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn read_int32_dead_gate_does_not_fire_for_any_real_positive_address() {
        // Quirk 6: with is_communicating left false (the constructor
        // default), a real (positive) address must NOT hit the
        // "Readbacks not available" gate -- it falls through to the normal
        // switch, matching capaNCDT6200Sup.c:645-649 exactly.
        let (mut driver, _keep_alive) = make_test_driver();
        let user = AsynUser::new(0).with_addr(A_DATA_PACKET_GOOD_COUNT);
        assert_eq!(driver.read_int32(&user).unwrap(), 0);
    }

    #[test]
    fn read_int32_dead_gate_fires_only_for_a_non_positive_address() {
        let (mut driver, _keep_alive) = make_test_driver();
        let user = AsynUser::new(0).with_addr(0);
        let err = driver.read_int32(&user).unwrap_err();
        match err {
            AsynError::Status { message, .. } => assert_eq!(message, "Readbacks not available"),
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn drv_user_create_maps_disp_addresses_to_the_shared_reason() {
        let (mut driver, _keep_alive) = make_test_driver();
        let disp_reason = driver.disp_reason;
        for addr in A_DISP_CHAN1..=A_DISP_CHAN4 {
            let info = driver.drv_user_create("", addr).unwrap();
            assert_eq!(info.reason, disp_reason);
        }
    }

    #[test]
    fn drv_user_create_never_rejects_any_address() {
        let (mut driver, _keep_alive) = make_test_driver();
        for addr in [-5, 0, 41, 999] {
            assert!(driver.drv_user_create("", addr).is_ok());
        }
    }
}
