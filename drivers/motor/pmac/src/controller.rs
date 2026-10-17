//! The PMAC controller: the octet endpoint shared by every axis, plus the
//! controller-wide state the C `pmacController` holds (global status, feed
//! rate, deferred moves, coordinate-system groups, per-axis configuration).
//!
//! Ported from `pmacApp/pmacAsynMotorPortSrc/pmacController.cpp`.
//!
//! ## Deferred-move ownership (structural change)
//!
//! C stores each pending deferred move on the *axis* (`pmacAxis::deferredMove_`
//! and friends) and has the controller reach into every axis object to build
//! the combined move. Here the pending moves live in the controller
//! (`PmacController::deferred`) — the same object that executes them — so
//! there is exactly one owner of the deferred state and no cross-axis locking.
//! The axis asks the controller whether moves are deferred and, if so, hands it
//! the demand instead of sending one.
//!
//! ## Controller-level parameters
//!
//! C's `PMAC_C_*` parameters and `MOTOR_DEFER_MOVES` live at address 0 of the
//! controller port, bound to records by `pmacController.template`:
//! [`PmacControllerParams`] is the `pmacController::writeInt32` that takes
//! them, and the controller posts what its poll reads to the same port.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{
    ControllerParams, FinishedWrite, ParamWrite, WriteHook, WriteValue, index,
};

use crate::cs_groups::{AxisPoll, CsGroups, DeferredMove, fast_deferred_command};
use crate::protocol::{HARDWARE_PROB, parse_feedrate, parse_global_status};

/// Response buffer size (C `PMAC_MAXBUF`).
pub const PMAC_MAXBUF: usize = 1024;

/// Command timeout (C `pmacController::PMAC_TIMEOUT_` = 5.0 s).
pub const PMAC_TIMEOUT: Duration = Duration::from_secs(5);

/// Feed-rate hysteresis (C `PMAC_FEEDRATE_DEADBAND_`).
const FEEDRATE_DEADBAND: i32 = 1;

/// `<BELL>` opens a PMAC error reply (`<BELL>ERRxxx<CR>`), documented in the C
/// `pmacAsynIPPort.c` header.
const BELL: u8 = 0x07;

/// C `DEFERRED_FAST_MOVES`: `MOTOR_DEFER_MOVES` 1 defers moves into one
/// combined jog line; the axes start together but are not interpolated.
pub const DEFERRED_FAST_MOVES: i32 = 1;

/// C `DEFERRED_COORDINATED_MOVES`: `MOTOR_DEFER_MOVES` 2 runs motion program
/// 101 in the axes' shared coordinate system, so all start and stop together.
pub const DEFERRED_COORDINATED_MOVES: i32 = 2;

/// Per-axis configuration set by the iocsh commands after the axis is created.
/// C keeps these on the axis object and mutates them through
/// `pmacDisableLimitsCheck` / `pmacSetOpenLoopEncoderAxis`; the axis object is
/// behind a `dyn AsynMotor` here, so they live on the controller — which is also
/// where the commands can reach them.
#[derive(Debug, Clone, Copy, Default)]
pub struct AxisConfig {
    /// C `limitsCheckDisable_`: skip the poll's "are hardware limits disabled?"
    /// check (which otherwise raises the record's PROBLEM bit).
    pub limits_check_disabled: bool,
    /// C `encoder_axis_`: the axis number an open-loop axis's encoder comes back
    /// on, or 0 for none.
    pub encoder_axis: i32,
    /// The encoder ratio applied when a set-position is forwarded to the encoder
    /// axis. C takes it from the record's `motorEncoderRatio_`
    /// (`SET_ENC_RATIO`), which the asyn-rs motor interface does not carry, so
    /// it is configured alongside the encoder axis (default 1.0, C's value
    /// before the record writes one).
    pub encoder_ratio: f64,
}

pub fn pmac_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Send one command on an octet port and return the reply, stripped of the NUL
/// padding the read leaves behind (the input EOS has already taken the
/// terminating `<ACK>`).
///
/// **Upstream fix.** C `pmacController::lowLevelWriteRead` (and the CS driver's
/// `motorAxisWriteRead`) report success for *any* reply the transport delivered,
/// so a command the controller rejected — `<BELL>ERRxxx<CR>`, the error form its
/// own `pmacAsynIPPort.c` header documents — is silently treated as done. Every
/// move, stop, home and set-position goes through this path, so a rejected
/// command left the record believing it had been accepted. Here a `<BELL>` reply
/// is an error.
pub fn octet_write_read(handle: &SyncIOHandle, command: &str) -> AsynResult<String> {
    let raw = handle.write_read(0, command.as_bytes(), PMAC_MAXBUF)?;
    if raw.first() == Some(&BELL) {
        let text = String::from_utf8_lossy(&raw);
        let text = text.trim_matches(|c: char| c.is_control() || c == '\0');
        return Err(pmac_err(format!(
            "PMAC rejected command \"{command}\": {text}"
        )));
    }
    Ok(String::from_utf8_lossy(&raw).trim_matches('\0').to_string())
}

pub struct PmacController {
    /// `None` when the low-level port refused an EOS set: C keeps the
    /// controller with a NULL `lowLevelPortUser_` and every command fails. A
    /// missing port keeps its unconnected user, which refuses every transfer.
    handle: Option<SyncIOHandle>,
    /// Axes created with `pmacCreateAxis`/`pmacCreateAxes` (1-based; address 0
    /// is reserved for controller parameters in C and is never an axis).
    axes: BTreeSet<i32>,
    config: BTreeMap<i32, AxisConfig>,

    /// C `movesDeferred_`: the last `MOTOR_DEFER_MOVES` value, 0 for none.
    moves_deferred: i32,
    deferred: BTreeMap<i32, DeferredMove>,
    /// Last poll result per axis, for the coordinated-move builder.
    poll_cache: BTreeMap<i32, AxisPoll>,

    cs_groups: CsGroups,

    feed_rate_poll: bool,
    feed_rate_limit: i32,
    feed_rate: i32,
    /// Cached global-status verdict and when it was read. C polls `???` once per
    /// controller poll cycle; the asyn-rs motor boundary polls per axis, so the
    /// read is cached for one poll period to keep the traffic the same.
    global_problem: bool,
    feed_rate_problem: bool,
    global_read_at: Option<Instant>,
    global_ttl: Duration,
    /// The controller port's parameters and, once published, its handle, to
    /// post what the poll reads.
    params: Option<PmacParams>,
    port: Option<PortHandle>,
}

impl PmacController {
    /// C's constructor: no deferral and no feed-rate poll until the
    /// template's records write them.
    pub fn new(handle: Option<SyncIOHandle>, global_ttl: Duration) -> Self {
        Self {
            handle,
            axes: BTreeSet::new(),
            config: BTreeMap::new(),
            moves_deferred: 0,
            deferred: BTreeMap::new(),
            poll_cache: BTreeMap::new(),
            cs_groups: CsGroups::new(),
            feed_rate_poll: false,
            feed_rate_limit: 0,
            feed_rate: 0,
            global_problem: false,
            feed_rate_problem: false,
            global_read_at: None,
            global_ttl,
            params: None,
            port: None,
        }
    }

    /// Post what the poll reads to the controller port from here on.
    pub fn attach_port(&mut self, port: PortHandle) {
        self.port = Some(port);
    }

    /// Whether the low-level port connected: C's `PMAC_C_COMMSERROR` at
    /// construction.
    pub fn connected(&self) -> bool {
        self.handle.is_some()
    }

    /// C `pmacController::lowLevelWriteRead`; see [`octet_write_read`].
    pub fn write_read(&self, command: &str) -> AsynResult<String> {
        let handle = self
            .handle
            .as_ref()
            .ok_or_else(|| pmac_err("no low level asynOctetSyncIO port"))?;
        octet_write_read(handle, command)
    }

    /// Send a command whose reply carries no value (moves, jogs, set-position).
    pub fn command(&self, command: &str) -> AsynResult<()> {
        self.write_read(command).map(|_| ())
    }

    // ---- axis registry / configuration ----

    /// C `pmacCreateAxis`. Axis 0 is reserved for controller parameters.
    pub fn add_axis(&mut self, axis: i32) -> Result<(), String> {
        if axis <= 0 {
            return Err(
                "axis number 0 is not allowed - that asyn address is reserved for \
                 controller-specific parameters"
                    .to_string(),
            );
        }
        if !self.axes.insert(axis) {
            return Err(format!("axis {axis} has already been created"));
        }
        self.config.insert(
            axis,
            AxisConfig {
                encoder_ratio: 1.0,
                ..AxisConfig::default()
            },
        );
        Ok(())
    }

    pub fn has_axis(&self, axis: i32) -> bool {
        self.axes.contains(&axis)
    }

    pub fn axes(&self) -> impl Iterator<Item = i32> + '_ {
        self.axes.iter().copied()
    }

    pub fn axis_config(&self, axis: i32) -> AxisConfig {
        self.config.get(&axis).copied().unwrap_or(AxisConfig {
            encoder_ratio: 1.0,
            ..AxisConfig::default()
        })
    }

    /// C `pmacController::pmacDisableLimitsCheck`. Errors on an axis that was
    /// never created, as C does.
    pub fn disable_limits_check(&mut self, axis: i32) -> Result<(), String> {
        let cfg = self
            .config
            .get_mut(&axis)
            .ok_or_else(|| format!("axis {axis} has not been configured using pmacCreateAxis"))?;
        cfg.limits_check_disabled = true;
        Ok(())
    }

    pub fn disable_limits_check_all(&mut self) {
        for cfg in self.config.values_mut() {
            cfg.limits_check_disabled = true;
        }
    }

    /// C `pmacController::pmacSetOpenLoopEncoderAxis`. Both the axis and the
    /// encoder axis must already exist.
    pub fn set_open_loop_encoder_axis(
        &mut self,
        axis: i32,
        encoder_axis: i32,
        encoder_ratio: f64,
    ) -> Result<(), String> {
        if !self.axes.contains(&encoder_axis) {
            return Err(format!(
                "encoder axis {encoder_axis} has not been configured using pmacCreateAxis"
            ));
        }
        let cfg = self
            .config
            .get_mut(&axis)
            .ok_or_else(|| format!("axis {axis} has not been configured using pmacCreateAxis"))?;
        cfg.encoder_axis = encoder_axis;
        cfg.encoder_ratio = encoder_ratio;
        Ok(())
    }

    // ---- global status ----

    /// C `pmacController::getGlobalStatus` + the `PMAC_C_GLOBALSTATUS` /
    /// `PMAC_C_FEEDRATE_PROBLEM` handling in `pmacController::poll`, refreshed at
    /// most once per poll period. Returns whether the controller has a problem
    /// that must raise the axis PROBLEM bit.
    pub fn controller_problem(&mut self) -> bool {
        let fresh = self
            .global_read_at
            .is_some_and(|t| t.elapsed() < self.global_ttl);
        if !fresh {
            self.global_read_at = Some(Instant::now());
            self.refresh_global();
        }
        self.global_problem || self.feed_rate_problem
    }

    fn refresh_global(&mut self) {
        match self
            .write_read("???")
            .ok()
            .as_deref()
            .and_then(parse_global_status)
        {
            Some(status) => self.global_problem = (status & HARDWARE_PROB) != 0,
            // A failed read leaves the previous verdict, as C does (it sets the
            // comms-error param and keeps the last global status).
            None => return self.post(|p| vec![(p.comms_error, 1)]),
        }
        let rate = self
            .feed_rate_poll
            .then(|| self.write_read("%").ok())
            .flatten()
            .as_deref()
            .and_then(parse_feedrate);
        if let Some(rate) = rate {
            self.feed_rate = rate;
            self.feed_rate_problem = rate < self.feed_rate_limit - FEEDRATE_DEADBAND;
        }
        let feed_rate_read = rate.is_some();
        // C `pmacController::poll`: the feed rate and its verdict are posted
        // only while polled.
        let (global, rate, problem) = (
            i32::from(self.global_problem),
            self.feed_rate,
            i32::from(self.feed_rate_problem),
        );
        self.post(|p| {
            let mut values = vec![(p.global_status, global), (p.comms_error, 0)];
            if feed_rate_read {
                values.extend([(p.feed_rate, rate), (p.feed_rate_problem, problem)]);
            }
            values
        });
    }

    /// Set address-0 parameters and run their callbacks.
    fn post(&self, values: impl FnOnce(&PmacParams) -> Vec<(usize, i32)>) {
        let (Some(port), Some(params)) = (&self.port, &self.params) else {
            return;
        };
        let updates = values(params)
            .into_iter()
            .map(|(reason, value)| ParamSetValue::new(reason, 0, ParamValue::Int32(value)))
            .collect();
        let _ = port.set_params_and_notify_blocking(0, updates);
    }

    /// The last polled global feed rate (C `PMAC_C_FEEDRATE`), 0 when feed-rate
    /// polling is off.
    pub fn feed_rate(&self) -> i32 {
        self.feed_rate
    }

    // ---- deferred moves ----

    pub fn moves_deferred(&self) -> bool {
        self.moves_deferred != 0
    }

    pub fn is_deferred(&self, axis: i32) -> bool {
        self.deferred.contains_key(&axis)
    }

    /// Record a demand for an axis while moves are deferred (C `pmacAxis::move`,
    /// deferred branch).
    pub fn defer_move(&mut self, axis: i32, move_: DeferredMove) {
        self.deferred.insert(axis, move_);
    }

    /// Drop a pending demand (C `pmacAxis::stop` clears `deferredMove_`).
    pub fn clear_deferred(&mut self, axis: i32) {
        self.deferred.remove(&axis);
    }

    /// C `pmacController::writeInt32`, `motorDeferMoves_` branch: 0 releases
    /// the pending moves in the mode the last value armed, then `value` is the
    /// mode.
    pub fn set_moves_deferred(&mut self, value: i32) -> AsynResult<()> {
        let result = match (value, self.moves_deferred) {
            (0, DEFERRED_FAST_MOVES) => self.process_deferred_moves(),
            (0, DEFERRED_COORDINATED_MOVES) => self.process_deferred_coord_moves(),
            _ => Ok(()),
        };
        if value == 0 {
            // C clears the pending moves whether or not the command succeeded.
            self.deferred.clear();
        }
        self.moves_deferred = value;
        result
    }

    /// C `pmacController::writeInt32`, `PMAC_C_FeedRate_` branch: the PMAC
    /// does not answer `%n`.
    pub fn set_feed_rate(&self, value: i32) -> AsynResult<()> {
        self.write_read(&format!("%{value}")).map(drop)
    }

    fn process_deferred_moves(&mut self) -> AsynResult<()> {
        match fast_deferred_command(&self.deferred) {
            Some(cmd) => self.command(&cmd),
            None => Ok(()),
        }
    }

    fn process_deferred_coord_moves(&mut self) -> AsynResult<()> {
        let cmds = self
            .cs_groups
            .coord_move_commands(&self.deferred, &self.poll_cache)
            .map_err(pmac_err)?;
        for cmd in cmds {
            self.command(&cmd)?;
        }
        Ok(())
    }

    // ---- coordinate-system groups ----

    pub fn cs_groups_mut(&mut self) -> &mut CsGroups {
        &mut self.cs_groups
    }

    /// C `pmacCreateCsGroup` + `pmacCsGroupAddAxis` are pure bookkeeping;
    /// `pmacCsGroups::switchToGroup` is the one that talks to the controller.
    pub fn switch_cs_group(&mut self, id: i32) -> AsynResult<()> {
        let cmds = self.cs_groups.switch_commands(id).map_err(pmac_err)?;
        for cmd in cmds {
            self.command(&cmd)?;
        }
        Ok(())
    }

    /// C `pmacCsGroups::abortMotion`, called from `pmacAxis::stop`.
    pub fn abort_cs_motion(&self, axis: i32) -> AsynResult<()> {
        match self.cs_groups.abort_command(axis) {
            Some(cmd) => self.command(&cmd),
            None => Ok(()),
        }
    }

    // ---- poll cache ----

    /// Record what the axis poll saw, for the coordinated-move builder (C reads
    /// `previous_position_` / `moving_` off the axis objects).
    pub fn record_poll(&mut self, axis: i32, poll: AxisPoll) {
        self.poll_cache.insert(axis, poll);
    }
}

/// C `pmacController`'s own parameters (pmacController.cpp:183-191), all
/// read and written at address 0.
#[derive(Clone, Copy, Debug)]
pub struct PmacParams {
    pub global_status: usize,
    pub comms_error: usize,
    pub feed_rate: usize,
    pub feed_rate_limit: usize,
    pub feed_rate_poll: usize,
    pub feed_rate_problem: usize,
    pub coord_sys_group: usize,
}

/// `pmacController`'s part of the controller port: its parameters and
/// `writeInt32`, ahead of the base's.
pub struct PmacControllerParams {
    controller: Arc<Mutex<PmacController>>,
}

impl PmacControllerParams {
    pub fn new(controller: Arc<Mutex<PmacController>>) -> Self {
        Self { controller }
    }
}

/// `pmacController::writeInt32` / `writeFloat64`'s opening FLOW line,
/// which the record's speed, accel and GET_INFO writes reach as well as
/// the port's. It prints once the write is done, not before.
struct PmacWriteHook;

impl WriteHook for PmacWriteHook {
    fn after_write(&self, write: &FinishedWrite<'_>) {
        let method = match write.value {
            WriteValue::Int32(_) => "writeInt32",
            WriteValue::Float64(_) => "writeFloat64",
        };
        write.print(TraceMask::FLOW, format_args!("pmacController::{method}"));
    }
}

fn lock(controller: &Mutex<PmacController>) -> std::sync::MutexGuard<'_, PmacController> {
    controller.lock().unwrap_or_else(|e| e.into_inner())
}

impl ControllerParams for PmacControllerParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let mut create = |name| base.create_param(name, ParamType::Int32);
        create("PMAC_C_FIRSTPARAM")?;
        let params = PmacParams {
            global_status: create("PMAC_C_GLOBALSTATUS")?,
            comms_error: create("PMAC_C_COMMSERROR")?,
            feed_rate: create("PMAC_C_FEEDRATE")?,
            feed_rate_limit: create("PMAC_C_FEEDRATE_LIMIT")?,
            feed_rate_poll: create("PMAC_C_FEEDRATE_POLL")?,
            feed_rate_problem: create("PMAC_C_FEEDRATE_PROBLEM")?,
            coord_sys_group: create("PMAC_C_COORDINATE_SYS_GROUP")?,
        };
        create("PMAC_C_LASTPARAM")?;
        // C's constructor (pmacController.cpp:194-205).
        let mut controller = lock(&self.controller);
        base.set_int32_param(params.comms_error, 0, i32::from(!controller.connected()))?;
        base.set_int32_param(params.global_status, 0, 0)?;
        base.set_int32_param(params.feed_rate_problem, 0, 0)?;
        controller.params = Some(params);
        Ok(())
    }

    /// Address 0 is C's `pAxisZero`, the axis that carries these parameters.
    fn is_controller_axis(&self, addr: i32) -> bool {
        addr == 0
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        Some(Arc::new(PmacWriteHook))
    }

    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        self.write_int32_own(base, user, value).into()
    }
}

/// The driver's own write code, which then passes to the base.
impl PmacControllerParams {
    fn write_int32_own(
        &mut self,
        _base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> AsynResult<()> {
        let mut controller = lock(&self.controller);
        let Some(p) = controller.params else {
            return Ok(());
        };
        match user.reason {
            r if r == p.feed_rate_poll => controller.feed_rate_poll = value != 0,
            // C reads the limit parameter at each poll.
            r if r == p.feed_rate_limit => controller.feed_rate_limit = value,
            r if r == p.feed_rate => return controller.set_feed_rate(value),
            r if r == p.coord_sys_group => return controller.switch_cs_group(value),
            // An axis address reaches the same branch through
            // `PmacAxis::set_deferred_moves`.
            index::DEFER_MOVES if user.addr == 0 => return controller.set_moves_deferred(value),
            _ => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::manager::PortManager;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::motor::devices::register_controller_port;

    use super::*;

    /// `body`'s FLOW lines in `port`'s trace at address 0.
    fn traced(port: &str, body: impl FnOnce()) -> Vec<String> {
        let path = std::env::temp_dir().join(format!("pmac-trace-{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), TraceMask::FLOW);
        trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
        // The writes go to address 0, whose device trace has its own file.
        trace.set_device_trace_file(port, 0, TraceFile::File(file));
        body();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        text.lines().map(str::to_owned).collect()
    }

    /// pmacController.cpp:396, 503: every write prints its method, the
    /// speed, accel and GET_INFO writes the record now sends included.
    #[test]
    fn every_write_prints_pmac_controllers_flow_line() {
        let ctrl = Arc::new(Mutex::new(PmacController::new(None, Duration::ZERO)));
        let params = PmacControllerParams::new(ctrl);
        let port = register_controller_port(&open_owner(), "pmacFlow", 2, params).unwrap();
        let lines = traced("pmacFlow", || {
            for reason in [index::VELOCITY, index::VEL_BASE, index::ACCEL] {
                port.write_float64_blocking(reason, 0, 2.5).unwrap();
            }
            port.write_int32_blocking(index::UPDATE_STATUS, 0, 1)
                .unwrap();
        });
        let mine: Vec<_> = lines
            .iter()
            .filter_map(|l| l.split_once("pmacController::").map(|(_, m)| m))
            .collect();
        assert_eq!(
            mine,
            ["writeFloat64", "writeFloat64", "writeFloat64", "writeInt32"],
            "{lines:?}"
        );
    }

    #[test]
    fn defer_value_arms_the_mode_and_zero_releases_it() {
        let mut c = PmacController::new(None, Duration::ZERO);
        c.set_moves_deferred(DEFERRED_COORDINATED_MOVES).unwrap();
        assert!(c.moves_deferred());
        c.set_moves_deferred(0).unwrap();
        assert!(!c.moves_deferred());
        // Releasing a fast deferral with a pending move sends the jog line,
        // which fails on a controller with no low-level port.
        c.set_moves_deferred(DEFERRED_FAST_MOVES).unwrap();
        c.defer_move(
            1,
            DeferredMove {
                position: 1.0,
                relative: false,
                time_ms: 0.0,
            },
        );
        assert!(c.set_moves_deferred(0).is_err());
        assert!(!c.is_deferred(1));
    }
}
