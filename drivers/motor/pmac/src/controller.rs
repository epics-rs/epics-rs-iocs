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
//! ([`PmacController::deferred`]) — the same object that executes them — so
//! there is exactly one owner of the deferred state and no cross-axis locking.
//! The axis asks the controller whether moves are deferred and, if so, hands it
//! the demand instead of sending one.
//!
//! ## Controller-level PVs (deviation)
//!
//! The C driver exposes `PMAC_C_GLOBALSTATUS`, `PMAC_C_FEEDRATE`,
//! `PMAC_C_FEEDRATE_LIMIT`, `PMAC_C_FEEDRATE_POLL`, `PMAC_C_FEEDRATE_PROBLEM`,
//! `PMAC_C_COMMSERROR` and `PMAC_C_COORDINATE_SYS_GROUP` as asyn parameters on
//! address 0, bound to records by `pmacController.template`. The asyn-rs motor
//! boundary carries no controller-level parameter port, so the two *inputs*
//! among them (feed-rate polling on/off and its limit) become configuration
//! arguments of `pmacCreateController`, and the CS-group selection becomes the
//! `pmacCsGroupSwitch` iocsh command. The *outputs* (global status, feed-rate
//! problem) keep their real effect: they raise the motor record's PROBLEM bit,
//! exactly as they do in C through `pmacAxis::getAxisStatus`.

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;

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

/// How the controller executes a deferred move when the record releases it.
///
/// C reads this from the motor record's `motorDeferMoves_` value (1 =
/// `DEFERRED_FAST_MOVES`, 2 = `DEFERRED_COORDINATED_MOVES`). The asyn-rs motor
/// interface carries deferral as a plain `bool`
/// ([`AsynMotor::set_deferred_moves`][epics_rs::asyn::interfaces::motor::AsynMotor::set_deferred_moves]),
/// so the *mode* is chosen once, at `pmacCreateController` time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeferredMode {
    /// C `DEFERRED_FAST_MOVES`: one combined jog line, axes start together but
    /// are not interpolated.
    Fast,
    /// C `DEFERRED_COORDINATED_MOVES`: motion program 101 in the axes' shared
    /// coordinate system — all axes start *and* stop together.
    Coordinated,
}

impl DeferredMode {
    /// Parse the C numeric selector (1 = fast, 2 = coordinated).
    pub fn from_code(code: i64) -> Result<Self, String> {
        match code {
            1 => Ok(Self::Fast),
            2 => Ok(Self::Coordinated),
            _ => Err(format!(
                "deferredMode must be 1 (fast) or 2 (coordinated), got {code}"
            )),
        }
    }
}

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
    handle.write_octet(0, command.as_bytes())?;
    let raw = handle.read_octet(0, PMAC_MAXBUF)?;
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
    handle: SyncIOHandle,
    /// Axes created with `pmacCreateAxis`/`pmacCreateAxes` (1-based; address 0
    /// is reserved for controller parameters in C and is never an axis).
    axes: BTreeSet<i32>,
    config: BTreeMap<i32, AxisConfig>,

    deferred_mode: DeferredMode,
    moves_deferred: bool,
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
}

impl PmacController {
    pub fn new(
        handle: SyncIOHandle,
        deferred_mode: DeferredMode,
        feed_rate_poll: bool,
        feed_rate_limit: i32,
        global_ttl: Duration,
    ) -> Self {
        Self {
            handle,
            axes: BTreeSet::new(),
            config: BTreeMap::new(),
            deferred_mode,
            moves_deferred: false,
            deferred: BTreeMap::new(),
            poll_cache: BTreeMap::new(),
            cs_groups: CsGroups::new(),
            feed_rate_poll,
            feed_rate_limit,
            feed_rate: 0,
            global_problem: false,
            feed_rate_problem: false,
            global_read_at: None,
            global_ttl,
        }
    }

    /// C `pmacController::lowLevelWriteRead`; see [`octet_write_read`].
    pub fn write_read(&self, command: &str) -> AsynResult<String> {
        octet_write_read(&self.handle, command)
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
            None => return,
        }
        if !self.feed_rate_poll {
            return;
        }
        if let Some(rate) = self
            .write_read("%")
            .ok()
            .as_deref()
            .and_then(parse_feedrate)
        {
            self.feed_rate = rate;
            self.feed_rate_problem = rate < self.feed_rate_limit - FEEDRATE_DEADBAND;
        }
    }

    /// The last polled global feed rate (C `PMAC_C_FEEDRATE`), 0 when feed-rate
    /// polling is off.
    pub fn feed_rate(&self) -> i32 {
        self.feed_rate
    }

    // ---- deferred moves ----

    pub fn moves_deferred(&self) -> bool {
        self.moves_deferred
    }

    pub fn deferred_mode(&self) -> DeferredMode {
        self.deferred_mode
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

    /// C `pmacController::writeInt32`, `motorDeferMoves_` branch: releasing the
    /// deferral executes the pending moves in the configured mode.
    pub fn set_deferred_moves(&mut self, defer: bool) -> AsynResult<()> {
        let release = !defer && self.moves_deferred;
        self.moves_deferred = defer;
        if !release {
            return Ok(());
        }
        let result = match self.deferred_mode {
            DeferredMode::Fast => self.process_deferred_moves(),
            DeferredMode::Coordinated => self.process_deferred_coord_moves(),
        };
        // C clears the pending moves whether or not the command succeeded.
        self.deferred.clear();
        result
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deferred_mode_from_code() {
        assert_eq!(DeferredMode::from_code(1).unwrap(), DeferredMode::Fast);
        assert_eq!(
            DeferredMode::from_code(2).unwrap(),
            DeferredMode::Coordinated
        );
        assert!(DeferredMode::from_code(0).is_err());
        assert!(DeferredMode::from_code(3).is_err());
    }
}
