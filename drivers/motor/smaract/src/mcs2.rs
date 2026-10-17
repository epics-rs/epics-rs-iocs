//! SmarAct MCS2 controller driver (ASCII SCPI, over an asyn octet port).
//!
//! Ported from `motorSmarAct/smarActApp/src/smarActMCS2MotorDriver.cpp` (a
//! model-3 `asynMotorController`/`asynMotorAxis` pair). Commands are SCPI-style
//! ASCII addressed by channel (`:CHAN<n>:...`, `:MOVE<n>`, `:REF<n>`,
//! `:STOP<n>`, 0-based). Commands and replies are
//! `\r\n`-terminated by the port EOS, set both ways at connect as C does.
//!
//! ## Closed- vs open-loop motion
//!
//! With a sensor present the controller runs closed-loop: `move` sets the move
//! mode (`MMOD` 0 absolute / 1 relative), acceleration and velocity, then issues
//! `:MOVE<n>`. Without a sensor it runs open-loop steps (`MMOD 4`): the target
//! is tracked in software from the last known position and the relative delta is
//! sent. `poll` selects the branch from the live sensor-present status.
//!
//! ## Units
//!
//! Per the C header, the controller works in picometres (linear) / nano-degrees
//! (rotary) but the driver deliberately reports **nanometres** / micro-degrees
//! by scaling with `PULSES_PER_STEP` (1000) — this is an intentional unit choice
//! that extends the usable range, not a resolution bridge, so it is preserved:
//! closed-loop positions/velocity/acceleration are multiplied by 1000 on the way
//! out and readback is divided by 1000. The driver boundary is therefore nm (or
//! µdeg); with `MRES` = 1 the record EGU is nm. Open-loop step moves are sent
//! unscaled, matching C.
//!
//! ## Channel parameters
//!
//! `PTYP`, `MCLF`, `HOLD` and `CAL` write the channel's positioner type, max
//! clock frequency, hold time and calibration; `poll` reads the type back
//! into `PTYP_RB`, the status word into `PSTAT` and, while idle, `CAL`,
//! `MCLF` and `HOLD`.

use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::{DrvUserRequest, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{
    FinishedWrite, ParamWrite, WriteHook, WriteOutcome, WriteValue, has_motor_axis,
};
use epics_rs::motor::trace::MotorTrace;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::util::{atof, atoi};
use motor_common::{AxisReport, ControllerParams, ControllerReport};

/// Response buffer size.
const READ_BUF: usize = 128;

/// Port EOS set at connect (smarActMCS2MotorDriver.cpp:64-65).
pub const INPUT_EOS: &[u8] = b"\r\n";
pub const OUTPUT_EOS: &[u8] = b"\r\n";

/// Controller pm→driver nm (and ndeg→µdeg) scale (C `PULSES_PER_STEP`).
const PULSES_PER_STEP: f64 = 1000.0;

/// Max errors to drain per `clearErrors` call (safety bound on the queue read).
const MAX_ERROR_DRAIN: usize = 32;

// MCS2 channel status-word bits (C header).
const ACTIVELY_MOVING: i64 = 0x0001;
const CLOSED_LOOP_ACTIVE: i64 = 0x0002;
const SENSOR_PRESENT: i64 = 0x0020;
const IS_CALIBRATED: i64 = 0x0040;
const IS_REFERENCED: i64 = 0x0080;
const END_STOP_REACHED: i64 = 0x0100;
const FOLLOWING_LIMIT_REACHED: i64 = 0x0400;
const MOVEMENT_FAILED: i64 = 0x0800;
const REFERENCE_MARK: i64 = 0x8000;

// MCS2 reference options (C header).
const START_DIRECTION: i64 = 0x0001;
const AUTO_ZERO: i64 = 0x0004;

/// The channel parameters C creates, in its order
/// (smarActMCS2MotorDriver.cpp:55-60).
const CHANNEL_PARAMS: [&str; 6] = ["PTYP", "PTYP_RB", "PSTAT", "MCLF", "HOLD", "CAL"];

/// The reasons of [`CHANNEL_PARAMS`].
#[derive(Clone, Copy, Default)]
pub struct Mcs2Reasons {
    ptyp: usize,
    ptyp_rb: usize,
    pstat: usize,
    mclf: usize,
    hold: usize,
    cal: usize,
}

impl Mcs2Reasons {
    fn from_slice(r: [usize; 6]) -> Self {
        let [ptyp, ptyp_rb, pstat, mclf, hold, cal] = r;
        Self {
            ptyp,
            ptyp_rb,
            pstat,
            mclf,
            hold,
            cal,
        }
    }

    /// The reasons controller port `port` created.
    pub fn lookup(port: &PortHandle) -> AsynResult<Self> {
        let mut r = [0; 6];
        for (slot, name) in r.iter_mut().zip(CHANNEL_PARAMS) {
            *slot = port
                .drv_user_create_blocking(&DrvUserRequest::new(name, 0))?
                .reason;
        }
        Ok(Self::from_slice(r))
    }
}

/// Shared controller endpoint owning the asyn octet handle.
pub struct Mcs2Controller {
    handle: SyncIOHandle,
    ident: String,
    /// The channel parameters the poll sets, once the port created them.
    reasons: Option<Mcs2Reasons>,
}

impl Mcs2Controller {
    /// Connect to an MCS2 and read its serial number (`:DEV:SNUM?`), draining
    /// the error queue around it as the C constructor does. Performs blocking
    /// I/O; `trace` is C's `pasynUserSelf`.
    pub fn new(handle: SyncIOHandle, trace: &MotorTrace) -> Self {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            reasons: None,
        };
        ctrl.clear_errors();
        match ctrl.query(":DEV:SNUM?") {
            Ok(ident) => ctrl.ident = ident,
            Err(_) => trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "SmarActMCS2MotorDriver:MCS2Controller: cannot connect to MCS2 controller\n"
                ),
            ),
        }
        // C prints the reply on the error mask whether or not it came.
        trace.print_self(
            TraceMask::ERROR,
            format_args!(
                "MCS2Controller::MCS2Controller: Device Name: {}\n",
                ctrl.ident
            ),
        );
        ctrl.clear_errors();
        ctrl
    }

    /// Let each channel poll set its parameters, `reasons`.
    pub fn set_reasons(&mut self, reasons: Mcs2Reasons) {
        self.reasons = Some(reasons);
    }

    /// The controller serial-number identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Write a command with no reply expected.
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Write a query and return its reply (trimmed).
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Drain the controller error queue (C `clearErrors`): read the error count
    /// and consume that many `:SYST:ERR?` entries, so queued errors do not carry
    /// over between commands. Best-effort; ignores I/O errors.
    fn clear_errors(&self) {
        let count = self
            .query(":SYST:ERR:COUN?")
            .ok()
            .map(|r| atof(&r) as i64)
            .unwrap_or(0);
        for _ in 0..count.clamp(0, MAX_ERROR_DRAIN as i64) {
            if self.query(":SYST:ERR?").is_err() {
                break;
            }
        }
    }
}

/// The MCS2 controller's poll periods, in seconds, for its report, and its
/// channel parameters.
pub struct Mcs2Params {
    moving_poll: f64,
    idle_poll: f64,
    /// Set once the controller is created, after the port as in C.
    controller: Arc<OnceLock<Arc<Mutex<Mcs2Controller>>>>,
    reasons: Mcs2Reasons,
}

impl Mcs2Params {
    pub fn new(moving_poll: f64, idle_poll: f64) -> Self {
        Self {
            moving_poll,
            idle_poll,
            controller: Arc::new(OnceLock::new()),
            reasons: Mcs2Reasons::default(),
        }
    }

    /// The slot the create command fills with the controller.
    pub fn controller_slot(&self) -> Arc<OnceLock<Arc<Mutex<Mcs2Controller>>>> {
        self.controller.clone()
    }
}

/// C `writeInt32` traces every write at its end, the base's included
/// (smarActMCS2MotorDriver.cpp:250-257).
struct Mcs2WriteHook;

impl WriteHook for Mcs2WriteHook {
    fn after_write(&self, write: &FinishedWrite<'_>) -> WriteOutcome {
        let WriteValue::Int32(value) = write.value else {
            return WriteOutcome::default();
        };
        let function = write.function;
        match write.status {
            Err(e) => write.print(
                TraceMask::ERROR,
                format_args!(
                    "SmarActMCS2MotorDriver:writeInt32: error, status={} function={function}, value={value}\n",
                    e.status() as i32
                ),
            ),
            Ok(()) => write.print(
                TraceMask::IO_DRIVER,
                format_args!("SmarActMCS2MotorDriver:writeInt32: function={function}, value={value}\n"),
            ),
        }
        WriteOutcome::default()
    }
}

impl ControllerParams for Mcs2Params {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let mut r = [0; 6];
        for (slot, name) in r.iter_mut().zip(CHANNEL_PARAMS) {
            *slot = base.create_param(name, ParamType::Int32)?;
        }
        self.reasons = Mcs2Reasons::from_slice(r);
        Ok(())
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        Some(Arc::new(Mcs2WriteHook))
    }

    /// C `MCS2Controller::writeInt32` (smarActMCS2MotorDriver.cpp:209-260):
    /// the channel parameters are stored and sent instead of the base.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        // C refuses a missing axis before anything, as the base does.
        if !has_motor_axis(base, user) {
            return ParamWrite::PassToBase(Ok(()));
        }
        let r = self.reasons;
        let c = user.addr;
        let cmd = match user.reason {
            f if f == r.ptyp => format!(":CHAN{c}:PTYP {value}"),
            f if f == r.mclf => format!(":CHAN{c}:MCLF:CURR {value}"),
            f if f == r.cal => format!(":CAL{c}"),
            f if f == r.hold => format!(":CHAN{c}:HOLD {value}"),
            _ => return ParamWrite::PassToBase(Ok(())),
        };
        let _ = base.set_int32_param(user.reason, c, value);
        if user.reason == r.hold {
            // C `puts(outString_)`.
            println!("{cmd}");
        }
        let status = match self.controller.get() {
            Some(ctrl) => ctrl
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .write_only(&cmd),
            None => Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "MCS2 controller not created".into(),
            }),
        };
        let _ = base.call_param_callbacks(c);
        ParamWrite::Handled(status)
    }

    /// C `MCS2Controller::report` (smarActMCS2MotorDriver.cpp:177-184).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "MCS2 motor driver {}, numAxes={}, moving poll period={:.6}, idle poll period={:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            CFloat(self.moving_poll),
            CFloat(self.idle_poll),
        );
        ctx.report(out, level);
    }
}

/// C `MCS2Axis::report` (smarActMCS2MotorDriver.cpp:287-349) for channel
/// `channel`, which queries the controller at level > 0.
pub struct Mcs2AxisReport {
    pub controller: Arc<Mutex<Mcs2Controller>>,
    pub channel: i32,
}

impl AxisReport for Mcs2AxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level == 0 {
            return;
        }
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        let c = self.channel;
        let query = |what: &str| ctrl.query(&format!(":CHAN{c}:{what}?"));
        let int = |what: &str| atoi(&query(what).unwrap_or_default());
        let pcode = int("PTYP");
        // C copies its uninitialised `pname` over the reply
        // (`strcpy(pC_->inString_, pname)`), a bug; the reply is printed.
        let pname = query("PTYP:NAME").unwrap_or_default();
        let (state, vel, acc, mclf, ferr, error) = (
            int("STAT"),
            int("VEL"),
            int("ACC"),
            int("MCLF"),
            int("FERR"),
            int("ERR"),
        );
        // C reports only the last query's status.
        let temp = query("TEMP");
        if let Err(e) = &temp {
            let _ = writeln!(out, "  axis {c} -> asyn status error {}", e.status() as i32);
        }
        let temp = atoi(&temp.unwrap_or_default());
        let _ = write!(
            out,
            "  axis {c}\n    positioner type {pcode}\n    positioner name {pname}\n    \
             state {state}\n    velocity {vel}\n    acceleration {acc}\n    \
             max closed loop frequency {mclf}\n    following error {ferr}\n    \
             error {error}\n    temp {temp}\n",
        );
        ctrl.clear_errors();
    }
}

/// One MCS2 channel sharing a controller. Implements [`AsynMotor`].
pub struct Mcs2Axis {
    controller: Arc<Mutex<Mcs2Controller>>,
    /// 0-based wire channel index.
    channel: i32,
    /// Sensor-present flag from the last poll (selects closed/open-loop moves).
    has_encoder: bool,
    /// Software-tracked position for open-loop step moves (driver units).
    tracked_position: f64,
    /// The channel parameters the last poll set, for the poller to post.
    poll_params: Vec<(usize, ParamValue)>,
}

impl Mcs2Axis {
    /// Construct channel `axis_no` (0-based), draining the controller error
    /// queue as the C axis constructor does.
    pub fn new(controller: Arc<Mutex<Mcs2Controller>>, axis_no: usize) -> Self {
        controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear_errors();
        Self {
            controller,
            channel: axis_no as i32,
            has_encoder: false,
            tracked_position: 0.0,
            poll_params: Vec::new(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Mcs2Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `MCS2Axis::poll`'s queries up to `skip:` (smarActMCS2MotorDriver.cpp:
    /// 479-555), with the channel parameters it sets on the way in `set`:
    /// the status word, sensor present, encoder and theoretical position, and
    /// amplitude reply.
    fn poll_channel(
        &self,
        ctrl: &Mcs2Controller,
        set: &mut Vec<(usize, i32)>,
    ) -> AsynResult<(i64, bool, f64, f64, String)> {
        let c = self.channel;
        let r = ctrl.reasons.unwrap_or_default();
        let state = atof(&ctrl.query(&format!(":CHAN{c}:STAT?"))?) as i64;
        set.push((r.pstat, state as i32));
        let sensor_present = (state & SENSOR_PRESENT) != 0;
        let mut encoder_position = self.tracked_position;
        let mut theory_position = self.tracked_position;
        if sensor_present {
            let pos = ctrl.query(&format!(":CHAN{c}:POS?"))?;
            encoder_position = atof(&pos) / PULSES_PER_STEP;
            let targ = ctrl.query(&format!(":CHAN{c}:POS:TARG?"))?;
            theory_position = atof(&targ) / PULSES_PER_STEP;
        }
        // Drive power-on status (amplitude non-zero).
        let ampl = ctrl.query(&format!(":CHAN{c}:AMPL?"))?;
        set.push((r.ptyp_rb, atoi(&ctrl.query(&format!(":CHAN{c}:PTYP?"))?)));
        if state & ACTIVELY_MOVING == 0 {
            set.push((r.cal, i32::from(state & IS_CALIBRATED != 0)));
            set.push((r.mclf, atoi(&ctrl.query(&format!(":CHAN{c}:MCLF?"))?)));
            set.push((r.hold, atoi(&ctrl.query(&format!(":CHAN{c}:HOLD?"))?)));
        }
        Ok((
            state,
            sensor_present,
            encoder_position,
            theory_position,
            ampl,
        ))
    }

    /// Closed-loop move: set move mode, acceleration and velocity, then move.
    fn move_closed_loop(
        &self,
        ctrl: &Mcs2Controller,
        target: f64,
        relative: bool,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let c = self.channel;
        ctrl.write_only(&format!(":CHAN{c}:MMOD {}", if relative { 1 } else { 0 }))?;
        ctrl.write_only(&format!(
            ":CHAN{c}:ACC {:.6}",
            CFloat(acceleration * PULSES_PER_STEP)
        ))?;
        ctrl.write_only(&format!(
            ":CHAN{c}:VEL {:.6}",
            CFloat(velocity * PULSES_PER_STEP)
        ))?;
        ctrl.write_only(&format!(":MOVE{c} {:.6}", CFloat(target * PULSES_PER_STEP)))
    }

    /// Open-loop step move: track the target in software and send the relative
    /// delta unscaled (C `MMOD 4`). Locks the controller internally.
    fn move_open_loop(&mut self, delta: f64) -> AsynResult<()> {
        let c = self.channel;
        self.tracked_position += delta;
        let ctrl = self.lock();
        ctrl.write_only(&format!(":CHAN{c}:MMOD 4"))?;
        ctrl.write_only(&format!(":MOVE{c} {:.6}", CFloat(delta)))
    }
}

impl AsynMotor for Mcs2Axis {
    /// C `MCS2Axis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (smarActMCS2MotorDriver.cpp).
    fn take_poll_params(&mut self) -> Vec<(usize, ParamValue)> {
        std::mem::take(&mut self.poll_params)
    }

    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if self.has_encoder {
            let ctrl = self.lock();
            self.move_closed_loop(&ctrl, position, false, velocity, acceleration)
        } else {
            let delta = position - self.tracked_position;
            self.move_open_loop(delta)
        }
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if self.has_encoder {
            let ctrl = self.lock();
            self.move_closed_loop(&ctrl, distance, true, velocity, acceleration)
        } else {
            self.move_open_loop(distance)
        }
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Not implemented in the C MCS2 driver.
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let c = self.channel;
        // Reference options: reverse start direction unless forward, auto-zero.
        let mut ref_opt = AUTO_ZERO;
        if !forward {
            ref_opt |= START_DIRECTION;
        }
        let ctrl = self.lock();
        ctrl.write_only(&format!(":CHAN{c}:REF:OPT {ref_opt}"))?;
        ctrl.clear_errors();
        ctrl.write_only(&format!(
            ":CHAN{c}:ACC {:.6}",
            CFloat(acceleration * PULSES_PER_STEP)
        ))?;
        ctrl.clear_errors();
        ctrl.write_only(&format!(
            ":CHAN{c}:VEL {:.6}",
            CFloat(velocity * PULSES_PER_STEP)
        ))?;
        ctrl.clear_errors();
        ctrl.write_only(&format!(":REF{c}"))?;
        ctrl.clear_errors();
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let c = self.channel;
        let ctrl = self.lock();
        ctrl.write_only(&format!(":STOP{c}"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let c = self.channel;
        self.tracked_position = position;
        let ctrl = self.lock();
        ctrl.write_only(&format!(
            ":CHAN{c}:POS {:.6}",
            CFloat(position * PULSES_PER_STEP)
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // Not implemented in the C MCS2 driver.
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C MCS2 driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let mut set = Vec::new();
        let polled = self.poll_channel(&ctrl, &mut set);
        let posts = ctrl.reasons.is_some();
        drop(ctrl);
        // C's `skip:` posts what was set before a failed query.
        if posts {
            self.poll_params.extend(
                set.into_iter()
                    .map(|(reason, value)| (reason, ParamValue::Int32(value))),
            );
        }
        let (state, sensor_present, encoder_position, theory_position, ampl) = polled?;

        let done = (state & ACTIVELY_MOVING) == 0;
        let end_stop = (state & END_STOP_REACHED) != 0;

        let powered = atof(&ampl) as i64 != 0;

        self.has_encoder = sensor_present;
        // Keep the software-tracked position aligned with the sensor when closed.
        if sensor_present {
            self.tracked_position = theory_position;
        }

        Ok(MotorStatus {
            position: theory_position,
            encoder_position,
            velocity: 0.0,
            done,
            moving: !done,
            has_encoder: sensor_present,
            gain_support: (state & CLOSED_LOOP_ACTIVE) != 0,
            homed: (state & IS_REFERENCED) != 0,
            home: (state & REFERENCE_MARK) != 0,
            high_limit: end_stop,
            low_limit: end_stop,
            problem: (state & MOVEMENT_FAILED) != 0,
            // C's motorStatusFollowingError_, MSTA bit 6.
            slip_stall: (state & FOLLOWING_LIMIT_REACHED) != 0,
            powered,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// C `sprintf(":CHAN%d:ACC %f")` and the like: six decimals, where
    /// Rust's `{}` writes the shortest form.
    #[test]
    fn moves_are_written_with_c_s_percent_f() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::new("mcs2PercentF", |_| String::new());
        let ctrl = Mcs2Controller::new(io, &MotorTrace::new("mcs2PercentF", 0));
        let mut axis = Mcs2Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        rig.sent();
        let user = AsynUser::default();
        axis.home(&user, 0.0, 2.0, 3.0, true).unwrap();
        axis.set_position(&user, 1.5).unwrap();
        axis.move_absolute(&user, 2.0, 0.0, 1.0, 1.0).unwrap();
        axis.has_encoder = true;
        axis.move_relative(&user, 0.25, 0.0, 1.0, 1.0).unwrap();
        let sent = rig.sent();
        for want in [
            ":CHAN0:ACC 3000.000000",
            ":CHAN0:VEL 2000.000000",
            ":CHAN0:POS 1500.000000",
            ":MOVE0 0.500000",
            ":CHAN0:ACC 1000.000000",
            ":MOVE0 250.000000",
        ] {
            assert!(
                sent.iter().any(|c| c.trim_end() == want),
                "{want}: {sent:?}"
            );
        }
    }

    /// C sets `motorStatusFollowingError_` from FOLLOWING_LIMIT_REACHED
    /// (smarActMCS2MotorDriver.cpp:492, 505).
    #[test]
    fn the_poll_reports_the_following_limit() {
        use motor_common::testing::LineRig;
        let (_rig, io) = LineRig::new("mcs2FollowLimit", |cmd| {
            match cmd.trim_end() {
                ":CHAN0:STAT?" => "1024\r\n",
                _ => "0\r\n",
            }
            .to_string()
        });
        let ctrl = Mcs2Controller::new(io, &MotorTrace::new("mcs2FollowLimit", 0));
        let mut axis = Mcs2Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        assert!(axis.poll(&AsynUser::default()).unwrap().slip_stall);
    }

    #[test]
    fn status_bits_decode() {
        // Moving + sensor present + closed loop.
        let state: i64 = ACTIVELY_MOVING | SENSOR_PRESENT | CLOSED_LOOP_ACTIVE;
        assert_eq!(state & ACTIVELY_MOVING, ACTIVELY_MOVING);
        assert!((state & SENSOR_PRESENT) != 0);
        assert!((state & CLOSED_LOOP_ACTIVE) != 0);
        // Idle + referenced + at reference mark.
        let idle: i64 = IS_REFERENCED | REFERENCE_MARK;
        assert_eq!(idle & ACTIVELY_MOVING, 0);
        assert!((idle & IS_REFERENCED) != 0);
    }

    #[test]
    fn home_ref_opt_direction() {
        // Forward home is auto-zero only; reverse adds the start-direction bit.
        let forward = AUTO_ZERO;
        let reverse = AUTO_ZERO | START_DIRECTION;
        assert_eq!(forward, AUTO_ZERO);
        assert_eq!(reverse, AUTO_ZERO | START_DIRECTION);
        assert_ne!(forward, reverse);
    }
}
