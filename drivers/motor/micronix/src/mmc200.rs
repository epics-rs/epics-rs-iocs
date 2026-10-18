//! Micronix MMC-100/103/110/200 motor controller driver (serial ASCII).
//!
//! Ported from `motorMicronix/micronixApp/src/MMC200Driver.cpp`, a model-3
//! `asynMotorController`/`asynMotorAxis` driver. Axes are numbered from 1 on
//! the wire; commands are `{n}CMD` and queries `{n}CMD?` reply with a leading
//! `#` (e.g. `#MMC-200 …`, `#8`, `#0.000000,0.000000`). Motion commands take
//! no reply — only `?` queries are read — so this port pairs
//! `Mmc200Controller::write` (fire-and-forget) with
//! `Mmc200Controller::query` (write + read) exactly as the C driver pairs
//! `writeController` with `writeReadController`.
//!
//! ## Units
//!
//! As C, the record's steps are scaled by the per-axis `resolution_` (mm-or-deg
//! per microstep or per encoder count, derived from `FBK?`/`REZ?`/`UST?`/
//! `ENC?`) into the controller's units, and `POS?` is divided back.
//!
//! ## Deviations from C (documented)
//!
//! - C's per-axis constructor sets `motorStatusProblem_` and continues when an
//!   init query fails; this port returns an error from [`Mmc200Axis::new`]
//!   instead (a controller that cannot answer `VER?`/`VMX?` cannot be
//!   operated), naming the failure at controller-create time.
//! - The MMC-ETH module (`#MMC-ETH`, C `model_ == 999`) is not a motor axis;
//!   as in C only its poll is disabled (returns an inert, done status). C does
//!   not guard its motion methods against `999` and neither does this port.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::util::{atof, atoi};
use motor_common::{AxisReport, ControllerParams, ControllerReport};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// Command line terminator (st.cmd configures `asynOctetSetOutputEos("\r")`).
const TERMINATOR: &[u8] = b"\r";

/// C `model_ == 999`: the MMC-ETH ethernet module, not a motor axis.
const MODEL_ETH: i32 = 999;

/// Status word bits (C `poll` `status`).
const STA_LOW_LIMIT: i32 = 0x1;
const STA_HIGH_LIMIT: i32 = 0x2;
const STA_DONE: i32 = 0x8;
const STA_ERROR: i32 = 0x80;

/// Shared controller endpoint: owns the serial handle and command framing.
/// The caller holds the `Arc<Mutex<_>>` lock across a write→read exchange.
pub struct Mmc200Controller {
    handle: SyncIOHandle,
}

impl Mmc200Controller {
    /// Wrap a connected octet handle. Axes are created separately (the count
    /// is a `MMC200CreateController` argument, not autodiscovered).
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command message with no reply (C `writeController`); the
    /// terminator is appended here.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a query and read its reply (C `writeReadController`), trimming the
    /// trailing line terminator. The leading `#` is left intact — callers skip
    /// it with `reply.get(1..)`, matching C's `&inString_[1]`.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, &Self::framed(cmd), READ_BUF)?;
        let reply = String::from_utf8_lossy(&raw);
        Ok(reply.trim_end_matches(['\r', '\n', '\0']).to_string())
    }
}

/// Parse the `VER?` reply into the C `model_` code: `#MMC-200` → 200,
/// `#MMC-100` → 100, `#MMC-103` → 103, `#MMC-110` → 110, `#MMC-ETH` → 999.
/// A too-short or unrecognized string is `-1` (C behaviour).
fn parse_model(version: &str) -> i32 {
    if version.len() <= 8 {
        return -1;
    }
    match &version[..8] {
        "#MMC-200" => 200,
        "#MMC-100" => 100,
        "#MMC-103" => 103,
        "#MMC-110" => 110,
        "#MMC-ETH" => MODEL_ETH,
        _ => -1,
    }
}

/// C `resolution_` in mm-or-deg per record step: an open-loop MMC-200's `REZ`
/// is tens of picometres per full step, an MMC-1xx's steps per µm (or
/// milli-degree); closed loop uses the encoder's `ENC` (µm per count).
fn resolution(model: i32, closed_loop: i32, rez: i32, micro_steps: i32, encoder: f64) -> f64 {
    if closed_loop != 0 {
        encoder / 1e3
    } else if model == 200 {
        f64::from(rez) * 1e-8 / f64::from(micro_steps)
    } else {
        1e-3 / f64::from(rez)
    }
}

/// Parse a `POS?` reply `#<theoretical>,<encoder>` into `(position, encoder)`
/// in controller units (C `sscanf("#%lf,%lf")`). A missing encoder field reads 0.
fn parse_position(reply: &str) -> (f64, f64) {
    let body = reply.get(1..).unwrap_or("");
    match body.split_once(',') {
        Some((pos, enc)) => (atof(pos), atof(enc)),
        None => (atof(body), 0.0),
    }
}

/// The controller's poll periods, in seconds, and `ignoreLimits_`, for its
/// report.
pub struct Mmc200Params {
    pub moving_poll: f64,
    pub idle_poll: f64,
    pub ignore_limits: bool,
}

impl ControllerParams for Mmc200Params {
    /// C `MMC200Controller::report` (MMC200Driver.cpp:90-101).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = write!(
            out,
            "MMC-200 motor driver\n  port name={}\n  num axes={}\n  moving poll period={:.6}\n  \
             idle poll period={:.6}\n  ignore limits={}\n",
            ctx.port_name(),
            ctx.num_axes(),
            CFloat(self.moving_poll),
            CFloat(self.idle_poll),
            i32::from(self.ignore_limits),
        );
        ctx.report(out, level);
    }
}

/// What C's `MMC200Axis::report` prints (MMC200Driver.cpp:281-298): the
/// axis's constructor probes, which never change. The MMC-ETH module's are
/// 0, where C leaves them unset.
#[derive(Default)]
pub struct Mmc200AxisReport {
    axis: usize,
    axis_index: i32,
    version: String,
    model: i32,
    closed_loop: i32,
    rez: i32,
    micro_steps: i32,
    encoder_resolution: f64,
    resolution: f64,
    max_velocity: f64,
}

impl AxisReport for Mmc200AxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let _ = write!(
                out,
                "  axis {}\n  axis index {}\n  version {}\n  model {}\n  feedback mode {}\n  \
                 rez {}\n  micro steps {}\n  encoder resolution {:.6}\n  resolution {:.6}\n  \
                 max velocity {:.6}\n",
                self.axis,
                self.axis_index,
                self.version,
                self.model,
                self.closed_loop,
                self.rez,
                self.micro_steps,
                CFloat(self.encoder_resolution),
                CFloat(self.resolution),
                CFloat(self.max_velocity),
            );
        }
    }
}

/// One MMC-200 axis sharing a controller. Implements [`AsynMotor`].
pub struct Mmc200Axis {
    controller: Arc<Mutex<Mmc200Controller>>,
    /// 1-based controller axis number (C `axisIndex_ = axisNo + 1`).
    axis_index: i32,
    /// C `model_` (200/100/103/110/999/-1).
    model: i32,
    /// False for the MMC-ETH module (`model_ == 999`): polling is disabled.
    active: bool,
    /// C `maxVelocity_` from `VMX?`, in controller units/sec — the jog `JOG %`
    /// denominator.
    max_velocity: f64,
    /// C `resolution_`: controller units per record step.
    resolution: f64,
    /// Controller-wide `ignoreLimits` flag (C `ignoreLimits_`).
    ignore_limits: bool,
    /// Last polled status, reused on the comm-failure early exit.
    last_status: MotorStatus,
    /// What the constructor read, for [`Self::report`].
    probed: Arc<Mmc200AxisReport>,
}

impl Mmc200Axis {
    /// Construct the axis at 0-based `index`, running the C per-motor init:
    /// `VER?` (model, for the MMC-ETH ignore) and, for a real axis, the
    /// `FBK?`/`REZ?`/`UST?`/`ENC?` resolution probes and `VMX?` (max velocity
    /// for jog). Performs blocking serial I/O under the controller lock.
    pub fn new(
        controller: Arc<Mutex<Mmc200Controller>>,
        index: usize,
        ignore_limits: bool,
    ) -> Self {
        let axis_index = index as i32 + 1;
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        // C carries on past a failed query with `errorFlag` set and reports
        // it as `motorStatusProblem_` (MMC200Driver.cpp:143-146, :263-266).
        let mut error = false;
        let mut query = |cmd: String| {
            ctrl.query(&cmd).unwrap_or_else(|_| {
                error = true;
                String::new()
            })
        };

        let version = query(format!("{axis_index}VER?"));
        let model = parse_model(&version);

        if model == MODEL_ETH {
            drop(ctrl);
            let probed = Mmc200AxisReport {
                axis: index,
                axis_index,
                version,
                model,
                ..Mmc200AxisReport::default()
            };
            return Self {
                controller,
                axis_index,
                model,
                active: false,
                max_velocity: 0.0,
                resolution: 1.0,
                ignore_limits,
                last_status: motor_common::status_before_poll(),
                probed: Arc::new(probed),
            };
        }

        let mut reply_int = |cmd: &str| {
            let reply = query(format!("{axis_index}{cmd}"));
            atoi(reply.get(1..).unwrap_or(""))
        };
        let closed_loop = reply_int("FBK?");
        let rez = reply_int("REZ?");
        // Only the MMC-200 has a variable microstep count.
        let micro_steps = if model == 200 { reply_int("UST?") } else { 1 };
        let enc = query(format!("{axis_index}ENC?"));
        let encoder_resolution = atof(enc.get(1..).unwrap_or(""));
        let resolution = resolution(model, closed_loop, rez, micro_steps, encoder_resolution);

        let vmx = query(format!("{axis_index}VMX?"));
        let max_velocity = atof(vmx.get(1..).unwrap_or(""));
        drop(ctrl);
        let probed = Mmc200AxisReport {
            axis: index,
            axis_index,
            version,
            model,
            closed_loop,
            rez,
            micro_steps,
            encoder_resolution,
            resolution,
            max_velocity,
        };

        Self {
            controller,
            axis_index,
            model,
            active: true,
            max_velocity,
            resolution,
            ignore_limits,
            // MMC200Driver.cpp:259-268, once.
            last_status: MotorStatus {
                problem: error,
                gain_support: true,
                has_encoder: true,
                ..motor_common::status_before_poll()
            },
            probed: Arc::new(probed),
        }
    }

    /// C `MMC200Axis::report`.
    pub fn report(&self) -> Arc<Mmc200AxisReport> {
        self.probed.clone()
    }

    fn lock(&self) -> MutexGuard<'_, Mmc200Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `sendAccelAndVelocity`: `VEL`, then `ACC` and `DEC` (accel used for
    /// both), each scaled by `resolution_`.
    fn send_accel_vel(
        &self,
        ctrl: &Mmc200Controller,
        acceleration: f64,
        velocity: f64,
    ) -> AsynResult<()> {
        let axis = self.axis_index;
        let res = self.resolution;
        ctrl.write(&format!("{axis}VEL{:.3}", CFloat(velocity * res)))?;
        ctrl.write(&format!("{axis}ACC{:.3}", CFloat(acceleration * res)))?;
        ctrl.write(&format!("{axis}DEC{:.3}", CFloat(acceleration * res)))?;
        Ok(())
    }

    /// The controller model reported by `VER?` (diagnostic).
    pub fn model(&self) -> i32 {
        self.model
    }
}

impl AsynMotor for Mmc200Axis {
    /// C `MMC200Axis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (MMC200Driver.cpp).
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
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_vel(&ctrl, acceleration, velocity)?;
        ctrl.write(&format!("{n}MVA{:.6}", CFloat(position * self.resolution)))?;
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_vel(&ctrl, acceleration, velocity)?;
        ctrl.write(&format!("{n}MVR{:.6}", CFloat(distance * self.resolution)))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: the JOG command takes a speed as a percentage of
        // max velocity, clamped to ±100.
        let n = self.axis_index;
        let jog_percent =
            (velocity * self.resolution / self.max_velocity * 100.0).clamp(-100.0, 100.0);
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{n}JAC{:.3}",
            CFloat(acceleration * self.resolution)
        ))?;
        ctrl.write(&format!("{n}JOG{:.3}", CFloat(jog_percent)))?;
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
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_vel(&ctrl, acceleration, velocity)?;
        // C `HCG1` selects the forward home direction, `HCG0` the reverse.
        ctrl.write(&format!("{n}HCG{}", if forward { 1 } else { 0 }))?;
        ctrl.write(&format!("{n}HOM"))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write(&format!("{n}STP"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `setPosition`: the MMC-200 can only zero the position (`ZRO`); a
        // non-zero request is a quiet success (no command).
        if position == 0.0 {
            let n = self.axis_index;
            let ctrl = self.lock();
            ctrl.write(&format!("{n}ZRO"))?;
        }
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C `setClosedLoop` toggles motor power (`MOT1`/`MOT0`).
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write(&format!("{n}MOT{}", if enable { 1 } else { 0 }))?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // C ignores the MMC-ETH module in poll (returns asynSuccess without
        // updating any parameter); keep the record inert.
        if !self.active {
            return Ok(self.last_status.clone());
        }

        let n = self.axis_index;
        let controller = Arc::clone(&self.controller);
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // C `poll`: `POS?` (position + encoder), `STA?` (status word, clearing
        // the error buffer with `CER` when the error bit is set), `MOT?`
        // (drive power), each written as it arrives. A read failure is C's
        // `goto skip`: what was written stays, and `poll_failure` adds PROBLEM.
        let pos_reply = ctrl.query(&format!("{n}POS?"))?;
        let (position, encoder) = parse_position(&pos_reply);
        self.last_status.position = position / self.resolution;
        self.last_status.encoder_position = encoder / self.resolution;

        let sta_reply = ctrl.query(&format!("{n}STA?"))?;
        let status = atoi(sta_reply.get(1..).unwrap_or(""));
        let done = status & STA_DONE != 0;
        self.last_status.done = done;
        self.last_status.moving = !done;
        (self.last_status.high_limit, self.last_status.low_limit) = if self.ignore_limits {
            (false, false)
        } else {
            (status & STA_HIGH_LIMIT != 0, status & STA_LOW_LIMIT != 0)
        };
        if status & STA_ERROR != 0 {
            // C stores this return in `status`, which nothing reads after
            // done/limits were taken from it: a refused CER neither skips nor
            // raises PROBLEM.
            let _ = ctrl.write(&format!("{n}CER"));
        }

        let mot_reply = ctrl.query(&format!("{n}MOT?"))?;
        let drive_on = atoi(mot_reply.get(1..).unwrap_or(""));
        self.last_status.powered = drive_on != 0;
        self.last_status.problem = false;
        Ok(self.last_status.clone())
    }

    fn take_failed_poll_status(&mut self) -> Option<MotorStatus> {
        Some(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use std::time::Duration;

    #[test]
    fn an_axis_of_an_unanswering_controller_is_built_with_a_problem() {
        let (handle, _) =
            SyncIOHandle::connect_port("MMC200_NONE", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(Mmc200Controller::new(handle)));
        let axis = Mmc200Axis::new(controller, 0, false);
        assert_eq!(axis.model, -1);
        // C's status_ starts at 0; only a good STA? sets Done (:482).
        assert!(axis.last_status.problem && !axis.last_status.done);
    }

    /// C's constructor returns early for an MMC-ETH module and its poll
    /// returns at once (MMC200Driver.cpp:191-193, :447-448): status_ stays 0.
    #[test]
    fn an_ethernet_module_axis_keeps_a_zero_status() {
        use motor_common::testing::LineRig;
        let (_rig, io) = LineRig::new("mmc200Eth", |_| "#MMC-ETH v1".into());
        let controller = Arc::new(Mutex::new(Mmc200Controller::new(io)));
        let mut axis = Mmc200Axis::new(controller, 0, false);
        assert_eq!(axis.model, MODEL_ETH);
        let status = axis.poll(&AsynUser::default()).unwrap();
        assert_eq!(status, motor_common::status_before_poll());
    }

    /// C sets GAIN_SUPPORT and EA_PRESENT once, in the constructor
    /// (MMC200Driver.cpp:259-261), and its `skip` label raises only
    /// PROBLEM and returns asynError (:519-522).
    #[test]
    fn a_failed_poll_keeps_the_constructor_status_bits() {
        let (handle, _) =
            SyncIOHandle::connect_port("MMC200_DEAD", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(Mmc200Controller::new(handle)));
        let mut axis = Mmc200Axis::new(controller, 0, false);
        assert!(axis.poll(&AsynUser::default()).is_err());
        let left = axis.take_failed_poll_status().expect("the status C left");
        assert!(left.gain_support && left.has_encoder, "{left:?}");
        assert!(!left.comms_error, "{left:?}");
    }

    /// MMC200Driver.cpp:505 stores the `CER` write's return in `status`, not
    /// `comStatus`, so a refused `CER` neither skips `MOT?` nor raises PROBLEM.
    #[test]
    fn rv4_a_refused_cer_does_not_fail_the_poll() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::failing(
            "mmc200RefusedCer",
            |cmd| {
                match cmd.trim_end() {
                    "1VER?" => "#MMC-200 v4.1",
                    "1REZ?" => "#1000",
                    "1UST?" => "#8",
                    "1VMX?" => "#2.5",
                    "1POS?" => "#1.000000,1.000000",
                    "1STA?" => "#136",
                    "1MOT?" => "#1",
                    _ => "#0",
                }
                .into()
            },
            |cmd| cmd.trim_end() == "1CER",
        );
        let controller = Arc::new(Mutex::new(Mmc200Controller::new(io)));
        let mut axis = Mmc200Axis::new(controller, 0, false);
        rig.sent();
        let status = axis.poll(&AsynUser::default()).unwrap();
        assert!(
            status.done && status.powered && !status.problem,
            "{status:?}"
        );
        let sent: Vec<String> = rig.sent().iter().map(|c| c.trim_end().to_owned()).collect();
        assert!(sent.contains(&"1CER".to_owned()), "{sent:?}");
        assert!(sent.contains(&"1MOT?".to_owned()), "{sent:?}");
    }

    /// C `sprintf("%dVEL%.3f")` / `"%dMVA%.6f"` spell a non-finite value
    /// `inf` / `nan`, where Rust writes `NaN`.
    #[test]
    fn a_non_finite_move_is_written_as_c_prints_it() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::new("mmc200NonFinite", |cmd| {
            match cmd.trim_end() {
                "1VER?" => "#MMC-200 v4.1",
                "1REZ?" => "#1000",
                "1UST?" => "#8",
                "1VMX?" => "#2.5",
                _ => "#0",
            }
            .into()
        });
        let controller = Arc::new(Mutex::new(Mmc200Controller::new(io)));
        let mut axis = Mmc200Axis::new(controller, 0, false);
        rig.sent();
        axis.move_absolute(&AsynUser::default(), f64::NAN, 0.0, f64::INFINITY, 1.0)
            .unwrap();
        let sent: Vec<String> = rig.sent().iter().map(|c| c.trim_end().to_owned()).collect();
        assert!(sent.contains(&"1VELinf".to_owned()), "{sent:?}");
        assert!(sent.contains(&"1MVAnan".to_owned()), "{sent:?}");
    }

    #[test]
    fn parse_model_recognizes_families() {
        assert_eq!(parse_model("#MMC-200 v1.2.3"), 200);
        assert_eq!(parse_model("#MMC-100 v1"), 100);
        assert_eq!(parse_model("#MMC-103 v1"), 103);
        assert_eq!(parse_model("#MMC-110 v1"), 110);
        assert_eq!(parse_model("#MMC-ETH v1"), MODEL_ETH);
        // Unrecognized model string.
        assert_eq!(parse_model("#MMC-999 v1"), -1);
        // Too short (C: strlen <= 8).
        assert_eq!(parse_model("#MMC-200"), -1);
        assert_eq!(parse_model("#"), -1);
    }

    #[test]
    fn resolution_follows_model_and_loop_mode() {
        assert_eq!(resolution(200, 0, 1000, 8, 0.0), 1000.0 * 1e-8 / 8.0);
        assert_eq!(resolution(100, 0, 50, 1, 0.0), 1e-3 / 50.0);
        assert_eq!(resolution(200, 1, 1000, 8, 0.01), 0.01 / 1e3);
    }

    #[test]
    fn parse_position_reads_theoretical_and_encoder() {
        assert_eq!(parse_position("#1.500000,1.499000"), (1.5, 1.499));
        assert_eq!(parse_position("#-0.250000,-0.250000"), (-0.25, -0.25));
        // Missing encoder field: encoder reads 0.
        assert_eq!(parse_position("#3.000000"), (3.0, 0.0));
    }

    #[test]
    fn status_bits_match_c_masks() {
        // done, high+low limits, error clear.
        assert_ne!(0x8 & STA_DONE, 0);
        assert_ne!(0x2 & STA_HIGH_LIMIT, 0);
        assert_ne!(0x1 & STA_LOW_LIMIT, 0);
        assert_ne!(0x80 & STA_ERROR, 0);
        // done bit isolated from limits.
        assert_eq!(0x8 & (STA_HIGH_LIMIT | STA_LOW_LIMIT), 0);
    }
}
