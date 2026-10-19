//! Newport Agilis AG-UC2 / AG-UC8 / AG-UC8PC controller driver.
//!
//! Ported from `motorNewport/newportApp/src/AG_UC.cpp` (`AG_UCController` /
//! `AG_UCAxis`). The Agilis is an open-loop piezo stepper controller driving up
//! to 8 axes over one serial line. Axes are grouped into channels of two: an
//! axis is addressed by an **axis ID** (`1` or `2`, the command prefix) within
//! a **channel** selected by the controller-wide `CC` command. The UC2 is a
//! single-channel model (channel 0, no `CC` needed).
//!
//! ## Shared-serial concurrency & channel selection
//!
//! Like [`crate::agap`], every axis is an independent [`AsynMotor`] sharing one
//! [`AgUcController`] behind an `Arc<Mutex<..>>`. Because channel selection is
//! controller-wide state, an axis operation must select its channel and issue
//! its command atomically — both happen under the controller lock, so a second
//! axis cannot change the channel in between.
//!
//! The Agilis needs a short delay between serial writes (C `WRITE_DELAY`); this
//! driver sleeps `WRITE_DELAY` after every exchange, matching the C driver.
//!
//! ## Open-loop stepper positioning
//!
//! There is no absolute encoder: `TP` returns an accumulated step count. The
//! driver tracks `current_position` (updated each poll) plus a
//! `position_offset` set by `set_position`, and an absolute move is issued as a
//! relative `PR` of `target - current_position` — faithful to the C driver.
//!
//! ## Parity notes
//!
//! C bugs, flagged at their sites:
//! - `AG_UC.cpp:242`: the reverse-amplitude default clobbers the *forward*
//!   amplitude; not copied (see `setup_amplitudes`).
//! - `AG_UC.cpp:383`: axis-2 limit test is `(lim == 3 || lim == 3)` — a
//!   duplicated condition; not copied (see `ph_limit`).

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use motor_common::AxisReport;

use crate::util::{nint, parse_int_at};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// C `WRITE_DELAY`: the Agilis needs a short delay between serial writes.
pub(crate) const WRITE_DELAY: Duration = Duration::from_millis(10);

/// C reset settle time after the `RS` command.
const RESET_SETTLE: Duration = Duration::from_millis(500);

/// C default forward step amplitude when the configured value is non-positive.
const DEFAULT_FORWARD_AMPLITUDE: i32 = 50;

/// Agilis controller model (C `AG_UCModel_t`, in its order: `as i32` is C's value).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AgUcModel {
    /// AG-UC2 — single channel (channel 0), two axes.
    Uc2,
    /// AG-UC8 — four channels of two axes.
    Uc8,
    /// AG-UC8PC — AG-UC8 with PC / piezo-crystal firmware variant.
    Uc8Pc,
}

impl AgUcModel {
    /// C constructor `strstr` chain: `AG-UC2`, then `AG-UC8PC`, then `AG-UC8`
    /// (UC8PC must precede UC8 since `AG-UC8` is a substring of `AG-UC8PC`).
    pub fn from_version(version: &str) -> Option<Self> {
        if version.contains("AG-UC2") {
            Some(Self::Uc2)
        } else if version.contains("AG-UC8PC") {
            Some(Self::Uc8Pc)
        } else if version.contains("AG-UC8") {
            Some(Self::Uc8)
        } else {
            None
        }
    }
}

fn agilis_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

fn parse_error(what: &str) -> AsynError {
    agilis_err(format!("Agilis: could not parse {what} response"))
}

/// C `AG_UCAxis::velocityToSpeedCode`: bucket the velocity magnitude to an
/// Agilis speed code (1..=4), signed by direction.
fn velocity_to_speed_code(velocity: f64) -> i32 {
    let a = velocity.abs();
    let speed = if a <= 5.0 {
        1
    } else if a <= 100.0 {
        2
    } else if a <= 666.0 {
        4
    } else {
        3
    };
    if velocity < 0.0 { -speed } else { speed }
}

/// C `AG_UCAxis`: axis ID within its channel, `(axisNo % 2) + 1` (1 or 2).
fn axis_id(axis_no: i32) -> i32 {
    (axis_no % 2) + 1
}

/// C `AG_UCAxis`: channel ID — 0 for the single-channel UC2, else
/// `axisNo / 2 + 1`.
fn channel_id(model: Option<AgUcModel>, axis_no: i32) -> i32 {
    if model == Some(AgUcModel::Uc2) {
        0
    } else {
        axis_no / 2 + 1
    }
}

/// C `AG_UCAxis` constructor step-amplitude setup, returning the (forward,
/// reverse) amplitudes to send in the two `SU` commands.
fn setup_amplitudes(forward_amplitude: i32, reverse_amplitude: i32) -> (i32, i32) {
    let forward = if forward_amplitude <= 0 {
        DEFAULT_FORWARD_AMPLITUDE
    } else {
        forward_amplitude
    };
    // AG_UC.cpp:242 assigns this default to forwardAmplitude_, a C defect.
    let reverse = if reverse_amplitude >= 0 {
        -50
    } else {
        reverse_amplitude
    };
    (forward, reverse)
}

/// C `AG_UCAxis::poll` limit test on the `PH` reply value.
fn ph_limit(axis_id: i32, lim: i32) -> bool {
    match axis_id {
        1 => lim == 1 || lim == 3,
        // AG_UC.cpp:383 tests `lim == 3 || lim == 3`, a C defect.
        2 => lim == 2 || lim == 3,
        _ => false,
    }
}

// --- Raw serial I/O, each followed by WRITE_DELAY (C spacing).

/// C `writeAgilis` / `setChannel` sleep after the write whatever its status.
fn write_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<()> {
    let status = handle.write_octet(0, cmd.as_bytes()).map(drop);
    thread::sleep(WRITE_DELAY);
    status
}

fn write_read_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<String> {
    let (reply, status) = exchange_raw(handle, cmd);
    status.map(|()| reply)
}

/// C `writeReadController` (pasynOctetSyncIO->writeRead): the bytes that
/// arrived, and the status. The caller's sleep follows a success only.
fn exchange_raw(handle: &SyncIOHandle, cmd: &str) -> (String, AsynResult<()>) {
    let (reply, status) = handle.write_read(0, cmd.as_bytes(), READ_BUF).into_parts();
    if status.is_ok() {
        thread::sleep(WRITE_DELAY);
    }
    (String::from_utf8_lossy(&reply).into_owned(), status)
}

/// Shared Agilis controller endpoint. Held behind a mutex so an axis's
/// channel-select plus command stay atomic on the shared line.
pub struct AgUcController {
    handle: SyncIOHandle,
    /// `None` when C returns before `startPoller` (VE failed or the model is
    /// unknown): the axes are still created but never polled.
    model: Option<AgUcModel>,
    /// C `controllerVersion_`, the `VE` reply; C leaves it unset when `VE`
    /// failed, empty here.
    version: String,
}

impl AgUcController {
    /// Connect and initialise an Agilis controller: reset (`RS`), enter remote
    /// mode (`MR`), read the firmware version (`VE`) and detect the model.
    /// Performs blocking serial I/O; `trace` is C's `pasynUserSelf`.
    pub fn new(handle: SyncIOHandle, trace: &MotorTrace) -> Self {
        // Reset, then settle; the controller needs time after RS. C ignores
        // the RS/MR write status.
        let _ = write_raw(&handle, "RS");
        thread::sleep(RESET_SETTLE);
        // Remote mode.
        let _ = write_raw(&handle, "MR");
        // Firmware version → model.
        let (version, status) = exchange_raw(&handle, "VE");
        // C sleeps after VE and prints `inString_` before testing its status
        // (AG_UC.cpp:71-74); exchange_raw slept only on success.
        if status.is_err() {
            thread::sleep(WRITE_DELAY);
        }
        println!("Agilis controller firmware version = {version}");
        let version = status.map(|()| version);
        let model = match &version {
            Err(_) => {
                trace.print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "AG_UCController::AG_UCController: cannot read version information from Agilis controller\n"
                    ),
                );
                None
            }
            Ok(version) => {
                let model = AgUcModel::from_version(version);
                if model.is_none() {
                    trace.print_self(
                        TraceMask::ERROR,
                        format_args!(
                            "AG_UCController::AG_UCController: unknown model, firmware string={version}\n"
                        ),
                    );
                }
                model
            }
        };
        Self {
            handle,
            model,
            version: version.unwrap_or_default(),
        }
    }

    pub fn model(&self) -> Option<AgUcModel> {
        self.model
    }

    /// C `controllerVersion_`.
    pub fn version(&self) -> &str {
        &self.version
    }

    fn write(&self, cmd: &str) -> AsynResult<()> {
        write_raw(&self.handle, cmd)
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        write_read_raw(&self.handle, cmd)
    }

    /// C `setChannel`: unconditionally select `channel_id` via `CC` (the C
    /// driver issues even `CC0` when polling a UC2). Caller holds the lock.
    /// Both C callers drop its status (AG_UC.cpp:162, :350).
    fn set_channel(&self, channel_id: i32) {
        let _ = self.write(&format!("CC{channel_id}"));
    }

    /// C `writeAgilis(channelID, ..)`: select the channel first when non-zero,
    /// then write the channel-scoped command. Caller holds the lock.
    fn write_on_channel(&self, channel_id: i32, cmd: &str) -> AsynResult<()> {
        if channel_id != 0 {
            self.set_channel(channel_id);
        }
        self.write(cmd)
    }
}

/// One Agilis axis sharing a controller. Implements [`AsynMotor`].
pub struct AgUcAxis {
    controller: Arc<Mutex<AgUcController>>,
    /// Command prefix within the channel (`1` or `2`; C `axisID_`).
    axis_id: i32,
    /// Channel selected via `CC` before commands (`0` for UC2; C `channelID_`).
    channel_id: i32,
    /// Whether the actuator has limit switches (C `hasLimits_`).
    has_limits: bool,
    /// C `forwardAmplitude_`/`reverseAmplitude_`, as sent with `SU`.
    forward_amplitude: i32,
    reverse_amplitude: i32,
    /// Shared with the axis's [`AgUcAxisReport`].
    steps: Arc<Mutex<Steps>>,
}

/// The step counts `AG_UCAxis::poll` and `setPosition` keep.
#[derive(Default)]
struct Steps {
    /// Accumulated step position last read from `TP`, plus offset (C
    /// `currentPosition_`).
    current: i32,
    /// Offset applied by `set_position` (C `positionOffset_`).
    offset: i32,
}

/// C `AG_UCAxis::report` (AG_UC.cpp:255-267).
pub struct AgUcAxisReport {
    axis_id: i32,
    channel_id: i32,
    has_limits: bool,
    forward_amplitude: i32,
    reverse_amplitude: i32,
    steps: Arc<Mutex<Steps>>,
}

impl AxisReport for AgUcAxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let steps = self.steps.lock().unwrap_or_else(|e| e.into_inner());
            let _ = write!(
                out,
                "  axisID={}, channelID={}, hasLimits={}\n  forwardAmplitude={}, \
                 reverseAmplitude={}\n  currentPosition={}, positionOffset={}\n",
                self.axis_id,
                self.channel_id,
                i32::from(self.has_limits),
                self.forward_amplitude,
                self.reverse_amplitude,
                steps.current,
                steps.offset
            );
        }
    }
}

impl AgUcAxis {
    /// Construct axis `axis_no` on a shared controller, sending the forward and
    /// reverse step-amplitude (`SU`) commands. Performs blocking serial I/O
    /// under the controller lock.
    pub fn new(
        controller: Arc<Mutex<AgUcController>>,
        axis_no: i32,
        has_limits: bool,
        forward_amplitude: i32,
        reverse_amplitude: i32,
    ) -> Self {
        let (axis_id, channel_id, forward_amplitude, reverse_amplitude) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let id = axis_id(axis_no);
            let ch = channel_id(ctrl.model, axis_no);
            let (forward, reverse) = setup_amplitudes(forward_amplitude, reverse_amplitude);
            let _ = ctrl.write_on_channel(ch, &format!("{id}SU{forward}"));
            let _ = ctrl.write_on_channel(ch, &format!("{id}SU{reverse}"));
            (id, ch, forward, reverse)
        };

        Self {
            controller,
            axis_id,
            channel_id,
            has_limits,
            forward_amplitude,
            reverse_amplitude,
            steps: Arc::default(),
        }
    }

    /// The axis's `report`.
    pub fn report(&self) -> AgUcAxisReport {
        AgUcAxisReport {
            axis_id: self.axis_id,
            channel_id: self.channel_id,
            has_limits: self.has_limits,
            forward_amplitude: self.forward_amplitude,
            reverse_amplitude: self.reverse_amplitude,
            steps: self.steps.clone(),
        }
    }

    fn steps(&self) -> std::sync::MutexGuard<'_, Steps> {
        self.steps.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, AgUcController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for AgUcAxis {
    /// C `AG_UCAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (AG_UC.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Open-loop: absolute move is a relative PR from the tracked position.
        let steps = nint(position - self.steps().current as f64);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}PR{steps}", self.axis_id))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let steps = nint(distance);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}PR{steps}", self.axis_id))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: jog (JA) at the speed code.
        let code = velocity_to_speed_code(velocity);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}JA{code}", self.axis_id))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `home`: only valid with limit switches; move to limit (MV) at the
        // speed code.
        if !self.has_limits {
            return Err(agilis_err("Agilis: home requires limit switches".into()));
        }
        let code = velocity_to_speed_code(velocity);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}MV{code}", self.axis_id))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}ST", self.axis_id))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `setPosition`: rebase the tracked position via an offset.
        let mut steps = self.steps();
        steps.offset = nint(position) - steps.current;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let controller = Arc::clone(&self.controller);
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // C never starts the poller: the axis reports its initial status only.
        if ctrl.model.is_none() {
            return Ok(());
        }

        // Select this axis's channel (C issues CC unconditionally, incl. CC0
        // on a UC2).
        ctrl.set_channel(self.channel_id);

        // C indexes `inString_`, which asyn ends with a NUL: a reply as long
        // as its offset reads 0, or for TS the terminator. A shorter one
        // reads an earlier reply's bytes there, a C defect refused here.
        // A failed read is C's `goto skip`: what was set before it stays, and
        // `poll_failure` adds PROBLEM.
        // Position (TP): accumulated step count at offset 3 ("1TP").
        let position = parse_int_at(&ctrl.write_read(&format!("{}TP", self.axis_id))?, 3)
            .ok_or_else(|| parse_error("TP"))?;
        let current = {
            let mut steps = self.steps();
            steps.current = position + steps.offset;
            steps.current
        };
        status.position = current as f64;

        // Moving status (TS): state char at index 3; '0' means done.
        let ts = ctrl.write_read(&format!("{}TS", self.axis_id))?;
        if ts.len() < 3 {
            return Err(parse_error("TS"));
        }
        let moving = ts.as_bytes().get(3) != Some(&b'0');
        (status.done, status.moving) = (!moving, moving);

        // Limit status (PH, controller-wide): value at offset 2.
        let lim = parse_int_at(&ctrl.write_read("PH")?, 2).ok_or_else(|| parse_error("PH"))?;
        let limit = ph_limit(self.axis_id, lim);
        // C sets both low and high limit to the same flag.
        (status.low_limit, status.high_limit) = (limit, limit);
        // `skip` on a good poll (AG_UC.cpp:388).
        status.problem = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use motor_common::testing::Polled;

    #[test]
    fn model_detected_from_firmware_string() {
        assert_eq!(AgUcModel::from_version("AG-UC2 v1"), Some(AgUcModel::Uc2));
        assert_eq!(AgUcModel::from_version("AG-UC8 v2"), Some(AgUcModel::Uc8));
        assert_eq!(
            AgUcModel::from_version("AG-UC8PC v2"),
            Some(AgUcModel::Uc8Pc)
        );
        assert_eq!(AgUcModel::from_version("mystery"), None);
    }

    /// AG_UC.cpp:52-81: a missing serial port still builds the controller and
    /// its axes; VE fails, so the poller is never started.
    #[test]
    fn a_controller_of_a_missing_port_is_built_unpolled() {
        let (handle, status) = SyncIOHandle::connect_port(
            "AG_UC_NONE",
            0,
            Duration::from_millis(10),
            InterfaceType::Octet,
        );
        assert!(status.is_err());
        let start = std::time::Instant::now();
        let controller = Arc::new(Mutex::new(AgUcController::new(
            handle,
            &MotorTrace::new("AG_UC_SELF", 0),
        )));
        // RS, MR and VE all fail and each still waits WRITE_DELAY, as in C.
        assert!(start.elapsed() >= RESET_SETTLE + 3 * WRITE_DELAY);
        assert_eq!(controller.lock().unwrap().model(), None);
        let mut axis = AgUcAxis::new(controller, 0, true, 50, -50);
        let status = axis.polled(&AsynUser::new(0)).unwrap();
        assert!(!status.moving);
    }

    #[test]
    fn uc8pc_detected_before_uc8_substring() {
        // "AG-UC8" is a substring of "AG-UC8PC"; the PC variant must win.
        assert_eq!(AgUcModel::from_version("AG-UC8PC"), Some(AgUcModel::Uc8Pc));
    }

    #[test]
    fn velocity_to_speed_code_buckets_and_signs() {
        assert_eq!(velocity_to_speed_code(3.0), 1);
        assert_eq!(velocity_to_speed_code(50.0), 2);
        assert_eq!(velocity_to_speed_code(500.0), 4);
        assert_eq!(velocity_to_speed_code(1000.0), 3);
        // Sign follows direction.
        assert_eq!(velocity_to_speed_code(-3.0), -1);
        assert_eq!(velocity_to_speed_code(-1000.0), -3);
    }

    #[test]
    fn axis_and_channel_ids_match_c() {
        // axisID = axisNo % 2 + 1.
        assert_eq!(axis_id(0), 1);
        assert_eq!(axis_id(1), 2);
        assert_eq!(axis_id(2), 1);
        assert_eq!(axis_id(7), 2);
        // UC2 → channel 0; UC8 → axisNo/2 + 1.
        assert_eq!(channel_id(Some(AgUcModel::Uc2), 0), 0);
        assert_eq!(channel_id(Some(AgUcModel::Uc2), 1), 0);
        assert_eq!(channel_id(Some(AgUcModel::Uc8), 0), 1);
        assert_eq!(channel_id(Some(AgUcModel::Uc8), 1), 1);
        assert_eq!(channel_id(Some(AgUcModel::Uc8), 2), 2);
        assert_eq!(channel_id(Some(AgUcModel::Uc8), 7), 4);
        // Unknown model (C never set AG_UCModel_): not UC2.
        assert_eq!(channel_id(None, 2), 2);
    }

    #[test]
    fn a_non_negative_reverse_amplitude_defaults_the_reverse_one() {
        assert_eq!(setup_amplitudes(0, -50), (50, -50));
        assert_eq!(setup_amplitudes(30, -20), (30, -20));
        assert_eq!(setup_amplitudes(30, 0), (30, -50));
        assert_eq!(setup_amplitudes(30, 20), (30, -50));
    }

    #[test]
    fn ph_reports_each_axis_by_its_own_bit() {
        assert!(ph_limit(1, 1));
        assert!(ph_limit(1, 3));
        assert!(!ph_limit(1, 2));
        assert!(ph_limit(2, 2));
        assert!(ph_limit(2, 3));
        assert!(!ph_limit(2, 1));
        assert!(!ph_limit(1, 0) && !ph_limit(2, 0));
    }

    /// writeAgilis and poll ignore what setChannel returns (AG_UC.cpp:162,
    /// :350): a refused `CC` still sends the command and polls the axis.
    #[test]
    fn a_refused_channel_select_still_sends_the_command() {
        use motor_common::testing::LineRig;
        let (line, io) = LineRig::failing(
            "ag_uc_refused_cc",
            |last| match last {
                "VE" => "AG-UC8 v2.2.1".into(),
                "1TP" => "1TP7".into(),
                "1TS" => "1TS0".into(),
                "PH" => "PH0".into(),
                _ => String::new(),
            },
            |last| last == "CC1",
        );
        let controller = AgUcController::new(io, &MotorTrace::new("AG_UC_REFUSED_CC", 0));
        let mut axis = AgUcAxis::new(Arc::new(Mutex::new(controller)), 0, true, 50, -50);
        line.sent();
        let user = AsynUser::new(0);
        axis.move_relative(&user, 5.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["CC1", "1PR5"]);
        let status = axis.polled(&user).unwrap();
        assert_eq!(status.position, 7.0);
        assert_eq!(line.sent(), ["CC1", "1TP", "1TS", "PH"]);
    }

    /// AG_UC.cpp:354-389: a failed read jumps to `skip`, whose
    /// callParamCallbacks still posts what the poll set before it, over
    /// the params as they stood, the Done=0 a move start set included.
    #[test]
    fn a_failed_read_leaves_what_was_read_before_it() {
        use motor_common::testing::LineRig;
        let refused = Arc::new(Mutex::new(String::new()));
        let pick = refused.clone();
        let ts = Arc::new(Mutex::new("1TS1".to_string()));
        let ts_reply = ts.clone();
        let (line, io) = LineRig::failing(
            "ag_uc_skip",
            move |last| match last {
                "VE" => "AG-UC8 v2.2.1".into(),
                "1TP" => "1TP7".into(),
                "1TS" => ts_reply.lock().unwrap().clone(),
                "PH" => "PH1".into(),
                _ => String::new(),
            },
            move |last| *pick.lock().unwrap() == last,
        );
        let controller = AgUcController::new(io, &MotorTrace::new("AG_UC_SKIP", 0));
        let mut axis = AgUcAxis::new(Arc::new(Mutex::new(controller)), 0, true, 50, -50);
        let user = AsynUser::new(0);
        let mut status = MotorStatus::default();
        line.sent();
        *refused.lock().unwrap() = "1TS".into();
        assert!(axis.poll(&user, &mut status).is_err());
        assert_eq!(line.sent(), ["CC1", "1TP", "1TS"]);
        assert!(
            status.position == 7.0 && !status.moving && !status.problem,
            "{status:?}"
        );
        *refused.lock().unwrap() = "PH".into();
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(
            status.moving && !status.done && !status.low_limit,
            "{status:?}"
        );
        refused.lock().unwrap().clear();
        // The PROBLEM the poller raised over the failure; `skip` clears it.
        status.problem = true;
        axis.poll(&user, &mut status).unwrap();
        assert!(status.low_limit && !status.problem, "{status:?}");
        *refused.lock().unwrap() = "1TP".into();
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(
            status.position == 7.0 && status.moving && status.low_limit,
            "{status:?}"
        );

        *ts.lock().unwrap() = "1TS0".into();
        refused.lock().unwrap().clear();
        axis.poll(&user, &mut status).unwrap();
        assert!(status.done, "{status:?}");
        // writeFloat64's Done=0 at a move start (asynMotorController.cpp:318).
        status.done = false;
        *refused.lock().unwrap() = "1TP".into();
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(!status.done, "{status:?}");
    }

    /// AG_UC.cpp:359,370,381 index `inString_`, which asyn ends with a NUL
    /// (asynInterposeEos.c:251): a reply as long as its offset reads 0, or
    /// for TS the terminator, so not done. A shorter one is refused.
    #[test]
    fn a_reply_as_long_as_its_offset_reads_the_terminator() {
        use motor_common::testing::LineRig;
        use std::collections::HashMap;
        let replies: Arc<Mutex<HashMap<String, String>>> = Arc::new(Mutex::new(
            [("1TP", "1TP"), ("1TS", "1TS"), ("PH", "PH")]
                .map(|(cmd, reply)| (cmd.to_string(), reply.to_string()))
                .into(),
        ));
        let table = replies.clone();
        let (_line, io) = LineRig::new("ag_uc_short", move |last| match last {
            "VE" => "AG-UC8 v2.2.1".into(),
            _ => table.lock().unwrap().get(last).cloned().unwrap_or_default(),
        });
        let controller = AgUcController::new(io, &MotorTrace::new("AG_UC_SHORT", 0));
        let mut axis = AgUcAxis::new(Arc::new(Mutex::new(controller)), 0, true, 50, -50);
        let user = AsynUser::new(0);
        let status = axis.polled(&user).unwrap();
        assert_eq!(status.position, 0.0);
        assert!(status.moving && !status.done && !status.low_limit);
        for (cmd, short) in [("1TP", "1T"), ("1TS", "1T"), ("PH", "P")] {
            let full = replies.lock().unwrap().insert(cmd.into(), short.into());
            assert!(axis.polled(&user).is_err(), "{cmd}");
            replies.lock().unwrap().insert(cmd.into(), full.unwrap());
        }
    }

    /// AG_UC.cpp:71-74 prints `inString_` before testing the `VE` status,
    /// so a reply cut by a timeout is still printed.
    #[test]
    fn a_version_cut_by_a_timeout_is_still_printed() {
        use motor_common::testing::{LineRig, child_stdout, in_child};
        if !in_child() {
            let out = child_stdout(
                module_path!(),
                "a_version_cut_by_a_timeout_is_still_printed",
            );
            let line = "Agilis controller firmware version = AG-UC8 v2.2.1\n";
            assert_eq!(out.matches(line).count(), 1, "{out}");
            return;
        }
        let (_line, io) = LineRig::cut(
            "ag_uc_cut",
            |last| match last {
                "VE" => "AG-UC8 v2.2.1".to_string(),
                _ => String::new(),
            },
            |last| last == "VE",
        );
        let controller = AgUcController::new(io, &MotorTrace::new("AG_UC_CUT", 0));
        // C returns before storing the version or the model.
        assert_eq!((controller.model(), controller.version()), (None, ""));
    }
}
