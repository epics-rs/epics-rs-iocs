//! Newport CONEX-AGP / CONEX-CC / CONEX-PP / DL / FCL200 controller driver.
//!
//! Ported from `motorNewport/newportApp/src/AG_CONEX.cpp` (`AG_CONEXAxis` /
//! `AG_CONEXController`). The C pair subclasses `asynMotorController` /
//! `asynMotorAxis`; here a single [`ConexAxis`] implements asyn-rs
//! [`AsynMotor`] and the controller/serial-port/record wiring is assembled by
//! the IOC binary (the analogue of `AG_CONEXCreateController`).
//!
//! A CONEX controller is single-axis; commands are prefixed by the numeric
//! controller ID (C `controllerID_`, e.g. `1TP`). The driver holds a
//! [`SyncIOHandle`] to a dedicated serial port and issues CR/LF-terminated
//! ASCII. Five models are supported, each with model-dependent step size,
//! velocity/PID capability, and status parsing — see [`ConexModel`].
//!
//! ## Units
//!
//! The record boundary is raw steps, as in AG_CONEX.cpp: moves, velocities
//! and accelerations are `steps * stepSize_` (computed per model from the
//! constructor's queries) and the `TP` readback is `position / stepSize_`,
//! so the record's `MRES` must equal `stepSize_`. The jog target (soft limit
//! ∓ one `stepSize_`) and the CONEX-PP home velocity go out unscaled, as C
//! sends them.
//!
//! C divides by a `stepSize_` of 0 or infinity (an `IF?`/`FRM?` reply of 0
//! makes it infinite at AG_CONEX.cpp:249/260, and the readback divides by it
//! at :515); this port refuses the axis's moves and polls instead, a C
//! defect not copied.
//!
//! ## Parity notes
//!
//! Two behaviors are ported faithfully from the C even though they look like C
//! bugs; they are flagged inline at their call sites:
//! - non-DL `TS` parsing reads only the 2-char controller-state byte, so the
//!   `0x100`/`0x200` limit-bit masks are inert (C `AG_CONEX.cpp:533,548`).
//! - `set_pid_gain(Derivative)` on CC/DL emits a `KI` command (C sends `KI`
//!   with `KDMax`, `AG_CONEX.cpp:480`), not `KD`.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use motor_common::AxisReport;

use crate::util::{leading_hex, parse_int_at, parse_value_at};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

// C `CONEX_TIMEOUT` (2 s) is applied where the transport is opened —
// `SyncIOHandle::from_handle` in `crate::ioc` — not per write here.

/// C `LINUX_WRITE_DELAY` (AG_CONEX.cpp and AGAP_CONEX.cpp).
pub(crate) const LINUX_WRITE_DELAY: Duration = Duration::from_millis(100);

/// C `HOME_TYPE_MINUS_EOR` — minus end-of-run home type forced for CONEX-PP.
const HOME_TYPE_MINUS_EOR: i32 = 4;

/// CONEX controller model (C `ConexModel_t`), detected from the firmware
/// version string at construction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConexModel {
    /// CONEX-AGP (piezo, no velocity/accel, PID + low-pass filter).
    Agp,
    /// CONEX-CC (DC servo).
    Cc,
    /// CONEX-PP (stepper).
    Pp,
    /// DL series.
    Dl,
    /// FCL200 (positions in mm).
    Fcl200,
    /// Firmware matched no model. C returns from the axis constructor with
    /// `conexModel_` unset and keeps polling; here it matches no model gate.
    Unknown,
}

/// Closed-loop gain maxima used to scale the 0..=1 motor-record gains onto the
/// controller's native range (C `KPMax_`/`KIMax_`/`KDMax_`/`LFMax_`).
struct GainMax {
    kp: f64,
    ki: f64,
    kd: f64,
    lf: f64,
}

impl ConexModel {
    /// C `AG_CONEXAxis::AG_CONEXAxis`: model detection via substring match on
    /// the firmware version string, in the same priority order as the C
    /// `strstr` chain. Returns `None` for an unrecognized string.
    pub fn from_version(version: &str) -> Option<Self> {
        if version.contains("CONEX-AGP") {
            Some(Self::Agp)
        } else if version.contains("CONEX-CC") {
            Some(Self::Cc)
        } else if version.contains("Conex PP") {
            Some(Self::Pp)
        } else if version.contains("FC series") {
            Some(Self::Fcl200)
        } else if version.contains("DL") {
            Some(Self::Dl)
        } else {
            None
        }
    }

    /// CC and AGP carry an incremental encoder; their step size derives from
    /// the encoder increment / interpolation factor, and they report gain
    /// support to the motor record (C `motorStatusGainSupport_`).
    fn is_cc_or_agp(self) -> bool {
        matches!(self, Self::Cc | Self::Agp)
    }

    /// CC/PP/FCL200/DL accept the `AC`/`VA` accel+velocity preamble on a move;
    /// AGP does not (C `AG_CONEXAxis::move`).
    fn supports_velocity(self) -> bool {
        !matches!(self, Self::Agp | Self::Unknown)
    }

    /// PID gain commands apply only to AGP/CC/DL (C `setPGain`/`setIGain`/
    /// `setDGain` guard).
    fn supports_pid(self) -> bool {
        matches!(self, Self::Agp | Self::Cc | Self::Dl)
    }

    /// C constructor gain maxima. C sets these only for AGP and CC; DL/PP/
    /// FCL200 leave them uninitialized (read as garbage in the C PID path).
    /// Rust uses a defined `0.0`, so PID writes on DL send `0` — flagged as a
    /// deliberate deviation from C's undefined behavior.
    fn gain_max(self) -> GainMax {
        match self {
            Self::Agp => GainMax {
                kp: 3000.,
                ki: 3000.,
                kd: 0.,
                lf: 1000.,
            },
            Self::Cc => GainMax {
                kp: 1.0e6,
                ki: 1.0e6,
                kd: 1.0e6,
                lf: 0.,
            },
            _ => GainMax {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                lf: 0.,
            },
        }
    }
}

/// C `AG_CONEXAxis::AG_CONEXAxis` step-size computation.
///
/// `encoder_increment`/`interpolation_factor` come from `SU?`/`IF?` (CC/AGP),
/// `full_step_size`/`micro_steps_per_full_step` from `FRS?`/`FRM?` (PP/FCL200).
fn compute_step_size(
    model: ConexModel,
    encoder_increment: f64,
    interpolation_factor: f64,
    full_step_size: f64,
    micro_steps_per_full_step: i32,
) -> f64 {
    match model {
        ConexModel::Agp | ConexModel::Cc => encoder_increment / interpolation_factor,
        ConexModel::Fcl200 => 1.0,
        // DL operates in mm; the motor record wants an integer readback, so C
        // scales by 1e4 and sets resolution 1e-4.
        ConexModel::Dl => 0.0001,
        // CONEX-PP.
        ConexModel::Pp => full_step_size / micro_steps_per_full_step as f64 / 1000.0,
        // C never computes stepSize_ for an unknown model.
        ConexModel::Unknown => 0.0,
    }
}

/// Decoded `TS` status: motion done + soft-limit flags.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct TsStatus {
    done: bool,
    low_limit: bool,
    high_limit: bool,
}

/// Parse a `TS` (tell-status) reply. C `AG_CONEXAxis::poll`:
/// - DL: `"%*dTS%*6c%x"` — skip 6 chars after `TS`, read the 2-char state hex;
///   moving when state `== 0x3c`.
/// - others: `"%*dTS%*4c%x"` — skip 4 chars after `TS`, read the state byte;
///   moving when `state & 0xff` is `0x1e` or `0x28`. Only CC/PP consult the
///   `0x100`/`0x200` limit bits — which the 2-char read can never set, so they
///   stay inert exactly as in C.
fn parse_ts(model: ConexModel, resp: &str) -> Option<TsStatus> {
    let idx = resp.find("TS")?;
    let after = &resp[idx + 2..];
    let skip = if model == ConexModel::Dl { 6 } else { 4 };
    let hex_part = after.get(skip..)?;
    let status = leading_hex(hex_part)?;

    let done = if model == ConexModel::Dl {
        status != 0x3c
    } else {
        let state = status & 0xff;
        !(state == 0x1e || state == 0x28)
    };

    let (mut low_limit, mut high_limit) = (false, false);
    if matches!(model, ConexModel::Cc | ConexModel::Pp) {
        if status & 0x100 != 0 {
            low_limit = true;
        }
        if status & 0x200 != 0 {
            high_limit = true;
        }
    }
    Some(TsStatus {
        done,
        low_limit,
        high_limit,
    })
}

/// Parse an `MM?` reply into the closed-loop (power-on) flag. C
/// `AG_CONEXAxis::getClosedLoop`: `sscanf(inString_, "%*dMM%x", &status)`, then
/// closed-loop when the state is in `0x1e..=0x34`.
fn parse_closed_loop(resp: &str) -> Option<bool> {
    let idx = resp.find("MM")?;
    let status = leading_hex(&resp[idx + 2..])?;
    Some((0x1e..=0x34).contains(&status))
}

// --- Command formatters (bare text, C `outString_`).

fn cmd_set_acceleration(id: i32, accel_egu: f64) -> String {
    format!("{id}AC{accel_egu:.6}", accel_egu = CFloat(accel_egu))
}
fn cmd_set_velocity(id: i32, velocity_egu: f64) -> String {
    format!(
        "{id}VA{velocity_egu:.6}",
        velocity_egu = CFloat(velocity_egu)
    )
}
fn cmd_move_absolute(id: i32, position_egu: f64) -> String {
    format!(
        "{id}PA{position_egu:.6}",
        position_egu = CFloat(position_egu)
    )
}
fn cmd_move_relative(id: i32, distance_egu: f64) -> String {
    format!(
        "{id}PR{distance_egu:.6}",
        distance_egu = CFloat(distance_egu)
    )
}
fn cmd_stop(id: i32) -> String {
    format!("{id}ST")
}
fn cmd_reset(id: i32) -> String {
    format!("{id}RS")
}
fn cmd_home(id: i32) -> String {
    format!("{id}OR")
}
fn cmd_dl_init(id: i32) -> String {
    format!("{id}IE")
}
fn cmd_home_velocity(id: i32, velocity: f64) -> String {
    format!("{id}OH{velocity:.6}", velocity = CFloat(velocity))
}
fn cmd_home_type(id: i32, home_type: i32) -> String {
    format!("{id}HT{home_type}")
}
fn cmd_set_closed_loop(id: i32, enable: bool) -> String {
    format!("{id}MM{}", if enable { 1 } else { 0 })
}
fn cmd_set_gain(id: i32, letters: &str, value: f64) -> String {
    format!("{id}{letters}{value:.6}", value = CFloat(value))
}
/// A bare query command: `{id}{code}` (e.g. `1TP`, `1SU?`).
fn query(id: i32, code: &str) -> String {
    format!("{id}{code}")
}

fn conex_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

fn parse_error(what: &str) -> AsynError {
    conex_err(format!("CONEX: could not parse {what} response"))
}

// --- Raw serial I/O (usable before `ConexAxis` exists, e.g. in `new`).

/// C `writeCONEX`: every write, failed or not, is followed by the Linux
/// `LINUX_WRITE_DELAY` before the next command.
fn write_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<()> {
    let status = handle.write_octet(0, cmd.as_bytes()).map(drop);
    if cfg!(target_os = "linux") {
        thread::sleep(LINUX_WRITE_DELAY);
    }
    status
}

fn write_read_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<String> {
    let (reply, status) = exchange(handle, cmd);
    status.map(|()| reply)
}

/// C `writeReadController`: `inString_` beside the status. The EOS layer
/// NUL-terminates the buffer on every read, so a failed read leaves the bytes
/// it delivered, which the callers that ignore the status parse.
fn exchange(handle: &SyncIOHandle, cmd: &str) -> (String, AsynResult<()>) {
    let (reply, status) = handle.write_read(0, cmd.as_bytes(), READ_BUF).into_parts();
    (String::from_utf8_lossy(&reply).into_owned(), status)
}

/// A Newport CONEX single-axis driver implementing [`AsynMotor`].
pub struct ConexAxis {
    handle: Arc<SyncIOHandle>,
    /// Numeric controller ID prefixed on every command (C `controllerID_`).
    controller_id: i32,
    model: ConexModel,
    /// Controller units per record step (C `stepSize_`).
    step_size: f64,
    /// Soft travel limits in controller EGU (C `lowLimit_`/`highLimit_`).
    low_limit: f64,
    high_limit: f64,
    /// C `controllerVersion_`, the `VE` reply past its echo.
    version: String,
    /// What only `report` prints. C leaves it unset for an unknown model.
    shown: Shown,
    /// C `currentPosition_`, which `poll` sets.
    current_position: Arc<Mutex<f64>>,
    /// The constructor's gain-support set (AG_CONEX.cpp:271-274).
    status_writes: Vec<StatusWrite>,
}

/// The constructor's readings that only C's `report` prints.
#[derive(Clone, Default)]
struct Shown {
    stage_id: String,
    encoder_increment: f64,
    interpolation_factor: f64,
    full_step_size: f64,
    micro_steps_per_full_step: i32,
}

/// C `AG_CONEXAxis::report` (AG_CONEX.cpp:283-319): the gains, read off
/// the controller at each report, then the constructor's readings.
pub struct ConexAxisReport {
    handle: Arc<SyncIOHandle>,
    controller_id: i32,
    model: ConexModel,
    step_size: f64,
    low_limit: f64,
    high_limit: f64,
    shown: Shown,
    current_position: Arc<Mutex<f64>>,
    /// C `KP_`/`KI_`/`LF_`, kept from the last report that read them.
    gains: Mutex<(f64, f64, f64)>,
}

impl AxisReport for ConexAxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level <= 0 {
            return;
        }
        // C ignores each read's status and parses what arrived. C defect, not
        // copied: a refused write leaves inString_ the previous reply, which
        // AG_CONEX.cpp:287-301 then parse.
        let read = |code: &str| {
            let (reply, _) = exchange(&self.handle, &query(self.controller_id, code));
            parse_value_at(&reply, 3).unwrap_or(0.0)
        };
        let mut gains = self.gains.lock().unwrap_or_else(|e| e.into_inner());
        if matches!(
            self.model,
            ConexModel::Agp | ConexModel::Cc | ConexModel::Dl
        ) {
            gains.0 = read("KP?");
            gains.1 = read("KI?");
            gains.2 = if self.model == ConexModel::Agp {
                read("LF?")
            } else {
                read("KD?")
            };
        }
        let (kp, ki, lf) = *gains;
        let shown = &self.shown;
        let current = *self
            .current_position
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let _ = write!(
            out,
            "  stageID={}\n  currentPosition={current:.6}, encoderIncrement={:.6}\n  \
             interpolationFactor={:.6}, stepSize={:.6}, lowLimit={:.6}, highLimit={:.6}\n  \
             KP={kp:.6}, KI={ki:.6}, KD/LF={lf:.6}\n  \
             fullStepSize={:.6}, microStepsPerFullStep={}\n",
            shown.stage_id,
            CFloat(shown.encoder_increment),
            CFloat(shown.interpolation_factor),
            CFloat(self.step_size),
            CFloat(self.low_limit),
            CFloat(self.high_limit),
            CFloat(shown.full_step_size),
            shown.micro_steps_per_full_step,
            current = CFloat(current),
            kp = CFloat(kp),
            ki = CFloat(ki),
            lf = CFloat(lf)
        );
    }
}

impl ConexAxis {
    /// Connect and identify a CONEX controller on `handle` with the given
    /// controller ID, running the C `AG_CONEXAxis` constructor's query
    /// sequence (version → model → stage/encoder/limit queries → step size).
    ///
    /// Performs blocking serial I/O; call it where that is acceptable (the
    /// iocsh create command, at IOC init). `None` only when `VE` fails.
    /// `trace` is C's `pasynUserSelf`.
    pub fn new(handle: SyncIOHandle, controller_id: i32, trace: &MotorTrace) -> Option<Self> {
        let handle = Arc::new(handle);
        // C writeReadController status is ignored past VE: a failed read is
        // parsed for what it delivered, and atof reads nothing as 0. C defect,
        // not copied: a refused write leaves inString_ the previous reply,
        // which AG_CONEX.cpp:216-267 then parse; here it is empty.
        let reply = |cmd: &str| exchange(&handle, &query(controller_id, cmd)).0;
        let value_at = |cmd: &str, at: usize| parse_value_at(&reply(cmd), at).unwrap_or(0.0);

        // Firmware version → model. C reads `&inString_[4]` for the VE reply
        // and creates neither the axis nor the poller when VE fails.
        let Ok(version_resp) = write_read_raw(&handle, &query(controller_id, "VE")) else {
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "AG_CONEXController::AG_CONEXController: cannot read version information from AG_CONEX controller\n"
                ),
            );
            return None;
        };
        let version = version_resp.get(4..).unwrap_or("").to_string();
        let Some(model) = ConexModel::from_version(&version) else {
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "AG_CONEXAxis::AG_CONEXAxis: unknown model, firmware string={version}\n"
                ),
            );
            return Some(Self {
                handle,
                controller_id,
                model: ConexModel::Unknown,
                step_size: 0.0,
                low_limit: 0.0,
                high_limit: 0.0,
                version,
                shown: Shown::default(),
                current_position: Arc::default(),
                status_writes: Vec::new(),
            });
        };

        let stage_id = reply("ID?").get(3..).unwrap_or("").to_string();

        // Encoder increment (CC/AGP only), else 1.0.
        let encoder_increment = if model.is_cc_or_agp() {
            value_at("SU?", 3)
        } else {
            1.0
        };

        // Interpolation factor (AGP only), else 1.0.
        let interpolation_factor = if model == ConexModel::Agp {
            value_at("IF?", 3)
        } else {
            1.0
        };

        // Full-step size / microsteps (PP and FCL200 only). C reads
        // `&inString_[4]` for the FRM?/FRS? replies.
        let (mut full_step_size, mut micro_steps_per_full_step) = (0.0, 0);
        if matches!(model, ConexModel::Pp | ConexModel::Fcl200) {
            micro_steps_per_full_step = parse_int_at(&reply("FRM?"), 4).unwrap_or(0);
            full_step_size = value_at("FRS?", 4);
        } else if model == ConexModel::Dl {
            // AG_CONEX.cpp:257 sets these for the report only.
            (full_step_size, micro_steps_per_full_step) = (1.0, 1);
        }

        let step_size = compute_step_size(
            model,
            encoder_increment,
            interpolation_factor,
            full_step_size,
            micro_steps_per_full_step,
        );

        // Soft travel limits (already controller EGU).
        let low_limit = value_at("SL?", 3);
        let high_limit = value_at("SR?", 3);

        Some(Self {
            handle,
            controller_id,
            model,
            step_size,
            low_limit,
            high_limit,
            version,
            shown: Shown {
                stage_id,
                encoder_increment,
                interpolation_factor,
                full_step_size,
                micro_steps_per_full_step,
            },
            current_position: Arc::default(),
            status_writes: if model.is_cc_or_agp() {
                vec![StatusWrite::GainSupport(true)]
            } else {
                Vec::new()
            },
        })
    }

    /// C `controllerVersion_`.
    pub fn version(&self) -> &str {
        &self.version
    }

    /// The axis's `report`.
    pub fn report(&self) -> ConexAxisReport {
        ConexAxisReport {
            handle: self.handle.clone(),
            controller_id: self.controller_id,
            model: self.model,
            step_size: self.step_size,
            low_limit: self.low_limit,
            high_limit: self.high_limit,
            shown: self.shown.clone(),
            current_position: self.current_position.clone(),
            gains: Mutex::default(),
        }
    }

    fn write(&self, cmd: &str) -> AsynResult<()> {
        write_raw(&self.handle, cmd)
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        write_read_raw(&self.handle, cmd)
    }

    /// C `AG_CONEXAxis::getClosedLoop`: the `MM?` reply parsed whatever the
    /// exchange returned, beside its status. AG_CONEX.cpp:430 reads an
    /// uninitialised status on a bad reply, a C defect; that is `None` here.
    fn get_closed_loop(&self) -> (Option<bool>, AsynResult<()>) {
        let (resp, status) = exchange(&self.handle, &query(self.controller_id, "MM?"));
        (parse_closed_loop(&resp), status)
    }

    /// `stepSize_`, refused at 0 or non-finite (module Units note).
    fn step(&self) -> AsynResult<f64> {
        let step = self.step_size;
        if step == 0.0 || !step.is_finite() {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!(
                    "CONEX {}: step size is {}",
                    self.controller_id,
                    CFloat(step).g()
                ),
            });
        }
        Ok(step)
    }

    /// C `AG_CONEXAxis::move` velocity/accel preamble (model-gated), in
    /// steps times `stepSize_`, whose statuses the move overwrites
    /// (AG_CONEX.cpp:322-342).
    fn send_accel_and_velocity(&self, acceleration: f64, velocity: f64) -> AsynResult<()> {
        let step = self.step()?;
        if self.model.supports_velocity() {
            let _ = self.write(&cmd_set_acceleration(
                self.controller_id,
                acceleration * step,
            ));
            let _ = self.write(&cmd_set_velocity(self.controller_id, velocity * step));
        }
        Ok(())
    }
}

impl AsynMotor for ConexAxis {
    /// C `AG_CONEXAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (AG_CONEX.cpp).
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
        self.send_accel_and_velocity(acceleration, velocity)?;
        let position = position * self.step()?;
        self.write(&cmd_move_absolute(self.controller_id, position))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `move()` has a native relative branch (PR); override the trait's
        // poll-then-absolute default to match.
        self.send_accel_and_velocity(acceleration, velocity)?;
        let distance = distance * self.step()?;
        self.write(&cmd_move_relative(self.controller_id, distance))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: no jog command; move almost to the soft limit in
        // the jog direction. Limits are already controller EGU.
        let position = if velocity > 0.0 {
            self.high_limit - self.step_size
        } else {
            self.low_limit + self.step_size
        };
        self.write(&cmd_move_absolute(self.controller_id, position))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `home()`: reset to unreferenced state, then home. The blocking
        // sleeps mirror C `epicsThreadSleep` and run in the same blocking
        // context as the driver's serial I/O. Each write's status is
        // overwritten by the next one's (AG_CONEX.cpp:344-377).
        let _ = self.write(&cmd_reset(self.controller_id));
        thread::sleep(Duration::from_secs(1));

        if self.model == ConexModel::Dl {
            let _ = self.write(&cmd_dl_init(self.controller_id));
            thread::sleep(Duration::from_secs(5));
        }

        // CONEX-PP: set home velocity (unscaled, as C) and force minus-EOR
        // home type.
        if self.model == ConexModel::Pp {
            let _ = self.write(&cmd_home_velocity(self.controller_id, velocity));
            let _ = self.write(&cmd_home_type(self.controller_id, HOME_TYPE_MINUS_EOR));
        }

        self.write(&cmd_home(self.controller_id))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.write(&cmd_stop(self.controller_id))
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C `setPosition` has no functional body — intentional no-op.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.write(&cmd_set_closed_loop(self.controller_id, enable))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C `setPGain`/`setIGain`/`setDGain`: gated to AGP/CC/DL; briefly drop
        // closed loop, write the scaled gain, then restore closed loop.
        if !self.model.supports_pid() {
            return Ok(());
        }
        let max = self.model.gain_max();
        // C ignores getClosedLoop's and setClosedLoop's statuses and returns
        // the gain write's (AG_CONEX.cpp:435-489).
        let was_closed = self.get_closed_loop().0.unwrap_or(false);
        let _ = self.write(&cmd_set_closed_loop(self.controller_id, false));

        let cmd = match kind {
            PidGainKind::Proportional => cmd_set_gain(self.controller_id, "KP", gain * max.kp),
            PidGainKind::Integral => cmd_set_gain(self.controller_id, "KI", gain * max.ki),
            PidGainKind::Derivative => match self.model {
                // Parity: C `setDGain` emits `KI` (with `KDMax`) for CC/DL.
                ConexModel::Cc | ConexModel::Dl => {
                    cmd_set_gain(self.controller_id, "KI", gain * max.kd)
                }
                // AGP repurposes D gain as the low-pass filter frequency (LF).
                ConexModel::Agp => cmd_set_gain(self.controller_id, "LF", gain * max.lf),
                _ => return Ok(()),
            },
        };
        let status = self.write(&cmd);

        if was_closed {
            let _ = self.write(&cmd_set_closed_loop(self.controller_id, true));
        }
        status
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    /// A failed read returns at `skip` as C returns asynError, keeping what
    /// the reads before it set (AG_CONEX.cpp:510-563).
    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        // Position (TP): controller units to record steps.
        let step = self.step()?;
        let position = parse_value_at(&self.write_read(&query(self.controller_id, "TP"))?, 3)
            .ok_or_else(|| parse_error("TP"))?
            / step;
        *self
            .current_position
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = position;
        status.position = position;

        // Status (TS): done + limits (model-dependent parse). AG_CONEX.cpp:530,534
        // posts PROBLEM 0 on a bad reply, a C defect.
        let ts = parse_ts(
            self.model,
            &self.write_read(&query(self.controller_id, "TS"))?,
        )
        .ok_or_else(|| parse_error("TS"))?;
        status.done = ts.done;
        status.moving = !ts.done;
        status.low_limit = ts.low_limit;
        status.high_limit = ts.high_limit;

        // Power-on = closed-loop enabled (MM?).
        let (powered, mm) = self.get_closed_loop();
        mm?;
        status.powered = powered.ok_or_else(|| parse_error("MM?"))?;
        status.problem = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use motor_common::testing::Polled;

    /// AG_CONEX.cpp:217-268 and :442 ignore writeReadController's status
    /// past VE and parse `inString_`, so replies cut by a timeout still set
    /// the step size and limits, and a gain write still restores the closed
    /// loop `MM?` reported.
    #[test]
    fn replies_cut_by_a_timeout_are_still_parsed() {
        let (line, io) = LineRig::cut(
            "conex_cut",
            |last| {
                match last {
                    "1VE" => "1VE CONEX-CC 2.0",
                    "1SU?" => "1SU0.0001",
                    "1SL?" => "1SL-12.5",
                    "1SR?" => "1SR12.5",
                    "1MM?" => "1MM33",
                    _ => "",
                }
                .to_string()
            },
            |last| last != "1VE",
        );
        let mut axis = ConexAxis::new(io, 1, &MotorTrace::new("conex_cut", 0)).unwrap();
        assert_eq!(
            (axis.step_size, axis.low_limit, axis.high_limit),
            (0.0001, -12.5, 12.5)
        );
        line.sent();
        axis.set_pid_gain(&AsynUser::default(), PidGainKind::Proportional, 0.5)
            .unwrap();
        let sent = line.sent();
        assert_eq!(
            sent.last().map(String::as_str),
            Some(cmd_set_closed_loop(1, true).as_str()),
            "{sent:?}"
        );
    }

    /// AG_CONEX.cpp: a CONEX-CC's stepSize_ is SU? / 1; moves, VA and AC
    /// are steps * stepSize_, TP / stepSize_, the jog target the soft limit
    /// less one stepSize_, unscaled.
    #[test]
    fn record_steps_scale_by_step_size() {
        let (line, io) = LineRig::new("conex_steps", |last| {
            match last {
                "1VE" => "1VE CONEX-CC 2.0",
                "1SU?" => "1SU0.0001",
                "1SL?" => "1SL-12.5",
                "1SR?" => "1SR12.5",
                "1TP" => "1TP2.5",
                "1TS" => "1TS000033",
                "1MM?" => "1MM33",
                _ => "",
            }
            .to_string()
        });
        let trace = MotorTrace::new("conex_steps", 0);
        let mut axis = ConexAxis::new(io, 1, &trace).unwrap();
        line.sent();
        let user = AsynUser::default();
        axis.move_absolute(&user, 20000.0, 0.0, 5000.0, 10000.0)
            .unwrap();
        axis.move_relative(&user, -100.0, 0.0, 5000.0, 10000.0)
            .unwrap();
        axis.move_velocity(&user, 0.0, 5000.0, 10000.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "1AC1.000000",
                "1VA0.500000",
                "1PA2.000000",
                "1AC1.000000",
                "1VA0.500000",
                "1PR-0.010000",
                "1PA12.499900"
            ]
        );
        assert_eq!(axis.polled(&user).unwrap().position, 25000.0);
    }

    /// AG_CONEX.cpp:516-522: a refused TS skips after TP's position is set,
    /// so the position survives and Done keeps what a move start set.
    #[test]
    fn a_failed_ts_keeps_the_position_tp_read() {
        let (_line, io) = LineRig::failing(
            "conex_ts_skip",
            |last| {
                match last {
                    "1VE" => "1VE CONEX-CC 2.0",
                    "1SU?" => "1SU0.0001",
                    "1TP" => "1TP2.5",
                    _ => "",
                }
                .to_string()
            },
            |cmd| cmd == "1TS",
        );
        let mut axis = ConexAxis::new(io, 1, &MotorTrace::new("conex_ts_skip", 0)).unwrap();
        let mut status = MotorStatus {
            done: false,
            ..Default::default()
        };
        assert!(axis.poll(&AsynUser::default(), &mut status).is_err());
        assert_eq!(status.position, 25000.0);
        assert!(!status.done);
    }

    /// AG_CONEXAxis::poll sets motorPosition_ only (AG_CONEX.cpp:516): the
    /// record's REP stays 0.
    #[test]
    fn the_poll_sets_no_encoder_position() {
        let (_line, io) = LineRig::new("conex_no_rep", |last| {
            match last {
                "1VE" => "1VE CONEX-CC 2.0",
                "1SU?" => "1SU0.0001",
                "1TP" => "1TP2.5",
                "1TS" => "1TS000033",
                "1MM?" => "1MM33",
                _ => "",
            }
            .to_string()
        });
        let mut axis = ConexAxis::new(io, 1, &MotorTrace::new("conex_no_rep", 0)).unwrap();
        let status = axis.polled(&AsynUser::default()).unwrap();
        assert_eq!((status.position, status.encoder_position), (25000.0, 0.0));
    }

    /// AG_CONEX.cpp:271-274: gain support is the constructor's, CC and AGP
    /// only.
    #[test]
    fn the_constructor_sets_gain_support_for_cc_and_agp() {
        for (version, writes) in [
            ("1VE CONEX-CC 2.0", vec![StatusWrite::GainSupport(true)]),
            ("1VE CONEX-AGP 2.0", vec![StatusWrite::GainSupport(true)]),
            ("1VE Conex PP 2.0", vec![]),
        ] {
            let (_line, io) = LineRig::new(version, move |last| match last {
                "1VE" => version.to_string(),
                _ => String::new(),
            });
            let mut axis = ConexAxis::new(io, 1, &MotorTrace::new("conex_gain", 0)).unwrap();
            assert_eq!(axis.take_status_writes(), writes, "{version}");
        }
    }

    /// C sets `status =` on each write and returns the last
    /// (AG_CONEX.cpp:322-377); a gain write returns the gain's status, the
    /// closed-loop writes around it unchecked (:435-450).
    #[test]
    fn a_refused_setting_still_sends_the_command_after_it() {
        let (line, io) = LineRig::failing(
            "conex_refused",
            |last| {
                match last {
                    "1VE" => "1VE CONEX-CC 2.0",
                    "1SU?" => "1SU0.0001",
                    "1MM?" => "1MM33",
                    _ => "",
                }
                .to_string()
            },
            |last| {
                last.starts_with("1AC")
                    || last.starts_with("1KP")
                    || ["1RS", "1MM0"].contains(&last)
            },
        );
        let mut axis = ConexAxis::new(io, 1, &MotorTrace::new("conex_refused", 0)).unwrap();
        line.sent();
        let user = AsynUser::default();
        axis.move_absolute(&user, 20000.0, 0.0, 5000.0, 10000.0)
            .unwrap();
        axis.home(&user, 0.0, 5000.0, 10000.0, true).unwrap();
        assert_eq!(
            line.sent(),
            ["1AC1.000000", "1VA0.500000", "1PA2.000000", "1RS", "1OR"]
        );
        assert!(
            axis.set_pid_gain(&user, PidGainKind::Proportional, 0.5)
                .is_err()
        );
        let sent = line.sent();
        assert_eq!(sent[1..], ["1MM0", sent[2].as_str(), "1MM1"], "{sent:?}");
    }

    /// AG_CONEX.cpp `writeCONEX`: the poll after a move waits out
    /// `LINUX_WRITE_DELAY` per write (AC, VA, PA on a CONEX-CC).
    #[cfg(target_os = "linux")]
    #[test]
    fn each_write_holds_the_line_for_the_write_delay() {
        let (_line, io) = LineRig::new("conex_delay", |last| {
            match last {
                "1VE" => "1VE CONEX-CC 2.0",
                "1SU?" => "1SU0.0001",
                _ => "",
            }
            .to_string()
        });
        let mut axis = ConexAxis::new(io, 1, &MotorTrace::new("conex_delay", 0)).unwrap();
        let start = std::time::Instant::now();
        axis.move_absolute(&AsynUser::default(), 100.0, 0.0, 5000.0, 10000.0)
            .unwrap();
        assert!(start.elapsed() >= 3 * LINUX_WRITE_DELAY);
    }

    #[test]
    fn model_detected_from_firmware_string() {
        assert_eq!(
            ConexModel::from_version("CONEX-AGP-blah"),
            Some(ConexModel::Agp)
        );
        assert_eq!(
            ConexModel::from_version("CONEX-CC v2"),
            Some(ConexModel::Cc)
        );
        assert_eq!(ConexModel::from_version("Conex PP"), Some(ConexModel::Pp));
        assert_eq!(
            ConexModel::from_version("FC series"),
            Some(ConexModel::Fcl200)
        );
        assert_eq!(
            ConexModel::from_version("DL something"),
            Some(ConexModel::Dl)
        );
        assert_eq!(ConexModel::from_version("unknown box"), None);
    }

    #[test]
    fn agp_and_cc_precede_dl_in_detection() {
        // "CONEX-CC" contains no "DL", but a stray "DL" substring must not
        // outrank a real CONEX match — order mirrors the C strstr chain.
        assert_eq!(
            ConexModel::from_version("CONEX-AGP DL"),
            Some(ConexModel::Agp)
        );
    }

    #[test]
    fn unknown_model_matches_no_model_gate() {
        let m = ConexModel::Unknown;
        assert!(!m.is_cc_or_agp() && !m.supports_velocity() && !m.supports_pid());
        assert_eq!(compute_step_size(m, 1.0, 1.0, 1.0, 1), 0.0);
        // Non-DL TS layout, no limit bits.
        assert_eq!(
            parse_ts(m, "1TS000028"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
    }

    #[test]
    fn step_size_is_model_specific() {
        // CC/AGP: encoderIncrement / interpolationFactor.
        assert_eq!(
            compute_step_size(ConexModel::Cc, 2.0e-5, 1.0, 0.0, 0),
            2.0e-5
        );
        assert_eq!(
            compute_step_size(ConexModel::Agp, 1.0e-3, 4.0, 0.0, 0),
            2.5e-4
        );
        // FCL200 in mm.
        assert_eq!(compute_step_size(ConexModel::Fcl200, 0.0, 1.0, 0.0, 0), 1.0);
        // DL fixed 1e-4.
        assert_eq!(compute_step_size(ConexModel::Dl, 0.0, 1.0, 0.0, 0), 0.0001);
        // PP: fullStepSize / microSteps / 1000.
        assert_eq!(
            compute_step_size(ConexModel::Pp, 0.0, 1.0, 2.0, 400),
            2.0 / 400.0 / 1000.0
        );
    }

    #[test]
    fn command_formatting_matches_c_sprintf() {
        assert_eq!(cmd_move_absolute(1, 5.0), "1PA5.000000");
        assert_eq!(cmd_move_relative(2, -1.25), "2PR-1.250000");
        assert_eq!(cmd_set_acceleration(1, 10.0), "1AC10.000000");
        assert_eq!(cmd_set_velocity(1, 2.5), "1VA2.500000");
        assert_eq!(cmd_stop(3), "3ST");
        assert_eq!(cmd_home(1), "1OR");
        assert_eq!(cmd_reset(1), "1RS");
        assert_eq!(cmd_set_closed_loop(1, true), "1MM1");
        assert_eq!(cmd_set_closed_loop(1, false), "1MM0");
        assert_eq!(cmd_home_type(1, HOME_TYPE_MINUS_EOR), "1HT4");
        assert_eq!(cmd_set_gain(1, "KP", 1500.0), "1KP1500.000000");
        assert_eq!(query(1, "TP"), "1TP");
        assert_eq!(query(2, "SU?"), "2SU?");
    }

    #[test]
    fn parse_ts_non_dl_moving_and_ready_states() {
        // "1TS" + 4 error chars (abcd) + 2 state chars. State 0x1e / 0x28 =
        // moving; 0x33 (READY) = done.
        assert_eq!(
            parse_ts(ConexModel::Cc, "1TS00001e"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
        assert_eq!(
            parse_ts(ConexModel::Cc, "1TS000028"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
        assert_eq!(
            parse_ts(ConexModel::Cc, "1TS000033"),
            Some(TsStatus {
                done: true,
                low_limit: false,
                high_limit: false,
            })
        );
    }

    #[test]
    fn parse_ts_dl_uses_last_two_state_chars() {
        // DL: "1TS" + 6 chars + 2 state chars; moving when state == 0x3c.
        assert_eq!(
            parse_ts(ConexModel::Dl, "1TS0000003c"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
        assert_eq!(
            parse_ts(ConexModel::Dl, "1TS00000032"),
            Some(TsStatus {
                done: true,
                low_limit: false,
                high_limit: false,
            })
        );
    }

    #[test]
    fn parse_ts_rejects_reply_too_short_to_hold_state() {
        assert_eq!(parse_ts(ConexModel::Cc, "1TS12"), None);
        assert_eq!(parse_ts(ConexModel::Cc, "garbage"), None);
    }

    #[test]
    fn parse_closed_loop_reads_mm_state_range() {
        // Closed loop for state in 0x1e..=0x34.
        assert_eq!(parse_closed_loop("1MM1e"), Some(true));
        assert_eq!(parse_closed_loop("1MM34"), Some(true));
        assert_eq!(parse_closed_loop("1MM0a"), Some(false));
        assert_eq!(parse_closed_loop("1MM3c"), Some(false));
        assert_eq!(parse_closed_loop("1XX"), None);
    }
}
