//! Newport New Focus PMNC 8750/8752 picomotor network-controller driver
//! (ASCII, prompt-framed).
//!
//! Ported from `motorNewFocus/newFocusApp/src/drvPMNC87xx.cc` +
//! `devPMNC87xx.cc` (a model-1 dev/drv pair, itself derived from
//! drvMM3000). A controller holds several *driver modules*, each addressed
//! `A{driver}`; a module is either an 8753 (3-channel open-loop) or an 8751
//! (1-channel closed-loop). An asyn-rs axis therefore maps to a
//! (driverType, driverNum, motorNum) triple discovered at startup.
//!
//! ## Prompt protocol
//!
//! Commands are terminated with CR (`\r`); the controller answers with the
//! reply text followed by a `>` prompt. [`PmncController::new`] sets the port
//! output EOS to CR and input EOS to the prompt, as C `motor_init` does, so
//! each `PmncController::command` reads exactly one reply. On firmware
//! 1.5.3/1.5.4, whose `STA` reply ends in `\n` instead, the input EOS is
//! switched to `\n` around that one read (C `changeEOS`).
//! Every command yields a prompt (C `cmnd_response = true`), so motion
//! commands consume and discard their reply just like queries.
//!
//! ## Units
//!
//! Picomotor moves are integer *steps*: positions, velocities (steps/sec,
//! clamped to 2000), and accelerations (steps/sec², clamped 16..32000) are
//! all counts: the record steps that cross the motor boundary, sent with C's
//! `NINT`, so any MRES works as in C.
//!
//! ## Open-loop position tracking (8753)
//!
//! An 8753 cannot report absolute position, so C accumulates it in software:
//! `POS` reads a per-move relative count that is zeroed after each move, and
//! the driver keeps a running `position += (reading - last_reading)`. An
//! absolute move becomes a relative `target - tracked_position`. The 8751
//! reports true absolute position and needs none of this. Both behaviours
//! are kept here per axis.
//!
//! ## Deviations from C (documented)
//!
//! - **Homing is a no-op**, exactly as C: `HOME_FOR`/`HOME_REV` send no
//!   command (the `FIN`/`RIN` paths are commented out upstream).

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::{atoi, nint};
use motor_common::model1::MoveStrings;

/// Response buffer size for a single controller reply (C `BUFF_SIZE` 100).
const READ_BUF: usize = 256;

/// Command line terminator (C `CMND_EOS`).
const CMND_EOS: &[u8] = b"\r";

/// Reply terminator for every command (C `PROMPT_EOS`).
const PROMPT_EOS: &[u8] = b">";

/// `STA` reply terminator on firmware 1.5.3/1.5.4 (C `NL_EOS`).
const NL_EOS: &[u8] = b"\n";

/// Identity substring that precedes the firmware digit (C `VER_STR`).
const VER_STR: &str = "Version 1.";

/// Motor characteristics in steps (C drvPMNCCom.h).
const MAX_VELOCITY: i64 = 2000;
const MIN_ACCEL: i64 = 16;
const MAX_ACCEL: i64 = 32000;

/// 8753 open-loop status byte bits (LSB-first union, C `Bits_8753`).
const S8753_IN_MOTION: u8 = 0x01;
const S8753_POWER_ON: u8 = 0x04;

/// 8751 closed-loop status byte bits (servo-on/power-on layout,
/// C `Bits_8751`).
const S8751_MOVE_DONE: u8 = 0x01;
const S8751_POWER_ON: u8 = 0x08;
const S8751_REV_LIMIT: u8 = 0x20;
const S8751_FOR_LIMIT: u8 = 0x40;
const S8751_HOMING: u8 = 0x80;

/// 8751 aux (DIAG) status bits (C `Bits_8751` aux union).
const AUX8751_SERVO_ON: u8 = 0x04;

fn pmnc_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

/// Controller communication state (C `cntrl->status`, the `CHECKRTN` macro):
/// one empty/garbled reply is retried silently; a second consecutive one is
/// a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Picomotor network-controller model (C `PMNC_model`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PmncModel {
    /// 8750: only 8753 driver modules, axis count found indirectly (`MPV`).
    Pmnc8750,
    /// 8752: mixed driver modules, queried directly (`DRT`).
    Pmnc8752,
}

/// Picomotor driver-module model (C `PMD_model`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PmdModel {
    /// 8753: 3-channel open-loop.
    Pmd8753,
    /// 8751: 1-channel closed-loop.
    Pmd8751,
}

/// One axis's controller addressing (C `PMD_axis`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AxisDef {
    driver_type: PmdModel,
    driver_num: i32,
    motor_num: i32,
}

/// Parse `A{driver}=0x{hex}` (C `sscanf(pStr, "A%d=0x%x", ...)`); returns the
/// status byte.
fn parse_status_byte(reply: &str) -> Option<u8> {
    let hex = reply.split_once("=0x")?.1;
    let n = hex.bytes().take_while(u8::is_ascii_hexdigit).count();
    if n == 0 {
        return None;
    }
    u8::from_str_radix(&hex[..n], 16).ok()
}

/// Parse `A{driver}={int}` (C `sscanf(pStr, "A%d=%ld", ...)`); returns the
/// integer after `=`.
fn parse_int_after_eq(reply: &str) -> Option<i64> {
    let rest = reply.split_once('=')?.1;
    let s = rest.trim_start();
    let end = s
        .char_indices()
        .skip_while(|(i, c)| *i == 0 && (*c == '+' || *c == '-'))
        .find(|(_, c)| !c.is_ascii_digit())
        .map(|(i, _)| i)
        .unwrap_or(s.len());
    s.get(..end)?.parse().ok()
}

/// Parse a `DRT` line `A{driver}={type}` into a driver-module model
/// (C: type 1 → 8753, type 2 → 8751).
fn parse_drt_line(line: &str) -> Option<(i32, PmdModel)> {
    let (a, rest) = line.split_once('=')?;
    let driver: i32 = a.trim().trim_start_matches('A').trim().parse().ok()?;
    let model = match rest.trim().parse::<i32>().ok()? {
        1 => PmdModel::Pmd8753,
        2 => PmdModel::Pmd8751,
        _ => return None,
    };
    Some((driver, model))
}

/// Parse an `MPV` line `A{driver} M{motor}=...` into (driver, motor)
/// (C `sscanf(buff, "A%d M%d", ...)`).
fn parse_mpv_line(line: &str) -> Option<(i32, i32)> {
    let mut it = line.split_whitespace();
    let driver: i32 = it.next()?.trim_start_matches('A').parse().ok()?;
    let motor: i32 = it
        .next()?
        .trim_start_matches('M')
        .split('=')
        .next()?
        .parse()
        .ok()?;
    Some((driver, motor))
}

/// Firmware digit after [`VER_STR`] → model, plus the changeEOS flag
/// (firmware `1.5.3`/`1.5.4`). C: `'0'` → 8750, `'5'`/`'6'` → 8752.
fn model_from_ident(ident: &str) -> Option<(PmncModel, bool)> {
    let tail = &ident[ident.find(VER_STR)? + VER_STR.len()..];
    let mut chars = tail.chars();
    let model = match chars.next()? {
        '0' => PmncModel::Pmnc8750,
        '5' | '6' => PmncModel::Pmnc8752,
        _ => return None,
    };
    // C inspects the char two positions on (the sub-minor digit).
    let change_eos = matches!(tail.as_bytes().get(2), Some(b'3') | Some(b'4'));
    Some((model, change_eos))
}

fn eos_user() -> AsynUser {
    AsynUser::new(0).with_addr(0)
}

/// Shared controller endpoint: owns the octet handle and the cross-axis
/// communication state. The caller holds the `Arc<Mutex<..>>` lock.
pub struct PmncController {
    handle: SyncIOHandle,
    /// The same port, for the per-read input-EOS switch.
    port: PortHandle,
    /// Firmware 1.5.3/1.5.4: `STA` replies end in `\n` (C `changeEOS`).
    change_eos: bool,
    ident: String,
    model: PmncModel,
    axes: Vec<AxisDef>,
    comm_state: CommState,
}

impl PmncController {
    /// Connect and identify a PMNC 8750/8752 (C `motor_init`): flush, read the
    /// identity (`VER`, up to 3 tries), decode the model, then enumerate axes
    /// — an 8750 lists its 8753 channels indirectly via `MPV`, an 8752 reads
    /// each module's type via `DRT` (8753 → 3 channels, 8751 → 1). Performs
    /// blocking I/O.
    pub fn new(port: PortHandle, timeout: Duration) -> AsynResult<Self> {
        // C ignores every setInputEos/setOutputEos status
        // (drvPMNC87xx.cc:586, 595, 720, 727, 873-874).
        let _ = port.set_input_eos_blocking(eos_user(), PROMPT_EOS);
        let _ = port.set_output_eos_blocking(eos_user(), CMND_EOS);
        let mut ctrl = Self {
            handle: SyncIOHandle::from_handle(port.clone(), 0, timeout),
            port,
            change_eos: false,
            ident: String::new(),
            model: PmncModel::Pmnc8750,
            axes: Vec::new(),
            comm_state: CommState::Normal,
        };
        let mut ident = String::new();
        for _ in 0..3 {
            if let Ok(reply) = ctrl.command("VER")
                && !reply.trim().is_empty()
            {
                ident = reply;
                break;
            }
        }
        if ident.trim().is_empty() {
            return Err(pmnc_err("PMNC: no response to VER identity query".into()));
        }
        ctrl.ident = ident.trim().to_string();
        let (model, change_eos) = model_from_ident(&ctrl.ident)
            .ok_or_else(|| pmnc_err(format!("PMNC: unknown version = {}", ctrl.ident)))?;
        ctrl.model = model;
        ctrl.change_eos = change_eos;

        // Axis enumeration.
        let mut axes = Vec::new();
        match model {
            PmncModel::Pmnc8750 => {
                // Each MPV line is one 8753 channel.
                let blob = ctrl.command("MPV")?;
                for line in blob.lines() {
                    if let Some((driver_num, motor_num)) = parse_mpv_line(line.trim()) {
                        axes.push(AxisDef {
                            driver_type: PmdModel::Pmd8753,
                            driver_num,
                            motor_num,
                        });
                    }
                }
            }
            PmncModel::Pmnc8752 => {
                // Each DRT line is a module; 8753 contributes 3 channels,
                // 8751 one.
                let blob = ctrl.command("DRT")?;
                for line in blob.lines() {
                    let Some((driver_num, driver_type)) = parse_drt_line(line.trim()) else {
                        continue;
                    };
                    let channels = match driver_type {
                        PmdModel::Pmd8753 => 3,
                        PmdModel::Pmd8751 => 1,
                    };
                    for motor_num in 0..channels {
                        axes.push(AxisDef {
                            driver_type,
                            driver_num,
                            motor_num,
                        });
                    }
                }
            }
        }
        if axes.is_empty() {
            return Err(pmnc_err("PMNC: no axes discovered".into()));
        }
        ctrl.axes = axes;
        Ok(ctrl)
    }

    /// Identity string from `VER`.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Detected controller model.
    pub fn model(&self) -> PmncModel {
        self.model
    }

    /// Number of discovered axes.
    pub fn num_axes(&self) -> usize {
        self.axes.len()
    }

    fn axis_def(&self, index: usize) -> Option<AxisDef> {
        self.axes.get(index).copied()
    }

    /// Write one command and read its one prompt-framed reply (C
    /// `send_recv_mess`), returned trimmed.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.command_bytes(cmd.as_bytes())
    }

    /// [`command`](Self::command) for a record string's raw bytes.
    fn command_bytes(&self, cmd: &[u8]) -> AsynResult<String> {
        Self::trimmed(self.handle.write_read(0, cmd, READ_BUF)?)
    }

    /// `STA` for one driver module (C `set_status`), read to `\n` instead of
    /// the prompt on `changeEOS` firmware and the prompt EOS restored after.
    fn status(&self, driver: i32) -> AsynResult<String> {
        let cmd = format!("STA {driver}");
        if !self.change_eos {
            return self.command(&cmd);
        }
        let _ = self.port.set_input_eos_blocking(eos_user(), NL_EOS);
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF);
        let _ = self.port.set_input_eos_blocking(eos_user(), PROMPT_EOS);
        Self::trimmed(raw?)
    }

    fn trimmed(raw: Vec<u8>) -> AsynResult<String> {
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '>', ' '])
            .to_string())
    }
}

/// One PMNC axis sharing a controller. Implements [`AsynMotor`]. Carries the
/// per-axis software position state C keeps in `motor_info` +
/// `cntrl->last_position`.
pub struct PmncAxis {
    controller: Arc<Mutex<PmncController>>,
    def: AxisDef,
    /// Accumulated logical position (steps). For an 8753 this is
    /// software-tracked; for an 8751 it mirrors the controller's absolute
    /// reading.
    position: i64,
    /// Accumulated encoder position (same tracking as `position`).
    encoder_position: i64,
    /// Last raw `POS` reading, the delta reference for open-loop
    /// accumulation (C `cntrl->last_position[signal]`).
    last_reading: i64,
    /// Consecutive polls without a position change (C `no_motion_count`).
    no_motion_count: u32,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl PmncAxis {
    /// Construct the axis at discovery index `index` (0-based across all
    /// modules). No controller I/O beyond looking up the discovered
    /// addressing.
    pub fn new(controller: Arc<Mutex<PmncController>>, index: usize) -> AsynResult<Self> {
        let def = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.axis_def(index)
                .ok_or_else(|| pmnc_err(format!("PMNC: axis index {index} out of range")))?
        };
        Ok(Self {
            controller,
            def,
            position: 0,
            encoder_position: 0,
            last_reading: 0,
            no_motion_count: 0,
            last_status: MotorStatus::default(),
            strings: MoveStrings::default(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, PmncController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn is_8753(&self) -> bool {
        self.def.driver_type == PmdModel::Pmd8753
    }

    /// Velocity/acceleration preamble as separate prompt-framed messages,
    /// matching the record's SET_VEL_BASE / SET_VELOCITY / SET_ACCEL parts.
    /// C clamps velocity to `MAX_VELOCITY` and acceleration to
    /// `[MIN_ACCEL, MAX_ACCEL]`; an 8753 additionally programs the minimum
    /// profile velocity (`MPV`) after parking `VEL` at max to dodge an
    /// out-of-range error. The record sends no SET_ACCEL for a zero rate
    /// (motorRecord.cc:874, 940), so there is no `ACC` then.
    fn send_speed(
        &self,
        ctrl: &PmncController,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let (d, m) = (self.def.driver_num, self.def.motor_num);
        let vel = nint(velocity).unsigned_abs().min(MAX_VELOCITY as u32) as i64;
        if self.is_8753() {
            let mpv = nint(min_velocity)
                .unsigned_abs()
                .min((MAX_VELOCITY - 1) as u32) as i64;
            ctrl.command(&format!("VEL A{d} {m}={MAX_VELOCITY}"))?;
            ctrl.command(&format!("MPV A{d} {m}={mpv}"))?;
        }
        ctrl.command(&format!("VEL A{d} {m}={vel}"))?;
        if acceleration > 0.0 {
            let accel = i64::from(nint(acceleration)).clamp(MIN_ACCEL, MAX_ACCEL);
            ctrl.command(&format!("ACC A{d} {m}={accel}"))?;
        }
        Ok(())
    }

    /// PREM as its own message ahead of the move's commands
    /// (devPMNC87xx.cc:204-219).
    fn send_prem(ctrl: &PmncController, prem: &[u8]) -> AsynResult<()> {
        if !prem.is_empty() {
            ctrl.command_bytes(prem)?;
        }
        Ok(())
    }
}

impl AsynMotor for PmncAxis {
    /// C `set_status` ignores `send_recv_mess`'s result (drvPMNC87xx.cc).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let d = self.def.driver_num;
        let target = nint(position) as i64;
        let prem = self.strings.arm();
        let ctrl = self.lock();
        self.send_speed(&ctrl, min_velocity, velocity, acceleration)?;
        Self::send_prem(&ctrl, &prem)?;
        if self.is_8753() {
            // Open loop: no absolute command — move relative to the
            // software-tracked position.
            let rel = target - self.position;
            ctrl.command(&format!("REL A{d}={rel}"))?;
        } else {
            ctrl.command(&format!("ABS A{d}={target}"))?;
        }
        ctrl.command(&format!("GO A{d}"))?;
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let d = self.def.driver_num;
        let rel = nint(distance) as i64;
        let prem = self.strings.arm();
        let ctrl = self.lock();
        self.send_speed(&ctrl, min_velocity, velocity, acceleration)?;
        Self::send_prem(&ctrl, &prem)?;
        ctrl.command(&format!("REL A{d}={rel}"))?;
        ctrl.command(&format!("GO A{d}"))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let (d, m) = (self.def.driver_num, self.def.motor_num);
        let accel = i64::from(nint(acceleration)).clamp(MIN_ACCEL, MAX_ACCEL);
        let velval = nint(velocity).unsigned_abs().min(MAX_VELOCITY as u32) as i64;
        let forward = velocity >= 0.0;
        // C hangs POST off the first jog message, and the GO message's
        // VELOCITY node replaces it, so C never sends it; not copied.
        let prem = self.strings.arm();
        let ctrl = self.lock();
        ctrl.command(&format!("ACC A{d} {m}={accel}"))?;
        Self::send_prem(&ctrl, &prem)?;
        if self.is_8753() {
            // Native slew commands.
            if forward {
                ctrl.command(&format!("FOR A{d}={velval}"))?;
            } else {
                ctrl.command(&format!("REV A{d}={velval}"))?;
            }
        } else {
            // 8751 slew does not report motion via FOR/REV, so C jogs with a
            // large relative move (kept bug-for-bug).
            ctrl.command(&format!("VEL A{d} {m}={velval}"))?;
            let rel = if forward { 1_000_000 } else { -1_000_000 };
            ctrl.command(&format!("REL A{d}={rel}"))?;
        }
        ctrl.command(&format!("GO A{d}"))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C sends no command for HOME_FOR/HOME_REV (the FIN/RIN paths are
        // commented out) — homing is a silent no-op (module Deviations) but
        // for the record's speed parts and its PREM. C hangs POST off a
        // message it never sends; not copied, POST goes at the next poll
        // that sees the axis done.
        let prem = self.strings.arm();
        let ctrl = self.lock();
        self.send_speed(&ctrl, min_velocity, velocity, acceleration)?;
        Self::send_prem(&ctrl, &prem)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS: HAL (smooth stop).
        let d = self.def.driver_num;
        let ctrl = self.lock();
        ctrl.command(&format!("HAL A{d}"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let (d, m) = (self.def.driver_num, self.def.motor_num);
        let value = nint(position) as i64;
        let ctrl = self.lock();
        if self.is_8753() {
            // Open loop: the controller position cannot be set, so C selects
            // the channel and redefines the software-tracked position.
            ctrl.command(&format!("CHL A{d}={m}"))?;
            drop(ctrl);
            self.position = value;
            self.encoder_position = value;
            self.last_reading = 0;
        } else {
            ctrl.command(&format!("POS {d}={value}"))?;
        }
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE / DISABL_TORQUE: 8751 toggles the servo (SER/NOS),
        // 8753 toggles motor power (MON/MOF).
        let d = self.def.driver_num;
        let ctrl = self.lock();
        let cmd = match (self.is_8753(), enable) {
            (false, true) => format!("SER A{d}"),
            (false, false) => format!("NOS A{d}"),
            (true, true) => format!("MON A{d}"),
            (true, false) => format!("MOF A{d}"),
        };
        ctrl.command(&cmd)?;
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C sends no command for soft limits (IDLE_STATE) — quiet no-op.
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C sends no command for PID gains (IDLE_STATE) — quiet no-op.
        Ok(())
    }

    /// `PRIMITIVE`: INIT as its own message.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().command_bytes(init)?;
        Ok(())
    }

    fn set_move_strings(
        &mut self,
        _user: &AsynUser,
        prem: &[u8],
        post: Option<PostMessage>,
    ) -> AsynResult<()> {
        self.strings.set(prem, post);
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Port of C `set_status`: channel-select probe (8753), STA status
        // byte, DIAG aux byte (8751), then POS if this motor is the selected
        // channel — all under the controller lock, applying the
        // NORMAL/RETRY/COMM_ERR machine (C `CHECKRTN`) on any failed reply.
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let (d, m) = (self.def.driver_num, self.def.motor_num);

        macro_rules! checked {
            ($reply:expr) => {
                match $reply {
                    Ok(s) if !s.is_empty() => {
                        ctrl.comm_state = CommState::Normal;
                        s
                    }
                    _ => {
                        if ctrl.comm_state == CommState::Normal {
                            ctrl.comm_state = CommState::Retry;
                            return Ok(self.last_status.clone());
                        }
                        ctrl.comm_state = CommState::CommErr;
                        self.last_status.comms_error = true;
                        self.last_status.problem = true;
                        return Ok(self.last_status.clone());
                    }
                }
            };
        }

        // 1. Which motor is selected on this driver? (8751 has only motor 0.)
        let select_motor = if self.is_8753() {
            let reply = checked!(ctrl.command(&format!("CHL {d}")));
            atoi(&reply)
        } else {
            0
        };

        // 2. Status byte.
        let sta = checked!(ctrl.status(d));
        let mstat = parse_status_byte(&sta).unwrap_or(0);

        // 3. Aux status (8751 only).
        let auxstat = if self.is_8753() {
            0
        } else {
            let diag = checked!(ctrl.command(&format!("DIAG {d}")));
            parse_status_byte(&diag).unwrap_or(0)
        };

        // 4. Position, only if this motor is currently selected.
        let mut reading = self.last_reading;
        if select_motor == m {
            let pos = checked!(ctrl.command(&format!("POS {d}")));
            if let Some(v) = parse_int_after_eq(&pos) {
                reading = v;
            }
        } else if self.is_8753() {
            // Not selected: the open-loop channel reports nothing; C zeroes
            // the reference so no phantom delta accumulates.
            reading = 0;
            self.last_reading = 0;
        }

        // --- status bit decode ---
        let mut done = self.last_status.done;
        let powered;
        let mut high_limit = false;
        let mut low_limit = false;
        let mut home = false;
        if self.is_8753() {
            powered = mstat & S8753_POWER_ON != 0;
            if mstat & S8753_IN_MOTION == 0 {
                done = true;
            } else if select_motor == m {
                done = false;
            }
        } else {
            done = mstat & S8751_MOVE_DONE != 0;
            powered = auxstat & AUX8751_SERVO_ON != 0;
            if mstat & S8751_POWER_ON != 0 && auxstat & AUX8751_SERVO_ON != 0 {
                high_limit = mstat & S8751_FOR_LIMIT != 0;
                low_limit = mstat & S8751_REV_LIMIT != 0;
                home = mstat & S8751_HOMING != 0;
            }
        }

        // --- position accumulation (C) ---
        // C reports plus-direction when the position did not change.
        let direction;
        if reading == self.last_reading {
            self.no_motion_count += 1;
            direction = true;
        } else {
            let delta = reading - self.last_reading;
            if self.is_8753() {
                self.position += delta;
                self.encoder_position += delta;
                direction = reading >= 0;
            } else {
                self.position = reading;
                self.encoder_position = reading;
                direction = delta >= 0;
            }
            self.last_reading = reading;
            self.no_motion_count = 0;
        };

        // Move done because of a limit switch?
        let ls_active = (direction && high_limit) || (!direction && low_limit);

        // Post-move cleanup for the open-loop driver: reset the reference and
        // clear the controller's per-move position on the selected channel.
        if (done || ls_active) && self.is_8753() {
            self.last_reading = 0;
            if select_motor == m {
                ctrl.command(&format!("CHL A{d}={m}"))?;
            }
        }
        // drvPMNC87xx.cc:511-537; C ignores the exchange's status too.
        if (done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.command_bytes(&post);
        }
        drop(ctrl);

        let vbas_supported = self.is_8753();
        self.last_status = MotorStatus {
            position: self.position as f64,
            encoder_position: self.encoder_position as f64,
            velocity: 0.0, // C: "Parse motor velocity? NEEDS WORK"
            done,
            moving: !done,
            direction,
            high_limit,
            low_limit,
            home,
            powered,
            problem: false, // C clears RA_PROBLEM unconditionally
            comms_error: false,
            gain_support: true, // C GAIN_SUPPORT = 1
            has_encoder: true,  // C EA_PRESENT = 1
            vbas_supported,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicU8, Ordering};

    /// An 8752 with one 8751 module on a [`LineRig`] whose `STA` byte is
    /// `sta`, with the construction traffic taken.
    fn strings_rig(name: &str) -> (LineRig, PmncAxis, Arc<AtomicU8>) {
        let sta = Arc::new(AtomicU8::new(S8751_MOVE_DONE | S8751_POWER_ON));
        let st = sta.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "VER" => "Version 1.6.0>".into(),
            "DRT" => "A0=2>".into(),
            "STA 0" => format!("A0=0x{:02x}>", st.load(Ordering::SeqCst)),
            "DIAG 0" => "A0=0x04>".into(),
            "POS 0" => "A0=0>".into(),
            _ => ">".into(),
        });
        let ctrl = PmncController::new(io.port().unwrap().clone(), Duration::from_secs(1));
        let axis = PmncAxis::new(Arc::new(Mutex::new(ctrl.unwrap())), 0).unwrap();
        line.sent();
        (line, axis, sta)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PmncAxis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| !["STA 0", "DIAG 0", "POS 0"].contains(&l.as_str()))
            .collect()
    }

    /// devPMNC87xx.cc:200-201 and 321-326: INIT is the whole message.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("pmncInit");
        axis.primitive(&AsynUser::default(), b"SER A0").unwrap();
        assert_eq!(line.sent(), ["SER A0"]);
    }

    #[test]
    fn a_zero_rate_move_has_no_acc() {
        let (line, mut axis, _) = strings_rig("pmncZeroAcc");
        let user = AsynUser::default();
        axis.move_absolute(&user, 5.0, 0.0, 100.0, 0.0).unwrap();
        axis.move_relative(&user, 5.0, 0.0, 100.0, 0.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "VEL A0 0=100",
                "ABS A0=5",
                "GO A0",
                "VEL A0 0=100",
                "REL A0=5",
                "GO A0"
            ]
        );
    }

    #[test]
    fn record_steps_go_out_with_c_nint() {
        let (line, mut axis, _) = strings_rig("pmncSteps");
        let user = AsynUser::default();
        axis.move_absolute(&user, 4000.5, 0.0, 4000.4, 200.0)
            .unwrap();
        axis.move_relative(&user, -2.5, 0.0, 100.0, 200.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "VEL A0 0=2000",
                "ACC A0 0=200",
                "ABS A0=4001",
                "GO A0",
                "VEL A0 0=100",
                "ACC A0 0=200",
                "REL A0=-3",
                "GO A0"
            ]
        );
    }

    /// devPMNC87xx.cc:204-219: PREM is its own message ahead of each move's
    /// command, for that move only; a home sends only the record's speed
    /// parts ahead of it.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("pmncPrem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"SER A0", None).unwrap();
        axis.move_absolute(&user, 5.0, 0.0, 100.0, 200.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "VEL A0 0=100",
                "ACC A0 0=200",
                "SER A0",
                "ABS A0=5",
                "GO A0"
            ]
        );
        axis.move_relative(&user, 5.0, 0.0, 100.0, 200.0).unwrap();
        assert_eq!(
            line.sent(),
            ["VEL A0 0=100", "ACC A0 0=200", "REL A0=5", "GO A0"]
        );
        axis.set_move_strings(&user, b"SER A0", None).unwrap();
        axis.home(&user, 0.0, 100.0, 200.0, true).unwrap();
        assert_eq!(line.sent(), ["VEL A0 0=100", "ACC A0 0=200", "SER A0"]);
        axis.home(&user, 0.0, 100.0, 0.0, true).unwrap();
        assert_eq!(line.sent(), ["VEL A0 0=100"]);
        axis.set_move_strings(&user, b"SER A0", None).unwrap();
        axis.move_velocity(&user, 0.0, -100.0, 200.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "ACC A0 0=200",
                "SER A0",
                "VEL A0 0=100",
                "REL A0=-1000000",
                "GO A0"
            ]
        );
    }

    /// drvPMNC87xx.cc:511-537: POST once the move is done, not while it
    /// runs; a jog's too.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, sta) = strings_rig("pmncPost");
        let user = AsynUser::default();
        for jog in [false, true] {
            axis.set_move_strings(&user, b"", Some(post(b"NOS A0")))
                .unwrap();
            if jog {
                axis.move_velocity(&user, 0.0, 100.0, 200.0).unwrap();
            } else {
                axis.move_relative(&user, 5.0, 0.0, 100.0, 200.0).unwrap();
            }
            line.sent();
            sta.store(S8751_POWER_ON, Ordering::SeqCst);
            assert!(poll_extra(&line, &mut axis).is_empty());
            sta.store(S8751_MOVE_DONE | S8751_POWER_ON, Ordering::SeqCst);
            assert_eq!(poll_extra(&line, &mut axis), ["NOS A0"]);
            assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
        }
    }

    /// drvPMNC87xx.cc:495-496: a limit in the move's direction ends it too.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let (line, mut axis, sta) = strings_rig("pmncPostLs");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"NOS A0")))
            .unwrap();
        axis.move_relative(&user, 5.0, 0.0, 100.0, 200.0).unwrap();
        line.sent();
        sta.store(S8751_POWER_ON | S8751_FOR_LIMIT, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["NOS A0"]);
    }

    #[test]
    fn model_and_change_eos_from_ident() {
        assert_eq!(
            model_from_ident("New Focus Version 1.0.2"),
            Some((PmncModel::Pmnc8750, false))
        );
        assert_eq!(
            model_from_ident("Version 1.5.4"),
            Some((PmncModel::Pmnc8752, true))
        );
        assert_eq!(
            model_from_ident("Version 1.5.3"),
            Some((PmncModel::Pmnc8752, true))
        );
        assert_eq!(
            model_from_ident("Version 1.6.0"),
            Some((PmncModel::Pmnc8752, false))
        );
        assert_eq!(model_from_ident("Version 1.9.0"), None);
        assert_eq!(model_from_ident("no version here"), None);
    }

    #[test]
    fn status_byte_parses_hex_after_prefix() {
        assert_eq!(parse_status_byte("A2=0x1f"), Some(0x1f));
        assert_eq!(parse_status_byte("A0=0x05"), Some(0x05));
        assert_eq!(parse_status_byte("A3=0xFF"), Some(0xff));
        assert_eq!(parse_status_byte("garbage"), None);
    }

    #[test]
    fn int_after_eq_parses_signed() {
        assert_eq!(parse_int_after_eq("A2=12345"), Some(12345));
        assert_eq!(parse_int_after_eq("A0=-678"), Some(-678));
        assert_eq!(parse_int_after_eq("A1=0"), Some(0));
        assert_eq!(parse_int_after_eq("nope"), None);
    }

    #[test]
    fn drt_line_maps_type_to_model() {
        assert_eq!(parse_drt_line("A0=1"), Some((0, PmdModel::Pmd8753)));
        assert_eq!(parse_drt_line("A3=2"), Some((3, PmdModel::Pmd8751)));
        assert_eq!(parse_drt_line("A1=9"), None);
    }

    #[test]
    fn mpv_line_reads_driver_and_motor() {
        assert_eq!(parse_mpv_line("A0 M0=100"), Some((0, 0)));
        assert_eq!(parse_mpv_line("A2 M1=250"), Some((2, 1)));
        assert_eq!(parse_mpv_line("junk"), None);
    }
    /// A 1.5.4 controller with one 8753 module, on a port with no EOS support
    /// of its own, so every EOS set is refused.
    struct Pmnc {
        base: epics_rs::asyn::port::PortDriverBase,
        reply: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for Pmnc {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.reply = match data {
                b"VER" => b"Version 1.5.4>".to_vec(),
                b"DRT" => b"A0=1>".to_vec(),
                b"STA 0" => b"A0=0x05\n".to_vec(),
                _ => b">".to_vec(),
            };
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    #[test]
    fn a_port_refusing_every_eos_set_still_configures_and_reads_status() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let pmnc = Pmnc {
            base: PortDriverBase::new("pmncEosRefused", 1, PortFlags::default()),
            reply: Vec::new(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("pmncEosRefused").unwrap(),
            pmnc,
            RuntimeConfig::default(),
        )
        .unwrap();
        let port = rt.port_handle().clone();
        assert!(port.set_input_eos_blocking(eos_user(), NL_EOS).is_err());
        let ctrl = PmncController::new(port, Duration::from_secs(1)).unwrap();
        assert!(ctrl.change_eos);
        assert_eq!(ctrl.num_axes(), 3);
        assert_eq!(ctrl.status(0).unwrap(), "A0=0x05");
    }
}
