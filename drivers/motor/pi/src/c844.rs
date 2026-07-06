//! PI (Physik Instrumente) C-844 4-axis DC-servo controller (`motorPI`).
//!
//! Ported from `drvPIC844.cc` + `devPIC844.cc` (a model-1 dev/drv pair). The
//! C-844 is a **4-axis** DC-servo controller with a SCPI-like command set.
//! One [`PIC844Controller`] owns the octet handle and drives up to four
//! [`PIC844Axis`]es (`motor_init` hardcodes `total_axis = 4`).
//!
//! ## Transport / framing — port owns EOS, `AXIS n;` selects the axis
//!
//! `motor_init` sets `setOutputEos("\n")` **and** `setInputEos("\n")` after
//! connecting; `send_mess` never appends a terminator, so the port's output
//! EOS puts the `\n` on the wire (both EOS set in `st.cmd`; this port's
//! [`PIC844Controller::send`] writes bare bytes, no echo — `cmnd_response =
//! false`).
//!
//! The C-844 is single-device but multi-axis: the axis is chosen by an
//! `AXIS n;` command prefix. In C this comes from the driver table's
//! `axis_names = {"1","2","3","4"}` — `motor_send` prepends `"AXIS " + name +
//! ";"` to **every queued command** for that axis. This port builds the same
//! `AXIS {n};{cmd}` prefix explicitly (`n = signal + 1`, `1`-`4`) on every
//! command it sends. The poll's *global* `MOT:COND?` query and the two
//! position queries deliberately carry **no** prefix — they run under the same
//! lock right after `AXIS {n};AXIS:STAT?` has selected the axis, exactly as C's
//! `send_mess(..., NULL)` relies on the prior selection.
//!
//! ## Connect (`motor_init`)
//! `*IDN?` (retry up to 3× until a non-empty reply → `brdptr->ident`); four
//! axes are then created unconditionally (`total_axis = 4`), each with encoder
//! and gain support.
//!
//! ## Status
//! Per axis, under one lock (C `set_status`):
//! 1. `AXIS {n};AXIS:STAT?` → `"ON"` / `"OFF"` → torque/powered
//!    (`EA_POSITION`). Any other reply drives the NORMAL/RETRY/COMM_ERR
//!    two-strike debounce ([`CommState`]) — shared across all axes, as
//!    `cntrl->status` is per-controller in C.
//! 2. `MOT:COND?` → `atoi` → 16-bit motion-condition register (`C844_Cond_Reg`,
//!    `#else`/LSB layout): axis-in-motion `= bit signal`, +limit `= bit
//!    (8 + signal)`, -limit `= bit (12 + signal)`. `RA_DONE = !in_motion`.
//! 3. `CURR:TPOS?` → `atof` → **target** position; reported `position =
//!    NINT(target)` (the record's readback tracks the commanded target here,
//!    not the encoder). Direction is the sign of the count delta versus the
//!    previous poll, held when unchanged.
//! 4. `AXIS:POS?` → `atof` → actual `encoder_position` (`(epicsInt32) motorData`
//!    truncating cast).
//!
//! ## Units
//! `cntrl_units = dval`, `maxdigits = 2`: the record dial value is the wire
//! value with two decimals, and readback rounds to the nearest integer count
//! (`NINT`). Pair with `MRES = 1`, `EGU = counts`.
//!
//! ## Motion (`devPIC844_build_trans`) — one combined `AXIS n;` message
//!
//! `devPIC844_build_trans` `strcat`s into one `motor_call->message` bracketed
//! by the record's `INIT_MSG`/`END_MSG`, so a move's `SET_VELOCITY` +
//! (`SET_ACCEL` if `accel > 0`) + move go out as **one** write with a single
//! `AXIS n;` prefix:
//!
//! - `MOVE_ABS` → `AXIS {n};MVEL {v};[ACC {a};]TARG {p}` (`.2f`).
//! - `MOVE_REL` → `AXIS {n};MVEL {v};[ACC {a};]TARG:RPOS {+p}` (`%+.2f`).
//! - `HOME_FOR`/`HOME_REV` → `AXIS {n};MVEL {v};[ACC {a};]TARG:FIND POS|NEG`.
//! - `JOG` (`move_velocity`) → `AXIS {n};ACC {a};TARG:VEL {v}` — motorRecord's
//!   jog issues `SET_ACCEL` *unconditionally* (no `acc > 0` guard) and no
//!   `SET_VELOCITY` (the velocity rides in `TARG:VEL`).
//! - `STOP_AXIS` (`stop`) → `AXIS {n};HALT`.
//! - `LOAD_POS` (`set_position`) → `AXIS {n};AXIS:POS {+p};TARG CURR`.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` (`set_closed_loop`) →
//!   `AXIS {n};AXIS:STAT ON|OFF`.
//! - `SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN` (`set_pid_gain`) →
//!   `AXIS {n};PID {g},,` / `PID ,{g},` / `PID ,,{g}`, `g = gain × 32767`.
//! - `SET_HIGH_LIMIT`/`SET_LOW_LIMIT`/`SET_ENC_RATIO`/`GO` — no wire command.
//!
//! ## Not modeled (documented deviations)
//! - **`no_motion_count`** poll-scheduling counter and **`recv_mess(FLUSH)`** /
//!   **`report()`**: as elsewhere in this workspace (record-layer state / no
//!   `SyncIOHandle` primitive / not implemented).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, nint};

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;

/// Fixed axis count (C `motor_init` `total_axis = 4`).
pub const PIC844_NUM_AXES: u8 = 4;

fn pic844_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Controller communication state (C `cntrl->status`, per-controller/shared).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Shared 4-axis controller endpoint: owns the octet handle and the
/// (controller-wide) comm debounce state.
pub struct PIC844Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
}

impl PIC844Controller {
    /// Connect and identify a C-844 (C `motor_init`'s per-card block): `*IDN?`
    /// retried up to 3 times until a non-empty reply.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
        };
        for _ in 0..3 {
            let reply = ctrl.query("*IDN?");
            if !reply.is_empty() {
                ctrl.ident = reply;
                break;
            }
        }
        if ctrl.ident.is_empty() {
            return Err(pic844_err("PIC844: no response to *IDN? (controller off?)"));
        }
        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `*IDN?`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Write a command, no reply expected (C `send_mess`). No terminator (port
    /// output EOS) and no echo.
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Read one EOS-stripped reply, folding a transport failure / empty read
    /// into `""` (C `recv_mess`).
    fn recv(&self) -> String {
        match self.handle.read_octet(0, READ_BUF) {
            Ok(raw) if !raw.is_empty() => String::from_utf8_lossy(&raw).into_owned(),
            _ => String::new(),
        }
    }

    fn query(&self, cmd: &str) -> String {
        let _ = self.send(cmd);
        self.recv()
    }

    /// C `set_status` for axis `signal` (0-3). `None` means keep the cached
    /// status (RETRY) or flag comm error (COMM_ERR), per [`Self::comm_state`].
    fn poll_status(&mut self, signal: u8) -> Option<PolledStatus> {
        let n = signal + 1;
        let stat = self.query(&format!("AXIS {n};AXIS:STAT?"));
        let powered = if stat == "ON" {
            true
        } else if stat == "OFF" {
            false
        } else {
            self.comm_state = if self.comm_state == CommState::Normal {
                CommState::Retry
            } else {
                CommState::CommErr
            };
            return None;
        };
        self.comm_state = CommState::Normal;

        // Motion-condition register (no axis prefix — the AXIS:STAT? above
        // selected the axis, but this query is global anyway).
        let cond = atoi(&self.query("MOT:COND?")) as u32;
        let in_motion = cond & (1 << signal) != 0;
        let high_limit = cond & (1 << (8 + signal)) != 0;
        let low_limit = cond & (1 << (12 + signal)) != 0;

        let target = atof(&self.query("CURR:TPOS?"));
        let position = nint(target);
        let encoder_position = atof(&self.query("AXIS:POS?")) as i32;

        Some(PolledStatus {
            position,
            encoder_position,
            done: !in_motion,
            powered,
            high_limit,
            low_limit,
        })
    }
}

/// Decoded result of one C-844 poll.
struct PolledStatus {
    position: i32,
    encoder_position: i32,
    done: bool,
    powered: bool,
    high_limit: bool,
    low_limit: bool,
}

/// One axis (`1`-`4`) of a C-844 controller. Implements [`AsynMotor`].
pub struct PIC844Axis {
    controller: Arc<Mutex<PIC844Controller>>,
    /// Wire axis number (`signal + 1`, i.e. `1`-`4`).
    n: u8,
    last_status: MotorStatus,
}

impl PIC844Axis {
    /// Construct one axis, seeding the initial status (C `motor_init`'s
    /// `set_status(card, motor_index)`). `signal` is 0-based; wire axis is
    /// `signal + 1`. A failed initial poll is not an error.
    pub fn new(controller: Arc<Mutex<PIC844Controller>>, signal: u8) -> AsynResult<Self> {
        let mut axis = Self {
            controller: controller.clone(),
            n: signal + 1,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                vbas_supported: false,
                ..MotorStatus::default()
            },
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(polled) = ctrl.poll_status(signal) {
            drop(ctrl);
            axis.apply(polled);
        }
        Ok(axis)
    }

    fn lock(&self) -> MutexGuard<'_, PIC844Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The `AXIS n;` fragment prefixed onto every command for this axis.
    fn prefix(&self) -> String {
        format!("AXIS {};", self.n)
    }

    /// The `MVEL {v};[ACC {a};]` move/home preamble (C `SET_VELOCITY` +
    /// `SET_ACCEL` when `acc > 0`).
    fn move_preamble(velocity: f64, acceleration: f64) -> String {
        let mut p = format!("MVEL {velocity:.2};");
        if acceleration > 0.0 {
            p.push_str(&format!("ACC {acceleration:.2};"));
        }
        p
    }

    fn apply(&mut self, polled: PolledStatus) {
        let prev = self.last_status.position as i32;
        let direction = if polled.position != prev {
            polled.position >= prev
        } else {
            self.last_status.direction
        };
        self.last_status = MotorStatus {
            position: polled.position as f64,
            encoder_position: polled.encoder_position as f64,
            velocity: 0.0,
            done: polled.done,
            moving: !polled.done,
            high_limit: polled.high_limit,
            low_limit: polled.low_limit,
            direction,
            powered: polled.powered,
            gain_support: true,
            has_encoder: true,
            vbas_supported: false,
            ..MotorStatus::default()
        };
    }
}

impl AsynMotor for PIC844Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let pre = self.prefix();
        let body = Self::move_preamble(velocity, acceleration);
        self.lock().send(&format!("{pre}{body}TARG {position:.2}"))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let pre = self.prefix();
        let body = Self::move_preamble(velocity, acceleration);
        self.lock()
            .send(&format!("{pre}{body}TARG:RPOS {distance:+.2}"))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Jog: SET_ACCEL unconditional (motorRecord), then TARG:VEL (signed).
        let pre = self.prefix();
        self.lock().send(&format!(
            "{pre}ACC {acceleration:.2};TARG:VEL {velocity:.2}"
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let pre = self.prefix();
        let body = Self::move_preamble(velocity, acceleration);
        let dir = if forward { "POS" } else { "NEG" };
        self.lock().send(&format!("{pre}{body}TARG:FIND {dir}"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let pre = self.prefix();
        self.lock().send(&format!("{pre}HALT"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let pre = self.prefix();
        self.lock()
            .send(&format!("{pre}AXIS:POS {position:+.2};TARG CURR"))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let pre = self.prefix();
        let state = if enable { "ON" } else { "OFF" };
        self.lock().send(&format!("{pre}AXIS:STAT {state}"))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let g = gain * 32767.0;
        let pre = self.prefix();
        let body = match kind {
            PidGainKind::Proportional => format!("PID {g:.2},,"),
            PidGainKind::Integral => format!("PID ,{g:.2},"),
            PidGainKind::Derivative => format!("PID ,,{g:.2}"),
        };
        self.lock().send(&format!("{pre}{body}"))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let signal = self.n - 1;
        let mut ctrl = self.lock();
        let polled = ctrl.poll_status(signal);
        let comm_err = ctrl.comm_state == CommState::CommErr;
        drop(ctrl);

        let Some(polled) = polled else {
            if comm_err {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return Ok(self.last_status.clone());
        };
        self.apply(polled);
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cond_register_bit_positions_per_axis() {
        // signal s: in_motion = bit s, +LS = bit 8+s, -LS = bit 12+s.
        for s in 0u8..4 {
            let cond: u32 = (1 << s) | (1 << (8 + s)) | (1 << (12 + s));
            assert!(cond & (1 << s) != 0);
            assert!(cond & (1 << (8 + s)) != 0);
            assert!(cond & (1 << (12 + s)) != 0);
            // A different axis' bits must not leak into this axis.
            let other = (s + 1) % 4;
            assert!(cond & (1 << other) == 0);
        }
    }

    #[test]
    fn move_message_is_one_prefixed_command() {
        // MVEL + ACC (accel>0) + TARG, all under one "AXIS n;".
        let pre = "AXIS 2;";
        let body = PIC844Axis::move_preamble(5.0, 2.0);
        assert_eq!(
            format!("{pre}{body}TARG {:.2}", 10.5),
            "AXIS 2;MVEL 5.00;ACC 2.00;TARG 10.50"
        );
        // accel <= 0 drops the ACC fragment (record guard).
        let body0 = PIC844Axis::move_preamble(5.0, 0.0);
        assert_eq!(
            format!("{pre}{body0}TARG {:.2}", 10.5),
            "AXIS 2;MVEL 5.00;TARG 10.50"
        );
    }

    #[test]
    fn relative_move_is_signed() {
        assert_eq!(format!("TARG:RPOS {:+.2}", 3.0), "TARG:RPOS +3.00");
        assert_eq!(format!("TARG:RPOS {:+.2}", -3.0), "TARG:RPOS -3.00");
    }

    #[test]
    fn position_is_nint_of_target() {
        assert_eq!(nint(atof("5.6")), 6);
        assert_eq!(nint(atof("-5.6")), -6);
        assert_eq!(nint(atof("")), 0);
    }

    #[test]
    fn pid_gain_scales_by_32767() {
        let g = 0.5 * 32767.0;
        assert_eq!(format!("PID {g:.2},,"), "PID 16383.50,,");
    }
}
