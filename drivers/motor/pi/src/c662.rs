//! PI (Physik Instrumente) C-662 / E-662 piezo controller (`motorPI`).
//!
//! Ported from `drvPIC662.cc` + `devPIC662.cc` (a model-1 dev/drv pair). The
//! E-662 is a **single-axis** piezo LVPZT position-servo controller with a
//! SCPI-like command set (`*IDN?`, `*ESR?`, `POS?`, `POS`, `POS:REL`,
//! `DEV:CONT REM`). One [`PIC662Controller`] owns the octet handle and drives
//! one [`PIC662Axis`] (C `MAX_AXES` is 1; `PIC662_NUM_CARDS` allows up to 8
//! separate `PIC662Config` calls).
//!
//! ## Transport / framing — the port owns EOS, no address, no echo
//!
//! `motor_init` sets `setOutputEos("\n")` **and** `setInputEos("\n")` after
//! connecting, and `send_mess`'s `local_buff` never appends a terminator
//! (`strcat(local_buff, com)` only) — so the port's output EOS puts the `\n` on
//! the wire. This port has no `SyncIOHandle` EOS hook, so both are set in
//! `st.cmd` (both `"\n"`) and [`PIC662Controller::send`] writes **bare** bytes.
//! There is no protocol address (SCPI, single device) and, unlike the C-630,
//! the device does **not** echo (`cmnd_response = false`): a command write is
//! never followed by a read.
//!
//! ## Connect (`motor_init`)
//! `*IDN?` (retry up to 3× until a non-empty reply → `brdptr->ident`), then a
//! fixed resolution (`res_decpts = 3`, `drive_resolution = 1/10³ = 0.001`),
//! then `DEV:CONT REM` to switch the controller to remote mode (it powers up in
//! Local mode).
//!
//! ## Status (`*ESR?`) — Standard Event Status Register
//! `mstat.All = atoi(&buff[0])`. Bits (header `#else`/non-`MSB_First` layout):
//! `OpComplete` = bit 0, `DevError` = bit 3, `ExeError` = bit 4. The C ORs
//! these three into `eventflgs`, and
//! `RA_DONE = (eventflgs || cntrl->stop_status)`. There is no hardware
//! done/limit feedback: the E-662 reports "done" via `OpComplete` (or a
//! device/execution error), or when [`stop`](AsynMotor::stop) has requested a
//! poll-side stop. `stop_status` is consumed (cleared) only on a successful
//! `*ESR?`.
//!
//! A failed `*ESR?` runs the same NORMAL/RETRY/COMM_ERR two-strike debounce as
//! the C-862 ([`CommState`]): the first empty reply is silently retried (cached
//! status kept), the second consecutive one sets `comms_error`/`problem`.
//!
//! ## Position (`POS?`)
//! `motorData = NINT(atof(buff) / drive_resolution)` — the reply is in physical
//! units (µm), and dividing by `0.001` yields integer counts (so `POS?` "5.012"
//! → 5012 counts). Direction is the sign of the count delta versus the previous
//! poll, held when unchanged (C compares against `motor_info->position`). No
//! limit switches (`plusLS = minusLS = false` always).
//!
//! ## Units
//! `MRES = 1`, `EGU = counts`: the record works in raw counts, and the driver
//! scales to/from physical units by `drive_resolution` (0.001) at the wire.
//!
//! ## Motion (`devPIC662_build_trans`) — only absolute/relative moves send
//! The E-662 has no velocity/accel/jog/home/load-position wire commands; the
//! device support marks all of those `send = false`. Only:
//!
//! - `MOVE_ABS` (`move_absolute`) → `DEV:CONT REM\nPOS {µm:.3}`
//!   (`µm = counts × drive_resolution`; the `DEV:CONT REM\n` prefix re-asserts
//!   remote mode ahead of every move, matching C).
//! - `MOVE_REL` (`move_relative`) → `DEV:CONT REM\nPOS:REL {µm:.3}`.
//! - `STOP_AXIS` (`stop`) → **no wire command**: there is no HALT; the C sets
//!   `cntrl->stop_status = true` to force the next poll's `RA_DONE`. Mirrored
//!   with a controller flag the poll consumes.
//! - `HOME_FOR`/`HOME_REV` (`home`), `JOG` (`move_velocity`), `LOAD_POS`
//!   (`set_position`), `SET_VELOCITY`/`SET_ACCEL`, `ENABLE_TORQUE`/
//!   `DISABL_TORQUE` (`set_closed_loop`), `SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN`
//!   (`set_pid_gain`), `SET_HIGH_LIMIT`/`SET_LOW_LIMIT`, `GO`, `SET_ENC_RATIO`
//!   — all `send = false` in C; mirrored as no-ops. (In particular `home`,
//!   `move_velocity`, and `set_position` deliberately do nothing rather than
//!   fall through to the `AsynMotor` default `move_velocity`, which would jog to
//!   ±1e9.)
//!
//! ## Not modeled (documented deviations)
//! - **`no_motion_count` "no done indicator" fallback**: C also declares done
//!   after the position is stable for `> 1` poll *while a move node is active* —
//!   record-layer state this interface does not carry. `OpComplete`/`ExeError`
//!   (the primary done signal) is honored; the stability fallback is omitted,
//!   matching the C-862 port's treatment of `no_motion_count`.
//! - **`recv_mess(..., FLUSH)`** / **`report()`**: no `SyncIOHandle` flush
//!   primitive / not implemented, as elsewhere in this workspace.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, nint};

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;

/// Fixed drive resolution (C `1.0 / pow(10, res_decpts)`, `res_decpts = 3`).
const DRIVE_RESOLUTION: f64 = 0.001;

// Command decimal precision for `POS`/`POS:REL` is C `maxdigits = res_decpts`
// = 3; embedded as the literal `.3` in the two move format strings below.

fn pic662_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Controller communication state (C `cntrl->status`): one failed `*ESR?` read
/// is retried silently; a second consecutive one is a hard comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Shared single-axis controller endpoint: owns the octet handle, the comm
/// debounce state, and the poll-side stop flag.
pub struct PIC662Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
    /// Set by [`PIC662Axis::stop`]; the next successful poll forces `done` and
    /// clears it (C `cntrl->stop_status`).
    stop_status: bool,
}

impl PIC662Controller {
    /// Connect and identify an E-662 (C `motor_init`'s per-card block): `*IDN?`
    /// retried up to 3 times until a non-empty reply, then `DEV:CONT REM` to
    /// enter remote mode.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
            stop_status: false,
        };

        for _ in 0..3 {
            let reply = ctrl.query("*IDN?");
            if !reply.is_empty() {
                ctrl.ident = reply;
                break;
            }
        }
        if ctrl.ident.is_empty() {
            return Err(pic662_err("PIC662: no response to *IDN? (controller off?)"));
        }

        ctrl.send("DEV:CONT REM")?;
        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `*IDN?`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Write a command, no reply expected (C `send_mess`). No terminator — the
    /// port's output EOS puts the `\n` on the wire. The device does not echo.
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Read one EOS-stripped reply, folding a transport failure or empty read
    /// into `""` (C `recv_mess`'s `if ((status != asynSuccess) || (nread <= 0))
    /// com[0] = '\0'`).
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

    /// C `set_status`: `*ESR?` (comm-debounced) then `POS?`. `None` means the
    /// caller should keep its cached status (RETRY) or flag comm error
    /// (COMM_ERR), decided by [`Self::comm_state`] after the call.
    fn poll_status(&mut self) -> Option<PolledStatus> {
        let esr = self.query("*ESR?");
        if esr.is_empty() {
            self.comm_state = if self.comm_state == CommState::Normal {
                CommState::Retry
            } else {
                CommState::CommErr
            };
            return None;
        }
        self.comm_state = CommState::Normal;

        // ESR: OpComplete=bit0, DevError=bit3, ExeError=bit4.
        let mstat = motor_common::util::atoi(&esr) as u32;
        let eventflgs = mstat & 0x01 != 0 || mstat & 0x08 != 0 || mstat & 0x10 != 0;
        let done = eventflgs || self.stop_status;
        self.stop_status = false;

        let pos_reply = self.query("POS?");
        let position = nint(atof(&pos_reply) / DRIVE_RESOLUTION);

        Some(PolledStatus { position, done })
    }
}

/// Decoded result of one `*ESR?` + `POS?` poll.
struct PolledStatus {
    position: i32,
    done: bool,
}

/// The single axis of an E-662 controller. Implements [`AsynMotor`].
pub struct PIC662Axis {
    controller: Arc<Mutex<PIC662Controller>>,
    last_status: MotorStatus,
}

impl PIC662Axis {
    /// Construct the controller's one axis, seeding the initial status (C
    /// `motor_init`'s `set_status(card, 0)`). A failed initial poll is not an
    /// error.
    pub fn new(controller: Arc<Mutex<PIC662Controller>>) -> AsynResult<Self> {
        let mut axis = Self {
            controller: controller.clone(),
            last_status: MotorStatus {
                done: true,
                ..MotorStatus::default()
            },
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(polled) = ctrl.poll_status() {
            drop(ctrl);
            axis.apply(polled);
        }
        Ok(axis)
    }

    fn lock(&self) -> MutexGuard<'_, PIC662Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Fold a fresh poll into `last_status`, deriving direction from the
    /// count delta (held across polls when the position is unchanged).
    fn apply(&mut self, polled: PolledStatus) {
        let prev = self.last_status.position as i32;
        let direction = if polled.position != prev {
            polled.position >= prev
        } else {
            self.last_status.direction
        };
        self.last_status = MotorStatus {
            position: polled.position as f64,
            encoder_position: polled.position as f64,
            velocity: 0.0,
            done: polled.done,
            moving: !polled.done,
            // E-662: no limit/home/encoder/gain feedback; piezo servo powers
            // its stage continuously. No base velocity (`SET_VEL_BASE` no-op).
            direction,
            powered: true,
            gain_support: false,
            has_encoder: false,
            vbas_supported: false,
            ..MotorStatus::default()
        };
    }
}

impl AsynMotor for PIC662Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let um = position * DRIVE_RESOLUTION;
        self.lock().send(&format!("DEV:CONT REM\nPOS {um:.3}"))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let um = distance * DRIVE_RESOLUTION;
        self.lock().send(&format!("DEV:CONT REM\nPOS:REL {um:.3}"))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C-662 JOG: send = false. No-op (must override the trait default,
        // which would jog to ±1e9).
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C-662 HOME_FOR/HOME_REV: send = false. No-op.
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // No HALT command; force the next poll to report done (C sets
        // cntrl->stop_status = true).
        self.lock().stop_status = true;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C-662 LOAD_POS: send = false ("Can't Load a Position"). No-op.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut ctrl = self.lock();
        let polled = ctrl.poll_status();
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
    fn esr_done_bits_match_c() {
        // eventflgs = OpComplete(0x01) | DevError(0x08) | ExeError(0x10).
        let event = |m: u32| m & 0x01 != 0 || m & 0x08 != 0 || m & 0x10 != 0;
        assert!(event(0x01)); // OpComplete
        assert!(event(0x08)); // DevError
        assert!(event(0x10)); // ExeError
        assert!(!event(0x02)); // ReqControl -> not done
        assert!(!event(0x00));
    }

    #[test]
    fn position_scales_by_drive_resolution() {
        // NINT(atof(reply) / 0.001): "5.012" -> 5012, "-1.5" -> -1500.
        assert_eq!(nint(atof("5.012") / DRIVE_RESOLUTION), 5012);
        assert_eq!(nint(atof("-1.5") / DRIVE_RESOLUTION), -1500);
        // atof junk -> 0 -> 0 counts (press-on, but comm error is handled
        // separately by the *ESR? debounce, not here).
        assert_eq!(nint(atof("") / DRIVE_RESOLUTION), 0);
    }

    #[test]
    fn move_target_scales_to_micrometres() {
        // um = counts * 0.001, printed with 3 decimals.
        assert_eq!(format!("{:.3}", 5012.0 * DRIVE_RESOLUTION), "5.012");
        assert_eq!(format!("{:.3}", -1500.0 * DRIVE_RESOLUTION), "-1.500");
    }
}
