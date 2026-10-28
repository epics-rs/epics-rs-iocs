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
//! the wire. `PIC662Config` sets both to `"\n"` and `PIC662Controller::send`
//! writes **bare** bytes.
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
//! device/execution error), or when [`stop`](Model1Motor::stop) has requested a
//! poll-side stop. `stop_status` is consumed (cleared) only on a successful
//! `*ESR?`.
//!
//! A failed `*ESR?` runs the same NORMAL/RETRY/COMM_ERR two-strike debounce as
//! the C-862 (`CommState`): the first empty reply fails the poll raising
//! nothing (RETRY), the second consecutive one sets `comms_error`/`problem`.
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
//! - **`recv_mess(..., FLUSH)`**: replaced by a flush before every query
//!   (`exchange`), a superset of C's error-path flush.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{Model1Motor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::request::OctetRead;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::{MoveStrings, init_poll, init_read};
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

        // A bare send_mess in C (drvPIC662.cc:558-559).
        let _ = ctrl.send("DEV:CONT REM");
        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `*IDN?`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Write a command, no reply expected (C `send_mess`). No terminator — the
    /// port's output EOS puts the `\n` on the wire. The device does not echo.
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.send_bytes(cmd.as_bytes())
    }

    /// [`send`](Self::send) for raw bytes; an empty message is not sent.
    fn send_bytes(&self, cmd: &[u8]) -> AsynResult<()> {
        if cmd.is_empty() {
            return Ok(());
        }
        self.handle.write_octet(0, cmd)?;
        Ok(())
    }

    /// Write a query and read its reply as one exchange that first flushes
    /// stale input. C reads with no flush past `motor_init` (drvPIC662.cc:415), so one
    /// late reply shifts every later answer: a C defect, not copied.
    fn exchange(&self, cmd: &str) -> String {
        Self::reply(self.handle.write_read(0, cmd.as_bytes(), READ_BUF))
    }

    /// Read one EOS-stripped reply, folding a transport failure or empty read
    /// into `""` (C `recv_mess`'s `if ((status != asynSuccess) || (nread <= 0))
    /// com[0] = '\0'`).
    fn reply(read: OctetRead) -> String {
        match read.into_parts() {
            (raw, Ok(())) if !raw.is_empty() => String::from_utf8_lossy(&raw).into_owned(),
            _ => String::new(),
        }
    }

    fn query(&self, cmd: &str) -> String {
        self.exchange(cmd)
    }

    /// C `set_status`: `*ESR?` (comm-debounced) then `POS?`. `None` means the
    /// caller fails the poll, raising nothing (RETRY) or comm error
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

/// The single axis of an E-662 controller. Implements [`Model1Motor`].
pub struct PIC662Axis {
    controller: Arc<Mutex<PIC662Controller>>,
    /// What the last failed poll raises: nothing on C `set_status`'s
    /// first missed reply (RETRY, drvPIC662.cc:240), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :245). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
    /// C `no_motion_count`: polls the position sat still while a move was
    /// held; a MOTION command restarts it (motordrvCom.cc:435).
    no_motion_count: u32,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl PIC662Axis {
    /// Construct the controller's one axis and read its status, as C
    /// `motor_init`'s `set_status(card, 0)` does (drvPIC662.cc:576).
    pub fn new(controller: Arc<Mutex<PIC662Controller>>) -> Self {
        let mut axis = Self {
            controller,
            failure: PollFailure::NONE,
            // drvPIC662.cc:565-574 prepares no bits.
            last_status: MotorStatus {
                vbas_supported: false,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
            no_motion_count: 0,
            init_status: None,
        };
        axis.init_status = Some(init_poll(&mut axis, &AsynUser::new(0)));
        axis
    }

    fn lock(&self) -> MutexGuard<'_, PIC662Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// One move transaction: PREM and a `;` ahead of the move's own command
    /// (devPIC662.cc:186-203), whose POST waits for the move to end.
    fn write_move(&mut self, command: &str) -> AsynResult<()> {
        let mut line = self.strings.arm();
        if !line.is_empty() {
            line.push(b';');
        }
        line.extend_from_slice(command.as_bytes());
        self.lock().send_bytes(&line)
    }

    /// Fold a fresh poll into `last_status`, deriving direction from the
    /// count delta (held across polls when the position is unchanged).
    fn apply(&mut self, polled: PolledStatus, in_motion: bool) {
        let prev = self.last_status.position as i32;
        let mut done = polled.done;
        let direction = if polled.position != prev {
            self.no_motion_count = 0;
            polled.position >= prev
        } else {
            // drvPIC662.cc:280-282: no done indicator, so a held move still
            // past two polls is done.
            if in_motion {
                done |= self.no_motion_count > 1;
                self.no_motion_count += 1;
            }
            self.last_status.direction
        };
        self.last_status = MotorStatus {
            position: polled.position as f64,
            velocity: 0.0,
            done,
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

impl Model1Motor for PIC662Axis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.last_status.clone(), self.poll_failure())
    }

    /// `update_delay` is 0 (`targs`, drvPIC662.cc:130).
    fn update_delay(&self) -> Duration {
        Duration::ZERO
    }

    /// SET_VELOCITY, SET_ACCEL and SET_VEL_BASE send nothing: a DC motor
    /// (devPIC662.cc).
    fn set_velocity(&mut self, _user: &AsynUser, _velocity: f64) -> AsynResult<()> {
        Ok(())
    }

    fn set_vel_base(&mut self, _user: &AsynUser, _velocity: f64) -> AsynResult<()> {
        Ok(())
    }

    fn set_accel(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        Ok(())
    }

    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let um = position * DRIVE_RESOLUTION;
        self.no_motion_count = 0;
        self.write_move(&format!("DEV:CONT REM\nPOS {:.3}", CFloat(um)))
    }

    fn move_relative(&mut self, _user: &AsynUser, distance: f64) -> AsynResult<()> {
        let um = distance * DRIVE_RESOLUTION;
        self.no_motion_count = 0;
        self.write_move(&format!("DEV:CONT REM\nPOS:REL {:.3}", CFloat(um)))
    }

    fn move_velocity(&mut self, _user: &AsynUser, _velocity: f64) -> AsynResult<()> {
        // C-662 JOG: send = false, so only PREM goes out; POST follows the
        // next done poll (must override the trait default, which would jog
        // to ±1e9).
        self.write_move("")
    }

    fn home(&mut self, _user: &AsynUser, _forward: bool) -> AsynResult<()> {
        // C-662 HOME_FOR/HOME_REV: send = false, so only PREM goes out;
        // POST follows the next done poll. Still a MOTION node, which
        // restarts the count.
        self.no_motion_count = 0;
        self.write_move("")
    }

    fn stop(&mut self, _user: &AsynUser) -> AsynResult<()> {
        // No HALT command; force the next poll to report done (C sets
        // cntrl->stop_status = true).
        self.lock().stop_status = true;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C-662 LOAD_POS: send = false ("Can't Load a Position"). No-op.
        Ok(())
    }

    /// `PRIMITIVE`: INIT as its own message (devPIC662.cc:183-184).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().send_bytes(init)
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

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        *status = self.poll_status(false)?;
        Ok(())
    }

    fn poll_in_motion(
        &mut self,
        _user: &AsynUser,
        _cdir: &mut bool,
        status: &mut MotorStatus,
    ) -> AsynResult<()> {
        *status = self.poll_status(true)?;
        Ok(())
    }
}

impl PIC662Axis {
    /// C `set_status`; `in_motion` is its `nodeptr != 0`.
    fn poll_status(&mut self, in_motion: bool) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let polled = ctrl.poll_status();
        let comm_err = ctrl.comm_state == CommState::CommErr;

        let Some(polled) = polled else {
            self.failure = if comm_err {
                PollFailure::PROBLEM_AND_COMMS_ERROR
            } else {
                PollFailure::NONE
            };
            return Err(pic662_err("no status reply"));
        };
        self.apply(polled, in_motion);
        // drvPIC662.cc:338-344: POST once done; the E-662 has no limits.
        if self.last_status.done
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.send_bytes(&post);
        }
        drop(ctrl);
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use motor_common::testing::Model1Polled;
    use motor_common::testing::RecordWrites;
    use std::sync::atomic::{AtomicU8, Ordering};

    /// C sprintf's `%.*f` and `%+.*f` spell a NaN as glibc does: `nan`,
    /// `-nan`, `+nan`.
    #[test]
    fn floats_go_out_as_c_sprintf() {
        let nan = f64::NAN;
        let neg = -f64::NAN;
        let (line, mut axis, _) = strings_rig("c662CFloat");
        let user = AsynUser::default();
        let mut sent = String::new();
        let mut run = |f: &mut dyn FnMut(&mut PIC662Axis)| {
            f(&mut axis);
            sent.extend(line.sent().into_iter().map(|w| w + "|"));
        };
        run(&mut |a| drop(a.record_move_absolute(&user, nan, 0.0, 1.0, 1.0)));
        run(&mut |a| drop(a.record_move_relative(&user, neg, 0.0, 1.0, 1.0)));
        assert!(!sent.contains("NaN"), "{sent}");
        for want in ["POS nan", "POS:REL -nan"] {
            assert!(sent.contains(want), "{want}: {sent}");
        }
    }

    /// An E-662 on a [`LineRig`] whose `*ESR?` reply follows `esr`.
    fn strings_rig(name: &str) -> (LineRig, PIC662Axis, Arc<AtomicU8>) {
        let esr = Arc::new(AtomicU8::new(1));
        let e = esr.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "*IDN?" => "E-662".into(),
            "*ESR?" => e.load(Ordering::SeqCst).to_string(),
            _ => "0.000".into(),
        });
        let ctrl = PIC662Controller::new(io).unwrap();
        let axis = PIC662Axis::new(Arc::new(Mutex::new(ctrl)));
        line.sent();
        (line, axis, esr)
    }

    /// devPIC662.cc:258: JOG_VELOCITY sends nothing (`send = false`).
    #[test]
    fn a_jvel_retune_sends_nothing() {
        let (line, mut axis, _) = strings_rig("pic662Jvel");
        axis.jog_velocity(&AsynUser::default(), 5.0).unwrap();
        assert!(line.sent().is_empty());
    }

    /// drvPIC662.cc:292 has the encoder_position set commented out, so REP
    /// keeps motor_init's 0 (:567).
    #[test]
    fn the_poll_sets_no_encoder_position() {
        let (_line, io) = LineRig::new("pic662NoRep", |last| match last {
            "*IDN?" => "E-662".into(),
            "*ESR?" => "1".into(),
            _ => "5.012".into(),
        });
        let ctrl = PIC662Controller::new(io).unwrap();
        let mut axis = PIC662Axis::new(Arc::new(Mutex::new(ctrl)));
        let s = axis.polled(&AsynUser::default()).unwrap();
        assert_eq!((s.position, s.encoder_position), (5012.0, 0.0));
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PIC662Axis) -> Vec<String> {
        axis.polled(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| l != "*ESR?" && l != "POS?")
            .collect()
    }

    /// drvPIC662.cc:235-250 exit stores what motor_init prepared (:565-574,
    /// no bits), with both COMM_ERR bits once past RETRY; the port's
    /// no-VBAS flag stays on it as on every poll.
    #[test]
    fn a_failed_init_read_keeps_what_motor_init_set() {
        let (_line, io) = LineRig::new("pic662InitFail", |last| match last {
            "*IDN?" => "E-662".into(),
            _ => String::new(),
        });
        let ctrl = Arc::new(Mutex::new(PIC662Controller::new(io).unwrap()));
        let mut first = PIC662Axis::new(ctrl.clone());
        let mut again = PIC662Axis::new(ctrl);
        let prepared = MotorStatus {
            vbas_supported: false,
            ..MotorStatus::default()
        };
        assert_eq!(first.take_init_status(), prepared);
        assert_eq!(
            again.take_init_status(),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..prepared
            }
        );
    }

    /// devPIC662.cc:183-184: INIT as its own message.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("pic662Init");
        axis.primitive(&AsynUser::default(), b"SYST:ERR?").unwrap();
        assert_eq!(line.sent(), ["SYST:ERR?"]);
    }

    /// devPIC662.cc:186-203: PREM and a `;` lead the move's own message, for
    /// that move only; a home or jog sends PREM alone.
    #[test]
    fn prem_leads_the_move() {
        let (line, mut axis, _) = strings_rig("pic662Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"VOLT 1", None).unwrap();
        axis.record_move_absolute(&user, 5012.0, 0.0, 1.0, 1.0)
            .unwrap();
        assert_eq!(line.sent(), ["VOLT 1;DEV:CONT REM\nPOS 5.012"]);
        axis.record_move_relative(&user, -1500.0, 0.0, 1.0, 1.0)
            .unwrap();
        assert_eq!(line.sent(), ["DEV:CONT REM\nPOS:REL -1.500"]);
        axis.set_move_strings(&user, b"VOLT 1", None).unwrap();
        axis.record_home(&user, 0.0, 1.0, 1.0, true).unwrap();
        assert_eq!(line.sent(), ["VOLT 1;"]);
        axis.set_move_strings(&user, b"", None).unwrap();
        axis.record_jog(&user, 0.0, 1.0, 1.0).unwrap();
        assert!(line.sent().is_empty());
    }

    /// drvPIC662.cc:338-344: POST once the move is done, not while it runs,
    /// and only once.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, esr) = strings_rig("pic662Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"VOLT 0")))
            .unwrap();
        axis.record_move_absolute(&user, 5012.0, 0.0, 1.0, 1.0)
            .unwrap();
        line.sent();
        esr.store(0, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        esr.store(1, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["VOLT 0"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    /// drvPIC662.cc:278-283: with no done indicator, a held move whose
    /// position sits still is done at the third still poll
    /// (`no_motion_count++ > 1`); the count runs only while a move is held
    /// and a MOTION command restarts it (motordrvCom.cc:435).
    #[test]
    fn a_held_move_still_for_three_polls_is_done() {
        let (_line, mut axis, esr) = strings_rig("pic662Settle");
        let user = AsynUser::default();
        esr.store(0, Ordering::SeqCst);
        for _ in 0..5 {
            assert!(!axis.polled(&user).unwrap().done);
        }
        let mut cdir = true;
        for _ in 0..2 {
            axis.record_move_absolute(&user, 0.0, 0.0, 1.0, 1.0)
                .unwrap();
            let done: Vec<bool> = (0..4)
                .map(|_| axis.polled_in_motion(&user, &mut cdir).unwrap().done)
                .collect();
            assert_eq!(done, [false, false, true, true]);
        }
    }

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

    /// drvPIC662.cc:231-251: a first failed status read is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a good read clears it.
    #[test]
    fn a_failed_status_read_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("c662Retry", move |last| {
            if q.load(Ordering::SeqCst) {
                return String::new();
            }
            match last {
                "*IDN?" => "E-662".into(),
                "*ESR?" => "1".into(),
                _ => "0.000".into(),
            }
        });
        let ctrl = PIC662Controller::new(io).unwrap();
        let mut axis = PIC662Axis::new(Arc::new(Mutex::new(ctrl)));
        let user = AsynUser::default();
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        let s = axis.polled(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }
}
