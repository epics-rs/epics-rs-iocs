//! piezosystem jena E-516 (PIJEDS) closed-loop piezo controller driver.
//!
//! Ported from `motorPiJena/piJenaApp/src/drvPIJEDS.cc` + `devPIJEDS.cc`. The
//! E-516 is a **closed-loop** capacitive-feedback piezo nanopositioner: it
//! moves to an absolute target immediately on a `set` command and reports its
//! feedback position with `mess`. There is no hardware "done" bit — motion
//! completion is inferred from the feedback position settling (two consecutive
//! polls within a tolerance), exactly as the C `set_status`.
//!
//! ## Wire protocol
//!
//! Commands are `\r`-terminated by the port's output EOS (C `EDS_OUT_EOS`);
//! replies are framed by an ETX (0x11) input EOS (C `EDS_IN_ETX`), optionally
//! led by an STX (0x13). `PIJEDSConfig` sets both, as C `motor_init` does. Each
//! command names the axis in place of `#`:
//!
//! - `set,<axis>,<pos>`   — move to absolute position (µm), write-only
//! - `sr,<axis>,<vel>`    — set slew velocity, write-only
//! - `cl,<axis>,<0|1>`    — closed-loop (torque) off/on, write-only
//! - `mess,<axis>`        — read feedback position → `mess,<axis>,<pos>`
//! - `stat,<axis>`        — read status word → `stat,<axis>,<n>`
//!
//! The `set`/`sr`/`cl` set-forms return no reply; only the `mess`/`stat`
//! query-forms do (this is what lets the port skip the C driver's port flush).
//!
//! ## Units
//!
//! As C, the record's steps are scaled by `drive_resolution`
//! (`1/10^EDS_MAX_RES` = 0.001) into the controller's µm on `set`/`sr`, and the
//! `mess` reply is `NINT(µm / drive_resolution)` steps.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the settle detection and the
//!   `mess`/`stat` reads run inside [`poll`](AsynMotor::poll).
//! - C `MOVE_REL` sends nothing (the E-516 has no relative command); this port
//!   synthesizes it as an absolute move to `last position + distance`.
//! - C sets velocity in a separate `SET_VELOCITY` transaction; asyn-rs bundles
//!   the velocity into the move, so [`move_absolute`](AsynMotor::move_absolute)
//!   emits `sr` before `set` when a positive velocity is supplied.
//! - C `STOP_AXIS`, `HOME_*` and `LOAD_POS` send nothing (unsupported on this
//!   absolute closed-loop stage); the port mirrors that as no-ops.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atof, atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 128;

/// Position resolution significant digits (C `EDS_MAX_RES`); the `set`/`sr`
/// value is formatted with this many decimals.
const MAX_RES: usize = 3;

/// µm per record step (C `drive_resolution = 1/10^EDS_MAX_RES`).
const DRIVE_RESOLUTION: f64 = 0.001;

/// Settle tolerance for done detection, in steps (C `fdbk_tolerance =
/// 10^(EDS_MAX_RES-1)`).
const TOLERANCE: f64 = 100.0;

/// Status-word bit 7: closed-loop (torque-enabled) mode.
const STATUS_CLOSE_LOOP: i32 = 0x80;

/// Maximum axes per controller (C `EDS_MAX_MOTORS`).
const MAX_MOTORS: usize = 6;

fn eds_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and command framing.
pub struct PiJedsController {
    handle: SyncIOHandle,
    ident: String,
    version: i32,
    num_axes: usize,
}

impl PiJedsController {
    /// Connect and bring an E-516 online (C `motor_init`): identify it (the
    /// reply must contain `DSM`, retried up to three times), parse its DSM
    /// version, and count present axes by probing `mess` until one answers
    /// `not present`. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            version: 0,
            num_axes: 0,
        };
        let mut online = false;
        for _ in 0..3 {
            // GET_IDENT is the empty command (just the terminator).
            if let Ok(reply) = ctrl.query("")
                && reply.contains("DSM")
            {
                ctrl.ident = reply;
                online = true;
                break;
            }
        }
        if !online {
            return Err(eds_err("PIJEDS: controller not online (no 'DSM' identity)"));
        }
        // Version: NINT(atof(after 'V') * 1000).
        if let Some(pos) = ctrl.ident.find('V') {
            ctrl.version = motor_common::util::nint(atof(&ctrl.ident[pos + 1..]) * 1000.0);
        }
        // Count axes: probe until a stage answers "not present".
        let mut total = 0;
        for axis in 0..MAX_MOTORS {
            match ctrl.query(&format!("mess,{axis}")) {
                Ok(reply) if reply.contains("not present") => break,
                Ok(_) => total += 1,
                Err(_) => break,
            }
        }
        ctrl.num_axes = total;
        Ok(ctrl)
    }

    /// The identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// The DSM firmware version (thousandths, e.g. 1959 for `V1.959`).
    pub fn version(&self) -> i32 {
        self.version
    }

    /// Number of present axes detected at init.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Send a write-only set command (`set`/`sr`/`cl`); the E-516 returns no
    /// reply to these forms.
    fn write_cmd(&self, cmd: &str) -> AsynResult<()> {
        self.write_bytes(cmd.as_bytes())
    }

    /// [`write_cmd`](Self::write_cmd) for a record string's raw bytes.
    fn write_bytes(&self, cmd: &[u8]) -> AsynResult<()> {
        self.handle.write_octet(0, cmd)?;
        Ok(())
    }

    /// Send a query command and return its reply, stripped of framing control
    /// bytes (STX/ETX) and surrounding whitespace.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_matches(|c: char| c.is_control() || c.is_whitespace())
            .to_string())
    }
}

/// Extract the value field of a reply (`mnemonic,<axis>,<value>` → `value`):
/// the substring after the second comma.
fn value_field(reply: &str) -> Option<&str> {
    let mut commas = reply.match_indices(',');
    commas.next()?;
    let (idx, _) = commas.next()?;
    Some(reply[idx + 1..].trim())
}

/// One E-516 axis sharing a controller. Implements [`AsynMotor`].
pub struct PiJedsAxis {
    controller: Arc<Mutex<PiJedsController>>,
    /// 0-based axis index; the wire axis number is the same (E-516 axes 0..5).
    axis: usize,
    /// Last feedback position (steps); C `motor_info->position`.
    prev_position: f64,
    /// Consecutive settled polls while moving; C `no_motion_count`.
    no_motion_count: u32,
    /// Whether a commanded move is in progress (C `nodeptr != 0`).
    moving: bool,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl PiJedsAxis {
    /// Construct axis `axis` (0-based). No hardware setup is needed beyond the
    /// controller identification already done in [`PiJedsController::new`].
    pub fn new(controller: Arc<Mutex<PiJedsController>>, axis: usize) -> Self {
        Self {
            controller,
            axis,
            prev_position: 0.0,
            no_motion_count: 0,
            moving: false,
            last_status: MotorStatus {
                done: true,
                // C motor_init: encoder_present = YES, EA_PRESENT = 1.
                has_encoder: true,
                gain_support: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, PiJedsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A record string as the motor queue sends it: `send_mess` puts the
    /// axis name in place of its first `#` (drvPIJEDS.cc:435-443).
    fn named(&self, text: &[u8]) -> Vec<u8> {
        let mut line = text.to_vec();
        if let Some(c) = line.iter_mut().find(|c| **c == b'#') {
            *c = b'0' + self.axis as u8;
        }
        line
    }

    /// PREM as its own message ahead of the move's (devPIJEDS.cc:195-213).
    fn send_prem(&self, ctrl: &PiJedsController, prem: &[u8]) -> AsynResult<()> {
        if prem.is_empty() {
            return Ok(());
        }
        ctrl.write_bytes(&self.named(prem))
    }

    /// Send an absolute move to `position` (steps), optionally setting
    /// the slew velocity first (C `SET_VELOCITY` + `MOVE_ABS`).
    fn issue_move(&mut self, position: f64, velocity: f64) -> AsynResult<()> {
        let prem = self.strings.arm();
        let ctrl = self.lock();
        if velocity > 0.0 {
            ctrl.write_cmd(&format!(
                "sr,{},{:.*}",
                self.axis,
                MAX_RES,
                velocity * DRIVE_RESOLUTION
            ))?;
        }
        self.send_prem(&ctrl, &prem)?;
        ctrl.write_cmd(&format!(
            "set,{},{:.*}",
            self.axis,
            MAX_RES,
            position * DRIVE_RESOLUTION
        ))?;
        drop(ctrl);
        self.moving = true;
        self.no_motion_count = 0;
        Ok(())
    }
}

impl AsynMotor for PiJedsAxis {
    /// C `set_status`'s COMM_ERR branch raises `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` (drvPIJEDS.cc). Its first failure's silent RETRY is not modelled:
    /// a transport error raises both at once.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM_AND_COMMS_ERROR
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.issue_move(position, velocity)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C sends nothing for MOVE_REL; synthesize from the last feedback
        // position (module Deviations).
        self.issue_move(self.prev_position + distance, velocity)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG only sets the slew rate — the E-516 has no continuous jog.
        let prem = self.strings.arm();
        let ctrl = self.lock();
        self.send_prem(&ctrl, &prem)?;
        ctrl.write_cmd(&format!(
            "sr,{},{:.*}",
            self.axis,
            MAX_RES,
            (velocity * DRIVE_RESOLUTION).abs()
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C HOME_* send nothing — no home on an absolute closed-loop stage —
        // but for PREM. C hangs POST off a message it never sends; not
        // copied, POST goes at the next poll that reports done.
        let prem = self.strings.arm();
        self.send_prem(&self.lock(), &prem)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS sends nothing and leaves the motion node, so the stage
        // still travels to target and the settle test still owns `moving`.
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C LOAD_POS sends nothing — position cannot be redefined on the
        // absolute closed-loop encoder.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_cmd(&format!("cl,{},{}", self.axis, i32::from(enable)))
    }

    /// `PRIMITIVE`: INIT as its own message. C's PRIMITIVE case sets
    /// `send = false`, so C never sends it (devPIJEDS.cc:190-191, 263-268);
    /// not copied.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().write_bytes(&self.named(init))
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
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let status_reply = ctrl.query(&format!("stat,{}", self.axis));
        let pos_reply = ctrl.query(&format!("mess,{}", self.axis));

        // A read is OK only when both queries returned a parseable value.
        let status_word = status_reply.ok().as_deref().and_then(value_field).map(atoi);
        let position = pos_reply
            .ok()
            .as_deref()
            .and_then(value_field)
            .map(|v| f64::from(nint(atof(v) / DRIVE_RESOLUTION)));

        let (Some(status_word), Some(position)) = (status_word, position) else {
            // C: first failure after NORMAL is a silent RETRY; a repeat failure
            // is a hard comms error. Keep the last position, flag the error.
            drop(ctrl);
            self.last_status = MotorStatus {
                comms_error: true,
                problem: true,
                ..self.last_status.clone()
            };
            return Ok(self.last_status.clone());
        };

        // Settle detection (C set_status): done once the feedback position has
        // stayed within tolerance for two consecutive polls while moving.
        let delta = (position - self.prev_position).abs();
        let mut done = false;
        let mut direction = self.last_status.direction;
        if delta < TOLERANCE {
            if self.no_motion_count > 0 {
                done = true;
            }
            if self.moving {
                self.no_motion_count += 1;
            }
        } else {
            direction = position >= self.prev_position;
            self.no_motion_count = 0;
        }
        self.prev_position = position;
        if done {
            self.moving = false;
        }
        // drvPIJEDS.cc:388-395: POST without the axis name, as C's
        // send_mess(card, buff, NULL) gives it no `#` substitution.
        if done && let Some(post) = self.strings.post_due() {
            let _ = ctrl.write_bytes(&post);
        }
        drop(ctrl);

        let powered = (status_word & STATUS_CLOSE_LOOP) != 0;
        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            direction,
            done,
            moving: !done,
            // E-516 has no limit or home switches (C set_status).
            high_limit: false,
            low_limit: false,
            home: done,
            powered,
            comms_error: false,
            problem: false,
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicI32, Ordering};

    /// Axis 0 of an E-516 on a [`LineRig`] whose `mess` reads `pos` µm,
    /// with the construction traffic taken.
    fn strings_rig(name: &str) -> (LineRig, PiJedsAxis, Arc<AtomicI32>) {
        let pos = Arc::new(AtomicI32::new(0));
        let p = pos.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "" => "DSM V1.959".into(),
            "mess,0" => format!("mess,0,{}", p.load(Ordering::SeqCst)),
            "stat,0" => "stat,0,133".into(),
            _ => "not present".into(),
        });
        let ctrl = PiJedsController::new(io).unwrap();
        let axis = PiJedsAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        (line, axis, pos)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PiJedsAxis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| l != "stat,0" && l != "mess,0")
            .collect()
    }

    /// devPIJEDS.cc:190-191: INIT as its own message, named like any queued
    /// one.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("pijedsInit");
        axis.primitive(&AsynUser::default(), b"cl,#,1").unwrap();
        assert_eq!(line.sent(), ["cl,0,1"]);
    }

    /// devPIJEDS.cc:195-213: PREM as its own message ahead of the move's,
    /// for that move only.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("pijedsPrem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"cl,#,1", None).unwrap();
        axis.move_absolute(&user, 1500.0, 0.0, 2000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["sr,0,2.000", "cl,0,1", "set,0,1.500"]);
        axis.move_absolute(&user, 1500.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["set,0,1.500"]);
        axis.set_move_strings(&user, b"cl,#,1", None).unwrap();
        axis.move_velocity(&user, 0.0, -2000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["cl,0,1", "sr,0,2.000"]);
        axis.set_move_strings(&user, b"cl,#,1", None).unwrap();
        axis.home(&user, 0.0, 2000.0, 0.0, true).unwrap();
        assert_eq!(line.sent(), ["cl,0,1"]);
    }

    /// drvPIJEDS.cc:388-395: POST once the move settles, not while it runs,
    /// with no axis name.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, pos) = strings_rig("pijedsPost");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"cl,#,0")))
            .unwrap();
        axis.move_absolute(&user, 5000.0, 0.0, 0.0, 0.0).unwrap();
        line.sent();
        pos.store(5, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        assert!(poll_extra(&line, &mut axis).is_empty());
        assert_eq!(poll_extra(&line, &mut axis), ["cl,#,0"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    /// An E-516 axis 0 that steps 1 µm toward its `set` target per `mess`.
    struct Stage {
        base: epics_rs::asyn::port::PortDriverBase,
        pos: f64,
        target: f64,
        reply: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for Stage {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).trim_end().to_string();
            let f: Vec<&str> = cmd.split(',').collect();
            self.reply = match f.as_slice() {
                [""] => b"DSM V1.959".to_vec(),
                ["mess", "0"] => {
                    self.pos += (self.target - self.pos).clamp(-1.0, 1.0);
                    format!("mess,0,{:.3}", self.pos).into_bytes()
                }
                ["stat", "0"] => b"stat,0,133".to_vec(),
                ["mess" | "stat", _] => b"not present".to_vec(),
                ["set", "0", t] => {
                    self.target = t.parse().unwrap();
                    Vec::new()
                }
                _ => Vec::new(),
            };
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let n = self.reply.len();
            buf[..n].copy_from_slice(&self.reply);
            Ok(n)
        }
    }

    /// Axis 0 of an E-516 on a fresh [`Stage`] port named `port`.
    fn stage_axis(port: &str) -> PiJedsAxis {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let stage = Stage {
            base: PortDriverBase::new(port, 1, PortFlags::default()),
            pos: 0.0,
            target: 0.0,
            reply: Vec::new(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(port).unwrap(), stage, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        PiJedsAxis::new(Arc::new(Mutex::new(PiJedsController::new(io).unwrap())), 0)
    }

    /// C STOP_AXIS sends nothing and keeps the motion node, so a stage stopped
    /// mid-travel still settles at its target and reports done.
    #[test]
    fn a_stop_mid_travel_still_ends_in_done() {
        let mut axis = stage_axis("pijeds_stop");
        let user = AsynUser::default();
        axis.move_absolute(&user, 5000.0, 0.0, 0.0, 0.0).unwrap();
        assert!(!axis.poll(&user).unwrap().done);
        axis.stop(&user, 0.0).unwrap();
        let done = (0..20).any(|_| axis.poll(&user).unwrap().done);
        assert!(done, "the stage settled at 5 but never reported done");
        assert_eq!(axis.poll(&user).unwrap().position, 5000.0);
    }

    /// C motor_init sets EA_PRESENT on every axis (drvPIJEDS.cc:666-667), so
    /// MSTA bit 8 holds from boot and on every poll.
    #[test]
    fn every_axis_reports_an_encoder() {
        let mut axis = stage_axis("pijeds_enc");
        assert!(axis.last_status.has_encoder);
        assert!(axis.poll(&AsynUser::default()).unwrap().has_encoder);
    }

    #[test]
    fn value_field_extracts_after_second_comma() {
        assert_eq!(value_field("stat,0,128"), Some("128"));
        assert_eq!(value_field("mess,3,1.234"), Some("1.234"));
        assert_eq!(value_field("sr,0, 5.0 "), Some("5.0"));
        assert_eq!(value_field("noComma"), None);
        assert_eq!(value_field("one,comma"), None);
    }

    #[test]
    fn close_loop_bit_is_bit_seven() {
        // "stat,0,128" → 0x80 set → powered.
        assert_ne!(atoi("128") & STATUS_CLOSE_LOOP, 0);
        // "stat,0,5" (motorExist|fdbk) → bit 7 clear → not powered.
        assert_eq!(atoi("5") & STATUS_CLOSE_LOOP, 0);
    }

    #[test]
    fn resolution_and_tolerance_match_c() {
        assert_eq!(DRIVE_RESOLUTION, 1.0 / 10f64.powi(MAX_RES as i32));
        assert_eq!(TOLERANCE, 10f64.powi(MAX_RES as i32 - 1));
    }
}
