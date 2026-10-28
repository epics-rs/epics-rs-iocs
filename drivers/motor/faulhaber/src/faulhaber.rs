//! Faulhaber MCDC2805 DC servo motor controller driver (serial ASCII).
//!
//! Ported from `motorFaulhaber/faulhaberApp/src/drvMCDC2805.cc` +
//! `devMCDC2805.cc`. The MCDC2805 is a DC servo positioner working in encoder
//! counts. Several modules can share one serial line, each addressed by a node
//! number prefixed to every command (e.g. node 0 status query is `0GST`).
//! Commands are terminated by the serial port's output EOS (CR, set in the
//! startup script). Set commands (`LA`/`LR`/`M`/`SP`/`V`/…) get no reply; only
//! the `VER`/`GST`/`POS`/`GAST`/`GV` queries return one.
//!
//! ## Units
//!
//! The controller works natively in encoder counts with no resolution scaling
//! (`POS` returns counts, `LA` takes counts), so the asyn-rs motor boundary is
//! counts: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is counts. Velocities cross the boundary in counts/s and
//! accelerations in counts/s²; the controller wants rev/min and rev/s², so they
//! are converted with the axis's counts-per-rev, sent as `ENCRES` ahead of
//! every `SP`/`AC`/`HOSP` as C does, from the record's `SREV`
//! (devMCDC2805.cc:227), which arrives with
//! [`set_record_fields`](AsynMotor::set_record_fields) ahead of each move.
//!
//! ## Done detection
//!
//! There is no reliable "reached target" bit for jog moves or velocity=0 stops,
//! so — exactly as C `set_status` — done is derived from the actual velocity
//! (`GV`) being zero.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `GST`/`POS`/`GAST`/`GV`
//!   reads and the limit-switch hold run inside [`poll`](AsynMotor::poll).

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage, RecordFields,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::{MoveStrings, init_poll, init_read};
use motor_common::util::{atof, nint};

/// Query read timeout (C `recv_mess` 1.0 s); writes use the caller's
/// `COMM_TIMEOUT`.
const RECV_TIMEOUT: Duration = Duration::from_secs(1);

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 128;

/// Maximum modules per serial line (C `MCDC2805_NUM_CARDS` axis table width).
const MAX_MOTORS: usize = 8;

fn faulhaber_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and node-prefixed framing.
pub struct FaulhaberController {
    handle: SyncIOHandle,
    reader: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

/// Controller communication state (C `cntrl->status`, shared by the axes):
/// one failed `GST` is retried silently, a second in a row is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

impl FaulhaberController {
    /// Connect and probe up to `num_motors` nodes with `VER` (C `motor_init`):
    /// each node is retried three times, and probing stops at the first node
    /// that never answers. The first node's reply becomes the identification.
    /// Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, num_motors: usize) -> AsynResult<Self> {
        let reader = handle.with_timeout(RECV_TIMEOUT);
        let mut ctrl = Self {
            handle,
            reader,
            ident: String::new(),
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        let n = num_motors.min(MAX_MOTORS);
        let mut total = 0;
        for node in 0..n {
            let mut reply = None;
            for _ in 0..3 {
                if let Ok(r) = ctrl.query(node, "VER")
                    && !r.is_empty()
                {
                    reply = Some(r);
                    break;
                }
            }
            match reply {
                Some(r) => {
                    if node == 0 {
                        ctrl.ident = r;
                    }
                    total += 1;
                }
                None => break,
            }
        }
        if total == 0 {
            return Err(faulhaber_err("MCDC2805: no nodes responded to VER"));
        }
        ctrl.num_axes = total;
        Ok(ctrl)
    }

    /// The identification string (node 0's `VER` reply).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of responding nodes detected at init.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(node: usize, cmd: impl AsRef<[u8]>) -> Vec<u8> {
        let mut out = node.to_string().into_bytes();
        out.extend_from_slice(cmd.as_ref());
        out
    }

    /// Send a write-only command to `node` (no reply expected).
    fn write_cmd(&self, node: usize, cmd: &str) -> AsynResult<()> {
        self.write_bytes(node, cmd.as_bytes())
    }

    /// [`write_cmd`](Self::write_cmd) for a record string's raw bytes.
    fn write_bytes(&self, node: usize, cmd: &[u8]) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(node, cmd))?;
        Ok(())
    }

    /// Send a query to `node` and return its reply, trimmed of framing.
    fn query(&self, node: usize, cmd: &str) -> AsynResult<String> {
        // C send_mess/recv_mess write then read with no flush
        // (drvMCDC2805.cc:442), so one stray line shifts every later reply;
        // write_read flushes first: a C defect, not copied.
        let (raw, status) = self
            .reader
            .write_read(0, &Self::framed(node, cmd), READ_BUF)
            .into_parts();
        status?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_matches(|c: char| c.is_control() || c.is_whitespace())
            .to_string())
    }
}

/// Convert a boundary velocity (counts/s) to the controller's rev/min.
fn vel_to_rpm(velocity: f64, counts_per_rev: f64) -> i32 {
    if counts_per_rev <= 0.0 {
        0
    } else {
        nint(velocity / counts_per_rev * 60.0)
    }
}

/// Convert a boundary acceleration (counts/s²) to the controller's rev/s².
fn accel_to_rev(acceleration: f64, counts_per_rev: f64) -> i32 {
    if counts_per_rev <= 0.0 {
        0
    } else {
        nint(acceleration / counts_per_rev)
    }
}

/// One MCDC2805 axis sharing a controller. Implements [`AsynMotor`].
pub struct FaulhaberAxis {
    controller: Arc<Mutex<FaulhaberController>>,
    /// 0-based node number prefixed to this axis's commands.
    node: usize,
    /// The record's VELO (counts/s) and SREV, as C build_trans reads `mr`.
    record: RecordFields,
    /// Last integer position (counts); C `motor_info->position`.
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed `GST` (RETRY, drvMCDC2805.cc:240), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :246). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl FaulhaberAxis {
    /// Construct axis `node` (0-based) and run the C `motor_init` per-axis setup:
    /// RS-232 velocity source (`SOR 0`) and the limit-switch and homing
    /// configuration, then its status read. Performs blocking serial I/O; as
    /// C's bare `send_mess` (drvMCDC2805.cc:600-623), a refused command keeps
    /// the axis.
    pub fn new(controller: Arc<Mutex<FaulhaberController>>, node: usize) -> Self {
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Velocity control source = RS-232, then the limit-switch /
            // homing configuration (C motor_init sequence).
            for cmd in [
                "SOR 0", "REFIN", "HP7", "HB6", "HD2", "HL1", "HA1", "CAHOSEQ",
            ] {
                let _ = ctrl.write_cmd(node, cmd);
            }
        }
        let mut axis = Self {
            controller,
            node,
            // Read only by a move, which set_record_fields leads.
            record: RecordFields {
                velocity: 0.0,
                srev: 0,
            },
            prev_position: 0,
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
            init_status: None,
        };
        axis.init_status = Some(init_poll(&mut axis, &AsynUser::new(0)));
        axis
    }

    fn counts_per_rev(&self) -> f64 {
        f64::from(self.record.srev)
    }

    fn lock(&self) -> MutexGuard<'_, FaulhaberController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn encres(&self) -> String {
        format!("ENCRES{}", self.record.srev)
    }

    /// The record's SET_VELOCITY, then SET_ACCEL unless the acceleration is
    /// zero (motorRecord.cc:938-942), each `ENCRES` then `SP` / `AC`
    /// (devMCDC2805.cc:248-270). Like every message of a transaction, each
    /// goes out whatever the one before it returned: `send_mess` ignores the
    /// write (drvMCDC2805.cc), and so does motordrvCom (motordrvCom.cc:397-484).
    fn program_motion(&self, velocity: f64, acceleration: f64) {
        let ctrl = self.lock();
        let _ = ctrl.write_cmd(self.node, &self.encres());
        let _ = ctrl.write_cmd(
            self.node,
            &format!("SP {}", vel_to_rpm(velocity, self.counts_per_rev())),
        );
        if acceleration > 0.0 {
            self.set_accel(&ctrl, acceleration);
        }
    }

    fn set_accel(&self, ctrl: &FaulhaberController, acceleration: f64) {
        let _ = ctrl.write_cmd(self.node, &self.encres());
        let _ = ctrl.write_cmd(
            self.node,
            &format!("AC {}", accel_to_rev(acceleration, self.counts_per_rev())),
        );
    }

    /// PREM and a blank lead a move's first message, and its POST waits for
    /// the move to end. C HOME/JOG strcpy their first split message over
    /// PREM and replace the motion node holding POST; that defect is not
    /// copied.
    fn lead_move(&mut self, command: &str) -> AsynResult<()> {
        let mut line = self.strings.arm();
        if !line.is_empty() {
            line.push(b' ');
        }
        line.extend_from_slice(command.as_bytes());
        self.lock().write_bytes(self.node, &line)
    }
}

impl Model1Motor for FaulhaberAxis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.last_status.clone(), self.poll_failure())
    }
}

impl AsynMotor for FaulhaberAxis {
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.program_motion(velocity, acceleration);
        let _ = self.lead_move(&format!("LA {}", nint(position)));
        self.lock().write_cmd(self.node, "M")
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.program_motion(velocity, acceleration);
        let _ = self.lead_move(&format!("LR {}", nint(distance)));
        self.lock().write_cmd(self.node, "M")
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The record's unguarded SET_ACCEL (motorRecord.cc:2141), then C JOG:
        // ENCRES, the speed, and velocity mode at the signed rate. C's SP is
        // signed too; a maximum speed is not, so that is not copied.
        // C never sends that SP: its node keeps type UNDEFINED, which
        // motor_send (motordrvCom.cc:563-576) refuses; that defect is not copied.
        self.set_accel(&self.lock(), acceleration);
        let rpm = vel_to_rpm(velocity, self.counts_per_rev());
        let _ = self.lead_move(&self.encres());
        let ctrl = self.lock();
        let _ = ctrl.write_cmd(self.node, &format!("SP {}", rpm.abs()));
        ctrl.write_cmd(self.node, &format!("V {rpm}"))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // The record's SET_VELOCITY/SET_ACCEL (motorRecord.cc:872-875), then C
        // HOME_*: ENCRES, the homing speed signed by direction, the homing
        // sequence, and GO's `M`.
        // C never sends HOSP: its node keeps type UNDEFINED, which motor_send
        // (motordrvCom.cc:563-576) refuses; that defect is not copied.
        self.program_motion(velocity, acceleration);
        // HOSP runs at VELO, not HVEL (devMCDC2805.cc:224); C divides VELO's
        // EGU/s by SREV without MRES, a unit mix not copied.
        let mut rpm = vel_to_rpm(self.record.velocity, self.counts_per_rev());
        if !forward {
            rpm = -rpm;
        }
        let _ = self.lead_move(&self.encres());
        let ctrl = self.lock();
        let _ = ctrl.write_cmd(self.node, &format!("HOSP{rpm}"));
        let _ = ctrl.write_cmd(self.node, "GOHOSEQ");
        ctrl.write_cmd(self.node, "M")
    }

    fn set_record_fields(&mut self, _user: &AsynUser, fields: &RecordFields) -> AsynResult<()> {
        self.record = *fields;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS: velocity 0.
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, "V 0")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C LOAD_POS: redefine the current position (HO = home offset).
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, &format!("HO {}", nint(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, if enable { "EN" } else { "DI" })
    }

    /// `PRIMITIVE`: a blank, then INIT.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut line = vec![b' '];
        line.extend_from_slice(init);
        self.lock().write_bytes(self.node, &line)
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

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C SET_PGAIN/SET_IGAIN scale the 0..1 gain by 255; derivative is
        // unsupported.
        let ctrl = self.lock();
        match kind {
            PidGainKind::Proportional => {
                ctrl.write_cmd(self.node, &format!("POR {}", nint(gain * 255.0)))
            }
            PidGainKind::Integral => {
                ctrl.write_cmd(self.node, &format!("I {}", nint(gain * 255.0)))
            }
            PidGainKind::Derivative => Ok(()),
        }
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // C drvMCDC2805.cc:229-251: an empty or failed GST fails the poll
        // raising nothing once (RETRY); a second in a row raises COMM_ERR.
        // Nothing else is sent on that poll.
        let gst = ctrl.query(self.node, "GST").ok().filter(|r| !r.is_empty());
        let Some(status_buff) = gst else {
            (ctrl.comm_state, self.failure) = match ctrl.comm_state {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(faulhaber_err(format!(
                "MCDC2805 node {}: no GST reply",
                self.node
            )));
        };
        ctrl.comm_state = CommState::Normal;
        // A later write's transport error, which C does not check, raises both.
        self.failure = PollFailure::PROBLEM_AND_COMMS_ERROR;
        let pos = ctrl.query(self.node, "POS");
        let gast = ctrl.query(self.node, "GAST");
        let gv = ctrl.query(self.node, "GV");

        let status_bytes = status_buff.as_bytes();
        let home = status_bytes.get(6) == Some(&b'1');

        // drvMCDC2805.cc:263-281: the raw POS against the last rounded
        // position; only a change moves it and sets the direction.
        let motor_data = pos.as_deref().map(atof).unwrap_or(0.0);
        let mut direction = self.last_status.direction;
        if motor_data != f64::from(self.prev_position) {
            let new_position = nint(motor_data);
            direction = new_position >= self.prev_position;
            self.prev_position = new_position;
        }
        let plusdir = direction;

        // Limit switches (GAST: byte 0 = plus, byte 1 = minus).
        let gast_bytes = gast.unwrap_or_default();
        let gast_bytes = gast_bytes.as_bytes();
        let plus_ls = gast_bytes.first() == Some(&b'1');
        let minus_ls = gast_bytes.get(1) == Some(&b'1');
        let ls_active = (plus_ls && plusdir) || (minus_ls && !plusdir);

        // If an active limit switch stopped motion, hold position with an
        // absolute move to the current position (C set_status). C's
        // send_mess status is not checked here or for POST.
        if ls_active {
            let _ = ctrl.write_cmd(self.node, &format!("LA {}", nint(motor_data)));
            let _ = ctrl.write_cmd(self.node, "M");
        }

        // Actual velocity → done (reliable across jog and velocity=0 stops).
        let velocity = gv.as_deref().map(|s| nint(atof(s))).unwrap_or(0);
        let done = velocity == 0;
        if (done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.write_bytes(self.node, &post);
        }
        drop(ctrl);

        let signed_velocity = if plusdir { velocity } else { -velocity };

        self.last_status = MotorStatus {
            position: f64::from(self.prev_position),
            // C `(int) motorData`.
            encoder_position: f64::from(motor_data as i32),
            velocity: signed_velocity as f64,
            direction,
            done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            home,
            comms_error: false,
            problem: false,
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        *status = self.last_status.clone();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use motor_common::testing::LineRig;
    use motor_common::testing::Polled;
    use std::sync::atomic::{AtomicBool, Ordering};

    /// A record's fields with SREV 1000.
    fn fields(velocity: f64) -> RecordFields {
        RecordFields {
            velocity,
            srev: 1000,
        }
    }

    struct Rig {
        line: LineRig,
        axis: FaulhaberAxis,
        moving: Arc<AtomicBool>,
        minus: Arc<AtomicBool>,
        pos: Arc<Mutex<&'static str>>,
        user: AsynUser,
    }

    impl Rig {
        /// Node 1 at 1000 counts/rev, its setup traffic taken.
        fn new(name: &str) -> Self {
            Self::refusing(name, |_| false)
        }

        /// [`Rig::new`], with the writes `refused` picks refused.
        fn refusing(name: &str, refused: impl Fn(&str) -> bool + Send + Sync + 'static) -> Self {
            let moving = Arc::new(AtomicBool::new(false));
            let minus = Arc::new(AtomicBool::new(false));
            let pos = Arc::new(Mutex::new("0"));
            let (m, l, p) = (moving.clone(), minus.clone(), pos.clone());
            let reply = move |last: &str| match last.get(1..).unwrap_or_default() {
                "VER" => "MCDC2805".into(),
                "GST" => "0000000".into(),
                "POS" => (*p.lock().unwrap()).into(),
                "GAST" => if l.load(Ordering::SeqCst) { "01" } else { "00" }.into(),
                "GV" => if m.load(Ordering::SeqCst) { "-60" } else { "0" }.into(),
                _ => String::new(),
            };
            let (line, io) = LineRig::failing(name, reply, refused);
            let controller = Arc::new(Mutex::new(FaulhaberController::new(io, 2).unwrap()));
            let mut axis = FaulhaberAxis::new(controller, 1);
            let user = AsynUser::new(0);
            axis.set_record_fields(&user, &fields(0.0)).unwrap();
            line.sent();
            Self {
                line,
                axis,
                moving,
                minus,
                pos,
                user,
            }
        }

        /// What one poll sent besides its queries.
        fn poll_extra(&mut self) -> Vec<String> {
            self.axis.polled(&self.user).unwrap();
            self.line
                .sent()
                .into_iter()
                .filter(|l| !["1GST", "1POS", "1GAST", "1GV"].contains(&l.as_str()))
                .collect()
        }
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// devMCDC2805.cc:185-189.
    #[test]
    fn init_goes_out_behind_a_blank() {
        let mut r = Rig::new("mcdc_init");
        r.axis.primitive(&r.user, b"EN").unwrap();
        assert_eq!(r.line.sent(), ["1 EN"]);
    }

    /// devMCDC2805.cc:191-205: PREM and a blank lead `LA`/`LR`, for that
    /// move only.
    #[test]
    fn prem_leads_the_target_message() {
        let mut r = Rig::new("mcdc_prem");
        r.axis.set_move_strings(&r.user, b"EN", None).unwrap();
        r.axis.move_absolute(&r.user, 100.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1ENCRES1000", "1SP 0", "1EN LA 100", "1M"]);
        r.axis.move_relative(&r.user, -5.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1ENCRES1000", "1SP 0", "1LR -5", "1M"]);
        r.axis.set_move_strings(&r.user, b"EN", None).unwrap();
        r.axis.move_relative(&r.user, -5.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1ENCRES1000", "1SP 0", "1EN LR -5", "1M"]);
    }

    /// devMCDC2805.cc:222-236 and 286-300 lose PREM and POST on HOME and
    /// JOG; rs sends both as for any other move. HOSP is VELO's, not the
    /// HVEL the record sent (devMCDC2805.cc:224).
    #[test]
    fn home_and_jog_send_prem_and_post() {
        let mut r = Rig::new("mcdc_split");
        r.axis
            .set_move_strings(&r.user, b"EN", Some(post(b"DI")))
            .unwrap();
        r.axis.set_record_fields(&r.user, &fields(2000.0)).unwrap();
        r.axis.home(&r.user, 0.0, 1000.0, 0.0, false).unwrap();
        assert_eq!(
            r.line.sent(),
            [
                "1ENCRES1000",
                "1SP 60",
                "1EN ENCRES1000",
                "1HOSP-120",
                "1GOHOSEQ",
                "1M"
            ]
        );
        assert_eq!(r.poll_extra(), ["1DI"]);
        r.axis
            .set_move_strings(&r.user, b"EN", Some(post(b"DI")))
            .unwrap();
        r.axis.move_velocity(&r.user, 0.0, -1000.0, 0.0).unwrap();
        assert_eq!(
            r.line.sent(),
            ["1ENCRES1000", "1AC 0", "1EN ENCRES1000", "1SP 60", "1V -60"]
        );
        assert_eq!(r.poll_extra(), ["1DI"]);
    }

    /// drvMCDC2805.cc:360-366: POST once the move is done, not while it
    /// runs.
    #[test]
    fn post_follows_the_move_once_done() {
        let mut r = Rig::new("mcdc_post");
        r.axis
            .set_move_strings(&r.user, b"", Some(post(b"DI")))
            .unwrap();
        r.axis.move_absolute(&r.user, 100.0, 0.0, 0.0, 0.0).unwrap();
        r.line.sent();
        r.moving.store(true, Ordering::SeqCst);
        assert!(r.poll_extra().is_empty());
        r.moving.store(false, Ordering::SeqCst);
        assert_eq!(r.poll_extra(), ["1DI"]);
        assert!(r.poll_extra().is_empty(), "sent once");
    }

    /// drvMCDC2805.cc:361: a limit in the move's direction ends it too,
    /// after the hold at the current position.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let mut r = Rig::new("mcdc_post_ls");
        r.axis
            .set_move_strings(&r.user, b"", Some(post(b"DI")))
            .unwrap();
        r.axis
            .move_absolute(&r.user, -100.0, 0.0, 0.0, 0.0)
            .unwrap();
        r.line.sent();
        r.moving.store(true, Ordering::SeqCst);
        r.minus.store(true, Ordering::SeqCst);
        assert_eq!(r.poll_extra(), ["1LA 0", "1M", "1DI"]);
    }

    /// motordrvCom sends each message of a transaction and ignores what
    /// send_mess returns (motordrvCom.cc:397-484): a refused ENCRES or
    /// GOHOSEQ still sends the messages after it.
    #[test]
    fn a_refused_message_still_sends_the_ones_after_it() {
        let mut r = Rig::refusing("mcdc_refused", |cmd| {
            ["1ENCRES1000", "1GOHOSEQ", "1V -60"].contains(&cmd)
        });
        r.axis.set_record_fields(&r.user, &fields(2000.0)).unwrap();
        r.axis.move_absolute(&r.user, 100.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1ENCRES1000", "1SP 0", "1LA 100", "1M"]);
        r.axis.home(&r.user, 0.0, 1000.0, 0.0, false).unwrap();
        assert_eq!(
            r.line.sent(),
            [
                "1ENCRES1000",
                "1SP 60",
                "1ENCRES1000",
                "1HOSP-120",
                "1GOHOSEQ",
                "1M"
            ]
        );
        assert!(r.axis.move_velocity(&r.user, 0.0, -1000.0, 0.0).is_err());
        assert_eq!(
            r.line.sent(),
            ["1ENCRES1000", "1AC 0", "1ENCRES1000", "1SP 60", "1V -60"]
        );
    }

    #[test]
    fn framed_prefixes_node() {
        assert_eq!(FaulhaberController::framed(0, "GST"), b"0GST");
        assert_eq!(FaulhaberController::framed(3, "LA 1000"), b"3LA 1000");
    }

    #[test]
    fn vel_to_rpm_converts_counts_per_sec() {
        // 1000 counts/s at 1000 counts/rev = 1 rev/s = 60 rpm.
        assert_eq!(vel_to_rpm(1000.0, 1000.0), 60);
        // Half a rev/s.
        assert_eq!(vel_to_rpm(500.0, 1000.0), 30);
        // Guard against zero counts/rev.
        assert_eq!(vel_to_rpm(1000.0, 0.0), 0);
    }

    #[test]
    fn accel_to_rev_converts_counts_per_sec2() {
        // 3000 counts/s² at 1000 counts/rev = 3 rev/s².
        assert_eq!(accel_to_rev(3000.0, 1000.0), 3);
        assert_eq!(accel_to_rev(3000.0, 0.0), 0);
    }

    /// drvMCDC2805.cc:589-623: a failed `motor_init` read stores the
    /// GAIN_SUPPORT and EA_PRESENT `motor_init` set, the first raising
    /// nothing (RETRY) and the next CNTRL_COMM_ERR and RA_PROBLEM
    /// (:238-250).
    #[test]
    fn a_failed_init_read_keeps_what_motor_init_set() {
        let (_line, io) = LineRig::new("faulhaberInitFail", |last| {
            match last.get(1..).unwrap_or_default() {
                "VER" => "MCDC2805".into(),
                _ => String::new(),
            }
        });
        let controller = Arc::new(Mutex::new(FaulhaberController::new(io, 2).unwrap()));
        let init = |node| FaulhaberAxis::new(controller.clone(), node).take_init_status();
        let prepared = MotorStatus {
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        assert_eq!(init(0), prepared);
        assert_eq!(
            init(1),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..prepared
            }
        );
    }

    /// drvMCDC2805.cc:229-251: a first missed `GST` is RETRY (nothing
    /// raised, nothing posted), the next COMM_ERR, and a reply clears it.
    #[test]
    fn a_missed_gst_is_retry_then_comm_error() {
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("faulhaberRetry", move |last| {
            match last.get(1..).unwrap_or_default() {
                "VER" => "MCDC2805".into(),
                "GST" if !q.load(Ordering::SeqCst) => "0000000".into(),
                "POS" | "GV" => "0".into(),
                "GAST" => "00".into(),
                _ => String::new(),
            }
        });
        let controller = Arc::new(Mutex::new(FaulhaberController::new(io, 2).unwrap()));
        let mut axis = FaulhaberAxis::new(controller, 1);
        let user = AsynUser::new(0);
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

    /// drvMCDC2805.cc:446 reads at 1.0 s while send_mess writes at
    /// COMM_TIMEOUT, so a silent node fails a poll in 1 s.
    #[test]
    fn queries_read_at_one_second_and_writes_keep_the_comm_timeout() {
        let (_line, io) = LineRig::new("faulhaberTimeout", |last| {
            match last.get(1..).unwrap_or_default() {
                "VER" => "MCDC2805".into(),
                _ => String::new(),
            }
        });
        let write_timeout = io.timeout();
        let ctrl = FaulhaberController::new(io, 1).unwrap();
        assert_eq!(ctrl.reader.timeout(), Some(Duration::from_secs(1)));
        assert_eq!(ctrl.handle.timeout(), write_timeout);
    }

    /// motorRecord.cc:938-945 with devMCDC2805.cc:248-274: SP before AC,
    /// each behind its own ENCRES, then the target and `M`.
    #[test]
    fn a_move_sends_the_records_transactions() {
        let mut r = Rig::new("mcdc_move");
        r.axis
            .move_absolute(&r.user, 100.0, 0.0, 2000.0, 3000.0)
            .unwrap();
        assert_eq!(
            r.line.sent(),
            [
                "1ENCRES1000",
                "1SP 120",
                "1ENCRES1000",
                "1AC 3",
                "1LA 100",
                "1M"
            ]
        );
    }

    /// drvMCDC2805.cc:263-281, 334-339: the position is NINT of POS, the
    /// encoder its truncation, and a fraction off the last position still
    /// sets the direction from the rounded one.
    #[test]
    fn position_rounds_encoder_truncates() {
        let mut r = Rig::new("mcdc_pos");
        *r.pos.lock().unwrap() = "-20";
        assert!(!r.axis.polled(&r.user).unwrap().direction);
        *r.pos.lock().unwrap() = "-20.4";
        let s = r.axis.polled(&r.user).unwrap();
        assert_eq!((s.position, s.encoder_position), (-20.0, -20.0));
        assert!(s.direction, "-20 >= -20");
        *r.pos.lock().unwrap() = "-20";
        assert!(r.axis.polled(&r.user).unwrap().direction, "kept");
        *r.pos.lock().unwrap() = "7.6";
        let s = r.axis.polled(&r.user).unwrap();
        assert_eq!((s.position, s.encoder_position), (8.0, 7.0));
    }
}
