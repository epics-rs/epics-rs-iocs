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
//! are converted with the axis's counts-per-rev (the `MCDC2805Config`
//! `countsPerRev` argument, which also programs the controller via `ENCRES`).
//! This replaces the C driver's use of the record's `SREV` field, which is not
//! visible at the asyn-rs boundary.
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
//! - `countsPerRev` is a config argument (sent once via `ENCRES` at init)
//!   rather than the record's `SREV` re-sent before every velocity command.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atof, nint};

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
        let mut ctrl = Self {
            handle,
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
        // C send_mess/recv_mess write then read with no flush, so one stray
        // line shifts every later reply; write_read flushes first.
        let raw = self
            .handle
            .write_read(0, &Self::framed(node, cmd), READ_BUF)?;
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
    counts_per_rev: f64,
    /// Last integer position (counts); C `motor_info->position`.
    prev_position: i32,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl FaulhaberAxis {
    /// Construct axis `node` (0-based) and run the C `motor_init` per-axis setup:
    /// RS-232 velocity source (`SOR 0`), limit-switch and homing configuration,
    /// and program the encoder resolution (`ENCRES`). Performs blocking serial
    /// I/O.
    pub fn new(
        controller: Arc<Mutex<FaulhaberController>>,
        node: usize,
        counts_per_rev: f64,
    ) -> AsynResult<Self> {
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Velocity control source = RS-232.
            ctrl.write_cmd(node, "SOR 0")?;
            // Limit-switch / homing configuration (C motor_init sequence).
            ctrl.write_cmd(node, "REFIN")?;
            ctrl.write_cmd(node, "HP7")?;
            ctrl.write_cmd(node, "HB6")?;
            ctrl.write_cmd(node, "HD2")?;
            ctrl.write_cmd(node, "HL1")?;
            ctrl.write_cmd(node, "HA1")?;
            ctrl.write_cmd(node, "CAHOSEQ")?;
            // Program the encoder resolution once (C sends this before every
            // velocity command using mr->srev).
            ctrl.write_cmd(node, &format!("ENCRES{}", nint(counts_per_rev)))?;
        }
        Ok(Self {
            controller,
            node,
            counts_per_rev,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, FaulhaberController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program acceleration then speed for an upcoming move, if supplied.
    fn program_motion(
        &self,
        ctrl: &FaulhaberController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if acceleration > 0.0 {
            ctrl.write_cmd(
                self.node,
                &format!("AC {}", accel_to_rev(acceleration, self.counts_per_rev)),
            )?;
        }
        if velocity > 0.0 {
            ctrl.write_cmd(
                self.node,
                &format!("SP {}", vel_to_rpm(velocity, self.counts_per_rev)),
            )?;
        }
        Ok(())
    }

    /// C `MOVE_ABS`/`MOVE_REL`: PREM and a blank lead the move's own
    /// message, and its POST waits for the move to end.
    fn load_target(&mut self, command: &str) -> AsynResult<()> {
        let mut line = self.strings.arm();
        if !line.is_empty() {
            line.push(b' ');
        }
        line.extend_from_slice(command.as_bytes());
        self.lock().write_bytes(self.node, &line)
    }
}

impl AsynMotor for FaulhaberAxis {
    /// C `set_status`'s COMM_ERR branch raises `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` (drvMCDC2805.cc). Its first failure's silent RETRY is not modelled:
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
        acceleration: f64,
    ) -> AsynResult<()> {
        self.program_motion(&self.lock(), velocity, acceleration)?;
        self.load_target(&format!("LA {}", nint(position)))?;
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
        self.program_motion(&self.lock(), velocity, acceleration)?;
        self.load_target(&format!("LR {}", nint(distance)))?;
        self.lock().write_cmd(self.node, "M")
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG: set speed, then enter velocity mode with the signed rate.
        let rpm = vel_to_rpm(velocity, self.counts_per_rev);
        self.strings.split();
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, &format!("SP {}", rpm.abs()))?;
        ctrl.write_cmd(self.node, &format!("V {rpm}"))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C HOME_*: set the homing speed (signed by direction) and start the
        // controller's homing sequence.
        let mut rpm = vel_to_rpm(velocity, self.counts_per_rev);
        if !forward {
            rpm = -rpm;
        }
        self.strings.split();
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, &format!("HOSP{rpm}"))?;
        ctrl.write_cmd(self.node, "GOHOSEQ")
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // C drvMCDC2805.cc:229-251: an empty or failed GST keeps the previous
        // status once (RETRY); a second in a row raises COMM_ERR. Nothing else
        // is sent on that poll.
        let gst = ctrl.query(self.node, "GST").ok().filter(|r| !r.is_empty());
        let Some(status_buff) = gst else {
            let hard = ctrl.comm_state != CommState::Normal;
            ctrl.comm_state = if hard {
                CommState::CommErr
            } else {
                CommState::Retry
            };
            drop(ctrl);
            if hard {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return Ok(self.last_status.clone());
        };
        ctrl.comm_state = CommState::Normal;
        let pos = ctrl.query(self.node, "POS");
        let gast = ctrl.query(self.node, "GAST");
        let gv = ctrl.query(self.node, "GV");

        let status_bytes = status_buff.as_bytes();
        let home = status_bytes.get(6) == Some(&b'1');

        let motor_data = pos.as_deref().map(atof).unwrap_or(0.0);
        let new_position = nint(motor_data);
        let direction = if new_position != self.prev_position {
            new_position >= self.prev_position
        } else {
            self.last_status.direction
        };
        let plusdir = direction;

        // Limit switches (GAST: byte 0 = plus, byte 1 = minus).
        let gast_bytes = gast.unwrap_or_default();
        let gast_bytes = gast_bytes.as_bytes();
        let plus_ls = gast_bytes.first() == Some(&b'1');
        let minus_ls = gast_bytes.get(1) == Some(&b'1');
        let ls_active = (plus_ls && plusdir) || (minus_ls && !plusdir);

        // If an active limit switch stopped motion, hold position with an
        // absolute move to the current position (C set_status).
        if ls_active {
            ctrl.write_cmd(self.node, &format!("LA {}", nint(motor_data)))?;
            ctrl.write_cmd(self.node, "M")?;
        }

        // Actual velocity → done (reliable across jog and velocity=0 stops).
        let velocity = gv.as_deref().map(|s| nint(atof(s))).unwrap_or(0);
        let done = velocity == 0;
        if (done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            ctrl.write_bytes(self.node, &post)?;
        }
        drop(ctrl);

        self.prev_position = new_position;
        let signed_velocity = if plusdir { velocity } else { -velocity };

        self.last_status = MotorStatus {
            position: motor_data,
            encoder_position: motor_data,
            velocity: signed_velocity as f64,
            direction,
            done,
            moving: !done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            home,
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
    use std::sync::atomic::{AtomicBool, Ordering};

    struct Rig {
        line: LineRig,
        axis: FaulhaberAxis,
        moving: Arc<AtomicBool>,
        minus: Arc<AtomicBool>,
        user: AsynUser,
    }

    impl Rig {
        /// Node 1 at 1000 counts/rev, its setup traffic taken.
        fn new(name: &str) -> Self {
            let moving = Arc::new(AtomicBool::new(false));
            let minus = Arc::new(AtomicBool::new(false));
            let (m, l) = (moving.clone(), minus.clone());
            let (line, io) =
                LineRig::new(name, move |last| match last.get(1..).unwrap_or_default() {
                    "VER" => "MCDC2805".into(),
                    "GST" => "0000000".into(),
                    "POS" => "0".into(),
                    "GAST" => if l.load(Ordering::SeqCst) { "01" } else { "00" }.into(),
                    "GV" => if m.load(Ordering::SeqCst) { "-60" } else { "0" }.into(),
                    _ => String::new(),
                });
            let controller = Arc::new(Mutex::new(FaulhaberController::new(io, 2).unwrap()));
            let axis = FaulhaberAxis::new(controller, 1, 1000.0).unwrap();
            line.sent();
            Self {
                line,
                axis,
                moving,
                minus,
                user: AsynUser::new(0),
            }
        }

        /// What one poll sent besides its queries.
        fn poll_extra(&mut self) -> Vec<String> {
            self.axis.poll(&self.user).unwrap();
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
        assert_eq!(r.line.sent(), ["1EN LA 100", "1M"]);
        r.axis.move_relative(&r.user, -5.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1LR -5", "1M"]);
        r.axis.set_move_strings(&r.user, b"EN", None).unwrap();
        r.axis.move_relative(&r.user, -5.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1EN LR -5", "1M"]);
    }

    /// devMCDC2805.cc:222-236 and 286-300: HOME and JOG strcpy their first
    /// message over PREM, and their last message replaces the motion node
    /// that held POST.
    #[test]
    fn split_moves_send_neither_prem_nor_post() {
        let mut r = Rig::new("mcdc_split");
        r.axis
            .set_move_strings(&r.user, b"EN", Some(post(b"DI")))
            .unwrap();
        r.axis.home(&r.user, 0.0, 1000.0, 0.0, false).unwrap();
        assert_eq!(r.line.sent(), ["1HOSP-60", "1GOHOSEQ"]);
        assert!(r.poll_extra().is_empty());
        r.axis
            .set_move_strings(&r.user, b"EN", Some(post(b"DI")))
            .unwrap();
        r.axis.move_velocity(&r.user, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1SP 60", "1V 60"]);
        assert!(r.poll_extra().is_empty());
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
}
