//! The model-1 MDrive driver (`drvMDrive.cc` + `devMDrive.cc`): records
//! reach drive `signal` of chain `card` through `DTYP "MDrive"`.
//!
//! Every command is prefixed by the drive's party-mode name, `1` to `8`.
//! Commands get no reply; `PR` queries get one line.
//!
//! The record's INIT goes out once at init, behind the name and a blank;
//! PREM and a blank lead a move's own command, and POST goes once that move
//! is done or runs into a limit (devMDrive.cc:187-207, drvMDrive.cc:399-406).

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{Model1Motor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_printf;

use motor_common::model1::{MoveStrings, init_poll, init_read};
use motor_common::util::{atof, atoi, nint};

/// `MDrive_NUM_CARDS`.
pub const MDRIVE_NUM_CARDS: i32 = 8;
/// `MDrive_axis[]`.
const MDRIVE_AXIS: [&str; 8] = ["1", "2", "3", "4", "5", "6", "7", "8"];
/// `BUFF_SIZE`.
const BUFF_SIZE: usize = 80;

/// C `CommStatus`, kept per chain.
#[derive(Clone, Copy, PartialEq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// One MDrive chain: its serial link and communication state.
pub struct MDriveController {
    handle: SyncIOHandle,
    comm: CommState,
}

impl MDriveController {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self {
            handle,
            comm: CommState::Normal,
        }
    }

    /// `send_mess` then `recv_mess`: the reply, empty when none came.
    fn ask(&self, msg: &str) -> String {
        // write_read flushes first; C flushes only at set_status
        // (drvMDrive.cc:259), so a late reply shifts its answers until the
        // next poll: a C defect, not copied.
        match self
            .handle
            .write_read(0, msg.as_bytes(), BUFF_SIZE)
            .into_parts()
        {
            (raw, Ok(())) => String::from_utf8_lossy(&raw).into_owned(),
            (_, Err(_)) => String::new(),
        }
    }

    /// `recv_mess` alone: the next line, empty when none came.
    fn read(&self) -> String {
        match self.handle.read_octet(0, BUFF_SIZE).into_parts() {
            (raw, Ok(())) => String::from_utf8_lossy(&raw).into_owned(),
            (_, Err(_)) => String::new(),
        }
    }

    fn send(&self, msg: impl AsRef<[u8]>) {
        let _ = self.handle.write_octet(0, msg.as_ref());
    }
}

/// The discrete input wired to each switch, 0 when none (`input_config`).
#[derive(Clone, Copy, Default)]
struct Inputs {
    plus: u8,
    minus: u8,
    home: u8,
}

/// What `motor_init` learns of one drive.
struct MDriveDrive {
    encoder: bool,
    inputs: Inputs,
}

/// `motor_init`'s probe: `PR VR` to each drive in turn, three tries each,
/// until one does not answer. C's `ident`, the first drive's `PR VR`, and
/// the number that answered.
pub fn mdrive_probe(ctrl: &MDriveController) -> (String, usize) {
    let mut ident = String::new();
    let mut found = 0;
    for name in MDRIVE_AXIS {
        let reply = (0..3)
            .map(|_| ctrl.ask(&format!("{name}PR VR")))
            .find(|r| !r.is_empty());
        let Some(reply) = reply else { break };
        if found == 0 {
            ident = reply;
        }
        found += 1;
    }
    (ident, found)
}

/// Drive `index`'s encoder and switch inputs, as `motor_init` reads them
/// ahead of its `set_status` (drvMDrive.cc:645-722).
fn probe_drive(ctrl: &MDriveController, card: i32, index: usize, ident: &str) -> MDriveDrive {
    let name = MDRIVE_AXIS[index];
    let encoder = ident.ends_with('E') || {
        let part = ctrl.ask(&format!("{name}PR PN"));
        ["E", "EQ", "EE", "EJM", "EJM-N"]
            .iter()
            .any(|suffix| part.ends_with(suffix))
    };
    // C reads one more line here, waiting out its timeout when the drive
    // sends none.
    ctrl.read();
    let inputs = read_inputs(ctrl, name);
    if inputs.minus == 0 || inputs.plus == 0 {
        let which = if inputs.minus == 0 { "Minus" } else { "Plus" };
        errlog_printf(&format!(
            "MDrive chain #{card}, motor #{index} {which} LS not configured.\n"
        ));
    }
    MDriveDrive { encoder, inputs }
}

/// `PR S1` .. `PR S4`: which input is the home (1), plus (2) or minus (3)
/// switch. C fills one table for the whole chain, so every drive polls the
/// last drive's inputs; each drive keeps its own here.
fn read_inputs(ctrl: &MDriveController, name: &str) -> Inputs {
    let mut inputs = Inputs::default();
    for input in 1..=4u8 {
        let reply = ctrl.ask(&format!("{name}PR S{input}"));
        if reply.is_empty() {
            errlog_printf("Error reading I/O configuration.\n");
            break;
        }
        match atoi(&reply) {
            0 => {}
            1 => inputs.home = input,
            2 => inputs.plus = input,
            3 => inputs.minus = input,
            other => errlog_printf(&format!("Invalid I/O type: {other}.\n")),
        }
    }
    inputs
}

/// One drive of a model-1 MDrive chain.
pub struct DrvMDriveAxis {
    controller: Arc<Mutex<MDriveController>>,
    name: &'static str,
    inputs: Inputs,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed `PR MV` (RETRY, drvMDrive.cc:272), `CNTRL_COMM_ERR` and `RA_PROBLEM` on
    /// the next (COMM_ERR, :278). Per axis, as the controller's state can
    /// move on under another axis's poll.
    failure: PollFailure,
    /// `motor_info->status`, which `set_status` starts from.
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
    /// MRES/ERES, from `SET_ENC_RATIO`: `LOAD_POS` loads the encoder with
    /// DVAL/ERES.
    encoder_ratio: f64,
    strings: MoveStrings,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl DrvMDriveAxis {
    /// Drive `signal` of chain `card`, whose first drive answered `ident`:
    /// its encoder and inputs, then its status read (drvMDrive.cc:725).
    pub fn new(
        controller: Arc<Mutex<MDriveController>>,
        card: i32,
        signal: usize,
        ident: &str,
    ) -> Self {
        let drive = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            probe_drive(&ctrl, card, signal, ident)
        };
        let mut axis = Self::with_drive(controller, signal, drive);
        axis.init_status = Some(init_poll(&mut axis, &AsynUser::new(0)));
        axis
    }

    fn with_drive(
        controller: Arc<Mutex<MDriveController>>,
        signal: usize,
        drive: MDriveDrive,
    ) -> Self {
        Self {
            controller,
            name: MDRIVE_AXIS[signal],
            inputs: drive.inputs,
            failure: PollFailure::NONE,
            status: MotorStatus {
                has_encoder: drive.encoder,
                gain_support: drive.encoder,
                ..MotorStatus::default()
            },
            position: 0,
            encoder_ratio: 1.0,
            strings: MoveStrings::default(),
            init_status: None,
        }
    }

    fn lock(&self) -> MutexGuard<'_, MDriveController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `send_mess`: `name`, then `prem` and a blank when there is one, then
    /// `command`.
    fn line(&self, prem: &[u8], command: &str) -> Vec<u8> {
        let mut line = self.name.as_bytes().to_vec();
        if !prem.is_empty() {
            line.extend_from_slice(prem);
            line.push(b' ');
        }
        line.extend_from_slice(command.as_bytes());
        line
    }

    /// Send each command of a transaction.
    fn send(&self, commands: &[String]) -> AsynResult<()> {
        let ctrl = self.lock();
        for command in commands {
            ctrl.send(self.line(b"", command));
        }
        Ok(())
    }

    /// A move, home or jog: its PREM leads the last command, the motion
    /// itself, and its POST becomes the one `poll` sends.
    fn send_move(&mut self, commands: &[String]) -> AsynResult<()> {
        let prem = self.strings.arm();
        let Some((last, rest)) = commands.split_last() else {
            return Ok(());
        };
        let ctrl = self.lock();
        for command in rest {
            ctrl.send(self.line(b"", command));
        }
        ctrl.send(self.line(&prem, last));
        Ok(())
    }

    /// A motion command, led by its PREM.
    fn motion(&mut self, motion: String) -> AsynResult<()> {
        self.send_move(&[motion])
    }
}

impl Model1Motor for DrvMDriveAxis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.status.clone(), self.poll_failure())
    }

    /// `update_delay` is 0 (`targs`, drvMDrive.cc:156).
    fn update_delay(&self) -> Duration {
        Duration::ZERO
    }

    fn set_velocity(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        self.send(&[format!("VM={}", nint(velocity))])
    }

    fn set_vel_base(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        self.send(&[format!("VI={}", nint(velocity))])
    }

    /// SET_ACCEL: `A=` then `D=A`.
    fn set_accel(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        self.send(&[format!("A={}", nint(acceleration)), "D=A".to_string()])
    }

    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.motion(format!("MA {}", nint(position)))
    }

    fn move_relative(&mut self, _user: &AsynUser, distance: f64) -> AsynResult<()> {
        self.motion(format!("MR {}", nint(distance)))
    }

    fn move_velocity(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        self.motion(format!("SL={}", nint(velocity)))
    }

    /// devMDrive.cc sends the IM483's home command, ` F1000 0|1`, as is.
    fn home(&mut self, _user: &AsynUser, forward: bool) -> AsynResult<()> {
        let way = if forward { 0 } else { 1 };
        self.motion(format!(" F1000 {way}"))
    }

    fn stop(&mut self, _user: &AsynUser) -> AsynResult<()> {
        self.send(&["SL 0".to_string()])
    }

    /// `LOAD_POS`: `P=`, then `C2=` for the encoder when there is one.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let mut commands = vec![format!("P={}", nint(position))];
        if self.status.has_encoder {
            commands.push(format!("C2={}", nint(position * self.encoder_ratio)));
        }
        self.send(&commands)
    }

    fn set_encoder_ratio(&mut self, _user: &AsynUser, ratio: f64) -> AsynResult<()> {
        self.encoder_ratio = ratio;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.send(&[format!("DE={}", i32::from(enable))])
    }

    /// `PRIMITIVE`: INIT behind the name and a blank.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut line = format!("{} ", self.name).into_bytes();
        line.extend_from_slice(init);
        self.lock().send(line);
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

    /// `set_status`: `PR MV` for moving, `PR P` for the position, `PR I<n>`
    /// for each wired switch, `PR C2` for the encoder and `PR V` for the
    /// velocity. A first unanswered `PR MV` fails the poll raising nothing; a
    /// second raises the comms error.
    fn poll(&mut self, _user: &AsynUser, out: &mut MotorStatus) -> AsynResult<()> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let name = self.name;
        let ask = |ctrl: &MDriveController, q: &str| ctrl.ask(&format!("{name}{q}"));
        let moving = ask(&ctrl, "PR MV");
        if moving.is_empty() {
            (ctrl.comm, self.failure) = match ctrl.comm {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("MDrive {name}: no PR MV reply"),
            });
        }
        ctrl.comm = CommState::Normal;
        let position = ask(&ctrl, "PR P");
        let input = |n: u8| atoi(ask(&ctrl, &format!("PR I{n}"))) as u8 != 0;
        let limits = (self.inputs.plus != 0 && self.inputs.minus != 0)
            .then(|| (input(self.inputs.plus), input(self.inputs.minus)));
        let home = self.inputs.home != 0 && input(self.inputs.home);
        let encoder = self.status.has_encoder.then(|| ask(&ctrl, "PR C2"));
        let velocity = ask(&ctrl, "PR V");
        drop(ctrl);

        let status = &mut self.status;
        status.comms_error = false;
        status.done = atoi(&moving) == 0;
        let data = atof(&position);
        if data != f64::from(self.position) {
            let new = nint(data);
            status.direction = new >= self.position;
            self.position = new;
        }
        status.position = f64::from(self.position);
        (status.high_limit, status.low_limit) = limits.unwrap_or_default();
        status.home = home;
        status.encoder_position = encoder.map_or(0.0, |e| atof(&e).trunc());
        status.problem = false;
        let speed = atof(&velocity).trunc();
        status.velocity = if status.direction { speed } else { -speed };
        let status = status.clone();
        let ls_active =
            (status.high_limit && status.direction) || (status.low_limit && !status.direction);
        if (status.done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let mut line = self.name.as_bytes().to_vec();
            line.extend_from_slice(&post);
            self.lock().send(line);
        }
        *out = status;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use motor_common::testing::Model1Polled;
    use motor_common::testing::RecordWrites;
    use std::sync::atomic::{AtomicBool, Ordering};

    struct Rig {
        line: LineRig,
        axis: DrvMDriveAxis,
        moving: Arc<AtomicBool>,
        minus: Arc<AtomicBool>,
        user: AsynUser,
    }

    impl Rig {
        /// Drive `1` with its minus limit on input 2 and plus on input 1.
        fn new(name: &str) -> Self {
            let moving = Arc::new(AtomicBool::new(false));
            let minus = Arc::new(AtomicBool::new(false));
            let (m, l) = (moving.clone(), minus.clone());
            let (line, io) = LineRig::new(name, move |last| {
                let on = if last.ends_with("PR MV") {
                    m.load(Ordering::SeqCst)
                } else if last.ends_with("PR I2") {
                    l.load(Ordering::SeqCst)
                } else {
                    false
                };
                u8::from(on).to_string()
            });
            let controller = Arc::new(Mutex::new(MDriveController::new(io)));
            let drive = MDriveDrive {
                encoder: false,
                inputs: Inputs {
                    plus: 1,
                    minus: 2,
                    home: 0,
                },
            };
            Self {
                line,
                axis: DrvMDriveAxis::with_drive(controller, 0, drive),
                moving,
                minus,
                user: AsynUser::new(0),
            }
        }

        fn move_to(&mut self, position: f64) {
            self.axis
                .record_move_absolute(&self.user, position, 1.0, 2.0, 0.0)
                .unwrap();
        }

        /// What one poll sent besides its queries.
        fn poll_extra(&mut self) -> Vec<String> {
            self.axis.polled(&self.user).unwrap();
            self.line
                .sent()
                .into_iter()
                .filter(|l| !l.starts_with("1PR "))
                .collect()
        }
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// devMDrive.cc:187-191.
    #[test]
    fn init_goes_out_behind_the_name_and_a_blank() {
        let mut r = Rig::new("mdrive_init");
        r.axis.primitive(&r.user, b"EE=1").unwrap();
        assert_eq!(r.line.sent(), ["1 EE=1"]);
    }

    /// devMDrive.cc:195-204: PREM and a blank lead the move's own message,
    /// for that move only.
    #[test]
    fn prem_leads_the_move_message() {
        let mut r = Rig::new("mdrive_prem");
        r.axis.set_move_strings(&r.user, b"DE=1", None).unwrap();
        r.move_to(100.0);
        assert_eq!(r.line.sent(), ["1VM=2", "1VI=1", "1DE=1 MA 100"]);
        r.move_to(50.0);
        assert_eq!(r.line.sent(), ["1VM=2", "1VI=1", "1MA 50"]);
        r.axis.set_move_strings(&r.user, b"DE=1", None).unwrap();
        r.axis.record_home(&r.user, 1.0, 2.0, 0.0, true).unwrap();
        assert_eq!(r.line.sent(), ["1VM=2", "1VI=1", "1DE=1  F1000 0"]);
        r.axis.set_move_strings(&r.user, b"DE=1", None).unwrap();
        r.axis.record_jog(&r.user, 1.0, 3.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1VI=1", "1A=0", "1D=A", "1DE=1 SL=3"]);
    }

    /// motorRecord.cc:2141, :3068: a jog's SET_ACCEL goes out at any rate,
    /// a move's only at a positive one (:874).
    #[test]
    fn a_jog_sends_set_accel_at_a_zero_rate() {
        let mut r = Rig::new("mdrive_jog_accel");
        r.axis.record_jog(&r.user, 1.0, 3.0, 0.0).unwrap();
        assert_eq!(r.line.sent(), ["1VI=1", "1A=0", "1D=A", "1SL=3"]);
        r.move_to(100.0);
        assert_eq!(r.line.sent(), ["1VM=2", "1VI=1", "1MA 100"]);
    }

    /// drvMDrive.cc:399-406: POST once the move is done, not while it runs.
    #[test]
    fn post_follows_the_move_once_done() {
        let mut r = Rig::new("mdrive_post");
        r.axis
            .set_move_strings(&r.user, b"", Some(post(b"DE=0")))
            .unwrap();
        r.move_to(100.0);
        r.line.sent();
        r.moving.store(true, Ordering::SeqCst);
        assert!(r.poll_extra().is_empty());
        r.moving.store(false, Ordering::SeqCst);
        assert_eq!(r.poll_extra(), ["1DE=0"]);
        assert!(r.poll_extra().is_empty(), "sent once");
    }

    /// drvMDrive.cc:400: a limit in the move's direction ends it too.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let mut r = Rig::new("mdrive_post_ls");
        r.axis
            .set_move_strings(&r.user, b"", Some(post(b"DE=0")))
            .unwrap();
        r.move_to(-100.0);
        r.line.sent();
        r.moving.store(true, Ordering::SeqCst);
        r.minus.store(true, Ordering::SeqCst);
        assert_eq!(r.poll_extra(), ["1DE=0"]);
    }

    /// drvMDrive.cc:639-725: a failed `motor_init` read stores the
    /// GAIN_SUPPORT and EA_PRESENT `motor_init` sets on a drive with an
    /// encoder, the first raising nothing (RETRY) and the next
    /// CNTRL_COMM_ERR and RA_PROBLEM (:270-282).
    #[test]
    fn a_failed_init_read_keeps_what_motor_init_set() {
        let (_line, io) = LineRig::new("mdriveInitFail", |_| String::new());
        let controller = Arc::new(Mutex::new(MDriveController::new(io)));
        let init = |signal, ident| {
            DrvMDriveAxis::new(controller.clone(), 0, signal, ident).take_init_status()
        };
        let prepared = MotorStatus {
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        assert_eq!(init(0, "MDI1E"), prepared);
        assert_eq!(
            init(1, "MDI1E"),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..prepared
            }
        );
        assert_eq!(
            init(2, "MDI1"),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..MotorStatus::default()
            }
        );
    }

    /// drvMDrive.cc:263-283: a first unanswered `PR MV` is RETRY (nothing
    /// raised, nothing posted), the next COMM_ERR, and a reply clears it.
    #[test]
    fn an_unanswered_moving_query_is_retry_then_comm_error() {
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("mdrive_retry", move |_| {
            if q.load(Ordering::SeqCst) {
                String::new()
            } else {
                "0".into()
            }
        });
        let controller = Arc::new(Mutex::new(MDriveController::new(io)));
        let drive = MDriveDrive {
            encoder: false,
            inputs: Inputs {
                plus: 0,
                minus: 0,
                home: 0,
            },
        };
        let mut axis = DrvMDriveAxis::with_drive(controller, 0, drive);
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
}
