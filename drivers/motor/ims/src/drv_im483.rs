//! The model-1 IM483 drivers (`drvIM483PL.cc` + `devIM483PL.cc`,
//! `drvIM483SM.cc` + `devIM483SM.cc`): records reach axis `signal` of `card`
//! through `DTYP "IM483PL"` or `"IM483SM"`.
//!
//! The party-line IM483PL prefixes every command with the axis name and a
//! space; the single-axis IM483SM sends it bare. Each command is answered by
//! one line, which is read and dropped.
//!
//! The record's INIT, PREM and POST go out as the C devices build them:
//! INIT once at init, PREM and `;` ahead of a move's last command, POST once
//! that move is done or runs into a limit.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::model1::MoveStrings;
use motor_common::util::{atof, atoi, nint};

/// `IM483PL_NUM_CARDS`.
pub const IM483PL_NUM_CARDS: i32 = 8;
/// `MAX_AXES` of drvIM483PL.cc.
pub const IM483PL_MAX_AXES: usize = 8;
/// `IM483PL_axis[]`.
const IM483PL_AXIS: [&str; IM483PL_MAX_AXES] = ["A", "B", "C", "D", "E", "F", "G", "H"];

/// `IM483SM_NUM_CARDS`.
pub const IM483SM_NUM_CARDS: i32 = 8;

/// `BUFF_SIZE`.
const BUFF_SIZE: usize = 50;

/// C `CommStatus`, kept per card.
#[derive(Clone, Copy, PartialEq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// One IM483 card: its serial link and communication state.
pub struct Im483Controller {
    handle: SyncIOHandle,
    comm: CommState,
}

impl Im483Controller {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self {
            handle,
            comm: CommState::Normal,
        }
    }

    /// `send_mess` then `recv_mess`: the reply, empty when none came.
    fn ask(&self, msg: impl AsRef<[u8]>) -> String {
        self.handle
            .write_read(0, msg.as_ref(), BUFF_SIZE)
            .map(|raw| String::from_utf8_lossy(&raw).into_owned())
            .unwrap_or_default()
    }
}

/// `motor_init`'s probe of an IM483PL: ` Z 0` to each axis in turn, three
/// tries each, until one does not answer. The number that did.
pub fn im483pl_probe(ctrl: &Im483Controller) -> usize {
    IM483PL_AXIS
        .iter()
        .take_while(|name| (0..3).any(|_| !ctrl.ask(format!("{name} Z 0")).is_empty()))
        .count()
}

/// `motor_init`'s probe of an IM483SM, three tries: reset with ETX, wait a
/// second, send a space and read the sign-on. Its identification when it
/// answered, which C keeps only for `dbior`.
pub fn im483sm_probe(ctrl: &Im483Controller) -> Option<String> {
    (0..3).find_map(|_| {
        let _ = ctrl.handle.write_octet(0, b"\x03");
        sleep(Duration::from_secs(1));
        let mut first = Some(ctrl.ask(" "));
        sign_on_ident(|| {
            let line = match first.take() {
                Some(line) => line.into_bytes(),
                None => ctrl.handle.read_octet(0, BUFF_SIZE).unwrap_or_default(),
            };
            Some(line).filter(|l| !l.is_empty())
        })
    })
}

/// C's copy of the sign-on into `brdptr->ident` (drvIM483SM.cc:557-598):
/// leading blanks of a line dropped, a run of blanks kept as one, lines
/// joined by a blank, an `AD` copyright line's 22 characters as `AMS`, 50
/// steps at most. `next` is C's `recv_mess`. `None` when the first read
/// gets nothing. C reads past the line's end into what an earlier, longer
/// line left in the buffer and, after 50 steps, leaves `ident`
/// unterminated; both are defects not copied, past the end reads as the
/// line's NUL.
fn sign_on_ident(mut next: impl FnMut() -> Option<Vec<u8>>) -> Option<String> {
    let at = |buff: &[u8], i: usize| buff.get(i).copied().unwrap_or(0);
    let mut buff = Vec::new();
    let mut src = 0;
    let mut ident = Vec::new();
    let mut itera = 0;
    while itera < 50 {
        if at(&buff, src) == 0 {
            let Some(line) = next() else {
                if itera == 0 {
                    return None;
                }
                break;
            };
            buff = line;
            src = 0;
            while at(&buff, src).is_ascii_whitespace() {
                src += 1;
            }
            if itera != 0 {
                ident.push(b' ');
                itera += 1;
            }
        } else if at(&buff, src).is_ascii_whitespace() {
            while at(&buff, src).is_ascii_whitespace() {
                src += 1;
            }
            src -= 1;
        } else if buff[src..].starts_with(b"AD") {
            ident.extend_from_slice(b"AMS");
            src += 22;
            itera += 3;
        }
        ident.push(at(&buff, src));
        src += 1;
        itera += 1;
    }
    let end = ident.iter().position(|&b| b == 0).unwrap_or(ident.len());
    Some(String::from_utf8_lossy(&ident[..end]).into_owned())
}

/// One axis of a model-1 IM483 card.
pub struct DrvIm483Axis {
    controller: Arc<Mutex<Im483Controller>>,
    /// `IM483PL_axis[signal]`, or nothing on an IM483SM: `send_mess`
    /// prefixes it to every message.
    name: &'static str,
    /// The blank an IM483PL command starts with, which PREM goes ahead of.
    sep: &'static str,
    /// What PRIMITIVE puts ahead of INIT: `"? "` on an IM483PL.
    init_prefix: &'static str,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed `^` (RETRY, drvIM483PL.cc:254, drvIM483SM.cc:252),
    /// `CNTRL_COMM_ERR` and `RA_PROBLEM` on the next (COMM_ERR, :260,
    /// :258). Per axis, as the controller's state can move on under another
    /// axis's poll.
    failure: PollFailure,
    /// `motor_info->status`, which `set_status` starts from.
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
    strings: MoveStrings,
}

impl DrvIm483Axis {
    /// Axis `signal` of an IM483PL, which C takes to carry an encoder.
    pub fn im483pl(controller: Arc<Mutex<Im483Controller>>, signal: usize) -> Self {
        Self {
            controller,
            name: IM483PL_AXIS[signal],
            sep: " ",
            init_prefix: "? ",
            failure: PollFailure::NONE,
            status: MotorStatus {
                has_encoder: true,
                ..MotorStatus::default()
            },
            position: 0,
            strings: MoveStrings::default(),
        }
    }

    /// The one axis of an IM483SM, which C takes to have no encoder.
    pub fn im483sm(controller: Arc<Mutex<Im483Controller>>) -> Self {
        Self {
            controller,
            name: "",
            sep: "",
            init_prefix: "",
            failure: PollFailure::NONE,
            status: MotorStatus::default(),
            position: 0,
            strings: MoveStrings::default(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Im483Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `name`, `prem;` when there is one, and `command`: the message C
    /// builds for `command` (devIM483PL.cc:191-205).
    fn line(&self, prem: &[u8], command: &str) -> Vec<u8> {
        let mut line = self.name.as_bytes().to_vec();
        if !prem.is_empty() {
            line.extend_from_slice(prem);
            line.push(b';');
        }
        line.extend_from_slice(self.sep.as_bytes());
        line.extend_from_slice(command.as_bytes());
        line
    }

    /// Send each command of a transaction, reading its reply.
    fn send(&self, commands: &[String]) -> AsynResult<()> {
        let ctrl = self.lock();
        for command in commands {
            ctrl.ask(self.line(b"", command));
        }
        Ok(())
    }

    /// A move, home or jog: its PREM rides ahead of the last command, and
    /// its POST, or none, becomes the one `poll` sends.
    fn send_move(&mut self, commands: &[String]) -> AsynResult<()> {
        let prem = self.strings.arm();
        let Some((last, rest)) = commands.split_last() else {
            return Ok(());
        };
        let ctrl = self.lock();
        for command in rest {
            ctrl.ask(self.line(b"", command));
        }
        ctrl.ask(self.line(&prem, last));
        Ok(())
    }

    /// The record's `SET_VELOCITY` and `SET_VEL_BASE`; `SET_ACCEL` sends
    /// nothing.
    fn speeds(min_velocity: f64, velocity: f64) -> [String; 2] {
        [
            format!("V{:.2};", CFloat(velocity)),
            format!("I{:.2};", CFloat(min_velocity)),
        ]
    }
}

impl AsynMotor for DrvIm483Axis {
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let [v, i] = Self::speeds(min_velocity, velocity);
        self.send_move(&[v, i, format!("R{:.2}", CFloat(position))])
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let [v, i] = Self::speeds(min_velocity, velocity);
        self.send_move(&[v, i, format!("{:+.2}", CFloat(distance))])
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.send_move(&[
            format!("I{:.2};", CFloat(min_velocity)),
            format!("M{:.2};", CFloat(velocity)),
        ])
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let [v, i] = Self::speeds(min_velocity, velocity);
        let way = if forward { 0 } else { 1 };
        self.send_move(&[v, i, format!("F1000 {way}")])
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.send(&["@ 0".to_string()])
    }

    /// `LOAD_POS`: only zero can be loaded, with `O`.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if position != 0.0 {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "IM483 loads position 0 only".into(),
            });
        }
        self.send(&["O".to_string()])
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let command = if enable { "MO;" } else { "MF;" };
        self.send(&[command.to_string()])
    }

    /// `PRIMITIVE`: INIT behind the axis name, with `"? "` on an IM483PL.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut line = format!("{}{}", self.name, self.init_prefix).into_bytes();
        line.extend_from_slice(init);
        self.lock().ask(line);
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

    /// `set_status`: `^` for moving, `Z 0` for the position, `] 0` for the
    /// limits, `] 1` for home, `z 0` for the encoder. A first unanswered `^`
    /// fails the poll raising nothing; a second raises the comms error.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let ask = |ctrl: &Im483Controller, q: &str| ctrl.ask(self.line(b"", q));
        let moving = ask(&ctrl, "^");
        if moving.is_empty() {
            (ctrl.comm, self.failure) = match ctrl.comm {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "IM483: no ^ reply".into(),
            });
        }
        ctrl.comm = CommState::Normal;
        let position = ask(&ctrl, "Z 0");
        let limits = ask(&ctrl, "] 0");
        let home = ask(&ctrl, "] 1");
        let encoder = self.status.has_encoder.then(|| ask(&ctrl, "z 0"));
        drop(ctrl);

        let status = &mut self.status;
        status.comms_error = false;
        status.done = atoi(from(&moving, 4)) == 0;
        let data = atof(from(&position, 5));
        if data != f64::from(self.position) {
            let new = nint(data);
            status.direction = new >= self.position;
            self.position = new;
        }
        status.position = f64::from(self.position);
        let limits = atoi(from(&limits, 5));
        status.high_limit = limits & 1 != 0;
        status.low_limit = limits & 2 != 0;
        status.home = home.as_bytes().get(5).is_some_and(|b| b & 1 != 0);
        status.encoder_position = encoder.map_or(0.0, |e| atof(from(&e, 5)).trunc());
        status.problem = false;
        let status = status.clone();
        // The move's POST, as the field holds it now, once the move is done
        // or a limit stops it in its direction (drvIM483PL.cc:357-366); an
        // emptied one is not sent (`send_mess`'s empty-message exit). C sends
        // it without reading its reply, which the next `^` then reads as its
        // own and every later query is a line behind; the reply is read here.
        let ls_active =
            (status.high_limit && status.direction) || (status.low_limit && !status.direction);
        if (status.done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let mut line = self.name.as_bytes().to_vec();
            line.extend_from_slice(&post);
            self.lock().ask(line);
        }
        Ok(status)
    }
}

/// The reply from byte `i`, as C's `&buff[i]`; empty past its end.
fn from(reply: &str, i: usize) -> &str {
    reply.get(i..).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ident_of(lines: &[&str]) -> Option<String> {
        let mut lines = lines.iter();
        sign_on_ident(|| lines.next().map(|l| l.as_bytes().to_vec()))
    }

    /// drvIM483SM.cc:557-598 over a scripted sign-on.
    #[test]
    fn the_sign_on_is_copied_as_c_copies_it_into_ident() {
        assert_eq!(ident_of(&[]), None);
        assert_eq!(
            ident_of(&["  IM483  Ver  1.2", "xAD01234567890123456789QZ"]).unwrap(),
            "IM483 Ver 1.2 xAMSQZ"
        );
        let endless = sign_on_ident(|| Some(b"ab".to_vec())).unwrap();
        assert_eq!(endless, vec!["ab"; 17].join(" "));
    }
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

    /// An IM483 line: records each message and answers the last one, `^`
    /// with `moving` and `] 0` with `limits`.
    struct Line {
        base: PortDriverBase,
        sent: Arc<Mutex<Vec<String>>>,
        moving: Arc<AtomicBool>,
        limits: Arc<AtomicI32>,
    }

    impl PortDriver for Line {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let line = String::from_utf8_lossy(data).into_owned();
            self.sent.lock().unwrap().push(line);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let last = self
                .sent
                .lock()
                .unwrap()
                .last()
                .cloned()
                .unwrap_or_default();
            let reply = if last.ends_with('^') {
                format!("A ^ {}", u8::from(self.moving.load(Ordering::SeqCst)))
            } else if last.ends_with("] 0") {
                format!("A ] 0{}", self.limits.load(Ordering::SeqCst))
            } else {
                "A Z 00".to_string()
            };
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            Ok(reply.len())
        }
    }

    struct Rig {
        axis: DrvIm483Axis,
        sent: Arc<Mutex<Vec<String>>>,
        moving: Arc<AtomicBool>,
        limits: Arc<AtomicI32>,
        user: AsynUser,
    }

    impl Rig {
        fn new(name: &str, party_line: bool) -> Self {
            let sent = Arc::new(Mutex::new(Vec::new()));
            let moving = Arc::new(AtomicBool::new(false));
            let limits = Arc::new(AtomicI32::new(0));
            let line = Line {
                base: PortDriverBase::new(
                    name,
                    1,
                    PortFlags::default(),
                    InterfaceMask::UNMIGRATED_ALL,
                    InterfaceMask::UNMIGRATED_INTERRUPTS,
                ),
                sent: sent.clone(),
                moving: moving.clone(),
                limits: limits.clone(),
            };
            let (rt, _jh) =
                create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                    .unwrap();
            let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
            let controller = Arc::new(Mutex::new(Im483Controller::new(io)));
            let axis = if party_line {
                DrvIm483Axis::im483pl(controller, 0)
            } else {
                DrvIm483Axis::im483sm(controller)
            };
            Self {
                axis,
                sent,
                moving,
                limits,
                user: AsynUser::new(0),
            }
        }

        fn sent(&self) -> Vec<String> {
            std::mem::take(&mut *self.sent.lock().unwrap())
        }

        fn move_to(&mut self, position: f64) {
            self.axis
                .move_absolute(&self.user, position, 1.0, 2.0, 0.0)
                .unwrap();
        }

        /// The messages one poll sent besides its five queries.
        fn poll_extra(&mut self) -> Vec<String> {
            self.axis.poll(&self.user).unwrap();
            let queries = ["^", "Z 0", "] 0", "] 1", "z 0"];
            self.sent()
                .into_iter()
                .filter(|l| !queries.iter().any(|q| l.ends_with(q)))
                .collect()
        }
    }

    /// devIM483PL.cc:183-187 and devIM483SM.cc:183-184.
    #[test]
    fn init_goes_out_behind_the_axis_name() {
        let mut pl = Rig::new("im483_init_pl", true);
        pl.axis.primitive(&pl.user, b"PM 3").unwrap();
        assert_eq!(pl.sent(), ["A? PM 3"]);
        let mut sm = Rig::new("im483_init_sm", false);
        sm.axis.primitive(&sm.user, b"PM 3").unwrap();
        assert_eq!(sm.sent(), ["PM 3"]);
    }

    /// devIM483PL.cc:191-205: PREM and `;` lead the move's own message.
    #[test]
    fn prem_leads_the_move_message() {
        let mut pl = Rig::new("im483_prem_pl", true);
        pl.axis.set_move_strings(&pl.user, b"MO", None).unwrap();
        pl.move_to(10.0);
        assert_eq!(pl.sent(), ["A V2.00;", "A I1.00;", "AMO; R10.00"]);
        pl.move_to(11.0);
        assert_eq!(pl.sent(), ["A V2.00;", "A I1.00;", "A R11.00"]);
        let mut sm = Rig::new("im483_prem_sm", false);
        sm.axis.set_move_strings(&sm.user, b"MO", None).unwrap();
        sm.move_to(10.0);
        assert_eq!(sm.sent(), ["V2.00;", "I1.00;", "MO;R10.00"]);
    }

    /// devIM483PL.cc's `%.*f` and `%+.*f` of a non-finite step value.
    #[test]
    fn a_nan_target_goes_out_as_c_printf_spells_it() {
        let mut pl = Rig::new("im483_nan", true);
        pl.axis
            .move_absolute(&pl.user, f64::NAN, f64::INFINITY, 2.0, 0.0)
            .unwrap();
        pl.axis
            .move_relative(&pl.user, f64::NAN, 1.0, 2.0, 0.0)
            .unwrap();
        assert_eq!(
            pl.sent(),
            [
                "A V2.00;", "A Iinf;", "A Rnan", "A V2.00;", "A I1.00;", "A +nan"
            ]
        );
    }

    /// drvIM483PL.cc:361-366: POST once the move is done, not while it runs.
    #[test]
    fn post_follows_the_move_once_done() {
        let mut pl = Rig::new("im483_post_pl", true);
        pl.axis
            .set_move_strings(&pl.user, b"", Some(post(b" MF")))
            .unwrap();
        pl.move_to(10.0);
        pl.sent();
        pl.moving.store(true, Ordering::SeqCst);
        assert!(pl.poll_extra().is_empty());
        pl.moving.store(false, Ordering::SeqCst);
        assert_eq!(pl.poll_extra(), ["A MF"]);
        assert!(pl.poll_extra().is_empty());
    }

    /// A limit in the direction of travel ends the move as done does.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let mut pl = Rig::new("im483_post_ls", true);
        pl.axis
            .set_move_strings(&pl.user, b"", Some(post(b" MF")))
            .unwrap();
        pl.move_to(10.0);
        pl.sent();
        pl.moving.store(true, Ordering::SeqCst);
        pl.limits.store(2, Ordering::SeqCst);
        assert_eq!(pl.poll_extra(), ["A MF"]);
    }

    /// The POST field is read when the move is done (`postmsgptr =
    /// &mr->post`): a put during the move is what goes out, and an emptied
    /// one sends nothing.
    #[test]
    fn post_is_read_when_the_move_is_done() {
        let mut pl = Rig::new("im483_post_late", true);
        let field = post(b" MF");
        pl.axis
            .set_move_strings(&pl.user, b"", Some(field.clone()))
            .unwrap();
        pl.move_to(10.0);
        pl.sent();
        field.set(b" XY");
        assert_eq!(pl.poll_extra(), ["A XY"]);

        pl.axis
            .set_move_strings(&pl.user, b"", Some(field.clone()))
            .unwrap();
        pl.move_to(11.0);
        pl.sent();
        field.set(b"");
        assert!(pl.poll_extra().is_empty());
        field.set(b" MF");
        assert!(pl.poll_extra().is_empty(), "sent once, or not at all");
    }

    fn post(text: &[u8]) -> PostMessage {
        let field = PostMessage::default();
        field.set(text);
        field
    }

    /// A later move without POST replaces the motion C would post for.
    #[test]
    fn a_move_without_post_drops_the_last_ones() {
        let mut pl = Rig::new("im483_post_drop", true);
        pl.axis
            .set_move_strings(&pl.user, b"", Some(post(b" MF")))
            .unwrap();
        pl.move_to(10.0);
        pl.move_to(11.0);
        pl.sent();
        assert!(pl.poll_extra().is_empty());
    }

    #[test]
    fn reply_offset_past_end_reads_as_empty() {
        assert_eq!(from("A ^ 1", 4), "1");
        assert_eq!(from("A ^", 4), "");
    }

    /// drvIM483PL.cc:245-265: a first unanswered `^` is RETRY (nothing
    /// raised, nothing posted), the next COMM_ERR, and a reply clears it.
    #[test]
    fn an_unanswered_moving_query_is_retry_then_comm_error() {
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = motor_common::testing::LineRig::new("im483_retry", move |last| {
            if q.load(Ordering::SeqCst) {
                String::new()
            } else if last.ends_with('^') {
                "A ^ 0".into()
            } else {
                "A Z 00".into()
            }
        });
        let controller = Arc::new(Mutex::new(Im483Controller::new(io)));
        let mut axis = DrvIm483Axis::im483pl(controller, 0);
        let user = AsynUser::new(0);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }
}
