//! The model-1 MVP 2001 (`drvMVP2001.cc` + `devMVP2001.cc`): `MicroMoSetup`
//! / `MicroMoConfig`, records reaching controller `signal + 1` of chain
//! `card` through `DTYP "MVP2001"`.
//!
//! Commands are `"<addr> <cmd>"` and get no reply. A query's reply is led by
//! a bare terminator, so `recv_mess` reads twice. The record's OUT parm is
//! `"ENC_CPR,MAX_mA"`; a PREM starting `ANO` sends the analog current limit
//! worked out from MAX_mA, any other PREM goes as it is, each ahead of a
//! move or jog.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage, RecordFields,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::device_support::{BuildTrans, DialLimitReset, LimitBuild};

use motor_common::model1::{init_poll, init_read};
use motor_common::util::{atoi, nint};

/// `MVP2001_NUM_CARDS`.
pub const MVP2001_NUM_CARDS: i32 = 8;
/// `MAX_AXIS` (motor.h): the most controllers `motor_init` probes on a chain.
const MAX_AXIS: usize = 32;
/// `BUFF_SIZE`.
const BUFF_SIZE: usize = 20;
/// `COMM_TIMEOUT` (drvMVP2001.h).
pub const COMM_TIMEOUT: Duration = Duration::from_secs(2);

/// C `CommStatus`, kept per chain.
#[derive(Clone, Copy, PartialEq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// One MVP 2001 chain: its serial link and communication state.
pub struct Mvp2001Chain {
    handle: SyncIOHandle,
    comm: CommState,
}

impl Mvp2001Chain {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self {
            handle,
            comm: CommState::Normal,
        }
    }

    /// `send_mess`: write-only, the MVP answers no command.
    fn send(&self, msg: impl AsRef<[u8]>) {
        let _ = self.handle.write_octet(0, msg.as_ref());
    }

    /// `recv_mess`: the bare terminator a reply starts with, then the reply,
    /// empty when none came.
    fn recv(&self) -> String {
        let _ = self.handle.read_octet(0, BUFF_SIZE);
        match self.handle.read_octet(0, BUFF_SIZE).into_parts() {
            (raw, Ok(())) if !raw.is_empty() => {
                sleep(Duration::from_millis(100));
                String::from_utf8_lossy(&raw).into_owned()
            }
            _ => String::new(),
        }
    }

    fn ask(&self, msg: &str) -> String {
        self.send(msg);
        self.recv()
    }

    /// `recv_mess(FLUSH)` until nothing is left. C asks for a zero timeout
    /// but reads with `COMM_TIMEOUT` anyway, so every flush of an idle line
    /// waits 4 s; this reads with the zero timeout it asks for.
    fn flush(&self) {
        let quick = self.handle.with_timeout(Duration::ZERO);
        while matches!(
            quick.read_octet(0, BUFF_SIZE).into_parts(),
            (raw, Ok(())) if !raw.is_empty()
        ) {}
    }
}

/// `strtoul(reply + 5, 16)` over at most `width` characters, 0 when none.
fn hex_at(reply: &str, width: usize) -> u32 {
    let field = reply.get(5..).unwrap_or("");
    let field = &field[..field.len().min(width)];
    let digits: String = field
        .trim_start()
        .chars()
        .take_while(char::is_ascii_hexdigit)
        .collect();
    u32::from_str_radix(&digits, 16).unwrap_or(0)
}

/// `motor_init`'s probe and set-up of one chain: the number of answering
/// controllers, each stopped, zeroed and enabled, and its travel limits.
/// Zero leaves the chain unconfigured, as C does.
pub fn motor_init(chain: &Mvp2001Chain) -> Vec<(i32, i32)> {
    let mut total = 0;
    while total < MAX_AXIS {
        chain.flush();
        let addr = total + 1;
        if !(0..3).any(|_| !chain.ask(&format!("{addr} ST")).is_empty()) {
            break;
        }
        total += 1;
    }
    (1..=total)
        .map(|addr| {
            chain.send(format!("{addr} V 0"));
            chain.send(format!("{addr} HO"));
            chain.send(format!("{addr} EN"));
            let low = hex_at(&chain.ask(&format!("{addr} LL -")), 8) as i32;
            let high = hex_at(&chain.ask(&format!("{addr} LL")), 8) as i32;
            (low, high)
        })
        .collect()
}

/// `MOTOR_STATUS` bits of an `ST` reply.
const MINUS_LS: u32 = 1 << 0;
const PLUS_LS: u32 = 1 << 2;
const NOT_POWER: u32 = 1 << 7;
const IN_MOTION: u32 = 1 << 15;

/// The analog current limit `ANO` sends for `max_ma`, 100 mA outside the
/// 0.1-2.3 A the manual allows.
fn ano(max_ma: i32) -> i32 {
    let max_ma = if (100..=2300).contains(&max_ma) {
        max_ma
    } else {
        100
    };
    nint(f64::from(max_ma) * 0.865909 + 2103.431)
}

/// devMVP2001's `build_trans` on the soft limits of one controller, from
/// the travel limits [`motor_init`] read for it. SET_HIGH_LIMIT and
/// SET_LOW_LIMIT send nothing (devMVP2001.cc:430-448).
pub struct Mvp2001Build {
    /// `motor_info->low_limit` / `high_limit`.
    limits: (i32, i32),
}

impl Mvp2001Build {
    pub fn new(limits: (i32, i32)) -> Self {
        Self { limits }
    }
}

impl BuildTrans for Mvp2001Build {
    /// A raw limit past the controller's is ERROR and resets the record's
    /// dial limit to it. devMVP2001.cc:436/447 write DHLM for
    /// SET_HIGH_LIMIT and DLLM for SET_LOW_LIMIT, but under MRES < 0 the
    /// record sends DHLM as SET_LOW_LIMIT (motorRecord.cc:4246), so C resets
    /// the other dial limit. That C defect is not copied: the dial limit
    /// the raw one maps to is reset.
    fn limit(&self, high: bool, raw: f64, mres: f64) -> LimitBuild {
        let (low, high_limit) = self.limits;
        let (limit, past) = if high {
            (high_limit, nint(raw) > high_limit)
        } else {
            (low, nint(raw) < low)
        };
        if !past {
            return LimitBuild::Built;
        }
        let dial = f64::from(limit) * mres;
        LimitBuild::Refused(Some(if high == (mres > 0.0) {
            DialLimitReset::Dhlm(dial)
        } else {
            DialLimitReset::Dllm(dial)
        }))
    }
}

/// One controller of a model-1 MVP 2001 chain.
pub struct DrvMvp2001Axis {
    chain: Arc<Mutex<Mvp2001Chain>>,
    /// The controller's chain address, `signal + 1`.
    addr: usize,
    /// The record's OUT parm, `"ENC_CPR,MAX_mA"`.
    parm: String,
    /// The record's VELO in steps/s, `mr->velo / fabs(mr->mres)`.
    record_velocity: f64,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed `ST` (RETRY, drvMVP2001.cc:320), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :326). Per axis, as the chain's
    /// state can move on under another axis's poll.
    failure: PollFailure,
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
    /// The record's PREM and POST for the move it sends next.
    strings: Option<(Vec<u8>, Option<PostMessage>)>,
    /// The move in flight's POST, `nodeptr->postmsgptr`. C never sends it:
    /// every `build_trans` restarts the transaction, which clears the
    /// pointer before the move is queued.
    post: Option<PostMessage>,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl DrvMvp2001Axis {
    /// Controller `signal + 1` of `chain`: zeroed once more and its status
    /// read, as `motor_init` ends (drvMVP2001.cc:711-720).
    pub fn new(chain: Arc<Mutex<Mvp2001Chain>>, signal: usize) -> Self {
        let mut axis = Self {
            chain,
            addr: signal + 1,
            parm: String::new(),
            record_velocity: 0.0,
            failure: PollFailure::NONE,
            status: MotorStatus {
                gain_support: true,
                ..MotorStatus::default()
            },
            position: 0,
            strings: None,
            post: None,
            init_status: None,
        };
        // HO too soon after EN reads back within ten counts of zero.
        sleep(Duration::from_millis(200));
        axis.send("HO");
        axis.init_status = Some(init_poll(&mut axis, &AsynUser::new(0)));
        axis
    }

    fn lock(&self) -> MutexGuard<'_, Mvp2001Chain> {
        self.chain.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn send(&self, command: &str) {
        self.lock().send(format!("{} {command}", self.addr));
    }

    /// The parm's leading digits, the encoder counts per revolution.
    fn encoder_cpr(&self) -> i32 {
        atoi(&self.parm)
    }

    /// The parm's digits after its first comma, the current limit in mA.
    /// C walks past the end of a parm with no comma.
    fn max_current(&self) -> i32 {
        self.parm.split_once(',').map_or(0, |(_, rest)| {
            let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
            atoi(&digits)
        })
    }

    /// A move or jog: its PREM, if any, goes ahead of the motion commands, and its POST becomes the one `poll` sends.
    fn send_move(&mut self, motion: &[String]) {
        let (prem, post) = self.strings.take().unwrap_or_default();
        self.post = post;
        if !prem.is_empty() {
            if prem.starts_with(b"ANO") {
                self.send(&format!("ANO {}", ano(self.max_current())));
            } else {
                let mut line = format!("{} ", self.addr).into_bytes();
                line.extend_from_slice(&prem);
                self.lock().send(line);
            }
        }
        for command in motion {
            self.send(command);
        }
    }
}

fn refused(message: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

impl Model1Motor for DrvMvp2001Axis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.status.clone(), self.poll_failure())
    }

    /// `update_delay` is 0 (drvMVP2001.cc:202).
    fn update_delay(&self) -> Duration {
        Duration::ZERO
    }

    fn set_velocity(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        self.send(&format!("SP {}", nint(velocity * 0.03)));
        Ok(())
    }

    /// SET_VEL_BASE: the MVP 2001 does not use base velocity.
    fn set_vel_base(&mut self, _user: &AsynUser, _velocity: f64) -> AsynResult<()> {
        Ok(())
    }

    /// SET_ACCEL caps at the record's VELO, not the move's speed
    /// (devMVP2001.cc:373), which differs for a backlash or jog.
    fn set_accel(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let sp = nint(self.record_velocity * 0.03);
        let ac = nint(acceleration * 0.03 * f64::from(self.encoder_cpr()) * 0.0000625);
        let ac = if ac < sp { ac.max(1) } else { sp };
        self.send(&format!("AC {ac}"));
        Ok(())
    }

    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn set_record_fields(&mut self, _user: &AsynUser, fields: &RecordFields) -> AsynResult<()> {
        self.record_velocity = fields.velocity;
        Ok(())
    }

    fn move_absolute(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send_move(&[format!("LA {}", nint(position)), "M".into()]);
        Ok(())
    }

    fn move_relative(&mut self, _user: &AsynUser, distance: f64) -> AsynResult<()> {
        self.send_move(&[format!("LR {}", nint(distance)), "M".into()]);
        Ok(())
    }

    /// `JOG`, which starts the motion itself.
    fn move_velocity(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        self.send_move(&[format!("V {}", nint(velocity * 0.03))]);
        Ok(())
    }

    /// The MVP 2001 cannot home: HOME sends nothing, but the record's
    /// SET_VELOCITY and SET_ACCEL before it go out, and GO's `M` starts
    /// the last loaded move again (devMVP2001.cc:323,332). No PREM or POST.
    fn home(&mut self, _user: &AsynUser, _forward: bool) -> AsynResult<()> {
        self.strings = None;
        self.send_move(&["M".into()]);
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser) -> AsynResult<()> {
        self.send("AB");
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(&format!("HO {}", nint(position)));
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.send(if enable { "EN" } else { "DI" });
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let command = match kind {
            PidGainKind::Proportional => format!("POR {}", nint(gain * 28000.0 + 4000.0)),
            PidGainKind::Integral => format!("I {}", nint(gain * 31999.0 + 1.0)),
            PidGainKind::Derivative => format!("DER {}", nint(gain * 31000.0 + 1000.0)),
        };
        self.send(&command);
        Ok(())
    }

    fn set_out_parm(&mut self, _user: &AsynUser, parm: &[u8]) -> AsynResult<()> {
        self.parm = String::from_utf8_lossy(parm).into_owned();
        Ok(())
    }

    /// `PRIMITIVE`: INIT behind the controller address.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        if init.is_empty() {
            return Ok(());
        }
        let mut line = format!("{} ", self.addr).into_bytes();
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
        self.strings = Some((prem.to_vec(), post));
        Ok(())
    }

    /// `set_status`: `ST` for motion, limits and power, `POS` for the
    /// position. A first unanswered `ST` fails the poll raising nothing; a
    /// second raises the comms error.
    fn poll(&mut self, _user: &AsynUser, out: &mut MotorStatus) -> AsynResult<()> {
        let chain = self.chain.clone();
        let mut ctrl = chain.lock().unwrap_or_else(|e| e.into_inner());
        let st = ctrl.ask(&format!("{} ST", self.addr));
        if st.is_empty() {
            (ctrl.comm, self.failure) = match ctrl.comm {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(refused(&format!("MVP2001 {}: no ST reply", self.addr)));
        }
        ctrl.comm = CommState::Normal;
        let pos = ctrl.ask(&format!("{} POS", self.addr));
        drop(ctrl);

        let mstat = hex_at(&st, 4);
        let status = &mut self.status;
        status.comms_error = false;
        status.done = mstat & IN_MOTION == 0;
        // C parses a missed POS reply as position 0; the position stands.
        if !pos.is_empty() {
            let data = hex_at(&pos, 8) as i32;
            if data != self.position {
                status.direction = data >= self.position;
                self.position = data;
            }
        }
        status.position = f64::from(self.position);
        status.high_limit = mstat & PLUS_LS != 0;
        status.low_limit = mstat & MINUS_LS != 0;
        status.home = false;
        status.powered = mstat & NOT_POWER == 0;
        status.encoder_position = 0.0;
        status.problem = false;
        let ls_active =
            (status.high_limit && status.direction) || (status.low_limit && !status.direction);
        let status = status.clone();
        if (status.done || ls_active)
            && let Some(post) = self.post.take().map(|p| p.get())
            && !post.is_empty()
        {
            // POST goes as the record holds it, with no address.
            self.lock().send(post);
        }
        *out = status;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::Model1Polled;
    use motor_common::testing::RecordWrites;

    #[test]
    fn hex_fields_follow_the_address() {
        assert_eq!(hex_at("0001 8004", 4), 0x8004);
        assert_eq!(hex_at("0001 FFFFFFF6", 8) as i32, -10);
        assert_eq!(hex_at("0001 00000010junk", 8), 16);
        assert_eq!(hex_at("", 8), 0);
    }

    /// drvMVP2001.cc:675-720: a failed `motor_init` read stores the
    /// GAIN_SUPPORT `motor_init` set, with EA_PRESENT off, the first raising
    /// nothing (RETRY) and the next CNTRL_COMM_ERR and RA_PROBLEM
    /// (:316-330).
    #[test]
    fn a_failed_init_read_keeps_gain_support() {
        let (_line, io) = motor_common::testing::LineRig::new("mvpInitFail", |_| String::new());
        let chain = Arc::new(Mutex::new(Mvp2001Chain::new(io)));
        let init = |signal| DrvMvp2001Axis::new(chain.clone(), signal).take_init_status();
        let prepared = MotorStatus {
            gain_support: true,
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

    /// devMVP2001.cc:430-448: a raw limit at the controller's is built, one
    /// past it is refused and resets the dial limit it maps to, which under
    /// MRES < 0 is the other one (motorRecord.cc:4246).
    #[test]
    fn a_limit_past_the_controllers_resets_its_dial_limit() {
        use DialLimitReset::{Dhlm, Dllm};
        let build = Mvp2001Build::new((-100000, 100000));
        assert_eq!(build.limit(true, 100000.4, 0.5), LimitBuild::Built);
        assert_eq!(build.limit(false, -100000.4, 0.5), LimitBuild::Built);
        let refused = |reset| LimitBuild::Refused(Some(reset));
        assert_eq!(build.limit(true, 100000.5, 0.5), refused(Dhlm(50000.0)));
        assert_eq!(build.limit(false, -100000.5, 0.5), refused(Dllm(-50000.0)));
        assert_eq!(build.limit(true, 200000.0, -0.5), refused(Dllm(-50000.0)));
        assert_eq!(build.limit(false, -200000.0, -0.5), refused(Dhlm(50000.0)));
    }

    #[test]
    fn ano_holds_the_current_to_the_manual_range() {
        assert_eq!(ano(500), nint(500.0 * 0.865909 + 2103.431));
        assert_eq!(ano(50), ano(100));
        assert_eq!(ano(3000), ano(100));
    }

    /// drvMVP2001.cc:311-333: a first unanswered `ST` is RETRY (nothing
    /// raised, nothing posted), the next COMM_ERR, and a reply clears it.
    #[test]
    fn an_unanswered_st_is_retry_then_comm_error() {
        use motor_common::testing::LineRig;
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("mvp2001Retry", move |last| match last {
            "1 ST" if !q.load(Ordering::SeqCst) => "0001 0000".into(),
            "1 POS" => "0001 00000010".into(),
            _ => String::new(),
        });
        let chain = Arc::new(Mutex::new(Mvp2001Chain::new(io)));
        let mut axis = DrvMvp2001Axis::new(chain, 0);
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

    /// devMVP2001.cc:373-386: SET_ACCEL caps at `mr->velo / mres`, so a
    /// backlash move at BVEL above VELO still takes VELO's AC.
    #[test]
    fn set_accel_caps_at_the_record_velocity() {
        use motor_common::testing::LineRig;
        let (line, io) = LineRig::new("mvp2001Accel", |last| match last {
            "1 ST" => "0001 0000".into(),
            "1 POS" => "0001 00000000".into(),
            _ => String::new(),
        });
        let chain = Arc::new(Mutex::new(Mvp2001Chain::new(io)));
        let mut axis = DrvMvp2001Axis::new(chain, 0);
        let user = AsynUser::new(0);
        axis.set_out_parm(&user, b"4000,500").unwrap();
        axis.set_record_fields(
            &user,
            &RecordFields {
                velocity: 1000.0,
                srev: 200,
            },
        )
        .unwrap();
        axis.record_move_relative(&user, 10.0, 0.0, 2000.0, 1e6)
            .unwrap();
        let sent = line.sent();
        let tail = &sent[sent.len() - 4..];
        assert_eq!(tail, ["1 SP 60", "1 AC 30", "1 LR 10", "1 M"]);
        axis.record_move_relative(&user, 10.0, 0.0, 2000.0, 1e3)
            .unwrap();
        let sent = line.sent();
        assert_eq!(sent[sent.len() - 3], "1 AC 8");
    }

    /// devMVP2001.cc:323-345: HOME itself sends nothing, the SET_VELOCITY,
    /// SET_ACCEL and GO the record sends around it do; PREM stays unsent.
    /// SET_ACCEL caps at VELO as for a move (:373), which the record hands
    /// over ahead of the transaction.
    #[test]
    fn home_sends_speed_accel_and_go() {
        use motor_common::testing::LineRig;
        let (line, io) = LineRig::new("mvp2001Home", |last| match last {
            "1 ST" => "0001 0000".into(),
            "1 POS" => "0001 00000000".into(),
            _ => String::new(),
        });
        let chain = Arc::new(Mutex::new(Mvp2001Chain::new(io)));
        let mut axis = DrvMvp2001Axis::new(chain, 0);
        let user = AsynUser::new(0);
        axis.set_out_parm(&user, b"4000,500").unwrap();
        axis.set_move_strings(&user, b"ZZ", None).unwrap();
        axis.set_record_fields(
            &user,
            &RecordFields {
                velocity: 1000.0,
                srev: 200,
            },
        )
        .unwrap();
        line.sent();
        axis.record_home(&user, 0.0, 1000.0, 1e3, true).unwrap();
        assert_eq!(line.sent(), ["1 SP 30", "1 AC 8", "1 M"]);
    }
}
