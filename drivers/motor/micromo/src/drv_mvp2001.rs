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
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

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
        match self.handle.read_octet(0, BUFF_SIZE) {
            Ok(raw) if !raw.is_empty() => {
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
        let Ok(port) = self.handle.port() else {
            return;
        };
        let quick = SyncIOHandle::from_handle(port.clone(), 0, Duration::ZERO);
        while quick
            .read_octet(0, BUFF_SIZE)
            .is_ok_and(|raw| !raw.is_empty())
        {}
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

/// One controller of a model-1 MVP 2001 chain.
pub struct DrvMvp2001Axis {
    chain: Arc<Mutex<Mvp2001Chain>>,
    /// The controller's chain address, `signal + 1`.
    addr: usize,
    /// `motor_info->low_limit` / `high_limit`, read at init.
    limits: (i32, i32),
    /// The record's OUT parm, `"ENC_CPR,MAX_mA"`.
    parm: String,
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
}

impl DrvMvp2001Axis {
    /// Controller `signal + 1` of `chain`, with the travel limits
    /// [`motor_init`] read for it: zeroed once more and its status read, as
    /// `motor_init` ends.
    pub fn new(chain: Arc<Mutex<Mvp2001Chain>>, signal: usize, limits: (i32, i32)) -> Self {
        let mut axis = Self {
            chain,
            addr: signal + 1,
            limits,
            parm: String::new(),
            failure: PollFailure::NONE,
            status: MotorStatus {
                gain_support: true,
                ..MotorStatus::default()
            },
            position: 0,
            strings: None,
            post: None,
        };
        // HO too soon after EN reads back within ten counts of zero.
        sleep(Duration::from_millis(200));
        axis.send("HO");
        let _ = axis.poll(&AsynUser::new(0));
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

    /// `SET_VELOCITY` and `SET_ACCEL` of a move. C works SET_ACCEL's speed
    /// out from the record's VELO; the record hands the driver the move's
    /// own speed, which is VELO for every move but a backlash or jog.
    fn speed_and_accel(&self, velocity: f64, acceleration: f64) -> [String; 2] {
        let sp = nint(velocity * 0.03);
        let ac = nint(acceleration * 0.03 * f64::from(self.encoder_cpr()) * 0.0000625);
        let ac = if ac < sp { ac.max(1) } else { sp };
        [format!("SP {sp}"), format!("AC {ac}")]
    }

    /// A move or jog: its PREM, if any, goes between the set-up and the
    /// motion commands, and its POST becomes the one `poll` sends.
    fn send_move(&mut self, setup: &[String], motion: &[String]) {
        let (prem, post) = self.strings.take().unwrap_or_default();
        self.post = post;
        for command in setup {
            self.send(command);
        }
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

impl AsynMotor for DrvMvp2001Axis {
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
        let setup = self.speed_and_accel(velocity, acceleration);
        self.send_move(&setup, &[format!("LA {}", nint(position)), "M".into()]);
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let setup = self.speed_and_accel(velocity, acceleration);
        self.send_move(&setup, &[format!("LR {}", nint(distance)), "M".into()]);
        Ok(())
    }

    /// `SET_ACCEL` then `JOG`, which starts the motion itself.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let [_, accel] = self.speed_and_accel(velocity.abs(), acceleration);
        self.send_move(&[accel], &[format!("V {}", nint(velocity * 0.03))]);
        Ok(())
    }

    /// The MVP 2001 cannot home. C's record still sends the SET_VELOCITY,
    /// SET_ACCEL and GO around the empty HOME, and that GO's `M` starts the
    /// last loaded move again; nothing is sent here.
    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        self.strings = None;
        self.post = None;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
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

    /// `SET_HIGH_LIMIT`: nothing is sent; a limit past the controller's is
    /// refused.
    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if nint(position) > self.limits.1 {
            return Err(refused("MVP2001: high limit past the controller's"));
        }
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if nint(position) < self.limits.0 {
            return Err(refused("MVP2001: low limit past the controller's"));
        }
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
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
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
        status.moving = !status.done;
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
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_fields_follow_the_address() {
        assert_eq!(hex_at("0001 8004", 4), 0x8004);
        assert_eq!(hex_at("0001 FFFFFFF6", 8) as i32, -10);
        assert_eq!(hex_at("0001 00000010junk", 8), 16);
        assert_eq!(hex_at("", 8), 0);
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
        let mut axis = DrvMvp2001Axis::new(chain, 0, (0, 0));
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
