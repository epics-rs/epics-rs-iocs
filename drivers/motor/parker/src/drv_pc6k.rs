//! The model-1 Parker 6K (`drvPC6K.cc` + `devPC6K.cc`): `PC6KSetup` /
//! `PC6KConfig` / `PC6KUpLoad`, records reaching axis `signal + 1` of card
//! `card` through `DTYP "PC6K"`.
//!
//! The 6K takes one command per line. A query's reply is `*` and the query
//! echoed, then its value; the `>` of the ERROK prompt ends every read.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, nint};

/// `PC6K_NUM_CARDS`.
pub const PC6K_NUM_CARDS: i32 = 16;
/// `MAX_AXIS` (motor.h).
const MAX_AXIS: usize = 32;
/// `PC6K_MSG_SIZE`.
const MSG_SIZE: usize = 120;
/// `BUFF_SIZE`: the longest line `PC6KUpLoad` sends, its terminator included.
const BUFF_SIZE: usize = 120;
/// `TIMEOUT`.
pub const TIMEOUT: Duration = Duration::from_secs(1);
/// `MOTION_DELAY`: no status is read this soon after a motion starts.
const MOTION_DELAY: Duration = Duration::from_millis(50);
/// `REPLY_CHAR`.
const REPLY_CHAR: u8 = b'*';

/// `TAS` characters (drvPC6K.h), counting the `_` every four bits.
const TAS_INMOTION: usize = 0;
const TAS_NEG: usize = 1;
const TAS_HOME: usize = 5;
const TAS_HPLUSTL: usize = 17;
const TAS_HMINUSTL: usize = 18;
const TAS_SPLUSTL: usize = 20;
const TAS_SMINUSTL: usize = 21;

/// C `CommStatus`, kept per card.
#[derive(Clone, Copy, PartialEq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// One 6K controller: its port and communication state.
pub struct Pc6kCard {
    handle: SyncIOHandle,
    comm: CommState,
}

impl Pc6kCard {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self {
            handle,
            comm: CommState::Normal,
        }
    }

    /// `send_mess`: write-only.
    pub fn send(&self, msg: impl AsRef<[u8]>) {
        let msg = msg.as_ref();
        if !msg.is_empty() {
            let _ = self.handle.write_octet(0, msg);
        }
    }

    /// A record command: `send_mess`, then the prompt read away as the motor
    /// task does for a card with `cmnd_response`.
    fn command(&self, msg: impl AsRef<[u8]>) {
        let msg = msg.as_ref();
        if !msg.is_empty() {
            self.send(msg);
            let _ = self.handle.read_octet(0, BUFF_SIZE);
        }
    }

    /// `send_recv_mess`: the reply, empty when none came.
    fn ask(&self, msg: &str) -> Vec<u8> {
        self.handle
            .write_read(0, msg.as_bytes(), MSG_SIZE)
            .unwrap_or_default()
    }

    /// The value of `query`'s reply: what follows `*` and the echoed query,
    /// `None` when the reply does not start with `*`.
    fn query(&self, query: &str) -> Option<String> {
        let reply = self.ask(query);
        (reply.first() == Some(&REPLY_CHAR)).then(|| {
            String::from_utf8_lossy(reply.get(query.len() + 1..).unwrap_or_default()).into_owned()
        })
    }
}

/// What `motor_init` read of one axis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AxisLimits {
    /// `motor_info->low_limit`, from `LSNEG`.
    pub low: f64,
    /// `motor_info->high_limit`, from `LSPOS`.
    pub high: f64,
}

/// `motor_init`'s probe and set-up of one card: `TREV` naming a 6K, echo
/// off, all axes stopped, continuous commands on, then each answering axis's
/// travel limits. `None` leaves the card unconfigured, as C does.
///
/// C keeps `cardFound` across cards, so a card after a found one counts as
/// found whatever it answers, and counts axes with no bound past
/// `MAX_AXIS`; each card is probed on its own here and stops at `MAX_AXIS`.
pub fn motor_init(card: &Pc6kCard) -> Option<Vec<AxisLimits>> {
    let found = (0..3).any(|_| {
        let reply = String::from_utf8_lossy(&card.ask("TREV")).into_owned();
        reply.contains("TREV") && reply.contains("6K")
    });
    if !found {
        return None;
    }
    card.ask("ECHO0");
    card.ask("!K");
    // The 6K answers again only some time after the all-stop.
    let _ = (0..3).any(|_| {
        let reply = card.ask("DRIVE");
        !reply.is_empty() && String::from_utf8_lossy(&reply).contains("DRIVE")
    });
    card.ask("COMEXC1");

    let mut total = 0;
    while total < MAX_AXIS && card.query(&format!("{}TPC", total + 1)).is_some() {
        total += 1;
    }
    let limits = (1..=total)
        .map(|axis| {
            // AXSDEF picks the resolution queried; C keeps it for nothing.
            let servo = card
                .query(&format!("{axis}AXSDEF"))
                .is_some_and(|v| v.starts_with('1'));
            card.query(&format!("{axis}{}", if servo { "ERES" } else { "DRES" }));
            let mut limits = AxisLimits::default();
            if let Some(v) = card.query(&format!("{axis}LSNEG")) {
                limits.low = atof(&v);
            }
            if let Some(v) = card.query(&format!("{axis}LSPOS")) {
                limits.high = atof(&v);
            }
            limits
        })
        .collect();
    Some(limits)
}

/// `PC6KUpLoad`'s lines of `text`: each `fgets` of at most `BUFF_SIZE - 1`
/// bytes, every unprintable byte (its newline too) made a space.
pub fn upload_lines(text: &[u8]) -> Vec<Vec<u8>> {
    let mut lines = Vec::new();
    let mut rest = text;
    while !rest.is_empty() {
        let mut end = rest.len().min(BUFF_SIZE - 1);
        if let Some(nl) = rest[..end].iter().position(|&b| b == b'\n') {
            end = nl + 1;
        }
        let line = rest[..end]
            .iter()
            .map(|&b| if (0x20..0x7f).contains(&b) { b } else { b' ' })
            .collect();
        lines.push(line);
        rest = &rest[end..];
    }
    lines
}

/// One axis of a model-1 6K.
pub struct DrvPc6kAxis {
    card: Arc<Mutex<Pc6kCard>>,
    /// The axis number, `signal + 1`.
    axis: usize,
    limits: AxisLimits,
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
    /// The record's PREM and POST for the move it sends next.
    strings: Option<(Vec<u8>, Option<PostMessage>)>,
    /// The move in flight's POST, `nodeptr->postmsgptr`. C never sends it:
    /// every `build_trans` restarts the transaction, which clears the
    /// pointer before the move is queued.
    post: Option<PostMessage>,
    /// When the last motion started: C's motor task reads no status until
    /// `MOTION_DELAY` after it, the GO's prompt read away by then.
    started: Option<Instant>,
}

impl DrvPc6kAxis {
    /// Axis `signal + 1` of `card`, with the limits [`motor_init`] read for
    /// it, its status read as `motor_init` ends.
    pub fn new(card: Arc<Mutex<Pc6kCard>>, signal: usize, limits: AxisLimits) -> Self {
        let mut axis = Self {
            card,
            axis: signal + 1,
            limits,
            status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
            position: 0,
            strings: None,
            post: None,
            started: None,
        };
        let _ = axis.poll(&AsynUser::new(0));
        axis
    }

    fn lock(&self) -> MutexGuard<'_, Pc6kCard> {
        self.card.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn send(&self, command: &str) {
        self.lock().command(format!("{}{command}", self.axis));
    }

    /// `SET_VELOCITY`.
    fn velocity(&self, velocity: f64) {
        self.send(&format!("V{}", nint(velocity)));
    }

    /// `SET_ACCEL`: acceleration and deceleration, their averages half.
    fn accel(&self, acceleration: f64) {
        let a = nint(acceleration);
        let half = nint(acceleration / 2.0);
        for command in [
            format!("A{a}"),
            format!("AA{half}"),
            format!("AD{a}"),
            format!("ADA{half}"),
        ] {
            self.send(&command);
        }
    }

    /// The record's PREM, sent as it is, and its POST kept for `poll`.
    fn prem(&mut self) {
        let (prem, post) = self.strings.take().unwrap_or_default();
        self.post = post;
        self.lock().command(prem);
    }

    /// `SET_VELOCITY`, `SET_ACCEL` unless zero, the PREM, `motion`, `GO`.
    fn start(&mut self, velocity: f64, acceleration: f64, motion: &[String]) {
        self.velocity(velocity);
        if acceleration > 0.0 {
            self.accel(acceleration);
        }
        self.prem();
        for command in motion {
            self.send(command);
        }
        self.send("GO");
        self.started = Some(Instant::now());
    }

    fn move_to(&mut self, mode: u8, target: f64, velocity: f64, acceleration: f64) {
        let motion = [
            format!("MA{mode}"),
            "MC0".into(),
            format!("D{}", nint(target)),
        ];
        self.start(velocity, acceleration, &motion);
    }
}

fn refused(message: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

impl AsynMotor for DrvPc6kAxis {
    /// `poll` never fails: `set_status`'s missed replies are handled in it.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.move_to(1, position, velocity, acceleration);
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
        self.move_to(0, distance, velocity, acceleration);
        Ok(())
    }

    /// `SET_ACCEL` then `JOG`. C sends the signed speed, `V-500`, which the
    /// 6K's unsigned `V` refuses; `D+`/`D-` carries the direction.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.accel(acceleration);
        self.prem();
        self.send("MC1");
        self.velocity(velocity.abs());
        self.send(if velocity >= 0.0 { "D+" } else { "D-" });
        self.send("GO");
        self.started = Some(Instant::now());
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let motion = [if forward { "HOM0" } else { "HOM1" }.to_string()];
        self.start(velocity, acceleration, &motion);
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.lock().command(format!("!{}S", self.axis));
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(&format!("PSET{}", nint(position)));
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.send(if enable { "DRIVE1" } else { "DRIVE0" });
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let command = match kind {
            PidGainKind::Proportional => "SGP",
            PidGainKind::Integral => "SGI",
            PidGainKind::Derivative => "SGV",
        };
        self.send(&format!("{command}{}", nint(gain)));
        Ok(())
    }

    /// `SET_HIGH_LIMIT`: nothing is sent; past the controller's `LSPOS`, when
    /// it has a travel range, the limit is refused.
    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let AxisLimits { low, high } = self.limits;
        if high > low && f64::from(nint(position)) > high {
            return Err(refused("PC6K: high limit past the controller's"));
        }
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let AxisLimits { low, high } = self.limits;
        if low < high && f64::from(nint(position)) < low {
            return Err(refused("PC6K: low limit past the controller's"));
        }
        Ok(())
    }

    /// `PRIMITIVE`: INIT as it is.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().command(init);
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

    /// `set_status`: `TAS`, `TPC`, `TPE`, `TVELA` and `DRIVE`, each retried
    /// once. A first failed round keeps the last status; a second raises
    /// the comms error.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        if let Some(started) = self.started.take() {
            sleep(MOTION_DELAY.saturating_sub(started.elapsed()));
        }
        let card = self.card.clone();
        let mut ctrl = card.lock().unwrap_or_else(|e| e.into_inner());
        let mut replies = Vec::with_capacity(5);
        for query in ["TAS", "TPC", "TPE", "TVELA", "DRIVE"] {
            let query = format!("{}{query}", self.axis);
            match ctrl.query(&query).or_else(|| ctrl.query(&query)) {
                Some(value) => replies.push(value),
                None => break,
            }
        }
        let [tas, tpc, tpe, tvela, drive] = match <[String; 5]>::try_from(replies) {
            Ok(replies) => replies,
            Err(_) => {
                if ctrl.comm == CommState::Normal {
                    ctrl.comm = CommState::Retry;
                    drop(ctrl);
                    sleep(MOTION_DELAY);
                } else {
                    ctrl.comm = CommState::CommErr;
                    self.status.comms_error = true;
                    self.status.problem = true;
                }
                return Ok(self.status.clone());
            }
        };
        ctrl.comm = CommState::Normal;
        drop(ctrl);

        let bit = |i: usize, c: u8| tas.as_bytes().get(i) == Some(&c);
        let status = &mut self.status;
        status.comms_error = false;
        status.direction = bit(TAS_NEG, b'0');
        status.home = bit(TAS_HOME, b'1');
        status.done = bit(TAS_INMOTION, b'0');
        status.moving = !status.done;
        status.high_limit = !(bit(TAS_HPLUSTL, b'0') && bit(TAS_SPLUSTL, b'0') || status.home);
        status.low_limit = !(bit(TAS_HMINUSTL, b'0') && bit(TAS_SMINUSTL, b'0') || status.home);
        let ls_active =
            (status.high_limit && status.direction) || (status.low_limit && !status.direction);
        status.powered = drive.starts_with('1');
        // As C, the encoder is read only when the commanded position moved.
        let data = atof(&tpc) as i32;
        if data != self.position {
            self.position = data;
            status.encoder_position = f64::from(atoi(&tpe));
        }
        status.position = f64::from(self.position);
        status.velocity = f64::from(atof(&tvela) as i32);
        status.problem = false;
        let status = status.clone();
        if (status.done || ls_active)
            && let Some(post) = self.post.take().map(|p| p.get())
            && !post.is_empty()
        {
            self.lock().send(post);
        }
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn upload_lines_are_fgets_chunks_with_unprintables_blanked() {
        let lines = upload_lines(b"A1\tB\nC\r\nlast");
        assert_eq!(
            lines,
            vec![b"A1 B ".to_vec(), b"C  ".to_vec(), b"last".to_vec()]
        );
        let long = vec![b'x'; 130];
        let lines = upload_lines(&long);
        assert_eq!(lines.len(), 2);
        assert_eq!((lines[0].len(), lines[1].len()), (119, 11));
    }
}
