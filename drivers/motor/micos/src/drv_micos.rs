//! The model-1 Micos MoCo dc driver (`drvMicos.cc` + `devMicos.cc`):
//! records reach MoCo address `signal` on the serial line of `card` through
//! `DTYP "Micos MoCo"`.
//!
//! Every command is `^A`, the address, then the mnemonic. Commands get no
//! reply; the `ts` and `tp` queries get one line.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::{MoveStrings, init_read};
use motor_common::util::{atoi, nint};

/// `MICOS_NUM_CARDS`.
pub const MICOS_NUM_CARDS: i32 = 16;
/// `MICOS_NUM_AXIS`.
pub const MICOS_NUM_AXIS: i32 = 16;
/// `CTLA`.
const CTLA: char = '\x01';
/// `BUFF_SIZE`.
const BUFF_SIZE: usize = 100;

/// One serial line of MoCo drivers.
pub struct MicosController {
    handle: SyncIOHandle,
}

impl MicosController {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    /// `send_mess` then `recv_mess`: the reply, empty when none came.
    fn ask(&self, msg: &str) -> String {
        // write_read flushes first; C reads with no flush past motor_init
        // (drvMicos.cc:355), so one late reply shifts every later answer: a
        // C defect, not copied.
        match self
            .handle
            .write_read(0, msg.as_bytes(), BUFF_SIZE)
            .into_parts()
        {
            (raw, Ok(())) => String::from_utf8_lossy(&raw).into_owned(),
            (_, Err(_)) => String::new(),
        }
    }

    fn send(&self, msg: &str) {
        self.send_bytes(msg.as_bytes());
    }

    /// `send_mess` of a record string, written as the field holds it.
    fn send_bytes(&self, msg: &[u8]) {
        let _ = self.handle.write_octet(0, msg);
    }
}

/// `motor_init`'s probe: `ts` to each of the `num_axis` addresses, three
/// tries each. False when an address never answered, which leaves the whole
/// card unconfigured.
///
/// C counts the tries across all addresses, so after the first three only
/// one try is left per address; each gets three here.
pub fn micos_probe(ctrl: &MicosController, num_axis: i32) -> bool {
    (0..num_axis).all(|n| (0..3).any(|_| !ctrl.ask(&format!("{CTLA}{n}ts")).is_empty()))
}

/// One MoCo driver: one motor.
pub struct DrvMicosAxis {
    controller: Arc<Mutex<MicosController>>,
    axis: i32,
    /// `motor_info->status`, which `set_status` starts from.
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
    /// The raw soft limits, which a jog moves to.
    high_limit: f64,
    low_limit: f64,
    strings: MoveStrings,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl DrvMicosAxis {
    /// MoCo address `axis` as `motor_init` takes it up: echo off (`ef`),
    /// stop (`ab1`), then its status (drvMicos.cc:527-553).
    pub fn new(controller: Arc<Mutex<MicosController>>, axis: i32) -> Self {
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.send(&format!("{CTLA}{axis}ef"));
            ctrl.send(&format!("{CTLA}{axis}ab1"));
        }
        let mut axis = Self {
            controller,
            axis,
            status: MotorStatus {
                has_encoder: true,
                gain_support: true,
                ..MotorStatus::default()
            },
            position: 0,
            high_limit: 0.0,
            low_limit: 0.0,
            strings: MoveStrings::default(),
            init_status: None,
        };
        axis.init_status = Some(axis.poll(&AsynUser::new(0)));
        axis
    }

    fn lock(&self) -> MutexGuard<'_, MicosController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn send(&self, commands: &[String]) -> AsynResult<()> {
        let ctrl = self.lock();
        for command in commands {
            ctrl.send(&format!("{CTLA}{}{command}", self.axis));
        }
        Ok(())
    }

    /// A MOVE_ABS, MOVE_REL, HOME or JOG: the record's earlier commands,
    /// PREM as its own message, then `motion`; POST is armed. C copies PREM
    /// into the message and starts a new transaction over it without ending
    /// the first (devMicos.cc:197-200), so C never sends PREM; not copied.
    fn send_move(&mut self, commands: Vec<String>, motion: String) -> AsynResult<()> {
        let prem = self.strings.arm();
        self.send(&commands)?;
        if !prem.is_empty() {
            self.lock().send_bytes(&prem);
        }
        self.send(&[motion])
    }

    /// `SET_ACCEL`.
    fn accel(commands: &mut Vec<String>, acceleration: f64) {
        commands.push(format!("da{},ud", clamp(acceleration, 1_000_000)));
    }

    /// The record's `SET_VELOCITY`, `SET_ACCEL` (`SET_VEL_BASE` sends
    /// nothing), then `motion`.
    fn motion(&mut self, velocity: f64, acceleration: f64, motion: String) -> AsynResult<()> {
        let mut commands = vec![format!("dv{},ud", clamp(velocity, 1_000_000))];
        // A move's or home's SET_ACCEL only at a positive rate
        // (motorRecord.cc:874); a jog's at any (:2141).
        if acceleration > 0.0 {
            Self::accel(&mut commands, acceleration);
        }
        self.send_move(commands, motion)
    }
}

/// `NINT(parm)` clamped to `0..=max`.
fn clamp(value: f64, max: i32) -> i32 {
    nint(value).clamp(0, max)
}

/// `strtol("0x" + two reply characters)`: the hex byte at `j`, 0 where the
/// reply has none.
fn hex_byte(reply: &[u8], j: usize) -> i64 {
    let digit = |i: usize| reply.get(i).and_then(|&b| (b as char).to_digit(16));
    match (digit(j), digit(j + 1)) {
        (Some(hi), Some(lo)) => i64::from(hi * 16 + lo),
        (Some(hi), None) => i64::from(hi),
        _ => 0,
    }
}

impl Model1Motor for DrvMicosAxis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, MotorStatus::default(), self.poll_failure())
    }
}

impl AsynMotor for DrvMicosAxis {
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
        self.motion(velocity, acceleration, format!("ma{}", nint(position)))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.motion(velocity, acceleration, format!("mr{}", nint(distance)))
    }

    /// The MoCo has no jog: move to the soft limit in the jog's direction.
    /// C picks DHLM/MRES for a positive raw velocity, which is the low raw
    /// limit under a negative MRES; this takes the raw limit the motion
    /// heads for.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let mut commands = Vec::new();
        Self::accel(&mut commands, acceleration);
        let limit = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        };
        self.send_move(commands, format!("ma{}", limit.trunc() as i64))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        self.motion(velocity, acceleration, "gh".to_string())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.send(&["ab1".to_string()])
    }

    /// `LOAD_POS`: the MoCo defines only its zero, with `dh`.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if position != 0.0 {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "Micos MoCo loads position 0 only".into(),
            });
        }
        self.send(&["dh,ud".to_string()])
    }

    /// devMicos.cc:305-314 marks SET_HIGH/LOW_LIMIT "nothing to send" with
    /// IDLE_STATE, then returns motor_end_trans_com's ERROR for that state,
    /// so the record reverts every HLM/LLM/DHLM/DLLM put; not copied.
    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.high_limit = position;
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.low_limit = position;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let command = if enable { "mn" } else { "mf" };
        self.send(&[command.to_string()])
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let mnemonic = match kind {
            PidGainKind::Proportional => "dp",
            PidGainKind::Integral => "di",
            PidGainKind::Derivative => "dd",
        };
        self.send(&[format!("{mnemonic}{},ud", clamp(gain, 32767))])
    }

    /// `PRIMITIVE`: INIT as its own message (devMicos.cc:182-188).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().send_bytes(init);
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

    /// `set_status`: `ts` for seven status bytes, `tp` for the position.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let state = ctrl.ask(&format!("{CTLA}{}ts", self.axis));
        let position = ctrl.ask(&format!("{CTLA}{}tp", self.axis));
        drop(ctrl);

        let start = if self.axis > 9 { 5 } else { 4 };
        let bytes: Vec<i64> = (0..7)
            .map(|i| hex_byte(state.as_bytes(), start + 3 * i))
            .collect();
        let positive = bytes[3] & 0x04 != 0;
        let status = &mut self.status;
        status.done = bytes[0] & 0x04 != 0;
        status.high_limit = bytes[5] & 0x04 != 0 && positive;
        status.low_limit = bytes[5] & 0x01 != 0 && !positive;
        status.slip_stall = bytes[3] & 0x48 != 0;
        if status.slip_stall {
            println!("drvMicos: set_status: EA_SLIP_STALL = 1, {}", bytes[3]);
        }
        let data = atoi(position.get(start..).unwrap_or(""));
        if data != self.position {
            status.direction = positive;
            self.position = data;
            status.position = f64::from(data);
            status.encoder_position = f64::from(data);
        }
        // drvMicos.cc:285-293: POST with no reply, once the move is done or
        // a limit in its direction is hit.
        if (status.done || status.high_limit || status.low_limit)
            && let Some(post) = self.strings.post_due()
        {
            let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.send_bytes(&post);
        }
        Ok(status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_byte_reads_two_hex_digits_as_strtol() {
        assert_eq!(hex_byte(b"0S0:0C 04", 4), 0x0c);
        assert_eq!(hex_byte(b"0S0:7", 4), 7);
        assert_eq!(hex_byte(b"0S0:", 4), 0);
        assert_eq!(hex_byte(b"0S0:x1", 4), 0);
    }

    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicBool, Ordering};

    /// MoCo address 0 on a [`LineRig`]; `ts` reports done unless `moving`.
    fn rig(name: &str) -> (LineRig, DrvMicosAxis, Arc<AtomicBool>) {
        let moving = Arc::new(AtomicBool::new(false));
        let m = moving.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "\x010ts" if m.load(Ordering::SeqCst) => "0S0:00 00 00 00 00 00 00".into(),
            "\x010ts" => "0S0:04 00 00 00 00 00 00".into(),
            "\x010tp" => "0P0:0".into(),
            _ => String::new(),
        });
        let ctrl = MicosController::new(io);
        let axis = DrvMicosAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        (line, axis, moving)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    #[test]
    fn init_goes_out_as_written() {
        let (line, mut axis, _) = rig("micosInit");
        axis.primitive(&AsynUser::default(), b"\x010mn").unwrap();
        assert_eq!(line.sent(), ["\x010mn"]);
    }

    /// PREM ahead of the motion command, POST once at the next done poll.
    #[test]
    fn prem_precedes_the_move_and_post_follows_done() {
        let (line, mut axis, moving) = rig("micosPrem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"\x010mn", Some(post(b"\x010mf")))
            .unwrap();
        axis.move_absolute(&user, 100.0, 0.0, 500.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["\x010dv500,ud", "\x010mn", "\x010ma100"]);
        moving.store(true, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["\x010ts", "\x010tp"]);
        moving.store(false, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["\x010ts", "\x010tp", "\x010mf"]);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["\x010ts", "\x010tp"]);
    }

    /// A jog and a home carry PREM too.
    #[test]
    fn jog_and_home_carry_prem() {
        let (line, mut axis, _) = rig("micosJog");
        let user = AsynUser::default();
        axis.set_high_limit(&user, 900.0).unwrap();
        axis.set_move_strings(&user, b"P", None).unwrap();
        axis.move_velocity(&user, 0.0, 10.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["\x010da0,ud", "P", "\x010ma900"]);
        axis.set_move_strings(&user, b"P", None).unwrap();
        axis.home(&user, 0.0, 10.0, 0.0, true).unwrap();
        assert_eq!(line.sent(), ["\x010dv10,ud", "P", "\x010gh"]);
    }

    /// motorRecord.cc:2141, :3068: a jog's SET_ACCEL goes out at any rate,
    /// a move's only at a positive one (:874).
    #[test]
    fn a_jog_sends_set_accel_at_a_zero_rate() {
        let (line, mut axis, _) = rig("micosJogAccel");
        let user = AsynUser::default();
        axis.set_high_limit(&user, 900.0).unwrap();
        axis.move_velocity(&user, 0.0, 10.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["\x010da0,ud", "\x010ma900"]);
        axis.move_absolute(&user, 100.0, 0.0, 500.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["\x010dv500,ud", "\x010ma100"]);
    }
}
