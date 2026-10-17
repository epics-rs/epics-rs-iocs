//! The model-1 MCB-4B (`drvMCB4B.cc` + `devMCB4B.cc`): `MCB4BSetup` /
//! `MCB4BConfig`, records reaching axis `signal` of `card` through
//! `DTYP "ACS MCB-4B"`.
//!
//! The wire commands are the model-3 driver's except jog, which programs the
//! jog rate with `C=` and moves 1,000,000 steps with `M`, and the move
//! sequences, which carry no `W` power query.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atoi, nint};

use crate::mcb4b::Mcb4bController;

/// `MCB4B_NUM_CARDS` (drvMCB4B.h).
pub const MCB4B_NUM_CARDS: i32 = 4;
/// `motor_init` assumes four axes on every card.
pub const MCB4B_NUM_AXES: usize = 4;

/// `motor_init`'s probe: `#00X`, tried three times, then `#%02dQ` to every
/// axis. False when the card never answered, which C leaves unconfigured.
pub fn motor_init(ctrl: &Mcb4bController) -> bool {
    let answered = (0..3).any(|_| ctrl.command("#00X").is_ok_and(|r| !r.is_empty()));
    if answered {
        for axis in 0..MCB4B_NUM_AXES {
            let _ = ctrl.command(&format!("#{axis:02}Q"));
        }
    }
    answered
}

/// One axis of a model-1 MCB-4B card.
pub struct DrvMcb4bAxis {
    controller: Arc<Mutex<Mcb4bController>>,
    axis: usize,
    /// `motor_info->position`, which `set_status` compares for direction.
    position: i32,
    direction: bool,
    strings: MoveStrings,
}

impl DrvMcb4bAxis {
    pub fn new(controller: Arc<Mutex<Mcb4bController>>, axis: usize) -> Self {
        Self {
            controller,
            axis,
            position: 0,
            direction: false,
            strings: MoveStrings::default(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Mcb4bController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `SET_VELOCITY`: `V` = 115200/velocity, 2..=255.
    fn set_velocity(&self, ctrl: &Mcb4bController, velocity: f64) -> AsynResult<()> {
        let ival = nint((115200.0 / velocity).abs()).clamp(2, 255);
        ctrl.command(&format!("#{:02}V={ival}", self.axis))?;
        Ok(())
    }

    /// `SET_ACCEL`: ramp index `R` = 256 - 720000/accel, 1..=255.
    fn set_accel(&self, ctrl: &Mcb4bController, acceleration: f64) -> AsynResult<()> {
        let ival = nint(256.0 - 720000.0 / acceleration).clamp(1, 255);
        ctrl.command(&format!("#{:02}R={ival}", self.axis))?;
        Ok(())
    }

    /// The PREM a MOVE_ABS, MOVE_REL, HOME or JOG sends as its own message
    /// ahead of the command, arming POST (devMCB4B.cc:200-216).
    fn prem(&mut self, ctrl: &Mcb4bController) -> AsynResult<()> {
        let prem = self.strings.arm();
        if !prem.is_empty() {
            ctrl.command_bytes(&prem)?;
        }
        Ok(())
    }

    /// The record's `SET_VELOCITY`, `SET_ACCEL`, then the motion command.
    fn motion(&mut self, velocity: f64, acceleration: f64, cmd: String) -> AsynResult<()> {
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        self.set_velocity(&ctrl, velocity)?;
        self.set_accel(&ctrl, acceleration)?;
        self.prem(&ctrl)?;
        ctrl.command(&cmd)?;
        Ok(())
    }
}

/// The reply byte at `i`, `0` past its end.
fn byte_at(reply: &str, i: usize) -> u8 {
    reply.as_bytes().get(i).copied().unwrap_or(0)
}

impl AsynMotor for DrvMcb4bAxis {
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
        let cmd = format!("#{:02}G{:+}", self.axis, nint(position));
        self.motion(velocity, acceleration, cmd)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let cmd = format!("#{:02}I{:+}", self.axis, nint(distance));
        self.motion(velocity, acceleration, cmd)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The record sends SET_ACCEL then JOG; JOG is PREM and two messages.
        // C's second start_trans drops the POST armed on C=; it is kept here.
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        self.set_accel(&ctrl, acceleration)?;
        self.prem(&ctrl)?;
        let ival = nint((115200.0 / velocity).abs()).clamp(2, 65535);
        ctrl.command(&format!("#{:02}C={ival}", self.axis))?;
        let sign = if velocity > 0.0 { '+' } else { '-' };
        ctrl.command(&format!("#{:02}M{sign}1000000", self.axis))?;
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
        let sign = if forward { '+' } else { '-' };
        let cmd = format!("#{:02}H{sign}", self.axis);
        self.motion(velocity, acceleration, cmd)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.lock().command(&format!("#{:02}Q", self.axis))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.lock()
            .command(&format!("#{:02}P={:+}", self.axis, nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.lock()
            .command(&format!("#{:02}W={}", self.axis, i32::from(enable)))?;
        Ok(())
    }

    /// `PRIMITIVE`: INIT as its own message (devMCB4B.cc:190-196).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().command_bytes(init)?;
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

    /// `set_status`: `X` for done, `E` for the limits, `P` for the
    /// position. C parses a read that timed out from a stale buffer; here
    /// it reads as done with no limits, and the position stands.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let ask = |q: char| {
            ctrl.command(&format!("#{:02}{q}", self.axis))
                .unwrap_or_default()
        };
        let moving = ask('X');
        let limits = ask('E');
        let position = ask('P');

        let done = byte_at(&moving, 5) != b'1';
        let high_limit = byte_at(&limits, 5) == b'1';
        let low_limit = byte_at(&limits, 6) == b'1';
        // drvMCB4B.cc:264-271: POST, its reply read and discarded.
        if (done || high_limit || low_limit)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.command_bytes(&post);
        }
        drop(ctrl);
        if high_limit {
            self.direction = true;
        }
        if low_limit {
            self.direction = false;
        }
        let data = position.get(5..).map_or(self.position, atoi);
        if data != self.position {
            self.direction = data >= self.position;
            self.position = data;
        }
        Ok(MotorStatus {
            position: self.position as f64,
            encoder_position: self.position as f64,
            done,
            moving: !done,
            high_limit,
            low_limit,
            direction: self.direction,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn byte_past_reply_end_is_not_a_flag() {
        assert_eq!(byte_at("#01X=1", 5), b'1');
        assert_eq!(byte_at("", 5), 0);
    }

    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicBool, Ordering};

    /// Axis 1 of a card on a [`LineRig`]; `X` reports moving while `moving`.
    fn rig(name: &str) -> (LineRig, DrvMcb4bAxis, Arc<AtomicBool>) {
        let moving = Arc::new(AtomicBool::new(false));
        let m = moving.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "#01X" => format!("#01X={}", u8::from(m.load(Ordering::SeqCst))),
            "#01E" => "#01E=00".into(),
            "#01P" => "#01P=+0".into(),
            _ => "OK".into(),
        });
        let ctrl = Mcb4bController::new(io, MCB4B_NUM_AXES);
        (
            line,
            DrvMcb4bAxis::new(Arc::new(Mutex::new(ctrl)), 1),
            moving,
        )
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = rig("mcb4bInit");
        axis.primitive(&AsynUser::default(), b"#01W=1").unwrap();
        assert_eq!(line.sent(), ["#01W=1"]);
    }

    /// devMCB4B.cc:200-216 and drvMCB4B.cc:264-271.
    #[test]
    fn prem_precedes_the_move_and_post_follows_its_end() {
        let (line, mut axis, moving) = rig("mcb4bPrem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"#01W=1", Some(post(b"#01W=0")))
            .unwrap();
        axis.move_absolute(&user, 100.0, 0.0, 1000.0, 1000.0)
            .unwrap();
        assert_eq!(line.sent(), ["#01V=115", "#01R=1", "#01W=1", "#01G+100"]);
        moving.store(true, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["#01X", "#01E", "#01P"]);
        moving.store(false, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["#01X", "#01E", "#01P", "#01W=0"]);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["#01X", "#01E", "#01P"]);
    }

    #[test]
    fn jog_sends_prem_ahead_of_its_two_messages() {
        let (line, mut axis, _) = rig("mcb4bJog");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"P", None).unwrap();
        axis.move_velocity(&user, 0.0, -1152.0, 1000.0).unwrap();
        assert_eq!(line.sent(), ["#01R=1", "P", "#01C=100", "#01M-1000000"]);
    }
}
