//! The model-1 MCB-4B (`drvMCB4B.cc` + `devMCB4B.cc`): `MCB4BSetup` /
//! `MCB4BConfig`, records reaching axis `signal` of `card` through
//! `DTYP "ACS MCB-4B"`.
//!
//! The wire commands are the model-3 driver's except jog, which programs the
//! jog rate with `C=` and moves 1,000,000 steps with `M`, and the move
//! sequences, which carry no `W` power query.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::user::AsynUser;

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
}

impl DrvMcb4bAxis {
    pub fn new(controller: Arc<Mutex<Mcb4bController>>, axis: usize) -> Self {
        Self {
            controller,
            axis,
            position: 0,
            direction: false,
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

    /// The record's `SET_VELOCITY`, `SET_ACCEL`, then the motion command.
    fn motion(&self, velocity: f64, acceleration: f64, cmd: String) -> AsynResult<()> {
        let ctrl = self.lock();
        self.set_velocity(&ctrl, velocity)?;
        self.set_accel(&ctrl, acceleration)?;
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
        // The record sends SET_ACCEL then JOG; JOG is two messages.
        let ctrl = self.lock();
        self.set_accel(&ctrl, acceleration)?;
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

    /// `set_status`: `X` for done, `E` for the limits, `P` for the
    /// position. C parses a read that timed out from a stale buffer; here
    /// it reads as done with no limits, and the position stands.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let ask = |q: char| {
            ctrl.command(&format!("#{:02}{q}", self.axis))
                .unwrap_or_default()
        };
        let moving = ask('X');
        let limits = ask('E');
        let position = ask('P');
        drop(ctrl);

        let done = byte_at(&moving, 5) != b'1';
        let high_limit = byte_at(&limits, 5) == b'1';
        let low_limit = byte_at(&limits, 6) == b'1';
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
}
