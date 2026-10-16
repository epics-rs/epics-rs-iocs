//! The model-1 Micos MoCo dc driver (`drvMicos.cc` + `devMicos.cc`):
//! records reach MoCo address `signal` on the serial line of `card` through
//! `DTYP "Micos MoCo"`.
//!
//! Every command is `^A`, the address, then the mnemonic. Commands get no
//! reply; the `ts` and `tp` queries get one line.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

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
        self.handle
            .write_read(0, msg.as_bytes(), BUFF_SIZE)
            .map(|raw| String::from_utf8_lossy(&raw).into_owned())
            .unwrap_or_default()
    }

    fn send(&self, msg: &str) {
        let _ = self.handle.write_octet(0, msg.as_bytes());
    }
}

/// `motor_init`: `ts` to each of the `num_axis` addresses, three tries
/// each; then echo off (`ef`) and stop (`ab1`) on every one. False when an
/// address never answered, which leaves the whole card unconfigured.
///
/// C counts the tries across all addresses, so after the first three only
/// one try is left per address; each gets three here.
pub fn micos_probe(ctrl: &MicosController, num_axis: i32) -> bool {
    let answered =
        (0..num_axis).all(|n| (0..3).any(|_| !ctrl.ask(&format!("{CTLA}{n}ts")).is_empty()));
    if answered {
        for n in 0..num_axis {
            ctrl.send(&format!("{CTLA}{n}ef"));
            ctrl.send(&format!("{CTLA}{n}ab1"));
        }
    }
    answered
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
}

impl DrvMicosAxis {
    pub fn new(controller: Arc<Mutex<MicosController>>, axis: i32) -> Self {
        Self {
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
        }
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

    /// `SET_ACCEL`, which the record sends only for a positive rate.
    fn accel(commands: &mut Vec<String>, acceleration: f64) {
        if acceleration > 0.0 {
            commands.push(format!("da{},ud", clamp(acceleration, 1_000_000)));
        }
    }

    /// The record's `SET_VELOCITY`, `SET_ACCEL` (`SET_VEL_BASE` sends
    /// nothing), then `motion`.
    fn motion(&self, velocity: f64, acceleration: f64, motion: String) -> AsynResult<()> {
        let mut commands = vec![format!("dv{},ud", clamp(velocity, 1_000_000))];
        Self::accel(&mut commands, acceleration);
        commands.push(motion);
        self.send(&commands)
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
        commands.push(format!("ma{}", limit.trunc() as i64));
        self.send(&commands)
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
        status.moving = !status.done;
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
}
