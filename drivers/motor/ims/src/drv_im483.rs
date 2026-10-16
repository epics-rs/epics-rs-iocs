//! The model-1 IM483 drivers (`drvIM483PL.cc` + `devIM483PL.cc`,
//! `drvIM483SM.cc` + `devIM483SM.cc`): records reach axis `signal` of `card`
//! through `DTYP "IM483PL"` or `"IM483SM"`.
//!
//! The party-line IM483PL prefixes every command with the axis name and a
//! space; the single-axis IM483SM sends it bare. Each command is answered by
//! one line, which is read and dropped.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

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
    fn ask(&self, msg: &str) -> String {
        self.handle
            .write_read(0, msg.as_bytes(), BUFF_SIZE)
            .map(|raw| String::from_utf8_lossy(&raw).into_owned())
            .unwrap_or_default()
    }
}

/// `motor_init`'s probe of an IM483PL: ` Z 0` to each axis in turn, three
/// tries each, until one does not answer. The number that did.
pub fn im483pl_probe(ctrl: &Im483Controller) -> usize {
    IM483PL_AXIS
        .iter()
        .take_while(|name| (0..3).any(|_| !ctrl.ask(&format!("{name} Z 0")).is_empty()))
        .count()
}

/// `motor_init`'s probe of an IM483SM, three tries: reset with ETX, wait a
/// second, send a space and read the sign-on. True when it answered; the
/// rest of the sign-on, up to `BUFF_SIZE` characters, is read and dropped,
/// as C keeps it only for `dbior`.
pub fn im483sm_probe(ctrl: &Im483Controller) -> bool {
    (0..3).any(|_| {
        let _ = ctrl.handle.write_octet(0, b"\x03");
        sleep(Duration::from_secs(1));
        let first = ctrl.ask(" ");
        if first.is_empty() {
            return false;
        }
        let mut read = first.len();
        while read < BUFF_SIZE {
            match ctrl.handle.read_octet(0, BUFF_SIZE) {
                Ok(line) if !line.is_empty() => read += line.len(),
                _ => break,
            }
        }
        true
    })
}

/// One axis of a model-1 IM483 card.
pub struct DrvIm483Axis {
    controller: Arc<Mutex<Im483Controller>>,
    /// The axis name and its space, prefixed to every command.
    prefix: String,
    /// `motor_info->status`, which `set_status` starts from.
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
}

impl DrvIm483Axis {
    /// Axis `signal` of an IM483PL, which C takes to carry an encoder.
    pub fn im483pl(controller: Arc<Mutex<Im483Controller>>, signal: usize) -> Self {
        Self {
            controller,
            prefix: format!("{} ", IM483PL_AXIS[signal]),
            status: MotorStatus {
                has_encoder: true,
                ..MotorStatus::default()
            },
            position: 0,
        }
    }

    /// The one axis of an IM483SM, which C takes to have no encoder.
    pub fn im483sm(controller: Arc<Mutex<Im483Controller>>) -> Self {
        Self {
            controller,
            prefix: String::new(),
            status: MotorStatus::default(),
            position: 0,
        }
    }

    fn lock(&self) -> MutexGuard<'_, Im483Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send each command of a transaction, reading its reply.
    fn send(&self, commands: &[String]) -> AsynResult<()> {
        let ctrl = self.lock();
        for command in commands {
            ctrl.ask(&format!("{}{command}", self.prefix));
        }
        Ok(())
    }

    /// The record's `SET_VELOCITY` and `SET_VEL_BASE`; `SET_ACCEL` sends
    /// nothing.
    fn speeds(min_velocity: f64, velocity: f64) -> [String; 2] {
        [format!("V{velocity:.2};"), format!("I{min_velocity:.2};")]
    }
}

impl AsynMotor for DrvIm483Axis {
    /// `poll` never fails: `set_status`'s missed replies are handled in it.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
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
        self.send(&[v, i, format!("R{position:.2}")])
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
        self.send(&[v, i, format!("{distance:+.2}")])
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.send(&[format!("I{min_velocity:.2};"), format!("M{velocity:.2};")])
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
        self.send(&[v, i, format!("F1000 {way}")])
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

    /// `set_status`: `^` for moving, `Z 0` for the position, `] 0` for the
    /// limits, `] 1` for home, `z 0` for the encoder. A first unanswered `^`
    /// keeps the last status; a second raises the comms error.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let ask = |ctrl: &Im483Controller, q: &str| ctrl.ask(&format!("{}{q}", self.prefix));
        let moving = ask(&ctrl, "^");
        if moving.is_empty() {
            if ctrl.comm == CommState::Normal {
                ctrl.comm = CommState::Retry;
            } else {
                ctrl.comm = CommState::CommErr;
                self.status.comms_error = true;
                self.status.problem = true;
            }
            return Ok(self.status.clone());
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
        status.moving = !status.done;
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
        Ok(status.clone())
    }
}

/// The reply from byte `i`, as C's `&buff[i]`; empty past its end.
fn from(reply: &str, i: usize) -> &str {
    reply.get(i..).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reply_offset_past_end_reads_as_empty() {
        assert_eq!(from("A ^ 1", 4), "1");
        assert_eq!(from("A ^", 4), "");
    }
}
