//! The model-1 MDrive driver (`drvMDrive.cc` + `devMDrive.cc`): records
//! reach drive `signal` of chain `card` through `DTYP "MDrive"`.
//!
//! Every command is prefixed by the drive's party-mode name, `1` to `8`.
//! Commands get no reply; `PR` queries get one line.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_printf;

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
        self.handle
            .write_read(0, msg.as_bytes(), BUFF_SIZE)
            .map(|raw| String::from_utf8_lossy(&raw).into_owned())
            .unwrap_or_default()
    }

    /// `recv_mess` alone: the next line, empty when none came.
    fn read(&self) -> String {
        self.handle
            .read_octet(0, BUFF_SIZE)
            .map(|raw| String::from_utf8_lossy(&raw).into_owned())
            .unwrap_or_default()
    }

    fn send(&self, msg: &str) {
        let _ = self.handle.write_octet(0, msg.as_bytes());
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
pub struct MDriveDrive {
    encoder: bool,
    inputs: Inputs,
}

/// `motor_init`'s probe: `PR VR` to each drive in turn, three tries each,
/// until one does not answer; then each answering drive's encoder and
/// switch inputs.
pub fn mdrive_probe(ctrl: &MDriveController, card: i32) -> Vec<MDriveDrive> {
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
    (0..found)
        .map(|index| {
            let name = MDRIVE_AXIS[index];
            let encoder = ident.ends_with('E') || {
                let part = ctrl.ask(&format!("{name}PR PN"));
                ["E", "EQ", "EE", "EJM", "EJM-N"]
                    .iter()
                    .any(|suffix| part.ends_with(suffix))
            };
            // C reads one more line here, waiting out its timeout when the
            // drive sends none.
            ctrl.read();
            let inputs = read_inputs(ctrl, name);
            if inputs.minus == 0 || inputs.plus == 0 {
                let which = if inputs.minus == 0 { "Minus" } else { "Plus" };
                errlog_printf(&format!(
                    "MDrive chain #{card}, motor #{index} {which} LS not configured.\n"
                ));
            }
            MDriveDrive { encoder, inputs }
        })
        .collect()
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
    /// `motor_info->status`, which `set_status` starts from.
    status: MotorStatus,
    /// `motor_info->position`.
    position: i32,
    /// MRES/ERES, from `SET_ENC_RATIO`: `LOAD_POS` loads the encoder with
    /// DVAL/ERES.
    encoder_ratio: f64,
}

impl DrvMDriveAxis {
    pub fn new(
        controller: Arc<Mutex<MDriveController>>,
        signal: usize,
        drive: MDriveDrive,
    ) -> Self {
        Self {
            controller,
            name: MDRIVE_AXIS[signal],
            inputs: drive.inputs,
            status: MotorStatus {
                has_encoder: drive.encoder,
                gain_support: drive.encoder,
                ..MotorStatus::default()
            },
            position: 0,
            encoder_ratio: 1.0,
        }
    }

    fn lock(&self) -> MutexGuard<'_, MDriveController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send each command of a transaction.
    fn send(&self, commands: &[String]) -> AsynResult<()> {
        let ctrl = self.lock();
        for command in commands {
            ctrl.send(&format!("{}{command}", self.name));
        }
        Ok(())
    }

    /// `SET_ACCEL`, which the record sends only for a positive rate, then
    /// `D=A`.
    fn accel(commands: &mut Vec<String>, acceleration: f64) {
        if acceleration > 0.0 {
            commands.push(format!("A={}", nint(acceleration)));
            commands.push("D=A".to_string());
        }
    }

    /// The record's `SET_VELOCITY`, `SET_VEL_BASE`, `SET_ACCEL`, then
    /// `motion`.
    fn motion(
        &self,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        motion: String,
    ) -> AsynResult<()> {
        let mut commands = vec![
            format!("VM={}", nint(velocity)),
            format!("VI={}", nint(min_velocity)),
        ];
        Self::accel(&mut commands, acceleration);
        commands.push(motion);
        self.send(&commands)
    }
}

impl AsynMotor for DrvMDriveAxis {
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
        acceleration: f64,
    ) -> AsynResult<()> {
        let motion = format!("MA {}", nint(position));
        self.motion(min_velocity, velocity, acceleration, motion)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let motion = format!("MR {}", nint(distance));
        self.motion(min_velocity, velocity, acceleration, motion)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let mut commands = vec![format!("VI={}", nint(min_velocity))];
        Self::accel(&mut commands, acceleration);
        commands.push(format!("SL={}", nint(velocity)));
        self.send(&commands)
    }

    /// devMDrive.cc sends the IM483's home command, ` F1000 0|1`, as is.
    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let way = if forward { 0 } else { 1 };
        self.motion(
            min_velocity,
            velocity,
            acceleration,
            format!(" F1000 {way}"),
        )
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
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

    /// `set_status`: `PR MV` for moving, `PR P` for the position, `PR I<n>`
    /// for each wired switch, `PR C2` for the encoder and `PR V` for the
    /// velocity. A first unanswered `PR MV` keeps the last status; a second
    /// raises the comms error.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let name = self.name;
        let ask = |ctrl: &MDriveController, q: &str| ctrl.ask(&format!("{name}{q}"));
        let moving = ask(&ctrl, "PR MV");
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
        let position = ask(&ctrl, "PR P");
        let input = |n: u8| atoi(&ask(&ctrl, &format!("PR I{n}"))) as u8 != 0;
        let limits = (self.inputs.plus != 0 && self.inputs.minus != 0)
            .then(|| (input(self.inputs.plus), input(self.inputs.minus)));
        let home = self.inputs.home != 0 && input(self.inputs.home);
        let encoder = self.status.has_encoder.then(|| ask(&ctrl, "PR C2"));
        let velocity = ask(&ctrl, "PR V");
        drop(ctrl);

        let status = &mut self.status;
        status.comms_error = false;
        status.done = atoi(&moving) == 0;
        status.moving = !status.done;
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
        Ok(status.clone())
    }
}
