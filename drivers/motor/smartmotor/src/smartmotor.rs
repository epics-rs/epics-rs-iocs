//! Animatics SmartMotor integrated servo controller driver (serial ASCII).
//!
//! Ported from `motorSmartMotor/smartMotorApp/src/drvSmartMotor.cc` +
//! `devSmartMotor.cc`. A SmartMotor is an integrated servo + drive; commands
//! use the Animatics language (`P=`, `D=`, `V=`, `A=`, `G`, `S`, `RP`, `RBt`,
//! …) and are terminated by the port output EOS `\n`; replies by input EOS
//! `\r` (`SmartMotorConfig` sets both, as C `motor_init`). Set commands produce no reply;
//! status is read back with `R*` query commands.
//!
//! ## Wire modes
//!
//! [`SmartMotorController::new`] auto-detects the mode by probing `RBe`, as C
//! `motor_init`:
//!
//! - **Single motor (no echo):** commands are sent verbatim and queries reply
//!   terminated by `\r`; one axis.
//! - **Daisy chain (echo on):** each command is prefixed with the binary
//!   address byte `129 + axis` and the chain echoes it terminated by `\n`
//!   before the `\r`-terminated response. Every transaction switches the port
//!   input EOS to `\n` to discard the echo and back to `\r` (C `send_mess`),
//!   and axes are counted up to the first that does not answer `RBe`.
//!
//! ## Units
//!
//! The SmartMotor works natively in encoder counts (`RP` returns counts, `P=`
//! takes counts) with no resolution scaling, so the asyn-rs motor boundary is
//! counts: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is counts.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `R*` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - `HOME_FOR`/`HOME_REV` and the PID-gain commands are unsupported in C
//!   (`build_trans` sends nothing and returns `ERROR`); [`home`](AsynMotor::home)
//!   returns an error and [`set_pid_gain`](AsynMotor::set_pid_gain) is a no-op.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 20;

/// Read timeout (C `recv_mess` 0.25 s); writes use the caller's timeout.
const RECV_TIMEOUT: Duration = Duration::from_millis(250);

/// Response terminator (C `input_terminator`).
const INPUT_EOS: &[u8] = b"\r";

/// Echo terminator in daisy-chain mode (C `echo_terminator`).
const ECHO_EOS: &[u8] = b"\n";

/// Command terminator (C `output_terminator`).
const OUTPUT_EOS: &[u8] = b"\n";

/// Most axes on one chain (C `MAX_AXIS`).
const MAX_AXIS: usize = 32;

/// Address byte of axis 0 in daisy-chain mode (C `SmartMotor_addr[0]`).
const FIRST_ADDR: u8 = 129;

/// Minimum acceleration the controller accepts (C forces `A >= 2`).
const MIN_ACCEL: i32 = 2;

/// Round an acceleration to the controller value, clamped to the minimum the
/// SmartMotor accepts (C overrides `A <= 1` to `2`).
fn clamped_accel(acceleration: f64) -> i32 {
    nint(acceleration).max(MIN_ACCEL)
}

fn smartmotor_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

fn eos_user() -> AsynUser {
    AsynUser::new(0).with_addr(0)
}

/// Shared controller endpoint: owns the serial port for one SmartMotor or a
/// daisy chain of them.
pub struct SmartMotorController {
    port: PortHandle,
    writer: SyncIOHandle,
    reader: SyncIOHandle,
    /// Daisy-chain echo mode (C `num_motors != 1` in `send_mess`).
    echo: bool,
    num_axes: usize,
    comm_state: CommState,
}

/// Controller communication state (C `cntrl->status`): one failed `RBt` is
/// retried silently, a second in a row is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

impl SmartMotorController {
    /// Connect and count the motors (C `motor_init`): set the port EOS, probe
    /// `RBe` on axis 0 bare, switch to echo mode if the command comes back
    /// echoed, then probe each axis address up to the first that does not
    /// answer `0`/`1` in three tries. Performs blocking serial I/O.
    pub fn new(port: PortHandle, timeout: Duration) -> AsynResult<Self> {
        port.set_output_eos_blocking(eos_user(), OUTPUT_EOS)?;
        port.set_input_eos_blocking(eos_user(), INPUT_EOS)?;
        let mut ctrl = Self {
            writer: SyncIOHandle::from_handle(port.clone(), 0, timeout),
            reader: SyncIOHandle::from_handle(port.clone(), 0, RECV_TIMEOUT),
            port,
            echo: false,
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        ctrl.flush();

        let mut total = 0;
        while total < MAX_AXIS {
            let mut answered = false;
            let mut retry = 0;
            while !answered && retry < 3 {
                let reply = ctrl.query(total, "RBe").unwrap_or_default();
                if total == 0 && !ctrl.echo && reply.starts_with("RBe") {
                    // Echo ON: start over with addressed commands.
                    ctrl.echo = true;
                    retry = 0;
                    ctrl.flush();
                }
                answered = reply.starts_with(['0', '1']);
                retry += 1;
            }
            if !answered {
                break;
            }
            total += 1;
            if !ctrl.echo {
                break;
            }
        }
        if total == 0 {
            return Err(smartmotor_err("SmartMotor: no valid response to RBe probe"));
        }
        ctrl.num_axes = total;
        Ok(ctrl)
    }

    /// Motors found by [`new`](Self::new): 1 without echo, else the chain length.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn flush(&self) {
        let user = AsynUser::new(0).with_addr(0).with_timeout(RECV_TIMEOUT);
        let _ = self.port.submit_blocking(RequestOp::Flush, user);
    }

    /// Write one command to `axis` (C `send_mess`). In echo mode it carries the
    /// axis address byte and the echo is read to `\n` and discarded, the input
    /// EOS restored to `\r` afterwards.
    fn send(&self, axis: usize, cmd: &str) -> AsynResult<()> {
        if !self.echo {
            self.writer.write_octet(0, cmd.as_bytes())?;
            return Ok(());
        }
        let mut out = Vec::with_capacity(cmd.len() + 1);
        out.push(FIRST_ADDR + axis as u8);
        out.extend_from_slice(cmd.as_bytes());
        self.port.set_input_eos_blocking(eos_user(), ECHO_EOS)?;
        let sent = self.writer.write_octet(0, &out);
        if sent.is_ok() {
            let _ = self.reader.read_octet(0, READ_BUF);
        }
        self.port.set_input_eos_blocking(eos_user(), INPUT_EOS)?;
        sent.map(|_| ())
    }

    /// Send a query to `axis` and return its `\r`-terminated reply (trimmed).
    fn query(&self, axis: usize, cmd: &str) -> AsynResult<String> {
        self.send(axis, cmd)?;
        let raw = self.reader.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }
}

/// One SmartMotor on the controller's port. Implements [`AsynMotor`].
pub struct SmartMotorAxis {
    controller: Arc<Mutex<SmartMotorController>>,
    axis: usize,
    prev_position: i32,
    last_status: MotorStatus,
}

impl SmartMotorAxis {
    /// Construct axis `axis` (0-based chain position) of a controller.
    pub fn new(controller: Arc<Mutex<SmartMotorController>>, axis: usize) -> Self {
        Self {
            controller,
            axis,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
        }
    }

    fn lock(&self) -> MutexGuard<'_, SmartMotorController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program speed/acceleration before a move (C sends `V=`/`A=` as separate
    /// transactions ahead of the move command). Non-positive values are skipped;
    /// acceleration is clamped to the controller minimum.
    fn program_speeds(
        ctrl: &SmartMotorController,
        axis: usize,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if velocity > 0.0 {
            ctrl.send(axis, &format!("V={}", nint(velocity)))?;
        }
        if acceleration > 0.0 {
            ctrl.send(axis, &format!("A={}", clamped_accel(acceleration)))?;
        }
        Ok(())
    }
}

impl AsynMotor for SmartMotorAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        SmartMotorAxis::program_speeds(&ctrl, self.axis, velocity, acceleration)?;
        ctrl.send(self.axis, &format!("P={}", nint(position)))?;
        ctrl.send(self.axis, "G")?;
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
        let ctrl = self.lock();
        SmartMotorAxis::program_speeds(&ctrl, self.axis, velocity, acceleration)?;
        ctrl.send(self.axis, &format!("D={}", nint(distance)))?;
        ctrl.send(self.axis, "G")?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG: "MV\rV=<v>\rG" — velocity mode, set signed velocity, go.
        let ctrl = self.lock();
        ctrl.send(self.axis, &format!("MV\rV={}\rG", nint(velocity)))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C: Animatics SmartMotors do not use home positions (build_trans ERROR).
        Err(smartmotor_err("SmartMotor: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send(self.axis, "S")?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send(self.axis, &format!("O={}", nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        if enable {
            // C ENABLE_TORQUE: position mode, hold at the current actual position.
            ctrl.send(self.axis, "MP\ra=@P\rP=a\rG")?;
        } else {
            // C DISABL_TORQUE: de-energize.
            ctrl.send(self.axis, "OFF")?;
        }
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C: SET_PGAIN/IGAIN/DGAIN send nothing and return ERROR — no-op here.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut ctrl = self.lock();

        // Trajectory-busy flag: RBt == 0 means the move is done. C
        // drvSmartMotor.cc:239-263: an empty or failed RBt keeps the previous
        // status once (RETRY); a second in a row raises COMM_ERR.
        let rbt = ctrl.query(self.axis, "RBt").ok().filter(|r| !r.is_empty());
        let Some(rbt) = rbt else {
            let hard = ctrl.comm_state != CommState::Normal;
            ctrl.comm_state = if hard {
                CommState::CommErr
            } else {
                CommState::Retry
            };
            drop(ctrl);
            if hard {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return Ok(self.last_status.clone());
        };
        ctrl.comm_state = CommState::Normal;
        let done = atoi(&rbt) == 0;

        let position = nint(atof(&ctrl.query(self.axis, "RP")?));

        let plus_ls = atoi(&ctrl.query(self.axis, "RBp")?) != 0;
        let minus_ls = atoi(&ctrl.query(self.axis, "RBm")?) != 0;

        // Latched right/left travel-limit flags: clear them if set (C Zr/Zl).
        if atoi(&ctrl.query(self.axis, "RBr")?) != 0 {
            ctrl.send(self.axis, "Zr")?;
        }
        if atoi(&ctrl.query(self.axis, "RBl")?) != 0 {
            ctrl.send(self.axis, "Zl")?;
        }

        // RBo != 0 means "not on commanded position" (C clears EA_POSITION).
        let on_position = atoi(&ctrl.query(self.axis, "RBo")?) == 0;

        let velocity_raw = atoi(&ctrl.query(self.axis, "RV")?);
        drop(ctrl);

        let direction = position >= self.prev_position;
        self.prev_position = position;

        let velocity = if direction {
            velocity_raw as f64
        } else {
            -(velocity_raw as f64)
        };

        let status = MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity,
            done,
            moving: !done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            direction,
            powered: on_position,
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        };
        self.last_status = status.clone();
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accel_clamps_to_minimum() {
        assert_eq!(clamped_accel(1.0), 2); // C overrides A <= 1 to 2
        assert_eq!(clamped_accel(0.0), 2);
        assert_eq!(clamped_accel(-5.0), 2);
        assert_eq!(clamped_accel(2.4), 2); // NINT rounds down
        assert_eq!(clamped_accel(2.5), 3); // NINT rounds up
        assert_eq!(clamped_accel(100.0), 100);
    }
}
