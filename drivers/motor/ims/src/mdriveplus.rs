//! IMS MDrivePlus / MForce / Lexium motor driver (ASCII, over an asyn octet
//! port).
//!
//! Ported from `motorIms/imsApp/src/ImsMDrivePlusMotorController.cpp` +
//! `ImsMDrivePlusMotorAxis.cpp` (the "model 3" asynMotorController /
//! asynMotorAxis driver for Intelligent Motion Systems MDrivePlus M17/M23/M34).
//!
//! The controller speaks the IMS MCode ASCII language. Each controller drives a
//! **single axis** (C `NUM_AXES = 1`), optionally in "party mode" where a device
//! name (`DN`) is prepended to every command to address one drive on a multidrop
//! bus. Set commands (`VM=`, `MA`, …) get no reply; queries (`PR X`) return the
//! value terminated by `\n`.
//!
//! ## Framing
//!
//! The port EOS is set at connect as C does ([`INPUT_EOS`]/[`OUTPUT_EOS`]); a
//! Lexium MDrive (`LMD`) then switches the output EOS to `\r` without a device
//! name or `\n` with one (C `readHomeAndLimitConfig` after `PR VR`). The
//! device name, when set, is prepended to every command with no separator.
//!
//! ## Units
//!
//! MCode positions, velocities and accelerations are integer motor steps (the C
//! casts every value to `(long)`). There is no resolution scaling, so the driver
//! works in native steps with `MRES` = 1.
//!
//! ## Not modeled (documented)
//!
//! - **Velocity / direction / power-on readback.** The C poller never sets
//!   `motorStatusActualVelocity`, `motorStatusDirection` or `motorStatusPowerOn`,
//!   so those are left at their defaults here too.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{ControllerParams, ParamWrite};
use epics_rs::motor::trace::MotorTrace;

use motor_common::util::{atof, atoi};

/// Reply read buffer size (C `MAX_BUFF_LEN`).
const READ_BUF: usize = 80;

/// Port EOS set at connect (ImsMDrivePlusMotorController.cpp:81-82).
pub const INPUT_EOS: &[u8] = b"\n";
pub const OUTPUT_EOS: &[u8] = b"\r\n";

/// `PR IS` read buffer and timeout (C `resp[1024]`, `IMS_TIMEOUT`).
const IS_READ_BUF: usize = 1024;
const IS_READ_TIMEOUT: Duration = Duration::from_secs(2);

/// C `DRIVER_NAME`, the prefix of the driver's console lines.
pub(crate) const DRIVER_NAME: &str = "ImsMDrivePlusMotorDriver";

/// Rule under the setup-config banner lines.
const RULE: &str = "-------------------------------------------";

/// `(input, function)` per line of a `PR IS` block: C turns every non-digit
/// into a separator and scans `%d %d %d` (input, function, active level) from
/// each LF-terminated line; text after the last LF is not scanned.
fn parse_input_setup(block: &[u8]) -> Vec<(i32, i32)> {
    let mut lines: Vec<&[u8]> = block.split(|&b| b == b'\n').collect();
    lines.pop();
    lines
        .into_iter()
        .filter_map(|line| {
            let mut nums = line
                .split(|b| !b.is_ascii_digit())
                .filter(|f| !f.is_empty())
                .map(|f| std::str::from_utf8(f).ok()?.parse::<i32>().ok());
            match (nums.next(), nums.next(), nums.next()) {
                (Some(Some(input)), Some(Some(function)), Some(Some(_))) => Some((input, function)),
                _ => None,
            }
        })
        .collect()
}

fn ims_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Build the on-wire command: device name (party mode) + command.
fn frame(device_name: &str, cmd: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(device_name.len() + cmd.len());
    out.extend_from_slice(device_name.as_bytes());
    out.extend_from_slice(cmd.as_bytes());
    out
}

/// The controller's IO port user and the device name C's
/// `writeController` / `writeReadController` prefix every command with,
/// shared by the axis and the controller port's IMS_* writes.
pub struct ImsLine {
    handle: SyncIOHandle,
    /// Device name (`DN`) prepended to every command for party mode; empty when
    /// not in party mode.
    device_name: String,
}

impl ImsLine {
    pub fn new(handle: SyncIOHandle, device_name: String) -> Arc<Self> {
        Arc::new(Self {
            handle,
            device_name,
        })
    }

    fn framed(&self, cmd: &str) -> Vec<u8> {
        frame(&self.device_name, cmd)
    }

    /// Write a set command; the controller sends no reply (C `writeController`).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &self.framed(cmd))?;
        Ok(())
    }

    /// Write a query and read one reply line (C `writeReadController`:
    /// `pasynOctetSyncIO->writeRead`, which flushes a stray line first).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, &self.framed(cmd), READ_BUF)?;
        let s = String::from_utf8_lossy(&raw);
        Ok(s.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }
}

/// ImsMDrivePlusMotorController's parameters (ImsMDrivePlusMotorController.cpp:
/// 85-89) and its `writeInt32` (:310-347), which handles them without the
/// base.
pub struct ImsParams {
    line: Arc<ImsLine>,
    save_to_nvm: Option<usize>,
    clear_lr_error: Option<usize>,
}

/// One of C `writeInt32`'s own functions: the command a 1 sends, and the
/// lines it prints.
struct Action {
    command: &'static str,
    done: &'static str,
    failed: &'static str,
    not_one: &'static str,
}

const SAVE_TO_NVM: Action = Action {
    command: "S",
    done: "Successfully saved to NVM",
    failed: "ERROR saving to NVM",
    not_one: "ERROR, value of 1 to save to NVM",
};

const CLEAR_LR_ERROR: Action = Action {
    command: "CF",
    done: "Successfully cleared LR error",
    failed: "ERROR clearing LR error",
    not_one: "ERROR, value of 1 to clear LR error",
};

impl ImsParams {
    pub fn new(line: Arc<ImsLine>) -> Self {
        Self {
            line,
            save_to_nvm: None,
            clear_lr_error: None,
        }
    }
}

impl ControllerParams for ImsParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.save_to_nvm = Some(base.create_param("IMS_SAVETONVM", ParamType::Int32)?);
        // Created and stored, never acted on: "NOT SUPPORTED YET"
        // (ImsMDrivePlusMotorController.h:56-57).
        base.create_param("IMS_LOADMCODE", ParamType::Octet)?;
        base.create_param("IMS_CLEARMCODE", ParamType::Octet)?;
        base.create_param("IMS_LOCKEDROTOR", ParamType::Int32)?;
        self.clear_lr_error = Some(base.create_param("IMS_CLEARLRERROR", ParamType::Int32)?);
        Ok(())
    }

    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        base.trace_print(
            TraceMask::IO_DRIVER,
            format_args!("{DRIVER_NAME}:writeInt32: val={value}\n"),
        );
        let action = match Some(user.reason) {
            r if r == self.save_to_nvm => SAVE_TO_NVM,
            r if r == self.clear_lr_error => CLEAR_LR_ERROR,
            _ => return ParamWrite::PassToBase(Ok(())),
        };
        let addr = match base.get_address(user.addr, user.reason) {
            Ok(addr) => addr,
            Err(e) => return ParamWrite::Handled(Err(e)),
        };
        let _ = base.set_int32_param(user.reason, addr, value);
        let status = if value == 1 {
            let status = self.line.write_only(action.command);
            let (mask, line) = match &status {
                Ok(()) => (TraceMask::FLOW, action.done),
                Err(_) => (TraceMask::ERROR, action.failed),
            };
            base.trace_print(mask, format_args!("{DRIVER_NAME}:writeInt32: {line}\n"));
            status
        } else {
            let not_one = action.not_one;
            base.trace_print(
                TraceMask::ERROR,
                format_args!("{DRIVER_NAME}:writeInt32: {not_one}\n"),
            );
            Ok(())
        };
        let _ = base.call_param_callbacks(addr);
        ParamWrite::Handled(status)
    }
}

/// One IMS MDrivePlus controller/axis (1:1, C `NUM_AXES = 1`). Implements
/// [`AsynMotor`].
pub struct ImsMDrivePlusAxis {
    line: Arc<ImsLine>,
    /// I/O input number configured as the home switch (C `homeSwitchInput`).
    home_switch: Option<i32>,
    /// I/O input number configured as the positive-limit switch.
    pos_limit_switch: Option<i32>,
    /// I/O input number configured as the negative-limit switch.
    neg_limit_switch: Option<i32>,
    has_encoder: bool,
    /// True between a `home` command and the move completing (drives the
    /// `R1=1` "homed" latch in [`Self::poll`]).
    homing: bool,
    /// The controller port and its IMS_LOCKEDROTOR, which the poll sets
    /// (ImsMDrivePlusMotorAxis.cpp:484).
    locked_rotor: Option<(PortHandle, usize)>,
}

impl ImsMDrivePlusAxis {
    /// Connect an axis on `line`. The line's octet port is the one whose
    /// output EOS a Lexium MDrive switches. As in C, a failed configuration is logged and the
    /// axis is still created. `motor_name` is the controller port, whose
    /// `pasynUserSelf` C prints these lines on.
    pub fn new(line: Arc<ImsLine>, motor_name: &str) -> Self {
        let trace = MotorTrace::new(motor_name, 0);
        let mut axis = Self {
            line,
            home_switch: None,
            pos_limit_switch: None,
            neg_limit_switch: None,
            has_encoder: false,
            homing: false,
            locked_rotor: None,
        };
        if axis.config_axis(&trace) == AsynStatus::Error {
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "{DRIVER_NAME}:ImsMDrivePlusMotorAxis(): controller config failed for motor port={motor_name}\n"
                ),
            );
        }
        axis.read_home_and_limit_config();
        axis
    }

    /// Set IMS_LOCKEDROTOR on the controller port `port` from each poll.
    pub fn post_locked_rotor_to(&mut self, port: PortHandle) -> AsynResult<()> {
        let io = SyncIOHandle::from_handle(port.clone(), 0, IS_READ_TIMEOUT);
        self.locked_rotor = Some((port, io.drv_user_create("IMS_LOCKEDROTOR")?));
        Ok(())
    }

    fn framed(&self, cmd: &str) -> Vec<u8> {
        self.line.framed(cmd)
    }

    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.line.write_only(cmd)
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.line.write_read(cmd)
    }

    /// C `configAxis`: up to three `PR VR` probes, then the `PR EE` encoder
    /// flag. An asyn error is logged and retried; any other reply shorter than
    /// two bytes ends the configuration without `PR EE`.
    fn config_axis(&mut self, trace: &MotorTrace) -> AsynStatus {
        for _ in 0..3 {
            let resp = match self
                .line
                .handle
                .write_read(0, &self.framed("PR VR"), READ_BUF)
            {
                Err(e) if e.status() == AsynStatus::Error => {
                    trace.print_self(
                        TraceMask::ERROR,
                        format_args!("{DRIVER_NAME}:configAxis(): Version inquiry FAILED.\n"),
                    );
                    continue;
                }
                Err(e) => e.partial_read().map(|p| p.data.clone()).unwrap_or_default(),
                Ok(raw) => raw,
            };
            let resp = resp.split(|&b| b == 0).next().unwrap_or_default();
            if resp.len() < 2 {
                trace.print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "{DRIVER_NAME}:configAxis(): Version inquiry FAILED version={}.\n",
                        String::from_utf8_lossy(resp)
                    ),
                );
                return AsynStatus::Error;
            }
            break;
        }
        match self.write_read("PR EE") {
            Ok(resp) => {
                self.has_encoder = atoi(&resp) != 0;
                AsynStatus::Success
            }
            Err(e) => e.status(),
        }
    }

    /// C `readHomeAndLimitConfig`, console lines included.
    fn read_home_and_limit_config(&mut self) {
        // MForce 1 answers `PR VR` with an error, so its status is ignored.
        let version = self.write_read("PR VR").unwrap_or_default();
        if !version.is_empty() {
            println!("Controller version {version}");
        }
        println!("Setup config:");
        let vlow = version.to_lowercase();
        if vlow.contains("lmm") || vlow.contains("lmd") {
            if vlow.contains("lmm") {
                println!("Lexium Motor Module detected");
                println!("{RULE}");
            } else {
                // Lexium MDrive: output EOS depends on party mode.
                println!("Lexium MDrive detected");
                println!("{RULE}");
                let eos: &[u8] = if self.line.device_name.is_empty() {
                    println!("Have no device name, line ending CR.");
                    b"\r"
                } else {
                    println!("Have device name going party modus, line ending LF.");
                    b"\n"
                };
                if let Ok(port) = self.line.handle.port() {
                    let _ = port.set_output_eos_blocking(AsynUser::new(0).with_addr(0), eos);
                }
            }
            let block = self.read_input_setup().unwrap_or_default();
            let text = block.split(|&b| b == 0).next().unwrap_or_default();
            println!("{}", String::from_utf8_lossy(text));
            for (input_no, function) in parse_input_setup(&block) {
                self.set_switch(function, input_no);
            }
        } else {
            // MForce: read the S1-S4 I/O setup and map any home/limit inputs.
            println!("MForce 1 driver detected");
            println!("{RULE}");
            println!("Checking setup for home, pos and neg limit switches:");
            for input_no in 1..=4 {
                let resp = self
                    .write_read(&format!("PR S{input_no}"))
                    .unwrap_or_default();
                println!("{resp}");
                self.set_switch(atoi(&resp), input_no);
            }
        }
        println!(
            "    HOME limit switch input line: {}",
            self.home_switch.unwrap_or(-1)
        );
        println!(
            "POSITIVE limit switch input line: {}",
            self.pos_limit_switch.unwrap_or(-1)
        );
        println!(
            "NEGATIVE limit switch input line: {}",
            self.neg_limit_switch.unwrap_or(-1)
        );
        println!("{RULE}");
    }

    /// Lexium `PR IS`: the whole I/O-setup block, read with a NUL input EOS
    /// under the port lock so the read runs to the timeout and gathers every
    /// line (ImsMDrivePlusMotorController.cpp:221-237). The input EOS is put
    /// back to `\n` as C does, on every path.
    fn read_input_setup(&self) -> AsynResult<Vec<u8>> {
        let port = self.line.handle.port()?;
        static NEXT_TOKEN: AtomicU64 = AtomicU64::new(1 << 48);
        let token = NEXT_TOKEN.fetch_add(1, Ordering::Relaxed);
        let user = || {
            let mut u = AsynUser::new(0).with_addr(0).with_timeout(IS_READ_TIMEOUT);
            u.block_token = Some(token);
            u
        };
        port.submit_blocking(RequestOp::BlockProcess { all_devices: true }, user())?;
        let read = (|| {
            port.set_input_eos_blocking(user(), b"\0")?;
            port.submit_blocking(
                RequestOp::OctetWrite {
                    data: self.framed("PR IS"),
                },
                user(),
            )?;
            match port.submit_blocking(
                RequestOp::OctetRead {
                    buf_size: IS_READ_BUF,
                },
                user(),
            ) {
                Ok(r) => Ok(r.data.unwrap_or_default()),
                Err(e) => e.partial_read().map(|p| p.data.clone()).ok_or(e),
            }
        })();
        let restored = port.set_input_eos_blocking(user(), INPUT_EOS);
        let unblocked =
            port.submit_blocking(RequestOp::UnblockProcess { all_devices: true }, user());
        let block = read?;
        restored?;
        unblocked?;
        Ok(block)
    }

    /// Map an I/O setup type code to the switch input it configures (C
    /// `set_switch_vars`: 1 = home, 2 = positive limit, 3 = negative limit).
    fn set_switch(&mut self, type_code: i32, input_no: i32) {
        let (slot, name) = match type_code {
            1 => (&mut self.home_switch, "HOME"),
            2 => (&mut self.pos_limit_switch, "POSITIVE"),
            3 => (&mut self.neg_limit_switch, "NEGATIVE"),
            _ => return,
        };
        *slot = Some(input_no);
        println!(
            "Setup type {type_code} found: {name} limit switch input line is set to {input_no}"
        );
    }

    /// Set base velocity (`VI`, only if > 0), max velocity (`VM`), acceleration
    /// and deceleration (`A`/`D`, only if non-zero), then reset the stall flag
    /// (`ST=0`). C `setAxisMoveParameters`.
    fn set_move_params(
        &self,
        min_velocity: f64,
        max_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if min_velocity > 0.0 {
            if min_velocity > max_velocity {
                return Err(ims_err(
                    "ims: base velocity cannot be greater than max velocity",
                ));
            }
            self.write_only(&format!("VI={}", min_velocity as i64))?;
        }
        self.write_only(&format!("VM={}", max_velocity as i64))?;
        if acceleration != 0.0 {
            self.write_only(&format!("A={}", acceleration as i64))?;
            self.write_only(&format!("D={}", acceleration as i64))?;
        }
        self.write_only("ST=0")?;
        Ok(())
    }

    fn do_move(
        &self,
        position: f64,
        relative: bool,
        min_velocity: f64,
        max_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.set_move_params(min_velocity, max_velocity, acceleration)?;
        let cmd = if relative {
            format!("MR {}", position as i64)
        } else {
            format!("MA {}", position as i64)
        };
        self.write_only(&cmd)
    }
}

impl AsynMotor for ImsMDrivePlusAxis {
    /// C `ImsMDrivePlusMotorAxis::handleAxisError` sets only
    /// `motorStatusProblem_`.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, min_velocity, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, min_velocity, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C jogs with |maxVelocity| for the parameters (JOGR passes a negative
        // velocity) but the signed value in the SL slew command.
        self.set_move_params(min_velocity, velocity.abs(), acceleration)?;
        self.write_only(&format!("SL {}", velocity as i64))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C reads `PR VI` to ensure a non-zero creep velocity when min_velocity
        // is 0, but never re-sends it — the creep relies on the drive's stored
        // VI. This port matches that (does not re-send VI).
        self.set_move_params(min_velocity, velocity, acceleration)?;
        // HM 3 homes in the plus direction, HM 1 in the minus direction.
        let direction = if forward { 3 } else { 1 };
        self.write_only(&format!("HM {direction}"))?;
        // Reset the "homed" latch (R1); poll sets it once the move completes.
        self.write_only("R1=0")?;
        self.homing = true;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        if acceleration != 0.0 {
            self.write_only(&format!("D={}", acceleration as i64))?;
        }
        self.write_only("SL 0")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.write_only(&format!("P={}", position as i64))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let position = atof(&self.write_read("PR P")?);
        let moving = atoi(&self.write_read("PR MV")?) != 0;

        // On the falling edge of a home move, latch R1 = 1 (homed).
        if self.homing && !moving {
            self.write_only("R1=1")?;
            self.homing = false;
        }

        // Home / limit switch inputs (only those configured on this drive).
        let encoder_home = match self.home_switch {
            Some(n) => atoi(&self.write_read(&format!("PR I{n}"))?) != 0,
            None => false,
        };
        let high_limit = match self.pos_limit_switch {
            Some(n) => atoi(&self.write_read(&format!("PR I{n}"))?) != 0,
            None => false,
        };
        let low_limit = match self.neg_limit_switch {
            Some(n) => atoi(&self.write_read(&format!("PR I{n}"))?) != 0,
            None => false,
        };

        let homed = atoi(&self.write_read("PR R1")?) > 0;
        // ST is the stall flag; C maps it to both following-error and slip. Only
        // slip/stall has a MotorStatus field, so both fold into slip_stall.
        let stall = atoi(&self.write_read("PR ST")?) > 0;
        let encoder_position = atof(&self.write_read("PR C2")?);
        let locked_rotor = atoi(&self.write_read("PR LR")?);
        if let Some((port, reason)) = &self.locked_rotor {
            let value = ParamSetValue::new(*reason, 0, ParamValue::Int32(locked_rotor));
            let _ = port.set_params_and_notify_blocking(0, vec![value]);
        }

        Ok(MotorStatus {
            position,
            encoder_position,
            done: !moving,
            moving,
            high_limit,
            low_limit,
            // C sets motorStatusHome_ (EA_HOME) from the home switch input.
            encoder_home,
            homed,
            slip_stall: stall,
            has_encoder: self.has_encoder,
            gain_support: self.has_encoder,
            // IMS honours a base velocity (VI).
            vbas_supported: true,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;

    /// ImsMDrivePlusMotorController.cpp:69-99: a missing IO port still builds
    /// the axis; configAxis and readHomeAndLimitConfig run on the dead user.
    #[test]
    fn an_axis_of_a_missing_port_is_still_built() {
        let (handle, status) = SyncIOHandle::connect_port(
            "IMS_NONE",
            0,
            Duration::from_millis(10),
            InterfaceType::Octet,
        );
        assert!(status.is_err());
        let axis = ImsMDrivePlusAxis::new(ImsLine::new(handle, String::new()), "IMS_MOTOR");
        assert!(!axis.has_encoder);
        assert_eq!(
            (
                axis.home_switch,
                axis.pos_limit_switch,
                axis.neg_limit_switch
            ),
            (None, None, None)
        );
    }

    #[test]
    fn frame_prepends_device_name() {
        // Party mode: device name prepended with no separator.
        assert_eq!(frame("M06", "MA 100"), b"M06MA 100");
        assert_eq!(frame("", "PR P"), b"PR P");
    }

    #[test]
    fn input_setup_scans_each_lf_terminated_line() {
        // Non-digits are separators; the unterminated tail is not scanned.
        let block = b"IS = 1, 1, 0\r\nIS = 2, 2, 1\r\nIS = 3, 0, 0\r\nIS = 4, 3";
        assert_eq!(parse_input_setup(block), vec![(1, 1), (2, 2), (3, 0)]);
        assert_eq!(parse_input_setup(b"junk\n"), vec![]);
    }
}
