//! RTDE control interface — the control-script command channel.
//!
//! Ported from `ur_rtde/src/rtde_control_interface.cpp`. The control interface
//! uploads a URScript (`rtde_control.script`) that spins on
//! `input_int_register_0`; every command is written into the input registers and
//! the script answers on `output_int_register_0`
//! (1 = ready for a command, 2 = done with the command).
//!
//! Only the commands urRobot's `RTDEControl` driver issues are ported; all 23
//! input recipes are still registered, because the controller numbers recipes by
//! registration order and the ones we use (1, 4, 6, 8, 13, 19) sit in the middle
//! of that sequence.

use std::time::{Duration, Instant};

use crate::dashboard::DashboardClient;
use crate::error::{UrError, UrResult, verify_within};
use crate::rtde::{CommandType, ControllerVersion, Payload, RobotCommand, default_frequency};
use crate::script::{self, Injection, SCRIPT_PORT, ScriptClient};
use crate::session::{DEFAULT_TIMEOUT, Session, SessionWriter};
use crate::stream::{Snapshot, StateStream};

/// `UR_CONTROLLER_RDY_FOR_CMD` (rtde_control_interface.h:19).
const CONTROLLER_RDY_FOR_CMD: i32 = 1;
/// `UR_CONTROLLER_DONE_WITH_CMD`.
const CONTROLLER_DONE_WITH_CMD: i32 = 2;
/// `UR_EXECUTION_TIMEOUT` — 300 s.
const EXECUTION_TIMEOUT: Duration = Duration::from_secs(300);
/// `UR_GET_READY_TIMEOUT` — 3 s.
const GET_READY_TIMEOUT: Duration = Duration::from_secs(3);
/// Poll interval for the two command handshakes.
///
/// The C++ sleeps 1 ms in the "done with command" loop but the "ready for
/// command" loop has **no sleep at all** (rtde_control_interface.cpp:2511): it
/// spins a core flat out for up to `GET_READY_TIMEOUT`. Both loops sleep here.
const HANDSHAKE_POLL: Duration = Duration::from_millis(1);
/// The control script must be running within this long after an upload
/// (`waitForProgramRunning`, 5 s).
const PROGRAM_RUNNING_TIMEOUT: Duration = Duration::from_secs(5);
const FIRST_STATE_TIMEOUT: Duration = Duration::from_secs(5);

/// UR joint/tool limits the C++ range-checks commands against
/// (rtde_control_interface.h:28).
///
/// The joint limit is 3.14 rad/s, written here as a quotient so it cannot be
/// read as `PI`: the controller's limit sits just *below* π, and widening it to
/// π would pass commands the robot itself rejects.
const JOINT_VELOCITY_MAX: f64 = 314.0 / 100.0;
const JOINT_ACCELERATION_MAX: f64 = 40.0;
const TOOL_VELOCITY_MAX: f64 = 3.0;
const TOOL_ACCELERATION_MAX: f64 = 150.0;

/// `RuntimeState` (rtde_control_interface.h:291).
pub mod runtime_state {
    pub const STOPPING: u32 = 0;
    pub const STOPPED: u32 = 1;
    pub const PLAYING: u32 = 2;
    pub const PAUSING: u32 = 3;
    pub const PAUSED: u32 = 4;
    pub const RESUMING: u32 = 5;
}

/// `SafetyStatus` bit positions in `safety_status_bits`.
const SAFETY_BIT_PROTECTIVE_STOPPED: u32 = 2;
const SAFETY_BIT_EMERGENCY_STOPPED: u32 = 7;

/// Recipe ids, in the order `setupRecipes()` registers them. Only the ones
/// urRobot reaches are named.
mod recipe {
    /// moveJ / moveL (async setpoint).
    pub const ASYNC_SETP: u8 = 1;
    /// NO_CMD, STOP_SCRIPT, TEACH_MODE, END_TEACH_MODE, PROTECTIVE_STOP, IS_STEADY.
    pub const NO_CMD: u8 = 4;
    /// SET_TCP, IS_POSE/JOINTS_WITHIN_SAFETY_LIMITS.
    pub const WRENCH: u8 = 6;
    /// SPEED_STOP.
    pub const FORCE_MODE_PARAMETERS: u8 = 8;
    /// SPEEDL.
    pub const SETP: u8 = 13;
    /// STOPL / STOPJ.
    pub const STOP: u8 = 19;
    /// Recipes are registered 1..=23.
    pub const COUNT: u8 = 23;
}

/// `AsyncOperationStatus` (rtde_control_interface.h:97) — the status word the
/// control script writes into `output_int_register_2`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AsyncOperationStatus(pub u32);

impl AsyncOperationStatus {
    pub fn value(self) -> u32 {
        self.0
    }

    /// Running bit.
    pub fn is_running(self) -> bool {
        self.0 & 0x8000 != 0
    }

    /// Incremented once per async operation started.
    pub fn operation_id(self) -> u32 {
        (self.0 >> 24) & 0x7f
    }

    /// Incremented whenever the status changes.
    pub fn change_count(self) -> u32 {
        (self.0 >> 16) & 0xff
    }

    /// Progress in percent while running.
    pub fn progress(self) -> i32 {
        if self.is_running() {
            (self.0 & 0x7fff) as i32
        } else {
            -1
        }
    }
}

/// RTDE control interface.
pub struct ControlInterface {
    hostname: String,
    version: ControllerVersion,
    register_offset: i32,
    use_upper_range_registers: bool,
    stream: Option<StateStream>,
    writer: Option<SessionWriter>,
    script: ScriptClient,
    /// Used only to stop a running program before re-uploading the control
    /// script, exactly as `db_client_->stop()` does upstream.
    dashboard: DashboardClient,
}

impl ControlInterface {
    /// Connect, register the recipes, start synchronisation and upload the
    /// control script.
    pub fn connect(hostname: &str, use_upper_range_registers: bool) -> UrResult<Self> {
        let mut me = Self {
            hostname: hostname.to_string(),
            version: ControllerVersion::default(),
            register_offset: if use_upper_range_registers { 24 } else { 0 },
            use_upper_range_registers,
            stream: None,
            writer: None,
            script: ScriptClient::new(hostname, SCRIPT_PORT, DEFAULT_TIMEOUT),
            dashboard: DashboardClient::new(hostname, DEFAULT_TIMEOUT),
        };
        me.start()?;
        Ok(me)
    }

    fn start(&mut self) -> UrResult<()> {
        let mut session = Session::new(&self.hostname, DEFAULT_TIMEOUT);
        session.connect()?;
        session.negotiate_protocol_version()?;
        self.version = session.controller_version()?;

        self.setup_recipes(&mut session)?;
        session.send_start()?;

        let writer = session.writer()?;
        let stream = StateStream::spawn(session);
        stream.wait_first_state(FIRST_STATE_TIMEOUT)?;
        self.stream = Some(stream);
        self.writer = Some(writer);

        // Clear the command register before anything else touches it.
        self.send_clear_command()?;

        self.dashboard.connect()?;
        self.script.connect()?;
        self.upload_script()?;
        Ok(())
    }

    /// `reconnect()`.
    pub fn reconnect(&mut self) -> UrResult<()> {
        self.disconnect();
        self.start()
    }

    /// `disconnect()`.
    pub fn disconnect(&mut self) {
        self.stream = None;
        self.writer = None;
        self.script.disconnect();
        self.dashboard.disconnect();
    }

    pub fn is_connected(&self) -> bool {
        self.stream.as_ref().is_some_and(StateStream::is_connected)
    }

    pub fn controller_version(&self) -> ControllerVersion {
        self.version
    }

    fn in_int_reg(&self, reg: i32) -> String {
        format!("input_int_register_{}", self.register_offset + reg)
    }

    fn in_double_reg(&self, reg: i32) -> String {
        format!("input_double_register_{}", self.register_offset + reg)
    }

    fn out_int_reg(&self, reg: i32) -> String {
        format!("output_int_register_{}", self.register_offset + reg)
    }

    fn out_double_reg(&self, reg: i32) -> String {
        format!("output_double_register_{}", self.register_offset + reg)
    }

    /// `setupRecipes()`: the output recipe plus 23 input recipes.
    fn setup_recipes(&mut self, session: &mut Session) -> UrResult<()> {
        let mut state_names: Vec<String> = [
            "timestamp",
            "robot_status_bits",
            "safety_status_bits",
            "runtime_state",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();

        let registers_supported = match self.version.major {
            3 => {
                if self.use_upper_range_registers {
                    self.version.minor >= 9
                } else {
                    self.version.minor >= 4
                }
            }
            // Same gate defect as in the receive interface: upstream writes the
            // lower-range case as `major >= 3 && minor >= 4`, which is false on
            // PolyScope 5.0-5.3 (rtde_control_interface.cpp:626). Without these
            // registers the command handshake has nothing to read, so the
            // interface would appear to hang.
            m if m >= 5 => {
                if self.use_upper_range_registers {
                    self.version.minor >= 3
                } else {
                    true
                }
            }
            _ => false,
        };
        if !registers_supported {
            return Err(UrError::Protocol(format!(
                "PolyScope {}.{} does not provide the {} range of RTDE output registers, \
                 which the control interface needs",
                self.version.major,
                self.version.minor,
                if self.use_upper_range_registers {
                    "upper"
                } else {
                    "lower"
                }
            )));
        }
        for i in 0..=2 {
            state_names.push(self.out_int_reg(i));
        }
        for i in 0..=5 {
            state_names.push(self.out_double_reg(i));
        }

        session.send_output_setup(&state_names, default_frequency(self.version))?;

        // The 23 input recipes, in registration order. Recipes we never send on
        // are still registered so that the ones we do send on keep their ids.
        let int_regs = |n: i32| (0..n).map(|i| self.in_int_reg(i)).collect::<Vec<_>>();
        let double_regs =
            |lo: i32, hi: i32| (lo..=hi).map(|i| self.in_double_reg(i)).collect::<Vec<_>>();
        let mut recipes: Vec<Vec<String>> = Vec::new();

        // 1: async setpoint (moveJ / moveL)
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 7));
        r.push(self.in_int_reg(1));
        recipes.push(r);
        // 2: servoJ
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 10));
        recipes.push(r);
        // 3: force mode
        let mut r = int_regs(8);
        r.extend(double_regs(0, 17));
        recipes.push(r);
        // 4: no command
        recipes.push(vec![self.in_int_reg(0)]);
        // 5: servoC
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 8));
        recipes.push(r);
        // 6: wrench
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 5));
        recipes.push(r);
        // 7: set payload
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 3));
        recipes.push(r);
        // 8: force mode parameters
        recipes.push(vec![self.in_int_reg(0), self.in_double_reg(0)]);
        // 9: get actual joint positions history
        recipes.push(vec![self.in_int_reg(0), self.in_int_reg(1)]);
        // 10: get inverse kinematics
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 13));
        recipes.push(r);
        // 11: watchdog
        recipes.push(vec![self.in_int_reg(0)]);
        // 12: pose trans
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 11));
        recipes.push(r);
        // 13: setp (speedL)
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 7));
        recipes.push(r);
        // 14: jog
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 13));
        recipes.push(r);
        // 15: async path
        recipes.push(vec![self.in_int_reg(0), self.in_int_reg(1)]);
        // 16: move until contact
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 12));
        recipes.push(r);
        // 17: freedrive mode
        let mut r = int_regs(7);
        r.extend(double_regs(0, 5));
        recipes.push(r);
        // 18: ft rtde input enable
        let mut r = vec![self.in_int_reg(0), self.in_int_reg(1)];
        r.extend(double_regs(0, 6));
        recipes.push(r);
        // 19: STOPL / STOPJ
        recipes.push(vec![
            self.in_int_reg(0),
            self.in_double_reg(0),
            self.in_int_reg(1),
        ]);
        // 20: set target payload
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 9));
        recipes.push(r);
        // 21: torque command
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 5));
        r.push(self.in_int_reg(1));
        recipes.push(r);
        // 22: get jacobian time derivative
        let mut r = vec![self.in_int_reg(0)];
        r.extend(double_regs(0, 17));
        recipes.push(r);
        // 23: external force/torque
        recipes.push(vec![self.in_int_reg(0), "external_force_torque".into()]);

        debug_assert_eq!(recipes.len(), recipe::COUNT as usize);

        for (i, names) in recipes.iter().enumerate() {
            let assigned = session.send_input_setup(names)?;
            let expected = (i + 1) as u8;
            if assigned != expected {
                return Err(UrError::Protocol(format!(
                    "controller assigned recipe id {assigned} to input recipe {expected}; \
                     the control interface addresses recipes positionally"
                )));
            }
        }
        Ok(())
    }

    /// The newest robot state.
    fn snapshot(&self) -> Snapshot {
        self.stream
            .as_ref()
            .map(StateStream::snapshot)
            .unwrap_or_default()
    }

    fn runtime_state(&self) -> UrResult<u32> {
        self.snapshot()
            .uint("runtime_state")
            .ok_or_else(|| UrError::Protocol("no state data for runtime_state".into()))
    }

    /// `isProgramRunning()`.
    pub fn is_program_running(&self) -> UrResult<bool> {
        Ok(self.runtime_state()? == runtime_state::PLAYING)
    }

    fn safety_status_bits(&self) -> UrResult<u32> {
        self.snapshot()
            .uint("safety_status_bits")
            .ok_or_else(|| UrError::Protocol("no state data for safety_status_bits".into()))
    }

    /// `isProtectiveStopped() || isEmergencyStopped()`.
    fn is_stopped_by_safety(&self) -> UrResult<bool> {
        let bits = self.safety_status_bits()?;
        Ok(bits & (1 << SAFETY_BIT_PROTECTIVE_STOPPED) != 0
            || bits & (1 << SAFETY_BIT_EMERGENCY_STOPPED) != 0)
    }

    /// `getControlScriptState()` — `output_int_register_0`.
    fn control_script_state(&self) -> UrResult<i32> {
        self.snapshot()
            .output_int_register(self.register_offset)
            .ok_or_else(|| UrError::Protocol(format!("no state data for {}", self.out_int_reg(0))))
    }

    /// The control script's command result, `output_int_register_1`.
    fn control_script_result(&self) -> UrResult<i32> {
        self.snapshot()
            .output_int_register(self.register_offset + 1)
            .ok_or_else(|| UrError::Protocol(format!("no state data for {}", self.out_int_reg(1))))
    }

    /// `getAsyncOperationProgressEx()` — `output_int_register_2`.
    pub fn async_operation_progress(&self) -> UrResult<AsyncOperationStatus> {
        let reg = self.register_offset + 2;
        self.snapshot()
            .output_int_register(reg)
            .map(|v| AsyncOperationStatus(v as u32))
            .ok_or_else(|| UrError::Protocol(format!("no state data for {}", self.out_int_reg(2))))
    }

    fn write(&mut self, cmd: &RobotCommand) -> UrResult<()> {
        self.writer
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("RTDE control".into()))?
            .send_command(cmd)
    }

    /// `sendClearCommand()` — NO_CMD on recipe 4, no handshake.
    fn send_clear_command(&mut self) -> UrResult<()> {
        self.write(&RobotCommand::new(
            recipe::NO_CMD,
            CommandType::NoCmd,
            Payload::None,
        ))
    }

    /// `sendCommand()` — the full handshake with the control script.
    ///
    /// Returns `Ok(false)` when the command was refused (script not running,
    /// safety stop, or a timeout), matching the C++ `bool` return; the command
    /// register is always cleared before returning so the script is ready for
    /// the next command.
    fn send_command(&mut self, cmd: &RobotCommand) -> UrResult<bool> {
        if self.runtime_state()? == runtime_state::STOPPED {
            self.send_clear_command()?;
            return Ok(false);
        }
        if !self.is_program_running()? {
            log::error!("ur-robot: the RTDE control script is not running");
            self.send_clear_command()?;
            return Ok(false);
        }

        // Wait for the script to be ready for a command.
        if !self.wait_for_state(CONTROLLER_RDY_FOR_CMD, GET_READY_TIMEOUT)? {
            self.send_clear_command()?;
            return Ok(false);
        }

        self.write(cmd)?;

        if cmd.command == CommandType::StopScript {
            // The script is going away; wait for the program to stop instead of
            // for a DONE that will never come.
            let deadline = Instant::now() + EXECUTION_TIMEOUT;
            while self.is_program_running()? {
                if self.is_stopped_by_safety()? {
                    self.send_clear_command()?;
                    return Ok(false);
                }
                if Instant::now() >= deadline {
                    self.send_clear_command()?;
                    return Ok(false);
                }
                std::thread::sleep(HANDSHAKE_POLL);
            }
            std::thread::sleep(HANDSHAKE_POLL);
            self.send_clear_command()?;
            return Ok(true);
        }

        // Wait for the script to report it has finished the command.
        let deadline = Instant::now() + EXECUTION_TIMEOUT;
        loop {
            if self.control_script_state()? == CONTROLLER_DONE_WITH_CMD {
                break;
            }
            // A script error (e.g. an unsolvable inverse kinematics) kills the
            // script, and then DONE never arrives.
            if !self.is_program_running()? {
                log::error!("ur-robot: the RTDE control script stopped mid-command");
                self.send_clear_command()?;
                return Ok(false);
            }
            if self.is_stopped_by_safety()? {
                self.send_clear_command()?;
                return Ok(false);
            }
            if Instant::now() >= deadline {
                self.send_clear_command()?;
                return Ok(false);
            }
            std::thread::sleep(HANDSHAKE_POLL);
        }

        self.send_clear_command()?;
        Ok(true)
    }

    /// Wait until `output_int_register_0` reads `want`, or the deadline passes.
    fn wait_for_state(&mut self, want: i32, timeout: Duration) -> UrResult<bool> {
        let deadline = Instant::now() + timeout;
        loop {
            if self.control_script_state()? == want {
                return Ok(true);
            }
            if self.is_stopped_by_safety()? {
                return Ok(false);
            }
            if Instant::now() >= deadline {
                return Ok(false);
            }
            // Upstream's ready-for-command loop has no sleep and busy-spins.
            std::thread::sleep(HANDSHAKE_POLL);
        }
    }

    /// A command whose result the script leaves in `output_int_register_1`.
    fn send_query(&mut self, cmd: &RobotCommand) -> UrResult<bool> {
        if self.send_command(cmd)? {
            Ok(self.control_script_result()? == 1)
        } else {
            Ok(false)
        }
    }

    // --- script management ---

    /// Build and send the control script, then wait for it to start running.
    fn upload_script(&mut self) -> UrResult<()> {
        let offset = self.register_offset.to_string();
        let injections = [
            Injection {
                search: script::INJECT_FLOAT_OFFSET.to_string(),
                inject: offset.clone(),
            },
            Injection {
                search: script::INJECT_INT_OFFSET.to_string(),
                inject: offset,
            },
        ];
        let text =
            script::build_control_script(self.version.major, self.version.minor, &injections)?;
        self.script.send(&text)?;
        self.wait_for_program_running()
    }

    /// `waitForProgramRunning()`.
    fn wait_for_program_running(&self) -> UrResult<()> {
        let deadline = Instant::now() + PROGRAM_RUNNING_TIMEOUT;
        loop {
            if self.is_program_running()? {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(UrError::Script(
                    "the RTDE control script did not start within 5 s".into(),
                ));
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// `reuploadScript()` — kill whatever is running, then send the script again.
    pub fn reupload_script(&mut self) -> UrResult<()> {
        if self.is_program_running()? {
            log::info!("ur-robot: a script was running on the controller, stopping it");
            self.stop_script()?;
            self.dashboard.stop()?;
            std::thread::sleep(Duration::from_millis(100));
        }
        if !self.script.is_connected() {
            self.script.connect()?;
        }
        self.upload_script()
    }

    /// `stopScript()`.
    pub fn stop_script(&mut self) -> UrResult<()> {
        self.send_command(&RobotCommand::new(
            recipe::NO_CMD,
            CommandType::StopScript,
            Payload::None,
        ))
        .map(|_| ())
    }

    /// Run a user URScript, wrapped so its completion bumps output int register
    /// 12 (urRobot's `wrap_script`).
    pub fn send_custom_script(&mut self, body: &str) -> UrResult<()> {
        self.stop_script()?;
        if !self.script.is_connected() {
            self.script.connect()?;
        }
        self.script.send(&script::wrap_custom_script(body))
    }

    // --- motion commands ---

    /// `moveJ(q, speed, acceleration, async)` — recipe 1.
    pub fn move_j(
        &mut self,
        q: &[f64; 6],
        speed: f64,
        acceleration: f64,
        asynchronous: bool,
    ) -> UrResult<bool> {
        verify_within(speed, 0.0, JOINT_VELOCITY_MAX)?;
        verify_within(acceleration, 0.0, JOINT_ACCELERATION_MAX)?;
        let mut val = q.to_vec();
        val.push(speed);
        val.push(acceleration);
        self.send_command(&RobotCommand::new(
            recipe::ASYNC_SETP,
            CommandType::MoveJ,
            Payload::VectorAsync { val, asynchronous },
        ))
    }

    /// `moveL(pose, speed, acceleration, async)` — recipe 1.
    pub fn move_l(
        &mut self,
        pose: &[f64; 6],
        speed: f64,
        acceleration: f64,
        asynchronous: bool,
    ) -> UrResult<bool> {
        verify_within(speed, 0.0, TOOL_VELOCITY_MAX)?;
        verify_within(acceleration, 0.0, TOOL_ACCELERATION_MAX)?;
        let mut val = pose.to_vec();
        val.push(speed);
        val.push(acceleration);
        self.send_command(&RobotCommand::new(
            recipe::ASYNC_SETP,
            CommandType::MoveL,
            Payload::VectorAsync { val, asynchronous },
        ))
    }

    /// `stopJ(a, async)` — recipe 19. `a` defaults to 2.0 rad/s^2 upstream.
    pub fn stop_j(&mut self, acceleration: f64, asynchronous: bool) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::STOP,
            CommandType::StopJ,
            Payload::VectorAsync {
                val: vec![acceleration],
                asynchronous,
            },
        ))
    }

    /// `stopL(a, async)` — recipe 19. `a` defaults to 10.0 m/s^2 upstream.
    pub fn stop_l(&mut self, acceleration: f64, asynchronous: bool) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::STOP,
            CommandType::StopL,
            Payload::VectorAsync {
                val: vec![acceleration],
                asynchronous,
            },
        ))
    }

    /// `speedL(xd, acceleration, time)` — recipe 13.
    pub fn speed_l(&mut self, xd: &[f64; 6], acceleration: f64, time: f64) -> UrResult<bool> {
        verify_within(acceleration, 0.0, TOOL_ACCELERATION_MAX)?;
        let mut val = xd.to_vec();
        val.push(acceleration);
        val.push(time);
        self.send_command(&RobotCommand::new(
            recipe::SETP,
            CommandType::SpeedL,
            Payload::Vector(val),
        ))
    }

    /// `speedStop(a)` — recipe 8. `a` defaults to 10.0 upstream.
    pub fn speed_stop(&mut self, acceleration: f64) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::FORCE_MODE_PARAMETERS,
            CommandType::SpeedStop,
            Payload::Vector(vec![acceleration]),
        ))
    }

    /// `setTcp(tcp_offset)` — recipe 6.
    pub fn set_tcp(&mut self, tcp_offset: &[f64; 6]) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::WRENCH,
            CommandType::SetTcp,
            Payload::Vector(tcp_offset.to_vec()),
        ))
    }

    /// `teachMode()` — recipe 4.
    pub fn teach_mode(&mut self) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::NO_CMD,
            CommandType::TeachMode,
            Payload::None,
        ))
    }

    /// `endTeachMode()` — recipe 4.
    pub fn end_teach_mode(&mut self) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::NO_CMD,
            CommandType::EndTeachMode,
            Payload::None,
        ))
    }

    /// `triggerProtectiveStop()` — recipe 4.
    pub fn trigger_protective_stop(&mut self) -> UrResult<bool> {
        self.send_command(&RobotCommand::new(
            recipe::NO_CMD,
            CommandType::ProtectiveStop,
            Payload::None,
        ))
    }

    /// `isSteady()` — recipe 4, answer in `output_int_register_1`.
    pub fn is_steady(&mut self) -> UrResult<bool> {
        self.send_query(&RobotCommand::new(
            recipe::NO_CMD,
            CommandType::IsSteady,
            Payload::None,
        ))
    }

    /// `isPoseWithinSafetyLimits(pose)` — recipe 6.
    pub fn is_pose_within_safety_limits(&mut self, pose: &[f64; 6]) -> UrResult<bool> {
        self.send_query(&RobotCommand::new(
            recipe::WRENCH,
            CommandType::IsPoseWithinSafetyLimits,
            Payload::Vector(pose.to_vec()),
        ))
    }

    /// `isJointsWithinSafetyLimits(q)` — recipe 6.
    pub fn is_joints_within_safety_limits(&mut self, q: &[f64; 6]) -> UrResult<bool> {
        self.send_query(&RobotCommand::new(
            recipe::WRENCH,
            CommandType::IsJointsWithinSafetyLimits,
            Payload::Vector(q.to_vec()),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn async_status_word_decodes_into_its_fields() {
        // running, operation 3, change count 7, progress 42
        let raw = (3u32 << 24) | (7u32 << 16) | 0x8000 | 42;
        let s = AsyncOperationStatus(raw);
        assert!(s.is_running());
        assert_eq!(s.operation_id(), 3);
        assert_eq!(s.change_count(), 7);
        assert_eq!(s.progress(), 42);
        assert_eq!(s.value(), raw);

        // Not running: progress reads -1 regardless of the low bits.
        let s = AsyncOperationStatus((3u32 << 24) | (7u32 << 16) | 42);
        assert!(!s.is_running());
        assert_eq!(s.progress(), -1);
        assert_eq!(s.operation_id(), 3);
    }

    #[test]
    fn movej_payload_is_q_then_speed_accel_then_async() {
        let q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        let mut val = q.to_vec();
        val.push(1.05);
        val.push(1.4);
        let cmd = RobotCommand::new(
            recipe::ASYNC_SETP,
            CommandType::MoveJ,
            Payload::VectorAsync {
                val: val.clone(),
                asynchronous: true,
            },
        );
        assert_eq!(cmd.recipe_id, 1);
        // 3 header + 1 recipe + 4 type + 8*8 doubles + 4 async
        assert_eq!(cmd.encode().len(), 3 + 1 + 4 + 64 + 4);
    }

    #[test]
    fn each_command_goes_to_the_recipe_the_script_listens_on() {
        assert_eq!(recipe::ASYNC_SETP, 1);
        assert_eq!(recipe::NO_CMD, 4);
        assert_eq!(recipe::WRENCH, 6);
        assert_eq!(recipe::FORCE_MODE_PARAMETERS, 8);
        assert_eq!(recipe::SETP, 13);
        assert_eq!(recipe::STOP, 19);
        assert_eq!(recipe::COUNT, 23);
    }

    #[test]
    fn safety_bits_flag_protective_and_emergency_stop() {
        // Bit 2 = protective stop, bit 7 = emergency stop.
        assert_eq!(1u32 << SAFETY_BIT_PROTECTIVE_STOPPED, 0x04);
        assert_eq!(1u32 << SAFETY_BIT_EMERGENCY_STOPPED, 0x80);
        // "normal" (bit 0 set) is neither.
        let normal = 1u32;
        assert_eq!(normal & (1 << SAFETY_BIT_PROTECTIVE_STOPPED), 0);
        assert_eq!(normal & (1 << SAFETY_BIT_EMERGENCY_STOPPED), 0);
    }

    #[test]
    fn range_checks_match_the_ur_limits() {
        assert!(verify_within(JOINT_VELOCITY_MAX, 0.0, JOINT_VELOCITY_MAX).is_ok());
        assert!(verify_within(3.15, 0.0, JOINT_VELOCITY_MAX).is_err());
        // The limit is below PI, so a command at PI rad/s must be refused.
        assert!(verify_within(std::f64::consts::PI, 0.0, JOINT_VELOCITY_MAX).is_err());
        assert!(verify_within(40.0, 0.0, JOINT_ACCELERATION_MAX).is_ok());
        assert!(verify_within(3.0, 0.0, TOOL_VELOCITY_MAX).is_ok());
        assert!(verify_within(-0.1, 0.0, TOOL_VELOCITY_MAX).is_err());
        assert!(verify_within(150.0, 0.0, TOOL_ACCELERATION_MAX).is_ok());
    }
}
