//! IOC integration for the Newport motor drivers.
//!
//! Exposes the Newport family's `*CreateController` iocsh commands and the
//! dynamic device-support factory that binds each created controller to its
//! motor record. A single [`NewportHolder`] serves the whole family (the Rust
//! analogue of `NewportRegister.cc`, which registers every Newport
//! `CreateController` function): the IOC binary registers whichever create
//! commands it needs, and all share one device-support store keyed by DTYP.
//!
//! Wiring chain (assembled in the IOC binary + st.cmd), e.g. SMC100:
//!
//! ```text
//! drvAsynSerialPortConfigure("SERIAL", "/dev/ttyUSB0", ...)   // built-in asyn
//!   -> SMC100CreateController("MOTOR", "SERIAL", eguPerStep, ...)
//!        get_port("SERIAL") -> SyncIOHandle -> Smc100Axis -> MotorBuilder
//!        -> spawn poll loop, store device support under DTYP "SMC100_MOTOR"
//!   -> dbLoadRecords(smc100.template, "P=..,M=..,PORT=MOTOR")   // DTYP match
//! ```
//!
//! CONEX is identical except the axis is a [`ConexAxis`] (which self-identifies
//! its model at construction) and the DTYP prefix is `CONEX_`.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::builder::{MotorBuilder, MotorSetup};
use epics_rs::motor::device_support::MotorDeviceSupport;
use epics_rs::motor::poll_loop::PollCommand;

use crate::agap::{self, AgapAxis, AgapController};
use crate::agilis::{AgUcAxis, AgUcController};
use crate::conex::ConexAxis;
use crate::smc100::Smc100Axis;
use crate::xps::{SocketMode, XpsAxis, XpsController, XpsSocket};

/// Default moving/idle poll intervals (ms) when the iocsh args are omitted.
const DEFAULT_MOVING_POLL_MS: u64 = 100;
const DEFAULT_IDLE_POLL_MS: u64 = 1000;

/// The 1-based axis number for the single-axis SMC100 (C `axisNo_ + 1`).
const SMC100_AXIS: u8 = 1;

/// Octet `addr` used on the serial transport port (single-device serial line).
const SERIAL_ADDR: i32 = 0;

/// Per-command serial I/O timeout: SMC100 (`from_handle` default) and CONEX /
/// AGAP (`CONEX_TIMEOUT`).
const SMC100_TIMEOUT: Duration = Duration::from_secs(1);
const CONEX_TIMEOUT: Duration = Duration::from_secs(2);
const AGAP_TIMEOUT: Duration = Duration::from_secs(2);
const AG_UC_TIMEOUT: Duration = Duration::from_secs(2);

/// XPS poll-socket timeout (C `XPS_POLL_TIMEOUT` = 2 s): waits for the full
/// `,EndOfAPI`-framed reply.
const XPS_POLL_TIMEOUT: Duration = Duration::from_secs(2);
/// XPS move-socket timeout (C sets `-0.1` s): the fire-and-forget write does
/// not wait for the move to finish, so the read times out quickly.
const XPS_MOVE_TIMEOUT: Duration = Duration::from_millis(100);

/// A shared Agilis controller registered by `AG_UCCreateController`, awaiting
/// its axes to be added by `AG_UCCreateAxis`. Carries the poll intervals from
/// the controller command so each later axis inherits them.
struct AgUcRegistration {
    controller: Arc<Mutex<AgUcController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// A shared XPS controller registered by `XPSCreateController`, awaiting its
/// axes to be added by `XPSCreateAxis`. Carries the poll intervals so each
/// later axis inherits them.
struct XpsRegistration {
    controller: Arc<Mutex<XpsController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// Holds Newport motor device-support instances created by the family's
/// `*CreateController` commands. Each controller is stored under a
/// `"{PREFIX}_{motor_port}"` DTYP key and consumed once by the dynamic
/// device-support factory during iocInit.
pub struct NewportHolder {
    motors: Mutex<HashMap<String, Option<MotorDeviceSupport>>>,
    poll_senders: Mutex<Vec<tokio::sync::mpsc::Sender<PollCommand>>>,
    /// Agilis controllers keyed by motor port, bridging the two-step
    /// `AG_UCCreateController` / `AG_UCCreateAxis` iocsh API.
    ag_uc_controllers: Mutex<HashMap<String, AgUcRegistration>>,
    /// XPS controllers keyed by motor port, bridging the two-step
    /// `XPSCreateController` / `XPSCreateAxis` iocsh API.
    xps_controllers: Mutex<HashMap<String, XpsRegistration>>,
}

impl NewportHolder {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            motors: Mutex::new(HashMap::new()),
            poll_senders: Mutex::new(Vec::new()),
            ag_uc_controllers: Mutex::new(HashMap::new()),
            xps_controllers: Mutex::new(HashMap::new()),
        })
    }

    /// Start polling on all registered controllers.
    /// Call after PINI processing to avoid queue buildup.
    pub fn start_all_polling(&self) {
        for tx in self
            .poll_senders
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
        {
            let _ = tx.try_send(PollCommand::StartPolling);
        }
    }

    /// Shared wiring for every `*CreateController` command: build the motor
    /// record + poll loop for `motor`, spawn the poll loop (starts idle), and
    /// store the device support under `dtyp_key` for later `dbLoadRecords`
    /// binding.
    fn install(
        &self,
        ctx: &CommandContext,
        dtyp_key: String,
        motor: Arc<Mutex<dyn AsynMotor>>,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
    ) {
        let MotorSetup {
            record: _,
            device_support,
            poll_loop,
            poll_cmd_tx,
        } = MotorBuilder::new(motor)
            .moving_poll_interval(Duration::from_millis(moving_poll_ms))
            .idle_poll_interval(Duration::from_millis(idle_poll_ms))
            .build();

        let device_support = device_support.with_dtyp_name(dtyp_key.clone());

        ctx.runtime_handle().spawn(poll_loop.run());

        self.poll_senders
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(poll_cmd_tx);
        self.motors
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(dtyp_key, Some(device_support));
    }

    /// Create the `SMC100CreateController` iocsh command.
    ///
    /// Usage:
    /// `SMC100CreateController(motorPort, serialPort, eguPerStep, [movingPollMs], [idlePollMs])`
    pub fn smc100_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "SMC100CreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("serialPort"),
                ArgDesc {
                    name: "eguPerStep",
                    arg_type: ArgType::Double,
                    optional: false,
                },
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "SMC100CreateController(motorPort, serialPort, eguPerStep, [movingPollMs], [idlePollMs]) - Create a Newport SMC100 controller",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let serial_port = req_string(args, 1, "serialPort")?;
                let egu_per_step = match &args[2] {
                    ArgValue::Double(v) => *v,
                    _ => return Err("eguPerStep must be a number".into()),
                };
                if egu_per_step == 0.0 {
                    return Err("eguPerStep must be non-zero".into());
                }
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

                let handle = connect_serial(&serial_port, SMC100_TIMEOUT)?;
                let dtyp_key = format!("SMC100_{motor_port}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Smc100Axis::new(
                    handle,
                    SMC100_AXIS,
                    egu_per_step,
                )));

                holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                println!(
                    "SMC100CreateController: motorPort={motor_port} serialPort={serial_port} eguPerStep={egu_per_step} poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP={dtyp_key})"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `AG_CONEXCreateController` iocsh command.
    ///
    /// Usage:
    /// `AG_CONEXCreateController(motorPort, serialPort, controllerID, [movingPollMs], [idlePollMs])`
    ///
    /// [`ConexAxis::new`] performs blocking serial I/O to identify the CONEX
    /// model and read its limits, so the controller must be reachable when
    /// this command runs.
    pub fn ag_conex_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "AG_CONEXCreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("serialPort"),
                ArgDesc {
                    name: "controllerID",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "AG_CONEXCreateController(motorPort, serialPort, controllerID, [movingPollMs], [idlePollMs]) - Create a Newport CONEX controller",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let serial_port = req_string(args, 1, "serialPort")?;
                let controller_id = match &args[2] {
                    ArgValue::Int(v) => *v as i32,
                    _ => return Err("controllerID must be an integer".into()),
                };
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

                let handle = connect_serial(&serial_port, CONEX_TIMEOUT)?;
                let axis = ConexAxis::new(handle, controller_id)
                    .map_err(|e| format!("AG_CONEXCreateController: {e}"))?;
                let dtyp_key = format!("CONEX_{motor_port}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));

                holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                println!(
                    "AG_CONEXCreateController: motorPort={motor_port} serialPort={serial_port} controllerID={controller_id} poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP={dtyp_key})"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `AGAP_CONEXCreateController` iocsh command.
    ///
    /// Usage:
    /// `AGAP_CONEXCreateController(motorPort, serialPort, controllerID, [movingPollMs], [idlePollMs])`
    ///
    /// The AGAP is a two-axis (`U`/`V`) controller: this creates one shared
    /// [`AgapController`] plus two [`AgapAxis`] device supports, registered
    /// under DTYP keys `AGAP_{motorPort}_U` and `AGAP_{motorPort}_V`. Both
    /// axes share one serial line; each axis operation locks the controller so
    /// its write→read exchange stays atomic. [`AgapController::new`] and
    /// [`AgapAxis::new`] perform blocking serial I/O, so the controller must be
    /// reachable when this command runs.
    pub fn agap_conex_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "AGAP_CONEXCreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("serialPort"),
                ArgDesc {
                    name: "controllerID",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "AGAP_CONEXCreateController(motorPort, serialPort, controllerID, [movingPollMs], [idlePollMs]) - Create a Newport CONEX-AGAP two-axis controller",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let serial_port = req_string(args, 1, "serialPort")?;
                let controller_id = match &args[2] {
                    ArgValue::Int(v) => *v as i32,
                    _ => return Err("controllerID must be an integer".into()),
                };
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

                let handle = connect_serial(&serial_port, AGAP_TIMEOUT)?;
                let controller = AgapController::new(handle, controller_id)
                    .map_err(|e| format!("AGAP_CONEXCreateController: {e}"))?;
                let controller = Arc::new(Mutex::new(controller));

                for axis_index in 0..agap::NUM_AXES {
                    let axis = AgapAxis::new(controller.clone(), axis_index)
                        .map_err(|e| format!("AGAP_CONEXCreateController: {e}"))?;
                    let dtyp_key = format!("AGAP_{motor_port}_{}", agap::axis_name(axis_index));
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                    holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
                }
                println!(
                    "AGAP_CONEXCreateController: motorPort={motor_port} serialPort={serial_port} controllerID={controller_id} poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=AGAP_{motor_port}_U, AGAP_{motor_port}_V)"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `AG_UCCreateController` iocsh command.
    ///
    /// Usage:
    /// `AG_UCCreateController(motorPort, serialPort, numAxes, [movingPollMs], [idlePollMs])`
    ///
    /// Registers a shared Agilis [`AgUcController`] under `motorPort`; the axes
    /// are added afterward by [`Self::ag_uc_create_axis_command`]. `numAxes` is
    /// accepted for C-parity but the axes are created individually.
    /// [`AgUcController::new`] resets the controller and performs blocking
    /// serial I/O, so it must be reachable when this command runs.
    pub fn ag_uc_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "AG_UCCreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("serialPort"),
                ArgDesc {
                    name: "numAxes",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "AG_UCCreateController(motorPort, serialPort, numAxes, [movingPollMs], [idlePollMs]) - Create a Newport Agilis AG-UC controller (add axes with AG_UCCreateAxis)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let serial_port = req_string(args, 1, "serialPort")?;
                let num_axes = match &args[2] {
                    ArgValue::Int(v) => *v,
                    _ => return Err("numAxes must be an integer".into()),
                };
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

                let handle = connect_serial(&serial_port, AG_UC_TIMEOUT)?;
                let controller = AgUcController::new(handle)
                    .map_err(|e| format!("AG_UCCreateController: {e}"))?;
                let model = controller.model();
                holder
                    .ag_uc_controllers
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .insert(
                        motor_port.clone(),
                        AgUcRegistration {
                            controller: Arc::new(Mutex::new(controller)),
                            moving_poll_ms,
                            idle_poll_ms,
                        },
                    );
                println!(
                    "AG_UCCreateController: motorPort={motor_port} serialPort={serial_port} model={model:?} numAxes={num_axes} poll=[{moving_poll_ms}/{idle_poll_ms}]ms (add axes with AG_UCCreateAxis)"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `AG_UCCreateAxis` iocsh command.
    ///
    /// Usage:
    /// `AG_UCCreateAxis(motorPort, axis, hasLimits, forwardAmplitude, reverseAmplitude)`
    ///
    /// Adds one [`AgUcAxis`] to the controller registered under `motorPort` by
    /// `AG_UCCreateController`, registered under DTYP `AG_UC_{motorPort}_{axis}`.
    /// [`AgUcAxis::new`] performs blocking serial I/O (step-amplitude setup).
    pub fn ag_uc_create_axis_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "AG_UCCreateAxis",
            vec![
                arg_str_req("motorPort"),
                ArgDesc {
                    name: "axis",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "hasLimits",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "forwardAmplitude",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "reverseAmplitude",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "AG_UCCreateAxis(motorPort, axis, hasLimits, forwardAmplitude, reverseAmplitude) - Add an axis to a Newport Agilis AG-UC controller",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let axis = match &args[1] {
                    ArgValue::Int(v) => *v as i32,
                    _ => return Err("axis must be an integer".into()),
                };
                let has_limits = match &args[2] {
                    ArgValue::Int(v) => *v != 0,
                    _ => return Err("hasLimits must be an integer".into()),
                };
                let forward_amplitude = match &args[3] {
                    ArgValue::Int(v) => *v as i32,
                    _ => return Err("forwardAmplitude must be an integer".into()),
                };
                let reverse_amplitude = match &args[4] {
                    ArgValue::Int(v) => *v as i32,
                    _ => return Err("reverseAmplitude must be an integer".into()),
                };

                let controller = {
                    let controllers = holder
                        .ag_uc_controllers
                        .lock()
                        .unwrap_or_else(|e| e.into_inner());
                    let reg = controllers.get(&motor_port).ok_or_else(|| {
                        format!(
                            "AG_UCCreateAxis: controller '{motor_port}' not found (call AG_UCCreateController first)"
                        )
                    })?;
                    (reg.controller.clone(), reg.moving_poll_ms, reg.idle_poll_ms)
                };
                let (controller, moving_poll_ms, idle_poll_ms) = controller;

                let ax = AgUcAxis::new(
                    controller,
                    axis,
                    has_limits,
                    forward_amplitude,
                    reverse_amplitude,
                )
                .map_err(|e| format!("AG_UCCreateAxis: {e}"))?;
                let dtyp_key = format!("AG_UC_{motor_port}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));

                holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                println!(
                    "AG_UCCreateAxis: motorPort={motor_port} axis={axis} hasLimits={has_limits} fwdAmp={forward_amplitude} revAmp={reverse_amplitude} (DTYP={dtyp_key})"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `XPSCreateController` iocsh command.
    ///
    /// Usage:
    /// `XPSCreateController(motorPort, pollPort, numAxes, [movingPollMs], [idlePollMs], [enableSetPosition], [setPositionSettlingMs])`
    ///
    /// `pollPort` is a `drvAsynIPPort` (TCP) to the XPS, shared by the
    /// controller and all its axes for reads. Add axes with `XPSCreateAxis`.
    /// Unlike the C `XPSCreateController` (which takes an IP/port and opens the
    /// sockets itself), this looks up asyn ports registered in `st.cmd`, matching
    /// the port-name convention of the other Newport Rust drivers.
    pub fn xps_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSCreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("pollPort"),
                ArgDesc {
                    name: "numAxes",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
                arg_int_opt("enableSetPosition"),
                arg_int_opt("setPositionSettlingMs"),
            ],
            "XPSCreateController(motorPort, pollPort, numAxes, [movingPollMs], [idlePollMs], [enableSetPosition], [setPositionSettlingMs]) - Create a Newport XPS controller (add axes with XPSCreateAxis)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let poll_port = req_string(args, 1, "pollPort")?;
                let num_axes = match &args[2] {
                    ArgValue::Int(v) => *v,
                    _ => return Err("numAxes must be an integer".into()),
                };
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
                let enable_set_position = match args.get(5) {
                    Some(ArgValue::Int(v)) => *v != 0,
                    None | Some(ArgValue::Missing) => false,
                    _ => return Err("enableSetPosition must be an integer".into()),
                };
                let settling_ms = match args.get(6) {
                    Some(ArgValue::Int(v)) if *v >= 0 => *v as u64,
                    None | Some(ArgValue::Missing) => 0,
                    _ => return Err("setPositionSettlingMs must be a non-negative integer".into()),
                };

                let handle = connect_ip(&poll_port, XPS_POLL_TIMEOUT)?;
                let poll_sock = XpsSocket::new(handle, SocketMode::Query);
                let controller = XpsController::new(
                    poll_sock,
                    enable_set_position,
                    Duration::from_millis(settling_ms),
                )
                .map_err(|e| format!("XPSCreateController: {e}"))?;
                let firmware = controller.firmware().to_string();
                holder
                    .xps_controllers
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .insert(
                        motor_port.clone(),
                        XpsRegistration {
                            controller: Arc::new(Mutex::new(controller)),
                            moving_poll_ms,
                            idle_poll_ms,
                        },
                    );
                println!(
                    "XPSCreateController: motorPort={motor_port} pollPort={poll_port} firmware=\"{firmware}\" numAxes={num_axes} enableSetPosition={enable_set_position} settling={settling_ms}ms poll=[{moving_poll_ms}/{idle_poll_ms}]ms (add axes with XPSCreateAxis)"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `XPSCreateAxis` iocsh command.
    ///
    /// Usage:
    /// `XPSCreateAxis(motorPort, movePort, axis, positionerName, stepsPerUnit)`
    ///
    /// Adds one [`XpsAxis`] to the controller registered under `motorPort` by
    /// `XPSCreateController`, registered under DTYP `XPS_{motorPort}_{axis}`.
    /// `movePort` is this axis's own `drvAsynIPPort` (TCP), used for the
    /// fire-and-forget move socket. `positionerName` is `group.positioner` and
    /// `stepsPerUnit` follows C: `stepSize = 1 / stepsPerUnit`. [`XpsAxis::new`]
    /// performs blocking RPC (reads the S-gamma jerk times), so the controller
    /// must be reachable when this command runs.
    pub fn xps_create_axis_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSCreateAxis",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("movePort"),
                ArgDesc {
                    name: "axis",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                arg_str_req("positionerName"),
                ArgDesc {
                    name: "stepsPerUnit",
                    arg_type: ArgType::Double,
                    optional: false,
                },
            ],
            "XPSCreateAxis(motorPort, movePort, axis, positionerName, stepsPerUnit) - Add an axis to a Newport XPS controller",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let move_port = req_string(args, 1, "movePort")?;
                let axis = match &args[2] {
                    ArgValue::Int(v) => *v as i32,
                    _ => return Err("axis must be an integer".into()),
                };
                let positioner_name = req_string(args, 3, "positionerName")?;
                let steps_per_unit = match &args[4] {
                    ArgValue::Double(v) => *v,
                    _ => return Err("stepsPerUnit must be a number".into()),
                };
                if steps_per_unit == 0.0 {
                    return Err("stepsPerUnit must be non-zero".into());
                }
                let step_size = 1.0 / steps_per_unit;

                let (controller, moving_poll_ms, idle_poll_ms) = {
                    let controllers = holder
                        .xps_controllers
                        .lock()
                        .unwrap_or_else(|e| e.into_inner());
                    let reg = controllers.get(&motor_port).ok_or_else(|| {
                        format!(
                            "XPSCreateAxis: controller '{motor_port}' not found (call XPSCreateController first)"
                        )
                    })?;
                    (reg.controller.clone(), reg.moving_poll_ms, reg.idle_poll_ms)
                };

                let handle = connect_ip(&move_port, XPS_MOVE_TIMEOUT)?;
                let move_sock = XpsSocket::new(handle, SocketMode::Fire);
                let ax = XpsAxis::new(controller, move_sock, &positioner_name, step_size)
                    .map_err(|e| format!("XPSCreateAxis: {e}"))?;
                let dtyp_key = format!("XPS_{motor_port}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));

                holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                println!(
                    "XPSCreateAxis: motorPort={motor_port} movePort={move_port} axis={axis} positioner={positioner_name} stepsPerUnit={steps_per_unit} (DTYP={dtyp_key})"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Return a dynamic device support factory that dispatches by DTYP name.
    /// Each device support is consumed once (take semantics).
    pub fn device_support_factory(
        self: &Arc<Self>,
    ) -> impl Fn(
        &epics_rs::ca::server::ioc_app::DeviceSupportContext,
    ) -> Option<Box<dyn epics_rs::base::server::device_support::DeviceSupport>>
    + Send
    + Sync
    + 'static {
        let holder = self.clone();
        move |ctx: &epics_rs::ca::server::ioc_app::DeviceSupportContext| {
            let mut motors = holder.motors.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(slot) = motors.get_mut(ctx.dtyp)
                && let Some(ds) = slot.take()
            {
                return Some(
                    Box::new(ds) as Box<dyn epics_rs::base::server::device_support::DeviceSupport>
                );
            }
            None
        }
    }
}

// --- iocsh arg helpers, shared by the family's create commands.

fn arg_str_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::String,
        optional: false,
    }
}

fn arg_int_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Int,
        optional: true,
    }
}

fn req_string(args: &[ArgValue], i: usize, name: &str) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.clone()),
        _ => Err(format!("{name} must be a string")),
    }
}

/// Parse the optional `movingPollMs`/`idlePollMs` trailing args, defaulting and
/// rejecting non-positive values.
fn poll_intervals(args: &[ArgValue], moving_i: usize, idle_i: usize) -> Result<(u64, u64), String> {
    let moving = poll_ms(args.get(moving_i), DEFAULT_MOVING_POLL_MS, "movingPollMs")?;
    let idle = poll_ms(args.get(idle_i), DEFAULT_IDLE_POLL_MS, "idlePollMs")?;
    Ok((moving, idle))
}

fn poll_ms(arg: Option<&ArgValue>, default: u64, name: &str) -> Result<u64, String> {
    match arg {
        Some(ArgValue::Int(v)) if *v > 0 => Ok(*v as u64),
        Some(ArgValue::Int(_)) => Err(format!("{name} must be positive")),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be an integer")),
    }
}

/// Connect a [`SyncIOHandle`] to a pre-configured serial octet port by name
/// (created by `drvAsynSerialPortConfigure`).
fn connect_serial(serial_port: &str, timeout: Duration) -> Result<SyncIOHandle, String> {
    let port = get_port(serial_port).ok_or_else(|| {
        format!("serial port '{serial_port}' not found (call drvAsynSerialPortConfigure first)")
    })?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        SERIAL_ADDR,
        timeout,
    ))
}

/// Connect a [`SyncIOHandle`] to a pre-configured TCP octet port by name
/// (created by `drvAsynIPPortConfigure`). Same lookup as [`connect_serial`] —
/// separate only for a TCP-appropriate error message.
fn connect_ip(ip_port: &str, timeout: Duration) -> Result<SyncIOHandle, String> {
    let port = get_port(ip_port).ok_or_else(|| {
        format!("IP port '{ip_port}' not found (call drvAsynIPPortConfigure first)")
    })?;
    Ok(SyncIOHandle::from_handle(
        port.handle.clone(),
        SERIAL_ADDR,
        timeout,
    ))
}
