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
//!   -> SMC100CreateController("MOTOR", "SERIAL", wireScale, ...)
//!        get_port("SERIAL") -> SyncIOHandle -> Smc100Axis -> MotorBuilder
//!        -> spawn poll loop, store device support under DTYP "SMC100_MOTOR"
//!   -> dbLoadRecords(smc100.template, "P=..,M=..,PORT=MOTOR")   // DTYP match
//! ```
//!
//! CONEX is identical except the axis is a [`ConexAxis`] (which self-identifies
//! its model at construction) and the DTYP prefix is `CONEX_`.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
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
use crate::esp300::{Esp300Axis, Esp300Controller};
use crate::hxp::{HXP_GROUP, HxpAxis, HxpController, MoveCoordSys, NUM_HXP_AXES};
use crate::smc100::Smc100Axis;
use crate::xps::{
    ExecutionPlan, GatheringReadback, MoveMode, PcoParams, Profile, SocketMode, XpsAxis,
    XpsController, XpsError, XpsSocket, ftp, gathering, pco, profile,
};

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

/// XPS PVT execution socket timeout. `MultipleAxesPVTExecution` — and the
/// move-to-start moves that precede it on the same socket — block until the
/// motion finishes, so this bounds a whole trajectory run rather than a single
/// RPC. C runs this on a dedicated socket with no explicit cap; 1 h is a
/// generous upper bound so a wedged run cannot hang the executor thread forever.
const XPS_PVT_EXEC_TIMEOUT: Duration = Duration::from_secs(3600);

/// HXP hexapod poll-socket timeout (C `HXP_POLL_TIMEOUT` = 2 s).
const HXP_POLL_TIMEOUT: Duration = Duration::from_secs(2);
/// HXP move-socket timeout (C sets `-0.1` s: fire-and-forget writes).
const HXP_MOVE_TIMEOUT: Duration = Duration::from_millis(100);

/// ESP300 serial command timeout (C `SERIAL_TIMEOUT` 5 s — "the ESP300 does
/// not respond for 2 to 5 seconds after hitting a travel limit").
const ESP300_TIMEOUT: Duration = Duration::from_secs(5);

/// Default XPS FTP account and trajectory directory. C uses the factory
/// `Administrator`/`Administrator` login; `/Admin/Public/Trajectories` is the
/// XPS-C/Q public trajectory folder. Both are overridable on `XPSBuildProfile`.
const XPS_FTP_USER: &str = "Administrator";
const XPS_FTP_PASSWORD: &str = "Administrator";
const XPS_TRAJECTORY_DIR: &str = "/Admin/Public/Trajectories";

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

/// A shared HXP hexapod controller registered by `HXPCreateController` (its
/// six axes are created with it). Keeps a dedicated `Fire`-mode move socket
/// for the driver-private commands (`HXPMoveAll` blocks until the motion
/// finishes on a `Query` socket, so it fires like the axis moves do).
struct HxpRegistration {
    controller: Arc<Mutex<HxpController>>,
    move_sock: XpsSocket,
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
    /// HXP hexapod controllers keyed by motor port, for the driver-private
    /// commands (`HXPMoveAll`, coordinate-system read/set, move coord-sys).
    hxp_controllers: Mutex<HashMap<String, HxpRegistration>>,
}

impl NewportHolder {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            motors: Mutex::new(HashMap::new()),
            poll_senders: Mutex::new(Vec::new()),
            ag_uc_controllers: Mutex::new(HashMap::new()),
            xps_controllers: Mutex::new(HashMap::new()),
            hxp_controllers: Mutex::new(HashMap::new()),
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
    /// `SMC100CreateController(motorPort, serialPort, wireScale, [movingPollMs], [idlePollMs])`
    ///
    /// `wireScale` is the controller-units-per-record-EGU factor — 1.0 in the
    /// normal configuration where the record EGU is the controller's native
    /// unit (mm). It fills C's `eguPerStep` argument slot but is NOT that
    /// value: the asyn-rs record boundary is EGU, not raw steps (see
    /// [`crate::smc100`] module Units note).
    pub fn smc100_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "SMC100CreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("serialPort"),
                ArgDesc {
                    name: "wireScale",
                    arg_type: ArgType::Double,
                    optional: false,
                },
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "SMC100CreateController(motorPort, serialPort, wireScale, [movingPollMs], [idlePollMs]) - Create a Newport SMC100 controller (wireScale: controller units per record EGU, normally 1.0)",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let serial_port = req_string(args, 1, "serialPort")?;
                let wire_scale = match &args[2] {
                    ArgValue::Double(v) => *v,
                    _ => return Err("wireScale must be a number".into()),
                };
                if wire_scale == 0.0 {
                    return Err("wireScale must be non-zero".into());
                }
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

                let handle = connect_serial(&serial_port, SMC100_TIMEOUT)?;
                let dtyp_key = format!("SMC100_{motor_port}");
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(Smc100Axis::new(handle, SMC100_AXIS, wire_scale)));

                holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                println!(
                    "SMC100CreateController: motorPort={motor_port} serialPort={serial_port} wireScale={wire_scale} poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP={dtyp_key})"
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
    /// `stepsPerUnit` fills C's argument slot; the driver applies
    /// `1 / stepsPerUnit` as a record-EGU → positioner-units wire scale, so
    /// pass 1 in the normal configuration where the record EGU is the
    /// positioner's native unit — NOT the C steps-per-unit value (the asyn-rs
    /// record boundary is EGU, not raw steps). [`XpsAxis::new`]
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

    /// Create the `XPSTclScriptExecute` iocsh command.
    ///
    /// Usage:
    /// `XPSTclScriptExecute(motorPort, tclFile, [taskName], [parameters])`
    ///
    /// Runs a TCL script file on the XPS registered under `motorPort`, over its
    /// shared poll socket. Ports `TCLScriptExecute`; `taskName`/`parameters`
    /// default to `"0"` as the C driver does. DEVIATION from C's record-driven
    /// trigger (`XPSTclScript_`/`XPSTclScriptExecute_` asyn params): this is an
    /// imperative iocsh command, the natural interface for a one-shot script.
    pub fn xps_tcl_script_execute_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSTclScriptExecute",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("tclFile"),
                ArgDesc {
                    name: "taskName",
                    arg_type: ArgType::String,
                    optional: true,
                },
                ArgDesc {
                    name: "parameters",
                    arg_type: ArgType::String,
                    optional: true,
                },
            ],
            "XPSTclScriptExecute(motorPort, tclFile, [taskName], [parameters]) - Run a TCL script file on a Newport XPS controller",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let tcl_file = req_string(args, 1, "tclFile")?;
                let task_name = opt_string(args, 2).unwrap_or_else(|| "0".to_string());
                let parameters = opt_string(args, 3).unwrap_or_else(|| "0".to_string());

                let controller = {
                    let controllers = holder
                        .xps_controllers
                        .lock()
                        .unwrap_or_else(|e| e.into_inner());
                    controllers
                        .get(&motor_port)
                        .map(|reg| reg.controller.clone())
                        .ok_or_else(|| {
                            format!(
                                "XPSTclScriptExecute: controller '{motor_port}' not found (call XPSCreateController first)"
                            )
                        })?
                };
                {
                    let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                    ctrl.poll_socket()
                        .tcl_script_execute(&tcl_file, &task_name, &parameters)
                        .map_err(|e| format!("XPSTclScriptExecute: {e}"))?;
                }
                println!(
                    "XPSTclScriptExecute: motorPort={motor_port} tclFile={tcl_file} task={task_name} params={parameters}"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Look up a registered XPS controller by motor port, or produce a
    /// `"{cmd}: controller '...' not found"` error. Shared by the PVT commands.
    fn xps_controller(
        &self,
        motor_port: &str,
        cmd: &str,
    ) -> Result<Arc<Mutex<XpsController>>, String> {
        let controllers = self
            .xps_controllers
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        controllers
            .get(motor_port)
            .map(|reg| reg.controller.clone())
            .ok_or_else(|| {
                format!(
                    "{cmd}: controller '{motor_port}' not found (call XPSCreateController first)"
                )
            })
    }

    /// Create the `XPSPositionCompare` iocsh command.
    ///
    /// `XPSPositionCompare(motorPort, positionerName, mode, [minPosition],
    /// [maxPosition], [positionStep], [pulseWidth], [settlingTime])`
    ///
    /// Applies a position-compare-output configuration to one positioner
    /// (C `XPSAxis::setPositionCompare`): mode 0=Disable, 1=Pulse,
    /// 2=AquadB-windowed, 3=AquadB-always. Positions/step are device units;
    /// `pulseWidth`/`settlingTime` are µs (valid widths {0.2,1,2.5,10}, valid
    /// settling {0.075,1,4,12}), defaulting to the smallest table entries.
    /// Driver-private: the C base-class PCO API (05b25c1d, motor PR #248) is an
    /// open, unmerged PR, so PCO is not part of the motor framework.
    pub fn xps_position_compare_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSPositionCompare",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("positionerName"),
                ArgDesc {
                    name: "mode",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                arg_double_opt("minPosition"),
                arg_double_opt("maxPosition"),
                arg_double_opt("positionStep"),
                arg_double_opt("pulseWidth"),
                arg_double_opt("settlingTime"),
            ],
            "XPSPositionCompare(motorPort, positionerName, mode, [minPosition], [maxPosition], [positionStep], [pulseWidth], [settlingTime]) - Configure position-compare output on an XPS positioner (mode 0=Disable 1=Pulse 2=AquadB-windowed 3=AquadB-always; positions in device units, widths/settling in us)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let positioner = req_string(args, 1, "positionerName")?;
                let mode = match args.get(2) {
                    Some(ArgValue::Int(v)) => *v as i32,
                    _ => return Err("mode must be an integer".into()),
                };
                let params = PcoParams {
                    mode,
                    min_position: opt_double(args, 3, 0.0, "minPosition")?,
                    max_position: opt_double(args, 4, 0.0, "maxPosition")?,
                    position_step: opt_double(args, 5, 0.0, "positionStep")?,
                    pulse_width_us: opt_double(
                        args,
                        6,
                        pco::XPS_PCO_DEFAULT_PULSE_WIDTH,
                        "pulseWidth",
                    )?,
                    settling_time_us: opt_double(
                        args,
                        7,
                        pco::XPS_PCO_DEFAULT_SETTLING_TIME,
                        "settlingTime",
                    )?,
                };

                let controller = holder.xps_controller(&motor_port, "XPSPositionCompare")?;
                {
                    let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                    pco::set_position_compare(ctrl.poll_socket(), &positioner, &params)
                        .map_err(|e| format!("XPSPositionCompare: {e}"))?;
                }
                println!(
                    "XPSPositionCompare: motorPort={motor_port} positioner={positioner} mode={mode} min={} max={} step={} pw={}us settle={}us",
                    params.min_position,
                    params.max_position,
                    params.position_step,
                    params.pulse_width_us,
                    params.settling_time_us,
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `XPSDefineProfileFromFile` iocsh command.
    ///
    /// `XPSDefineProfileFromFile(motorPort, group, csvFile, [moveMode])`
    ///
    /// Loads a PVT profile for `group` from a CSV points file: each row is
    /// `time, pos_0, pos_1, ...` with one position column per positioner in the
    /// group (controller registration order), in device units and seconds; `#`
    /// comments and blank lines are skipped. `moveMode` is `absolute` (default)
    /// or `relative`, selecting whether the execute-time move to the trajectory
    /// start is absolute or relative. Driver-private: the epics-rs motor
    /// framework has no profileMove record subsystem, so profiles are defined by
    /// this command instead of by the C `profileMove` database.
    pub fn xps_define_profile_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSDefineProfileFromFile",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("group"),
                arg_str_req("csvFile"),
                ArgDesc {
                    name: "moveMode",
                    arg_type: ArgType::String,
                    optional: true,
                },
            ],
            "XPSDefineProfileFromFile(motorPort, group, csvFile, [moveMode]) - Load a PVT profile for an XPS group from a CSV points file (time + one position column per group positioner; moveMode absolute|relative, default absolute)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let group = req_string(args, 1, "group")?;
                let csv_file = req_string(args, 2, "csvFile")?;
                let move_mode = parse_move_mode(opt_string(args, 3))?;

                let controller = holder.xps_controller(&motor_port, "XPSDefineProfileFromFile")?;
                let csv = std::fs::read_to_string(&csv_file).map_err(|e| {
                    format!("XPSDefineProfileFromFile: cannot read '{csv_file}': {e}")
                })?;

                let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                let positioners = ctrl.positioners_in_group(&group);
                let num_axes = positioners.len();
                let profile = Profile::from_csv(&group, move_mode, &positioners, &csv)
                    .map_err(|e| format!("XPSDefineProfileFromFile: {e}"))?;
                let num_points = profile.num_points();
                ctrl.define_profile(profile)
                    .map_err(|e| format!("XPSDefineProfileFromFile: {e}"))?;
                drop(ctrl);
                println!(
                    "XPSDefineProfileFromFile: motorPort={motor_port} group={group} points={num_points} axes={num_axes} mode={move_mode:?}"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `XPSBuildProfile` iocsh command.
    ///
    /// `XPSBuildProfile(motorPort, fileName, host, [ftpUser], [ftpPassword], [ftpDir])`
    ///
    /// Generates the defined profile's trajectory file, FTP-uploads it to the
    /// XPS trajectory folder, then verifies it against the group dynamics and
    /// each axis's soft limits (C `buildProfile`). `fileName` is the bare
    /// trajectory file name the controller opens; `host` is the XPS IP/hostname
    /// (no port). Credentials and directory default to the XPS factory
    /// `Administrator` account and `/Admin/Public/Trajectories`; `ftpDir` must
    /// name the controller's actual trajectory folder if overridden.
    pub fn xps_build_profile_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSBuildProfile",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("fileName"),
                arg_str_req("host"),
                ArgDesc {
                    name: "ftpUser",
                    arg_type: ArgType::String,
                    optional: true,
                },
                ArgDesc {
                    name: "ftpPassword",
                    arg_type: ArgType::String,
                    optional: true,
                },
                ArgDesc {
                    name: "ftpDir",
                    arg_type: ArgType::String,
                    optional: true,
                },
            ],
            "XPSBuildProfile(motorPort, fileName, host, [ftpUser], [ftpPassword], [ftpDir]) - Generate the defined profile's trajectory file, FTP it to the XPS, and verify it against dynamics + soft limits",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let file_name = req_string(args, 1, "fileName")?;
                let host = req_string(args, 2, "host")?;
                let ftp_user = opt_string(args, 3).unwrap_or_else(|| XPS_FTP_USER.to_string());
                let ftp_pass = opt_string(args, 4).unwrap_or_else(|| XPS_FTP_PASSWORD.to_string());
                let ftp_dir = opt_string(args, 5).unwrap_or_else(|| XPS_TRAJECTORY_DIR.to_string());

                let controller = holder.xps_controller(&motor_port, "XPSBuildProfile")?;

                // Generate and verify hold the controller lock (they use the
                // poll socket); the FTP upload touches no controller state, so
                // it runs between the two without the lock held.
                let built = {
                    let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                    ctrl.build_profile_text()
                        .map_err(|e| format!("XPSBuildProfile: {e}"))?
                };
                ftp::upload_trajectory(
                    &host,
                    &ftp_dir,
                    &file_name,
                    &built.text,
                    &ftp_user,
                    &ftp_pass,
                )
                .map_err(|e| format!("XPSBuildProfile: {e}"))?;
                {
                    let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                    ctrl.verify_profile(&file_name, &built)
                        .map_err(|e| format!("XPSBuildProfile: {e}"))?;
                }
                println!(
                    "XPSBuildProfile: motorPort={motor_port} file={file_name} uploaded to {host}:{ftp_dir} and verified"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `XPSExecuteProfile` iocsh command.
    ///
    /// `XPSExecuteProfile(motorPort, execPort, [executions], [startPulses],
    /// [endPulses], [numPulses])`
    ///
    /// Moves the group to the trajectory start, configures gathering (each
    /// trajectory pulse samples every registered positioner's
    /// `SetpointPosition`+`CurrentPosition` via a `GatheringOneData` event),
    /// then runs the built PVT profile with `MultipleAxesPVTExecution`. The
    /// moves and the execution block until the motion finishes, so everything
    /// runs on a background thread over a dedicated socket (`execPort`, a
    /// `drvAsynIPPort` to the XPS) — the shared poll socket keeps polling
    /// meanwhile (C configures gathering on the poll socket; same RPCs, our
    /// exec socket keeps the poll socket uncontended). `executions` (default 1)
    /// repeats the trajectory. `startPulses`/`endPulses` (1-based trajectory
    /// elements, defaults: the whole profile) window the pulse output and
    /// `numPulses` (default one per element) spreads that many pulses over it,
    /// exactly as C `executeProfile`. Read the samples back afterwards with
    /// `XPSReadbackProfile`.
    pub fn xps_execute_profile_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSExecuteProfile",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("execPort"),
                arg_int_opt("executions"),
                arg_int_opt("startPulses"),
                arg_int_opt("endPulses"),
                arg_int_opt("numPulses"),
            ],
            "XPSExecuteProfile(motorPort, execPort, [executions], [startPulses], [endPulses], [numPulses]) - Move the group to the trajectory start and run the built PVT profile with gathering on a background thread over its own socket (execPort: a drvAsynIPPort to the XPS; executions default 1; pulse window defaults to one pulse per trajectory element)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let exec_port = req_string(args, 1, "execPort")?;
                let executions = match args.get(2) {
                    Some(ArgValue::Int(v)) if *v >= 1 => *v as i32,
                    None | Some(ArgValue::Missing) => 1,
                    Some(ArgValue::Int(_)) => return Err("executions must be >= 1".into()),
                    _ => return Err("executions must be an integer".into()),
                };

                let controller = holder.xps_controller(&motor_port, "XPSExecuteProfile")?;
                let plan = {
                    let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                    ctrl.execution_plan()
                }
                .ok_or_else(|| {
                    "XPSExecuteProfile: no built profile (call XPSBuildProfile first)".to_string()
                })?;

                // Pulse window: default one pulse per trajectory element over
                // the whole profile (C reads these from records).
                let num_elements = plan.times.len().saturating_sub(1) as i64;
                let start_pulses = opt_int(args, 3, 1, "startPulses")?;
                let end_pulses = opt_int(args, 4, num_elements, "endPulses")?;
                let num_pulses = opt_int(args, 5, num_elements, "numPulses")?;
                let (pulse_start, pulse_end, pulse_period) = profile::pulse_output_window(
                    &plan.times,
                    start_pulses as i32,
                    end_pulses as i32,
                    num_pulses as i32,
                )
                .map_err(|e| format!("XPSExecuteProfile: {e}"))?;

                // A dedicated blocking socket so the seconds-to-minutes-long
                // execution never contends with the shared poll socket.
                let handle = connect_ip(&exec_port, XPS_PVT_EXEC_TIMEOUT)?;
                let exec_sock = XpsSocket::new(handle, SocketMode::Query);

                println!(
                    "XPSExecuteProfile: motorPort={motor_port} group={} file={} executions={executions} pulses={num_pulses} over elements {start_pulses}..{end_pulses} (running on background thread)",
                    plan.group, plan.file_name
                );
                thread::spawn(move || {
                    for (positioner, target) in &plan.start_moves {
                        let moved = match plan.move_mode {
                            MoveMode::Absolute => {
                                exec_sock.group_move_absolute(positioner, *target)
                            }
                            MoveMode::Relative => {
                                exec_sock.group_move_relative(positioner, *target)
                            }
                        };
                        if let Err(e) = moved {
                            eprintln!(
                                "XPSExecuteProfile: move to start for {positioner} failed: {e}"
                            );
                            return;
                        }
                    }

                    // Configure gathering + the per-pulse sampling event
                    // (C executeProfile order: reset, configure, pulse window,
                    // trigger, action, start).
                    let event_id = match Self::xps_start_gathering(
                        &exec_sock,
                        &plan,
                        pulse_start,
                        pulse_end,
                        pulse_period,
                    ) {
                        Ok(id) => id,
                        Err(e) => {
                            eprintln!("XPSExecuteProfile: {e}");
                            return;
                        }
                    };

                    let run = exec_sock.multiple_axes_pvt_execution(
                        &plan.group,
                        &plan.file_name,
                        executions,
                    );

                    // Tear down the event and stop gathering even on a failed
                    // or aborted run (C removes the event and stops gathering
                    // unconditionally after MultipleAxesPVTExecution).
                    if let Err(e) = exec_sock.event_extended_remove(event_id) {
                        eprintln!("XPSExecuteProfile: EventExtendedRemove failed: {e}");
                    }
                    match exec_sock.gathering_stop() {
                        // -30: gathering never started (aborted before one
                        // element completed); C tolerates it.
                        Ok(()) | Err(XpsError::Api(-30)) => {}
                        Err(e) => eprintln!("XPSExecuteProfile: GatheringStop failed: {e}"),
                    }

                    match run {
                        Ok(()) => println!(
                            "XPSExecuteProfile: trajectory '{}' complete (read samples with XPSReadbackProfile)",
                            plan.file_name
                        ),
                        // -27: the trajectory was aborted (C reports it as such).
                        Err(XpsError::Api(-27)) => {
                            eprintln!("XPSExecuteProfile: MultipleAxesPVTExecution aborted")
                        }
                        Err(e) => eprintln!(
                            "XPSExecuteProfile: execution of '{}' failed: {e}",
                            plan.file_name
                        ),
                    }
                });
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Configure and start gathering for a trajectory run: reset the buffer,
    /// declare `SetpointPosition`+`CurrentPosition` per registered positioner,
    /// window the trajectory pulse train, and fire `GatheringOneData` on every
    /// pulse (C `executeProfile`, XPSController.cpp:935-1060). Returns the
    /// extended-event ID to remove after the run.
    fn xps_start_gathering(
        sock: &XpsSocket,
        plan: &ExecutionPlan,
        pulse_start: i32,
        pulse_end: i32,
        pulse_period: f64,
    ) -> Result<i32, String> {
        sock.gathering_reset()
            .map_err(|e| format!("GatheringReset failed: {e}"))?;
        let types: Vec<String> = plan
            .gathering_positioners
            .iter()
            .flat_map(|p| {
                [
                    format!("{p}.SetpointPosition"),
                    format!("{p}.CurrentPosition"),
                ]
            })
            .collect();
        sock.gathering_configuration_set(&types)
            .map_err(|e| format!("GatheringConfigurationSet failed: {e}"))?;
        sock.multiple_axes_pvt_pulse_output_set(&plan.group, pulse_start, pulse_end, pulse_period)
            .map_err(|e| format!("MultipleAxesPVTPulseOutputSet failed: {e}"))?;
        let triggers = vec![
            "Always".to_string(),
            format!("{}.PVT.TrajectoryPulse", plan.group),
        ];
        sock.event_extended_configuration_trigger_set(&triggers)
            .map_err(|e| format!("EventExtendedConfigurationTriggerSet failed: {e}"))?;
        sock.event_extended_configuration_action_set(&["GatheringOneData".to_string()])
            .map_err(|e| format!("EventExtendedConfigurationActionSet failed: {e}"))?;
        sock.event_extended_start()
            .map_err(|e| format!("EventExtendedStart failed: {e}"))
    }

    /// Create the `XPSReadbackProfile` iocsh command.
    ///
    /// `XPSReadbackProfile(motorPort, outputFile)`
    ///
    /// Reads the gathering samples collected during the last
    /// `XPSExecuteProfile` run over the shared poll socket
    /// (`GatheringCurrentNumberGet` + chunked `GatheringDataMultipleLinesGet`,
    /// halving the request on a controller error exactly as C
    /// `readbackProfile`) and writes them to `outputFile` as CSV — one row per
    /// trajectory pulse with `actual, following_error` per registered
    /// positioner, the file-based counterpart of C posting the readback
    /// waveforms (positions are device units, like the profile CSV).
    pub fn xps_readback_profile_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "XPSReadbackProfile",
            vec![arg_str_req("motorPort"), arg_str_req("outputFile")],
            "XPSReadbackProfile(motorPort, outputFile) - Read the gathering samples from the last XPSExecuteProfile run and write them to outputFile as CSV (actual position + following error per registered positioner, device units)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let output_file = req_string(args, 1, "outputFile")?;

                let controller = holder.xps_controller(&motor_port, "XPSReadbackProfile")?;
                let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                let positioners = ctrl.registered_positioners();
                if positioners.is_empty() {
                    return Err("XPSReadbackProfile: no axes registered".into());
                }
                let sock = ctrl.poll_socket();

                let (current_samples, _max_samples) = sock
                    .gathering_current_number_get()
                    .map_err(|e| format!("GatheringCurrentNumberGet failed: {e}"))?;

                let mut readback = GatheringReadback::new(positioners.len());
                let mut num_read = 0i32;
                while num_read < current_samples {
                    // Ask for everything left; on a controller error halve the
                    // request until it fits (C readbackProfile).
                    let mut lines = current_samples - num_read;
                    let buffer = loop {
                        match sock.gathering_data_multiple_lines_get(num_read, lines) {
                            Ok(buf) => break buf,
                            Err(_) if lines > 1 => lines /= 2,
                            Err(e) => {
                                return Err(format!("GatheringDataMultipleLinesGet failed: {e}"));
                            }
                        }
                    };
                    let parsed = gathering::parse_gathering_buffer(
                        &buffer,
                        positioners.len(),
                        &mut readback,
                    )
                    .map_err(|e| format!("XPSReadbackProfile: {e}"))?;
                    if parsed == 0 {
                        return Err("XPSReadbackProfile: gathering returned no lines".into());
                    }
                    num_read += parsed as i32;
                }

                let csv = gathering::readback_csv(&positioners, &readback);
                std::fs::write(&output_file, csv)
                    .map_err(|e| format!("XPSReadbackProfile: writing {output_file}: {e}"))?;
                println!(
                    "XPSReadbackProfile: wrote {num_read} samples x {} positioners to {output_file}",
                    positioners.len()
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `HXPCreateController` iocsh command.
    ///
    /// `HXPCreateController(motorPort, pollPort, movePort, [movingPollMs],
    /// [idlePollMs])`
    ///
    /// Creates a Newport HXP hexapod controller and all six axes
    /// (X, Y, Z, U, V, W — DTYP `HXP_{motorPort}_{0..5}`) in one step, as the C
    /// `HXPController` constructor does. `pollPort`/`movePort` are
    /// `drvAsynIPPort`s to the hexapod (DEVIATION: C opens raw TCP itself from
    /// an IP address, one move socket per axis; here the six axes share the
    /// one `movePort` connection for their fire-and-forget moves — hexapod
    /// moves are whole-group commands, so they never overlap meaningfully).
    pub fn hxp_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "HXPCreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("pollPort"),
                arg_str_req("movePort"),
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "HXPCreateController(motorPort, pollPort, movePort, [movingPollMs], [idlePollMs]) - Create a Newport HXP hexapod controller with its six axes X,Y,Z,U,V,W (DTYP HXP_{motorPort}_{0..5})",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let poll_port = req_string(args, 1, "pollPort")?;
                let move_port = req_string(args, 2, "movePort")?;
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

                let handle = connect_ip(&poll_port, HXP_POLL_TIMEOUT)?;
                let poll_sock = XpsSocket::new(handle, SocketMode::Query);
                // One group poll serves all six axis polls per period: cache
                // for half the fastest poll interval.
                let cache_ttl = Duration::from_millis(moving_poll_ms / 2);
                let controller = Arc::new(Mutex::new(HxpController::new(poll_sock, cache_ttl)));

                for axis_no in 0..NUM_HXP_AXES {
                    let handle = connect_ip(&move_port, HXP_MOVE_TIMEOUT)?;
                    let move_sock = XpsSocket::new(handle, SocketMode::Fire);
                    let ax = HxpAxis::new(controller.clone(), move_sock, axis_no);
                    let dtyp_key = format!("HXP_{motor_port}_{axis_no}");
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
                }

                let handle = connect_ip(&move_port, HXP_MOVE_TIMEOUT)?;
                let move_sock = XpsSocket::new(handle, SocketMode::Fire);
                holder
                    .hxp_controllers
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .insert(
                        motor_port.clone(),
                        HxpRegistration {
                            controller,
                            move_sock,
                        },
                    );
                println!(
                    "HXPCreateController: motorPort={motor_port} pollPort={poll_port} movePort={move_port} axes=XYZUVW poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=HXP_{motor_port}_{{0..5}})"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Run `f` with the HXP registration for `motor_port` (the sockets are not
    /// clonable, so callers borrow the registration under the map lock — every
    /// hexapod command RPC is a single bounded exchange). Shared by the
    /// driver-private hexapod commands.
    fn with_hxp_registration<R>(
        &self,
        motor_port: &str,
        cmd: &str,
        f: impl FnOnce(&HxpRegistration) -> Result<R, String>,
    ) -> Result<R, String> {
        let controllers = self
            .hxp_controllers
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let reg = controllers.get(motor_port).ok_or_else(|| {
            format!("{cmd}: controller '{motor_port}' not found (call HXPCreateController first)")
        })?;
        f(reg)
    }

    /// Create the `HXPMoveAll` iocsh command.
    ///
    /// `HXPMoveAll(motorPort, x, y, z, u, v, w)`
    ///
    /// Moves all six hexapod axes to absolute Work-coordinate targets with a
    /// single `HexapodMoveAbsolute` (C `HXPController::moveAll`, which reads
    /// the targets from the `HXP_MOVE_ALL_TARGET_*` records; here they are
    /// arguments). Fire-and-forget on the move socket — poll the motor records
    /// for completion.
    pub fn hxp_move_all_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "HXPMoveAll",
            vec![
                arg_str_req("motorPort"),
                arg_double_req("x"),
                arg_double_req("y"),
                arg_double_req("z"),
                arg_double_req("u"),
                arg_double_req("v"),
                arg_double_req("w"),
            ],
            "HXPMoveAll(motorPort, x, y, z, u, v, w) - Move all six hexapod axes to absolute Work-coordinate targets in one motion (device units)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let mut targets = [0.0; 6];
                for (i, t) in targets.iter_mut().enumerate() {
                    *t = req_double(args, i + 1, ["x", "y", "z", "u", "v", "w"][i])?;
                }
                holder.with_hxp_registration(&motor_port, "HXPMoveAll", |reg| {
                    reg.move_sock
                        .hexapod_move_absolute(HXP_GROUP, "Work", &targets)
                        .map_err(|e| format!("HXPMoveAll: {e}"))
                })?;
                println!(
                    "HXPMoveAll: motorPort={motor_port} targets={targets:?} (Work coordinates, fire-and-forget)"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `HXPCoordSysRead` iocsh command.
    ///
    /// `HXPCoordSysRead(motorPort)`
    ///
    /// Reads and prints the Tool, Work, and Base coordinate-system definitions
    /// (C `HXPController::readAllCS`, which posts them to the
    /// `HXP_COORD_SYS_*` records; here they are printed). Runs on the poll
    /// socket (DEVIATION: C reads over the move socket's 0.1 s best-effort
    /// read; the `Query` poll socket waits for the reply reliably).
    pub fn hxp_coord_sys_read_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "HXPCoordSysRead",
            vec![arg_str_req("motorPort")],
            "HXPCoordSysRead(motorPort) - Read and print the hexapod Tool/Work/Base coordinate-system definitions",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let controller =
                    holder.with_hxp_registration(&motor_port, "HXPCoordSysRead", |reg| {
                        Ok(reg.controller.clone())
                    })?;
                let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                let sock = ctrl.poll_socket();
                for cs in ["Tool", "Work", "Base"] {
                    let p = sock
                        .hexapod_coordinate_system_get(HXP_GROUP, cs)
                        .map_err(|e| format!("HXPCoordSysRead: {cs}: {e}"))?;
                    println!(
                        "HXPCoordSysRead: {cs:<4} X={:.6} Y={:.6} Z={:.6} U={:.6} V={:.6} W={:.6}",
                        p[0], p[1], p[2], p[3], p[4], p[5]
                    );
                }
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `HXPCoordSysSet` iocsh command.
    ///
    /// `HXPCoordSysSet(motorPort, coordSystem, x, y, z, u, v, w)`
    ///
    /// Redefines the origin of one hexapod coordinate system
    /// (`Work`/`Tool`/`Base`, or C's record encoding `1`/`2`/`3`) —
    /// C `HXPController::setCS`. Runs on the poll socket (same DEVIATION as
    /// `HXPCoordSysRead`).
    pub fn hxp_coord_sys_set_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "HXPCoordSysSet",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("coordSystem"),
                arg_double_req("x"),
                arg_double_req("y"),
                arg_double_req("z"),
                arg_double_req("u"),
                arg_double_req("v"),
                arg_double_req("w"),
            ],
            "HXPCoordSysSet(motorPort, coordSystem, x, y, z, u, v, w) - Redefine the origin of a hexapod coordinate system (Work/Tool/Base)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let cs_arg = req_string(args, 1, "coordSystem")?;
                let cs = match cs_arg.to_ascii_lowercase().as_str() {
                    "work" | "1" => "Work",
                    "tool" | "2" => "Tool",
                    "base" | "3" => "Base",
                    other => {
                        return Err(format!(
                            "coordSystem must be Work, Tool or Base, got '{other}'"
                        ));
                    }
                };
                let mut origin = [0.0; 6];
                for (i, t) in origin.iter_mut().enumerate() {
                    *t = req_double(args, i + 2, ["x", "y", "z", "u", "v", "w"][i])?;
                }
                let controller =
                    holder.with_hxp_registration(&motor_port, "HXPCoordSysSet", |reg| {
                        Ok(reg.controller.clone())
                    })?;
                let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                ctrl.poll_socket()
                    .hexapod_coordinate_system_set(HXP_GROUP, cs, &origin)
                    .map_err(|e| format!("HXPCoordSysSet: {e}"))?;
                println!("HXPCoordSysSet: motorPort={motor_port} {cs} origin={origin:?}");
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `HXPSetMoveCoordSys` iocsh command.
    ///
    /// `HXPSetMoveCoordSys(motorPort, coordSystem)`
    ///
    /// Selects the coordinate system motor-record moves use: `Work`/`0`
    /// (default) or `Tool`/`1` — the C `HXP_MOVE_COORD_SYS` record parameter
    /// as a command.
    pub fn hxp_set_move_coord_sys_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "HXPSetMoveCoordSys",
            vec![arg_str_req("motorPort"), arg_str_req("coordSystem")],
            "HXPSetMoveCoordSys(motorPort, coordSystem) - Select the coordinate system for motor-record moves: Work/0 (default) or Tool/1",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let cs_arg = req_string(args, 1, "coordSystem")?;
                let cs = match cs_arg.to_ascii_lowercase().as_str() {
                    "work" | "0" => MoveCoordSys::Work,
                    "tool" | "1" => MoveCoordSys::Tool,
                    other => {
                        return Err(format!("coordSystem must be Work or Tool, got '{other}'"));
                    }
                };
                let controller =
                    holder.with_hxp_registration(&motor_port, "HXPSetMoveCoordSys", |reg| {
                        Ok(reg.controller.clone())
                    })?;
                controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .set_move_coord_sys(cs);
                println!("HXPSetMoveCoordSys: motorPort={motor_port} coordSystem={cs:?}");
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Create the `ESP300CreateController` iocsh command.
    ///
    /// `ESP300CreateController(motorPort, serialPort, [movingPollMs],
    /// [idlePollMs])`
    ///
    /// Creates a Newport ESP100/ESP300/ESP301 controller on a pre-configured
    /// serial (or GPIB octet) port, discovers its axis count (C `motor_init`:
    /// stop each axis until "axis number out of range"), and creates one motor
    /// axis per discovered stage (DTYP `ESP300_{motorPort}_{0..}`). Replaces
    /// the C `ESP300Setup`/`ESP300Config` pair; the scan rate is the poll
    /// intervals.
    pub fn esp300_create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "ESP300CreateController",
            vec![
                arg_str_req("motorPort"),
                arg_str_req("serialPort"),
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "ESP300CreateController(motorPort, serialPort, [movingPollMs], [idlePollMs]) - Create a Newport ESP100/300/301 controller, discovering its axes (DTYP ESP300_{motorPort}_{0..})",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let serial_port = req_string(args, 1, "serialPort")?;
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

                let handle = connect_serial(&serial_port, ESP300_TIMEOUT)?;
                let controller = Esp300Controller::new(handle)
                    .map_err(|e| format!("ESP300CreateController: {e}"))?;
                let ident = controller.ident().to_string();
                let num_axes = controller.num_axes();
                if num_axes == 0 {
                    return Err(format!(
                        "ESP300CreateController: no axes found on '{serial_port}' (ident \"{ident}\")"
                    ));
                }
                let controller = Arc::new(Mutex::new(controller));

                for axis in 1..=num_axes {
                    let ax = Esp300Axis::new(controller.clone(), axis)
                        .map_err(|e| format!("ESP300CreateController: axis {axis}: {e}"))?;
                    let dtyp_key = format!("ESP300_{motor_port}_{}", axis - 1);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
                }
                println!(
                    "ESP300CreateController: motorPort={motor_port} serialPort={serial_port} ident=\"{ident}\" axes={num_axes} poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=ESP300_{motor_port}_{{0..{}}})",
                    num_axes - 1
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

fn arg_double_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Double,
        optional: true,
    }
}

fn arg_double_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Double,
        optional: false,
    }
}

/// Read a required double arg.
fn req_double(args: &[ArgValue], i: usize, name: &str) -> Result<f64, String> {
    match args.get(i) {
        Some(ArgValue::Double(v)) => Ok(*v),
        _ => Err(format!("{name} must be a number")),
    }
}

/// Read an optional double arg, defaulting when absent.
fn opt_double(args: &[ArgValue], i: usize, default: f64, name: &str) -> Result<f64, String> {
    match args.get(i) {
        Some(ArgValue::Double(v)) => Ok(*v),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be a number")),
    }
}

/// Read an optional integer arg, defaulting when absent.
fn opt_int(args: &[ArgValue], i: usize, default: i64, name: &str) -> Result<i64, String> {
    match args.get(i) {
        Some(ArgValue::Int(v)) => Ok(*v),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be an integer")),
    }
}

/// Parse the optional `moveMode` arg for the PVT commands: `absolute`/`abs`/`0`
/// (default) or `relative`/`rel`/`1`.
fn parse_move_mode(arg: Option<String>) -> Result<MoveMode, String> {
    match arg.as_deref().map(str::to_ascii_lowercase).as_deref() {
        None | Some("") | Some("absolute") | Some("abs") | Some("0") => Ok(MoveMode::Absolute),
        Some("relative") | Some("rel") | Some("1") => Ok(MoveMode::Relative),
        Some(other) => Err(format!(
            "moveMode must be absolute or relative, got '{other}'"
        )),
    }
}

fn req_string(args: &[ArgValue], i: usize, name: &str) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.clone()),
        _ => Err(format!("{name} must be a string")),
    }
}

/// Read an optional string arg: `Some` if a non-empty string was given, else
/// `None` (absent or `Missing`).
fn opt_string(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) if !s.is_empty() => Some(s.clone()),
        _ => None,
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
