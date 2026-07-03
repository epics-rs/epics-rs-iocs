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
use crate::conex::ConexAxis;
use crate::smc100::Smc100Axis;

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

/// Holds Newport motor device-support instances created by the family's
/// `*CreateController` commands. Each controller is stored under a
/// `"{PREFIX}_{motor_port}"` DTYP key and consumed once by the dynamic
/// device-support factory during iocInit.
pub struct NewportHolder {
    motors: Mutex<HashMap<String, Option<MotorDeviceSupport>>>,
    poll_senders: Mutex<Vec<tokio::sync::mpsc::Sender<PollCommand>>>,
}

impl NewportHolder {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            motors: Mutex::new(HashMap::new()),
            poll_senders: Mutex::new(Vec::new()),
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
