//! iocsh command for the Aerotech Ensemble driver.
//!
//! Mirrors the C `EnsembleAsynSetup` + `EnsembleAsynConfig` pair collapsed into
//! one command: `EnsembleAsynConfig` connects the asyn octet port, pings the
//! controller, then probes axes `0..ENSEMBLE_MAX_AXES` and attaches the first
//! `numAxes` that exist (their `AxisName` parameter ACKs) to `DTYP`-keyed motor
//! device supports (`ENSEMBLE_{card}_{axis}`, using the controller axis number).
//! Both port kinds (`drvAsynIPPort` / `drvAsynSerialPort`) work, so `connect_ip`
//! is used (the lookup is transport-agnostic).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_req, opt_double, opt_int, poll_intervals,
    req_int, req_string,
};

use crate::a3200::{A3200_MAX_AXES, A3200Axis, A3200Controller};
use crate::ensemble::{ENSEMBLE_MAX_AXES, EnsembleAxis, EnsembleController};

/// Default communication timeout when the `timeoutMs` arg is omitted.
const DEFAULT_TIMEOUT_MS: f64 = 2000.0;

/// Build the `EnsembleAsynConfig(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs], [timeoutMs])` command bound to `holder`.
pub fn ensemble_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "EnsembleAsynConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_double_opt("timeoutMs"),
        ],
        "EnsembleAsynConfig(card, asynPort, numAxes, [movingPollMs], [idlePollMs], \
         [timeoutMs]) - Create an Aerotech Ensemble controller (DTYP ENSEMBLE_{card}_{axis}) \
         with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if !(1..=ENSEMBLE_MAX_AXES as i64).contains(&num_axes) {
                return Err(format!(
                    "EnsembleAsynConfig: numAxes must be 1..={ENSEMBLE_MAX_AXES}"
                ));
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let timeout_ms = opt_double(args, 5, DEFAULT_TIMEOUT_MS, "timeoutMs")?;

            let handle = connect_ip(
                &asyn_port,
                Duration::from_millis(timeout_ms.max(0.0) as u64),
            )?;
            let controller = Arc::new(Mutex::new(EnsembleController::new(handle)));
            {
                let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                ctrl.ping()
                    .map_err(|e| format!("EnsembleAsynConfig: no response from controller: {e}"))?;
                ctrl.wait_mode_nowait()
                    .map_err(|e| format!("EnsembleAsynConfig: WAIT MODE NOWAIT failed: {e}"))?;
            }

            let mut found = 0i64;
            let mut axis = 0i32;
            while axis < ENSEMBLE_MAX_AXES && found < num_axes {
                let exists = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .axis_exists(axis);
                if exists {
                    let ax = EnsembleAxis::new(controller.clone(), axis)
                        .map_err(|e| format!("EnsembleAsynConfig: axis {axis}: {e}"))?;
                    let dtyp_key = format!("ENSEMBLE_{card}_{axis}");
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                    println!("EnsembleAsynConfig: {dtyp_key} (axis={axis})");
                    found += 1;
                }
                axis += 1;
            }

            if found < num_axes {
                return Err(format!(
                    "EnsembleAsynConfig: found only {found} of {num_axes} requested axes"
                ));
            }
            println!(
                "EnsembleAsynConfig: card={card} asynPort={asyn_port} axes={found} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `A3200AsynConfig(card, asynPort, numAxes, [taskNumber], [linear],
/// [movingPollMs], [idlePollMs], [timeoutMs])` command bound to `holder`.
///
/// Unlike the Ensemble (which probes for existing axes by index), the A3200
/// discovers each axis `0..numAxes` by its name string (`GETPARMSTRING`) and
/// installs it at `DTYP A3200_{card}_{axisName}`. `taskNumber` defaults to 1 and
/// `linear` (linear vs single-axis move commands) defaults to 1.
pub fn a3200_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "A3200AsynConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("taskNumber"),
            arg_int_opt("linear"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_double_opt("timeoutMs"),
        ],
        "A3200AsynConfig(card, asynPort, numAxes, [taskNumber], [linear], [movingPollMs], \
         [idlePollMs], [timeoutMs]) - Create an Aerotech A3200 controller (DTYP \
         A3200_{card}_{axisName}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if !(1..=A3200_MAX_AXES as i64).contains(&num_axes) {
                return Err(format!(
                    "A3200AsynConfig: numAxes must be 1..={A3200_MAX_AXES}"
                ));
            }
            let task_number = opt_int(args, 3, 1, "taskNumber")?;
            if task_number < 0 {
                return Err("A3200AsynConfig: taskNumber must be >= 0".to_string());
            }
            let linear = opt_int(args, 4, 1, "linear")? != 0;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 5, 6)?;
            let timeout_ms = opt_double(args, 7, DEFAULT_TIMEOUT_MS, "timeoutMs")?;

            let handle = connect_ip(
                &asyn_port,
                Duration::from_millis(timeout_ms.max(0.0) as u64),
            )?;
            let controller = Arc::new(Mutex::new(A3200Controller::new(
                handle,
                task_number as u32,
                linear,
            )));
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .init_task()
                .map_err(|e| format!("A3200AsynConfig: task init failed: {e}"))?;

            for axis in 0..num_axes as i32 {
                let axis_name = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .discover_axis_name(axis)
                    .map_err(|e| format!("A3200AsynConfig: axis {axis}: name discovery: {e}"))?;
                if axis_name.is_empty() {
                    return Err(format!("A3200AsynConfig: axis {axis}: empty axis name"));
                }
                let ax = A3200Axis::new(controller.clone(), axis_name.clone())
                    .map_err(|e| format!("A3200AsynConfig: axis {axis} ({axis_name}): {e}"))?;
                let dtyp_key = format!("A3200_{card}_{axis_name}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
                println!("A3200AsynConfig: {dtyp_key} (axis={axis} name={axis_name})");
            }

            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .finalize()
                .map_err(|e| format!("A3200AsynConfig: finalize failed: {e}"))?;

            println!(
                "A3200AsynConfig: card={card} asynPort={asyn_port} axes={num_axes} \
                 task={task_number} linear={linear} poll=[{moving_poll_ms}/{idle_poll_ms}]ms"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
