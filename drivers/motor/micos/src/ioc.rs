//! `SMCcorvusCreateController` iocsh command for the Micos SMC corvus.
//!
//! Mirrors C `SMCcorvusCreateController(portName, corvusPort, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and create
//! controller port `portName` whose `numAxes` axes records address as
//! `@asyn(portName,axis)`; hydra and Taurus likewise. The corvus connects over either a
//! `drvAsynIPPort` or a `drvAsynSerialPort`, so `connect_ip` is used (the lookup
//! is transport-agnostic).

use std::collections::HashMap;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_controller, connect_with_eos};
use motor_common::iocsh::{
    arg_double_req, arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_double, req_int,
    req_string,
};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::{
    Controller, ControllerParams, ControllerReport, asyn_user, model1_device, register_port,
    register_port_with,
};

use crate::corvus::{CorvusAxis, CorvusController};
use crate::drv_micos::{
    DrvMicosAxis, MICOS_NUM_AXIS, MICOS_NUM_CARDS, MicosController, micos_probe,
};
use crate::hydra::{HydraAxis, HydraController};
use crate::taurus::{TaurusAxis, TaurusController};

/// Command timeout.
const CORVUS_TIMEOUT: Duration = Duration::from_secs(2);

/// Hydra command timeout.
const HYDRA_TIMEOUT: Duration = Duration::from_secs(2);

/// Taurus command timeout.
const TAURUS_TIMEOUT: Duration = Duration::from_secs(2);

/// The SMC controllers' `report` override: their one line, then
/// `asynMotorController::report` (SMChydraDriver.cpp:86-93).
struct ControllerLine {
    /// What C's line names the driver.
    driver: &'static str,
    /// C's `movingPollPeriod_`/`idlePollPeriod_`, in seconds.
    moving_poll_s: f64,
    idle_poll_s: f64,
}

impl ControllerLine {
    fn new(driver: &'static str, moving_poll_ms: u64, idle_poll_ms: u64) -> Self {
        Self {
            driver,
            moving_poll_s: moving_poll_ms as f64 / 1000.0,
            idle_poll_s: idle_poll_ms as f64 / 1000.0,
        }
    }
}

impl ControllerParams for ControllerLine {
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "{} motor driver {}, numAxes={}, moving poll period={:.6}, idle poll period={:.6}",
            self.driver,
            ctx.port_name(),
            ctx.num_axes(),
            self.moving_poll_s,
            self.idle_poll_s
        );
        ctx.report(out, level);
    }
}

/// An axis whose C `axisRes_` a `*ChangeResolution` command replaces.
trait ChangeResolution: Send {
    fn change_resolution(&mut self, new_resolution: f64);
}

impl ChangeResolution for CorvusAxis {
    fn change_resolution(&mut self, new_resolution: f64) {
        CorvusAxis::change_resolution(self, new_resolution);
    }
}

impl ChangeResolution for HydraAxis {
    fn change_resolution(&mut self, new_resolution: f64) {
        HydraAxis::change_resolution(self, new_resolution);
    }
}

impl ChangeResolution for TaurusAxis {
    fn change_resolution(&mut self, new_resolution: f64) {
        TaurusAxis::change_resolution(self, new_resolution);
    }
}

type ResolutionAxes = HashMap<String, Vec<Arc<Mutex<dyn ChangeResolution>>>>;

/// The axes of every controller port, as C's `findAsynPortDriver` +
/// `getAxis` reach them.
fn resolution_axes() -> &'static Mutex<ResolutionAxes> {
    static AXES: OnceLock<Mutex<ResolutionAxes>> = OnceLock::new();
    AXES.get_or_init(Default::default)
}

fn register_resolution_axis(port: &str, axis: Arc<Mutex<dyn ChangeResolution>>) {
    resolution_axes()
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .entry(port.to_owned())
        .or_default()
        .push(axis);
}

/// C `<name>(portName, axisNo, newResolution)`; `source` is the C file its
/// not-found line names.
fn change_resolution_command(
    name: &'static str,
    port_arg: &'static str,
    source: &'static str,
) -> CommandDef {
    CommandDef::new(
        name,
        vec![
            arg_str_req(port_arg),
            arg_int_req("Axis number"),
            arg_double_req("Axis resolution"),
        ],
        format!("{name}(portName, axisNo, newResolution)"),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port = req_string(args, 0, port_arg)?;
            let axis_no = req_int(args, 1, "Axis number")?;
            let new_resolution = req_double(args, 2, "Axis resolution")?;
            let axes = resolution_axes().lock().unwrap_or_else(|e| e.into_inner());
            let Some(axes) = axes.get(&port) else {
                println!("{source}:{name}: Error port {port} not found");
                return Ok(CommandOutcome::Continue);
            };
            let axis = usize::try_from(axis_no)
                .ok()
                .and_then(|i| axes.get(i))
                .ok_or_else(|| format!("{name}: no axis {axis_no} on port {port}"))?;
            axis.lock()
                .unwrap_or_else(|e| e.into_inner())
                .change_resolution(new_resolution);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SMCcorvusChangeResolution(portName, axisNo, newResolution)`
/// command.
pub fn corvus_change_resolution_command() -> CommandDef {
    change_resolution_command(
        "SMCcorvusChangeResolution",
        "SMC corvus port name",
        "SMCcorvusDriver.cpp",
    )
}

/// Build the `SMCcorvusCreateController(portName, SMCcorvusPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn corvus_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMCcorvusCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC corvus port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "SMCcorvusCreateController(portName, SMCcorvusPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let corvus_port = req_string(args, 1, "SMC corvus port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if !(1..=3).contains(&num_axes) {
                return Err("SMCcorvusCreateController: numAxes must be 1..=3".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port_with(
                ctx,
                &port_name,
                num_axes as i32,
                ControllerLine::new("SMC corvus", moving_poll_ms, idle_poll_ms),
            )?;

            let (handle, _) = connect_controller(
                &corvus_port,
                CORVUS_TIMEOUT,
                &port_name,
                format_args!(
                    "SMCcorvusController::SMCcorvusController: cannot connect to SMC corvus controller"
                ),
            );
            let controller = Arc::new(Mutex::new(CorvusController::new(handle, num_axes as usize)));

            // C startPoller, SMCcorvusDriver.cpp:58
            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let ax = CorvusAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMCcorvusCreateController: axis {index}: {e}"))?;
                let report = ax.members();
                let ax = Arc::new(Mutex::new(ax));
                register_resolution_axis(&port_name, ax.clone());
                let motor: Arc<Mutex<dyn AsynMotor>> = ax;
                poller.install_with_report(asyn_user(&port_name, index as i32), motor, report)?;
            }
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SMChydraChangeResolution(portName, axisNo, newResolution)`
/// command.
pub fn hydra_change_resolution_command() -> CommandDef {
    change_resolution_command(
        "SMChydraChangeResolution",
        "SMC hydra port name",
        "SMChydraDriver.cpp",
    )
}

/// Build the `SMChydraCreateController(portName, SMChydraPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn hydra_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMChydraCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC hydra port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "SMChydraCreateController(portName, SMChydraPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let hydra_port = req_string(args, 1, "SMC hydra port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if !(1..=2).contains(&num_axes) {
                return Err("SMChydraCreateController: numAxes must be 1..=2".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port_with(
                ctx,
                &port_name,
                num_axes as i32,
                ControllerLine::new("SMC hydra", moving_poll_ms, idle_poll_ms),
            )?;

            let (handle, _) = connect_controller(
                &hydra_port,
                HYDRA_TIMEOUT,
                &port_name,
                format_args!(
                    "SMChydraController::SMChydraController: cannot connect to SMC hydra controller"
                ),
            );
            let controller = Arc::new(Mutex::new(HydraController::new(handle)));

            // C startPoller, SMChydraDriver.cpp:60
            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let axis = HydraAxis::new(controller.clone(), index);
                let report = axis.members();
                let ax = Arc::new(Mutex::new(axis));
                register_resolution_axis(&port_name, ax.clone());
                let motor: Arc<Mutex<dyn AsynMotor>> = ax;
                poller.install_with_report(asyn_user(&port_name, index as i32), motor, report)?;
            }
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SMCTaurusChangeResolution(portName, axisNo, newResolution)`
/// command.
pub fn taurus_change_resolution_command() -> CommandDef {
    change_resolution_command(
        "SMCTaurusChangeResolution",
        "SMC Taurus port name",
        "SMCTaurusDriver.cpp",
    )
}

/// Build the `SMCTaurusCreateController(portName, SMCTaurusPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn taurus_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMCTaurusCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC Taurus port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "SMCTaurusCreateController(portName, SMCTaurusPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let taurus_port = req_string(args, 1, "SMC Taurus port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("SMCTaurusCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, num_axes as i32)?;

            let (handle, _) = connect_controller(
                &taurus_port,
                TAURUS_TIMEOUT,
                &port_name,
                format_args!(
                    "SMCTaurusController::SMCTaurusController: cannot connect to SMC Taurus controller"
                ),
            );
            let controller = Arc::new(Mutex::new(TaurusController::new(handle)));

            // C startPoller, SMCTaurusDriver.cpp:41
            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let ax = Arc::new(Mutex::new(TaurusAxis::new(controller.clone(), index)));
                register_resolution_axis(&port_name, ax.clone());
                let motor: Arc<Mutex<dyn AsynMotor>> = ax;
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// drvMicos.cc's `COMM_TIMEOUT`.
const MICOS_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devMicos,"Micos MoCo")` (devMicos.dbd).
static DEV_MICOS: DevSup = model1_device("devMicos", "Micos MoCo");

static MICOS: Model1 = Model1::new(&DEV_MICOS, MICOS_NUM_CARDS, SCAN_RATE);

/// `MicosSetup`'s drivers per card, `Micos_num_axis`.
static MICOS_NUM_AXES: AtomicI32 = AtomicI32::new(MICOS_NUM_AXIS);

/// `MicosSetup(num_cards, num_channels, scan_rate)`.
pub fn micos_setup_command() -> CommandDef {
    MICOS.register_row();
    CommandDef::new(
        "MicosSetup",
        vec![
            arg_int_req("Max. controller count"),
            arg_int_req("Max. motor count"),
            arg_int_req("Polling rate"),
        ],
        "MicosSetup(num_cards, num_channels, scan_rate)",
        |args: &[ArgValue], _ctx: &CommandContext| {
            let num_axis = c_int(args, 1);
            let num_axis = if (1..=MICOS_NUM_AXIS).contains(&num_axis) {
                num_axis
            } else {
                MICOS_NUM_AXIS
            };
            MICOS_NUM_AXES.store(num_axis, Ordering::Relaxed);
            MICOS.setup(c_int(args, 0), c_int(args, 2));
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `MicosConfig(card, name)`. Connects and probes the card at once, where
/// C's `motor_init` does so at iocInit; a card where any of the
/// `num_channels` drivers is silent is left unconfigured, as there.
pub fn micos_config_command() -> CommandDef {
    CommandDef::new(
        "MicosConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "MicosConfig(card, name)",
        |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !MICOS.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            // The MoCo ends a reply with CR LF ETX. C asks for all three as
            // the input EOS, which asyn refuses (eoslen > 2), so its reads
            // end only on timeout; the ETX alone ends one here.
            let handle = connect_with_eos(
                &asyn_port,
                MICOS_TIMEOUT,
                b"\x03",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let controller = MicosController::new(handle);
            let num_axis = MICOS_NUM_AXES.load(Ordering::Relaxed);
            if !micos_probe(&controller, num_axis) {
                return Ok(CommandOutcome::Continue);
            }
            let controller = Arc::new(Mutex::new(controller));
            for signal in 0..num_axis {
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(DrvMicosAxis::new(controller.clone(), signal)));
                MICOS.install(ctx, card, signal, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(all(test, target_os = "linux"))]
mod one_poller {
    use super::*;
    use motor_common::testing::{LineRig, pollers_after};

    fn args(port: &str, line: &str, num_axes: i64) -> Vec<ArgValue> {
        vec![
            ArgValue::String(port.into()),
            ArgValue::String(line.into()),
            ArgValue::Int(num_axes),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ]
    }

    /// C asynMotorController's one poller for every axis of the controller.
    #[epics_rs::base::epics_test]
    async fn smc_corvus_create_controller_polls_its_axes_on_one_poller() {
        let (line, _io) = LineRig::new("corvusLine", |_| "1.0 1.0 1.0".into());
        let cmd = corvus_create_controller_command();
        assert_eq!(pollers_after(cmd, args("corvus", "corvusLine", 3)).await, 1);
        assert!(!line.sent().is_empty());
    }

    #[epics_rs::base::epics_test]
    async fn smc_hydra_create_controller_polls_its_axes_on_one_poller() {
        let (line, _io) = LineRig::new("hydraLine", |_| "1".into());
        let cmd = hydra_create_controller_command();
        assert_eq!(pollers_after(cmd, args("hydra", "hydraLine", 2)).await, 1);
        assert!(!line.sent().is_empty());
    }

    #[epics_rs::base::epics_test]
    async fn smc_taurus_create_controller_polls_its_axes_on_one_poller() {
        let (line, _io) = LineRig::new("taurusLine", |_| "1".into());
        let cmd = taurus_create_controller_command();
        assert_eq!(pollers_after(cmd, args("taurus", "taurusLine", 3)).await, 1);
        assert!(!line.sent().is_empty());
    }
}

#[cfg(test)]
mod report {
    use std::time::{Duration, Instant};

    use super::*;
    use motor_common::testing::{LineRig, port_report, run_command};

    fn args(port: &str, line: &str, num_axes: i64) -> Vec<ArgValue> {
        vec![
            ArgValue::String(port.into()),
            ArgValue::String(line.into()),
            ArgValue::Int(num_axes),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ]
    }

    /// Wait for the poller to have sent `cmd`, the poll that reads the
    /// limit configuration C's report prints.
    async fn polled(line: &LineRig, cmd: &str) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !line.sent().iter().any(|s| s == cmd) {
            assert!(Instant::now() < deadline, "no {cmd}");
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
    }

    /// SMCcorvusController::report then SMCcorvusAxis::report
    /// (SMCcorvusDriver.cpp:128-135, :202-217).
    #[epics_rs::base::epics_test]
    async fn smc_corvus_report_prints_the_controller_then_each_axis() {
        let (line, _io) = LineRig::new("corvusRepLine", |cmd| {
            let verb = cmd.rsplit(' ').next().unwrap_or_default();
            match verb {
                "getpitch" => "1.0",
                "getpolepairs" => "50",
                "getclperiod" => "0.0001",
                "getnlimit" => "-100 100",
                "getsw" => "2 0",
                "pos" => "0 0",
                _ => "0",
            }
            .into()
        });
        run_command(
            corvus_create_controller_command(),
            args("corvusRep", "corvusRepLine", 1),
        )
        .await;
        polled(&line, "1 getsw").await;
        let text = port_report("corvusRep", 1);
        let want = "SMC corvus motor driver corvusRep, numAxes=1, moving poll period=0.100000, \
                    idle poll period=1.000000\n  axis 0\n    pitch 1.000000\n    \
                    polePairs 50\n    clPeriod 0.000100\n    axisRes 0.005000\n    \
                    lowLimitConfig 2\n    highLimitConfig 0\n    posTravelLimit 100.000000\n    \
                    negTravelLimit -100.000000\nPort: corvusRep\n";
        assert!(text.contains(want), "{text}");
    }

    /// SMChydraController::report then SMChydraAxis::report
    /// (SMChydraDriver.cpp:86-93, SMChydraAxis.cpp:76-94).
    #[epics_rs::base::epics_test]
    async fn smc_hydra_report_prints_the_controller_then_each_axis() {
        let (line, _io) = LineRig::new("hydraRepLine", |cmd| {
            let verb = cmd.rsplit(' ').next().unwrap_or_default();
            match verb {
                "getmotor" => "0",
                "getpitch" => "1.0",
                "getpolepairs" => "50",
                "getclperiod" => "0.0001",
                "getnlimit" => "-100 100",
                "getsw" => "2 0",
                _ => "0",
            }
            .into()
        });
        run_command(
            hydra_create_controller_command(),
            args("hydraRep", "hydraRepLine", 1),
        )
        .await;
        polled(&line, "1 getsw").await;
        let text = port_report("hydraRep", 1);
        let want = "SMC hydra motor driver hydraRep, numAxes=1, moving poll period=0.100000, \
                    idle poll period=1.000000\n  axis 0\n    motorForm 0\n    pitch 1.000000\n    \
                    polePairs 50\n    clPeriod 0.000100\n    axisRes 0.005000\n    \
                    lowLimitConfig 2\n    highLimitConfig 0\n    posTravelLimit 100.000000\n    \
                    negTravelLimit -100.000000\nPort: hydraRep\n";
        assert!(text.contains(want), "{text}");
        let text = port_report("hydraRep", 0);
        assert!(
            text.contains(
                "SMC hydra motor driver hydraRep, numAxes=1, moving poll period=0.100000, \
                 idle poll period=1.000000\nPort: hydraRep\n"
            ),
            "{text}"
        );
    }
}
