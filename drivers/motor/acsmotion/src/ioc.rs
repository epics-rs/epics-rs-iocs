//! `AcsMotionConfig` and `SPiiPlusCreateProfile` iocsh commands for the
//! ACS SPiiPlus.
//!
//! C `AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod,
//! idlePollPeriod, virtualAxisList)`: connect the serial/TCP octet port,
//! make its comm, identify the controller ([`SpiiPlusController::new`]), and create controller
//! port `acsPort` whose axes records address as `@asyn(acsPort,axis)`. The poll
//! periods are seconds and `virtualAxisList` is comma-separated 0-based axis
//! indices, as in C.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::manager::PortManager;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::controller_port::ControllerPort;

use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double, opt_string,
    req_int, req_string,
};
use motor_common::model1::c_int;
use motor_common::{Controller, asyn_user, register_controller_port};

use crate::comm::{SpiiPlusComm, register_comm_port};
use crate::spiiplus::{AcsMotionParams, SpiiPlusAxis, SpiiPlusAxisReport, SpiiPlusController};

/// Parse the `virtualAxisList` string (comma-separated 0-based indices)
/// into a set, ignoring blanks and out-of-range/unparseable entries (C warns and
/// continues). Returns the set of valid virtual axis indices.
fn parse_virtual_axes(list: &str, num_axes: usize) -> HashSet<usize> {
    let mut set = HashSet::new();
    for tok in list.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        match tok.parse::<usize>() {
            Ok(idx) if idx < num_axes => {
                set.insert(idx);
            }
            _ => {
                println!("AcsMotionConfig: ignoring invalid virtual axis \"{tok}\"");
            }
        }
    }
    set
}

/// Build the `AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod,
/// idlePollPeriod, virtualAxisList)` command.
pub fn acsmotion_config_command() -> CommandDef {
    CommandDef::new(
        "AcsMotionConfig",
        vec![
            arg_str_req("ACS port name"),
            arg_str_req("asyn port name"),
            arg_int_req("num axes"),
            arg_double_opt("Moving polling rate"),
            arg_double_opt("Idle polling rate"),
            arg_str_opt("Virtual axis list"),
        ],
        "AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod, idlePollPeriod, \
         virtualAxisList)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "ACS port name")?;
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let num_axes = req_int(args, 2, "num axes")?;
            if num_axes < 0 {
                return Err("AcsMotionConfig: numAxes must not be negative".into());
            }
            let num_axes = num_axes as usize;
            let moving_poll = opt_double(args, 3, 0.0, "Moving polling rate")?;
            let idle_poll = opt_double(args, 4, 0.0, "Idle polling rate")?;
            let virtual_list = opt_string(args, 5)?.unwrap_or_default();

            let virtual_axes = parse_virtual_axes(&virtual_list, num_axes);

            // C's constructor makes the comm `<ACSPortName>Comm` first.
            let comm_port = format!("{port_name}Comm");
            let comm = SpiiPlusComm::connect(&comm_port, &asyn_port)?;
            let owner = ctx.db().scan_control().interrupt_gate();
            register_comm_port(owner, &comm_port, num_axes as i32)?;
            let mut controller = SpiiPlusController::new(comm, &port_name, num_axes, &virtual_axes);
            controller.set_poll_periods(moving_poll, idle_poll);
            let controller = Arc::new(Mutex::new(controller));

            let (params, profile_thread) = AcsMotionParams::new(controller.clone());
            let port = register_controller_port(
                ctx.db().scan_control().interrupt_gate(),
                &port_name,
                num_axes as i32,
                params,
            )?;
            // C ignores a failed epicsThreadCreate: the profile never runs.
            if let Err(e) = profile_thread.spawn(port) {
                println!("AcsMotionConfig: SPiiPlusProfile thread not started: {e}");
            }
            let polled = controller.clone();
            // C startPoller, SPiiPlusDriver.cpp:269
            let poller = Controller::new().controller_poll(move || {
                // C's poller ignores poll()'s status.
                let _ = polled.lock().unwrap_or_else(|e| e.into_inner()).poll();
            });
            for index in 0..num_axes {
                let ax = SpiiPlusAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                let report = Arc::new(SpiiPlusAxisReport {
                    controller: controller.clone(),
                    axis: index,
                });
                poller.install_with_report(asyn_user(&port_name, index as i32), motor, report)?;
            }
            poller.start(
                &port_name,
                (moving_poll * 1000.0) as u64,
                (idle_poll * 1000.0) as u64,
                2,
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `SPiiPlusCreateProfile(SPiiPlusName, maxPoints, maxPulses)`: size the
/// profile of controller port `SPiiPlusName` and make its data collection
/// arrays in the controller.
pub fn spiiplus_create_profile_command() -> CommandDef {
    CommandDef::new(
        "SPiiPlusCreateProfile",
        vec![
            arg_str_opt("ACS port name"),
            arg_int_opt("Max points"),
            // C registers "Max points" here as well.
            arg_int_opt("Max pulses"),
        ],
        "SPiiPlusCreateProfile(acsPort, maxPoints, maxPulses)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = opt_string(args, 0)?.unwrap_or_default();
            // C passes a negative count on to its allocations.
            let max_points = c_int(args, 1).max(0) as usize;
            let max_pulses = c_int(args, 2).max(0) as usize;
            if !create_profile(&port_name, max_points, max_pulses) {
                println!(
                    "SPiiPlusController:SPiiPlusCreateProfile: Error port {port_name} not found"
                );
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `SPiiPlusCreateProfile`'s `initializeProfile` under the port's lock;
/// `false` where there is no SPiiPlus controller port `port_name`.
fn create_profile(port_name: &str, max_points: usize, max_pulses: usize) -> bool {
    let Ok(port) = PortManager::global().find_port_handle(port_name) else {
        return false;
    };
    port.with_driver_blocking::<ControllerPort<AcsMotionParams>, _>(move |d| {
        d.with_params(|params, ctx| {
            // C drops the status.
            let _ = params.initialize_profile_pulses(ctx, max_points, max_pulses);
        })
    })
    .is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::tests::{SERVOS, rig};

    #[test]
    fn create_profile_initializes_the_named_controller_port() {
        let rig = rig("spiiCreate", 1, SERVOS);
        assert!(create_profile("spiiCreateCtrl", 4, 2));
        let sent = rig.sent();
        assert!(sent.contains(&"GLOBAL REAL DC_DATA_1 (3)(4)".to_string()));
        let max_points = rig.with(|_, ctx| ctx.profile.max_points);
        assert_eq!(max_points, 4);
    }

    #[test]
    fn create_profile_needs_a_spiiplus_controller_port() {
        let _rig = rig("spiiCreateNot", 1, SERVOS);
        assert!(!create_profile("spiiCreateNone", 4, 2));
        // The octet line is a port, but not a SPiiPlus controller's.
        assert!(!create_profile("spiiCreateNot", 4, 2));
    }

    /// `AcsMotionConfig` on a fake line, its poller's first pass taken, and
    /// the controller port's `asynReport` at `level`.
    async fn configured_report(name: &str, level: i32) -> String {
        use epics_rs::base::runtime::task::{BlockingBridge, sleep};
        use epics_rs::base::server::database::PvDatabase;

        let line = crate::fake::line(&format!("{name}Line"));
        for (var, values) in [
            ("MFLAGS", &[12.0, 0.0][..]),
            ("STEPF", &[0.5, 0.25]),
            ("EFAC", &[2.0, 4.0]),
            ("E2FAC", &[3.0]),
            ("E_TYPE", &[1.0]),
            ("E2_TYPE", &[2.0]),
            ("APOS", &[5.0]),
            ("RPOS", &[6.0]),
            ("EPOS", &[7.0]),
            ("FPOS", &[8.0]),
            ("F2POS", &[9.0]),
            ("ROFFS", &[1.5]),
            ("EOFFS", &[2.5]),
            ("E2OFFS", &[3.5]),
            ("MST", &[1.0]),
            ("XVEL", &[100.0]),
            ("XACC", &[1000.0]),
        ] {
            line.lock().unwrap().set(var, values);
        }
        let bridge = BlockingBridge::capture();
        let (name, line_name) = (name.to_string(), format!("{name}Line"));
        let shell = std::thread::spawn(move || {
            let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
            let args = [
                ArgValue::String(name.as_str().into()),
                ArgValue::String(line_name.into()),
                ArgValue::Int(2),
                ArgValue::Double(0.5),
                ArgValue::Double(1.0),
            ];
            acsmotion_config_command()
                .handler
                .call(&args, &ctx)
                .unwrap();
            motor_common::testing::await_first_polls(&name);
            PortManager::global()
                .find_port_handle(&name)
                .unwrap()
                .report_blocking(level)
                .unwrap()
        });
        while !shell.is_finished() {
            sleep(std::time::Duration::from_millis(5)).await;
        }
        shell.join().unwrap()
    }

    const DRIVER: &str = "====================================
SPiiPlus motor driver:
    asyn port: spiiReport1
    num axes: 2
    moving poll period: 0.500000
    idle poll period: 1.000000
    firmware version: v3.13
    virtual feedback position support: No

";

    /// C `SPiiPlusController::report` then, at level > 0, the base's at
    /// level - 1, which runs each `SPiiPlusAxis::report`
    /// (SPiiPlusDriver.cpp:3029-3104).
    #[epics_rs::base::epics_test]
    async fn the_port_report_prints_the_controller_then_each_axis_as_c() {
        let text = configured_report("spiiReport1", 1).await;
        let axis0 = "Configuration for axis 0:
  mflags: 12
    dummy:  0
    open:  0
    micro:  4
    homed:  8
    stepper:  0
    encloop:  0
    stepenc:  0
    brushl:  0
    brushok  0
    phase2:  0
    defcon:  0
    linear:  0
    abscomm:  0
    hall:  0
  resolution: 5.000000e-01
  reference offset: 1.500000
  homing method: 0
  max velocity: 100.000000
  max acceleration: 1000.000000
  virtual: No
Encoder info for axis 0:
  encoder type: 1
  encoder resolution: 2.000000e+00
  encoder offset: 2.500000
  encoder 2 type: 2
  encoder 2 resolution: 3.000000e+00
  encoder 2 offset: 3.500000
Position info for axis 0:
  axis position: 5.000000
  reference position: 6.000000
  encoder position: 7.000000
  feedback position: 8.000000
  feedback 2 position: 9.000000
  virtual feedback position: 0.000000
Status for axis 0:
  moving: 0
  axis status: 0
  motor status: 1
  fault status: 0

Configuration for axis 1:
  mflags: 0
";
        assert!(text.contains(&format!("{DRIVER}{axis0}")), "{text}");
        assert!(text.contains("  resolution: 4.000000e+00\n"), "{text}");
        assert!(
            text.ends_with("\n====================================\n"),
            "{text}"
        );
    }

    /// At level 0 C prints its own lines alone.
    #[epics_rs::base::epics_test]
    async fn a_level_0_report_prints_the_controller_alone() {
        let text = configured_report("spiiReport0", 0).await;
        let own = DRIVER.replace("spiiReport1", "spiiReport0");
        assert!(
            text.ends_with(&format!("{own}====================================\n")),
            "{text}"
        );
        assert!(!text.contains("Configuration for axis"), "{text}");
    }
}

/// C iocsh passes a missing numAxes as 0 (iocsh.cpp:839-840), and C's
/// controller then has no axes.
#[cfg(test)]
mod missing_num_axes {
    use super::*;
    use motor_common::testing::port_after;

    #[epics_rs::base::epics_test]
    async fn an_spiiplus_with_no_axes() {
        let (_line, _io) = motor_common::testing::LineRig::new("spiiNoAxesLine", |_| "0".into());
        let args = vec![
            ArgValue::String("spiiNoAxes".into()),
            ArgValue::String("spiiNoAxesLine".into()),
            ArgValue::Missing,
        ];
        assert!(port_after(acsmotion_config_command(), args, "spiiNoAxes").await);
    }
}
