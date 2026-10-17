//! iocsh commands for the AMCI controllers.
//!
//! - **ANF2** (`ANF2CreateController` + `ANF2CreateAxis` + `ANF2StartPoller`):
//!   three commands, as C. The first connects the In/Out Modbus ports and
//!   creates controller port `portName`; the second looks it up by port name
//!   (C `findAsynPortDriver`) and adds one axis, `@asyn(portName,axis)`.
//! - **ANG1** (`ANG1CreateController`): single command — connects the ports
//!   and installs all `numAxes` axes immediately, matching the C constructor.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::trace::MotorTrace;

use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{Controller, asyn_user, register_port_with};

use crate::anf2::{Anf2Axis, Anf2AxisReport, Anf2Controller, Anf2Params, MAX_AXES};
use crate::ang1::{Ang1Axis, Ang1AxisReport, Ang1Controller, Ang1Params};

/// An `ANF2CreateController` controller and its poller, which
/// `ANF2StartPoller` starts.
#[derive(Clone)]
struct Anf2Port {
    controller: Arc<Mutex<Anf2Controller>>,
    poller: Arc<Controller>,
}

/// ANF2 controllers registered by `ANF2CreateController`, keyed by port name
/// (C `findAsynPortDriver(ANF2Name)` — the C driver looks the controller back
/// up by its own asyn port name, not a numeric card).
fn anf2_registry() -> &'static Mutex<HashMap<String, Anf2Port>> {
    static REG: OnceLock<Mutex<HashMap<String, Anf2Port>>> = OnceLock::new();
    REG.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Build `ANF2CreateController(portName, inPort, outPort, numAxes)`.
pub fn anf2_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ANF2CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("ANF2 In port name"),
            arg_str_req("ANF2 Out port name"),
            arg_int_req("Number of axes"),
        ],
        "ANF2CreateController(portName, ANF2InPortName, ANF2OutPortName, numAxes)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let in_port = req_string(args, 1, "In port name")?;
            let out_port = req_string(args, 2, "Out port name")?;
            // Stricter than C by design: C (ANF2Driver.cpp:88-97) clamps only
            // the upper bound and lets a negative/zero numAxes through. Clamp
            // the lower bound to 0 as well. This is a defensive narrowing, not
            // a behavioral divergence — the only runtime use of num_axes is the
            // `axes_created != num_axes` poll gate, which a zero-clamped value
            // and a negative C value both leave permanently un-satisfied.
            let num_axes = req_int(args, 3, "Number of axes")?.clamp(0, MAX_AXES as i64) as usize;

            let (controller, out_status) = Anf2Controller::new(&in_port, &out_port, num_axes);
            let controller = Arc::new(Mutex::new(controller));
            let params = Anf2Params::new(controller.clone());
            register_port_with(ctx, &port_name, num_axes as i32, params)?;
            if out_status.is_err() {
                MotorTrace::new(port_name.clone(), 0).print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "ANF2Controller::ANF2Controller: cannot connect to ANF2 controller\n"
                    ),
                );
            }
            anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(
                    port_name.clone(),
                    Anf2Port {
                        controller,
                        poller: Arc::new(Controller::new()),
                    },
                );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `strtoul(hexConfig, NULL, 16)`: leading blanks and an optional `0x`,
/// then the longest run of hex digits, 0 when there is none; `None` where C's
/// `errno` is set (overflow).
fn strtoul_hex(text: &str) -> Option<u64> {
    let text = text.trim_start();
    let text = text
        .strip_prefix("0x")
        .or_else(|| text.strip_prefix("0X"))
        .unwrap_or(text);
    let end = text
        .find(|c: char| !c.is_ascii_hexdigit())
        .unwrap_or(text.len());
    match &text[..end] {
        "" => Some(0),
        digits => u64::from_str_radix(digits, 16).ok(),
    }
}

/// Build `ANF2CreateAxis(ANF2Name, axis, hexConfig, baseSpeed, homingTimeout)`,
/// printing what C's does. The axis is polled once `ANF2StartPoller` runs.
pub fn anf2_create_axis_command() -> CommandDef {
    CommandDef::new(
        "ANF2CreateAxis",
        vec![
            arg_str_req("Port name"),
            arg_int_req("Axis number"),
            arg_str_req("Hex config"),
            arg_int_req("Base speed"),
            arg_int_req("Homing timeout"),
        ],
        "ANF2CreateAxis(ANF2Name, axis, hexConfig, baseSpeed, homingTimeout)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let axis = req_int(args, 1, "Axis number")?;
            let hex_config = req_string(args, 2, "Hex config")?;
            let Some(Anf2Port { controller, poller }) = anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&port_name)
                .cloned()
            else {
                println!("ANF2MotorDriver:ANF2CreateAxis: Error port {port_name} not found");
                return Ok(CommandOutcome::Continue);
            };
            let Some(config) = strtoul_hex(&hex_config) else {
                println!("ANF2MotorDriver:ANF2CreateAxis: Error invalid config={hex_config}");
                return Ok(CommandOutcome::Continue);
            };
            println!("ANF2MotorDriver:ANF2CreateAxis: Config=0x{config:x}");
            let config = config as i32;
            let base_speed = req_int(args, 3, "Base speed")?.clamp(1, 1_000_000) as i32;
            let homing_timeout = req_int(args, 4, "Homing timeout")?.clamp(0, 300) as i32;

            let report = Arc::new(Anf2AxisReport {
                controller: controller.clone(),
                axis: axis as i32,
            });
            let ax = Anf2Axis::new(controller, axis as i32, config, base_speed, homing_timeout);
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            poller.install_with_report(asyn_user(&port_name, axis as i32), motor, report)?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANF2StartPoller(ANF2Name, movingPollPeriod, idlePollPeriod)`, which
/// starts the controller's poller at these periods.
pub fn anf2_start_poller_command() -> CommandDef {
    CommandDef::new(
        "ANF2StartPoller",
        vec![
            arg_str_req("Port name"),
            arg_int_req("Moving poll period (ms)"),
            arg_int_req("Idle poll period (ms)"),
        ],
        "ANF2StartPoller(ANF2Name, movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let Some(Anf2Port { controller, poller }) = anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&port_name)
                .cloned()
            else {
                println!("ANF2MotorDriver:ANF2StartPoller: Error port {port_name} not found");
                return Ok(CommandOutcome::Continue);
            };
            let moving_poll_ms = req_int(args, 1, "Moving poll period (ms)")?.max(0) as u64;
            let idle_poll_ms = req_int(args, 2, "Idle poll period (ms)")?.max(0) as u64;
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .set_poll_periods(moving_poll_ms as f64 / 1000.0, idle_poll_ms as f64 / 1000.0);
            // C startPoller, ANF2Driver.cpp:129
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANG1CreateController(portName, ANG1InPortName, ANG1OutPortName,
/// numAxes, movingPollPeriod, idlePollPeriod)`. Matches the C constructor:
/// connects the ports and installs all `numAxes` axes immediately (no
/// separate create-axis command).
pub fn ang1_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ANG1CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("ANG1 In port name"),
            arg_str_req("ANG1 Out port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "ANG1CreateController(portName, ANG1InPortName, ANG1OutPortName, numAxes, \
         movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let in_port = req_string(args, 1, "In port name")?;
            let out_port = req_string(args, 2, "Out port name")?;
            let num_axes = req_int(args, 3, "Number of axes")?;
            // Stricter than C by design: C's axis loop (ANG1Driver.cpp:94)
            // simply doesn't execute for numAxes<=0, silently creating a
            // zero-axis controller with no error. A zero-axis controller is a
            // no-op in practice, so rejecting numAxes < 1 here only surfaces a
            // likely startup-script mistake earlier rather than diverging in
            // any runtime behavior.
            if num_axes < 1 {
                return Err("ANG1CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 4, 5)?;

            // Connected ahead of the port, which writes through it; nothing
            // shows before the port exists.
            let (controller, out_status) = Ang1Controller::new(&port_name, &in_port, &out_port);
            let controller = Arc::new(Mutex::new(controller));
            let params = Ang1Params::new(
                controller.clone(),
                moving_poll_ms as f64 / 1000.0,
                idle_poll_ms as f64 / 1000.0,
            );
            register_port_with(ctx, &port_name, num_axes as i32, params)?;
            if out_status.is_err() {
                MotorTrace::new(port_name.clone(), 0).print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "ANG1Controller::ANG1Controller: cannot connect to ANG1 controller\n"
                    ),
                );
            }

            let poller = Controller::new();
            for axis in 0..num_axes {
                // Infallible, matching C: axis construction ignores the
                // initial setPosition status and never aborts this command.
                let ax = Ang1Axis::new(controller.clone(), axis as i32);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                let report = Arc::new(Ang1AxisReport(axis as i32));
                poller.install_with_report(asyn_user(&port_name, axis as i32), motor, report)?;
            }
            // C startPoller, ANG1Driver.cpp:98
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);

            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `cmds` in turn on a thread of their own, then the asynReport of
    /// `port` at `level`.
    async fn report_after(
        cmds: Vec<(CommandDef, Vec<ArgValue>)>,
        port: &str,
        level: i32,
    ) -> String {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::base::runtime::task::{BlockingBridge, sleep};
        use epics_rs::base::server::database::PvDatabase;

        let bridge = BlockingBridge::capture();
        let port = port.to_string();
        let shell = std::thread::spawn(move || {
            let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
            for (cmd, args) in cmds {
                cmd.handler.call(&args, &ctx).unwrap();
            }
            PortManager::global()
                .find_port_handle(&port)
                .unwrap()
                .report_blocking(level)
                .unwrap()
        });
        // An axis joining the poller waits on a task this runtime runs.
        while !shell.is_finished() {
            sleep(std::time::Duration::from_millis(5)).await;
        }
        shell.join().unwrap()
    }

    /// A two-axis ANF2 on fake Modbus ports whose input register `r` reads
    /// `r`, axis 0 configured `0x86280015`, polled at 100/1000 ms.
    async fn anf2_report(port: &str, level: i32) -> String {
        report_after(anf2_commands(port, true), port, level).await
    }

    /// `anf2_report`'s st.cmd, with or without `ANF2StartPoller`.
    fn anf2_commands(port: &str, start: bool) -> Vec<(CommandDef, Vec<ArgValue>)> {
        let s = |v: &str| ArgValue::String(v.into());
        let (inp, out) = (format!("{port}In"), format!("{port}Out"));
        crate::regs::fake::publish(&inp);
        crate::regs::fake::publish(&out);
        let mut cmds = vec![(
            anf2_create_controller_command(),
            vec![s(port), s(&inp), s(&out), ArgValue::Int(2)],
        )];
        for (axis, config) in [(0, "0x86280015"), (1, "0x0")] {
            let args = vec![
                s(port),
                ArgValue::Int(axis),
                s(config),
                ArgValue::Int(100),
                ArgValue::Int(10),
            ];
            cmds.push((anf2_create_axis_command(), args));
        }
        if start {
            let start = vec![s(port), ArgValue::Int(100), ArgValue::Int(1000)];
            cmds.push((anf2_start_poller_command(), start));
        }
        cmds
    }

    const RULE: &str = "====================================\n";

    /// The controller block and register table C's ANF2Controller::report
    /// prints ahead of the base's (ANF2Driver.cpp:140-183).
    fn anf2_head(port: &str) -> String {
        let mut want = format!(
            "{RULE}ANF2 motor driver:\n    asyn port: {port}\n    num axes: 2\n    \
             axes created: 2\n    moving poll period: 0.100000\n    \
             idle poll period: 1.000000\n\nInput registers:\n\n Reg\tAxis 0\tAxis 1\t\n"
        );
        for i in 0..10 {
            want += &format!("  {i}\t0x{:04x}\t0x{:04x}\t\n", i, 10 + i);
        }
        want + "\n"
    }

    #[epics_rs::base::epics_test]
    async fn the_anf2_report_prints_registers_then_each_axis_s_configuration() {
        let text = anf2_report("anf2Report1", 1).await;
        assert!(text.contains(&anf2_head("anf2Report1")), "{text}");
        let axis0 = "Configuration for axis 0 [0x86280015]:\n  Base Speed: 100\n  \
                     Homing Timeout: 10\n  Capture Input: 0 (Active State: 1)\n  \
                     External Input: 0 (Active State: 0)\n  Home Input: 0 (Active State: 1)\n  \
                     CW Input: 1 (Active State: 0)\n  CCW Input: 1 (Active State: 1)\n  \
                     Backplane Home Proximity Operation: 0\n  Quadrature Encoder: 0\n  \
                     Diagnostic Feedback: 1\n  Output Pulse Type: 1\n  Home Operation: 0\n  \
                     Card Axis: 0\n  Operation Mode for Axis: 1\n\n";
        assert!(text.contains(axis0), "{text}");
        assert!(text.contains("Configuration for axis 1 [0x0]:\n"), "{text}");
        assert!(text.ends_with(RULE), "{text}");
    }

    fn ang1_args(port: &str) -> Vec<ArgValue> {
        vec![
            ArgValue::String(port.into()),
            ArgValue::String(format!("{port}In").into()),
            ArgValue::String(format!("{port}Out").into()),
            ArgValue::Int(2),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ]
    }

    /// C `ANG1Controller::report`, then the base's, which runs each
    /// `ANG1Axis::report` (ANG1Driver.cpp:127-133, 313-321).
    #[epics_rs::base::epics_test]
    async fn the_ang1_report_prints_the_driver_line_then_each_axis_as_c() {
        let cmd = (ang1_create_controller_command(), ang1_args("ang1Report1"));
        let text = report_after(vec![cmd], "ang1Report1", 1).await;
        let want = "ANG1 motor driver ang1Report1, numAxes=2, \
                    moving poll period=0.100000, idle poll period=1.000000\n  axis 0\n  axis 1\n";
        assert!(text.contains(want), "{text}");
    }

    #[epics_rs::base::epics_test]
    async fn a_level_0_ang1_report_prints_no_axis_line() {
        let cmd = (ang1_create_controller_command(), ang1_args("ang1Report0"));
        let text = report_after(vec![cmd], "ang1Report0", 0).await;
        assert!(
            text.contains("ANG1 motor driver ang1Report0, numAxes=2, "),
            "{text}"
        );
        assert!(!text.contains("  axis "), "{text}");
    }

    #[epics_rs::base::epics_test]
    async fn a_level_0_anf2_report_prints_no_axis_configuration() {
        let text = anf2_report("anf2Report0", 0).await;
        assert!(text.contains(&anf2_head("anf2Report0")), "{text}");
        assert!(!text.contains("Configuration for axis"), "{text}");
        assert!(text.ends_with(RULE), "{text}");
    }

    /// `cmd` on a thread of its own, awaited.
    async fn on_shell<T: Send + 'static>(cmd: impl FnOnce() -> T + Send + 'static) -> T {
        use epics_rs::base::runtime::task::sleep;
        let shell = std::thread::spawn(cmd);
        while !shell.is_finished() {
            sleep(std::time::Duration::from_millis(5)).await;
        }
        shell.join().unwrap()
    }

    /// Write `value` to the port parameter `name` at `addr`, then read it
    /// back.
    async fn put_int32(port: &str, name: &'static str, addr: i32, value: i32) -> i32 {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::port::DrvUserRequest;
        let port = port.to_string();
        on_shell(move || {
            let handle = PortManager::global().find_port_handle(&port).unwrap();
            let reason = handle
                .drv_user_create_blocking(&DrvUserRequest::new(name, addr))
                .unwrap()
                .reason;
            handle.write_int32_blocking(reason, addr, value).unwrap();
            handle.read_int32_blocking(reason, addr).unwrap()
        })
        .await
    }

    /// C `ANF2Controller::writeInt32` (ANF2Driver.cpp:211-246): a 1 on
    /// `ANF2_RESET_ERRORS` sends the axis `resetErrors` command block
    /// (:480-496); any value is stored.
    #[epics_rs::base::epics_test]
    async fn anf2_reset_errors_sends_the_reset_command_on_1() {
        anf2_report("anf2Reset", 0).await;
        crate::regs::fake::written("anf2ResetOut");
        assert_eq!(put_int32("anf2Reset", "ANF2_RESET_ERRORS", 1, 0).await, 0);
        assert_eq!(crate::regs::fake::written("anf2ResetOut"), []);
        assert_eq!(put_int32("anf2Reset", "ANF2_RESET_ERRORS", 1, 1).await, 1);
        assert_eq!(
            crate::regs::fake::written("anf2ResetOut"),
            [(10, vec![0x800 << 16, 0, 0, 0, 0])]
        );
    }

    /// A user at address -1 resets axis 0, getAddress's axis for it
    /// (asynPortDriver.cpp:1901-1914).
    #[epics_rs::base::epics_test]
    async fn anf2_reset_errors_at_address_minus_one_resets_axis_0() {
        anf2_report("anf2ResetM1", 0).await;
        crate::regs::fake::written("anf2ResetM1Out");
        assert_eq!(
            put_int32("anf2ResetM1", "ANF2_RESET_ERRORS", -1, 1).await,
            1
        );
        assert_eq!(
            crate::regs::fake::written("anf2ResetM1Out"),
            [(0, vec![0x800 << 16, 0, 0, 0, 0])]
        );
    }

    /// A 1 on `ANF2_GET_INFO` runs `getInfo` (ANF2Driver.cpp:498-513): a
    /// force read at the axis's input block. Unpolled, so the poller's own
    /// force reads cannot land in between.
    #[epics_rs::base::epics_test]
    async fn anf2_get_info_force_reads_the_axis_on_1() {
        let port = "anf2Info";
        report_after(anf2_commands(port, false), port, 0).await;
        crate::regs::fake::forced("anf2InfoIn");
        assert_eq!(put_int32(port, "ANF2_GET_INFO", 1, 0).await, 0);
        assert!(crate::regs::fake::forced("anf2InfoIn").is_empty());
        assert_eq!(put_int32(port, "ANF2_GET_INFO", 1, 1).await, 1);
        assert_eq!(crate::regs::fake::forced("anf2InfoIn"), [10]);
    }

    /// C `ANG1Controller::writeInt32` (ANG1Driver.cpp:161-185): a write to
    /// `ANG1_JERK` is stored and written to the `JERK` register (9).
    #[epics_rs::base::epics_test]
    async fn ang1_jerk_writes_the_jerk_register() {
        let port = "ang1Jerk";
        crate::regs::fake::publish("ang1JerkIn");
        crate::regs::fake::publish("ang1JerkOut");
        let cmd = (ang1_create_controller_command(), ang1_args(port));
        report_after(vec![cmd], port, 0).await;
        crate::regs::fake::written("ang1JerkOut");
        assert_eq!(put_int32(port, "ANG1_JERK", 0, 1234).await, 1234);
        let jerk: Vec<_> = crate::regs::fake::written("ang1JerkOut")
            .into_iter()
            .filter(|(reg, _)| *reg == 9)
            .collect();
        assert_eq!(jerk, [(9, vec![1234])]);
    }

    /// Write `value` to `name` at `addr`, whose port has no axis there.
    async fn put_int32_status(port: &str, name: &'static str, addr: i32, value: i32) -> bool {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::port::DrvUserRequest;
        let port = port.to_string();
        on_shell(move || {
            let handle = PortManager::global().find_port_handle(&port).unwrap();
            let reason = handle
                .drv_user_create_blocking(&DrvUserRequest::new(name, 0))
                .unwrap()
                .reason;
            handle.write_int32_blocking(reason, addr, value).is_ok()
        })
        .await
    }

    /// C's ANF2 and ANG1 `writeInt32` dereference the null axis of an
    /// address with no axis (ANF2Driver.cpp:215-219, ANG1Driver.cpp:165-169);
    /// rs refuses there as the base does, and writes nothing.
    #[epics_rs::base::epics_test]
    async fn a_write_with_no_axis_is_refused_and_sends_nothing() {
        anf2_report("anf2NoAxis", 0).await;
        crate::regs::fake::written("anf2NoAxisOut");
        assert!(!put_int32_status("anf2NoAxis", "ANF2_RESET_ERRORS", 2, 1).await);
        assert_eq!(crate::regs::fake::written("anf2NoAxisOut"), []);

        let port = "ang1NoAxis";
        crate::regs::fake::publish("ang1NoAxisIn");
        crate::regs::fake::publish("ang1NoAxisOut");
        let cmd = (ang1_create_controller_command(), ang1_args(port));
        report_after(vec![cmd], port, 0).await;
        crate::regs::fake::written("ang1NoAxisOut");
        assert!(!put_int32_status(port, "ANG1_JERK", 5, 1234).await);
        assert_eq!(crate::regs::fake::written("ang1NoAxisOut"), []);
    }

    #[test]
    fn hex_config_reads_as_strtoul() {
        assert_eq!(strtoul_hex("0x86280000"), Some(0x8628_0000));
        assert_eq!(strtoul_hex(" 86zz"), Some(0x86));
        assert_eq!(strtoul_hex("zz"), Some(0));
        assert_eq!(strtoul_hex("0x1ffffffffffffffff"), None);
    }
}
