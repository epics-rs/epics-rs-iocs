//! C `phytronController` starts one `asynMotorPoller` in its constructor
//! (phytronAxisMotor.cpp:240), `phytronCreateAxis` axes join it, and its
//! `poll` reads an MCC's `SUI` once per pass for all of them.

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext, CommandDef};
use motor_common::testing::{LineRig, pollers, pollers_after};

/// The command of an MCC frame `STX <addr> <cmd> ETX`.
fn command(frame: &str) -> &str {
    &frame[2..frame.len() - 1]
}

/// An MCC at address 0 that ACKs every frame, with idle, fault-free axes.
fn mcc_reply(frame: &str) -> String {
    let payload = match command(frame) {
        "SUI" => "I=00000000",
        c if c.ends_with("=H") => "E",
        c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "0",
        _ => "",
    };
    format!("\x02\x06{payload}\x03")
}

/// `phytronCreateMCC` on a fresh line, then four `phytronCreateAxis` axes;
/// the line and the poller count after the controller command.
async fn mcc_with_four_axes(name: &str) -> (LineRig, usize) {
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, mcc_reply);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis) = (
        commands.next().unwrap(),
        commands.next().unwrap(),
        commands.next().unwrap(),
    );
    let args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(0),
        ArgValue::Int(20),
        ArgValue::Int(20),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ];
    let started = pollers_after(create_mcc, args).await;
    run(create_axis, name, 4);
    (line, started)
}

/// `phytronCreateAxis(name, 0, index)` for index 1..=n, on a thread of its
/// own as the shell runs it.
fn run(create_axis: CommandDef, name: &str, n: i64) {
    let bridge = BlockingBridge::capture();
    let name = name.to_owned();
    std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        for index in 1..=n {
            let args = vec![
                ArgValue::String(name.clone().into()),
                ArgValue::Int(0),
                ArgValue::Int(index),
            ];
            create_axis.handler.call(&args, &ctx).unwrap();
        }
        motor_common::testing::await_first_polls(&name);
    })
    .join()
    .unwrap();
}

#[epics_rs::base::epics_test]
async fn four_mcc_axes_poll_on_the_one_poller_the_controller_starts() {
    let (_line, started) = mcc_with_four_axes("PHY_ONE").await;
    assert_eq!(started, 1);
    sleep(Duration::from_millis(200)).await;
    assert_eq!(pollers(), 1);
}

#[epics_rs::base::epics_test]
async fn an_mcc_poll_pass_reads_sui_once_for_all_its_axes() {
    let (line, _) = mcc_with_four_axes("PHY_SUI").await;
    sleep(Duration::from_millis(300)).await;
    line.sent();
    sleep(Duration::from_millis(300)).await;
    let sent: Vec<String> = line.sent().iter().map(|f| command(f).to_owned()).collect();
    let passes: Vec<&[String]> = sent.split(|c| c == "SUI").collect();
    assert!(passes.len() > 2, "{sent:?}");
    // The first and last slices are cut by the window.
    for pass in &passes[1..passes.len() - 1] {
        let axes: Vec<&str> = pass.iter().filter_map(|c| c.strip_suffix("=H")).collect();
        assert_eq!(axes, ["1", "2", "3", "4"], "{sent:?}");
    }
}

/// C finds the controller by its port, so setMovingPollPeriod and
/// setIdlePollPeriod pace the poller phytronCreateMCC started before any
/// phytronCreateAxis.
#[epics_rs::base::epics_test]
async fn the_poll_periods_pace_a_controller_with_no_axes_yet() {
    let name = "PHY_PERIOD";
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, mcc_reply);
    let create_mcc = motor_phytron::ioc::phytron_commands()
        .into_iter()
        .nth(1)
        .unwrap();
    let args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(0),
        ArgValue::Int(3_600_000),
        ArgValue::Int(3_600_000),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ];
    pollers_after(create_mcc, args).await;
    line.sent();
    // The wakeup reloads the 5 forced fast polls, at the moving period.
    for set in [
        epics_rs::motor::ioc::set_moving_poll_period_command(),
        epics_rs::motor::ioc::set_idle_poll_period_command(),
    ] {
        let args = vec![ArgValue::String(name.into()), ArgValue::Double(0.01)];
        motor_common::testing::run_command(set, args).await;
    }
    sleep(Duration::from_millis(300)).await;
    let suis = line.sent().iter().filter(|f| command(f) == "SUI").count();
    assert!(suis >= 10, "{suis} SUI");
}
