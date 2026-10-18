//! C `phytronController::report` and `phytronAxis::report`
//! (phytronAxisMotor.cpp:836-843, 1377-1384), reached through asynReport.

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::LineRig;

/// An MCC at address 0 that ACKs every frame, with idle, fault-free axes.
fn mcc_reply(frame: &str) -> String {
    let payload = match &frame[2..frame.len() - 1] {
        "SUI" => "I=00000000",
        c if c.ends_with("=H") => "E",
        c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "0",
        _ => "",
    };
    format!("\x02\x06{payload}\x03")
}

/// `phytronCreateMCC(name, ..., 100, 1000)` and `phytronCreateAxis(name, 0,
/// 1)`, then the asynReport of `name` at `level`.
async fn mcc_report(name: &str, level: i32) -> String {
    let io_port = format!("{name}_IO");
    let (_line, _io) = LineRig::new(&io_port, mcc_reply);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis) = (
        commands.next().unwrap(),
        commands.next().unwrap(),
        commands.next().unwrap(),
    );
    let mcc_args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(0),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ];
    let axis_args = vec![
        ArgValue::String(name.into()),
        ArgValue::Int(0),
        ArgValue::Int(1),
    ];
    let bridge = BlockingBridge::capture();
    let port = name.to_owned();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        create_mcc.handler.call(&mcc_args, &ctx).unwrap();
        create_axis.handler.call(&axis_args, &ctx).unwrap();
        motor_common::testing::await_first_polls();
        PortManager::global()
            .find_port_handle(&port)
            .unwrap()
            .report_blocking(level)
            .unwrap()
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap()
}

/// C prints "phyMOTION" for an MCC controller too, and numAxes is the
/// port's 0xFF.
fn driver_line(port: &str) -> String {
    format!(
        "Phytron phyMOTION motor driver {port}, numAxes=255, moving poll period=0.100000, \
         idle poll period=1.000000\n"
    )
}

#[epics_rs::base::epics_test]
async fn the_phytron_report_prints_the_driver_then_each_axis_as_c() {
    let text = mcc_report("phyReport1", 1).await;
    assert!(text.contains(&driver_line("phyReport1")), "{text}");
    assert!(text.contains("  axis 1\n"), "{text}");
}

#[epics_rs::base::epics_test]
async fn a_level_0_phytron_report_prints_no_axis() {
    let text = mcc_report("phyReport0", 0).await;
    assert!(text.contains(&driver_line("phyReport0")), "{text}");
    assert!(!text.contains("  axis 1\n"), "{text}");
}
