//! The asynReport of a C300 / LC400 controller port, as C's
//! `C300Controller::report` and `LC400Controller::report` print it.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext, CommandDef};
use motor_common::testing::LineRig;

/// `cmd` with `args` on a thread of its own, then the asynReport of
/// `port` at `level` once the poller thread's startup poll has run.
async fn report_after(cmd: CommandDef, args: Vec<ArgValue>, port: &str, level: i32) -> String {
    let bridge = BlockingBridge::capture();
    let port = port.to_string();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx).unwrap();
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

fn create_args(port: &str, line: &str) -> Vec<ArgValue> {
    vec![
        ArgValue::String(port.into()),
        ArgValue::String(line.into()),
        ArgValue::Int(1),
        ArgValue::Int(100),
        ArgValue::Int(1000),
    ]
}

/// A one-axis C300 whose stage spans 400 units and whose startup poll reads
/// encoder 20 and set point 100.
async fn c300_report(port: &str, level: i32) -> String {
    let line = format!("{port}Line");
    let (_rig, _io) = LineRig::new(&line, |cmd| {
        match cmd.trim_end() {
            "SYST:CHAN1:STAG:RANG?" => "400".into(),
            "SYST:CHAN1:DI:FACT?" => "2".into(),
            // Analog-in 10 at offset 28, position monitor 50 at 112.
            "CHAN1:DATA?" => format!("{:28}{:<84}50", "", "10"),
            "CHAN1:POS?" => "100".into(),
            _ => String::new(),
        }
    });
    let cmd = motor_npoint::ioc::c300_create_controller_command();
    report_after(cmd, create_args(port, &line), port, level).await
}

#[epics_rs::base::epics_test]
async fn the_c300_report_prints_each_axis_at_level_1() {
    let text = c300_report("c300Report1", 1).await;
    let want = "C300 motor driver c300Report1, numAxes=1, moving poll period=0.100000, \
                idle poll period=1.000000\n  axis 0\n  bitsPerUnit_ = 2621.440000\n    \
                encoder position=20.000000\n    theory position=100.000000\n    limits=0x0\n";
    assert!(text.contains(want), "{text}");
}

#[epics_rs::base::epics_test]
async fn a_level_0_c300_report_prints_the_driver_line_alone() {
    let text = c300_report("c300Report0", 0).await;
    let want = "C300 motor driver c300Report0, numAxes=1, moving poll period=0.100000, \
                idle poll period=1.000000\n";
    assert!(text.contains(want), "{text}");
    assert!(!text.contains("  axis 0"), "{text}");
}

/// C `LC400Controller::report` (LC400MotorDriver.cpp:181-199) on a
/// two-axis controller whose port is missing.
async fn lc400_report(port: &str, level: i32) -> String {
    let mut args = create_args(port, &format!("{port}None"));
    args[2] = ArgValue::Int(2);
    let cmd = motor_npoint::ioc_lc400::lc400_commands().remove(0);
    report_after(cmd, args, port, level).await
}

#[epics_rs::base::epics_test]
async fn the_lc400_report_prints_each_axis_at_level_1() {
    let text = lc400_report("lc400Report1", 1).await;
    let want = "LC400 motor driver lc400Report1, numAxes=2, moving poll period=0.100000, \
                idle poll period=1.000000\n axis 0\n axis 1\n";
    assert!(text.contains(want), "{text}");
}

#[epics_rs::base::epics_test]
async fn a_level_0_lc400_report_prints_the_driver_line_alone() {
    let text = lc400_report("lc400Report0", 0).await;
    let want = "LC400 motor driver lc400Report0, numAxes=2, moving poll period=0.100000, \
                idle poll period=1.000000\n";
    assert!(text.contains(want), "{text}");
    assert!(!text.contains(" axis 0\n"), "{text}");
}
