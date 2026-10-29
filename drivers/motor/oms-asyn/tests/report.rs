//! C `omsBaseController::report` (omsBaseController.cpp:130-161), reached
//! through asynReport.

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::LineRig;

/// A MAXnet with firmware 1.33.4 and one encoder axis at 100, encoder 7.
fn maxnet_reply(cmd: &str) -> String {
    match cmd.trim_end() {
        "WY" => "MAXnet ver:1.33.4, s/n:1",
        "AA RP;" => "0",
        "AX PS?" => "=E",
        "AX LT?" => "=h",
        "AM;RI;" => "PNND",
        "AM PP;" => "100",
        "AM PE;" => "7",
        "AM;CL?;" => "on",
        "AM;RV;" => "0",
        "AM;EA;" => "EN",
        "AM;QL;" => "0",
        "#WS" => "=0",
        _ => "",
    }
    .into()
}

/// The asynReport of `port` at `level`, from a thread of its own.
async fn report(port: &str, level: i32) -> String {
    let port = port.to_owned();
    let shell = std::thread::spawn(move || {
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

/// `omsMAXnetConfig(port, 1, line, 100, 1000, "")` on a fresh line.
async fn maxnet(port: &str) {
    let line = format!("{port}Line");
    let (rig, _io) = LineRig::new(&line, maxnet_reply);
    std::mem::forget(rig);
    let config = motor_oms_asyn::ioc::oms_commands()
        .into_iter()
        .next()
        .unwrap();
    let args = vec![
        ArgValue::String(port.into()),
        ArgValue::Int(1),
        ArgValue::String(line.into()),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::String("".into()),
    ];
    let bridge = BlockingBridge::capture();
    let port = port.to_owned();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        config.handler.call(&args, &ctx).unwrap();
        motor_common::testing::await_first_polls(&port);
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap();
}

#[epics_rs::base::epics_test]
async fn the_oms_report_prints_the_firmware_and_each_axis_as_c() {
    maxnet("omsReport1").await;
    let axis = "  axis 0\n    Current position = 100.000000, velocity = 0.000000\n    \
                Encoder position 7.000000\n";
    let text = report("omsReport1", 1).await;
    assert!(
        text.contains("Oms MAXnet motor driver omsReport1, numAxes=1; Firmware: 1.33.4\n"),
        "{text}"
    );
    assert!(text.contains(axis), "{text}");
    assert!(!text.contains("Currently homing"), "{text}");
}

/// C prints each axis number at every level.
#[epics_rs::base::epics_test]
async fn a_level_0_oms_report_prints_only_the_axis_numbers() {
    maxnet("omsReport0").await;
    let text = report("omsReport0", 0).await;
    assert!(
        text.contains(
            "Oms MAXnet motor driver omsReport0, numAxes=1; Firmware: 1.33.4\n  axis 0\n"
        ),
        "{text}"
    );
    assert!(!text.contains("Current position"), "{text}");
}

/// A home sent through the port leaves C `homing` set until the axis reads
/// done, and the level-1 report says so (omsBaseController.cpp:156).
#[epics_rs::base::epics_test]
async fn a_homing_axis_is_reported_as_c() {
    use epics_rs::motor::controller_port::index;

    maxnet("omsReportHoming").await;
    let shell = std::thread::spawn(|| {
        PortManager::global()
            .find_port_handle("omsReportHoming")
            .unwrap()
            .write_float64_blocking(index::HOME, 0, 1.0)
            .unwrap()
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap();
    let text = report("omsReportHoming", 1).await;
    assert!(
        text.contains("Encoder position 7.000000\n    Currently homing axis\n"),
        "{text}"
    );
}
