//! The asynReport of an MCS2 controller port, as C's
//! `MCS2Controller::report` and `MCS2Axis::report` print it
//! (smarActMCS2MotorDriver.cpp:177-184, 287-349).

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::LineRig;

/// A one-channel MCS2 on line `{port}Line`, created and reported at `level`.
async fn mcs2_report(port: &str, level: i32) -> String {
    let line = format!("{port}Line");
    let (_rig, _io) = LineRig::new(&line, |cmd| {
        match cmd.trim_end() {
            ":DEV:SNUM?" => "MCS2-00001",
            ":CHAN0:PTYP?" => "300",
            ":CHAN0:PTYP:NAME?" => "SR2013",
            ":CHAN0:STAT?" => "3",
            ":CHAN0:VEL?" => "1000",
            ":CHAN0:ACC?" => "5000",
            ":CHAN0:MCLF?" => "18500",
            ":CHAN0:FERR?" => "7",
            ":CHAN0:ERR?" => "2",
            ":CHAN0:TEMP?" => "25",
            _ => "0",
        }
        .into()
    });
    let args = vec![
        ArgValue::String(port.into()),
        ArgValue::String(line.into()),
        ArgValue::Int(1),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Int(0),
    ];
    let cmd = motor_smaract::ioc::mcs2_create_controller_command();
    let bridge = BlockingBridge::capture();
    let port = port.to_string();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx).unwrap();
        motor_common::testing::await_first_polls(&port);
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

const DRIVER: &str = "MCS2 motor driver {port}, numAxes=1, moving poll period=0.100000, \
                      idle poll period=1.000000\n";

/// C copies its uninitialised `pname` over the reply instead of the reply
/// into it (`strcpy(pC_->inString_, pname)`); the reply is printed here.
#[epics_rs::base::epics_test]
async fn the_mcs2_report_prints_each_channel_s_state_at_level_1() {
    let text = mcs2_report("mcs2Report1", 1).await;
    let want = DRIVER.replace("{port}", "mcs2Report1")
        + "  axis 0\n    positioner type 300\n    positioner name SR2013\n    state 3\n    \
           velocity 1000\n    acceleration 5000\n    max closed loop frequency 18500\n    \
           following error 7\n    error 2\n    temp 25\n";
    assert!(text.contains(&want), "{text}");
}

#[epics_rs::base::epics_test]
async fn a_level_0_mcs2_report_prints_the_driver_line_alone() {
    let text = mcs2_report("mcs2Report0", 0).await;
    assert!(
        text.contains(&DRIVER.replace("{port}", "mcs2Report0")),
        "{text}"
    );
    assert!(!text.contains("  axis 0"), "{text}");
}
