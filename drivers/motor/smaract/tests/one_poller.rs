//! C `MCS2Controller` starts one `asynMotorPoller` for all its axes
//! (smarActMCS2MotorDriver.cpp:95); the SCU and MCS controllers start theirs
//! in the constructor and their `CreateAxis` axes join it
//! (smarActSCUMotorDriver.cpp:114, smarActMCSMotorDriver.cpp:215).

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext, CommandDef};
use motor_common::testing::{pollers, pollers_after};

#[epics_rs::base::epics_test]
async fn four_mcs2_axes_poll_on_one_poller() {
    let args = vec![
        ArgValue::String("MCS2_ONE".into()),
        ArgValue::String("MCS2_ONE_SERIAL".into()),
        ArgValue::Int(4),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Int(0),
    ];
    let cmd = motor_smaract::ioc::mcs2_create_controller_command();
    assert_eq!(pollers_after(cmd, args).await, 1);
}

/// Run each `(cmd, args)` as the shell does, on a thread of its own.
fn run(lines: Vec<(CommandDef, Vec<ArgValue>)>) {
    let bridge = BlockingBridge::capture();
    std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        for (cmd, args) in lines {
            cmd.handler.call(&args, &ctx).unwrap();
        }
    })
    .join()
    .unwrap();
}

/// Four `create_axis` axes on the poller `controller` started on `port`.
async fn four_axes_after(
    controller: CommandDef,
    controller_args: Vec<ArgValue>,
    create_axis: CommandDef,
) -> usize {
    let port = controller_args[0].clone();
    assert_eq!(pollers_after(controller, controller_args).await, 1);
    run((0..4)
        .map(|axis| {
            let args = vec![port.clone(), ArgValue::Int(axis), ArgValue::Int(axis)];
            (create_axis.clone(), args)
        })
        .collect());
    sleep(Duration::from_millis(200)).await;
    pollers()
}

fn controller_args(port: &str) -> Vec<ArgValue> {
    vec![
        ArgValue::String(port.into()),
        ArgValue::String(format!("{port}_SERIAL").into()),
        ArgValue::Int(4),
        ArgValue::Double(0.1),
        ArgValue::Double(1.0),
    ]
}

#[epics_rs::base::epics_test]
async fn four_scu_axes_poll_on_one_poller() {
    let count = four_axes_after(
        motor_smaract::ioc::scu_create_controller_command(),
        controller_args("SCU_ONE"),
        motor_smaract::ioc::scu_create_axis_command(),
    )
    .await;
    assert_eq!(count, 1);
}

#[epics_rs::base::epics_test]
async fn four_mcs_axes_poll_on_one_poller() {
    let mut args = controller_args("MCS_ONE");
    args.push(ArgValue::Int(0));
    let count = four_axes_after(
        motor_smaract::ioc::mcs_create_controller_command(),
        args,
        motor_smaract::ioc::mcs_create_axis_command(),
    )
    .await;
    assert_eq!(count, 1);
}
