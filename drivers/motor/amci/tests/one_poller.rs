//! C `ANG1Controller` starts one `asynMotorPoller` for all its axes
//! (ANG1Driver.cpp:98), and `ANF2StartPoller` one for the axes
//! `ANF2CreateAxis` made (ANF2Driver.cpp:129).

#![cfg(target_os = "linux")]

use std::sync::Arc;

use epics_rs::base::runtime::task::BlockingBridge;
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::{pollers, pollers_after};

#[epics_rs::base::epics_test]
async fn four_ang1_axes_poll_on_one_poller() {
    let args = vec![
        ArgValue::String("ANG1_ONE".into()),
        ArgValue::String("ANG1_ONE_IN".into()),
        ArgValue::String("ANG1_ONE_OUT".into()),
        ArgValue::Int(4),
        ArgValue::Int(100),
        ArgValue::Int(1000),
    ];
    let cmd = motor_amci::ioc::ang1_create_controller_command();
    assert_eq!(pollers_after(cmd, args).await, 1);
}

#[epics_rs::base::epics_test]
async fn four_anf2_axes_poll_on_the_one_poller_anf2_start_poller_starts() {
    let bridge = BlockingBridge::capture();
    std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        let s = |v: &str| ArgValue::String(v.into());
        motor_amci::ioc::anf2_create_controller_command()
            .handler
            .call(
                &[s("ANF2_ONE"), s("ANF2_IN"), s("ANF2_OUT"), ArgValue::Int(4)],
                &ctx,
            )
            .unwrap();
        for axis in 0..4 {
            let args = [
                s("ANF2_ONE"),
                ArgValue::Int(axis),
                s("0x86000000"),
                ArgValue::Int(100),
                ArgValue::Int(0),
            ];
            motor_amci::ioc::anf2_create_axis_command()
                .handler
                .call(&args, &ctx)
                .unwrap();
        }
    })
    .join()
    .unwrap();
    assert_eq!(pollers(), 0, "C starts no poller before ANF2StartPoller");
    let args = vec![
        ArgValue::String("ANF2_ONE".into()),
        ArgValue::Int(100),
        ArgValue::Int(1000),
    ];
    let cmd = motor_amci::ioc::anf2_start_poller_command();
    assert_eq!(pollers_after(cmd, args).await, 1);
}
