//! C `C300Controller` starts one `asynMotorPoller` for all its axes
//! (C300MotorDriver.cpp:82).

#![cfg(target_os = "linux")]

use epics_rs::base::server::iocsh::registry::ArgValue;
use motor_common::testing::pollers_after;

#[epics_rs::base::epics_test]
async fn four_c300_axes_poll_on_one_poller() {
    let args = vec![
        ArgValue::String("C300_ONE".into()),
        ArgValue::String("C300_ONE_SERIAL".into()),
        ArgValue::Int(4),
        ArgValue::Int(100),
        ArgValue::Int(1000),
    ];
    let cmd = motor_npoint::ioc::c300_create_controller_command();
    assert_eq!(pollers_after(cmd, args).await, 1);
}
